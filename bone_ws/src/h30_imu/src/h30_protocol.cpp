#include "h30_imu/h30_protocol.hpp"
#include <algorithm>
#include <cstring>
namespace {
int16_t i16(const uint8_t *p) { return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8)); }
int32_t i32(const uint8_t *p) {
  const uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  int32_t out; std::memcpy(&out, &v, sizeof(out)); return out;
}
uint16_t u16(const uint8_t *p) { return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8); }
uint32_t u32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24);
}
uint16_t checksum(const uint8_t *p, std::size_t n) {
  uint8_t a = 0, b = 0; for (std::size_t i = 0; i < n; ++i) { a = static_cast<uint8_t>(a + p[i]); b = static_cast<uint8_t>(b + a); }
  return static_cast<uint16_t>(a) | (static_cast<uint16_t>(b) << 8);
}
}  // namespace
namespace h30_imu {
namespace {
constexpr uint16_t kH30TidMin = 1u;
constexpr uint16_t kH30TidMax = 60000u;
constexpr uint32_t kExpectedSampleTimestampStepUs = 5000u;
constexpr uint32_t kTimestampWrapThreshold = 0x80000000u;

void update_tid_stats(ParserStats &stats, uint16_t tid) {
  if (stats.have_last_tid) {
    if (tid == stats.last_tid) {
      ++stats.tid_duplicates;
    } else if (tid > stats.last_tid) {
      const uint64_t gap = static_cast<uint64_t>(tid - stats.last_tid);
      if (gap > 1u) {
        stats.true_tid_missing_frames += gap - 1u;
      }
    } else {
      const bool plausible_h30_wrap = stats.last_tid >= 59000u && tid <= 1000u;
      if (plausible_h30_wrap) {
        ++stats.tid_wraps;
        const uint64_t gap = static_cast<uint64_t>(kH30TidMax - stats.last_tid) + static_cast<uint64_t>(tid);
        if (gap > 1u) {
          stats.true_tid_missing_frames += gap - 1u;
        }
      } else {
        ++stats.tid_resets;
        ++stats.tid_out_of_order;
      }
    }
  }
  stats.tid_dropped_frames = stats.true_tid_missing_frames;
  stats.last_tid = tid;
  stats.have_last_tid = true;
}

void update_sample_timestamp_stats(ParserStats &stats, uint32_t t_us) {
  ++stats.sample_timestamp_count;
  if (stats.have_last_sample_timestamp) {
    if (t_us == stats.last_sample_timestamp_us) {
      ++stats.sample_timestamp_duplicates;
    } else if (t_us > stats.last_sample_timestamp_us) {
      const uint32_t dt = t_us - stats.last_sample_timestamp_us;
      if (dt > kExpectedSampleTimestampStepUs + 1u) {
        stats.sample_timestamp_missing_samples += (dt + kExpectedSampleTimestampStepUs / 2u) / kExpectedSampleTimestampStepUs - 1u;
      }
    } else {
      const uint32_t backwards = stats.last_sample_timestamp_us - t_us;
      if (backwards > kTimestampWrapThreshold) {
        ++stats.sample_timestamp_wraps;
      } else {
        ++stats.sample_timestamp_resets;
        ++stats.sample_timestamp_out_of_order;
      }
    }
  }
  stats.last_sample_timestamp_us = t_us;
  stats.have_last_sample_timestamp = true;
}

void update_data_ready_timestamp_stats(ParserStats &stats, uint32_t t_us) {
  ++stats.data_ready_timestamp_count;
  if (stats.have_last_data_ready_timestamp) {
    if (t_us == stats.last_data_ready_timestamp_us) {
      ++stats.data_ready_timestamp_duplicates;
    } else if (t_us < stats.last_data_ready_timestamp_us) {
      const uint32_t backwards = stats.last_data_ready_timestamp_us - t_us;
      if (backwards > kTimestampWrapThreshold) {
        ++stats.data_ready_timestamp_wraps;
      } else {
        ++stats.data_ready_timestamp_resets;
        ++stats.data_ready_timestamp_out_of_order;
      }
    }
  }
  stats.last_data_ready_timestamp_us = t_us;
  stats.have_last_data_ready_timestamp = true;
}
}  // namespace

std::vector<Sample> ProtocolParser::feed(const uint8_t *data, std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size); std::vector<Sample> output;
  while (true) {
    auto header = buffer_.begin();
    while (header != buffer_.end() && !(header + 1 != buffer_.end() && *header == 0x59 && *(header + 1) == 0x53)) ++header;
    if (header == buffer_.end()) {
      const bool keep = !buffer_.empty() && buffer_.back() == 0x59; buffer_.clear(); if (keep) buffer_.push_back(0x59); break;
    }
    buffer_.erase(buffer_.begin(), header); if (buffer_.size() < 7) break;
    const std::size_t payload_size = buffer_[4], frame_size = payload_size + 7;
    if (buffer_.size() < frame_size) break;
    if (checksum(buffer_.data() + 2, payload_size + 3) != u16(buffer_.data() + 5 + payload_size)) {
      ++stats_.checksum_errors; buffer_.erase(buffer_.begin()); continue;
    }
    Sample s; s.tid = u16(buffer_.data() + 2);
    ++stats_.valid_frames;

    std::size_t pos = 5, end = pos + payload_size;
    while (pos + 2 <= end) {
      const uint8_t id = buffer_[pos]; const std::size_t len = buffer_[pos + 1]; pos += 2;
      if (pos + len > end) {
        ++stats_.malformed_tlv_errors;
        break;
      }
      const uint8_t *p = buffer_.data() + pos;
      constexpr double k = 1e-6;
      bool recognized = true;
      if (id == 0x01 && len == 2) s.temperature = i16(p) * 0.01;
      else if (id == 0x10 && len == 12) { s.ax=i32(p)*k; s.ay=i32(p+4)*k; s.az=i32(p+8)*k; }
      else if (id == 0x20 && len == 12) { s.gx=i32(p)*k; s.gy=i32(p+4)*k; s.gz=i32(p+8)*k; }
      else if (id == 0x30 && len == 12) { s.mx=i32(p)*k; s.my=i32(p+4)*k; s.mz=i32(p+8)*k; }
      else if (id == 0x40 && len == 12) { s.pitch=i32(p)*k; s.roll=i32(p+4)*k; s.yaw=i32(p+8)*k; }
      else if (id == 0x41 && len == 16) { s.qw=i32(p)*k; s.qx=i32(p+4)*k; s.qy=i32(p+8)*k; s.qz=i32(p+12)*k; }
      else if (id == 0x51 && len == 4) s.sample_timestamp_us = u32(p);
      else if (id == 0x52 && len == 4) s.data_ready_timestamp_us = u32(p);
      else if ((id == 0x50 && len == 11) || (id == 0x68 && len == 20) ||
               (id == 0x70 && len == 12) || (id == 0x80 && len == 1)) {
      } else {
        recognized = false;
      }
      if (!recognized) {
        ++stats_.unknown_data_id_errors;
      }
      pos += len;
    }
    update_tid_stats(stats_, s.tid);
    if (s.sample_timestamp_us.has_value()) {
      update_sample_timestamp_stats(stats_, *s.sample_timestamp_us);
    }
    if (s.data_ready_timestamp_us.has_value()) {
      update_data_ready_timestamp_stats(stats_, *s.data_ready_timestamp_us);
    }
    output.push_back(s); buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
  return output;
}
}  // namespace h30_imu
