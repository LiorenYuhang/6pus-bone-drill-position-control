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
uint32_t u32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24);
}
uint16_t u16(const uint8_t *p) { return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8); }
uint16_t checksum(const uint8_t *p, std::size_t n) {
  uint8_t a = 0, b = 0; for (std::size_t i = 0; i < n; ++i) { a = static_cast<uint8_t>(a + p[i]); b = static_cast<uint8_t>(b + a); }
  return static_cast<uint16_t>(a) | (static_cast<uint16_t>(b) << 8);
}
}  // namespace
namespace h30_imu {
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
    if (stats_.have_last_tid) {
      const uint16_t delta = static_cast<uint16_t>(s.tid - stats_.last_tid);
      // YIS documents TID as 1..60000; the physical H30 wraps 60000 -> 1.
      // Keep the normal uint16_t wrap handling for other YIS-compatible devices.
      const bool h30_wrap = stats_.last_tid == 60000u && s.tid == 1u;
      if (h30_wrap) {
        ++stats_.tid_wraps;
      } else if (delta == 0) {
        ++stats_.tid_duplicates;
      } else if (delta == 1) {
        if (s.tid < stats_.last_tid) {
          ++stats_.tid_wraps;
        }
      } else if (delta < 0x8000u) {
        stats_.tid_dropped_frames += static_cast<uint64_t>(delta - 1u);
      } else {
        ++stats_.tid_out_of_order;
      }
    }
    stats_.last_tid = s.tid;
    stats_.have_last_tid = true;

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
      else if (id == 0x31 && len == 12) { s.raw_mx=i32(p)*0.001; s.raw_my=i32(p+4)*0.001; s.raw_mz=i32(p+8)*0.001; }
      else if (id == 0x40 && len == 12) { s.pitch=i32(p)*k; s.roll=i32(p+4)*k; s.yaw=i32(p+8)*k; }
      else if (id == 0x41 && len == 16) { s.qw=i32(p)*k; s.qx=i32(p+4)*k; s.qy=i32(p+8)*k; s.qz=i32(p+12)*k; }
      else if (id == 0x51 && len == 4) s.sample_timestamp_us = u32(p);
      else if (id == 0x52 && len == 4) s.data_ready_timestamp_us = u32(p);
      else if ((id == 0x50 && len == 11) || (id == 0x68 && len == 20) ||
               (id == 0x70 && len == 12) || (id == 0x80 && len == 1)) {
        // Valid YIS fields not needed by the IMU measurement baseline.
      } else {
        recognized = false;
      }
      if (!recognized) {
        ++stats_.unknown_data_id_errors;
      }
      pos += len;
    }
    output.push_back(s); buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
  return output;
}
}  // namespace h30_imu
