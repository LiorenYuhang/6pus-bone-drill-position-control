#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
namespace h30_imu {
struct Sample {
  uint16_t tid{};
  std::optional<uint32_t> sample_timestamp_us;
  std::optional<uint32_t> data_ready_timestamp_us;
  std::optional<double> temperature;
  std::optional<double> ax, ay, az;
  std::optional<double> gx, gy, gz;
  std::optional<double> mx, my, mz;
  std::optional<double> roll, pitch, yaw;
  std::optional<double> qw, qx, qy, qz;
};

struct ParserStats {
  uint64_t valid_frames{};
  uint64_t checksum_errors{};
  uint64_t malformed_tlv_errors{};
  uint64_t unknown_data_id_errors{};
  uint64_t tid_dropped_frames{};  // Backward-compatible alias for true_tid_missing_frames.
  uint64_t true_tid_missing_frames{};
  uint64_t tid_duplicates{};
  uint64_t tid_out_of_order{};
  uint64_t tid_wraps{};
  uint64_t tid_resets{};
  uint16_t last_tid{};
  bool have_last_tid{};
  uint64_t sample_timestamp_count{};
  uint64_t sample_timestamp_missing_samples{};
  uint64_t sample_timestamp_duplicates{};
  uint64_t sample_timestamp_out_of_order{};
  uint64_t sample_timestamp_wraps{};
  uint64_t sample_timestamp_resets{};
  uint32_t last_sample_timestamp_us{};
  bool have_last_sample_timestamp{};
  uint64_t data_ready_timestamp_count{};
  uint64_t data_ready_timestamp_duplicates{};
  uint64_t data_ready_timestamp_out_of_order{};
  uint64_t data_ready_timestamp_wraps{};
  uint64_t data_ready_timestamp_resets{};
  uint32_t last_data_ready_timestamp_us{};
  bool have_last_data_ready_timestamp{};
};

class ProtocolParser {
public:
  std::vector<Sample> feed(const uint8_t * data, std::size_t size);
  std::size_t checksum_errors() const { return stats_.checksum_errors; }
  ParserStats stats() const { return stats_; }
private:
  std::vector<uint8_t> buffer_;
  ParserStats stats_;
};
}  // namespace h30_imu
