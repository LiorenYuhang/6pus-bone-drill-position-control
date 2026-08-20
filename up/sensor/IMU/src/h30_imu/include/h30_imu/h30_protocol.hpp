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
  std::optional<double> raw_mx, raw_my, raw_mz;
  std::optional<double> roll, pitch, yaw;
  std::optional<double> qw, qx, qy, qz;
};

struct ParserStats {
  uint64_t valid_frames{};
  uint64_t checksum_errors{};
  uint64_t malformed_tlv_errors{};
  uint64_t unknown_data_id_errors{};
  uint64_t tid_dropped_frames{};
  uint64_t tid_duplicates{};
  uint64_t tid_out_of_order{};
  uint64_t tid_wraps{};
  uint16_t last_tid{};
  bool have_last_tid{};
};

class ProtocolParser {
public:
  std::vector<Sample> feed(const uint8_t * data, std::size_t size);
  ParserStats stats() const { return stats_; }
private:
  std::vector<uint8_t> buffer_;
  ParserStats stats_;
};
}  // namespace h30_imu
