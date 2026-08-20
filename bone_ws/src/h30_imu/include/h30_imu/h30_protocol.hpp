#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
namespace h30_imu {
struct Sample {
  uint16_t tid{};
  std::optional<double> temperature;
  std::optional<double> ax, ay, az;
  std::optional<double> gx, gy, gz;
  std::optional<double> mx, my, mz;
  std::optional<double> roll, pitch, yaw;
  std::optional<double> qw, qx, qy, qz;
};
class ProtocolParser {
public:
  std::vector<Sample> feed(const uint8_t * data, std::size_t size);
  std::size_t checksum_errors() const { return checksum_errors_; }
private:
  std::vector<uint8_t> buffer_;
  std::size_t checksum_errors_{};
};
}  // namespace h30_imu
