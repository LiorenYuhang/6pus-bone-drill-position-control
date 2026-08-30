#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace h30_imu {

struct SourceCaptureRow {
  uint64_t publisher_sequence{};
  uint16_t tid{};
  bool has_sample_timestamp{};
  uint32_t sample_timestamp_us{};
  bool has_data_ready_timestamp{};
  uint32_t data_ready_timestamp_us{};
  int64_t host_monotonic_arrival_time_ns{};
  int64_t ros_header_stamp_ns{};
  double angular_velocity_rad_s[3]{};
  double linear_acceleration_m_s2[3]{};
  double orientation_xyzw[4]{};
};

struct SourceCaptureStats {
  bool enabled{};
  uint64_t enqueued_rows{};
  uint64_t written_rows{};
  uint64_t overflow_drops{};
  uint64_t writer_errors{};
  std::size_t queue_depth{};
  std::size_t queue_capacity{};
  std::string path;
};

class SourceCaptureWriter {
public:
  SourceCaptureWriter() = default;
  ~SourceCaptureWriter();

  SourceCaptureWriter(const SourceCaptureWriter &) = delete;
  SourceCaptureWriter & operator=(const SourceCaptureWriter &) = delete;

  bool start(const std::string & path, std::size_t queue_capacity);
  bool enqueue(const SourceCaptureRow & row);
  void stop();
  SourceCaptureStats stats() const;

private:
  void run();
  void write_header();
  void write_row(const SourceCaptureRow & row);

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<SourceCaptureRow> queue_;
  std::thread writer_thread_;
  std::ofstream out_;
  bool enabled_{};
  bool stopping_{};
  uint64_t enqueued_rows_{};
  uint64_t written_rows_{};
  uint64_t overflow_drops_{};
  uint64_t writer_errors_{};
  std::size_t queue_capacity_{};
  std::string path_;
};

}  // namespace h30_imu
