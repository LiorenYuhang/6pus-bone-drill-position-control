#include <gtest/gtest.h>

#include "h30_imu/source_capture_writer.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace {

h30_imu::SourceCaptureRow row(uint64_t sequence) {
  h30_imu::SourceCaptureRow out;
  out.publisher_sequence = sequence;
  out.tid = static_cast<uint16_t>(sequence + 1u);
  out.has_sample_timestamp = true;
  out.sample_timestamp_us = static_cast<uint32_t>(sequence * 5000u);
  out.has_data_ready_timestamp = true;
  out.data_ready_timestamp_us = out.sample_timestamp_us + 200u;
  out.host_monotonic_arrival_time_ns = 1000000000LL + static_cast<int64_t>(sequence) * 5000000LL;
  out.ros_header_stamp_ns = out.host_monotonic_arrival_time_ns;
  out.angular_velocity_rad_s[0] = 0.1;
  out.linear_acceleration_m_s2[2] = 9.8;
  out.orientation_xyzw[3] = 1.0;
  return out;
}

}  // namespace

TEST(SourceCaptureWriter, WritesRowsAsynchronouslyAndPreservesSequence) {
  const auto path = std::filesystem::temp_directory_path() / "h30_source_capture_writer_test.csv";
  std::filesystem::remove(path);

  h30_imu::SourceCaptureWriter writer;
  ASSERT_TRUE(writer.start(path.string(), 32u));
  for (uint64_t i = 0; i < 4u; ++i) {
    ASSERT_TRUE(writer.enqueue(row(i)));
  }
  writer.stop();

  const auto stats = writer.stats();
  EXPECT_EQ(stats.enqueued_rows, 4u);
  EXPECT_EQ(stats.written_rows, 4u);
  EXPECT_EQ(stats.overflow_drops, 0u);
  EXPECT_EQ(stats.writer_errors, 0u);

  std::ifstream in(path);
  ASSERT_TRUE(in.is_open());
  std::string line;
  ASSERT_TRUE(std::getline(in, line));
  EXPECT_NE(line.find("publisher_sequence"), std::string::npos);
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(std::getline(in, line));
    EXPECT_EQ(line.substr(0, line.find(',')), std::to_string(i));
  }
  EXPECT_FALSE(std::getline(in, line));
  std::filesystem::remove(path);
}

TEST(SourceCaptureWriter, ReportsBoundedQueueOverflowWithoutBlocking) {
  const auto path = std::filesystem::temp_directory_path() / "h30_source_capture_writer_overflow_test.csv";
  std::filesystem::remove(path);

  h30_imu::SourceCaptureWriter writer;
  ASSERT_TRUE(writer.start(path.string(), 1u));
  ASSERT_TRUE(writer.enqueue(row(0)));

  bool saw_overflow = false;
  for (uint64_t i = 1; i < 1000u; ++i) {
    if (!writer.enqueue(row(i))) {
      saw_overflow = true;
      break;
    }
  }
  writer.stop();

  const auto stats = writer.stats();
  EXPECT_TRUE(saw_overflow);
  EXPECT_GT(stats.overflow_drops, 0u);
  EXPECT_LE(stats.written_rows, stats.enqueued_rows);
  EXPECT_EQ(stats.writer_errors, 0u);
  std::filesystem::remove(path);
}
