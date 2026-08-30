#include <gtest/gtest.h>

#include "h30_imu/msg/relative_attitude.hpp"
#include "h30_imu/msg/timed_imu.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

TEST(J9BTimedImuInterface, PreservesIndependentTimestampDomains) {
  h30_imu::msg::TimedImu msg;
  msg.header.stamp.sec = 10;
  msg.header.stamp.nanosec = 20;
  msg.header.frame_id = "pus6_base_imu_link";
  msg.tid = 42;
  msg.publisher_sequence = 1234u;
  msg.has_sample_timestamp = true;
  msg.sample_timestamp_us = 123456789u;
  msg.has_data_ready_timestamp = true;
  msg.data_ready_timestamp_us = 123457001u;
  msg.host_monotonic_arrival_time_ns = 987654321000ll;
  msg.imu.header = msg.header;
  msg.imu.angular_velocity.x = 0.1;
  msg.imu.angular_velocity.y = -0.2;
  msg.imu.angular_velocity.z = 0.3;

  rclcpp::Serialization<h30_imu::msg::TimedImu> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);
  h30_imu::msg::TimedImu roundtrip;
  serializer.deserialize_message(&serialized, &roundtrip);

  EXPECT_EQ(roundtrip.header.stamp.sec, 10);
  EXPECT_EQ(roundtrip.publisher_sequence, 1234u);
  EXPECT_EQ(roundtrip.sample_timestamp_us, 123456789u);
  EXPECT_EQ(roundtrip.data_ready_timestamp_us, 123457001u);
  EXPECT_EQ(roundtrip.host_monotonic_arrival_time_ns, 987654321000ll);
  EXPECT_LT(roundtrip.sample_timestamp_us, roundtrip.data_ready_timestamp_us);
  EXPECT_EQ(roundtrip.imu.header.frame_id, "pus6_base_imu_link");
}

TEST(J9BRelativeAttitudeInterface, CarriesRelativeStateAndPairTiming) {
  h30_imu::msg::RelativeAttitude msg;
  msg.header.frame_id = "pus6_base_imu_link";
  msg.relative_quaternion.w = 1.0;
  msg.relative_angular_velocity.z = 0.25;
  msg.base_corrected_time_us = 1000000.0;
  msg.platform_corrected_time_us = 1000000.0;
  msg.dt_us = 5000.0;
  msg.base_tid = 100;
  msg.platform_tid = 101;
  msg.sequence = 7;

  rclcpp::Serialization<h30_imu::msg::RelativeAttitude> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg, &serialized);
  h30_imu::msg::RelativeAttitude roundtrip;
  serializer.deserialize_message(&serialized, &roundtrip);

  EXPECT_EQ(roundtrip.header.frame_id, "pus6_base_imu_link");
  EXPECT_DOUBLE_EQ(roundtrip.relative_quaternion.w, 1.0);
  EXPECT_DOUBLE_EQ(roundtrip.relative_angular_velocity.z, 0.25);
  EXPECT_DOUBLE_EQ(roundtrip.dt_us, 5000.0);
  EXPECT_EQ(roundtrip.sequence, 7u);
}
