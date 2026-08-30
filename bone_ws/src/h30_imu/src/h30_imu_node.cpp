#include "h30_imu/h30_protocol.hpp"
#include "h30_imu/source_capture_writer.hpp"
#include "h30_imu/msg/timed_imu.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;
namespace {
speed_t baud_flag(int baud) {
  switch (baud) {
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw std::invalid_argument("unsupported baud_rate (use 115200/230400/460800/921600)");
  }
}
double deg_to_rad(double x) { return x * 3.14159265358979323846 / 180.0; }
rclcpp::QoS make_timed_qos(const std::string &profile, int depth, const std::string &reliability) {
  if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS();
  }
  const std::size_t d = static_cast<std::size_t>(std::max(1, depth));
  rclcpp::QoS qos{rclcpp::KeepLast(d)};
  qos.durability_volatile();
  if (profile == "calibration" || reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  return qos;
}
}  // namespace
class H30Node final : public rclcpp::Node {
public:
  H30Node() : Node("h30_imu") {
    port_ = declare_parameter<std::string>("port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00");
    baud_ = declare_parameter<int>("baud_rate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    temperature_topic_ = declare_parameter<std::string>("temperature_topic", "imu/temperature");
    euler_topic_ = declare_parameter<std::string>("euler_topic", "imu/euler");
    timed_topic_ = declare_parameter<std::string>("timed_topic", "imu/timed");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "imu/diagnostics");
    timed_qos_profile_ = declare_parameter<std::string>("timed_qos_profile", "sensor_data");
    timed_qos_depth_ = declare_parameter<int>("timed_qos_depth", 10);
    timed_qos_reliability_ = declare_parameter<std::string>("timed_qos_reliability", "best_effort");
    source_capture_enabled_ = declare_parameter<bool>("source_capture_enabled", false);
    source_capture_path_ = declare_parameter<std::string>("source_capture_path", "");
    source_capture_queue_capacity_ = declare_parameter<int>("source_capture_queue_capacity", 65536);
    publish_euler_ = declare_parameter<bool>("publish_euler", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_, rclcpp::SensorDataQoS());
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic_, rclcpp::SensorDataQoS());
    timed_pub_ = create_publisher<h30_imu::msg::TimedImu>(
      timed_topic_, make_timed_qos(timed_qos_profile_, timed_qos_depth_, timed_qos_reliability_));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    if (source_capture_enabled_) {
      if (!source_capture_.start(source_capture_path_, static_cast<std::size_t>(std::max(1, source_capture_queue_capacity_)))) {
        RCLCPP_ERROR(get_logger(), "Failed to start source capture writer at '%s'", source_capture_path_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Source capture enabled at %s capacity=%d", source_capture_path_.c_str(), source_capture_queue_capacity_);
      }
    }
    timer_ = create_wall_timer(2ms, std::bind(&H30Node::poll, this));
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&H30Node::publish_diagnostics, this));
    RCLCPP_INFO(get_logger(), "H30 driver configured for %s at %d baud", port_.c_str(), baud_);
  }
  ~H30Node() override {
    source_capture_.stop();
    if (fd_ >= 0) ::close(fd_);
  }
private:
  void open_serial() {
    const auto t = std::chrono::steady_clock::now();
    if (t - last_open_attempt_ < 1s) return;
    last_open_attempt_ = t;
    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open %s: %s", port_.c_str(), std::strerror(errno)); return;
    }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { disconnect("tcgetattr failed"); return; }
    cfmakeraw(&tty); const speed_t speed = baud_flag(baud_);
    cfsetispeed(&tty, speed); cfsetospeed(&tty, speed); tty.c_cflag |= CLOCAL | CREAD; tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { disconnect("tcsetattr failed"); return; }
    tcflush(fd_, TCIFLUSH); RCLCPP_INFO(get_logger(), "Opened H30 serial port %s", port_.c_str());
  }
  void disconnect(const char *reason) {
    RCLCPP_ERROR(get_logger(), "%s on %s: %s", reason, port_.c_str(), std::strerror(errno));
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = -1;
  }
  void poll() {
    if (fd_ < 0) { open_serial(); return; }
    std::array<uint8_t, 4096> bytes{}; const ssize_t n = ::read(fd_, bytes.data(), bytes.size());
    if (n > 0) {
      const auto arrival = std::chrono::steady_clock::now();
      for (const auto &s : parser_.feed(bytes.data(), static_cast<std::size_t>(n))) publish(s, arrival);
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) disconnect("serial read failed");
  }
  void publish(const h30_imu::Sample &s, std::chrono::steady_clock::time_point arrival) {
    const auto stamp = now(); sensor_msgs::msg::Imu msg; msg.header.stamp = stamp; msg.header.frame_id = frame_id_;
    bool has_imu = false;
    if (s.ax && s.ay && s.az) { msg.linear_acceleration.x=*s.ax; msg.linear_acceleration.y=*s.ay; msg.linear_acceleration.z=*s.az; has_imu=true; }
    else msg.linear_acceleration_covariance[0] = -1.0;
    if (s.gx && s.gy && s.gz) { msg.angular_velocity.x=deg_to_rad(*s.gx); msg.angular_velocity.y=deg_to_rad(*s.gy); msg.angular_velocity.z=deg_to_rad(*s.gz); has_imu=true; }
    else msg.angular_velocity_covariance[0] = -1.0;
    if (s.qw && s.qx && s.qy && s.qz) {
      const double norm=std::sqrt(*s.qw**s.qw + *s.qx**s.qx + *s.qy**s.qy + *s.qz**s.qz);
      if (norm > 1e-9) { msg.orientation.w=*s.qw/norm; msg.orientation.x=*s.qx/norm; msg.orientation.y=*s.qy/norm; msg.orientation.z=*s.qz/norm; has_imu=true; }
      else msg.orientation_covariance[0] = -1.0;
    } else msg.orientation_covariance[0] = -1.0;
    if (has_imu) imu_pub_->publish(msg);
    if (publish_tf_ && tf_broadcaster_ && msg.orientation_covariance[0] != -1.0) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.header.frame_id = world_frame_;
      tf.child_frame_id = frame_id_;
      tf.transform.rotation = msg.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
    if (s.temperature) { sensor_msgs::msg::Temperature t; t.header=msg.header; t.temperature=*s.temperature; t.variance=0.0; temp_pub_->publish(t); }
    if (publish_euler_ && s.roll && s.pitch && s.yaw) {
      geometry_msgs::msg::Vector3Stamped e; e.header=msg.header; e.vector.x=deg_to_rad(*s.roll); e.vector.y=deg_to_rad(*s.pitch); e.vector.z=deg_to_rad(*s.yaw); euler_pub_->publish(e);
    }

    const uint64_t sequence = published_timed_imu_count_;
    const int64_t ros_header_stamp_ns = stamp.nanoseconds();

    h30_imu::msg::TimedImu timed;
    timed.header = msg.header;
    timed.tid = s.tid;
    timed.publisher_sequence = sequence;
    timed.has_sample_timestamp = s.sample_timestamp_us.has_value();
    timed.sample_timestamp_us = s.sample_timestamp_us.value_or(0u);
    timed.has_data_ready_timestamp = s.data_ready_timestamp_us.has_value();
    timed.data_ready_timestamp_us = s.data_ready_timestamp_us.value_or(0u);
    timed.host_monotonic_arrival_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      arrival.time_since_epoch()).count();
    timed.imu = msg;
    if (source_capture_enabled_) {
      h30_imu::SourceCaptureRow row;
      row.publisher_sequence = sequence;
      row.tid = s.tid;
      row.has_sample_timestamp = timed.has_sample_timestamp;
      row.sample_timestamp_us = timed.sample_timestamp_us;
      row.has_data_ready_timestamp = timed.has_data_ready_timestamp;
      row.data_ready_timestamp_us = timed.data_ready_timestamp_us;
      row.host_monotonic_arrival_time_ns = timed.host_monotonic_arrival_time_ns;
      row.ros_header_stamp_ns = ros_header_stamp_ns;
      row.angular_velocity_rad_s[0] = timed.imu.angular_velocity.x;
      row.angular_velocity_rad_s[1] = timed.imu.angular_velocity.y;
      row.angular_velocity_rad_s[2] = timed.imu.angular_velocity.z;
      row.linear_acceleration_m_s2[0] = timed.imu.linear_acceleration.x;
      row.linear_acceleration_m_s2[1] = timed.imu.linear_acceleration.y;
      row.linear_acceleration_m_s2[2] = timed.imu.linear_acceleration.z;
      row.orientation_xyzw[0] = timed.imu.orientation.x;
      row.orientation_xyzw[1] = timed.imu.orientation.y;
      row.orientation_xyzw[2] = timed.imu.orientation.z;
      row.orientation_xyzw[3] = timed.imu.orientation.w;
      (void)source_capture_.enqueue(row);
    }
    timed_pub_->publish(timed);
    ++published_timed_imu_count_;
  }

  static void add_key(diagnostic_msgs::msg::DiagnosticStatus &s, const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value; s.values.push_back(kv);
  }

  void publish_diagnostics() {
    const auto stats = parser_.stats();
    if (!diagnostics_baseline_set_) {
      baseline_checksum_errors_ = stats.checksum_errors;
      baseline_true_tid_missing_frames_ = stats.true_tid_missing_frames;
      baseline_tid_duplicates_ = stats.tid_duplicates;
      baseline_tid_out_of_order_ = stats.tid_out_of_order;
      baseline_tid_resets_ = stats.tid_resets;
      baseline_sample_timestamp_missing_samples_ = stats.sample_timestamp_missing_samples;
      baseline_sample_timestamp_duplicates_ = stats.sample_timestamp_duplicates;
      baseline_sample_timestamp_out_of_order_ = stats.sample_timestamp_out_of_order;
      baseline_sample_timestamp_resets_ = stats.sample_timestamp_resets;
      diagnostics_baseline_set_ = true;
    }
    const auto runtime_checksum_errors = stats.checksum_errors - baseline_checksum_errors_;
    const auto runtime_tid_missing = stats.true_tid_missing_frames - baseline_true_tid_missing_frames_;
    const auto runtime_tid_duplicates = stats.tid_duplicates - baseline_tid_duplicates_;
    const auto runtime_tid_out_of_order = stats.tid_out_of_order - baseline_tid_out_of_order_;
    const auto runtime_tid_resets = stats.tid_resets - baseline_tid_resets_;
    const auto runtime_sample_missing = stats.sample_timestamp_missing_samples - baseline_sample_timestamp_missing_samples_;
    const auto runtime_sample_duplicates = stats.sample_timestamp_duplicates - baseline_sample_timestamp_duplicates_;
    const auto runtime_sample_out_of_order = stats.sample_timestamp_out_of_order - baseline_sample_timestamp_out_of_order_;
    const auto runtime_sample_resets = stats.sample_timestamp_resets - baseline_sample_timestamp_resets_;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name();
    status.hardware_id = port_;
    status.level = (runtime_checksum_errors == 0 && stats.malformed_tlv_errors == 0 &&
      runtime_tid_missing == 0 && runtime_tid_out_of_order == 0 && runtime_tid_resets == 0 &&
      runtime_sample_missing == 0 && runtime_sample_out_of_order == 0 &&
      runtime_sample_resets == 0) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = fd_ >= 0 ? "streaming" : "serial_closed";
    add_key(status, "valid_frames", std::to_string(stats.valid_frames));
    add_key(status, "checksum_errors", std::to_string(stats.checksum_errors));
    add_key(status, "malformed_tlv_errors", std::to_string(stats.malformed_tlv_errors));
    add_key(status, "unknown_data_id_errors", std::to_string(stats.unknown_data_id_errors));
    add_key(status, "tid_dropped_frames", std::to_string(stats.tid_dropped_frames));
    add_key(status, "true_tid_missing_frames", std::to_string(stats.true_tid_missing_frames));
    add_key(status, "baseline_true_tid_missing_frames", std::to_string(baseline_true_tid_missing_frames_));
    add_key(status, "runtime_true_tid_missing_frames", std::to_string(runtime_tid_missing));
    add_key(status, "tid_duplicates", std::to_string(stats.tid_duplicates));
    add_key(status, "runtime_tid_duplicates", std::to_string(runtime_tid_duplicates));
    add_key(status, "tid_out_of_order", std::to_string(stats.tid_out_of_order));
    add_key(status, "runtime_tid_out_of_order", std::to_string(runtime_tid_out_of_order));
    add_key(status, "tid_wraps", std::to_string(stats.tid_wraps));
    add_key(status, "tid_resets", std::to_string(stats.tid_resets));
    add_key(status, "runtime_tid_resets", std::to_string(runtime_tid_resets));
    add_key(status, "sample_timestamp_count", std::to_string(stats.sample_timestamp_count));
    add_key(status, "sample_timestamp_missing_samples", std::to_string(stats.sample_timestamp_missing_samples));
    add_key(status, "runtime_sample_timestamp_missing_samples", std::to_string(runtime_sample_missing));
    add_key(status, "sample_timestamp_duplicates", std::to_string(stats.sample_timestamp_duplicates));
    add_key(status, "runtime_sample_timestamp_duplicates", std::to_string(runtime_sample_duplicates));
    add_key(status, "sample_timestamp_out_of_order", std::to_string(stats.sample_timestamp_out_of_order));
    add_key(status, "runtime_sample_timestamp_out_of_order", std::to_string(runtime_sample_out_of_order));
    add_key(status, "sample_timestamp_wraps", std::to_string(stats.sample_timestamp_wraps));
    add_key(status, "sample_timestamp_resets", std::to_string(stats.sample_timestamp_resets));
    add_key(status, "runtime_sample_timestamp_resets", std::to_string(runtime_sample_resets));
    add_key(status, "data_ready_timestamp_count", std::to_string(stats.data_ready_timestamp_count));
    add_key(status, "data_ready_timestamp_duplicates", std::to_string(stats.data_ready_timestamp_duplicates));
    add_key(status, "data_ready_timestamp_out_of_order", std::to_string(stats.data_ready_timestamp_out_of_order));
    add_key(status, "data_ready_timestamp_wraps", std::to_string(stats.data_ready_timestamp_wraps));
    add_key(status, "data_ready_timestamp_resets", std::to_string(stats.data_ready_timestamp_resets));
    add_key(status, "published_timed_imu_count", std::to_string(published_timed_imu_count_));
    add_key(status, "next_publisher_sequence", std::to_string(published_timed_imu_count_));
    add_key(status, "timed_qos_profile", timed_qos_profile_);
    add_key(status, "timed_qos_depth", std::to_string(timed_qos_depth_));
    add_key(status, "timed_qos_reliability", timed_qos_reliability_);
    const auto capture_stats = source_capture_.stats();
    add_key(status, "source_capture_enabled", capture_stats.enabled ? "true" : "false");
    add_key(status, "source_capture_path", capture_stats.path);
    add_key(status, "source_capture_enqueued_rows", std::to_string(capture_stats.enqueued_rows));
    add_key(status, "source_capture_written_rows", std::to_string(capture_stats.written_rows));
    add_key(status, "source_capture_overflow_drops", std::to_string(capture_stats.overflow_drops));
    add_key(status, "source_capture_writer_errors", std::to_string(capture_stats.writer_errors));
    add_key(status, "source_capture_queue_depth", std::to_string(capture_stats.queue_depth));
    add_key(status, "source_capture_queue_capacity", std::to_string(capture_stats.queue_capacity));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string port_, frame_id_, world_frame_, imu_topic_, temperature_topic_, euler_topic_, timed_topic_, diagnostics_topic_;
  std::string timed_qos_profile_, timed_qos_reliability_;
  std::string source_capture_path_;
  int baud_{}; int timed_qos_depth_{}; int source_capture_queue_capacity_{};
  bool source_capture_enabled_{}; bool publish_euler_{}; bool publish_tf_{}; int fd_{-1};
  uint64_t published_timed_imu_count_{};
  bool diagnostics_baseline_set_{};
  uint64_t baseline_checksum_errors_{};
  uint64_t baseline_true_tid_missing_frames_{};
  uint64_t baseline_tid_duplicates_{};
  uint64_t baseline_tid_out_of_order_{};
  uint64_t baseline_tid_resets_{};
  uint64_t baseline_sample_timestamp_missing_samples_{};
  uint64_t baseline_sample_timestamp_duplicates_{};
  uint64_t baseline_sample_timestamp_out_of_order_{};
  uint64_t baseline_sample_timestamp_resets_{};
  h30_imu::ProtocolParser parser_; std::chrono::steady_clock::time_point last_open_attempt_{};
  rclcpp::TimerBase::SharedPtr timer_; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<h30_imu::msg::TimedImu>::SharedPtr timed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  h30_imu::SourceCaptureWriter source_capture_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); try { rclcpp::spin(std::make_shared<H30Node>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("h30_imu"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
