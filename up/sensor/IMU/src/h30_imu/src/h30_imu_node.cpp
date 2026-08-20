#include "h30_imu/h30_protocol.hpp"
#include "h30_imu/msg/timed_imu.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
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
double value_or_nan(const std::optional<double> &value) {
  return value ? *value : std::numeric_limits<double>::quiet_NaN();
}
template<typename T>
void csv_optional(std::ostream &out, const std::optional<T> &value) {
  if (value) {
    out << *value;
  }
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
    publish_euler_ = declare_parameter<bool>("publish_euler", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    terminal_rate_hz_ = declare_parameter<double>("terminal_rate_hz", 5.0);
    if (terminal_rate_hz_ <= 0.0) {
      throw std::invalid_argument("terminal_rate_hz must be positive");
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_, rclcpp::SensorDataQoS());
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic_, rclcpp::SensorDataQoS());
    timed_pub_ = create_publisher<h30_imu::msg::TimedImu>(timed_topic_, rclcpp::SensorDataQoS());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    timer_ = create_wall_timer(2ms, std::bind(&H30Node::poll, this));
    const auto report_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / terminal_rate_hz_));
    terminal_timer_ = create_wall_timer(
      std::max(1ms, report_period), std::bind(&H30Node::report, this));
    if (!csv_path_.empty()) {
      csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_file_.is_open()) {
        throw std::runtime_error("cannot open csv_path: " + csv_path_);
      }
      csv_file_ << "ros_time_ns,host_arrival_time_ns,host_monotonic_arrival_time_ns,arrival_interval_ms,tid,"
                   "sample_timestamp_us,data_ready_timestamp_us,"
                   "ax_m_s2,ay_m_s2,az_m_s2,"
                   "wx_rad_s,wy_rad_s,wz_rad_s,"
                   "roll_deg,pitch_deg,yaw_deg,"
                   "qw,qx,qy,qz,quat_norm,temperature_c,"
                   "raw_mx_mgauss,raw_my_mgauss,raw_mz_mgauss\n";
    }
    RCLCPP_INFO(get_logger(), "H30 driver configured for %s at %d baud", port_.c_str(), baud_);
    if (!csv_path_.empty()) {
      RCLCPP_INFO(get_logger(), "High-rate CSV logging enabled: %s", csv_path_.c_str());
    }
  }
  ~H30Node() override {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();
    }
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
      // This is the host receive time for every complete frame decoded from this read.
      const auto arrival = std::chrono::steady_clock::now();
      for (const auto &s : parser_.feed(bytes.data(), static_cast<std::size_t>(n))) {
        publish(s, arrival);
      }
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) disconnect("serial read failed");
  }
  void publish(const h30_imu::Sample &s, std::chrono::steady_clock::time_point arrival) {
    const auto stamp = now();
    const auto wall_arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto monotonic_arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      arrival.time_since_epoch()).count();
    double arrival_interval_ms = 0.0;
    if (have_last_arrival_) {
      arrival_interval_ms = std::chrono::duration<double, std::milli>(arrival - last_arrival_).count();
    }
    last_arrival_ = arrival;
    have_last_arrival_ = true;

    std::array<double, 4> normalized_quat{
      value_or_nan(s.qw), value_or_nan(s.qx), value_or_nan(s.qy), value_or_nan(s.qz)};
    double quat_norm = std::numeric_limits<double>::quiet_NaN();
    if (s.qw && s.qx && s.qy && s.qz) {
      quat_norm = std::sqrt(*s.qw * *s.qw + *s.qx * *s.qx + *s.qy * *s.qy + *s.qz * *s.qz);
      if (quat_norm > 1e-9) {
        for (double &component : normalized_quat) {
          component /= quat_norm;
        }
      }
    }

    sensor_msgs::msg::Imu msg; msg.header.stamp = stamp; msg.header.frame_id = frame_id_;
    bool has_imu = false;
    if (s.ax && s.ay && s.az) { msg.linear_acceleration.x=*s.ax; msg.linear_acceleration.y=*s.ay; msg.linear_acceleration.z=*s.az; has_imu=true; }
    else msg.linear_acceleration_covariance[0] = -1.0;
    if (s.gx && s.gy && s.gz) { msg.angular_velocity.x=deg_to_rad(*s.gx); msg.angular_velocity.y=deg_to_rad(*s.gy); msg.angular_velocity.z=deg_to_rad(*s.gz); has_imu=true; }
    else msg.angular_velocity_covariance[0] = -1.0;
    if (s.qw && s.qx && s.qy && s.qz) {
      if (quat_norm > 1e-9) { msg.orientation.w=normalized_quat[0]; msg.orientation.x=normalized_quat[1]; msg.orientation.y=normalized_quat[2]; msg.orientation.z=normalized_quat[3]; has_imu=true; }
      else msg.orientation_covariance[0] = -1.0;
    } else msg.orientation_covariance[0] = -1.0;
    if (has_imu) imu_pub_->publish(msg);
    if (publish_tf_ && msg.orientation_covariance[0] != -1.0) {
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

    h30_imu::msg::TimedImu timed;
    timed.header = msg.header;
    timed.tid = s.tid;
    timed.has_sample_timestamp = s.sample_timestamp_us.has_value();
    timed.sample_timestamp_us = s.sample_timestamp_us.value_or(0u);
    timed.has_data_ready_timestamp = s.data_ready_timestamp_us.has_value();
    timed.data_ready_timestamp_us = s.data_ready_timestamp_us.value_or(0u);
    timed.host_monotonic_arrival_time_ns = monotonic_arrival_ns;
    timed.imu = msg;
    timed_pub_->publish(timed);

    if (csv_file_.is_open()) {
      csv_file_ << stamp.nanoseconds() << ',' << wall_arrival_ns << ',' << monotonic_arrival_ns << ','
                << std::setprecision(9)
                << arrival_interval_ms << ',' << s.tid << ',';
      csv_optional(csv_file_, s.sample_timestamp_us); csv_file_ << ',';
      csv_optional(csv_file_, s.data_ready_timestamp_us); csv_file_ << ',';
      csv_optional(csv_file_, s.ax); csv_file_ << ',';
      csv_optional(csv_file_, s.ay); csv_file_ << ',';
      csv_optional(csv_file_, s.az); csv_file_ << ',';
      if (s.gx) { csv_file_ << deg_to_rad(*s.gx); } csv_file_ << ',';
      if (s.gy) { csv_file_ << deg_to_rad(*s.gy); } csv_file_ << ',';
      if (s.gz) { csv_file_ << deg_to_rad(*s.gz); } csv_file_ << ',';
      csv_optional(csv_file_, s.roll); csv_file_ << ',';
      csv_optional(csv_file_, s.pitch); csv_file_ << ',';
      csv_optional(csv_file_, s.yaw); csv_file_ << ',';
      if (std::isfinite(normalized_quat[0])) { csv_file_ << normalized_quat[0]; } csv_file_ << ',';
      if (std::isfinite(normalized_quat[1])) { csv_file_ << normalized_quat[1]; } csv_file_ << ',';
      if (std::isfinite(normalized_quat[2])) { csv_file_ << normalized_quat[2]; } csv_file_ << ',';
      if (std::isfinite(normalized_quat[3])) { csv_file_ << normalized_quat[3]; } csv_file_ << ',';
      if (std::isfinite(quat_norm)) { csv_file_ << quat_norm; } csv_file_ << ',';
      csv_optional(csv_file_, s.temperature); csv_file_ << ',';
      csv_optional(csv_file_, s.raw_mx); csv_file_ << ',';
      csv_optional(csv_file_, s.raw_my); csv_file_ << ',';
      csv_optional(csv_file_, s.raw_mz); csv_file_ << '\n';
    }

    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_ = LatestFrame{s, stamp.nanoseconds(), wall_arrival_ns, arrival_interval_ms,
                          normalized_quat, quat_norm};
  }
  void report() {
    const auto now_steady = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now_steady - last_report_).count();
    const auto stats = parser_.stats();
    const uint64_t frame_delta = stats.valid_frames - last_report_frames_;
    const double rate_hz = elapsed_s > 0.0 ? static_cast<double>(frame_delta) / elapsed_s : 0.0;
    last_report_ = now_steady;
    last_report_frames_ = stats.valid_frames;
    if (csv_file_.is_open()) {
      csv_file_.flush();
    }

    std::optional<LatestFrame> latest;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest = latest_;
    }
    if (!latest) {
      RCLCPP_INFO(get_logger(), "H30 waiting for frames: rate=%.2f Hz checksum=%llu", rate_hz,
                  static_cast<unsigned long long>(stats.checksum_errors));
      return;
    }
    const auto &s = latest->sample;
    RCLCPP_INFO(get_logger(),
      "H30 rate=%.2f Hz dt=%.3f ms tid=%u sample_us=%s ready_us=%s "
      "a=[%.6f %.6f %.6f] m/s^2 w=[%.6f %.6f %.6f] rad/s "
      "rpy=[%.4f %.4f %.4f] deg q=[%.7f %.7f %.7f %.7f] norm=%.8f temp=%.2f C "
      "drops=%llu dup=%llu ooo=%llu wraps=%llu checksum=%llu malformed=%llu unknown=%llu",
      rate_hz, latest->arrival_interval_ms, static_cast<unsigned>(s.tid),
      s.sample_timestamp_us ? std::to_string(*s.sample_timestamp_us).c_str() : "-",
      s.data_ready_timestamp_us ? std::to_string(*s.data_ready_timestamp_us).c_str() : "-",
      value_or_nan(s.ax), value_or_nan(s.ay), value_or_nan(s.az),
      deg_to_rad(value_or_nan(s.gx)), deg_to_rad(value_or_nan(s.gy)), deg_to_rad(value_or_nan(s.gz)),
      value_or_nan(s.roll), value_or_nan(s.pitch), value_or_nan(s.yaw),
      latest->quat[0], latest->quat[1], latest->quat[2], latest->quat[3], latest->quat_norm,
      value_or_nan(s.temperature),
      static_cast<unsigned long long>(stats.tid_dropped_frames),
      static_cast<unsigned long long>(stats.tid_duplicates),
      static_cast<unsigned long long>(stats.tid_out_of_order),
      static_cast<unsigned long long>(stats.tid_wraps),
      static_cast<unsigned long long>(stats.checksum_errors),
      static_cast<unsigned long long>(stats.malformed_tlv_errors),
      static_cast<unsigned long long>(stats.unknown_data_id_errors));
  }
  struct LatestFrame {
    h30_imu::Sample sample;
    int64_t ros_time_ns{};
    int64_t wall_arrival_ns{};
    double arrival_interval_ms{};
    std::array<double, 4> quat{};
    double quat_norm{};
  };

  std::string port_, frame_id_, world_frame_, imu_topic_, temperature_topic_, euler_topic_, timed_topic_, csv_path_;
  int baud_{}; bool publish_euler_{}; bool publish_tf_{}; double terminal_rate_hz_{}; int fd_{-1};
  h30_imu::ProtocolParser parser_; std::chrono::steady_clock::time_point last_open_attempt_{};
  std::chrono::steady_clock::time_point last_arrival_{};
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
  bool have_last_arrival_{};
  uint64_t last_report_frames_{};
  std::mutex latest_mutex_;
  std::optional<LatestFrame> latest_;
  std::ofstream csv_file_;
  rclcpp::TimerBase::SharedPtr timer_; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<h30_imu::msg::TimedImu>::SharedPtr timed_pub_;
  rclcpp::TimerBase::SharedPtr terminal_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); try { rclcpp::spin(std::make_shared<H30Node>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("h30_imu"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
