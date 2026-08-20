#include "h30_imu/h30_protocol.hpp"
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
}  // namespace
class H30Node final : public rclcpp::Node {
public:
  H30Node() : Node("h30_imu") {
    port_ = declare_parameter<std::string>("port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00");
    baud_ = declare_parameter<int>("baud_rate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu/data");
    publish_euler_ = declare_parameter<bool>("publish_euler", true);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", rclcpp::SensorDataQoS());
    euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/euler", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(2ms, std::bind(&H30Node::poll, this));
    RCLCPP_INFO(get_logger(), "H30 driver configured for %s at %d baud", port_.c_str(), baud_);
  }
  ~H30Node() override { if (fd_ >= 0) ::close(fd_); }
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
    if (n > 0) { for (const auto &s : parser_.feed(bytes.data(), static_cast<std::size_t>(n))) publish(s); }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) disconnect("serial read failed");
  }
  void publish(const h30_imu::Sample &s) {
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
    if (msg.orientation_covariance[0] != -1.0) {
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
  }
  std::string port_, frame_id_, world_frame_, imu_topic_; int baud_{}; bool publish_euler_{}; int fd_{-1};
  h30_imu::ProtocolParser parser_; std::chrono::steady_clock::time_point last_open_attempt_{};
  rclcpp::TimerBase::SharedPtr timer_; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); try { rclcpp::spin(std::make_shared<H30Node>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("h30_imu"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
