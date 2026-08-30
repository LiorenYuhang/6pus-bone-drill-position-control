#include "h30_imu/msg/relative_attitude.hpp"
#include "h30_imu/msg/timed_imu.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;

struct Vec3 {
  double x{};
  double y{};
  double z{};
  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

struct Quat {
  double w{1.0};
  double x{};
  double y{};
  double z{};

  Quat operator*(const Quat &o) const {
    return {
      w * o.w - x * o.x - y * o.y - z * o.z,
      w * o.x + x * o.w + y * o.z - z * o.y,
      w * o.y - x * o.z + y * o.w + z * o.x,
      w * o.z + x * o.y - y * o.x + z * o.w};
  }

  void normalize() {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n > 1e-12) {
      w /= n; x /= n; y /= n; z /= n;
    }
  }

  Quat conjugate() const { return {w, -x, -y, -z}; }

  Vec3 rotate(const Vec3 &v) const {
    const Quat p{0.0, v.x, v.y, v.z};
    const Quat r = (*this) * p * conjugate();
    return {r.x, r.y, r.z};
  }
};

Quat exp_rotation(const Vec3 &omega, double dt) {
  const double angle = std::sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z) * dt;
  if (angle < 1e-12) {
    return {1.0, 0.5 * omega.x * dt, 0.5 * omega.y * dt, 0.5 * omega.z * dt};
  }
  const double scale = std::sin(0.5 * angle) / std::sqrt(omega.x * omega.x + omega.y * omega.y + omega.z * omega.z);
  return {std::cos(0.5 * angle), scale * omega.x, scale * omega.y, scale * omega.z};
}

geometry_msgs::msg::Quaternion to_ros(const Quat &q) {
  geometry_msgs::msg::Quaternion out;
  out.w = q.w; out.x = q.x; out.y = q.y; out.z = q.z;
  return out;
}

Vec3 from_ros(const geometry_msgs::msg::Vector3 &v) { return {v.x, v.y, v.z}; }

struct TimedSample {
  double corrected_us{};
  uint16_t tid{};
  Vec3 gyro{};
  rclcpp::Time ros_stamp;
};

class ClockModel {
public:
  explicit ClockModel(double affine_a, double affine_b_us, bool anchor_to_first_host_arrival)
  : affine_a_(affine_a), configured_b_us_(affine_b_us), effective_b_us_(affine_b_us),
    anchor_to_first_host_arrival_(anchor_to_first_host_arrival) {}

  double map(uint32_t raw, int64_t host_ns) {
    if (have_raw_ && raw < last_raw_ && (last_raw_ - raw) > 0x80000000u) {
      ++wraps_;
    } else if (have_raw_ && raw < last_raw_) {
      ++resets_;
      wraps_ = 0;
      anchored_ = false;
      have_mapped_ = false;
    }
    last_raw_ = raw;
    have_raw_ = true;
    const double unwrapped = static_cast<double>(raw) + static_cast<double>(wraps_) * 4294967296.0;
    if (!anchored_) {
      anchor_device_us_ = unwrapped;
      anchor_host_us_ = static_cast<double>(host_ns) * 1e-3;
      if (anchor_to_first_host_arrival_) {
        effective_b_us_ = anchor_host_us_ - affine_a_ * anchor_device_us_ + configured_b_us_;
      } else {
        effective_b_us_ = configured_b_us_;
      }
      anchored_ = true;
    }
    const double mapped = affine_a_ * unwrapped + effective_b_us_;
    if (have_mapped_ && mapped <= last_mapped_us_) {
      ++mapped_nonmonotonic_;
    }
    last_mapped_us_ = mapped;
    have_mapped_ = true;
    return mapped;
  }

  double affine_a() const { return affine_a_; }
  double effective_b_us() const { return effective_b_us_; }
  double ppm_equivalent() const { return (affine_a_ - 1.0) * 1e6; }
  uint64_t wraps() const { return wraps_; }
  uint64_t resets() const { return resets_; }
  uint64_t mapped_nonmonotonic() const { return mapped_nonmonotonic_; }
  bool anchored() const { return anchored_; }
  double anchor_host_us() const { return anchor_host_us_; }
  double anchor_device_us() const { return anchor_device_us_; }

private:
  double affine_a_{1.0};
  double configured_b_us_{};
  double effective_b_us_{};
  bool anchor_to_first_host_arrival_{true};
  uint32_t last_raw_{};
  uint64_t wraps_{};
  uint64_t resets_{};
  uint64_t mapped_nonmonotonic_{};
  double anchor_device_us_{};
  double anchor_host_us_{};
  double last_mapped_us_{};
  bool have_raw_{};
  bool anchored_{};
  bool have_mapped_{};
};
  enum class BiasInitState {
    WAITING_STATIC,
    COLLECTING,
    COMPLETE,
    FALLBACK_TIMEOUT,
    FIXED_FALLBACK
  };

  std::string bias_init_state_to_string(BiasInitState s) {
    switch (s) {
      case BiasInitState::WAITING_STATIC: return "WAITING_STATIC";
      case BiasInitState::COLLECTING: return "COLLECTING";
      case BiasInitState::COMPLETE: return "COMPLETE";
      case BiasInitState::FALLBACK_TIMEOUT: return "FALLBACK_TIMEOUT";
      case BiasInitState::FIXED_FALLBACK: return "FIXED_FALLBACK";
    }
    return "UNKNOWN";
  }

  double calc_deque_std(const std::deque<Vec3> &q) {
    if (q.size() < 10) return 0.0;
    double mx = 0.0, my = 0.0, mz = 0.0;
    for (const auto &v : q) { mx += v.x; my += v.y; mz += v.z; }
    const double inv_n = 1.0 / static_cast<double>(q.size());
    mx *= inv_n; my *= inv_n; mz *= inv_n;
    double var_sum = 0.0;
    for (const auto &v : q) {
      const double dx = v.x - mx, dy = v.y - my, dz = v.z - mz;
      var_sum += dx * dx + dy * dy + dz * dz;
    }
    return std::sqrt(var_sum * inv_n);
  }

  double calc_trimmed_mean(std::vector<double> &vals, double trim_ratio) {
    if (vals.empty()) return 0.0;
    std::sort(vals.begin(), vals.end());
    const std::size_t n = vals.size();
    const std::size_t k = static_cast<std::size_t>(n * trim_ratio);
    if (n <= 2 * k) {
      double sum = 0.0;
      for (double v : vals) sum += v;
      return sum / static_cast<double>(n);
    }
    double sum = 0.0;
    for (std::size_t i = k; i < n - k; ++i) {
      sum += vals[i];
    }
    return sum / static_cast<double>(n - 2 * k);
  }
}  // namespace

class H30RelativeAttitudeNode final : public rclcpp::Node {
public:
  H30RelativeAttitudeNode()
  : Node("h30_relative_attitude"),
    base_clock_(declare_parameter<double>("base_clock_affine_a", 0.999985373783),
                declare_parameter<double>("base_clock_affine_b_us", 0.0),
                declare_parameter<bool>("base_clock_anchor_to_first_host_arrival", true)),
    platform_clock_(declare_parameter<double>("platform_clock_affine_a", 0.999982444509),
                    declare_parameter<double>("platform_clock_affine_b_us", 0.0),
                    declare_parameter<bool>("platform_clock_anchor_to_first_host_arrival", true)) {
    base_topic_ = declare_parameter<std::string>("base_topic", "/pus6/imu_base/timed");
    platform_topic_ = declare_parameter<std::string>("platform_topic", "/pus6/imu_platform/timed");
    relative_topic_ = declare_parameter<std::string>("relative_topic", "/pus6/imu_relative/data");
    quaternion_topic_ = declare_parameter<std::string>("quaternion_topic", "/pus6/imu_relative/quaternion");
    angular_velocity_topic_ = declare_parameter<std::string>("angular_velocity_topic", "/pus6/imu_relative/angular_velocity");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/pus6/imu_relative/diagnostics");
    frame_id_ = declare_parameter<std::string>("frame_id", "pus6_base_imu_link");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");
    platform_link_frame_ = declare_parameter<std::string>("platform_link_frame", "platform_link");
    base_imu_frame_ = declare_parameter<std::string>("base_imu_frame", "pus6_base_imu_link");
    platform_imu_frame_ = declare_parameter<std::string>("platform_imu_frame", "pus6_platform_imu_link");
    robot_extrinsics_status_ = declare_parameter<std::string>("robot_extrinsics_status", "UNCALIBRATED_UNSET");
    max_interpolation_gap_us_ = declare_parameter<double>("max_interpolation_gap_us", 15000.0);
    max_dt_s_ = declare_parameter<double>("max_dt_s", 0.020);

    enable_auto_bias_init_ = declare_parameter<bool>("enable_auto_bias_init", true);
    bias_init_duration_s_ = declare_parameter<double>("bias_init_duration_s", 6.0);
    max_static_gyro_norm_ = declare_parameter<double>("max_static_gyro_norm", 0.012);
    max_static_gyro_std_ = declare_parameter<double>("max_static_gyro_std", 0.004);
    bias_init_timeout_s_ = declare_parameter<double>("bias_init_timeout_s", 30.0);
    trim_ratio_ = declare_parameter<double>("trim_ratio", 0.05);
    min_static_samples_ = static_cast<std::size_t>(std::max(10.0, bias_init_duration_s_ * 200.0));

    const auto bias_b = declare_parameter<std::vector<double>>(
      "base_gyro_bias", {-7.9678928e-05, -7.09914984e-05, 4.02002268e-04});
    const auto bias_p = declare_parameter<std::vector<double>>(
      "platform_gyro_bias", {1.13674e-03, -9.5788e-04, 9.0635e-04});
    const auto q_init = declare_parameter<std::vector<double>>(
      "r_sb_sp_xyzw", {-0.00271995, 0.00834791, 0.01094542, 0.99990155});
    if (bias_b.size() != 3 || bias_p.size() != 3 || q_init.size() != 4) {
      throw std::invalid_argument("bias vectors must have 3 elements and r_sb_sp_xyzw must have 4");
    }
    fallback_base_bias_ = {bias_b[0], bias_b[1], bias_b[2]};
    fallback_platform_bias_ = {bias_p[0], bias_p[1], bias_p[2]};
    base_bias_ = fallback_base_bias_;
    platform_bias_ = fallback_platform_bias_;

    if (enable_auto_bias_init_) {
      bias_init_state_ = BiasInitState::WAITING_STATIC;
      RCLCPP_INFO(get_logger(),
        "Auto gyro bias init enabled: window=%.1f s (%zu samples), max_norm=%.4f rad/s, max_std=%.4f rad/s, trim=%.2f, timeout=%.1f s",
        bias_init_duration_s_, min_static_samples_, max_static_gyro_norm_, max_static_gyro_std_, trim_ratio_, bias_init_timeout_s_);
    } else {
      bias_init_state_ = BiasInitState::FIXED_FALLBACK;
      RCLCPP_INFO(get_logger(), "Auto gyro bias init disabled, using fixed parameter bias");
    }

    state_ = {q_init[3], q_init[0], q_init[1], q_init[2]};
    state_.normalize();
    initial_state_ = state_;

    relative_pub_ = create_publisher<h30_imu::msg::RelativeAttitude>(relative_topic_, rclcpp::SensorDataQoS());
    quaternion_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(quaternion_topic_, rclcpp::SensorDataQoS());
    angular_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(angular_velocity_topic_, rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    base_sub_ = create_subscription<h30_imu::msg::TimedImu>(
      base_topic_, rclcpp::SensorDataQoS(), std::bind(&H30RelativeAttitudeNode::on_base, this, std::placeholders::_1));
    platform_sub_ = create_subscription<h30_imu::msg::TimedImu>(
      platform_topic_, rclcpp::SensorDataQoS(), std::bind(&H30RelativeAttitudeNode::on_platform, this, std::placeholders::_1));
    process_timer_ = create_wall_timer(1ms, std::bind(&H30RelativeAttitudeNode::process, this));
    diagnostics_timer_ = create_wall_timer(1s, std::bind(&H30RelativeAttitudeNode::publish_diagnostics, this));
    RCLCPP_INFO(get_logger(), "Relative attitude propagation ready: base=%s platform=%s", base_topic_.c_str(), platform_topic_.c_str());
    RCLCPP_INFO(get_logger(), "R_next=Exp(-omega_B*dt)*R*Exp(omega_P*dt); gyro frames are not pre-rotated");
  }

private:
  void on_base(const h30_imu::msg::TimedImu::SharedPtr msg) {
    if (!msg->has_sample_timestamp || !valid_gyro(msg->imu)) { ++invalid_base_; return; }
    TimedSample s;
    s.corrected_us = base_clock_.map(msg->sample_timestamp_us, msg->host_monotonic_arrival_time_ns);
    s.tid = msg->tid; s.gyro = from_ros(msg->imu.angular_velocity); s.ros_stamp = msg->header.stamp;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    base_queue_.push_back(s); trim_queue(base_queue_);
    ++base_received_;
  }

  void on_platform(const h30_imu::msg::TimedImu::SharedPtr msg) {
    if (!msg->has_sample_timestamp || !valid_gyro(msg->imu)) { ++invalid_platform_; return; }
    TimedSample s;
    s.corrected_us = platform_clock_.map(msg->sample_timestamp_us, msg->host_monotonic_arrival_time_ns);
    s.tid = msg->tid; s.gyro = from_ros(msg->imu.angular_velocity); s.ros_stamp = msg->header.stamp;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    platform_queue_.push_back(s); trim_queue(platform_queue_);
    ++platform_received_;
  }

  static bool valid_gyro(const sensor_msgs::msg::Imu &msg) {
    return std::isfinite(msg.angular_velocity.x) && std::isfinite(msg.angular_velocity.y) &&
           std::isfinite(msg.angular_velocity.z) && msg.angular_velocity_covariance[0] != -1.0;
  }

  static void trim_queue(std::deque<TimedSample> &q) {
    while (q.size() > 256) q.pop_front();
  }

  bool interpolate_platform(double t, Vec3 &gyro, uint16_t &tid) {
    while (platform_queue_.size() >= 2 && platform_queue_[1].corrected_us <= t) {
      platform_queue_.pop_front();
    }
    if (platform_queue_.empty() || platform_queue_.front().corrected_us > t) return false;
    if (platform_queue_.size() == 1 || platform_queue_.back().corrected_us < t) return false;
    const auto &p0 = platform_queue_[0];
    const auto &p1 = platform_queue_[1];
    const double span = p1.corrected_us - p0.corrected_us;
    if (span <= 0.0 || t - p0.corrected_us > max_interpolation_gap_us_ || p1.corrected_us - t > max_interpolation_gap_us_) return false;
    const double alpha = (t - p0.corrected_us) / span;
    gyro = p0.gyro * (1.0 - alpha) + p1.gyro * alpha;
    tid = alpha < 0.5 ? p0.tid : p1.tid;
    return true;
  }

  void process() {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Check timeout for auto bias init
    if (enable_auto_bias_init_ && bias_init_state_ != BiasInitState::COMPLETE &&
        bias_init_state_ != BiasInitState::FALLBACK_TIMEOUT && bias_init_state_ != BiasInitState::FIXED_FALLBACK) {
      const auto now_steady = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now_steady - init_start_time_).count();
      if (elapsed > bias_init_timeout_s_) {
        base_bias_ = fallback_base_bias_;
        platform_bias_ = fallback_platform_bias_;
        bias_init_state_ = BiasInitState::FALLBACK_TIMEOUT;
        RCLCPP_ERROR(get_logger(),
          "Auto gyro bias init timed out after %.1f s without %.1f s continuous static state. Falling back to fixed parameter bias.",
          elapsed, bias_init_duration_s_);
      }
    }

    while (!base_queue_.empty()) {
      const TimedSample base = base_queue_.front();
      Vec3 platform_gyro; uint16_t platform_tid{};
      if (!interpolate_platform(base.corrected_us, platform_gyro, platform_tid)) {
        if (!platform_queue_.empty() && platform_queue_.back().corrected_us < base.corrected_us - max_interpolation_gap_us_) {
          base_queue_.pop_front(); ++interpolation_misses_;
          continue;
        }
        break;
      }
      base_queue_.pop_front();

      // Startup auto bias initialization phase
      if (enable_auto_bias_init_ && bias_init_state_ != BiasInitState::COMPLETE &&
          bias_init_state_ != BiasInitState::FALLBACK_TIMEOUT && bias_init_state_ != BiasInitState::FIXED_FALLBACK) {
        // Update rolling window (size 50, 0.25s)
        rolling_b_gyro_.push_back(base.gyro);
        if (rolling_b_gyro_.size() > 50) rolling_b_gyro_.pop_front();
        rolling_p_gyro_.push_back(platform_gyro);
        if (rolling_p_gyro_.size() > 50) rolling_p_gyro_.pop_front();

        const double b_norm = std::sqrt(base.gyro.x * base.gyro.x + base.gyro.y * base.gyro.y + base.gyro.z * base.gyro.z);
        const double p_norm = std::sqrt(platform_gyro.x * platform_gyro.x + platform_gyro.y * platform_gyro.y + platform_gyro.z * platform_gyro.z);
        const double b_std = calc_deque_std(rolling_b_gyro_);
        const double p_std = calc_deque_std(rolling_p_gyro_);

        bool is_static = (b_norm <= max_static_gyro_norm_) && (p_norm <= max_static_gyro_norm_);
        if (rolling_b_gyro_.size() >= 20) {
          is_static = is_static && (b_std <= max_static_gyro_std_) && (p_std <= max_static_gyro_std_);
        }

        if (!is_static) {
          if (bias_sample_count_ > 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
              "Motion detected during gyro bias init (norm_b=%.4f norm_p=%.4f std_b=%.4f std_p=%.4f > thresholds [%.4f, %.4f]). Resetting 6.0s collection window.",
              b_norm, p_norm, b_std, p_std, max_static_gyro_norm_, max_static_gyro_std_);
          }
          bias_samples_bx_.clear(); bias_samples_by_.clear(); bias_samples_bz_.clear();
          bias_samples_px_.clear(); bias_samples_py_.clear(); bias_samples_pz_.clear();
          bias_sample_count_ = 0;
          bias_init_state_ = BiasInitState::WAITING_STATIC;
          continue;
        }

        bias_init_state_ = BiasInitState::COLLECTING;
        bias_samples_bx_.push_back(base.gyro.x);
        bias_samples_by_.push_back(base.gyro.y);
        bias_samples_bz_.push_back(base.gyro.z);
        bias_samples_px_.push_back(platform_gyro.x);
        bias_samples_py_.push_back(platform_gyro.y);
        bias_samples_pz_.push_back(platform_gyro.z);
        ++bias_sample_count_;

        if (bias_sample_count_ >= min_static_samples_) {
          base_bias_.x = calc_trimmed_mean(bias_samples_bx_, trim_ratio_);
          base_bias_.y = calc_trimmed_mean(bias_samples_by_, trim_ratio_);
          base_bias_.z = calc_trimmed_mean(bias_samples_bz_, trim_ratio_);
          platform_bias_.x = calc_trimmed_mean(bias_samples_px_, trim_ratio_);
          platform_bias_.y = calc_trimmed_mean(bias_samples_py_, trim_ratio_);
          platform_bias_.z = calc_trimmed_mean(bias_samples_pz_, trim_ratio_);
          
          bias_samples_bx_.clear(); bias_samples_by_.clear(); bias_samples_bz_.clear();
          bias_samples_px_.clear(); bias_samples_py_.clear(); bias_samples_pz_.clear();
          bias_init_state_ = BiasInitState::COMPLETE;
          RCLCPP_INFO(get_logger(),
            "Auto gyro bias init COMPLETE (%zu samples, %.2f s, 5%% trimmed mean): base=[%.8f, %.8f, %.8f] rad/s, platform=[%.8f, %.8f, %.8f] rad/s",
            bias_sample_count_, static_cast<double>(bias_sample_count_) * 0.005,
            base_bias_.x, base_bias_.y, base_bias_.z,
            platform_bias_.x, platform_bias_.y, platform_bias_.z);
        }
        continue;
      }

      const Vec3 omega_b = base.gyro - base_bias_;
      const Vec3 omega_p = platform_gyro - platform_bias_;
      if (!have_state_) {
        have_state_ = true; last_base_time_us_ = base.corrected_us;
        publish(base, platform_gyro, platform_tid, 0.0);
        continue;
      }
      const double dt = (base.corrected_us - last_base_time_us_) * 1e-6;
      last_base_time_us_ = base.corrected_us;
      if (!(dt > 0.0) || dt > max_dt_s_) {
        ++dt_resets_; state_ = initial_state_; state_.normalize();
        continue;
      }
      state_ = exp_rotation(omega_b * -1.0, dt) * state_ * exp_rotation(omega_p, dt);
      state_.normalize();
      publish(base, platform_gyro, platform_tid, dt * 1e6);
      ++processed_;
    }
  }

  void publish(const TimedSample &base, const Vec3 &platform_gyro, uint16_t platform_tid, double dt_us) {
    const Vec3 omega_b = base.gyro - base_bias_;
    const Vec3 omega_p = platform_gyro - platform_bias_;
    const Vec3 relative_omega = state_.rotate(omega_p) - omega_b;
    const auto q = to_ros(state_);
    h30_imu::msg::RelativeAttitude out;
    out.header.stamp = base.ros_stamp; out.header.frame_id = frame_id_;
    out.relative_quaternion = q;
    out.relative_angular_velocity.x = relative_omega.x;
    out.relative_angular_velocity.y = relative_omega.y;
    out.relative_angular_velocity.z = relative_omega.z;
    out.base_corrected_time_us = base.corrected_us;
    out.platform_corrected_time_us = base.corrected_us;
    out.dt_us = dt_us; out.base_tid = base.tid; out.platform_tid = platform_tid; out.sequence = sequence_++;
    relative_pub_->publish(out);
    geometry_msgs::msg::QuaternionStamped qmsg; qmsg.header = out.header; qmsg.quaternion = q; quaternion_pub_->publish(qmsg);
    geometry_msgs::msg::Vector3Stamped wmsg; wmsg.header = out.header;
    wmsg.vector.x = relative_omega.x; wmsg.vector.y = relative_omega.y; wmsg.vector.z = relative_omega.z;
    angular_velocity_pub_->publish(wmsg);
    last_corrected_time_us_ = base.corrected_us; last_dt_us_ = dt_us; last_platform_tid_ = platform_tid;
  }

  void publish_diagnostics() {
    const auto now = this->now();
    const auto steady_now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(steady_now - last_diagnostics_steady_).count();
    const double processed_rate_hz = elapsed_s > 0.0 ?
      static_cast<double>(processed_ - last_diagnostic_processed_) / elapsed_s : 0.0;
    last_diagnostics_steady_ = steady_now;
    last_diagnostic_processed_ = processed_;
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus status; status.name = get_fully_qualified_name();
    status.hardware_id = "H30-dual";
    status.level = (base_received_ > 0 && platform_received_ > 0 && dt_resets_ == 0) ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ? "propagating" : "waiting_or_reset";
    add_key(status, "processed_frames", std::to_string(processed_));
    add_key(status, "processed_rate_hz", std::to_string(processed_rate_hz));
    add_key(status, "base_received", std::to_string(base_received_));
    add_key(status, "platform_received", std::to_string(platform_received_));
    add_key(status, "interpolation_misses", std::to_string(interpolation_misses_));
    add_key(status, "dt_resets", std::to_string(dt_resets_));
    add_key(status, "invalid_base", std::to_string(invalid_base_));
    add_key(status, "invalid_platform", std::to_string(invalid_platform_));
    add_key(status, "last_corrected_time_us", std::to_string(last_corrected_time_us_));
    add_key(status, "last_dt_us", std::to_string(last_dt_us_));
    add_key(status, "clock_model", "host_monotonic_us = affine_a * unwrapped_0x51_sample_timestamp_us + affine_b_us");
    add_key(status, "base_clock_affine_a", std::to_string(base_clock_.affine_a()));
    add_key(status, "base_clock_effective_b_us", std::to_string(base_clock_.effective_b_us()));
    add_key(status, "base_clock_ppm_equivalent", std::to_string(base_clock_.ppm_equivalent()));
    add_key(status, "base_clock_anchor_device_us", std::to_string(base_clock_.anchor_device_us()));
    add_key(status, "base_clock_anchor_host_us", std::to_string(base_clock_.anchor_host_us()));
    add_key(status, "platform_clock_affine_a", std::to_string(platform_clock_.affine_a()));
    add_key(status, "platform_clock_effective_b_us", std::to_string(platform_clock_.effective_b_us()));
    add_key(status, "platform_clock_ppm_equivalent", std::to_string(platform_clock_.ppm_equivalent()));
    add_key(status, "platform_clock_anchor_device_us", std::to_string(platform_clock_.anchor_device_us()));
    add_key(status, "platform_clock_anchor_host_us", std::to_string(platform_clock_.anchor_host_us()));
    add_key(status, "base_clock_wraps", std::to_string(base_clock_.wraps()));
    add_key(status, "platform_clock_wraps", std::to_string(platform_clock_.wraps()));
    add_key(status, "base_clock_resets", std::to_string(base_clock_.resets()));
    add_key(status, "platform_clock_resets", std::to_string(platform_clock_.resets()));
    add_key(status, "base_clock_mapped_nonmonotonic", std::to_string(base_clock_.mapped_nonmonotonic()));
    add_key(status, "platform_clock_mapped_nonmonotonic", std::to_string(platform_clock_.mapped_nonmonotonic()));
    add_key(status, "base_link_frame", base_link_frame_);
    add_key(status, "platform_link_frame", platform_link_frame_);
    add_key(status, "base_imu_frame", base_imu_frame_);
    add_key(status, "platform_imu_frame", platform_imu_frame_);
    add_key(status, "robot_extrinsics_status", robot_extrinsics_status_);

    // J5-B3-5A Bias initialization diagnostics
    add_key(status, "bias_init_state", bias_init_state_to_string(bias_init_state_));
    add_key(status, "bias_samples", std::to_string(bias_sample_count_) + "/" + std::to_string(min_static_samples_));
    char base_buf[128], plat_buf[128];
    std::snprintf(base_buf, sizeof(base_buf), "[%.8f, %.8f, %.8f]", base_bias_.x, base_bias_.y, base_bias_.z);
    std::snprintf(plat_buf, sizeof(plat_buf), "[%.8f, %.8f, %.8f]", platform_bias_.x, platform_bias_.y, platform_bias_.z);
    add_key(status, "estimated_base_bias", base_buf);
    add_key(status, "estimated_platform_bias", plat_buf);
    add_key(status, "max_static_gyro_norm_th", std::to_string(max_static_gyro_norm_));
    add_key(status, "max_static_gyro_std_th", std::to_string(max_static_gyro_std_));

    array.status.push_back(status); diagnostics_pub_->publish(array);
  }

  static void add_key(diagnostic_msgs::msg::DiagnosticStatus &s, const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value; s.values.push_back(kv);
  }

  std::string base_topic_, platform_topic_, relative_topic_, quaternion_topic_, angular_velocity_topic_, diagnostics_topic_, frame_id_;
  std::string base_link_frame_, platform_link_frame_, base_imu_frame_, platform_imu_frame_, robot_extrinsics_status_;
  double max_interpolation_gap_us_{};
  double max_dt_s_{};
  bool enable_auto_bias_init_{true};
  double bias_init_duration_s_{6.0};
  double max_static_gyro_norm_{0.012};
  double max_static_gyro_std_{0.004};
  double bias_init_timeout_s_{30.0};
  double trim_ratio_{0.05};
  std::size_t min_static_samples_{1200};
  BiasInitState bias_init_state_{BiasInitState::WAITING_STATIC};
  std::size_t bias_sample_count_{0};
  std::chrono::steady_clock::time_point init_start_time_{std::chrono::steady_clock::now()};
  std::deque<Vec3> rolling_b_gyro_, rolling_p_gyro_;
  std::vector<double> bias_samples_bx_, bias_samples_by_, bias_samples_bz_;
  std::vector<double> bias_samples_px_, bias_samples_py_, bias_samples_pz_;
  Vec3 fallback_base_bias_{}, fallback_platform_bias_{};
  Vec3 base_bias_{}, platform_bias_{};
  Quat state_{}, initial_state_{};
  ClockModel base_clock_, platform_clock_;
  bool have_state_{};
  double last_base_time_us_{};
  double last_corrected_time_us_{};
  double last_dt_us_{};
  uint16_t last_platform_tid_{};
  uint64_t sequence_{}, base_received_{}, platform_received_{}, processed_{}, interpolation_misses_{}, dt_resets_{}, invalid_base_{}, invalid_platform_{};
  uint64_t last_diagnostic_processed_{};
  std::chrono::steady_clock::time_point last_diagnostics_steady_{std::chrono::steady_clock::now()};
  std::deque<TimedSample> base_queue_, platform_queue_;
  std::mutex queue_mutex_;
  rclcpp::Subscription<h30_imu::msg::TimedImu>::SharedPtr base_sub_, platform_sub_;
  rclcpp::Publisher<h30_imu::msg::RelativeAttitude>::SharedPtr relative_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_velocity_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_, diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<H30RelativeAttitudeNode>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("h30_relative_attitude"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
