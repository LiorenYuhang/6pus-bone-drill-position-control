"""ROS 2 Forward Kinematics (FK) Node for 6PUS Parallel Manipulator.

Subscribes to timed joint feedback telemetry, maintains thread-safe sliding
timeline history, performs strictly-bracketed 0-extrapolation configuration
reconstruction, and solves real-time 6DoF Cartesian platform pose with
branch-continuity gate.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from pus6_msgs.msg import TimedJointFeedback, FkStateStamped
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import ForwardKinematicsResult
from pus6_kinematics.state_synchronizer import (
    FixedLagJointSynchronizer,
    SyncConfigurationResult,
    SyncStatus,
)


def euler_zyx_to_quaternion(alpha: float, beta: float, gamma: float) -> tuple[float, float, float, float]:
    """Convert ZYX Euler angles (roll, pitch, yaw) to unit quaternion (x, y, z, w)."""
    ca, sa = math.cos(alpha * 0.5), math.sin(alpha * 0.5)
    cb, sb = math.cos(beta * 0.5), math.sin(beta * 0.5)
    cg, sg = math.cos(gamma * 0.5), math.sin(gamma * 0.5)

    w = cg * cb * ca + sg * sb * sa
    x = cg * cb * sa - sg * sb * ca
    y = cg * sb * ca + sg * cb * sa
    z = sg * cb * ca - cg * sb * sa
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm > 1e-12:
        return (x/norm, y/norm, z/norm, w/norm)
    return (0.0, 0.0, 0.0, 1.0)


def so3_geodesic_distance_rad(R1: list[list[float]], R2: list[list[float]]) -> float:
    """Compute SO(3) geodesic angular distance between two 3x3 rotation matrices in radians."""
    tr = 0.0
    for r in range(3):
        for c in range(3):
            tr += R1[r][c] * R2[r][c]
    cos_theta = (tr - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.acos(cos_theta)


class Pus6FkNode(Node):
    """Real-time Forward Kinematics and Fixed-Lag Synchronizer Node."""

    def __init__(
        self,
        node_name: str = "pus6_fk_node",
        max_residual_m: float = 1e-4,
        max_pos_jump_m: float = 0.015,     # 15mm max step jump
        max_ang_jump_rad: float = 0.15,    # ~8.59 deg max SO(3) step jump
        query_rate_hz: float = 200.0,
        fixed_lag_delay_ms: float = 50.0,  # Query past timestamp (t_now - fixed_lag_delay)
        perform_fk: bool = True,
        publish_fk: bool = True,
        log_diagnostics: bool = True,
        diagnostic_log_interval_s: float = 1.0,
    ) -> None:
        super().__init__(node_name)

        self.kinematics = Analytical6PusKinematics()
        self.synchronizer = FixedLagJointSynchronizer(max_history_size=500)

        self.max_residual_m = max_residual_m
        self.max_pos_jump_m = max_pos_jump_m
        self.max_ang_jump_rad = max_ang_jump_rad
        self.query_rate_hz = query_rate_hz
        self.fixed_lag_delay_ns = int(fixed_lag_delay_ms * 1e6)
        self.perform_fk = bool(perform_fk)
        self.publish_fk = bool(publish_fk)
        self.log_diagnostics = bool(log_diagnostics)
        self.diagnostic_log_interval_ns = int(max(0.0, diagnostic_log_interval_s) * 1e9)
        self._last_diagnostic_log_ns: dict[str, int] = {}

        self._last_successful_pose: Optional[list[float]] = None
        self._last_successful_R: Optional[list[list[float]]] = None

        # Metrics
        self._total_queries: int = 0
        self._synced_count: int = 0
        self._wait_count: int = 0
        self._fk_success_count: int = 0
        self._fk_fail_count: int = 0
        self._jump_reject_count: int = 0
        self._last_timer_stamp_ns: int = 0
        self._timer_periods_ns: list[int] = []
        self._callback_durations_ns: list[int] = []
        self._feedback_callback_durations_ns: list[int] = []
        self._feedback_callback_count: int = 0
        self._compute_times_us: list[float] = []

        # Subscriptions
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback,
            "/pus6/joint_feedback/timed",
            self._on_timed_feedback,
            qos_profile_sensor_data,
        )

        # Publishers
        self.fk_state_pub = self.create_publisher(
            FkStateStamped,
            "/pus6/fk/state",
            qos_profile_sensor_data,
        )
        self.fk_pose_pub = self.create_publisher(
            PoseStamped,
            "/pus6/fk/pose",
            qos_profile_sensor_data,
        )

        # Periodic query timer if query_rate_hz > 0
        if self.query_rate_hz > 0.0:
            timer_period = 1.0 / self.query_rate_hz
            self.query_timer = self.create_timer(timer_period, self._on_query_timer)
        else:
            self.query_timer = None

        self.get_logger().info(
            f"Pus6FkNode initialized (query_rate={query_rate_hz}Hz, fixed_lag_delay={fixed_lag_delay_ms}ms, max_residual={max_residual_m*1000:.2f}mm)"
        )

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        """Ingest feedback frame into thread-safe synchronizer."""
        callback_start_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self._feedback_callback_count += 1
        try:
            self.synchronizer.push_feedback(msg)
        finally:
            self._feedback_callback_durations_ns.append(
                time.clock_gettime_ns(time.CLOCK_MONOTONIC) - callback_start_ns
            )

    def _log_diagnostic(self, level: str, key: str, message: str) -> None:
        """Rate-limit repeated failure logs without suppressing safety counters."""
        if not self.log_diagnostics:
            return
        now_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        last_ns = self._last_diagnostic_log_ns.get(key, 0)
        if now_ns - last_ns < self.diagnostic_log_interval_ns:
            return
        self._last_diagnostic_log_ns[key] = now_ns
        if level == "warn":
            self.get_logger().warn(message)
        elif level == "error":
            self.get_logger().error(message)
        else:
            raise ValueError(f"Unsupported diagnostic log level: {level}")

    def _on_query_timer(self) -> None:
        """Periodic query triggered at target frequency (e.g. 200Hz)."""
        callback_start_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        t_now_mono_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        try:
            if self._last_timer_stamp_ns > 0:
                self._timer_periods_ns.append(t_now_mono_ns - self._last_timer_stamp_ns)
            self._last_timer_stamp_ns = t_now_mono_ns

            query_timestamp_ns = t_now_mono_ns - self.fixed_lag_delay_ns
            self.process_query(query_timestamp_ns)
        finally:
            self._callback_durations_ns.append(
                time.clock_gettime_ns(time.CLOCK_MONOTONIC) - callback_start_ns
            )

    def process_query(self, query_timestamp_ns: int) -> Optional[FkStateStamped]:
        """Execute fixed-lag reconstruction, solve FK with warm start, check gates, and publish."""
        self._total_queries += 1

        sync_res: SyncConfigurationResult = self.synchronizer.query_sync_configuration(query_timestamp_ns)
        if sync_res.status != SyncStatus.SYNCED:
            self._wait_count += 1
            return None

        self._synced_count += 1
        if not self.perform_fk:
            return None
        t_start_solve = time.perf_counter()

        # Warm start: use last successful pose or default neutral height
        initial_guess = self._last_successful_pose
        try:
            fk_res: ForwardKinematicsResult = self.kinematics.forward_kinematics_with_metrics(
                sync_res.positions_meters,
                initial_guess=initial_guess,
                max_iter=25,
                tol=1e-10,
            )
        except (WorkspaceLimitError, SingularityError, KinematicsError, ValueError, ArithmeticError, Exception) as exc:
            self._fk_fail_count += 1
            self._log_diagnostic(
                "warn", "solver_exception",
                f"FK solver exception at query t={query_timestamp_ns}: {exc}",
            )
            return None

        compute_time_us = (time.perf_counter() - t_start_solve) * 1e6
        self._compute_times_us.append(compute_time_us)

        if not fk_res.converged or fk_res.residual_norm > self.max_residual_m:
            self._fk_fail_count += 1
            self._log_diagnostic(
                "warn", "solver_failure",
                f"FK solver failed/diverged at query t={query_timestamp_ns}: residual={fk_res.residual_norm:.4e}m, converged={fk_res.converged}",
            )
            return None

        # Branch continuity / Pose jump gate (Translation + SO(3) geodesic)
        if self._last_successful_pose is not None and self._last_successful_R is not None:
            dx = fk_res.pose[0] - self._last_successful_pose[0]
            dy = fk_res.pose[1] - self._last_successful_pose[1]
            dz = fk_res.pose[2] - self._last_successful_pose[2]
            pos_jump = math.sqrt(dx*dx + dy*dy + dz*dz)

            so3_jump = so3_geodesic_distance_rad(self._last_successful_R, fk_res.rotation_matrix)

            if pos_jump > self.max_pos_jump_m or so3_jump > self.max_ang_jump_rad:
                self._jump_reject_count += 1
                self._log_diagnostic(
                    "error", "branch_jump",
                    f"FK branch continuity gate triggered: pos_jump={pos_jump*1000:.2f}mm > {self.max_pos_jump_m*1000:.2f}mm, so3_jump={math.degrees(so3_jump):.2f}deg > {math.degrees(self.max_ang_jump_rad):.2f}deg. Rejecting pose.",
                )
                return None

        # Accept pose and update warm-start anchor
        self._last_successful_pose = list(fk_res.pose)
        self._last_successful_R = [list(row) for row in fk_res.rotation_matrix]
        self._fk_success_count += 1

        qx, qy, qz, qw = euler_zyx_to_quaternion(fk_res.pose[3], fk_res.pose[4], fk_res.pose[5])

        # Construct and publish FkStateStamped
        msg = FkStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.query_monotonic_timestamp_ns = query_timestamp_ns
        msg.synced_positions_meters = list(sync_res.positions_meters)
        msg.synced_raw_steps = list(sync_res.raw_steps)
        msg.pose.position.x = fk_res.pose[0]
        msg.pose.position.y = fk_res.pose[1]
        msg.pose.position.z = fk_res.pose[2]
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.euler_angles_rad = [fk_res.pose[3], fk_res.pose[4], fk_res.pose[5]]
        msg.converged = bool(fk_res.converged)
        msg.iterations = int(fk_res.iterations)
        msg.residual_norm_meters = float(fk_res.residual_norm)
        msg.branch_continuity_passed = True
        msg.compute_time_us = float(compute_time_us)

        if not self.publish_fk:
            return msg

        self.fk_state_pub.publish(msg)

        # Standard PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        self.fk_pose_pub.publish(pose_msg)

        return msg

    @property
    def metrics(self) -> dict:
        """Return diagnostic counters and timing distributions."""
        return {
            "total_queries": self._total_queries,
            "synced_count": self._synced_count,
            "wait_count": self._wait_count,
            "fk_success_count": self._fk_success_count,
            "fk_fail_count": self._fk_fail_count,
            "jump_reject_count": self._jump_reject_count,
            "timer_periods_ns": list(self._timer_periods_ns),
            "callback_durations_ns": list(self._callback_durations_ns),
            "feedback_callback_durations_ns": list(self._feedback_callback_durations_ns),
            "feedback_callback_count": self._feedback_callback_count,
            "compute_times_us": list(self._compute_times_us),
        }


def main(args=None):
    rclpy.init(args=args)
    node = Pus6FkNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # A process-level SIGTERM can invalidate the context while spin() is
        # blocked in its wait set. Treat that external shutdown as normal;
        # propagate RCLErrors raised while the context is still alive.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        # rclpy's signal handler may already have shut down the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
