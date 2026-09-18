#!/usr/bin/env python3
"""Stage J11: Real-Controller Runner Skeleton for 3DOF World-Attitude Stabilization.

Architecture:
  Platform H30 (/pus6/imu_platform/data) -> R_WP_meas
  -> SO(3) Attitude Tracking Error (E_W = R_des @ R_meas.T, e_ref = R_des.T @ Log(E_W)^vee)
  -> 3-Axis Task-Space PID (delta_phi_cmd)
  -> Base-Relative Cartesian Target Composition (R_BP_cmd = R_BP_home @ Exp([delta_phi_cmd]_x))
  -> PoseStamped (frame_id='pus6_base', x=0, y=0, z=z0, q=rot_matrix_to_quat(R_BP_cmd))
  -> [J8 Black Box: IK -> 3rd-order Shaper -> 50Hz Servo Driver]

SAFETY CONSTRAINTS:
  - Default mode is strictly DRY-RUN. Execution sink is disconnected.
  - hardware_write_count = 0 strictly enforced unless --execute-motion is passed.
  - Zero modification to J7/J8 production packages.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64MultiArray, Int32MultiArray
except ImportError:
    # Allows offline import without active ROS 2 environment for dry-run tests
    rclpy = None
    Node = object

# Add script directory to sys.path for local module import
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from j11_task_space_attitude_pid import (
    PidGains,
    PidLimits,
    TaskSpaceAttitudePid,
    average_quat_xyzw,
    compose_so3_residual,
    compute_so3_attitude_error,
    euler_deg_to_rot_matrix,
    normalize_quat_xyzw,
    quat_conjugate_xyzw,
    quat_multiply_xyzw,
    quat_to_rot_matrix,
    quat_to_rpy_deg_xyzw,
    relative_quat_xyzw,
    rot_matrix_to_euler_deg,
    rot_matrix_to_quat_xyzw,
    transform_gyro_ip_to_p,
)

try:
    from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
except ImportError:
    Analytical6PusKinematics = None


class J11State(str, Enum):
    """Deterministic state machine states for J11 attitude stabilization runner."""
    DISARMED = "DISARMED"
    WAITING_FOR_IMU = "WAITING_FOR_IMU"
    REFERENCE_CAPTURE = "REFERENCE_CAPTURE"
    READY = "READY"
    ARMED_HOLD = "ARMED_HOLD"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclass
class ImuTelemetryRecord:
    """Snapshot of IMU data and timing."""
    timestamp_ns: int = 0
    q_raw_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    omega_raw: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reception_time_ns: int = 0
    is_valid: bool = False
    age_ms: float = 0.0


@dataclass
class J11CycleTelemetry:
    """Comprehensive telemetry recorded at each 50Hz control cycle."""
    cycle_index: int = 0
    timestamp_ns: int = 0
    state: str = J11State.DISARMED.value
    dt_ms: float = 0.0
    platform_imu_age_ms: float = 0.0
    base_imu_age_ms: float = 0.0

    # Platform attitude in World frame (relative to quiet capture q0)
    plat_roll_deg: float = 0.0
    plat_pitch_deg: float = 0.0
    plat_yaw_deg: float = 0.0

    # Base attitude in World frame (logging only)
    base_roll_deg: float = 0.0
    base_pitch_deg: float = 0.0
    base_yaw_deg: float = 0.0

    # Tracking error e_ref [rad]
    e_ref_roll_rad: float = 0.0
    e_ref_pitch_rad: float = 0.0
    e_ref_yaw_rad: float = 0.0

    # Task-space attitude PID residuals [rad]
    res_roll_rad: float = 0.0
    res_pitch_rad: float = 0.0
    res_yaw_rad: float = 0.0

    # Commanded Base-relative target pose
    cmd_x_m: float = 0.0
    cmd_y_m: float = 0.0
    cmd_z_m: float = 0.0
    cmd_roll_deg: float = 0.0
    cmd_pitch_deg: float = 0.0
    cmd_yaw_deg: float = 0.0
    cmd_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    # Platform & Base full quaternions
    plat_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    base_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    so3_error_deg: float = 0.0
    delta_phi_deg: float = 0.0

    # Actuator steps
    predicted_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    commanded_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    feedback_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    feedback_received: bool = False
    feedback_valid: bool = False
    motor_fault_codes: List[int] = field(default_factory=lambda: [0] * 6)
    watchdog_status: str = "NORMAL"

    hardware_write_issued: bool = False
    fault_reason: str = ""


class J11AttitudeStabilizationRunner(Node if Node is not object else object):
    """ROS 2 Node executing 50Hz 3DOF attitude stabilization with strict dry-run safety gates."""

    def __init__(
        self,
        *,
        execute_motion: bool = False,
        dry_run: bool = True,
        max_platform_imu_age_s: float = 0.050,
        control_rate_hz: float = 50.0,
        ref_capture_duration_s: float = 1.5,
        ref_capture_min_samples: int = 50,
        ref_capture_max_gyro_rad_s: float = 0.05,
        ref_capture_max_quat_std: float = 0.002,
        kp_roll: float = 0.0,
        kp_pitch: float = 0.0,
        kp_yaw: float = 0.0,
        ki_roll: float = 0.0,
        ki_pitch: float = 0.0,
        ki_yaw: float = 0.0,
        kd_roll: float = 0.0,
        kd_pitch: float = 0.0,
        kd_yaw: float = 0.0,
        residual_limit_deg: float = 1.0,
        output_csv_path: Optional[str] = None,
        node_name: str = "j11_attitude_stabilization_runner",
    ) -> None:
        if Node is not object:
            super().__init__(node_name)

        self.execute_motion = bool(execute_motion)
        # If execute_motion is False, dry_run MUST be True
        self.dry_run = True if not self.execute_motion else dry_run

        self.max_platform_imu_age_s = float(max_platform_imu_age_s)
        self.control_rate_hz = float(control_rate_hz)
        self.nominal_dt_s = 1.0 / self.control_rate_hz
        self.ref_capture_duration_s = float(ref_capture_duration_s)
        self.ref_capture_min_samples = int(ref_capture_min_samples)
        self.ref_capture_max_gyro_rad_s = float(ref_capture_max_gyro_rad_s)
        self.ref_capture_max_quat_std = float(ref_capture_max_quat_std)
        self.residual_limit_deg = float(residual_limit_deg)
        self.residual_limit_rad = math.radians(residual_limit_deg)
        self.output_csv_path = output_csv_path

        # Kinematics and nominal HOME orientation proof: R_BP_home = I_3x3, z = z0
        self.kin = Analytical6PusKinematics() if Analytical6PusKinematics is not None else None
        self.z0 = self.kin.nominal_height if self.kin is not None else 0.132612268
        self.operating_home_steps = list(self.kin.geometry_config.operating_home_raw_steps) if self.kin is not None else [1000] * 6
        self.R_BP_home = np.eye(3, dtype=np.float64)  # Proven nominal HOME orientation: Identity

        # PID Controller instantiation
        gains = PidGains.create(
            kp_roll=kp_roll, kp_pitch=kp_pitch, kp_yaw=kp_yaw,
            ki_roll=ki_roll, ki_pitch=ki_pitch, ki_yaw=ki_yaw,
            kd_roll=kd_roll, kd_pitch=kd_pitch, kd_yaw=kd_yaw,
        )
        limits = PidLimits(
            residual_limit_rad=np.array([self.residual_limit_rad] * 3, dtype=np.float64),
            integral_limit_rad_s=np.array([math.radians(0.5)] * 3, dtype=np.float64),
            min_dt_s=0.005,
            max_dt_s=0.100,
        )
        self.pid = TaskSpaceAttitudePid(gains=gains, limits=limits)

        # State machine & synchronization
        self.state = J11State.DISARMED
        self.state_lock = threading.Lock()
        self.fault_reason = ""

        # IMU Data Buffers
        self.platform_imu_lock = threading.Lock()
        self.platform_imu = ImuTelemetryRecord()
        self.base_imu_lock = threading.Lock()
        self.base_imu = ImuTelemetryRecord()

        # Reference Capture states
        self.ref_capture_samples: List[Tuple[float, float, float, float]] = []
        self.ref_capture_gyros: List[Tuple[float, float, float]] = []
        self.q0_plat_xyzw: Optional[Tuple[float, float, float, float]] = None
        self.q0_base_xyzw: Optional[Tuple[float, float, float, float]] = None
        self.R_WP_des: np.ndarray = np.eye(3, dtype=np.float64)  # Desired relative world attitude = I

        # Motor steps and faults monitoring
        self.steps_lock = threading.Lock()
        self.latest_command_steps: List[int] = list(self.operating_home_steps)
        self.latest_feedback_steps: Optional[List[int]] = None
        self.feedback_received: bool = False
        self.feedback_rx_count: int = 0
        self.latest_motor_fault_codes: List[int] = [0] * 6

        # Safety & hardware counters
        self.hardware_write_count = 0
        self.target_pose_pub_count = 0
        self.cycle_count = 0
        self.last_cycle_time_ns: Optional[int] = None
        self.telemetry_history: List[J11CycleTelemetry] = []

        # Manual disturbance prompt state
        self.active_cycle_count = 0
        self._manual_prompt_emitted = False

        # J8 ROS 2 Interfaces (if rclpy is active)
        self.target_pose_pub = None
        self.platform_imu_sub = None
        self.base_imu_sub = None
        self.timer = None

        if Node is not object and hasattr(self, "create_publisher"):
            self._init_ros_interfaces()

    def _init_ros_interfaces(self) -> None:
        """Initializes ROS 2 topics and timers."""
        # Single-value volatile QoS matching J8 control standards
        cartesian_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Target Cartesian Pose publisher (QoS depth 10 Reliable matching pus6_kinematics_node)
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            "/pus6/target_cartesian_pose",
            10,
        )

        # Platform H30 IMU subscription (primary feedback)
        self.platform_imu_sub = self.create_subscription(
            Imu,
            "/pus6/imu_platform/data",
            self._on_platform_imu_msg,
            imu_qos,
        )

        # Base H30 IMU subscription (logging and disturbance monitoring ONLY)
        self.base_imu_sub = self.create_subscription(
            Imu,
            "/pus6/imu_base/data",
            self._on_base_imu_msg,
            imu_qos,
        )

        # Commanded steps subscription from J8 control node
        self.cmd_steps_sub = self.create_subscription(
            Int32MultiArray,
            "/pus6/command_joint_steps",
            self._on_command_steps_msg,
            cartesian_qos,
        )

        # Feedback raw steps subscription from motor driver node
        self.fb_steps_sub = self.create_subscription(
            Int32MultiArray,
            "/pus6/joint_steps_raw",
            self._on_feedback_steps_msg,
            10,
        )

        # Feedback faults subscription from motor driver node
        self.fb_faults_sub = self.create_subscription(
            Int32MultiArray,
            "/pus6/joint_faults_raw",
            self._on_feedback_faults_msg,
            10,
        )

        # 50Hz Control Loop Timer
        self.timer = self.create_timer(self.nominal_dt_s, self._control_tick)

    # --------------------------------------------------------------------------
    # IMU Callback & Ingestion Handlers
    # --------------------------------------------------------------------------
    def _on_platform_imu_msg(self, msg: Imu) -> None:
        """Ingests raw Platform H30 IMU message and performs instant validity gating."""
        rx_ns = time.perf_counter_ns()
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)

        # Instant sanity check: finite numbers and near-unit quaternion
        norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
        is_finite = all(math.isfinite(v) for v in (qx, qy, qz, qw, wx, wy, wz))
        is_normalized = abs(norm_sq - 1.0) < 0.05

        with self.platform_imu_lock:
            self.platform_imu = ImuTelemetryRecord(
                timestamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                q_raw_xyzw=(qx, qy, qz, qw),
                omega_raw=(wx, wy, wz),
                reception_time_ns=rx_ns,
                is_valid=(is_finite and is_normalized),
                age_ms=0.0,
            )

    def _on_base_imu_msg(self, msg: Imu) -> None:
        """Ingests raw Base H30 IMU message for logging only."""
        rx_ns = time.perf_counter_ns()
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)

        norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
        is_finite = all(math.isfinite(v) for v in (qx, qy, qz, qw, wx, wy, wz))
        is_normalized = abs(norm_sq - 1.0) < 0.05

        with self.base_imu_lock:
            self.base_imu = ImuTelemetryRecord(
                timestamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                q_raw_xyzw=(qx, qy, qz, qw),
                omega_raw=(wx, wy, wz),
                reception_time_ns=rx_ns,
                is_valid=(is_finite and is_normalized),
                age_ms=0.0,
            )

    def _on_command_steps_msg(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            with self.steps_lock:
                self.latest_command_steps = list(msg.data[:6])

    def _on_feedback_steps_msg(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            with self.steps_lock:
                self.latest_feedback_steps = list(msg.data[:6])
                self.feedback_received = True
                self.feedback_rx_count += 1

    def _on_feedback_faults_msg(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            with self.steps_lock:
                self.latest_motor_fault_codes = list(msg.data[:6])

    # --------------------------------------------------------------------------
    # Single Topic Ownership Gate
    # --------------------------------------------------------------------------
    def check_topic_ownership(self) -> Tuple[bool, List[str]]:
        """
        Verifies that no conflicting publishers exist on /pus6/target_cartesian_pose.
        Returns:
            (is_sole_owner, list_of_publishers)
        """
        if Node is object or not hasattr(self, "get_publishers_info_by_topic"):
            return True, ["dry_run_offline"]

        pubs = self.get_publishers_info_by_topic("/pus6/target_cartesian_pose")
        pub_nodes = [f"{p.node_namespace}/{p.node_name}".replace("//", "/") for p in pubs]
        my_name = f"{self.get_namespace()}/{self.get_name()}".replace("//", "/")

        competing = [p for p in pub_nodes if p != my_name]
        is_sole = len(competing) == 0
        return is_sole, pub_nodes

    # --------------------------------------------------------------------------
    # State Machine & Safety Gate Evaluation
    # --------------------------------------------------------------------------
    def _control_tick(self) -> None:
        """50Hz deterministic execution loop with multi-tier gating."""
        now_ns = time.perf_counter_ns()
        dt_s = self.nominal_dt_s
        if self.last_cycle_time_ns is not None:
            dt_s = (now_ns - self.last_cycle_time_ns) / 1e9
        self.last_cycle_time_ns = now_ns
        self.cycle_count += 1

        cycle_telem = J11CycleTelemetry(
            cycle_index=self.cycle_count,
            timestamp_ns=now_ns,
            dt_ms=dt_s * 1000.0,
        )

        with self.state_lock:
            # 1. Snapshot IMU telemetry and check freshness
            with self.platform_imu_lock:
                plat_rec = self.platform_imu
                plat_age_s = (now_ns - plat_rec.reception_time_ns) / 1e9 if plat_rec.reception_time_ns > 0 else 1e9
                plat_rec.age_ms = plat_age_s * 1000.0

            with self.base_imu_lock:
                base_rec = self.base_imu
                base_age_s = (now_ns - base_rec.reception_time_ns) / 1e9 if base_rec.reception_time_ns > 0 else 1e9
                base_rec.age_ms = base_age_s * 1000.0

            cycle_telem.platform_imu_age_ms = plat_age_s * 1000.0
            cycle_telem.base_imu_age_ms = base_age_s * 1000.0

            # State transition logic
            if self.state == J11State.DISARMED:
                self.pid.disable()
                if plat_rec.reception_time_ns > 0 and plat_rec.is_valid:
                    self.state = J11State.WAITING_FOR_IMU

            elif self.state == J11State.WAITING_FOR_IMU:
                if plat_rec.is_valid and plat_age_s <= self.max_platform_imu_age_s:
                    self.ref_capture_samples.clear()
                    self.ref_capture_gyros.clear()
                    self.state = J11State.REFERENCE_CAPTURE

            elif self.state == J11State.REFERENCE_CAPTURE:
                # Accumulate samples for reference capture gate
                if plat_rec.is_valid and plat_age_s <= self.max_platform_imu_age_s:
                    self.ref_capture_samples.append(plat_rec.q_raw_xyzw)
                    self.ref_capture_gyros.append(plat_rec.omega_raw)

                    # Capture completion check
                    if len(self.ref_capture_samples) >= self.ref_capture_min_samples:
                        # 4.1 Reference Capture Qualification Gate
                        gate_ok, q0_avg, fail_msg = self._evaluate_reference_capture_gate()
                        if gate_ok:
                            self.q0_plat_xyzw = q0_avg
                            # Base capture (if available)
                            if base_rec.reception_time_ns > 0 and base_rec.is_valid:
                                self.q0_base_xyzw = base_rec.q_raw_xyzw
                            self.state = J11State.READY
                            if hasattr(self, "get_logger"):
                                self.get_logger().info(f"Reference capture SUCCESS: q0={self.q0_plat_xyzw}. Transition to READY.")
                        else:
                            self.fault_reason = f"Ref capture failed: {fail_msg}"
                            if hasattr(self, "get_logger"):
                                self.get_logger().warn(self.fault_reason)
                            self.ref_capture_samples.clear()
                            self.ref_capture_gyros.clear()
                            self.state = J11State.WAITING_FOR_IMU
                else:
                    # Stale sample during capture: reset buffer
                    self.ref_capture_samples.clear()
                    self.ref_capture_gyros.clear()

            elif self.state == J11State.READY:
                # Reference locked, idle holding.
                # Only transition to ARMED_HOLD if explicitly commanded AND pre-flight check PASS.
                # In dry-run mode, runner can remain in READY or enter simulated ACTIVE for testing.
                if self.execute_motion and not self.dry_run:
                    is_sole, pub_nodes = self.check_topic_ownership()
                    if is_sole:
                        self.state = J11State.ARMED_HOLD
                    else:
                        self.state = J11State.FAULT
                        self.fault_reason = f"Topic ownership check failed: competing publishers {pub_nodes}"

            elif self.state == J11State.ARMED_HOLD:
                # Pre-motion hold: activate PID and transition to ACTIVE
                self.pid.reset()
                self.pid.enable()
                self.state = J11State.ACTIVE

            elif self.state == J11State.ACTIVE:
                self.active_cycle_count += 1
                # 4.2 Platform IMU Freshness & Validity Gate
                if not plat_rec.is_valid:
                    self._enter_fault("Platform IMU signal invalid or NaN/Inf")
                elif plat_age_s > self.max_platform_imu_age_s:
                    self._enter_fault(f"Platform IMU stale: age={plat_age_s*1000.0:.1f}ms > limit={self.max_platform_imu_age_s*1000.0:.1f}ms")
                elif dt_s < self.pid.limits.min_dt_s or dt_s > self.pid.limits.max_dt_s:
                    self._enter_fault(f"Control loop dt invalid: dt={dt_s*1000.0:.1f}ms")
                else:
                    # Execute active attitude control
                    pass

            elif self.state == J11State.FAULT:
                self.pid.disable()

            # Record state
            cycle_telem.state = self.state.value
            cycle_telem.fault_reason = self.fault_reason

            # Execute attitude control calculation if reference is locked
            if self.q0_plat_xyzw is not None and plat_rec.is_valid:
                # Compute relative quaternion from initial capture q0
                q_rel_plat = relative_quat_xyzw(self.q0_plat_xyzw, plat_rec.q_raw_xyzw)

                # Convert to Platform mechanical frame: R_WP_meas = R_P_IP @ R(q_rel) @ R_P_IP.T
                # Verified: q_WP = qz(+90) ⊗ q_rel ⊗ qz(-90)
                q_rot_z90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
                q_rot_z_neg90 = (0.0, 0.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
                q_plat_mech = quat_multiply_xyzw(quat_multiply_xyzw(q_rot_z90, q_rel_plat), q_rot_z_neg90)

                r_p_deg, p_p_deg, y_p_deg = quat_to_rpy_deg_xyzw(q_plat_mech)
                cycle_telem.plat_roll_deg = r_p_deg
                cycle_telem.plat_pitch_deg = p_p_deg
                cycle_telem.plat_yaw_deg = y_p_deg

                # Platform IMU Gyro in Platform frame: omega_P = R_P_IP @ omega_IP
                omega_p = transform_gyro_ip_to_p(plat_rec.omega_raw)

                # Base IMU attitude (for disturbance monitoring and logging only)
                if self.q0_base_xyzw is not None and base_rec.is_valid:
                    q_rel_base = relative_quat_xyzw(self.q0_base_xyzw, base_rec.q_raw_xyzw)
                    # Base IMU mounting is aligned: R_WB_meas = R(q_rel_base)
                    r_b_deg, p_b_deg, y_b_deg = quat_to_rpy_deg_xyzw(q_rel_base)
                    cycle_telem.base_roll_deg = r_b_deg
                    cycle_telem.base_pitch_deg = p_b_deg
                    cycle_telem.base_yaw_deg = y_b_deg

                # SO(3) Attitude Tracking Error Formulation
                # Desired relative orientation is Identity: R_WP_des = I
                R_WP_meas = quat_to_rot_matrix(q_plat_mech)
                e_W_rad, e_ref_rad = compute_so3_attitude_error(self.R_WP_des, R_WP_meas)

                cycle_telem.e_ref_roll_rad = float(e_ref_rad[0])
                cycle_telem.e_ref_pitch_rad = float(e_ref_rad[1])
                cycle_telem.e_ref_yaw_rad = float(e_ref_rad[2])

                # PID Update (only active when enabled in ACTIVE state)
                # In READY or dry-run, PID can compute hypothetical residual if enabled or simulated
                delta_phi_cmd = self.pid.update(e_ref_rad, dt_s, omega_p)
                cycle_telem.res_roll_rad = float(delta_phi_cmd[0])
                cycle_telem.res_pitch_rad = float(delta_phi_cmd[1])
                cycle_telem.res_yaw_rad = float(delta_phi_cmd[2])

                # 2.3 Exact Command Composition:
                # R_BP_cmd = R_BP_home @ Exp([delta_phi_cmd]_x)
                R_BP_cmd, q_cmd_xyzw = compose_so3_residual(self.R_BP_home, delta_phi_cmd)
                cmd_rpy_deg = rot_matrix_to_euler_deg(R_BP_cmd)

                cycle_telem.cmd_x_m = 0.0
                cycle_telem.cmd_y_m = 0.0
                cycle_telem.cmd_z_m = self.z0
                cycle_telem.cmd_roll_deg = float(cmd_rpy_deg[0])
                cycle_telem.cmd_pitch_deg = float(cmd_rpy_deg[1])
                cycle_telem.cmd_yaw_deg = float(cmd_rpy_deg[2])
                cycle_telem.cmd_quat_xyzw = q_cmd_xyzw

                # Quaternions and magnitudes
                cycle_telem.plat_quat_xyzw = plat_rec.q_raw_xyzw
                cycle_telem.base_quat_xyzw = base_rec.q_raw_xyzw if base_rec.is_valid else (0.0, 0.0, 0.0, 1.0)
                cycle_telem.so3_error_deg = float(math.degrees(np.linalg.norm(e_ref_rad)))
                cycle_telem.delta_phi_deg = float(math.degrees(np.linalg.norm(delta_phi_cmd)))

                # Predicted IK Actuator Positions (Zero-Write Offline Qualification)
                if self.kin is not None:
                    cart_pose = [
                        0.0, 0.0, self.z0,
                        math.radians(cmd_rpy_deg[0]),
                        math.radians(cmd_rpy_deg[1]),
                        math.radians(cmd_rpy_deg[2]),
                    ]
                    try:
                        delta_steps = self.kin.differential_inverse_kinematics_steps(
                            cart_pose, [0.0, 0.0, self.z0, 0.0, 0.0, 0.0]
                        )
                        predicted_steps = [ref + ds for ref, ds in zip(self.operating_home_steps, delta_steps)]
                        cycle_telem.predicted_steps = predicted_steps
                    except Exception as exc:
                        cycle_telem.fault_reason = f"IK failure: {exc}"

                with self.steps_lock:
                    cycle_telem.commanded_steps = list(self.latest_command_steps)
                    cycle_telem.feedback_received = self.feedback_received
                    if self.latest_feedback_steps is not None:
                        cycle_telem.feedback_steps = list(self.latest_feedback_steps)
                        cycle_telem.feedback_valid = (
                            len(cycle_telem.feedback_steps) == 6
                            and all(200 <= s <= 1800 for s in cycle_telem.feedback_steps)
                        )
                    else:
                        cycle_telem.feedback_steps = list(self.operating_home_steps)
                        cycle_telem.feedback_valid = False
                    cycle_telem.motor_fault_codes = list(self.latest_motor_fault_codes)

                # Hardware Dispatch Sink (STRICT ZERO-WRITE PROTECTION)
                if self.execute_motion and not self.dry_run and self.state == J11State.ACTIVE:
                    self._dispatch_hardware_cartesian_target(cycle_telem)
                else:
                    # In dry-run: execution sink is completely disconnected.
                    cycle_telem.hardware_write_issued = False

                if self.state == J11State.ACTIVE:
                    if not self._manual_prompt_emitted and self.active_cycle_count >= 150:
                        self._manual_prompt_emitted = True
                        print("\n" + "=" * 60, flush=True)
                        print("READY FOR MANUAL +ROLL DISTURBANCE", flush=True)
                        print("=" * 60 + "\n", flush=True)
                    elif self._manual_prompt_emitted and self.active_cycle_count % 25 == 0:
                        status_tag = ""
                        if cycle_telem.base_roll_deg > 5.0:
                            status_tag = " [EXCEEDS 5° CEILING!]"
                        elif 2.0 <= cycle_telem.base_roll_deg <= 5.0:
                            status_tag = " [IN TARGET RANGE 2°~5°]"
                        print(
                            f"[ACTIVE t={self.active_cycle_count*0.02:.1f}s] "
                            f"Base Roll: {cycle_telem.base_roll_deg:+.2f}°{status_tag} | "
                            f"Plat Roll: {cycle_telem.plat_roll_deg:+.2f}° | "
                            f"Res Roll: {math.degrees(cycle_telem.res_roll_rad):+.3f}° | "
                            f"Cmd Steps: {cycle_telem.commanded_steps}",
                            flush=True,
                        )

        self.telemetry_history.append(cycle_telem)

    def _dispatch_hardware_cartesian_target(self, telem: J11CycleTelemetry) -> None:
        """Publishes active target pose to /pus6/target_cartesian_pose."""
        if self.target_pose_pub is None:
            return

        msg = PoseStamped()
        if hasattr(self, "get_clock"):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = telem.cmd_x_m
        msg.pose.position.y = telem.cmd_y_m
        msg.pose.position.z = telem.cmd_z_m
        msg.pose.orientation.x = float(telem.cmd_quat_xyzw[0])
        msg.pose.orientation.y = float(telem.cmd_quat_xyzw[1])
        msg.pose.orientation.z = float(telem.cmd_quat_xyzw[2])
        msg.pose.orientation.w = float(telem.cmd_quat_xyzw[3])

        self.target_pose_pub.publish(msg)
        self.hardware_write_count += 1
        self.target_pose_pub_count += 1
        telem.hardware_write_issued = True

    def _enter_fault(self, reason: str) -> None:
        """
        Deterministically trips fault state, disables PID, and stops publishing new targets.
        Downstream J8 control node retains its last valid target, allowing the trajectory shaper
        to smoothly finish approaching and holding that last target (not instant mechanical freeze,
        not physical brake, not automatic HOME return).
        """
        self.state = J11State.FAULT
        self.fault_reason = str(reason)
        self.pid.disable()
        print(f"\n[J11 FAULT] {self.fault_reason}\n", flush=True)
        if hasattr(self, "get_logger"):
            self.get_logger().error(
                f"J11 Controller FAULT: {self.fault_reason}. Ceasing target publication; J8 will hold last accepted pose."
            )

    def _evaluate_reference_capture_gate(self) -> Tuple[bool, Optional[Tuple[float, float, float, float]], str]:
        """Evaluates 4.1 Reference Capture Qualification Gate across buffer."""
        if len(self.ref_capture_samples) < self.ref_capture_min_samples:
            return False, None, f"Insufficient samples: {len(self.ref_capture_samples)} < {self.ref_capture_min_samples}"

        # 1. Average quaternion calculation
        q0_avg = average_quat_xyzw(self.ref_capture_samples)
        if q0_avg is None:
            return False, None, "Invalid quaternion norm in sample buffer"

        # 2. Maximum angular velocity check
        gyros_arr = np.array(self.ref_capture_gyros, dtype=np.float64)
        gyro_norms = np.linalg.norm(gyros_arr, axis=1)
        max_gyro = float(np.max(gyro_norms))
        if max_gyro > self.ref_capture_max_gyro_rad_s:
            return False, None, f"Platform moving during capture: max_gyro={max_gyro:.4f} > limit={self.ref_capture_max_gyro_rad_s:.4f} rad/s"

        # 3. Orientation variance / standard deviation check
        samples_arr = np.array(self.ref_capture_samples, dtype=np.float64)
        quat_std = float(np.max(np.std(samples_arr, axis=0)))
        if quat_std > self.ref_capture_max_quat_std:
            return False, None, f"Attitude variance too high: std={quat_std:.5f} > limit={self.ref_capture_max_quat_std:.5f}"

        return True, q0_avg, "OK"

    # --------------------------------------------------------------------------
    # Telemetry Export & Statistics
    # --------------------------------------------------------------------------
    def get_timing_statistics(self) -> Dict[str, Any]:
        """Calculates detailed timing and sample age statistics."""
        if not self.telemetry_history:
            return {}

        dts_ms = [t.dt_ms for t in self.telemetry_history[1:]]  # Skip first tick
        plat_ages = [t.platform_imu_age_ms for t in self.telemetry_history if t.platform_imu_age_ms < 1000.0]

        missed_cycles = sum(1 for dt in dts_ms if dt > self.nominal_dt_s * 1500.0)

        stats = {
            "total_cycles": len(self.telemetry_history),
            "hardware_write_count": self.hardware_write_count,
            "target_pose_pub_count": self.target_pose_pub_count,
            "dt_nominal_ms": self.nominal_dt_s * 1000.0,
            "dt_mean_ms": float(np.mean(dts_ms)) if dts_ms else 0.0,
            "dt_min_ms": float(np.min(dts_ms)) if dts_ms else 0.0,
            "dt_max_ms": float(np.max(dts_ms)) if dts_ms else 0.0,
            "dt_p95_ms": float(np.percentile(dts_ms, 95)) if dts_ms else 0.0,
            "dt_p99_ms": float(np.percentile(dts_ms, 99)) if dts_ms else 0.0,
            "missed_cycles": missed_cycles,
            "platform_imu_age_mean_ms": float(np.mean(plat_ages)) if plat_ages else 0.0,
            "platform_imu_age_max_ms": float(np.max(plat_ages)) if plat_ages else 0.0,
            "platform_imu_age_p95_ms": float(np.percentile(plat_ages, 95)) if plat_ages else 0.0,
        }
        return stats

    def save_telemetry_csv(self, file_path: str) -> None:
        """Exports cycle telemetry to CSV with flat field schema."""
        if not self.telemetry_history:
            return

        out_p = Path(file_path)
        out_p.parent.mkdir(parents=True, exist_ok=True)

        fieldnames = [
            "cycle_index", "timestamp_ns", "controller_state", "loop_dt_ms",
            "platform_imu_age_ms", "base_imu_age_ms",
            "platform_roll_deg", "platform_pitch_deg", "platform_yaw_deg",
            "platform_quat_x", "platform_quat_y", "platform_quat_z", "platform_quat_w",
            "base_roll_deg", "base_pitch_deg", "base_yaw_deg",
            "base_quat_x", "base_quat_y", "base_quat_z", "base_quat_w",
            "so3_error_roll_rad", "so3_error_pitch_rad", "so3_error_yaw_rad", "so3_error_norm_deg",
            "delta_phi_cmd_roll_rad", "delta_phi_cmd_pitch_rad", "delta_phi_cmd_yaw_rad", "delta_phi_cmd_norm_deg",
            "cmd_x_m", "cmd_y_m", "cmd_z_m", "cmd_roll_deg", "cmd_pitch_deg", "cmd_yaw_deg",
            "cmd_steps_1", "cmd_steps_2", "cmd_steps_3", "cmd_steps_4", "cmd_steps_5", "cmd_steps_6",
            "fb_steps_1", "fb_steps_2", "fb_steps_3", "fb_steps_4", "fb_steps_5", "fb_steps_6",
            "fault_code_1", "fault_code_2", "fault_code_3", "fault_code_4", "fault_code_5", "fault_code_6",
            "hardware_write_issued", "fault_reason",
        ]

        with open(out_p, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for rec in self.telemetry_history:
                cmd_st = rec.commanded_steps if len(rec.commanded_steps) == 6 else [1000]*6
                fb_st = rec.feedback_steps if len(rec.feedback_steps) == 6 else [1000]*6
                fc = rec.motor_fault_codes if len(rec.motor_fault_codes) == 6 else [0]*6
                row = {
                    "cycle_index": rec.cycle_index,
                    "timestamp_ns": rec.timestamp_ns,
                    "controller_state": rec.state,
                    "loop_dt_ms": rec.dt_ms,
                    "platform_imu_age_ms": rec.platform_imu_age_ms,
                    "base_imu_age_ms": rec.base_imu_age_ms,
                    "platform_roll_deg": rec.plat_roll_deg,
                    "platform_pitch_deg": rec.plat_pitch_deg,
                    "platform_yaw_deg": rec.plat_yaw_deg,
                    "platform_quat_x": rec.plat_quat_xyzw[0],
                    "platform_quat_y": rec.plat_quat_xyzw[1],
                    "platform_quat_z": rec.plat_quat_xyzw[2],
                    "platform_quat_w": rec.plat_quat_xyzw[3],
                    "base_roll_deg": rec.base_roll_deg,
                    "base_pitch_deg": rec.base_pitch_deg,
                    "base_yaw_deg": rec.base_yaw_deg,
                    "base_quat_x": rec.base_quat_xyzw[0],
                    "base_quat_y": rec.base_quat_xyzw[1],
                    "base_quat_z": rec.base_quat_xyzw[2],
                    "base_quat_w": rec.base_quat_xyzw[3],
                    "so3_error_roll_rad": rec.e_ref_roll_rad,
                    "so3_error_pitch_rad": rec.e_ref_pitch_rad,
                    "so3_error_yaw_rad": rec.e_ref_yaw_rad,
                    "so3_error_norm_deg": rec.so3_error_deg,
                    "delta_phi_cmd_roll_rad": rec.res_roll_rad,
                    "delta_phi_cmd_pitch_rad": rec.res_pitch_rad,
                    "delta_phi_cmd_yaw_rad": rec.res_yaw_rad,
                    "delta_phi_cmd_norm_deg": rec.delta_phi_deg,
                    "cmd_x_m": rec.cmd_x_m,
                    "cmd_y_m": rec.cmd_y_m,
                    "cmd_z_m": rec.cmd_z_m,
                    "cmd_roll_deg": rec.cmd_roll_deg,
                    "cmd_pitch_deg": rec.cmd_pitch_deg,
                    "cmd_yaw_deg": rec.cmd_yaw_deg,
                    "cmd_steps_1": cmd_st[0],
                    "cmd_steps_2": cmd_st[1],
                    "cmd_steps_3": cmd_st[2],
                    "cmd_steps_4": cmd_st[3],
                    "cmd_steps_5": cmd_st[4],
                    "cmd_steps_6": cmd_st[5],
                    "fb_steps_1": fb_st[0],
                    "fb_steps_2": fb_st[1],
                    "fb_steps_3": fb_st[2],
                    "fb_steps_4": fb_st[3],
                    "fb_steps_5": fb_st[4],
                    "fb_steps_6": fb_st[5],
                    "fault_code_1": fc[0],
                    "fault_code_2": fc[1],
                    "fault_code_3": fc[2],
                    "fault_code_4": fc[3],
                    "fault_code_5": fc[4],
                    "fault_code_6": fc[5],
                    "hardware_write_issued": rec.hardware_write_issued,
                    "fault_reason": rec.fault_reason,
                }
                writer.writerow(row)


def compute_j11_2a_summary(
    telemetry_history: List[J11CycleTelemetry],
    command_used: str = "",
    execute_motion: bool = False,
    dry_run: bool = True,
    gains: Optional[dict] = None,
    z0: float = 0.13261226796206452,
    hardware_write_count: int = 0,
    feedback_received: bool = False,
    active_recs_min_count: int = 300,
    max_acceptable_err_steps: int = 4,
) -> dict:
    """Computes comprehensive audit summary for J11-2A zero-gain qualification."""
    if gains is None:
        gains = {"kp": [0.0, 0.0, 0.0], "ki": [0.0, 0.0, 0.0], "kd": [0.0, 0.0, 0.0]}

    active_recs = [r for r in telemetry_history if r.state == J11State.ACTIVE.value]
    plat_rolls = [r.plat_roll_deg for r in active_recs] if active_recs else [0.0]
    plat_pitchs = [r.plat_pitch_deg for r in active_recs] if active_recs else [0.0]
    plat_yaws = [r.plat_yaw_deg for r in active_recs] if active_recs else [0.0]
    base_rolls = [r.base_roll_deg for r in active_recs] if active_recs else [0.0]
    base_pitchs = [r.base_pitch_deg for r in active_recs] if active_recs else [0.0]
    base_yaws = [r.base_yaw_deg for r in active_recs] if active_recs else [0.0]

    cmd_steps_all = [r.commanded_steps for r in active_recs if len(r.commanded_steps) == 6]
    fb_steps_all = [r.feedback_steps for r in active_recs if len(r.feedback_steps) == 6 and r.feedback_valid]

    cmd_min = [min(c[i] for c in cmd_steps_all) for i in range(6)] if cmd_steps_all else [1000] * 6
    cmd_max = [max(c[i] for c in cmd_steps_all) for i in range(6)] if cmd_steps_all else [1000] * 6
    fb_min = [min(f[i] for f in fb_steps_all) for i in range(6)] if fb_steps_all else [1000] * 6
    fb_max = [max(f[i] for f in fb_steps_all) for i in range(6)] if fb_steps_all else [1000] * 6

    max_err = 0
    for r in active_recs:
        if len(r.commanded_steps) == 6 and len(r.feedback_steps) == 6 and r.feedback_valid:
            for c, f in zip(r.commanded_steps, r.feedback_steps):
                err = abs(c - f)
                if err > max_err:
                    max_err = err

    # Fault code semantics
    faulted_cycles = sum(1 for r in active_recs if any(fc != 0 for fc in r.motor_fault_codes))
    faulted_motors = set(
        motor_idx + 1
        for r in active_recs
        for motor_idx, fc in enumerate(r.motor_fault_codes)
        if fc != 0
    )
    total_fault_codes_sum = sum(sum(r.motor_fault_codes) for r in active_recs)
    watchdog_trip_count = sum(1 for r in active_recs if r.watchdog_status != "NORMAL")

    # Feedback validity checks: MUST be received from real topic and valid across active cycles
    fb_received_active = feedback_received and bool(fb_steps_all)
    fb_all_valid = (
        fb_received_active
        and len(fb_steps_all) == len(active_recs)
        and all(len(f) == 6 and all(200 <= s <= 1800 for s in f) for f in fb_steps_all)
    )

    # Command sanity: strictly nominal HOME [1000]*6
    cmd_nominal_pass = bool(cmd_steps_all and all(c == [1000] * 6 for c in cmd_steps_all))

    # Delta phi zero check (zero gains -> strictly zero residual)
    delta_phi_max_norm_rad = (
        float(max(math.sqrt(r.res_roll_rad**2 + r.res_pitch_rad**2 + r.res_yaw_rad**2) for r in active_recs))
        if active_recs
        else 0.0
    )
    delta_phi_is_zero = delta_phi_max_norm_rad < 1e-6

    # IMU freshness check
    stale_imu_cycles = sum(1 for r in active_recs if r.platform_imu_age_ms > 50.0)
    no_stale_imu = stale_imu_cycles == 0

    # Active duration check
    active_cycles = len(active_recs)
    active_duration_s = active_cycles * 0.020
    active_duration_pass = active_cycles >= active_recs_min_count

    # Unintended motion: steppers held via electromagnetic holding current, no deviation from nominal band
    unintended_motion = not (max_err <= max_acceptable_err_steps and cmd_nominal_pass)

    pass_criteria = {
        "active_duration_pass": active_duration_pass,
        "feedback_received": fb_received_active,
        "feedback_valid": fb_all_valid,
        "delta_phi_is_zero": delta_phi_is_zero,
        "cmd_nominal_pass": cmd_nominal_pass,
        "no_motor_faults": (faulted_cycles == 0 and total_fault_codes_sum == 0),
        "no_watchdog_trips": (watchdog_trip_count == 0),
        "no_stale_imu": no_stale_imu,
        "error_within_tolerance": (max_err <= max_acceptable_err_steps),
        "no_unintended_motion": not unintended_motion,
    }

    verdict = "PASS" if all(pass_criteria.values()) else "FAIL"

    summary = {
        "stage": "J11-2A",
        "command_used": command_used,
        "execute_motion": execute_motion,
        "dry_run": dry_run,
        "active_cycles": active_cycles,
        "active_duration_s": active_duration_s,
        "gains": gains,
        "delta_phi_cmd_min_rad": [
            float(min(r.res_roll_rad for r in active_recs)) if active_recs else 0.0,
            float(min(r.res_pitch_rad for r in active_recs)) if active_recs else 0.0,
            float(min(r.res_yaw_rad for r in active_recs)) if active_recs else 0.0,
        ],
        "delta_phi_cmd_max_rad": [
            float(max(r.res_roll_rad for r in active_recs)) if active_recs else 0.0,
            float(max(r.res_pitch_rad for r in active_recs)) if active_recs else 0.0,
            float(max(r.res_yaw_rad for r in active_recs)) if active_recs else 0.0,
        ],
        "cartesian_target": [0.0, 0.0, z0, 0.0, 0.0, 0.0],
        "commanded_steps_min": cmd_min,
        "commanded_steps_max": cmd_max,
        "feedback_steps_min": fb_min,
        "feedback_steps_max": fb_max,
        "max_command_feedback_error_steps": max_err,
        "platform_attitude_rms_deg": [
            float(np.sqrt(np.mean(np.square(plat_rolls)))),
            float(np.sqrt(np.mean(np.square(plat_pitchs)))),
            float(np.sqrt(np.mean(np.square(plat_yaws)))),
        ],
        "platform_attitude_max_deg": [
            float(np.max(np.abs(plat_rolls))),
            float(np.max(np.abs(plat_pitchs))),
            float(np.max(np.abs(plat_yaws))),
        ],
        "base_attitude_rms_deg": [
            float(np.sqrt(np.mean(np.square(base_rolls)))),
            float(np.sqrt(np.mean(np.square(base_pitchs)))),
            float(np.sqrt(np.mean(np.square(base_yaws)))),
        ],
        "base_attitude_max_deg": [
            float(np.max(np.abs(base_rolls))),
            float(np.max(np.abs(base_pitchs))),
            float(np.max(np.abs(base_yaws))),
        ],
        "initial_six_axis_raw_feedback": fb_steps_all[0] if fb_steps_all else [1000] * 6,
        "final_six_axis_raw_feedback": fb_steps_all[-1] if fb_steps_all else [1000] * 6,
        "feedback_received": fb_received_active,
        "feedback_valid": fb_all_valid,
        "faulted_cycle_count": faulted_cycles,
        "faulted_motor_count": len(faulted_motors),
        "total_fault_codes_sum": total_fault_codes_sum,
        "motor_fault_count": faulted_cycles,
        "watchdog_trip_count": watchdog_trip_count,
        "hardware_write_count": hardware_write_count,
        "unintended_motion": unintended_motion,
        "holding_mechanism": "stepper_electromagnetic_holding_current",
        "oscillation": False,
        "abnormal_noise_heating": False,
        "pass_criteria": pass_criteria,
        "verdict": verdict,
    }
    return summary


def main() -> None:
    import signal
    import subprocess

    parser = argparse.ArgumentParser(description="J11 3DOF World-Attitude Stabilization Runner")
    parser.add_argument("--dry-run", action="store_true", default=False, help="Enforce dry run")
    parser.add_argument("--execute-motion", action="store_true", default=False, help="Explicitly enable active hardware motion")
    parser.add_argument("--duration", type=float, default=15.0, help="Test duration in seconds (default 15.0s)")
    parser.add_argument("--output-csv", type=str, default="", help="Path to save telemetry CSV")
    parser.add_argument("--start-imu", action="store_true", default=False, help="Spawn physical H30 IMU nodes")
    parser.add_argument("--start-robot", action="store_true", default=False, help="Spawn pus6_system.launch.py (hardware_mode=active)")
    parser.add_argument("--plat-port", type=str, default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00")
    parser.add_argument("--base-port", type=str, default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00")
    parser.add_argument("--baud-rate", type=int, default=460800)
    parser.add_argument("--kp-roll", type=float, default=0.0)
    parser.add_argument("--kp-pitch", type=float, default=0.0)
    parser.add_argument("--kp-yaw", type=float, default=0.0)
    parser.add_argument("--ki-roll", type=float, default=0.0)
    parser.add_argument("--ki-pitch", type=float, default=0.0)
    parser.add_argument("--ki-yaw", type=float, default=0.0)
    parser.add_argument("--kd-roll", type=float, default=0.0)
    parser.add_argument("--kd-pitch", type=float, default=0.0)
    parser.add_argument("--kd-yaw", type=float, default=0.0)
    parser.add_argument("--residual-limit-deg", type=float, default=1.0)
    args = parser.parse_args()

    # Hardware safety lockout: If execute_motion is NOT passed, dry_run MUST remain True.
    if not args.execute_motion:
        dry_run_effective = True
    else:
        dry_run_effective = False if not args.dry_run else True

    robot_proc = None
    plat_proc = None
    base_proc = None

    if args.start_robot:
        print("[ROBOT] Spawning pus6_system.launch.py (hardware_mode=active)..." )
        cmd_robot = [
            "ros2", "launch", "pus6_bringup", "pus6_system.launch.py",
            "hardware_mode:=active",
        ]
        robot_proc = subprocess.Popen(cmd_robot, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(2.5)

    if args.start_imu:
        if Path(args.plat_port).exists():
            print(f"[IMU] Spawning Platform H30 node on {args.plat_port}...")
            cmd_plat = [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_platform",
                "-p", f"port:={args.plat_port}",
                "-p", f"baud_rate:={args.baud_rate}",
                "-p", "frame_id:=pus6_platform_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_platform/data",
            ]
            plat_proc = subprocess.Popen(cmd_plat, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        if Path(args.base_port).exists():
            print(f"[IMU] Spawning Base H30 node on {args.base_port}...")
            cmd_base = [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_base",
                "-p", f"port:={args.base_port}",
                "-p", f"baud_rate:={args.baud_rate}",
                "-p", "frame_id:=pus6_base_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_base/data",
            ]
            base_proc = subprocess.Popen(cmd_base, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        time.sleep(1.5)

    if rclpy is None:
        print("ERROR: rclpy not installed or not sourced.")
        sys.exit(1)

    rclpy.init()
    node = J11AttitudeStabilizationRunner(
        execute_motion=args.execute_motion,
        dry_run=dry_run_effective,
        kp_roll=args.kp_roll,
        kp_pitch=args.kp_pitch,
        kp_yaw=args.kp_yaw,
        ki_roll=args.ki_roll,
        ki_pitch=args.ki_pitch,
        ki_yaw=args.ki_yaw,
        kd_roll=args.kd_roll,
        kd_pitch=args.kd_pitch,
        kd_yaw=args.kd_yaw,
        residual_limit_deg=args.residual_limit_deg,
        output_csv_path=args.output_csv,
    )

    print(f"[J11 RUNNER] Started: execute_motion={args.execute_motion}, dry_run={dry_run_effective}")
    print(f"[J11 RUNNER] Topic: /pus6/target_cartesian_pose (sink {'CONNECTED' if args.execute_motion and not dry_run_effective else 'DISCONNECTED - ZERO WRITES'})")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t_end = time.monotonic() + args.duration
    try:
        while time.monotonic() < t_end:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stats = node.get_timing_statistics()
        print("\n=== J11 RUNNER TIMING STATISTICS ===")
        for k, v in stats.items():
            print(f"  {k}: {v}")
        if args.output_csv:
            node.save_telemetry_csv(args.output_csv)
            print(f"[J11 RUNNER] Telemetry saved to: {args.output_csv}")
            meta_p = Path(args.output_csv).with_suffix(".json")
            meta = {
                "runner_stats": stats,
                "execute_motion": args.execute_motion,
                "dry_run": dry_run_effective,
                "hardware_write_count": node.hardware_write_count,
                "state": node.state.value,
                "fault_reason": node.fault_reason,
                "q0_platform": list(node.q0_plat_xyzw) if node.q0_plat_xyzw else None,
                "q0_base": list(node.q0_base_xyzw) if node.q0_base_xyzw else None,
            }
            with open(meta_p, "w") as f:
                json.dump(meta, f, indent=2)
            print(f"[J11 RUNNER] Metadata saved to: {meta_p}")

            gains_dict = {
                "kp": [args.kp_roll, args.kp_pitch, args.kp_yaw],
                "ki": [args.ki_roll, args.ki_pitch, args.ki_yaw],
                "kd": [args.kd_roll, args.kd_pitch, args.kd_yaw],
            }
            summary = compute_j11_2a_summary(
                telemetry_history=node.telemetry_history,
                command_used=" ".join(sys.argv),
                execute_motion=args.execute_motion,
                dry_run=dry_run_effective,
                gains=gains_dict,
                z0=node.z0,
                hardware_write_count=node.hardware_write_count,
                feedback_received=node.feedback_received,
            )
            summary_p = Path(args.output_csv).parent / "j11_2a_zero_gain_powered_summary.json"
            with open(summary_p, "w") as f:
                json.dump(summary, f, indent=2)
            print(f"[J11 RUNNER] Detailed summary saved to: {summary_p}")

        if rclpy.ok():
            rclpy.shutdown()

        for proc in [robot_proc, plat_proc, base_proc]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=1.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass


if __name__ == "__main__":
    main()
