#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J13-2A1: Dual-IMU LADRC Shadow-Mode ROS 2 Runner.

Architecture:
  - 100% Retained Classical P/PD Controller retains physical control authority over the robot.
  - LADRC (First-Order Effective Plant + Exact Discrete Second-Order LESO) executes in parallel
    in STRICT SHADOW MODE (LOG-ONLY, ZERO CONTROL AUTHORITY).
  - Observer uses the actual physical plant input (u_plant_actual_{k-1} from Classical P/PD),
    never its own hypothetical shadow output.
  - Real attitude composition receives ONLY u_classical_actual.
  - LADRC output u_ladrc_shadow has NO path to kinematics, target poses, or motor drivers.
  - Safety: default dry_run = True, execute_motion = False, hardware_write_count = 0.
"""

from __future__ import annotations

import argparse
import csv
import enum
import json
import math
from pathlib import Path
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int32MultiArray
    from rcl_interfaces.msg import Log
except ImportError:
    rclpy = None
    Node = object
    Log = object

import numpy as np

# Path resolution for local dependencies
_SCRIPT_DIR = Path(__file__).resolve().parent
_KINEMATICS_DIR = _SCRIPT_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPT_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

# Kinematics imports
try:
    from analytical_6pus import Analytical6PusKinematics
    from geometry_config import load_geometry_config
except ImportError:
    Analytical6PusKinematics = None
    load_geometry_config = None

# Math and Classical PID imports from qualified J11 baseline
from j11_m2_pid_residual_runner import (
    M2TaskSpaceAttitudePid,
    M2PidGains,
    M2PidLimits,
    compute_so3_platform_error,
    compose_so3_pid_residual,
    normalize_quat,
    relative_quat,
    apply_z90_transform,
    quat_to_rpy_deg,
    quat_to_rot_matrix,
    rot_matrix_to_quat_xyzw,
    rot_matrix_to_euler_deg,
    rot_matrix_angle_deg,
    rot_matrix_tilt_deg,
)

# LADRC Core
from j13_ladrc import FirstOrderLadrcChannel, ThreeAxisLadrc


class J13ShadowState(enum.Enum):
    DISARMED = "DISARMED"
    WAITING_FOR_PREFLIGHT = "WAITING_FOR_PREFLIGHT"
    WAITING_FOR_TRIGGER = "WAITING_FOR_TRIGGER"
    CAPTURING_REFERENCE = "CAPTURING_REFERENCE"
    READY = "READY"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclass
class J13LadrcTelemetry:
    """Authoritative Unified Telemetry Schema for Stage J13-2A1 Shadow Mode."""
    timestamp_ns: int = 0
    cycle_index: int = 0
    dt_ms: float = 20.0
    state: str = J13ShadowState.DISARMED.value
    controller_mode: str = "classical_pid_with_ladrc_shadow"

    # Base IMU
    base_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    base_rel_roll_deg: float = 0.0
    base_rel_pitch_deg: float = 0.0
    base_rel_yaw_deg: float = 0.0
    base_so3_magnitude_deg: float = 0.0
    base_imu_age_ms: float = 0.0
    R_WB: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())

    # Platform IMU
    platform_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    platform_rel_roll_deg: float = 0.0
    platform_rel_pitch_deg: float = 0.0
    platform_rel_yaw_deg: float = 0.0
    platform_so3_error_deg: float = 0.0
    platform_tilt_error_deg: float = 0.0
    platform_imu_age_ms: float = 0.0
    R_WP: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())

    # Reference orientations
    R_WP0: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    q0_base_raw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    q0_plat_raw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    # Feedforward (Base Cancellation)
    R_BP_FF: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    ff_roll_deg: float = 0.0
    ff_pitch_deg: float = 0.0
    ff_yaw_deg: float = 0.0

    # Tracking Error e_R_W (World Frame, rad & deg)
    e_R_W_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    e_R_W_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # --------------------------------------------------------------------------
    # Classical P/PD Controller (Real Control Authority)
    # --------------------------------------------------------------------------
    classical_u_raw_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    classical_u_raw_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    classical_u_actual_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    classical_u_actual_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    classical_saturated_axes: List[bool] = field(default_factory=lambda: [False, False, False])

    # J11 backward compatibility fields
    pid_p_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_p_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_i_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_d_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_d_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    e_dot_R_W_raw_rad_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    e_dot_R_W_filt_rad_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_total_residual_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_total_residual_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_saturated_axes: List[bool] = field(default_factory=lambda: [False, False, False])

    # --------------------------------------------------------------------------
    # LADRC Shadow Controller (LOG ONLY, ZERO CONTROL AUTHORITY)
    # --------------------------------------------------------------------------
    ladrc_z1_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_z1_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_z2_rad_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_z2_deg_s: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_u_raw_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_u_raw_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_u_shadow_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_u_shadow_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ladrc_saturated_axes: List[bool] = field(default_factory=lambda: [False, False, False])

    # Observer actual input tracking
    observer_previous_plant_input_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    observer_previous_plant_input_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # Direct comparison metric
    delta_u_shadow_vs_classical_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    delta_u_shadow_vs_classical_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # Active parameters
    b0: List[float] = field(default_factory=lambda: [10.0, 10.0, 3.0])
    omega_c: List[float] = field(default_factory=lambda: [1.0, 1.0, 0.35])
    omega_o: List[float] = field(default_factory=lambda: [4.0, 4.0, 1.8])

    # Command: Delta_R_PID & R_BP_cmd (Governed 100% by Classical P/PD)
    Delta_R_PID_W: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    Delta_R_PID_B: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    R_BP_cmd: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    cmd_roll_deg: float = 0.0
    cmd_pitch_deg: float = 0.0
    cmd_yaw_deg: float = 0.0
    cmd_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    # Actuator Steps & Status
    predicted_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    commanded_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    feedback_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    command_feedback_errors: List[int] = field(default_factory=lambda: [0] * 6)
    joint_currents_raw_mA: List[int] = field(default_factory=lambda: [0] * 6)
    faults: List[int] = field(default_factory=lambda: [0] * 6)

    # Runtime Safety & Diagnostics
    driver_feedback_age_ms: float = 0.0
    warning_hold: bool = False
    hard_stop: bool = False
    hardware_write_issued: bool = False
    fault_reason: str = ""
    ladrc_fault: bool = False
    ladrc_fault_reason: str = ""


class J13LadrcShadowRunner(Node if Node is not object else object):
    """ROS 2 Node executing Classical P/PD in real control and LADRC in Shadow Mode."""

    def __init__(
        self,
        execute_motion: bool = False,
        dry_run: bool = True,
        output_dir: str = "results/j13/shadow_mode",
        auto_start: bool = True,
        # Classical PID Parameters (Retained J11 baseline)
        kp_roll: float = 0.05,
        kp_pitch: float = 0.05,
        kp_yaw: float = 0.025,
        ki_roll: float = 0.0,
        ki_pitch: float = 0.0,
        ki_yaw: float = 0.0,
        kd_roll: float = 0.0,
        kd_pitch: float = 0.0015,
        kd_yaw: float = 0.0015,
        derivative_enabled: bool = True,
        derivative_lpf_cutoff_hz: float = 5.0,
        max_residual_roll_deg: float = 0.5,
        max_residual_pitch_deg: float = 0.5,
        max_residual_yaw_deg: float = 0.5,
        # LADRC Parameters (J13 Conservative Hardware Start)
        ladrc_b0_roll: float = 10.0,
        ladrc_b0_pitch: float = 10.0,
        ladrc_b0_yaw: float = 3.0,
        ladrc_omega_c_roll: float = 1.0,
        ladrc_omega_c_pitch: float = 1.0,
        ladrc_omega_c_yaw: float = 0.35,
        ladrc_omega_o_roll: float = 4.0,
        ladrc_omega_o_pitch: float = 4.0,
        ladrc_omega_o_yaw: float = 1.8,
        ladrc_max_residual_roll_deg: float = 0.5,
        ladrc_max_residual_pitch_deg: float = 0.5,
        ladrc_max_residual_yaw_deg: float = 0.5,
    ) -> None:
        if Node is not object:
            super().__init__("j13_ladrc_shadow_runner")

        self.execute_motion = execute_motion
        self.dry_run = dry_run
        self.output_dir = Path(output_dir)
        self.auto_start = auto_start
        self.hardware_write_count = 0

        # Kinematics & Config
        if load_geometry_config is not None and Analytical6PusKinematics is not None:
            self.geom_cfg = load_geometry_config()
            self.kin = Analytical6PusKinematics()
            self.z0 = self.kin.nominal_height
            self.operating_home_steps = list(self.geom_cfg.operating_home_raw_steps)
        else:
            self.geom_cfg = None
            self.kin = None
            self.z0 = 0.132612268
            self.operating_home_steps = [1000] * 6

        # State machine
        self.state = J13ShadowState.DISARMED
        self.fault_reason = ""
        self.cycle_index = 0
        self.active_cycle_count = 0
        self._manual_prompt_emitted = False

        # Classical Controller (REAL CONTROL)
        gains = M2PidGains(
            kp=np.array([kp_roll, kp_pitch, kp_yaw], dtype=np.float64),
            ki=np.array([ki_roll, ki_pitch, ki_yaw], dtype=np.float64),
            kd=np.array([kd_roll, kd_pitch, kd_yaw], dtype=np.float64),
        )
        limits = M2PidLimits(
            residual_limit_rad=np.array([
                math.radians(max_residual_roll_deg),
                math.radians(max_residual_pitch_deg),
                math.radians(max_residual_yaw_deg),
            ], dtype=np.float64),
            derivative_enabled=derivative_enabled,
            derivative_lpf_cutoff_hz=derivative_lpf_cutoff_hz,
        )
        self.pid = M2TaskSpaceAttitudePid(gains=gains, limits=limits)

        # LADRC Controller (SHADOW MODE - LOG ONLY)
        self.ladrc = ThreeAxisLadrc(
            b0_roll=ladrc_b0_roll,
            b0_pitch=ladrc_b0_pitch,
            b0_yaw=ladrc_b0_yaw,
            omega_c_roll=ladrc_omega_c_roll,
            omega_c_pitch=ladrc_omega_c_pitch,
            omega_c_yaw=ladrc_omega_c_yaw,
            omega_o_roll=ladrc_omega_o_roll,
            omega_o_pitch=ladrc_omega_o_pitch,
            omega_o_yaw=ladrc_omega_o_yaw,
            u_limit_roll_deg=ladrc_max_residual_roll_deg,
            u_limit_pitch_deg=ladrc_max_residual_pitch_deg,
            u_limit_yaw_deg=ladrc_max_residual_yaw_deg,
            nominal_dt=0.020,
            min_dt=0.001,
            max_dt=0.100,
        )

        # Historical actual control applied to the plant (from Classical P/PD)
        self.u_plant_actual: np.ndarray = np.zeros(3, dtype=np.float64)

        # Reference quaternions
        self.q0_base_raw: Optional[Tuple[float, float, float, float]] = None
        self.q0_plat_raw: Optional[Tuple[float, float, float, float]] = None
        self.ref_capture_samples_base: List[Tuple[float, float, float, float]] = []
        self.ref_capture_samples_plat: List[Tuple[float, float, float, float]] = []

        # Latest IMU messages
        self.latest_base_q_raw: Optional[Tuple[float, float, float, float]] = None
        self.latest_base_rx_time_ns: int = 0
        self.latest_plat_q_raw: Optional[Tuple[float, float, float, float]] = None
        self.latest_plat_rx_time_ns: int = 0

        # Motor telemetry buffers
        self.latest_cmd_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_rx_time_ns: int = 0
        self.latest_fault_codes: List[int] = [0] * 6
        self.latest_fb_currents: List[int] = [0] * 6
        self.has_received_real_feedback: bool = False
        self.driver_stop_latched: bool = False
        self.driver_fault_msg: str = ""

        # Telemetry history
        self.telemetry_history: List[J13LadrcTelemetry] = []
        self.node_init_time_sec = time.time()
        self.last_tick_time_ns: Optional[int] = None

        # Marker file cleanup
        for marker in [
            "/tmp/j13_active_signaled",
            "/tmp/j13_ready_for_start",
            "/tmp/j13_driver_stopped",
            "/tmp/j13_start_signal",
        ]:
            p = Path(marker)
            if p.exists():
                try:
                    p.unlink()
                except OSError:
                    pass

        # ROS 2 Interfaces
        self.target_pose_pub = None
        if Node is not object:
            qos_reliable = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            qos_sensor = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            qos_rosout = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.target_pose_pub = self.create_publisher(
                PoseStamped, "/pus6/target_cartesian_pose", qos_reliable
            )
            self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_sensor)
            self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_sensor)
            self.create_subscription(
                Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, qos_sensor
            )
            self.create_subscription(
                Int32MultiArray, "/pus6/joint_steps_raw", self._on_fb_steps, qos_sensor
            )
            self.create_subscription(
                Int32MultiArray, "/pus6/joint_faults_raw", self._on_fb_faults, qos_sensor
            )
            self.create_subscription(
                Int32MultiArray, "/pus6/joint_currents_raw", self._on_fb_currents, qos_sensor
            )
            self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)
            self.timer = self.create_timer(0.020, self._on_control_tick)

    def _on_base_imu(self, msg: Imu) -> None:
        self.latest_base_q_raw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.latest_base_rx_time_ns = time.perf_counter_ns()

    def _on_plat_imu(self, msg: Imu) -> None:
        self.latest_plat_q_raw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.latest_plat_rx_time_ns = time.perf_counter_ns()

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_cmd_steps = list(msg.data[:6])

    def _on_fb_steps(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fb_steps = list(msg.data[:6])
            self.latest_fb_rx_time_ns = time.perf_counter_ns()
            self.has_received_real_feedback = True

    def _on_fb_faults(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fault_codes = list(msg.data[:6])

    def _on_fb_currents(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fb_currents = list(msg.data[:6])

    def _on_rosout(self, msg: Log) -> None:
        if "pus6_motor_driver" in msg.name and msg.level >= 40:
            msg_time_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if 0 < msg_time_sec < self.node_init_time_sec:
                return
            text = msg.msg
            if any(
                k in text
                for k in [
                    "STOP latched",
                    "scheduler_deadline_exceeded",
                    "overcurrent",
                    "Timeout recovery target rejected",
                    "reconnect_max_jump_steps",
                ]
            ):
                self.driver_stop_latched = True
                self.driver_fault_msg = text
                self._enter_fault(f"Driver fatal latch detected via /rosout: {text}")

    def _enter_fault(self, reason: str) -> None:
        if self.state != J13ShadowState.FAULT:
            self.state = J13ShadowState.FAULT
            self.fault_reason = str(reason)
            self.pid.disable()
            self.ladrc.reset()
            print(f"\n[J13-SHADOW FAULT / STOP DETECTED] {self.fault_reason}\n", flush=True)
            for marker in ["/tmp/j13_active_signaled", "/tmp/j13_ready_for_start"]:
                try:
                    Path(marker).unlink(missing_ok=True)
                except OSError:
                    pass
            try:
                Path("/tmp/j13_driver_stopped").touch()
            except OSError:
                pass

    def _on_control_tick(self) -> None:
        """Executes one control cycle in strict causal tick order."""
        now_ns = time.perf_counter_ns()
        dt_s = (
            (now_ns - self.last_tick_time_ns) * 1e-9
            if self.last_tick_time_ns is not None
            else 0.020
        )
        self.last_tick_time_ns = now_ns
        self.cycle_index += 1

        telem = J13LadrcTelemetry(
            timestamp_ns=now_ns,
            cycle_index=self.cycle_index,
            dt_ms=dt_s * 1000.0,
            controller_mode="classical_pid_with_ladrc_shadow",
            b0=self.ladrc.b0.tolist(),
            omega_c=self.ladrc.omega_c.tolist(),
            omega_o=self.ladrc.omega_o.tolist(),
        )

        base_age_ms = (
            (now_ns - self.latest_base_rx_time_ns) * 1e-6
            if self.latest_base_rx_time_ns > 0
            else 1e9
        )
        plat_age_ms = (
            (now_ns - self.latest_plat_rx_time_ns) * 1e-6
            if self.latest_plat_rx_time_ns > 0
            else 1e9
        )
        fb_age_ms = (
            (now_ns - self.latest_fb_rx_time_ns) * 1e-6
            if self.latest_fb_rx_time_ns > 0
            else 1e9
        )

        telem.base_imu_age_ms = base_age_ms
        telem.platform_imu_age_ms = plat_age_ms
        telem.driver_feedback_age_ms = fb_age_ms

        telem.commanded_steps = list(self.latest_cmd_steps)
        telem.feedback_steps = list(self.latest_fb_steps)
        telem.command_feedback_errors = [
            c - f for c, f in zip(telem.commanded_steps, telem.feedback_steps)
        ]
        telem.faults = list(self.latest_fault_codes)
        telem.joint_currents_raw_mA = list(self.latest_fb_currents)

        base_q_raw = self.latest_base_q_raw
        plat_q_raw = self.latest_plat_q_raw

        # State Machine Transitions
        if self.state == J13ShadowState.DISARMED:
            self.state = J13ShadowState.WAITING_FOR_PREFLIGHT

        elif self.state == J13ShadowState.WAITING_FOR_PREFLIGHT:
            imu_ok = (
                base_q_raw is not None
                and plat_q_raw is not None
                and base_age_ms < 100.0
                and plat_age_ms < 100.0
            )
            if self.dry_run or not self.execute_motion:
                motors_ok = True
            else:
                motors_ok = self.has_received_real_feedback and fb_age_ms < 150.0
                if motors_ok:
                    errs = [abs(s - 1000) for s in self.latest_fb_steps]
                    if any(e > 15 for e in errs):
                        self._enter_fault(
                            f"Preflight FAIL: Motors not at OPERATING_HOME [1000]*6 "
                            f"(actual={self.latest_fb_steps}, max err={max(errs)} steps)"
                        )
                        return

            if imu_ok and motors_ok:
                print("\n" + "=" * 70)
                print(
                    "[J13-SHADOW] Preflight check PASSED: Dual IMU healthy (<100ms), "
                    "6 axes verified at OPERATING_HOME."
                )
                if self.auto_start:
                    print(
                        "[J13-SHADOW] auto_start enabled. Initiating 1.0s stationary reference "
                        "capture at OPERATING_HOME..."
                    )
                    print("=" * 70 + "\n", flush=True)
                    self.ref_capture_samples_base = []
                    self.ref_capture_samples_plat = []
                    self.state = J13ShadowState.CAPTURING_REFERENCE
                else:
                    print(
                        "[J13-SHADOW] Waiting for start trigger (/tmp/j13_start_signal) to "
                        "capture reference and engage ACTIVE..."
                    )
                    print("=" * 70 + "\n", flush=True)
                    try:
                        Path("/tmp/j13_ready_for_start").touch()
                    except OSError:
                        pass
                    self.state = J13ShadowState.WAITING_FOR_TRIGGER

        elif self.state == J13ShadowState.WAITING_FOR_TRIGGER:
            if Path("/tmp/j13_start_signal").exists():
                try:
                    Path("/tmp/j13_start_signal").unlink()
                except OSError:
                    pass
                print(
                    "\n[J13-SHADOW] Start signal received! Initiating reference capture...",
                    flush=True,
                )
                self.ref_capture_samples_base = []
                self.ref_capture_samples_plat = []
                self.state = J13ShadowState.CAPTURING_REFERENCE

        elif self.state == J13ShadowState.CAPTURING_REFERENCE:
            if base_q_raw is None or plat_q_raw is None:
                return
            self.ref_capture_samples_base.append(base_q_raw)
            self.ref_capture_samples_plat.append(plat_q_raw)
            if len(self.ref_capture_samples_base) >= 50:
                self.q0_base_raw = self.ref_capture_samples_base[-1]
                self.q0_plat_raw = self.ref_capture_samples_plat[-1]
                self.state = J13ShadowState.READY

        elif self.state == J13ShadowState.READY:
            if not self.dry_run and self.execute_motion:
                if hasattr(self, "get_publishers_info_by_topic"):
                    pubs = self.get_publishers_info_by_topic("/pus6/target_cartesian_pose")
                    pub_nodes = [
                        f"{p.node_namespace}/{p.node_name}".replace("//", "/") for p in pubs
                    ]
                    my_node_name = self.get_fully_qualified_name()
                    competing = [n for n in pub_nodes if n != my_node_name]
                    if competing:
                        self._enter_fault(
                            f"Competing publisher on /pus6/target_cartesian_pose: {competing}"
                        )
                        return

            # Compute initial error at transition
            q_rel_plat_init = relative_quat(self.q0_plat_raw, plat_q_raw)
            q_plat_mech_init = apply_z90_transform(q_rel_plat_init)
            R_WP_init = quat_to_rot_matrix(q_plat_mech_init)
            _, e_R_W_init = compute_so3_platform_error(np.eye(3, dtype=np.float64), R_WP_init)

            # Controller initializations
            self.pid.reset()
            self.pid.enable()

            # Initialize LADRC with current measured tracking error and zero disturbance
            self.ladrc.reset(
                z1_init=e_R_W_init,
                z2_init=[0.0, 0.0, 0.0],
                previous_u_actual=self.u_plant_actual,
            )

            self.state = J13ShadowState.ACTIVE
            try:
                Path("/tmp/j13_active_signaled").touch()
            except OSError:
                pass
            print(
                "\n[J13-SHADOW] State transition: READY -> ACTIVE. "
                "Dual-IMU Feedforward + Classical PID engaged, LADRC Shadow observing.\n",
                flush=True,
            )

        elif self.state == J13ShadowState.ACTIVE:
            self.active_cycle_count += 1
            if base_age_ms > 100.0:
                self._enter_fault("Base IMU age timeout (>100ms)")
                return
            if plat_age_ms > 100.0:
                self._enter_fault("Platform IMU age timeout (>100ms)")
                return
            if self.driver_stop_latched:
                self._enter_fault(f"Driver STOP latched: {self.driver_fault_msg}")
                return
            if not self.dry_run and self.execute_motion:
                if fb_age_ms > 1500.0:
                    self._enter_fault(
                        f"Motor driver feedback stalled: fb_age={fb_age_ms:.1f}ms > 1500ms"
                    )
                    return
            if any(fc != 0 for fc in self.latest_fault_codes):
                self._enter_fault(
                    f"Motor driver reported hardware fault: {self.latest_fault_codes}"
                )
                return

        telem.state = self.state.value
        telem.fault_reason = self.fault_reason

        # Computation of Tracking Error, Control Loops & Shadow Evaluation
        if (
            self.q0_base_raw is not None
            and base_q_raw is not None
            and self.q0_plat_raw is not None
            and plat_q_raw is not None
            and self.state in (J13ShadowState.READY, J13ShadowState.ACTIVE)
        ):
            telem.q0_base_raw = self.q0_base_raw
            telem.q0_plat_raw = self.q0_plat_raw
            telem.R_WP0 = np.eye(3).tolist()

            # 1. Base IMU -> R_WB
            q_rel_base = relative_quat(self.q0_base_raw, base_q_raw)
            q_base_mech = apply_z90_transform(q_rel_base)
            r_b, p_b, y_b = quat_to_rpy_deg(q_base_mech)
            R_WB = quat_to_rot_matrix(q_base_mech)

            telem.base_rel_roll_deg = r_b
            telem.base_rel_pitch_deg = p_b
            telem.base_rel_yaw_deg = y_b
            telem.base_rel_quat_xyzw = q_base_mech
            telem.base_so3_magnitude_deg = rot_matrix_angle_deg(R_WB)
            telem.R_WB = R_WB.tolist()

            # 2. Feedforward command: R_BP_FF = R_WB^T
            R_BP_FF = R_WB.T
            ff_r, ff_p, ff_y = rot_matrix_to_euler_deg(R_BP_FF)
            telem.R_BP_FF = R_BP_FF.tolist()
            telem.ff_roll_deg = ff_r
            telem.ff_pitch_deg = ff_p
            telem.ff_yaw_deg = ff_y

            # 3. Platform IMU -> R_WP
            q_rel_plat = relative_quat(self.q0_plat_raw, plat_q_raw)
            q_plat_mech = apply_z90_transform(q_rel_plat)
            r_p, p_p, y_p = quat_to_rpy_deg(q_plat_mech)
            R_WP = quat_to_rot_matrix(q_plat_mech)

            telem.platform_rel_roll_deg = r_p
            telem.platform_rel_pitch_deg = p_p
            telem.platform_rel_yaw_deg = y_p
            telem.platform_rel_quat_xyzw = q_plat_mech
            telem.platform_so3_error_deg = rot_matrix_angle_deg(R_WP)
            telem.platform_tilt_error_deg = rot_matrix_tilt_deg(R_WP)
            telem.R_WP = R_WP.tolist()

            # 4. SO(3) Platform World Error
            R_WP_des = np.eye(3, dtype=np.float64)
            _, e_R_W_rad = compute_so3_platform_error(R_WP_des, R_WP)
            telem.e_R_W_rad = e_R_W_rad.tolist()
            telem.e_R_W_deg = np.degrees(e_R_W_rad).tolist()

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 3 & 4: LADRC SHADOW UPDATE & EVALUATION
            # Observer takes previous applied plant input (self.u_plant_actual)
            # ------------------------------------------------------------------
            telem.observer_previous_plant_input_rad = self.u_plant_actual.tolist()
            telem.observer_previous_plant_input_deg = np.degrees(self.u_plant_actual).tolist()

            try:
                u_ladrc_shadow, u_ladrc_raw, z1, z2, ladrc_sat = self.ladrc.step(
                    e_R_W_rad=e_R_W_rad,
                    actual_previous_input_rad=self.u_plant_actual,
                    dt=dt_s,
                )
                telem.ladrc_z1_rad = z1.tolist()
                telem.ladrc_z1_deg = np.degrees(z1).tolist()
                telem.ladrc_z2_rad_s = z2.tolist()
                telem.ladrc_z2_deg_s = np.degrees(z2).tolist()
                telem.ladrc_u_raw_rad = u_ladrc_raw.tolist()
                telem.ladrc_u_raw_deg = np.degrees(u_ladrc_raw).tolist()
                telem.ladrc_u_shadow_rad = u_ladrc_shadow.tolist()
                telem.ladrc_u_shadow_deg = np.degrees(u_ladrc_shadow).tolist()
                telem.ladrc_saturated_axes = list(ladrc_sat)
            except Exception as e_ladrc:
                telem.ladrc_fault = True
                telem.ladrc_fault_reason = f"LADRC shadow computation error: {e_ladrc}"
                u_ladrc_shadow = np.zeros(3, dtype=np.float64)

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 5: CLASSICAL P/PD UPDATE (AUTHORITATIVE REAL CONTROL)
            # ------------------------------------------------------------------
            u_classical_actual = self.pid.update(e_R_W_rad, dt_s)
            u_classical_raw = (
                self.pid.last_p_term + self.pid.last_i_term + self.pid.last_d_term
            )

            telem.classical_u_raw_rad = u_classical_raw.tolist()
            telem.classical_u_raw_deg = np.degrees(u_classical_raw).tolist()
            telem.classical_u_actual_rad = u_classical_actual.tolist()
            telem.classical_u_actual_deg = np.degrees(u_classical_actual).tolist()
            telem.classical_saturated_axes = list(self.pid.saturated_axes)

            # J11 backward compatibility fields
            telem.pid_p_rad = self.pid.last_p_term.tolist()
            telem.pid_p_deg = np.degrees(self.pid.last_p_term).tolist()
            telem.pid_i_rad = self.pid.last_i_term.tolist()
            telem.pid_d_rad = self.pid.last_d_term.tolist()
            telem.pid_d_deg = np.degrees(self.pid.last_d_term).tolist()
            telem.e_dot_R_W_raw_rad_s = self.pid.last_raw_derivative.tolist()
            telem.e_dot_R_W_filt_rad_s = self.pid.last_filtered_derivative.tolist()
            telem.pid_total_residual_rad = u_classical_actual.tolist()
            telem.pid_total_residual_deg = np.degrees(u_classical_actual).tolist()
            telem.pid_saturated_axes = list(self.pid.saturated_axes)

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 6: DELTA COMPARISON (SHADOW vs CLASSICAL)
            # ------------------------------------------------------------------
            delta_u = u_ladrc_shadow - u_classical_actual
            telem.delta_u_shadow_vs_classical_rad = delta_u.tolist()
            telem.delta_u_shadow_vs_classical_deg = np.degrees(delta_u).tolist()

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 7: AUTHORITATIVE RESIDUAL COMPOSITION
            # STRICT REQUIREMENT: ONLY u_classical_actual ENTERS COMPOSE.
            # u_ladrc_shadow IS NEVER PASSED TO COMPOSE OR KINEMATICS.
            # ------------------------------------------------------------------
            Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw = compose_so3_pid_residual(
                R_WB, R_BP_FF, u_classical_actual
            )
            cmd_r, cmd_p, cmd_y = rot_matrix_to_euler_deg(R_BP_cmd)

            telem.Delta_R_PID_W = Delta_R_PID_W.tolist()
            telem.Delta_R_PID_B = Delta_R_PID_B.tolist()
            telem.R_BP_cmd = R_BP_cmd.tolist()
            telem.cmd_roll_deg = cmd_r
            telem.cmd_pitch_deg = cmd_p
            telem.cmd_yaw_deg = cmd_y
            telem.cmd_quat_xyzw = q_cmd_xyzw

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 8: UPDATE HISTORICAL PLANT INPUT FOR NEXT TICK
            # ------------------------------------------------------------------
            self.u_plant_actual = u_classical_actual.copy()

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 9: KINEMATICS & SAFETY EVALUATION
            # ------------------------------------------------------------------
            if self.kin is not None:
                cart_pose = [
                    0.0,
                    0.0,
                    self.z0,
                    math.radians(cmd_r),
                    math.radians(cmd_p),
                    math.radians(cmd_y),
                ]
                home_pose = [0.0, 0.0, self.z0, 0.0, 0.0, 0.0]
                try:
                    delta_steps = self.kin.differential_inverse_kinematics_steps(
                        cart_pose, home_pose
                    )
                    pred_steps = [
                        ref + ds for ref, ds in zip(self.operating_home_steps, delta_steps)
                    ]
                    telem.predicted_steps = pred_steps

                    if any(s < 0 or s > 2000 for s in pred_steps):
                        self._enter_fault(
                            f"Actuator predicted step out of [0, 2000] limit: {pred_steps}"
                        )
                        return
                except Exception as exc:
                    self._enter_fault(f"IK computation failed: {exc}")
                    return

            # ------------------------------------------------------------------
            # CAUSAL TICK STEP 10: HARDWARE DISPATCH SINK
            # ------------------------------------------------------------------
            if self.execute_motion and not self.dry_run and self.state == J13ShadowState.ACTIVE:
                self._dispatch_cartesian_target(telem)
                telem.hardware_write_issued = True
            else:
                telem.hardware_write_issued = False

            # Console diagnostic output
            if self.state == J13ShadowState.ACTIVE:
                if not self._manual_prompt_emitted and self.active_cycle_count >= 50:
                    self._manual_prompt_emitted = True
                    print("\n" + "=" * 70, flush=True)
                    print(
                        "[J13-SHADOW] Classical Control ACTIVE | LADRC Shadow Observing",
                        flush=True,
                    )
                    print("=" * 70 + "\n", flush=True)
                elif self._manual_prompt_emitted and self.active_cycle_count % 25 == 0:
                    print(
                        f"[J13 SHADOW t={self.active_cycle_count*0.02:.1f}s] "
                        f"Plat Err: (R:{telem.platform_rel_roll_deg:+.2f}°, "
                        f"P:{telem.platform_rel_pitch_deg:+.2f}°, Y:{telem.platform_rel_yaw_deg:+.2f}°) | "
                        f"Classical_u: (R:{telem.classical_u_actual_deg[0]:+.3f}°, "
                        f"P:{telem.classical_u_actual_deg[1]:+.3f}°, Y:{telem.classical_u_actual_deg[2]:+.3f}°) | "
                        f"LADRC_u: (R:{telem.ladrc_u_shadow_deg[0]:+.3f}°, "
                        f"P:{telem.ladrc_u_shadow_deg[1]:+.3f}°, Y:{telem.ladrc_u_shadow_deg[2]:+.3f}°) | "
                        f"Delta_u: (R:{telem.delta_u_shadow_vs_classical_deg[0]:+.3f}°, "
                        f"P:{telem.delta_u_shadow_vs_classical_deg[1]:+.3f}°, "
                        f"Y:{telem.delta_u_shadow_vs_classical_deg[2]:+.3f}°)",
                        flush=True,
                    )

        self.telemetry_history.append(telem)

    def _dispatch_cartesian_target(self, telem: J13LadrcTelemetry) -> None:
        """Publishes commanded Cartesian pose to pus6 kinematics node."""
        if self.target_pose_pub is None:
            return
        msg = PoseStamped()
        if hasattr(self, "get_clock"):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = self.z0
        msg.pose.orientation.x = float(telem.cmd_quat_xyzw[0])
        msg.pose.orientation.y = float(telem.cmd_quat_xyzw[1])
        msg.pose.orientation.z = float(telem.cmd_quat_xyzw[2])
        msg.pose.orientation.w = float(telem.cmd_quat_xyzw[3])

        self.target_pose_pub.publish(msg)
        self.hardware_write_count += 1


def run_j13_shadow_session(
    execute_motion: bool = False,
    dry_run: bool = True,
    duration_s: float = 10.0,
    output_dir: str = "results/j13/shadow_mode",
    stage: str = "j13_2a1_software_qualification",
    auto_start: bool = True,
    # Classical P/PD
    kp_roll: float = 0.05,
    kp_pitch: float = 0.05,
    kp_yaw: float = 0.025,
    ki_roll: float = 0.0,
    ki_pitch: float = 0.0,
    ki_yaw: float = 0.0,
    kd_roll: float = 0.0,
    kd_pitch: float = 0.0015,
    kd_yaw: float = 0.0015,
    derivative_enabled: bool = True,
    derivative_lpf_cutoff_hz: float = 5.0,
    max_residual_roll_deg: float = 0.5,
    max_residual_pitch_deg: float = 0.5,
    max_residual_yaw_deg: float = 0.5,
    # LADRC
    ladrc_b0_roll: float = 10.0,
    ladrc_b0_pitch: float = 10.0,
    ladrc_b0_yaw: float = 3.0,
    ladrc_omega_c_roll: float = 1.0,
    ladrc_omega_c_pitch: float = 1.0,
    ladrc_omega_c_yaw: float = 0.35,
    ladrc_omega_o_roll: float = 4.0,
    ladrc_omega_o_pitch: float = 4.0,
    ladrc_omega_o_yaw: float = 1.8,
    ladrc_max_residual_roll_deg: float = 0.5,
    ladrc_max_residual_pitch_deg: float = 0.5,
    ladrc_max_residual_yaw_deg: float = 0.5,
) -> dict:
    mode_str = (
        "POWERED REAL EXECUTION"
        if (execute_motion and not dry_run)
        else "DRY-RUN (hardware_write_count = 0)"
    )
    print(f"\n[{stage.upper()}] Starting J13 LADRC Shadow Runner in {mode_str}...")

    if rclpy is not None:
        rclpy.init()
    runner = J13LadrcShadowRunner(
        execute_motion=execute_motion,
        dry_run=dry_run,
        output_dir=output_dir,
        auto_start=auto_start,
        kp_roll=kp_roll,
        kp_pitch=kp_pitch,
        kp_yaw=kp_yaw,
        ki_roll=ki_roll,
        ki_pitch=ki_pitch,
        ki_yaw=ki_yaw,
        kd_roll=kd_roll,
        kd_pitch=kd_pitch,
        kd_yaw=kd_yaw,
        derivative_enabled=derivative_enabled,
        derivative_lpf_cutoff_hz=derivative_lpf_cutoff_hz,
        max_residual_roll_deg=max_residual_roll_deg,
        max_residual_pitch_deg=max_residual_pitch_deg,
        max_residual_yaw_deg=max_residual_yaw_deg,
        ladrc_b0_roll=ladrc_b0_roll,
        ladrc_b0_pitch=ladrc_b0_pitch,
        ladrc_b0_yaw=ladrc_b0_yaw,
        ladrc_omega_c_roll=ladrc_omega_c_roll,
        ladrc_omega_c_pitch=ladrc_omega_c_pitch,
        ladrc_omega_c_yaw=ladrc_omega_c_yaw,
        ladrc_omega_o_roll=ladrc_omega_o_roll,
        ladrc_omega_o_pitch=ladrc_omega_o_pitch,
        ladrc_omega_o_yaw=ladrc_omega_o_yaw,
        ladrc_max_residual_roll_deg=ladrc_max_residual_roll_deg,
        ladrc_max_residual_pitch_deg=ladrc_max_residual_pitch_deg,
        ladrc_max_residual_yaw_deg=ladrc_max_residual_yaw_deg,
    )

    t_start = time.monotonic()
    spin_thread = None
    if rclpy is not None:
        spin_thread = threading.Thread(target=rclpy.spin, args=(runner,), daemon=True)
        spin_thread.start()

    stop_file = Path("/tmp/j13_stop_requested")
    if stop_file.exists():
        try:
            stop_file.unlink()
        except OSError:
            pass

    try:
        while time.monotonic() - t_start < duration_s:
            if stop_file.exists():
                print(f"\n[{stage.upper()}] Stop requested via /tmp/j13_stop_requested.")
                break
            if runner.state == J13ShadowState.FAULT:
                print(f"\n[{stage.upper()}] Terminating early due to FAULT: {runner.fault_reason}")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n[{stage.upper()}] Interrupted by user.")
    finally:
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        for marker in ["/tmp/j13_active_signaled", "/tmp/j13_ready_for_start"]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass

    out_p = Path(output_dir)
    out_p.mkdir(parents=True, exist_ok=True)
    csv_file = out_p / (
        f"{stage}_telemetry_powered.csv"
        if (execute_motion and not dry_run)
        else f"{stage}_dry_run_telemetry.csv"
    )
    summary_file = out_p / (
        f"{stage}_summary_powered.json"
        if (execute_motion and not dry_run)
        else f"{stage}_dry_run_summary.json"
    )

    # Save Unified Telemetry CSV
    if runner.telemetry_history:
        keys = list(asdict(runner.telemetry_history[0]).keys())
        with open(csv_file, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for rec in runner.telemetry_history:
                d = asdict(rec)
                for k, v in d.items():
                    if isinstance(v, (list, tuple)):
                        d[k] = json.dumps(v)
                w.writerow(d)
        print(
            f"[{stage.upper()}] Unified Shadow Telemetry CSV saved to: {csv_file} "
            f"({len(runner.telemetry_history)} rows)"
        )

    summary = {
        "stage": stage,
        "mode": mode_str,
        "hardware_write_count": runner.hardware_write_count,
        "total_cycles": len(runner.telemetry_history),
        "final_state": runner.state.value,
        "fault_reason": runner.fault_reason,
    }
    with open(summary_file, "w") as f:
        json.dump(summary, f, indent=2)

    return summary


def main():
    parser = argparse.ArgumentParser(
        description="J13-2A1: Dual-IMU LADRC Shadow-Mode ROS 2 Runner"
    )
    parser.add_argument("--execute-motion", action="store_true", default=False)
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--no-dry-run", action="store_false", dest="dry_run")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output-dir", default="results/j13/shadow_mode")
    parser.add_argument("--stage", default="j13_2a1_software_qualification")
    parser.add_argument("--auto-start", action="store_true", default=True)
    parser.add_argument("--no-auto-start", action="store_false", dest="auto_start")

    # Classical PID arguments
    parser.add_argument("--kp-roll", type=float, default=0.05)
    parser.add_argument("--kp-pitch", type=float, default=0.05)
    parser.add_argument("--kp-yaw", type=float, default=0.025)
    parser.add_argument("--ki-roll", type=float, default=0.0)
    parser.add_argument("--ki-pitch", type=float, default=0.0)
    parser.add_argument("--ki-yaw", type=float, default=0.0)
    parser.add_argument("--kd-roll", type=float, default=0.0)
    parser.add_argument("--kd-pitch", type=float, default=0.0015)
    parser.add_argument("--kd-yaw", type=float, default=0.0015)
    parser.add_argument("--enable-derivative", action="store_true", default=True)
    parser.add_argument("--disable-derivative", action="store_false", dest="enable_derivative")
    parser.add_argument("--derivative-lpf-cutoff-hz", type=float, default=5.0)
    parser.add_argument("--max-residual-roll-deg", type=float, default=0.5)
    parser.add_argument("--max-residual-pitch-deg", type=float, default=0.5)
    parser.add_argument("--max-residual-yaw-deg", type=float, default=0.5)

    # LADRC Shadow arguments
    parser.add_argument("--ladrc-b0-roll", type=float, default=10.0)
    parser.add_argument("--ladrc-b0-pitch", type=float, default=10.0)
    parser.add_argument("--ladrc-b0-yaw", type=float, default=3.0)
    parser.add_argument("--ladrc-omega-c-roll", type=float, default=1.0)
    parser.add_argument("--ladrc-omega-c-pitch", type=float, default=1.0)
    parser.add_argument("--ladrc-omega-c-yaw", type=float, default=0.35)
    parser.add_argument("--ladrc-omega-o-roll", type=float, default=4.0)
    parser.add_argument("--ladrc-omega-o-pitch", type=float, default=4.0)
    parser.add_argument("--ladrc-omega-o-yaw", type=float, default=1.8)
    parser.add_argument("--ladrc-max-residual-roll-deg", type=float, default=0.5)
    parser.add_argument("--ladrc-max-residual-pitch-deg", type=float, default=0.5)
    parser.add_argument("--ladrc-max-residual-yaw-deg", type=float, default=0.5)

    args = parser.parse_args()

    run_j13_shadow_session(
        execute_motion=args.execute_motion,
        dry_run=args.dry_run,
        duration_s=args.duration,
        output_dir=args.output_dir,
        stage=args.stage,
        auto_start=args.auto_start,
        kp_roll=args.kp_roll,
        kp_pitch=args.kp_pitch,
        kp_yaw=args.kp_yaw,
        ki_roll=args.ki_roll,
        ki_pitch=args.ki_pitch,
        ki_yaw=args.ki_yaw,
        kd_roll=args.kd_roll,
        kd_pitch=args.kd_pitch,
        kd_yaw=args.kd_yaw,
        derivative_enabled=args.enable_derivative,
        derivative_lpf_cutoff_hz=args.derivative_lpf_cutoff_hz,
        max_residual_roll_deg=args.max_residual_roll_deg,
        max_residual_pitch_deg=args.max_residual_pitch_deg,
        max_residual_yaw_deg=args.max_residual_yaw_deg,
        ladrc_b0_roll=args.ladrc_b0_roll,
        ladrc_b0_pitch=args.ladrc_b0_pitch,
        ladrc_b0_yaw=args.ladrc_b0_yaw,
        ladrc_omega_c_roll=args.ladrc_omega_c_roll,
        ladrc_omega_c_pitch=args.ladrc_omega_c_pitch,
        ladrc_omega_c_yaw=args.ladrc_omega_c_yaw,
        ladrc_omega_o_roll=args.ladrc_omega_o_roll,
        ladrc_omega_o_pitch=args.ladrc_omega_o_pitch,
        ladrc_omega_o_yaw=args.ladrc_omega_o_yaw,
        ladrc_max_residual_roll_deg=args.ladrc_max_residual_roll_deg,
        ladrc_max_residual_pitch_deg=args.ladrc_max_residual_pitch_deg,
        ladrc_max_residual_yaw_deg=args.ladrc_max_residual_yaw_deg,
    )


if __name__ == "__main__":
    main()
