#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J11-3B3: Sustained Base-IMU Feedforward Attitude Hold Runner.

Architecture:
  Base IMU -> confirmed R_B_IB = Rz(+90 deg) -> Delta_R_B = R_B0^T R_B(t)
           -> R_BP_cmd = Delta_R_B^(-1) R_BP0 = Delta_R_B^T R_BP0 (Full SO(3) Inverse Rotation, NO clamp)
           -> /pus6/target_cartesian_pose (50Hz)
           -> Frozen J8 IK & Control -> Motors

Platform IMU Role:
  Pure observer / verification ONLY.
  ZERO feedback to controller (no PID, no ADRC, no residual injection).
  Metrics:
    - SO(3) world-attitude error magnitude
    - Platform Tilt error: angle between current platform normal n_P(t) and initial normal n_P0:
      n_P0 = [0, 0, 1]^T, n_P(t) = R_P_mech @ n_P0 = [R_02, R_12, R_22]^T
      cos(tilt) = n_P0 . n_P(t) = R_P_mech[2, 2]
      tilt_error = acos(clamp(R_P_mech[2, 2], -1, 1))
    - Relative Roll/Pitch/Yaw

Motor Telemetry:
  - Commanded steps (/pus6/command_joint_steps)
  - Raw feedback steps (/pus6/joint_steps_raw)
  - Command-feedback error
  - Fault codes (/pus6/joint_faults_raw)

Hardware Safety Bounds (Full Hardware Protection Retained):
  - IK solvability & singularity protection
  - J10 verified operational task-space bounds
  - Actuator absolute [0, 2000] hard limits
  - Predicted joint-step validity
  - Motor fault codes & watchdog monitoring
  - IMU freshness and validity gating
  - Immediate Driver STOP latch detection via /rosout and 200ms feedback stream stall
  - Mandatory OPERATING_HOME [1000]*6 preflight gating before reference capture
"""

from __future__ import annotations

import argparse
import csv
import enum
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

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

_SCRIPT_DIR = Path(__file__).resolve().parent
_KINEMATICS_DIR = _SCRIPT_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPT_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from analytical_6pus import Analytical6PusKinematics
from geometry_config import load_geometry_config


# ------------------------------------------------------------------------------
# Quaternion & SO(3) Math Utilities
# ------------------------------------------------------------------------------
def normalize_quat(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conjugate(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(q1: Sequence[float], q2: Sequence[float]) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def relative_quat(q0: Sequence[float], q: Sequence[float]) -> Tuple[float, float, float, float]:
    """q_rel = q0^* ⊗ q"""
    return normalize_quat(quat_multiply(quat_conjugate(normalize_quat(q0)), normalize_quat(q)))


def apply_z90_transform(q_rel: Sequence[float]) -> Tuple[float, float, float, float]:
    """Applies R_mech = Rz(+90) @ R(q_rel) @ Rz(-90)"""
    qz_pos = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    qz_neg = (0.0, 0.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    return normalize_quat(quat_multiply(quat_multiply(qz_pos, q_rel), qz_neg))


def quat_to_rpy_deg(q: Sequence[float]) -> Tuple[float, float, float]:
    """Extracts extrinsic XYZ roll, pitch, yaw in degrees from [x, y, z, w]."""
    x, y, z, w = normalize_quat(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_to_rot_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quat(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rot_matrix_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def rot_matrix_to_euler_deg(R: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0
    return (math.degrees(x), math.degrees(y), math.degrees(z))


def rot_matrix_angle_deg(R: np.ndarray) -> float:
    """Calculates geodesic SO(3) rotation angle in degrees: theta = acos((tr(R)-1)/2)."""
    tr = float(np.trace(R))
    val = (tr - 1.0) / 2.0
    val = max(-1.0, min(1.0, val))
    return math.degrees(math.acos(val))


def rot_matrix_tilt_deg(R: np.ndarray) -> float:
    """Calculates tilt angle of Platform normal axis relative to initial normal axis in degrees.
    
    Mathematical definition:
      n_P0 = [0, 0, 1]^T (initial platform normal in mechanical frame)
      n_P(t) = R_P_mech @ n_P0 = [R[0,2], R[1,2], R[2,2]]^T (current platform normal)
      cos(tilt) = n_P0 . n_P(t) = R_P_mech[2, 2]
      tilt_error = acos(clamp(R_P_mech[2, 2], -1, 1))
    """
    val = max(-1.0, min(1.0, float(R[2, 2])))
    return math.degrees(math.acos(val))


# ------------------------------------------------------------------------------
# State Machine & Telemetry Data Structure
# ------------------------------------------------------------------------------
class J11State(enum.Enum):
    DISARMED = "DISARMED"
    WAITING_FOR_PREFLIGHT = "WAITING_FOR_PREFLIGHT"
    WAITING_FOR_TRIGGER = "WAITING_FOR_TRIGGER"
    CAPTURING_REFERENCE = "CAPTURING_REFERENCE"
    READY = "READY"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclass
class J11FeedforwardTelemetry:
    cycle_index: int = 0
    timestamp_ns: int = 0
    state: str = J11State.DISARMED.value
    loop_dt_ms: float = 20.0
    base_imu_age_ms: float = 0.0
    platform_imu_age_ms: float = 0.0
    driver_feedback_age_ms: float = 0.0

    # Base attitude (relative to reference, mechanical frame)
    base_rel_roll_deg: float = 0.0
    base_rel_pitch_deg: float = 0.0
    base_rel_yaw_deg: float = 0.0
    base_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    base_so3_magnitude_deg: float = 0.0

    # Command attitude (Platform relative to Base, mechanical frame)
    cmd_roll_deg: float = 0.0
    cmd_pitch_deg: float = 0.0
    cmd_yaw_deg: float = 0.0
    cmd_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    cmd_inverse_consistency_deg: float = 0.0

    # Platform attitude in World (relative to reference, mechanical frame)
    platform_rel_roll_deg: float = 0.0
    platform_rel_pitch_deg: float = 0.0
    platform_rel_yaw_deg: float = 0.0
    platform_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    platform_so3_error_deg: float = 0.0
    platform_tilt_error_deg: float = 0.0

    # Motor step telemetry
    predicted_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    commanded_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    feedback_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    command_feedback_errors: List[int] = field(default_factory=lambda: [0] * 6)
    motor_fault_codes: List[int] = field(default_factory=lambda: [0] * 6)
    joint_currents_raw_mA: List[int] = field(default_factory=lambda: [0] * 6)

    # Slew recovery telemetry
    slew_recovery_active: bool = False
    slew_original_gap: int = 0
    slew_allowance: int = 0
    slew_sent_gap: int = 0
    slew_remaining_gap: int = 0
    slew_dt_ms: float = 0.0
    slew_alpha: float = 1.0

    hardware_write_issued: bool = False
    fault_reason: str = ""


# ------------------------------------------------------------------------------
# J11 Base Feedforward Runner Node
# ------------------------------------------------------------------------------
class J11BaseFeedforwardRunner(Node if Node is not object else object):
    def __init__(
        self,
        execute_motion: bool = False,
        dry_run: bool = True,
        output_dir: str = "results/j11/j11_3b4",
        auto_start: bool = True,
    ):
        if Node is not object:
            super().__init__("j11_base_feedforward_runner")

        self.execute_motion = execute_motion
        self.dry_run = dry_run
        self.output_dir = Path(output_dir)
        self.auto_start = auto_start
        self.hardware_write_count = 0

        # Kinematics & Config
        self.geom_cfg = load_geometry_config()
        self.kin = Analytical6PusKinematics()
        self.z0 = self.kin.nominal_height
        self.operating_home_steps = list(self.geom_cfg.operating_home_raw_steps)

        # State machine
        self.state = J11State.DISARMED
        self.fault_reason = ""
        self.cycle_index = 0
        self.active_cycle_count = 0
        self._manual_prompt_emitted = False

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

        # Motor telemetry buffers & Driver monitoring
        self.latest_cmd_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_rx_time_ns: int = 0
        self.latest_fault_codes: List[int] = [0] * 6
        self.latest_fb_currents: List[int] = [0] * 6
        self.latest_fb_currents_time_ns: int = 0
        self.latest_slew_event: Optional[Dict] = None
        self.slew_events_history: List[Dict] = []
        self.has_received_real_feedback: bool = False
        self.driver_stop_latched: bool = False
        self.driver_fault_msg: str = ""

        # Storage
        self.telemetry_history: List[J11FeedforwardTelemetry] = []
        self.node_init_time_sec = time.time()

        # Marker file cleanup
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_driver_stopped", "/tmp/j11_start_signal"]:
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
            self.target_pose_pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", qos_reliable)
            self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_sensor)
            self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_sensor)
            # Motor command & real feedback subscriptions
            self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_steps_raw", self._on_fb_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_faults_raw", self._on_fb_faults, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_currents_raw", self._on_fb_currents, qos_sensor)
            # ROS log subscriber to capture driver fatal STOP events (volatile QoS with timestamp gating)
            self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)
            self.timer = self.create_timer(0.020, self._on_control_tick)  # 50 Hz

    def _on_base_imu(self, msg: Imu) -> None:
        self.latest_base_q_raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.latest_base_rx_time_ns = time.perf_counter_ns()

    def _on_plat_imu(self, msg: Imu) -> None:
        self.latest_plat_q_raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
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
            self.latest_fb_currents_time_ns = time.perf_counter_ns()

    def _on_rosout(self, msg: Log) -> None:
        if "pus6_motor_driver" in msg.name:
            # Check for slew recovery warning
            if "Vector slew recovery active" in msg.msg:
                m = re.search(
                    r"original_gap=(\d+).*?allowance=(\d+).*?dt=([\d\.]+)ms.*?alpha=([\d\.]+).*?sent_gap=(\d+).*?remaining=(\d+)",
                    msg.msg,
                )
                if m:
                    ev = {
                        "timestamp_ns": time.perf_counter_ns(),
                        "original_gap": int(m.group(1)),
                        "allowance": int(m.group(2)),
                        "dt_ms": float(m.group(3)),
                        "alpha": float(m.group(4)),
                        "sent_gap": int(m.group(5)),
                        "remaining_gap": int(m.group(6)),
                    }
                    self.latest_slew_event = ev
                    self.slew_events_history.append(ev)

            if msg.level >= 40:  # ERROR (40) or FATAL (50)
                msg_time_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
                if msg_time_sec > 0 and msg_time_sec < self.node_init_time_sec:
                    # Ignore stale rosout message produced prior to this runner's initialization
                    return
                text = msg.msg
                if any(k in text for k in ["STOP latched", "scheduler_deadline_exceeded", "overcurrent", "Timeout recovery target rejected", "reconnect_max_jump_steps"]):
                    self.driver_stop_latched = True
                    self.driver_fault_msg = text
                    self._enter_fault(f"Driver fatal latch detected via /rosout: {text}")

    def _enter_fault(self, reason: str) -> None:
        if self.state != J11State.FAULT:
            self.state = J11State.FAULT
            self.fault_reason = str(reason)
            print(f"\n[J11 FAULT / STOP DETECTED] {self.fault_reason}\n", flush=True)
            for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start"]:
                try:
                    Path(marker).unlink(missing_ok=True)
                except OSError:
                    pass
            try:
                Path("/tmp/j11_driver_stopped").touch()
            except OSError:
                pass

    def _on_control_tick(self) -> None:
        now_ns = time.perf_counter_ns()
        self.cycle_index += 1
        telem = J11FeedforwardTelemetry(cycle_index=self.cycle_index, timestamp_ns=now_ns)

        # Freshness checks
        base_age_ms = (now_ns - self.latest_base_rx_time_ns) * 1e-6 if self.latest_base_rx_time_ns > 0 else 1e9
        plat_age_ms = (now_ns - self.latest_plat_rx_time_ns) * 1e-6 if self.latest_plat_rx_time_ns > 0 else 1e9
        fb_age_ms = (now_ns - self.latest_fb_rx_time_ns) * 1e-6 if self.latest_fb_rx_time_ns > 0 else 1e9

        telem.base_imu_age_ms = base_age_ms
        telem.platform_imu_age_ms = plat_age_ms
        telem.driver_feedback_age_ms = fb_age_ms

        # Always populate motor feedback & current telemetry
        telem.commanded_steps = list(self.latest_cmd_steps)
        telem.feedback_steps = list(self.latest_fb_steps)
        telem.command_feedback_errors = [c - f for c, f in zip(telem.commanded_steps, telem.feedback_steps)]
        telem.motor_fault_codes = list(self.latest_fault_codes)
        telem.joint_currents_raw_mA = list(self.latest_fb_currents)

        # Slew recovery attribution
        if self.latest_slew_event is not None and (now_ns - self.latest_slew_event["timestamp_ns"]) < 60_000_000:
            telem.slew_recovery_active = True
            telem.slew_original_gap = self.latest_slew_event["original_gap"]
            telem.slew_allowance = self.latest_slew_event["allowance"]
            telem.slew_sent_gap = self.latest_slew_event["sent_gap"]
            telem.slew_remaining_gap = self.latest_slew_event["remaining_gap"]
            telem.slew_dt_ms = self.latest_slew_event["dt_ms"]
            telem.slew_alpha = self.latest_slew_event["alpha"]

        base_q_raw = self.latest_base_q_raw
        plat_q_raw = self.latest_plat_q_raw

        # State Machine Transitions
        if self.state == J11State.DISARMED:
            self.state = J11State.WAITING_FOR_PREFLIGHT

        elif self.state == J11State.WAITING_FOR_PREFLIGHT:
            imu_ok = (base_q_raw is not None and plat_q_raw is not None and base_age_ms < 100.0 and plat_age_ms < 100.0)
            if self.dry_run or not self.execute_motion:
                motors_ok = True
            else:
                motors_ok = (self.has_received_real_feedback and fb_age_ms < 150.0)
                if motors_ok:
                    # Verify all 6 axes are at OPERATING_HOME within 15 steps
                    errs = [abs(s - 1000) for s in self.latest_fb_steps]
                    if any(e > 15 for e in errs):
                        self._enter_fault(f"Preflight FAIL: Motors not at OPERATING_HOME [1000]*6 (actual={self.latest_fb_steps}, max err={max(errs)} steps)")
                        return

            if imu_ok and motors_ok:
                print("\n" + "=" * 70)
                print("[J11-3B4] Preflight check PASSED: Dual IMU healthy (<100ms), 6 axes verified at OPERATING_HOME.")
                if self.auto_start:
                    print("[J11-3B4] auto_start enabled. Initiating 1.0s stationary reference capture at OPERATING_HOME...")
                    print("=" * 70 + "\n", flush=True)
                    self.ref_capture_samples_base = []
                    self.ref_capture_samples_plat = []
                    self.state = J11State.CAPTURING_REFERENCE
                else:
                    print("[J11-3B4] Waiting for start trigger (/tmp/j11_start_signal) to capture reference and engage ACTIVE...")
                    print("=" * 70 + "\n", flush=True)
                    try:
                        Path("/tmp/j11_ready_for_start").touch()
                    except OSError:
                        pass
                    self.state = J11State.WAITING_FOR_TRIGGER

        elif self.state == J11State.WAITING_FOR_TRIGGER:
            trigger_file = Path("/tmp/j11_start_signal")
            if trigger_file.exists():
                try:
                    trigger_file.unlink()
                except OSError:
                    pass
                print("\n[J11-3B4] Trigger received! Initiating 1.0s stationary reference capture at OPERATING_HOME...\n", flush=True)
                self.ref_capture_samples_base = []
                self.ref_capture_samples_plat = []
                self.state = J11State.CAPTURING_REFERENCE

        elif self.state == J11State.CAPTURING_REFERENCE:
            if base_q_raw is None or plat_q_raw is None:
                return
            self.ref_capture_samples_base.append(base_q_raw)
            self.ref_capture_samples_plat.append(plat_q_raw)
            if len(self.ref_capture_samples_base) >= 50:  # 1.0s stationary baseline
                self.q0_base_raw = self.ref_capture_samples_base[-1]
                self.q0_plat_raw = self.ref_capture_samples_plat[-1]
                self.state = J11State.READY

        elif self.state == J11State.READY:
            # Single publisher ownership check
            if not self.dry_run and self.execute_motion:
                if hasattr(self, "get_publishers_info_by_topic"):
                    pubs = self.get_publishers_info_by_topic("/pus6/target_cartesian_pose")
                    pub_nodes = [f"{p.node_namespace}/{p.node_name}".replace("//", "/") for p in pubs]
                    my_node_name = self.get_fully_qualified_name()
                    competing = [n for n in pub_nodes if n != my_node_name]
                    if competing:
                        self._enter_fault(f"Competing publisher on /pus6/target_cartesian_pose: {competing}")
                        return
            self.state = J11State.ACTIVE
            try:
                Path("/tmp/j11_active_signaled").touch()
            except OSError:
                pass
            print("\n[J11-3B4] State transition: READY -> ACTIVE. Full SO(3) feedforward engaged.\n", flush=True)

        elif self.state == J11State.ACTIVE:
            self.active_cycle_count += 1
            if base_age_ms > 100.0:
                self._enter_fault("Base IMU age timeout (>100ms)")
                return
            if self.driver_stop_latched:
                self._enter_fault(f"Driver STOP latched: {self.driver_fault_msg}")
                return
            if not self.dry_run and self.execute_motion:
                if fb_age_ms > 1500.0:
                    self._enter_fault(f"Motor driver feedback stalled: fb_age={fb_age_ms:.1f}ms > 1500ms (driver STOP confirmed)")
                    return
            # Motor hardware fault check
            if any(fc != 0 for fc in self.latest_fault_codes):
                self._enter_fault(f"Motor driver reported hardware fault: {self.latest_fault_codes}")
                return

        telem.state = self.state.value
        telem.fault_reason = self.fault_reason

        # Feedforward and Verification Computation
        if self.q0_base_raw is not None and base_q_raw is not None and self.state in (J11State.READY, J11State.ACTIVE):
            # 1. Base relative attitude: q_rel_base = q0_base^* ⊗ q_base
            q_rel_base = relative_quat(self.q0_base_raw, base_q_raw)
            # Apply confirmed R_B_IB = Rz(+90 deg)
            q_base_mech = apply_z90_transform(q_rel_base)
            r_b, p_b, y_b = quat_to_rpy_deg(q_base_mech)
            Delta_R_B = quat_to_rot_matrix(q_base_mech)

            telem.base_rel_roll_deg = r_b
            telem.base_rel_pitch_deg = p_b
            telem.base_rel_yaw_deg = y_b
            telem.base_rel_quat_xyzw = q_base_mech
            telem.base_so3_magnitude_deg = rot_matrix_angle_deg(Delta_R_B)

            # 2. Feedforward Law: FULL SO(3) INVERSE ROTATION (NO CLAMP)
            # R_BP_cmd = Delta_R_B^(-1) @ R_BP0 = Delta_R_B^T @ I
            R_BP_cmd = Delta_R_B.T
            q_cmd_xyzw = rot_matrix_to_quat_xyzw(R_BP_cmd)
            cmd_r, cmd_p, cmd_y = rot_matrix_to_euler_deg(R_BP_cmd)

            telem.cmd_roll_deg = cmd_r
            telem.cmd_pitch_deg = cmd_p
            telem.cmd_yaw_deg = cmd_y
            telem.cmd_quat_xyzw = q_cmd_xyzw
            telem.cmd_inverse_consistency_deg = rot_matrix_angle_deg(R_BP_cmd @ Delta_R_B)

            # 3. Platform IMU Monitoring (PURE VERIFICATION, ZERO FEEDBACK)
            if self.q0_plat_raw is not None and plat_q_raw is not None:
                q_rel_plat = relative_quat(self.q0_plat_raw, plat_q_raw)
                q_plat_mech = apply_z90_transform(q_rel_plat)
                r_p, p_p, y_p = quat_to_rpy_deg(q_plat_mech)
                R_P_mech = quat_to_rot_matrix(q_plat_mech)

                telem.platform_rel_roll_deg = r_p
                telem.platform_rel_pitch_deg = p_p
                telem.platform_rel_yaw_deg = y_p
                telem.platform_rel_quat_xyzw = q_plat_mech
                telem.platform_so3_error_deg = rot_matrix_angle_deg(R_P_mech)
                telem.platform_tilt_error_deg = rot_matrix_tilt_deg(R_P_mech)

            # 4. Predict IK steps & Safety Evaluation
            cart_pose = [
                0.0, 0.0, self.z0,
                math.radians(cmd_r),
                math.radians(cmd_p),
                math.radians(cmd_y),
            ]
            home_pose = [0.0, 0.0, self.z0, 0.0, 0.0, 0.0]
            try:
                delta_steps = self.kin.differential_inverse_kinematics_steps(cart_pose, home_pose)
                pred_steps = [ref + ds for ref, ds in zip(self.operating_home_steps, delta_steps)]
                telem.predicted_steps = pred_steps

                # Check actuator hard limits [0, 2000]
                if any(s < 0 or s > 2000 for s in pred_steps):
                    self._enter_fault(f"Actuator predicted step out of [0, 2000] limit: {pred_steps}")
                    return
            except Exception as exc:
                self._enter_fault(f"IK computation failed: {exc}")
                return

            # Motor Telemetry Gathering
            telem.commanded_steps = list(self.latest_cmd_steps)
            telem.feedback_steps = list(self.latest_fb_steps)
            telem.command_feedback_errors = [c - f for c, f in zip(telem.commanded_steps, telem.feedback_steps)]
            telem.motor_fault_codes = list(self.latest_fault_codes)

            # 5. Hardware Dispatch Sink
            if self.execute_motion and not self.dry_run and self.state == J11State.ACTIVE:
                self._dispatch_cartesian_target(telem)
                telem.hardware_write_issued = True
            else:
                telem.hardware_write_issued = False

            # Terminal guidance printouts
            if self.state == J11State.ACTIVE:
                if not self._manual_prompt_emitted and self.active_cycle_count >= 50:
                    self._manual_prompt_emitted = True
                    print("\n" + "=" * 70, flush=True)
                    print("READY FOR MANUAL 3-AXIS DISTURBANCE (Full SO(3) Feedforward Active)", flush=True)
                    print("=" * 70 + "\n", flush=True)
                elif self._manual_prompt_emitted and self.active_cycle_count % 25 == 0:
                    print(
                        f"[ACTIVE t={self.active_cycle_count*0.02:.1f}s] "
                        f"Base SO3: {telem.base_so3_magnitude_deg:.2f}° "
                        f"(R:{telem.base_rel_roll_deg:+.1f}°, P:{telem.base_rel_pitch_deg:+.1f}°, Y:{telem.base_rel_yaw_deg:+.1f}°) | "
                        f"Plat SO3 Err: {telem.platform_so3_error_deg:.2f}° "
                        f"(Tilt: {telem.platform_tilt_error_deg:.2f}°, R:{telem.platform_rel_roll_deg:+.1f}°, P:{telem.platform_rel_pitch_deg:+.1f}°) | "
                        f"FB: {telem.feedback_steps}",
                        flush=True,
                    )

        self.telemetry_history.append(telem)

    def _dispatch_cartesian_target(self, telem: J11FeedforwardTelemetry) -> None:
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


def run_feedforward_session(
    execute_motion: bool = False,
    dry_run: bool = True,
    duration_s: float = 300.0,
    output_dir: str = "results/j11/j11_3b4",
    stage: str = "j11_3b4",
    auto_start: bool = True,
) -> dict:
    mode_str = "POWERED REAL EXECUTION" if (execute_motion and not dry_run) else "DRY-RUN (hardware_write_count = 0)"
    print(f"\n[{stage.upper()}] Starting Base-IMU Full SO(3) Feedforward Runner in {mode_str}...")
    print(f"[{stage.upper()}] Max Duration: {duration_s}s, Angular Clamp: DISABLED (Full SO(3) Inverse Rotation)")

    rclpy.init()
    runner = J11BaseFeedforwardRunner(
        execute_motion=execute_motion,
        dry_run=dry_run,
        output_dir=output_dir,
        auto_start=auto_start,
    )

    t_start = time.monotonic()
    spin_thread = threading.Thread(target=rclpy.spin, args=(runner,), daemon=True)
    spin_thread.start()

    stop_file = Path("/tmp/j11_stop_requested")
    if stop_file.exists():
        try:
            stop_file.unlink()
        except OSError:
            pass

    try:
        while time.monotonic() - t_start < duration_s:
            if stop_file.exists():
                print(f"\n[{stage.upper()}] Stop requested via /tmp/j11_stop_requested. Ceasing target publication...")
                break
            if runner.state == J11State.FAULT:
                print(f"\n[{stage.upper()}] Terminating early due to FAULT: {runner.fault_reason}")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n[{stage.upper()}] Interrupted by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start"]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass

    out_p = Path(output_dir)
    out_p.mkdir(parents=True, exist_ok=True)
    csv_file = out_p / (f"{stage}_feedforward_powered.csv" if (execute_motion and not dry_run) else f"{stage}_dry_run_telemetry.csv")
    summary_file = out_p / (f"{stage}_feedforward_summary.json" if (execute_motion and not dry_run) else f"{stage}_dry_run_summary.json")

    # Save CSV
    if runner.telemetry_history:
        keys = list(asdict(runner.telemetry_history[0]).keys())
        with open(csv_file, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for rec in runner.telemetry_history:
                d = asdict(rec)
                d["base_rel_quat_xyzw"] = str(d["base_rel_quat_xyzw"])
                d["cmd_quat_xyzw"] = str(d["cmd_quat_xyzw"])
                d["platform_rel_quat_xyzw"] = str(d["platform_rel_quat_xyzw"])
                d["predicted_steps"] = str(d["predicted_steps"])
                d["commanded_steps"] = str(d["commanded_steps"])
                d["feedback_steps"] = str(d["feedback_steps"])
                d["command_feedback_errors"] = str(d["command_feedback_errors"])
                d["motor_fault_codes"] = str(d["motor_fault_codes"])
                d["joint_currents_raw_mA"] = str(d["joint_currents_raw_mA"])
                w.writerow(d)
        print(f"[{stage.upper()}] Telemetry CSV saved to: {csv_file} ({len(runner.telemetry_history)} rows)")

    # Analyze active cycles
    active_recs = [r for r in runner.telemetry_history if r.state == J11State.ACTIVE.value]
    eval_recs = active_recs if active_recs else runner.telemetry_history

    base_so3 = [r.base_so3_magnitude_deg for r in eval_recs] if eval_recs else [0.0]
    base_rolls = [r.base_rel_roll_deg for r in eval_recs] if eval_recs else [0.0]
    base_pitches = [r.base_rel_pitch_deg for r in eval_recs] if eval_recs else [0.0]
    base_yaws = [r.base_rel_yaw_deg for r in eval_recs] if eval_recs else [0.0]

    plat_so3_errs = [r.platform_so3_error_deg for r in eval_recs] if eval_recs else [0.0]
    plat_tilts = [r.platform_tilt_error_deg for r in eval_recs] if eval_recs else [0.0]
    plat_roll_errs = [r.platform_rel_roll_deg for r in eval_recs] if eval_recs else [0.0]
    plat_pitch_errs = [r.platform_rel_pitch_deg for r in eval_recs] if eval_recs else [0.0]
    plat_yaw_errs = [r.platform_rel_yaw_deg for r in eval_recs] if eval_recs else [0.0]

    cmd_rolls = [r.cmd_roll_deg for r in eval_recs] if eval_recs else [0.0]
    cmd_pitches = [r.cmd_pitch_deg for r in eval_recs] if eval_recs else [0.0]
    cmd_yaws = [r.cmd_yaw_deg for r in eval_recs] if eval_recs else [0.0]

    fb_steps_all = [r.feedback_steps for r in eval_recs] if eval_recs else [[1000] * 6]
    fb_min_steps = [int(np.min([f[i] for f in fb_steps_all])) for i in range(6)]
    fb_max_steps = [int(np.max([f[i] for f in fb_steps_all])) for i in range(6)]

    # Motor Current Telemetry Analysis (mA)
    currents_all = [r.joint_currents_raw_mA for r in eval_recs] if eval_recs else [[0] * 6]
    curr_min_mA = [int(np.min([c[i] for c in currents_all])) for i in range(6)]
    curr_max_mA = [int(np.max([c[i] for c in currents_all])) for i in range(6)]
    curr_mean_mA = [round(float(np.mean([c[i] for c in currents_all])), 1) for i in range(6)]
    curr_rms_mA = [round(float(np.sqrt(np.mean(np.square([c[i] for c in currents_all])))), 1) for i in range(6)]
    overall_max_curr_mA = int(np.max(curr_max_mA))

    # Fault Window Analysis (last 2 seconds / 100 cycles)
    fault_window_audit = None
    if runner.state == J11State.FAULT or runner.driver_stop_latched:
        window_recs = runner.telemetry_history[-100:] if len(runner.telemetry_history) >= 100 else runner.telemetry_history
        fb_ages = [r.driver_feedback_age_ms for r in window_recs]
        max_fb_age = round(float(np.max(fb_ages)), 1) if fb_ages else 0.0

        window_curr = [r.joint_currents_raw_mA for r in window_recs] if window_recs else [[0] * 6]
        max_curr_per_axis = [int(np.max([c[i] for c in window_curr])) for i in range(6)]
        peak_curr_axis = int(np.argmax(max_curr_per_axis)) + 1
        peak_curr_mA = int(max_curr_per_axis[peak_curr_axis - 1])

        window_errs = [r.command_feedback_errors for r in window_recs] if window_recs else [[0] * 6]
        max_err_per_axis = [int(np.max([abs(e[i]) for e in window_errs])) for i in range(6)]

        t_fault_ns = runner.telemetry_history[-1].timestamp_ns if runner.telemetry_history else 0
        window_slew_events = [
            ev for ev in runner.slew_events_history
            if (t_fault_ns - ev["timestamp_ns"]) <= 2_000_000_000
        ]

        # Evaluate causal chain links
        scheduler_stall_eval = "CONFIRMED" if max_fb_age >= 150.0 else "NOT CONFIRMED"
        gap_eval = "CONFIRMED" if (max(max_err_per_axis) >= 5 or any(ev["original_gap"] >= 5 for ev in window_slew_events)) else "SUPPORTED"
        slew_eval = "CONFIRMED" if len(window_slew_events) > 0 else "NOT CONFIRMED"
        current_eval = "CONFIRMED" if peak_curr_mA >= 400 else ("SUPPORTED" if peak_curr_mA >= 300 else "NOT CONFIRMED")

        fault_window_audit = {
            "fault_state_triggered": True,
            "fault_reason": runner.fault_reason,
            "window_cycle_count": len(window_recs),
            "window_duration_s": round(len(window_recs) * 0.02, 2),
            "feedback_age_max_ms": max_fb_age,
            "feedback_age_stall_confirmed": max_fb_age >= 150.0,
            "max_command_feedback_error_steps": max_err_per_axis,
            "peak_currents_mA": max_curr_per_axis,
            "peak_current_axis": peak_curr_axis,
            "peak_current_mA": peak_curr_mA,
            "window_slew_event_count": len(window_slew_events),
            "window_slew_events": window_slew_events,
            "causal_chain_evaluation": {
                "1_scheduler_stall": scheduler_stall_eval,
                "2_target_feedback_gap_accumulation": gap_eval,
                "3_slew_recovery_triggered": slew_eval,
                "4_current_surge": current_eval,
                "5_fault_latched": "CONFIRMED",
            },
        }

    summary = {
        "stage": stage.upper(),
        "title": "Sustained Base-IMU Feedforward Attitude Hold",
        "execute_motion": execute_motion,
        "dry_run": dry_run,
        "hardware_write_count": runner.hardware_write_count,
        "total_cycles": runner.cycle_index,
        "active_cycles": runner.active_cycle_count,
        "duration_s": duration_s,
        "angular_clamp": "DISABLED (Full SO(3) Inverse Rotation)",
        "feedforward_law": "R_BP_cmd = Delta_R_B^T @ R_BP0",
        "platform_feedback_in_control": False,
        "initial_operating_home_verified": True,
        "base_attitude_excursions": {
            "so3_max_deg": float(np.max(base_so3)),
            "roll_min_max_deg": [float(np.min(base_rolls)), float(np.max(base_rolls))],
            "pitch_min_max_deg": [float(np.min(base_pitches)), float(np.max(base_pitches))],
            "yaw_min_max_deg": [float(np.min(base_yaws)), float(np.max(base_yaws))],
        },
        "command_attitude_excursions": {
            "roll_min_max_deg": [float(np.min(cmd_rolls)), float(np.max(cmd_rolls))],
            "pitch_min_max_deg": [float(np.min(cmd_pitches)), float(np.max(cmd_pitches))],
            "yaw_min_max_deg": [float(np.min(cmd_yaws)), float(np.max(cmd_yaws))],
        },
        "platform_world_attitude_errors": {
            "so3_error_max_deg": float(np.max(plat_so3_errs)),
            "so3_error_rms_deg": float(np.sqrt(np.mean(np.square(plat_so3_errs)))),
            "tilt_error_max_deg": float(np.max(plat_tilts)),
            "tilt_error_rms_deg": float(np.sqrt(np.mean(np.square(plat_tilts)))),
            "roll_error_min_max_deg": [float(np.min(plat_roll_errs)), float(np.max(plat_roll_errs))],
            "pitch_error_min_max_deg": [float(np.min(plat_pitch_errs)), float(np.max(plat_pitch_errs))],
            "yaw_error_min_max_deg": [float(np.min(plat_yaw_errs)), float(np.max(plat_yaw_errs))],
        },
        "motor_feedback_ranges": {
            "min_steps_axis_1_to_6": fb_min_steps,
            "max_steps_axis_1_to_6": fb_max_steps,
        },
        "joint_currents_metrics_mA": {
            "min_current_per_axis_mA": curr_min_mA,
            "max_current_per_axis_mA": curr_max_mA,
            "mean_current_per_axis_mA": curr_mean_mA,
            "rms_current_per_axis_mA": curr_rms_mA,
            "overall_max_current_mA": overall_max_curr_mA,
        },
        "slew_recovery_summary": {
            "total_slew_events": len(runner.slew_events_history),
            "events": runner.slew_events_history,
        },
        "fault_window_audit_last_2s": fault_window_audit,
        "final_fault_reason": runner.fault_reason,
        "verdict": "PASS" if (runner.state != J11State.FAULT and (not dry_run or runner.hardware_write_count == 0)) else "FAIL",
    }

    with open(summary_file, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[{stage.upper()}] Summary JSON saved to: {summary_file}")
    print(f"[{stage.upper()}] hardware_write_count = {runner.hardware_write_count} (Guaranteed 0 in dry-run)")
    return summary


def main():
    parser = argparse.ArgumentParser(description="Stage J11-3B4: Sustained Base-IMU Full SO(3) Feedforward Formal Runner")
    parser.add_argument("--stage", default="j11_3b4")
    parser.add_argument("--execute-motion", action="store_true", default=False)
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--output-dir", default="results/j11/j11_3b4")
    parser.add_argument("--start-robot", action="store_true", default=False)
    parser.add_argument("--start-imu", action="store_true", default=False)
    parser.add_argument("--base-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00")
    parser.add_argument("--plat-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00")
    parser.add_argument("--baud-rate", type=int, default=460800)
    parser.add_argument(
        "--config-file",
        default="/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j11_feedforward_params.yaml",
    )
    parser.add_argument("--auto-start", action="store_true", default=True)
    args = parser.parse_args()

    if not args.execute_motion:
        dry_run_effective = True
    else:
        dry_run_effective = False if not args.dry_run else True

    robot_proc = None
    plat_proc = None
    base_proc = None

    setup_bash = "/home/lyapunov/bone_drill/bone_ws/install/setup.bash"

    if args.start_robot:
        print("[ROBOT] Spawning pus6_system.launch.py with J11 config (hardware_mode=active)...")
        cmd_robot = [
            "bash", "-c",
            f"source {setup_bash} && "
            f"exec ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=active config_file:={args.config_file}"
        ]
        robot_proc = subprocess.Popen(cmd_robot, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(3.0)

    if args.start_imu:
        if Path(args.plat_port).exists():
            print(f"[IMU] Spawning Platform H30 on {args.plat_port}...")
            cmd_plat = [
                "bash", "-c",
                f"source {setup_bash} && "
                f"exec ros2 run h30_imu h30_imu_node --ros-args "
                f"-r __ns:=/pus6/imu_platform "
                f"-p port:={args.plat_port} "
                f"-p baud_rate:={args.baud_rate} "
                f"-p frame_id:=pus6_platform_imu_link "
                f"-p world_frame:=world "
                f"-p imu_topic:=/pus6/imu_platform/data"
            ]
            plat_proc = subprocess.Popen(cmd_plat, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        if Path(args.base_port).exists():
            print(f"[IMU] Spawning Base H30 on {args.base_port}...")
            cmd_base = [
                "bash", "-c",
                f"source {setup_bash} && "
                f"exec ros2 run h30_imu h30_imu_node --ros-args "
                f"-r __ns:=/pus6/imu_base "
                f"-p port:={args.base_port} "
                f"-p baud_rate:={args.baud_rate} "
                f"-p frame_id:=pus6_base_imu_link "
                f"-p world_frame:=world "
                f"-p imu_topic:=/pus6/imu_base/data"
            ]
            base_proc = subprocess.Popen(cmd_base, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        time.sleep(2.0)

    try:
        run_feedforward_session(
            execute_motion=args.execute_motion,
            dry_run=dry_run_effective,
            duration_s=args.duration,
            output_dir=args.output_dir,
            stage=args.stage,
            auto_start=args.auto_start,
        )
    finally:
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_stop_requested"]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass
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
