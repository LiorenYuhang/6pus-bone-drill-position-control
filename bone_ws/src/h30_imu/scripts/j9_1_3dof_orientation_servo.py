#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J9.1-3: Unified 3DOF Orientation Servo Runner.

Architecture:
  3DOF Reference [roll_ref, pitch_ref, yaw_ref]
        ↓
  Platform H30 IMU Orientation [roll_meas, pitch_meas, yaw_meas] (R_PI = Rz(+90 deg))
        ↓
  Active Axes Filtering & P-Only Channel Residual Computation:
    e = ref - meas
    delta = clamp(Kp * e, -limit, +limit)  (if axis active, else 0.0)
        ↓
  SO(3) Orientation Composition:
    q_ref = euler_to_quat(ref)
    q_delta = euler_to_quat(delta)
    q_cmd = normalize(q_ref ⊗ q_delta)
        ↓
  /pus6/target_cartesian_pose (50Hz PoseStamped with q_cmd)
        ↓
  [J8 frozen black box: Pus6KinematicsNode -> Pus6ControlNode -> Pus6MotorDriverNode]
        ↓
  Real 6PUS Manipulator

Baseline: v1.0-j8-active-servo-hardware-qualified (UNTAMPERED BLACK BOX).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Sequence, Set, Tuple

# Path resolution for workspace packages
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"]:
    p = WS_SRC / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

scripts_dir = WS_SRC / "h30_imu" / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_msgs.msg import TimedJointFeedback

from j9_1_platform_imu_recorder import (
    average_quat_xyzw,
    normalize_quat_xyzw,
    quat_conjugate_xyzw,
    quat_multiply_xyzw,
    quat_to_rpy_deg_xyzw,
    relative_quat_xyzw,
)

CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "ros_stamp_sec",
    "ros_stamp_nanosec",
    "stage",
    "active_axes",
    "roll_ref_deg",
    "pitch_ref_deg",
    "yaw_ref_deg",
    "roll_meas_deg",
    "pitch_meas_deg",
    "yaw_meas_deg",
    "roll_error_deg",
    "pitch_error_deg",
    "yaw_error_deg",
    "roll_residual_deg",
    "pitch_residual_deg",
    "yaw_residual_deg",
    "roll_cartesian_cmd_deg",
    "pitch_cartesian_cmd_deg",
    "yaw_cartesian_cmd_deg",
    "cmd_qx",
    "cmd_qy",
    "cmd_qz",
    "cmd_qw",
    "qx",
    "qy",
    "qz",
    "qw",
    "raw_rel_roll_deg",
    "raw_rel_pitch_deg",
    "raw_rel_yaw_deg",
    "plat_roll_deg",
    "plat_pitch_deg",
    "plat_yaw_deg",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
    "joint_cmd_1",
    "joint_cmd_2",
    "joint_cmd_3",
    "joint_cmd_4",
    "joint_cmd_5",
    "joint_cmd_6",
    "joint_cmd_stamp_ns",
    "joint_fb_1",
    "joint_fb_2",
    "joint_fb_3",
    "joint_fb_4",
    "joint_fb_5",
    "joint_fb_6",
    "joint_fb_stamp_ns",
    "hardware_write_count",
    "fault_codes",
]


def check_motor_read_only_preflight(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    duration_s: float = 0.2,
) -> Tuple[bool, str]:
    """Performs a non-intrusive 0x30 status scan on all 6 motors before enabling driver."""
    from run_j8_c_hardware_readonly_audit import run_j8_c_audit

    try:
        res = run_j8_c_audit(port=port, baudrate=baudrate, duration_s=duration_s)
        if res.get("status") == "PASS":
            return True, "All 6 motors online, fault=0, within HOME band"
        return False, f"Audit returned status: {res.get('status')}"
    except Exception as exc:
        return False, f"Preflight audit exception: {exc}"


def euler_deg_to_quat_xyzw(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    """Converts extrinsic ZYX Euler angles in degrees to unit quaternion (qx, qy, qz, qw)."""
    cr = math.cos(math.radians(roll_deg) * 0.5)
    sr = math.sin(math.radians(roll_deg) * 0.5)
    cp = math.cos(math.radians(pitch_deg) * 0.5)
    sp = math.sin(math.radians(pitch_deg) * 0.5)
    cy = math.cos(math.radians(yaw_deg) * 0.5)
    sy = math.sin(math.radians(yaw_deg) * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        return (qx / norm, qy / norm, qz / norm, qw / norm)
    return (0.0, 0.0, 0.0, 1.0)


def compose_so3_orientation(
    ref_rpy_deg: Tuple[float, float, float],
    residual_rpy_deg: Tuple[float, float, float],
) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float]]:
    """
    Composes nominal reference orientation with small platform-frame residual:
      q_cmd = normalize(q_ref ⊗ q_residual)
    Returns:
      (q_cmd_xyzw, cmd_rpy_deg)
    """
    q_ref = euler_deg_to_quat_xyzw(*ref_rpy_deg)
    q_res = euler_deg_to_quat_xyzw(*residual_rpy_deg)
    q_composed = quat_multiply_xyzw(q_ref, q_res)
    q_norm = normalize_quat_xyzw(q_composed)
    assert q_norm is not None
    cmd_rpy = quat_to_rpy_deg_xyzw(q_norm)
    return q_norm, cmd_rpy


class J913DofOrientationServo(Node):
    """
    Unified 3DOF Orientation Servo controller and telemetry harness.
    """

    def __init__(
        self,
        *,
        csv_path: Path,
        imu_topic: str = "/pus6/imu_platform/data",
        active_axes: Sequence[str] = ("roll",),
        kp_roll: float = 0.5,
        kp_pitch: float = 0.5,
        kp_yaw: float = 0.5,
        residual_limit_roll_deg: float = 0.25,
        residual_limit_pitch_deg: float = 0.25,
        residual_limit_yaw_deg: float = 0.50,
        init_quiet_s: float = 2.0,
        min_init_samples: int = 50,
    ) -> None:
        super().__init__("j9_1_3dof_orientation_servo")

        self.csv_path = csv_path
        self.imu_topic = imu_topic
        self.active_axes: Set[str] = set(a.lower() for a in active_axes)

        self.kp_roll = float(kp_roll)
        self.kp_pitch = float(kp_pitch)
        self.kp_yaw = float(kp_yaw)

        self.residual_limit_roll = float(residual_limit_roll_deg)
        self.residual_limit_pitch = float(residual_limit_pitch_deg)
        self.residual_limit_yaw = float(residual_limit_yaw_deg)

        self.init_quiet_s = float(init_quiet_s)
        self.min_init_samples = int(min_init_samples)

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self.lock = threading.Lock()

        # Closed-loop 3DOF state variables
        self.current_stage: str = "pre_init"
        self.ref_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.meas_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.error_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.residual_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.cmd_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.cmd_q: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

        # Most recent sensor & actuator readings
        self.latest_plat_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.latest_raw_rel_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.latest_q: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self.latest_gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self.latest_cmd_steps: List[int] = [1000] * 6
        self.latest_cmd_stamp_ns: int = 0

        self.latest_fb_steps: List[int] = [1000] * 6
        self.latest_fb_currents: List[int] = [0] * 6
        self.latest_fb_faults: List[int] = [0] * 6
        self.latest_fb_stamp_ns: int = 0
        self.hardware_write_count: int = 0

        # Baseline zero setup
        self._first_imu_host_ns: int | None = None
        self._init_quats: List[Tuple[float, float, float, float]] = []
        self.q0_xyzw: Tuple[float, float, float, float] | None = None

        self.recorded_sample_count: int = 0
        self.total_imu_samples: int = 0
        self.samples_by_stage: Dict[str, List[Dict[str, Any]]] = {}

        # Canonical Rz(+90 deg) quaternion: [0, 0, sin(pi/4), cos(pi/4)]
        self.q_PI = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))

        # ROS 2 QoS setups
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publisher for Cartesian Target
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", reliable_qos
        )

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self.cmd_steps_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, sensor_qos
        )
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_feedback, sensor_qos
        )

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def set_stage(self, stage: str, ref_rpy_deg: Tuple[float, float, float]) -> None:
        with self.lock:
            self.current_stage = stage
            self.ref_rpy = tuple(float(x) for x in ref_rpy_deg)
            if stage not in self.samples_by_stage:
                self.samples_by_stage[stage] = []

    def compute_control_step(
        self,
    ) -> Tuple[
        Tuple[float, float, float],
        Tuple[float, float, float],
        Tuple[float, float, float],
        Tuple[float, float, float, float],
    ]:
        """
        Computes 3DOF P-only residuals on active axes and performs SO(3) composition.
        Returns:
          (errors_rpy, residuals_rpy, cmd_rpy, cmd_quat_xyzw)
        """
        with self.lock:
            m_r, m_p, m_y = self.latest_plat_rpy
            r_ref, p_ref, y_ref = self.ref_rpy

            # 1. Compute per-channel tracking error
            e_r = r_ref - m_r
            e_p = p_ref - m_p
            e_y = y_ref - m_y

            # 2. Compute per-channel residuals (active vs disabled)
            res_r = 0.0
            if "roll" in self.active_axes:
                raw_r = self.kp_roll * e_r
                res_r = max(-self.residual_limit_roll, min(self.residual_limit_roll, raw_r))

            res_p = 0.0
            if "pitch" in self.active_axes:
                raw_p = self.kp_pitch * e_p
                res_p = max(-self.residual_limit_pitch, min(self.residual_limit_pitch, raw_p))

            res_y = 0.0
            if "yaw" in self.active_axes:
                raw_y = self.kp_yaw * e_y
                res_y = max(-self.residual_limit_yaw, min(self.residual_limit_yaw, raw_y))

            # 3. SO(3) Orientation Composition
            q_cmd, cmd_rpy = compose_so3_orientation(
                (r_ref, p_ref, y_ref), (res_r, res_p, res_y)
            )

            self.meas_rpy = (m_r, m_p, m_y)
            self.error_rpy = (e_r, e_p, e_y)
            self.residual_rpy = (res_r, res_p, res_y)
            self.cmd_rpy = cmd_rpy
            self.cmd_q = q_cmd

            return self.error_rpy, self.residual_rpy, self.cmd_rpy, self.cmd_q

    def publish_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        q_xyzw: Tuple[float, float, float, float],
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = float(q_xyzw[0])
        msg.pose.orientation.y = float(q_xyzw[1])
        msg.pose.orientation.z = float(q_xyzw[2])
        msg.pose.orientation.w = float(q_xyzw[3])

        self.target_pose_pub.publish(msg)

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            if len(msg.data) >= 6:
                self.latest_cmd_steps = list(msg.data[:6])
                self.latest_cmd_stamp_ns = t_ns

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            if len(msg.raw_steps) >= 6:
                self.latest_fb_steps = [int(s) for s in msg.raw_steps[:6]]
                self.latest_fb_currents = [int(c) for c in msg.currents_ma[:6]]
                self.latest_fb_faults = [int(f) for f in msg.fault_codes[:6]]
                self.latest_fb_stamp_ns = t_ns
                self.hardware_write_count = int(getattr(msg, "uart_cycle_sequence", 0))

    def _on_imu(self, msg: Imu) -> None:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()

        with self.lock:
            if self._csv_file.closed:
                return
            self.total_imu_samples += 1
            if self._first_imu_host_ns is None:
                self._first_imu_host_ns = host_ns

            q = normalize_quat_xyzw(
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            )
            if q is None or msg.orientation_covariance[0] == -1.0:
                return

            elapsed_init_s = (host_ns - self._first_imu_host_ns) / 1e9

            # Phase 0: Accumulate quiet HOME samples to establish q0
            if self.q0_xyzw is None:
                self._init_quats.append(q)
                if elapsed_init_s >= self.init_quiet_s and len(self._init_quats) >= self.min_init_samples:
                    self.q0_xyzw = average_quat_xyzw(self._init_quats)
                    self.get_logger().info(
                        f"HOME relative-zero q0 initialized from {len(self._init_quats)} samples: {self.q0_xyzw}"
                    )
                return

            # Compute HOME relative orientation
            q_rel = relative_quat_xyzw(self.q0_xyzw, q)
            if q_rel is None:
                return
            raw_r, raw_p, raw_y = quat_to_rpy_deg_xyzw(q_rel)

            # Canonical Rz(+90 deg) transform to Platform frame
            q_p_rel = quat_multiply_xyzw(
                quat_multiply_xyzw(self.q_PI, q_rel), quat_conjugate_xyzw(self.q_PI)
            )
            plat_r, plat_p, plat_y = quat_to_rpy_deg_xyzw(q_p_rel)

            gx = float(msg.angular_velocity.x)
            gy = float(msg.angular_velocity.y)
            gz = float(msg.angular_velocity.z)

            self.latest_q = q
            self.latest_raw_rel_rpy = (raw_r, raw_p, raw_y)
            self.latest_plat_rpy = (plat_r, plat_p, plat_y)
            self.latest_gyro = (gx, gy, gz)

            row_data = {
                "host_log_time_ns": host_ns,
                "host_monotonic_time_ns": mono_ns,
                "ros_stamp_sec": int(msg.header.stamp.sec),
                "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
                "stage": self.current_stage,
                "active_axes": "+".join(sorted(self.active_axes)),
                "roll_ref_deg": self.ref_rpy[0],
                "pitch_ref_deg": self.ref_rpy[1],
                "yaw_ref_deg": self.ref_rpy[2],
                "roll_meas_deg": plat_r,
                "pitch_meas_deg": plat_p,
                "yaw_meas_deg": plat_y,
                "roll_error_deg": self.ref_rpy[0] - plat_r,
                "pitch_error_deg": self.ref_rpy[1] - plat_p,
                "yaw_error_deg": self.ref_rpy[2] - plat_y,
                "roll_residual_deg": self.residual_rpy[0],
                "pitch_residual_deg": self.residual_rpy[1],
                "yaw_residual_deg": self.residual_rpy[2],
                "roll_cartesian_cmd_deg": self.cmd_rpy[0],
                "pitch_cartesian_cmd_deg": self.cmd_rpy[1],
                "yaw_cartesian_cmd_deg": self.cmd_rpy[2],
                "cmd_qx": self.cmd_q[0],
                "cmd_qy": self.cmd_q[1],
                "cmd_qz": self.cmd_q[2],
                "cmd_qw": self.cmd_q[3],
                "qx": q[0],
                "qy": q[1],
                "qz": q[2],
                "qw": q[3],
                "raw_rel_roll_deg": raw_r,
                "raw_rel_pitch_deg": raw_p,
                "raw_rel_yaw_deg": raw_y,
                "plat_roll_deg": plat_r,
                "plat_pitch_deg": plat_p,
                "plat_yaw_deg": plat_y,
                "gx_rad_s": gx,
                "gy_rad_s": gy,
                "gz_rad_s": gz,
                "joint_cmd_1": self.latest_cmd_steps[0],
                "joint_cmd_2": self.latest_cmd_steps[1],
                "joint_cmd_3": self.latest_cmd_steps[2],
                "joint_cmd_4": self.latest_cmd_steps[3],
                "joint_cmd_5": self.latest_cmd_steps[4],
                "joint_cmd_6": self.latest_cmd_steps[5],
                "joint_cmd_stamp_ns": self.latest_cmd_stamp_ns,
                "joint_fb_1": self.latest_fb_steps[0],
                "joint_fb_2": self.latest_fb_steps[1],
                "joint_fb_3": self.latest_fb_steps[2],
                "joint_fb_4": self.latest_fb_steps[3],
                "joint_fb_5": self.latest_fb_steps[4],
                "joint_fb_6": self.latest_fb_steps[5],
                "joint_fb_stamp_ns": self.latest_fb_stamp_ns,
                "hardware_write_count": self.hardware_write_count,
                "fault_codes": ":".join(str(c) for c in self.latest_fb_faults),
            }

            self._writer.writerow(row_data)
            self.recorded_sample_count += 1
            if self.current_stage in self.samples_by_stage:
                self.samples_by_stage[self.current_stage].append(row_data)


def compute_3dof_stage_settled_metrics(
    samples: List[Dict[str, Any]],
    target_axis: str = "roll",
    ref_angle_deg: float = 0.5,
    settle_skip_ratio: float = 0.5,
) -> Dict[str, Any]:
    """Computes settled steady-state metrics over the latter half of a motion stage."""
    if not samples:
        return {
            "count": 0,
            "total_stage_count": 0,
            "mean_plat_r": 0.0, "mean_plat_p": 0.0, "mean_plat_y": 0.0,
            "mean_error": 0.0, "max_error": 0.0,
            "mean_residual": 0.0, "max_residual": 0.0,
            "mean_cmd": 0.0, "peak_meas": 0.0,
            "abs_overshoot_deg": 0.0, "overshoot_pct": 0.0,
        }

    start_idx = int(len(samples) * settle_skip_ratio)
    settled = samples[start_idx:] if start_idx < len(samples) else samples

    plat_r = [float(s.get("plat_roll_deg", 0.0)) for s in settled]
    plat_p = [float(s.get("plat_pitch_deg", 0.0)) for s in settled]
    plat_y = [float(s.get("plat_yaw_deg", 0.0)) for s in settled]

    all_target_meas = [float(s.get(f"plat_{target_axis}_deg", 0.0)) for s in samples]
    target_errors = [float(s.get(f"{target_axis}_error_deg", 0.0)) for s in settled]
    target_residuals = [float(s.get(f"{target_axis}_residual_deg", 0.0)) for s in settled]
    target_cmds = [float(s.get(f"{target_axis}_cartesian_cmd_deg", 0.0)) for s in settled]

    if ref_angle_deg < 0.0:
        peak_meas = min(all_target_meas) if all_target_meas else 0.0
        abs_overshoot_deg = max(0.0, ref_angle_deg - peak_meas)
        overshoot_pct = (abs_overshoot_deg / abs(ref_angle_deg) * 100.0) if ref_angle_deg != 0.0 else 0.0
    else:
        peak_meas = max(all_target_meas) if all_target_meas else 0.0
        abs_overshoot_deg = max(0.0, peak_meas - ref_angle_deg)
        overshoot_pct = (abs_overshoot_deg / ref_angle_deg * 100.0) if ref_angle_deg != 0.0 else 0.0

    return {
        "count": len(settled),
        "total_stage_count": len(samples),
        "mean_plat_r": statistics.fmean(plat_r) if plat_r else 0.0,
        "mean_plat_p": statistics.fmean(plat_p) if plat_p else 0.0,
        "mean_plat_y": statistics.fmean(plat_y) if plat_y else 0.0,
        "mean_error": statistics.fmean(target_errors) if target_errors else 0.0,
        "max_error": max(target_errors, key=abs) if target_errors else 0.0,
        "mean_residual": statistics.fmean(target_residuals) if target_residuals else 0.0,
        "max_residual": max(target_residuals, key=abs) if target_residuals else 0.0,
        "mean_cmd": statistics.fmean(target_cmds) if target_cmds else 0.0,
        "min_cmd": min(target_cmds) if target_cmds else 0.0,
        "max_cmd": max(target_cmds) if target_cmds else 0.0,
        "peak_meas": peak_meas,
        "abs_overshoot_deg": abs_overshoot_deg,
        "overshoot_pct": overshoot_pct,
    }


def run_3dof_orientation_servo_experiment(
    *,
    ref_roll_deg: float = 0.5,
    ref_pitch_deg: float = 0.0,
    ref_yaw_deg: float = 0.0,
    active_axes: Sequence[str] = ("roll",),
    kp_roll: float = 0.5,
    kp_pitch: float = 0.5,
    kp_yaw: float = 0.5,
    residual_limit_roll_deg: float = 0.25,
    residual_limit_pitch_deg: float = 0.25,
    residual_limit_yaw_deg: float = 0.50,
    hold_s: float = 3.0,
    init_quiet_s: float = 2.0,
    output_dir: Path = Path("/home/lyapunov/bone_drill/results/j9_1/closed_loop_attitude"),
    motor_port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    motor_baud: int = 921600,
    imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    imu_baud: int = 460800,
    imu_topic: str = "/pus6/imu_platform/data",
    hardware_mode: str = "active",
    start_h30: bool = True,
    require_preflight: bool = True,
) -> Dict[str, Any]:
    """Runs the unified 3DOF orientation servo experiment."""

    active_set = set(a.lower() for a in active_axes)

    # Software Sign Sanity Check on all active axes
    for ax, ref_val, kp, lim in [
        ("roll", ref_roll_deg, kp_roll, residual_limit_roll_deg),
        ("pitch", ref_pitch_deg, kp_pitch, residual_limit_pitch_deg),
        ("yaw", ref_yaw_deg, kp_yaw, residual_limit_yaw_deg),
    ]:
        if ax in active_set and abs(ref_val) > 1e-4:
            test_meas = 0.8 * ref_val
            test_err = ref_val - test_meas
            test_delta = kp * test_err
            test_cmd = ref_val + test_delta
            if ref_val >= 0.0:
                if test_cmd <= ref_val:
                    raise ValueError(f"Sign sanity failure on {ax}: Kp={kp} caused delta <= 0 when ref > meas!")
            else:
                if test_cmd >= ref_val:
                    raise ValueError(f"Sign sanity failure on {ax}: Kp={kp} caused delta >= 0 when ref < meas!")
            print(f"[SIGN SANITY PASS] {ax.upper()}: Kp={kp}: for ref={ref_val:+.2f} vs meas={test_meas:+.2f}, delta={test_delta:+.4f} -> cmd={test_cmd:+.4f} reinforces toward ref.")

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"j9_1_3dof_orientation_servo_{stamp_str}.csv"
    meta_path = output_dir / f"j9_1_3dof_orientation_servo_{stamp_str}_metadata.json"

    experiment_report: Dict[str, Any] = {
        "csv_path": str(csv_path),
        "metadata_path": str(meta_path),
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware_mode": hardware_mode,
        "experiment_type": "3dof_orientation_servo",
        "active_axes": list(active_set),
        "ref_rpy_deg": [float(ref_roll_deg), float(ref_pitch_deg), float(ref_yaw_deg)],
        "kp": {"roll": float(kp_roll), "pitch": float(kp_pitch), "yaw": float(kp_yaw)},
        "residual_limits_deg": {
            "roll": float(residual_limit_roll_deg),
            "pitch": float(residual_limit_pitch_deg),
            "yaw": float(residual_limit_yaw_deg),
        },
        "stages_executed": [],
        "stage_metrics": {},
        "status": "INCOMPLETE",
    }

    print("\n" + "=" * 90)
    print("           J9.1-3 UNIFIED 3DOF ORIENTATION SERVO RUNNER")
    print("=" * 90)
    print(f"Reference RPY:         R={ref_roll_deg:+.2f}°, P={ref_pitch_deg:+.2f}°, Y={ref_yaw_deg:+.2f}°")
    print(f"Active Axes:           {list(active_set)}")
    print(f"Fixed P-Gains (Kp):    Roll={kp_roll}, Pitch={kp_pitch}, Yaw={kp_yaw}")
    print(f"Residual Limits:       Roll=+/-{residual_limit_roll_deg:.2f}°, Pitch=+/-{residual_limit_pitch_deg:.2f}°, Yaw=+/-{residual_limit_yaw_deg:.2f}°")
    print(f"Hold per stage:        {hold_s:.1f} s")
    print(f"Hardware mode:         {hardware_mode}")
    print(f"Motor UART port:       {motor_port} @ {motor_baud}")
    print(f"H30 IMU port:          {imu_port} @ {imu_baud}")
    print(f"H30 Topic:             {imu_topic}")
    print(f"Output Directory:      {output_dir}")
    print("=" * 90 + "\n")

    # Step 1: Preflight read-only check on motors if in active hardware mode
    if hardware_mode == "active" and require_preflight:
        print("[PREFLIGHT] Running non-intrusive 0x30 status scan on 6 motors...")
        ok, msg = check_motor_read_only_preflight(port=motor_port, baudrate=motor_baud)
        print(f"[PREFLIGHT RESULT] {msg}")
        if not ok:
            raise RuntimeError(f"Motor preflight failed: {msg}")
        print("[PREFLIGHT PASS] Motors verified online and within HOME band.")

    # Step 2: Ensure H30 node is running
    h30_proc: subprocess.Popen | None = None
    if start_h30 and hardware_mode == "active":
        print(f"[H30] Spawning H30 IMU node process for {imu_port}...")
        h30_cmd = [
            "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
            "-r", "__ns:=/pus6/imu_platform",
            "-p", f"port:={imu_port}",
            "-p", f"baud_rate:={imu_baud}",
            "-p", "frame_id:=pus6_platform_imu_link",
            "-p", "world_frame:=world",
            "-p", f"imu_topic:={imu_topic}",
            "-p", "publish_euler:=true",
        ]
        h30_proc = subprocess.Popen(h30_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        print(f"[H30] Started H30 node (PID {h30_proc.pid}), waiting 1.5s for initialization...")
        time.sleep(1.5)

    rclpy.init()
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    print(f"[GEOMETRY] Nominal Cartesian HOME z0 = {z0:.6f} m ({z0*1000.0:.3f} mm)")

    print("[INIT] Instantiating J8 ROS 2 nodes...")
    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode=hardware_mode,
        port=motor_port,
        baudrate=motor_baud,
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0)

    harness = J913DofOrientationServo(
        csv_path=csv_path,
        imu_topic=imu_topic,
        active_axes=active_axes,
        kp_roll=kp_roll,
        kp_pitch=kp_pitch,
        kp_yaw=kp_yaw,
        residual_limit_roll_deg=residual_limit_roll_deg,
        residual_limit_pitch_deg=residual_limit_pitch_deg,
        residual_limit_yaw_deg=residual_limit_yaw_deg,
        init_quiet_s=init_quiet_s,
    )

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor running 5 nodes.")

    try:
        # Step 3: Phase 0 - HOME Relative-Zero Initialization
        print(f"\n--- [Phase 0: HOME Relative-Zero Initialization] Holding HOME for {init_quiet_s:.1f}s ---")
        harness.set_stage("init_home", (0.0, 0.0, 0.0))
        q_home_orient = (0.0, 0.0, 0.0, 1.0)
        t_init_end = time.monotonic() + init_quiet_s
        while time.monotonic() < t_init_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0, q_xyzw=q_home_orient)
            time.sleep(0.020)

        # Wait up to additional 2.0s if q0 not ready
        t_wait_q0 = time.monotonic() + 2.0
        while time.monotonic() < t_wait_q0 and harness.q0_xyzw is None:
            time.sleep(0.050)

        if harness.q0_xyzw is None:
            raise RuntimeError("Failed to establish H30 relative-zero q0! Check if H30 is publishing.")
        print(f"[Phase 0 OK] Relative zero established: {harness.q0_xyzw}")

        # Determine primary evaluation axis
        if "pitch" in active_set and "roll" not in active_set and "yaw" not in active_set:
            primary_axis = "pitch"
            primary_ref = ref_pitch_deg
        elif "yaw" in active_set and "roll" not in active_set and "pitch" not in active_set:
            primary_axis = "yaw"
            primary_ref = ref_yaw_deg
        else:
            primary_axis = "roll"
            primary_ref = ref_roll_deg

        action_stage_tag = f"{primary_axis}_pos_closed_loop" if primary_ref >= 0.0 else f"{primary_axis}_neg_closed_loop"
        ret_stage_tag = f"{primary_axis}_ret_home"
        target_ref_tuple = (ref_roll_deg, ref_pitch_deg, ref_yaw_deg)
        stages = [
            (action_stage_tag, target_ref_tuple, f"ACTION RPY={target_ref_tuple} (3DOF P-only closed loop)"),
            (ret_stage_tag, (0.0, 0.0, 0.0), "Return HOME (3DOF closed loop)"),
        ]

        print("\n" + "#" * 60)
        print("       STARTING 3DOF ORIENTATION SERVO MOTION")
        print("#" * 60)

        for stage_tag, target_ref, stage_desc in stages:
            print(f"\n>>> Stage [{stage_tag}]: {stage_desc} (holding {hold_s:.1f}s) <<<")
            harness.set_stage(stage_tag, target_ref)
            experiment_report["stages_executed"].append(stage_tag)

            t_end = time.monotonic() + hold_s
            while time.monotonic() < t_end:
                t_loop_start = time.monotonic()

                # Compute 3DOF P-only control step
                errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()

                # Send Cartesian target to J8 black box at 50Hz
                harness.publish_target_pose(
                    x=0.0,
                    y=0.0,
                    z=z0,
                    q_xyzw=cmd_q,
                )

                # Safety check
                if hardware_mode == "active":
                    if any(f != 0 for f in harness.latest_fb_faults):
                        raise RuntimeError(f"Motor fault detected during {stage_tag}: {harness.latest_fb_faults}")
                    # Divergence check on active axes
                    if stage_tag == action_stage_tag:
                        axis_indices = {"roll": 0, "pitch": 1, "yaw": 2}
                        ref_values = {"roll": ref_roll_deg, "pitch": ref_pitch_deg, "yaw": ref_yaw_deg}
                        for ax in active_set:
                            ref_v = ref_values[ax]
                            if abs(ref_v) > 1e-4:
                                sign = 1.0 if ref_v >= 0.0 else -1.0
                                meas_v = harness.latest_plat_rpy[axis_indices[ax]]
                                if sign * meas_v < -0.30:
                                    raise RuntimeError(
                                        f"Divergence detected on {ax}: measured={meas_v:.4f}° opposite to ref={ref_v:+.2f}°!"
                                    )

                # 50Hz cycle timing
                elapsed = time.monotonic() - t_loop_start
                sleep_t = max(0.0, 0.020 - elapsed)
                time.sleep(sleep_t)

            print(f">>> Stage [{stage_tag}] complete. Settled steps: {harness.latest_fb_steps} <<<")

        # Step 5: Final quiet hold at HOME (1.0s)
        harness.set_stage("final_home", (0.0, 0.0, 0.0))
        t_final_end = time.monotonic() + 1.0
        while time.monotonic() < t_final_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0, q_xyzw=q_home_orient)
            time.sleep(0.020)

        # Stage Evaluation
        act_metrics = compute_3dof_stage_settled_metrics(
            harness.samples_by_stage.get(action_stage_tag, []),
            target_axis=primary_axis,
            ref_angle_deg=primary_ref,
        )
        ret_metrics = compute_3dof_stage_settled_metrics(
            harness.samples_by_stage.get(ret_stage_tag, []),
            target_axis=primary_axis,
            ref_angle_deg=0.0,
        )

        target_meas = act_metrics[f"mean_plat_{primary_axis[0]}"]
        print("\n" + "=" * 90)
        print(f"                 J9.1-3 3DOF ORIENTATION SERVO EVALUATION ({primary_axis.upper()} REGRESSION)")
        print("=" * 90)
        print(f"Reference RPY:                 R={ref_roll_deg:+.4f}°, P={ref_pitch_deg:+.4f}°, Y={ref_yaw_deg:+.4f}°")
        print(f"Active Axes:                   {list(active_set)}")
        print(f"Settled H30 Platform {primary_axis.capitalize()}:    {target_meas:+.4f} deg")
        print(f"Steady-State Error:            {act_metrics['mean_error']:+.4f} deg")
        print(f"Peak/Extr Measured {primary_axis.capitalize()}:      {act_metrics['peak_meas']:+.4f} deg")
        print(f"Absolute Overshoot:            {act_metrics['abs_overshoot_deg']:.4f} deg ({act_metrics['overshoot_pct']:.1f}%)")
        for ax in ["roll", "pitch", "yaw"]:
            if ax != primary_axis:
                print(f"Cross-Axis {ax.capitalize()}:             {act_metrics[f'mean_plat_{ax[0]}']:+.4f} deg")
        print(f"Mean {primary_axis.capitalize()} Residual:           {act_metrics['mean_residual']:+.4f} deg (max={act_metrics['max_residual']:+.4f} deg)")
        print(f"Mean Cartesian {primary_axis.capitalize()} Cmd:      {act_metrics['mean_cmd']:+.4f} deg (min/max={act_metrics['min_cmd']:+.4f}/{act_metrics['max_cmd']:+.4f} deg)")
        print(f"Settled Return HOME {primary_axis.capitalize()}:     {ret_metrics[f'mean_plat_{primary_axis[0]}']:+.4f} deg")
        print(f"Hardware writes:               {harness.hardware_write_count}")
        print(f"Motor faults:                  {harness.latest_fb_faults}")
        print("=" * 90)

        experiment_report["stage_metrics"][action_stage_tag] = act_metrics
        experiment_report["stage_metrics"][ret_stage_tag] = ret_metrics
        experiment_report["status"] = "PASS"
        print(f"\n[EXPERIMENT PASS] 3DOF Orientation Servo {primary_axis.upper()} single-axis regression completed successfully.")

    finally:
        print("\n[TEARDOWN] Shutting down ROS 2 nodes and flushing CSV...")
        try:
            motor_driver_node.destroy_node()
        except Exception:
            pass
        try:
            control_node.destroy_node()
            kinematics_node.destroy_node()
            fk_node.destroy_node()
            harness.destroy_node()
            executor.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        harness.close()

        if h30_proc and h30_proc.poll() is None:
            print(f"[TEARDOWN] Terminating H30 IMU process (PID {h30_proc.pid})...")
            try:
                os.killpg(os.getpgid(h30_proc.pid), signal.SIGINT)
                h30_proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(h30_proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    # Compile and write metadata JSON
    experiment_report["end_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    experiment_report["total_recorded_samples"] = harness.recorded_sample_count
    experiment_report["total_imu_samples"] = harness.total_imu_samples
    experiment_report["final_write_count"] = harness.hardware_write_count
    experiment_report["final_motor_steps"] = harness.latest_fb_steps
    experiment_report["final_motor_faults"] = harness.latest_fb_faults
    experiment_report["q0_xyzw"] = list(harness.q0_xyzw) if harness.q0_xyzw else None

    meta_path.write_text(json.dumps(experiment_report, indent=2), encoding="utf-8")
    print(f"[METADATA] Saved metadata JSON to {meta_path}")
    print(f"[CSV] Saved CSV to {csv_path}")

    return experiment_report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J9.1-3 3DOF Orientation Servo Runner")
    parser.add_argument("--ref-roll-deg", type=float, default=0.5, help="Roll reference angle in degrees (default: 0.5)")
    parser.add_argument("--ref-pitch-deg", type=float, default=0.0, help="Pitch reference angle in degrees (default: 0.0)")
    parser.add_argument("--ref-yaw-deg", type=float, default=0.0, help="Yaw reference angle in degrees (default: 0.0)")
    parser.add_argument("--active-axes", default="roll", help="Comma-separated active axes: roll,pitch,yaw (default: roll)")
    parser.add_argument("--kp-roll", type=float, default=0.5, help="Roll Kp (default: 0.5)")
    parser.add_argument("--kp-pitch", type=float, default=0.5, help="Pitch Kp (default: 0.5)")
    parser.add_argument("--kp-yaw", type=float, default=0.5, help="Yaw Kp (default: 0.5)")
    parser.add_argument("--residual-limit-roll-deg", type=float, default=0.25, help="Roll residual limit (default: 0.25)")
    parser.add_argument("--residual-limit-pitch-deg", type=float, default=0.25, help="Pitch residual limit (default: 0.25)")
    parser.add_argument("--residual-limit-yaw-deg", type=float, default=0.50, help="Yaw residual limit (default: 0.50)")
    parser.add_argument("--hold-time", type=float, default=3.0, help="Hold time per stage in seconds")
    parser.add_argument("--init-quiet-time", type=float, default=2.0, help="Initial quiet hold time at HOME to establish q0")
    parser.add_argument("--output-dir", default="/home/lyapunov/bone_drill/results/j9_1/closed_loop_attitude", help="Output directory")
    parser.add_argument("--motor-port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", help="Motor serial port")
    parser.add_argument("--motor-baud", type=int, default=921600, help="Motor baud rate")
    parser.add_argument("--imu-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00", help="Platform H30 serial port")
    parser.add_argument("--imu-baud", type=int, default=460800, help="H30 baud rate")
    parser.add_argument("--imu-topic", default="/pus6/imu_platform/data", help="H30 ROS 2 topic")
    parser.add_argument("--hardware-mode", choices=["active", "mock"], default="active", help="Hardware mode: active or mock")
    parser.add_argument("--no-start-h30", dest="start_h30", action="store_false", help="Do not spawn H30 node process automatically")
    parser.add_argument("--no-preflight", dest="require_preflight", action="store_false", help="Skip preflight read-only check")

    args = parser.parse_args(list(argv) if argv is not None else None)
    active_axes = [a.strip() for a in args.active_axes.split(",") if a.strip()]

    try:
        report = run_3dof_orientation_servo_experiment(
            ref_roll_deg=args.ref_roll_deg,
            ref_pitch_deg=args.ref_pitch_deg,
            ref_yaw_deg=args.ref_yaw_deg,
            active_axes=active_axes,
            kp_roll=args.kp_roll,
            kp_pitch=args.kp_pitch,
            kp_yaw=args.kp_yaw,
            residual_limit_roll_deg=args.residual_limit_roll_deg,
            residual_limit_pitch_deg=args.residual_limit_pitch_deg,
            residual_limit_yaw_deg=args.residual_limit_yaw_deg,
            hold_s=args.hold_time,
            init_quiet_s=args.init_quiet_time,
            output_dir=Path(args.output_dir),
            motor_port=args.motor_port,
            motor_baud=args.motor_baud,
            imu_port=args.imu_port,
            imu_baud=args.imu_baud,
            imu_topic=args.imu_topic,
            hardware_mode=args.hardware_mode,
            start_h30=args.start_h30,
            require_preflight=args.require_preflight,
        )
        return 0 if report.get("status") == "PASS" else 1
    except Exception as e:
        print(f"\n[FATAL ERROR] 3DOF closed-loop experiment failed: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
