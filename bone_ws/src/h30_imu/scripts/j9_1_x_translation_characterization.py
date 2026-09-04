#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J9.1-4B: Known Small Translation (+0.5 mm X) -> Dual-IMU Inertial Signature Runner.

Executes a single known-amplitude X-axis open-loop translation:
  0 -> X = +0.5 mm -> 0
using the frozen J8 ROS 2 control chain:
  /pus6/target_cartesian_pose -> Pus6KinematicsNode
  -> /pus6/desired_joint_deltas -> Pus6ControlNode
  -> /pus6/command_joint_steps -> Pus6MotorDriverNode (50Hz Active Servo)
  -> /pus6/joint_feedback/timed -> Pus6FkNode

Simultaneously records:
  - Base H30 IMU (/pus6/imu_base/data)
  - Platform H30 IMU (/pus6/imu_platform/data)
  - Joint commands (1..6)
  - Joint feedbacks (1..6)
  - Forward kinematics (p_BP, R_BP)

Offline Analysis:
  - FK local X displacement & cross-axis deviations
  - Platform & Base H30 world acceleration peaks (pos/neg) and motion RMS
  - Platform & Base gyro responses
  - Base <-> Platform timestamp pairing statistics
  - Pre-HOME vs Post-HOME acceleration bias stability
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
from typing import Any, Dict, List, Sequence, Tuple

# Workspace path setup
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"]:
    p = WS_SRC / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback

G_STANDARD = 9.80665

CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "stage",
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "target_roll_deg",
    "target_pitch_deg",
    "target_yaw_deg",
    "joint_cmd_1",
    "joint_cmd_2",
    "joint_cmd_3",
    "joint_cmd_4",
    "joint_cmd_5",
    "joint_cmd_6",
    "joint_fb_1",
    "joint_fb_2",
    "joint_fb_3",
    "joint_fb_4",
    "joint_fb_5",
    "joint_fb_6",
    "fk_x_mm",
    "fk_y_mm",
    "fk_z_mm",
    "fk_roll_deg",
    "fk_pitch_deg",
    "fk_yaw_deg",
    # Platform H30 IMU
    "p_stamp_ns",
    "p_qx",
    "p_qy",
    "p_qz",
    "p_qw",
    "p_gx_rad_s",
    "p_gy_rad_s",
    "p_gz_rad_s",
    "p_ax_raw",
    "p_ay_raw",
    "p_az_raw",
    "p_aw_x",
    "p_aw_y",
    "p_aw_z",
    # Base H30 IMU
    "b_stamp_ns",
    "b_qx",
    "b_qy",
    "b_qz",
    "b_qw",
    "b_gx_rad_s",
    "b_gy_rad_s",
    "b_gz_rad_s",
    "b_ax_raw",
    "b_ay_raw",
    "b_az_raw",
    "b_aw_x",
    "b_aw_y",
    "b_aw_z",
    # Safety
    "hardware_write_count",
    "fault_codes",
]


def check_motor_read_only_preflight(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
) -> Tuple[bool, List[int], str]:
    """Non-intrusive 0x30 read-only status check before starting active servo."""
    from run_j8_c_hardware_readonly_audit import run_j8_c_audit

    try:
        res = run_j8_c_audit(port=port, baudrate=baudrate, duration_s=0.2)
        if res.get("status") == "PASS":
            steps = res.get("initial_scan", {}).get("steps", [1000] * 6)
            return True, steps, "All 6 motors online at HOME, fault=0"
        return False, [1000] * 6, f"Preflight returned: {res.get('status')}"
    except Exception as exc:
        return False, [1000] * 6, f"Preflight exception: {exc}"


class XTranslationCharacterizationHarness(Node):
    """Orchestrates Cartesian open-loop X translation and synchronizes telemetry."""

    def __init__(
        self,
        *,
        csv_path: Path,
        z0: float,
        target_dx_m: float = 0.0005,
    ) -> None:
        super().__init__("j9_1_x_translation_characterization_harness")

        self.csv_path = csv_path
        self.z0 = z0
        self.target_dx_m = target_dx_m

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self.lock = threading.Lock()
        self.current_stage = "initializing"
        self.target_x_mm = 0.0
        self.target_y_mm = 0.0
        self.target_z_mm = z0 * 1000.0
        self.target_rpy_deg = [0.0, 0.0, 0.0]

        # Robot state
        self.latest_cmd_steps = [1000] * 6
        self.latest_fb_steps = [1000] * 6
        self.latest_fault_codes = [0] * 6
        self.hardware_write_count = 0
        self.fk_xyz_m = [0.0, 0.0, z0]
        self.fk_rpy_rad = [0.0, 0.0, 0.0]

        # Latest Base IMU state
        self.latest_base_imu: Dict[str, Any] = {
            "stamp_ns": 0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "gyro": [0.0, 0.0, 0.0],
            "accel_raw": [0.0, 0.0, G_STANDARD],
            "aw": [0.0, 0.0, 0.0],
        }

        # Latest Platform IMU state
        self.latest_plat_imu: Dict[str, Any] = {
            "stamp_ns": 0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "gyro": [0.0, 0.0, 0.0],
            "accel_raw": [0.0, 0.0, G_STANDARD],
            "aw": [0.0, 0.0, 0.0],
        }

        self.kin = Analytical6PusKinematics()

        # QoS setups
        sensor_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        reliable_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        # Publishers & Subscriptions
        self.target_pose_pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", reliable_qos)

        self.cmd_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, sensor_qos
        )
        self.fb_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_feedback, sensor_qos
        )
        self.fk_sub = self.create_subscription(
            FkStateStamped, "/pus6/fk_state", self._on_fk_state, sensor_qos
        )
        self.base_imu_sub = self.create_subscription(
            Imu, "/pus6/imu_base/data", self._on_base_imu, qos_profile_sensor_data
        )
        self.plat_imu_sub = self.create_subscription(
            Imu, "/pus6/imu_platform/data", self._on_platform_imu, qos_profile_sensor_data
        )

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def set_stage(self, stage: str, x_m: float, y_m: float = 0.0, z_m: float | None = None) -> None:
        if z_m is None:
            z_m = self.z0
        with self.lock:
            self.current_stage = stage
            self.target_x_mm = x_m * 1000.0
            self.target_y_mm = y_m * 1000.0
            self.target_z_mm = z_m * 1000.0

    def publish_target_pose(self, x_m: float, y_m: float = 0.0, z_m: float | None = None) -> None:
        if z_m is None:
            z_m = self.z0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x_m)
        msg.pose.position.y = float(y_m)
        msg.pose.position.z = float(z_m)
        # Identity orientation (R=P=Y=0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        with self.lock:
            if len(msg.data) >= 6:
                self.latest_cmd_steps = list(msg.data[:6])

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            if len(msg.raw_steps) >= 6:
                self.latest_fb_steps = [int(s) for s in msg.raw_steps[:6]]
                self.latest_fault_codes = [int(f) for f in msg.fault_codes[:6]]
                self.hardware_write_count = int(getattr(msg, "uart_cycle_sequence", 0))

                # Local offline FK calculation to guarantee immediate synchronization
                slider_m = [0.05 + (s - 1000) / 40000.0 for s in self.latest_fb_steps]
                fk_res = self.kin.forward_kinematics(slider_m)
                self.fk_xyz_m = fk_res[0:3]
                self.fk_rpy_rad = fk_res[3:6]

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        with self.lock:
            self.fk_xyz_m = [float(msg.x_m), float(msg.y_m), float(msg.z_m)]
            self.fk_rpy_rad = [float(msg.roll_rad), float(msg.pitch_rad), float(msg.yaw_rad)]

    def _on_base_imu(self, msg: Imu) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)

        rot = R.from_quat([qx, qy, qz, qw])
        f_world = rot.apply(np.array([ax, ay, az], dtype=np.float64))
        aw_x = float(f_world[0])
        aw_y = float(f_world[1])
        aw_z = float(f_world[2]) - G_STANDARD

        with self.lock:
            self.latest_base_imu = {
                "stamp_ns": stamp_ns,
                "quat": [qx, qy, qz, qw],
                "gyro": [gx, gy, gz],
                "accel_raw": [ax, ay, az],
                "aw": [aw_x, aw_y, aw_z],
            }

    def _on_platform_imu(self, msg: Imu) -> None:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)

        rot = R.from_quat([qx, qy, qz, qw])
        f_world = rot.apply(np.array([ax, ay, az], dtype=np.float64))
        aw_x = float(f_world[0])
        aw_y = float(f_world[1])
        aw_z = float(f_world[2]) - G_STANDARD

        with self.lock:
            if self._csv_file.closed:
                return

            self.latest_plat_imu = {
                "stamp_ns": stamp_ns,
                "quat": [qx, qy, qz, qw],
                "gyro": [gx, gy, gz],
                "accel_raw": [ax, ay, az],
                "aw": [aw_x, aw_y, aw_z],
            }

            b = self.latest_base_imu
            row = {
                "host_log_time_ns": host_ns,
                "host_monotonic_time_ns": mono_ns,
                "stage": self.current_stage,
                "target_x_mm": self.target_x_mm,
                "target_y_mm": self.target_y_mm,
                "target_z_mm": self.target_z_mm,
                "target_roll_deg": self.target_rpy_deg[0],
                "target_pitch_deg": self.target_rpy_deg[1],
                "target_yaw_deg": self.target_rpy_deg[2],
                "joint_cmd_1": self.latest_cmd_steps[0],
                "joint_cmd_2": self.latest_cmd_steps[1],
                "joint_cmd_3": self.latest_cmd_steps[2],
                "joint_cmd_4": self.latest_cmd_steps[3],
                "joint_cmd_5": self.latest_cmd_steps[4],
                "joint_cmd_6": self.latest_cmd_steps[5],
                "joint_fb_1": self.latest_fb_steps[0],
                "joint_fb_2": self.latest_fb_steps[1],
                "joint_fb_3": self.latest_fb_steps[2],
                "joint_fb_4": self.latest_fb_steps[3],
                "joint_fb_5": self.latest_fb_steps[4],
                "joint_fb_6": self.latest_fb_steps[5],
                "fk_x_mm": self.fk_xyz_m[0] * 1000.0,
                "fk_y_mm": self.fk_xyz_m[1] * 1000.0,
                "fk_z_mm": self.fk_xyz_m[2] * 1000.0,
                "fk_roll_deg": math.degrees(self.fk_rpy_rad[0]),
                "fk_pitch_deg": math.degrees(self.fk_rpy_rad[1]),
                "fk_yaw_deg": math.degrees(self.fk_rpy_rad[2]),
                # Platform
                "p_stamp_ns": stamp_ns,
                "p_qx": qx,
                "p_qy": qy,
                "p_qz": qz,
                "p_qw": qw,
                "p_gx_rad_s": gx,
                "p_gy_rad_s": gy,
                "p_gz_rad_s": gz,
                "p_ax_raw": ax,
                "p_ay_raw": ay,
                "p_az_raw": az,
                "p_aw_x": aw_x,
                "p_aw_y": aw_y,
                "p_aw_z": aw_z,
                # Base
                "b_stamp_ns": b["stamp_ns"],
                "b_qx": b["quat"][0],
                "b_qy": b["quat"][1],
                "b_qz": b["quat"][2],
                "b_qw": b["quat"][3],
                "b_gx_rad_s": b["gyro"][0],
                "b_gy_rad_s": b["gyro"][1],
                "b_gz_rad_s": b["gyro"][2],
                "b_ax_raw": b["accel_raw"][0],
                "b_ay_raw": b["accel_raw"][1],
                "b_az_raw": b["accel_raw"][2],
                "b_aw_x": b["aw"][0],
                "b_aw_y": b["aw"][1],
                "b_aw_z": b["aw"][2],
                "hardware_write_count": self.hardware_write_count,
                "fault_codes": str(self.latest_fault_codes),
            }
            self._writer.writerow(row)


def analyze_x_translation_experiment(csv_path: Path, target_dx_mm: float = 0.5) -> Dict[str, Any]:
    """Parses recorded telemetry and computes comprehensive analysis metrics."""
    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
    if not rows:
        return {"status": "EMPTY"}

    pre_home_rows = [r for r in rows if r["stage"] == "pre_home"]
    action_rows = [r for r in rows if r["stage"].startswith("action_x")]
    post_home_rows = [r for r in rows if r["stage"] == "post_home"]

    # Pre-home baselines
    pre_fk_x = statistics.fmean([float(r["fk_x_mm"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0
    pre_fk_y = statistics.fmean([float(r["fk_y_mm"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0
    pre_fk_z = statistics.fmean([float(r["fk_z_mm"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0
    pre_fk_r = statistics.fmean([float(r["fk_roll_deg"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0
    pre_fk_p = statistics.fmean([float(r["fk_pitch_deg"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0
    pre_fk_y_rot = statistics.fmean([float(r["fk_yaw_deg"]) for r in pre_home_rows[-100:]]) if pre_home_rows else 0.0

    pre_p_aw = [
        statistics.fmean([float(r["p_aw_x"]) for r in pre_home_rows[-100:]]),
        statistics.fmean([float(r["p_aw_y"]) for r in pre_home_rows[-100:]]),
        statistics.fmean([float(r["p_aw_z"]) for r in pre_home_rows[-100:]]),
    ] if pre_home_rows else [0.0, 0.0, 0.0]

    pre_b_aw = [
        statistics.fmean([float(r["b_aw_x"]) for r in pre_home_rows[-100:]]),
        statistics.fmean([float(r["b_aw_y"]) for r in pre_home_rows[-100:]]),
        statistics.fmean([float(r["b_aw_z"]) for r in pre_home_rows[-100:]]),
    ] if pre_home_rows else [0.0, 0.0, 0.0]

    # Action stage metrics
    action_fk_x_all = [float(r["fk_x_mm"]) for r in action_rows]
    action_fk_y_all = [float(r["fk_y_mm"]) for r in action_rows]
    action_fk_z_all = [float(r["fk_z_mm"]) for r in action_rows]
    action_fk_r_all = [float(r["fk_roll_deg"]) for r in action_rows]
    action_fk_p_all = [float(r["fk_pitch_deg"]) for r in action_rows]
    action_fk_y_rot_all = [float(r["fk_yaw_deg"]) for r in action_rows]

    # Last 100 samples of action stage (settled)
    settled_action = action_rows[-100:] if len(action_rows) >= 100 else action_rows
    settled_fk_x = statistics.fmean([float(r["fk_x_mm"]) for r in settled_action])
    settled_fk_y = statistics.fmean([float(r["fk_y_mm"]) for r in settled_action])
    settled_fk_z = statistics.fmean([float(r["fk_z_mm"]) for r in settled_action])
    settled_fk_r = statistics.fmean([float(r["fk_roll_deg"]) for r in settled_action])
    settled_fk_p = statistics.fmean([float(r["fk_pitch_deg"]) for r in settled_action])
    settled_fk_y_rot = statistics.fmean([float(r["fk_yaw_deg"]) for r in settled_action])

    local_dx = settled_fk_x - pre_fk_x
    local_dy = settled_fk_y - pre_fk_y
    local_dz = settled_fk_z - pre_fk_z
    local_dr = settled_fk_r - pre_fk_r
    local_dp = settled_fk_p - pre_fk_p
    local_dyaw = settled_fk_y_rot - pre_fk_y_rot

    # Acceleration peaks and RMS during action stage
    p_aw_x = [float(r["p_aw_x"]) for r in action_rows]
    p_aw_y = [float(r["p_aw_y"]) for r in action_rows]
    p_aw_z = [float(r["p_aw_z"]) for r in action_rows]

    b_aw_x = [float(r["b_aw_x"]) for r in action_rows]
    b_aw_y = [float(r["b_aw_y"]) for r in action_rows]
    b_aw_z = [float(r["b_aw_z"]) for r in action_rows]

    def _rms(vals: List[float]) -> float:
        return float(math.sqrt(statistics.fmean([x * x for x in vals]))) if vals else 0.0

    p_peaks = {
        "x_pos_peak": float(max(p_aw_x)),
        "x_neg_peak": float(min(p_aw_x)),
        "y_pos_peak": float(max(p_aw_y)),
        "y_neg_peak": float(min(p_aw_y)),
        "z_pos_peak": float(max(p_aw_z)),
        "z_neg_peak": float(min(p_aw_z)),
        "x_rms": _rms(p_aw_x),
        "y_rms": _rms(p_aw_y),
        "z_rms": _rms(p_aw_z),
    }

    b_peaks = {
        "x_pos_peak": float(max(b_aw_x)),
        "x_neg_peak": float(min(b_aw_x)),
        "y_pos_peak": float(max(b_aw_y)),
        "y_neg_peak": float(min(b_aw_y)),
        "z_pos_peak": float(max(b_aw_z)),
        "z_neg_peak": float(min(b_aw_z)),
        "x_rms": _rms(b_aw_x),
        "y_rms": _rms(b_aw_y),
        "z_rms": _rms(b_aw_z),
    }

    # Gyro response
    p_gx = [float(r["p_gx_rad_s"]) for r in action_rows]
    p_gy = [float(r["p_gy_rad_s"]) for r in action_rows]
    p_gz = [float(r["p_gz_rad_s"]) for r in action_rows]
    p_gyro_peak = {
        "gx_peak_deg_s": math.degrees(max(abs(x) for x in p_gx)),
        "gy_peak_deg_s": math.degrees(max(abs(x) for x in p_gy)),
        "gz_peak_deg_s": math.degrees(max(abs(x) for x in p_gz)),
    }

    b_gx = [float(r["b_gx_rad_s"]) for r in action_rows]
    b_gy = [float(r["b_gy_rad_s"]) for r in action_rows]
    b_gz = [float(r["b_gz_rad_s"]) for r in action_rows]
    b_gyro_peak = {
        "gx_peak_deg_s": math.degrees(max(abs(x) for x in b_gx)),
        "gy_peak_deg_s": math.degrees(max(abs(x) for x in b_gy)),
        "gz_peak_deg_s": math.degrees(max(abs(x) for x in b_gz)),
    }

    # Post-home baselines & stability check
    post_p_aw = [
        statistics.fmean([float(r["p_aw_x"]) for r in post_home_rows[-100:]]),
        statistics.fmean([float(r["p_aw_y"]) for r in post_home_rows[-100:]]),
        statistics.fmean([float(r["p_aw_z"]) for r in post_home_rows[-100:]]),
    ] if post_home_rows else [0.0, 0.0, 0.0]

    post_b_aw = [
        statistics.fmean([float(r["b_aw_x"]) for r in post_home_rows[-100:]]),
        statistics.fmean([float(r["b_aw_y"]) for r in post_home_rows[-100:]]),
        statistics.fmean([float(r["b_aw_z"]) for r in post_home_rows[-100:]]),
    ] if post_home_rows else [0.0, 0.0, 0.0]

    # Timestamp pairing analysis (p_stamp_ns vs b_stamp_ns)
    pairing_diffs_ms: List[float] = []
    for r in rows:
        t_p = int(r["p_stamp_ns"])
        t_b = int(r["b_stamp_ns"])
        if t_p > 0 and t_b > 0:
            pairing_diffs_ms.append(abs(t_p - t_b) / 1e6)

    offsets = np.array(pairing_diffs_ms) if pairing_diffs_ms else np.array([0.0])

    pairing_stats = {
        "mean_offset_ms": float(np.mean(offsets)),
        "median_offset_ms": float(np.median(offsets)),
        "p95_offset_ms": float(np.percentile(offsets, 95)),
        "p99_offset_ms": float(np.percentile(offsets, 99)),
        "max_offset_ms": float(np.max(offsets)),
    }

    # Joint commands & feedback in settled action state
    last_action_row = settled_action[-1] if settled_action else rows[-1]
    cmd_steps = [int(last_action_row[f"joint_cmd_{i}"]) for i in range(1, 7)]
    fb_steps = [int(last_action_row[f"joint_fb_{i}"]) for i in range(1, 7)]

    return {
        "target_dx_mm": target_dx_mm,
        "fk_local_displacement": {
            "dx_mm": local_dx,
            "dy_mm": local_dy,
            "dz_mm": local_dz,
            "droll_deg": local_dr,
            "dpitch_deg": local_dp,
            "dyaw_deg": local_dyaw,
            "settled_fk_x_mm": settled_fk_x,
            "pre_fk_x_mm": pre_fk_x,
        },
        "joint_state": {
            "cmd_steps": cmd_steps,
            "fb_steps": fb_steps,
            "delta_steps": [fb_steps[i] - 1000 for i in range(6)],
        },
        "platform_accel_peaks": p_peaks,
        "base_accel_peaks": b_peaks,
        "platform_gyro_peaks": p_gyro_peak,
        "base_gyro_peaks": b_gyro_peak,
        "pre_home_bias": {
            "platform_aw_m_s2": pre_p_aw,
            "base_aw_m_s2": pre_b_aw,
        },
        "post_home_bias": {
            "platform_aw_m_s2": post_p_aw,
            "base_aw_m_s2": post_b_aw,
        },
        "bias_shift_m_s2": {
            "platform_aw_shift": [post_p_aw[i] - pre_p_aw[i] for i in range(3)],
            "base_aw_shift": [post_b_aw[i] - pre_b_aw[i] for i in range(3)],
        },
        "timestamp_pairing_stats": pairing_stats,
        "total_samples": len(rows),
        "status": "PASS",
    }


def run_x_translation_experiment(
    target_dx_mm: float = 0.5,
    hold_s: float = 8.0,
    quiet_s: float = 5.0,
    output_dir: Path = Path("/home/lyapunov/bone_drill/results/j9_1/translation_characterization"),
    motor_port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    base_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_motor: int = 921600,
    baud_imu: int = 460800,
) -> Dict[str, Any]:
    """Full execution function for J9.1-4B X open-loop translation experiment."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    dir_tag = f"pos{int(round(abs(target_dx_mm)*10)):02d}mm" if target_dx_mm >= 0 else f"neg{int(round(abs(target_dx_mm)*10)):02d}mm"
    csv_path = output_dir / f"j9_1_x_translation_{dir_tag}_{stamp_str}.csv"
    meta_path = output_dir / f"j9_1_x_translation_{dir_tag}_{stamp_str}_metadata.json"

    print("\n" + "=" * 90)
    print(f"      J9.1-4B KNOWN SMALL TRANSLATION ({target_dx_mm:+.2f} mm X) -> DUAL-IMU INERTIAL SIGNATURE")
    print("=" * 90)
    print(f"Target Displacement:     X = {target_dx_mm:+.2f} mm")
    print(f"Hold Duration:           {hold_s:.1f} s")
    print(f"Quiet / Baseline:        {quiet_s:.1f} s pre / post")
    print(f"Output CSV:              {csv_path}")
    print("=" * 90 + "\n")

    # Step 1: Preflight 0x30 audit
    print("[PREFLIGHT] Checking motor online status via non-intrusive 0x30 scan...")
    ok, home_steps, audit_msg = check_motor_read_only_preflight(port=motor_port, baudrate=baud_motor)
    print(f"[PREFLIGHT RESULT] {audit_msg}")
    if not ok:
        raise RuntimeError(f"Motor preflight audit failed: {audit_msg}")

    # Check nominal height
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    target_dx_m = target_dx_mm * 1e-3

    # Step 2: Spawn Base and Platform H30 driver nodes
    print("[SPAWNING] Starting Base H30 node (/pus6/imu_base/data)...")
    cmd_base = [
        "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
        "-r", "__ns:=/pus6/imu_base",
        "-p", f"port:={base_imu_port}",
        "-p", f"baud_rate:={baud_imu}",
        "-p", "frame_id:=pus6_base_imu_link",
        "-p", "world_frame:=world",
        "-p", "imu_topic:=/pus6/imu_base/data",
    ]
    base_proc = subprocess.Popen(cmd_base, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    print("[SPAWNING] Starting Platform H30 node (/pus6/imu_platform/data)...")
    cmd_plat = [
        "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
        "-r", "__ns:=/pus6/imu_platform",
        "-p", f"port:={plat_imu_port}",
        "-p", f"baud_rate:={baud_imu}",
        "-p", "frame_id:=pus6_platform_imu_link",
        "-p", "world_frame:=world",
        "-p", "imu_topic:=/pus6/imu_platform/data",
    ]
    plat_proc = subprocess.Popen(cmd_plat, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    time.sleep(1.5)

    # Step 3: Initialize ROS 2 nodes for frozen J8 execution chain
    rclpy.init()

    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=motor_port,
        baudrate=baud_motor,
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0)
    harness = XTranslationCharacterizationHarness(
        csv_path=csv_path,
        z0=z0,
        target_dx_m=target_dx_m,
    )

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor running full frozen J8 control stack + harness.")

    report: Dict[str, Any] = {}

    try:
        # -------------------------------------------------------------
        # Phase 1: Pre-HOME Hold (Quiet baseline)
        # -------------------------------------------------------------
        print(f"\n>>> [Stage 1: Pre-HOME Hold] Holding static HOME for {quiet_s:.1f} s <<<")
        harness.set_stage("pre_home", 0.0)
        t_end = time.monotonic() + quiet_s
        while time.monotonic() < t_end:
            harness.publish_target_pose(x_m=0.0, y_m=0.0, z_m=z0)
            time.sleep(0.020)

        # -------------------------------------------------------------
        # Phase 2: Action (Signed X translation)
        # -------------------------------------------------------------
        action_stage = "action_x_pos" if target_dx_mm >= 0 else "action_x_neg"
        print(f"\n>>> [Stage 2: Action {target_dx_mm:+.2f} mm X] Stepping to X = {target_dx_mm:+.2f} mm (holding {hold_s:.1f} s) <<<")
        harness.set_stage(action_stage, target_dx_m)
        t_end = time.monotonic() + hold_s
        while time.monotonic() < t_end:
            harness.publish_target_pose(x_m=target_dx_m, y_m=0.0, z_m=z0)
            time.sleep(0.020)

            # Continuous safety fault check
            if any(f != 0 for f in harness.latest_fault_codes):
                raise RuntimeError(f"Motor fault detected during motion: {harness.latest_fault_codes}")

        # -------------------------------------------------------------
        # Phase 3: Return to HOME
        # -------------------------------------------------------------
        print(f"\n>>> [Stage 3: Return to HOME] Returning to X = 0.0 mm (holding 4.0 s) <<<")
        harness.set_stage("return_home", 0.0)
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x_m=0.0, y_m=0.0, z_m=z0)
            time.sleep(0.020)
            if any(f != 0 for f in harness.latest_fault_codes):
                raise RuntimeError(f"Motor fault detected during return: {harness.latest_fault_codes}")

        # -------------------------------------------------------------
        # Phase 4: Post-HOME Hold
        # -------------------------------------------------------------
        print(f"\n>>> [Stage 4: Post-HOME Hold] Holding static post-HOME for {quiet_s:.1f} s <<<")
        harness.set_stage("post_home", 0.0)
        t_end = time.monotonic() + quiet_s
        while time.monotonic() < t_end:
            harness.publish_target_pose(x_m=0.0, y_m=0.0, z_m=z0)
            time.sleep(0.020)

        print("\n[MOTION FINISHED] Motion profile completed cleanly.")

    finally:
        print("[SHUTDOWN] Closing telemetry CSV and stopping ROS 2 nodes...")
        harness.close()
        try:
            executor.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

        for proc, tag in [(base_proc, "Base H30"), (plat_proc, "Platform H30")]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=2.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass

    # Step 4: Perform post-experiment analysis
    print("[ANALYSIS] Computing statistical metrics and inertial signatures...")
    metrics = analyze_x_translation_experiment(csv_path, target_dx_mm=target_dx_mm)

    final_report = {
        "experiment_name": "J9.1-4B_X_Translation_Characterization",
        "timestamp": stamp_str,
        "csv_path": str(csv_path),
        "target_dx_mm": target_dx_mm,
        "z0_m": z0,
        "metrics": metrics,
        "hardware_write_count": harness.hardware_write_count,
        "final_fault_codes": harness.latest_fault_codes,
        "status": "PASS",
    }

    meta_path.write_text(json.dumps(final_report, indent=2), encoding="utf-8")
    print(f"[REPORT] Metadata written to {meta_path}")

    return final_report


def main() -> int:
    parser = argparse.ArgumentParser(description="J9.1-4B Known Small Translation Runner")
    parser.add_argument("--dx-mm", type=float, default=0.5, help="Target X displacement in mm (default: 0.5)")
    parser.add_argument("--hold-s", type=float, default=8.0, help="Action hold duration in s (default: 8.0)")
    parser.add_argument("--quiet-s", type=float, default=5.0, help="Pre/post quiet hold duration in s (default: 5.0)")
    parser.add_argument("--output-dir", default="/home/lyapunov/bone_drill/results/j9_1/translation_characterization")
    args = parser.parse_args()

    try:
        report = run_x_translation_experiment(
            target_dx_mm=args.dx_mm,
            hold_s=args.hold_s,
            quiet_s=args.quiet_s,
            output_dir=Path(args.output_dir),
        )
        return 0 if report.get("status") == "PASS" else 1
    except Exception as e:
        print(f"[FATAL ERROR] {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
