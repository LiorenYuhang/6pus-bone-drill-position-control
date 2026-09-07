#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J9.1-4G dedicated rotation measurement for Platform IMU lever-arm analysis.

This runner performs one known small attitude motion and records the acceleration
telemetry missing from prior J9.1 attitude artifacts. It uses the frozen J8 ROS2
motion chain and does not implement a new controller or modify frozen runners.

Default motion:
  HOME -> Roll/Pitch/Yaw +/-0.5 deg -> HOME

Recorded telemetry follows the J9.1 translation characterization schema:
Platform/Base H30 quaternion, gyro, raw sensor-frame acceleration,
gravity-removed world acceleration, joint command/feedback, and FK.
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
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

scripts_dir = WS_SRC / "h30_imu" / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R

import j9_1_z_translation_characterization as z4d
from j9_1_platform_imu_recorder import (
    average_quat_xyzw,
    normalize_quat_xyzw,
    quat_conjugate_xyzw,
    quat_multiply_xyzw,
    quat_to_rpy_deg_xyzw,
    relative_quat_xyzw,
)
from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode


CSV_FIELDS = list(z4d.CSV_FIELDS)
G_STANDARD = z4d.G_STANDARD
DEFAULT_OUTPUT_DIR = Path("/home/lyapunov/bone_drill/results/j9_1/rotation_lever_arm_characterization")
R_P_IP_DESCRIPTION = "R_P_IP = Rz(+90 deg); vector transform v_P = R_P_IP * v_IP"


def euler_deg_to_quat_xyzw(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    rot = R.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True)
    q = rot.as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def steps_for_attitude(kin: Analytical6PusKinematics, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[List[float], List[int]]:
    pose = [0.0, 0.0, kin.nominal_height, math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]
    q = kin.inverse_kinematics(pose)
    continuous = [1000.0 + (value - 0.05) * 40000.0 for value in q]
    return continuous, [round(value) for value in continuous]


def axis_angle_to_rpy(axis: str, angle_deg: float) -> Tuple[float, float, float]:
    if axis == "roll":
        return (float(angle_deg), 0.0, 0.0)
    if axis == "pitch":
        return (0.0, float(angle_deg), 0.0)
    if axis == "yaw":
        return (0.0, 0.0, float(angle_deg))
    raise ValueError(f"Unsupported rotation axis: {axis}")


class RotationLeverArmHarness(z4d.ZTranslationCharacterizationHarness):
    """J8-chain telemetry harness with a configurable attitude target."""

    def __init__(self, *, csv_path: Path, z0: float, axis: str, angle_deg: float) -> None:
        super().__init__(csv_path=csv_path, z0=z0, target_dz_m=0.0)
        self.axis = axis
        self.angle_deg = float(angle_deg)

    def set_rotation_stage(self, stage: str, roll_deg: float, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> None:
        with self.lock:
            self.current_stage = stage
            self.target_x_mm = 0.0
            self.target_y_mm = 0.0
            self.target_z_mm = self.z0 * 1000.0
            self.target_rpy_deg = [float(roll_deg), float(pitch_deg), float(yaw_deg)]

    def publish_rotation_target(self, roll_deg: float, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> None:
        qx, qy, qz, qw = euler_deg_to_quat_xyzw(roll_deg, pitch_deg, yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(self.z0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.target_pose_pub.publish(msg)


def _finite_values(rows: Sequence[Dict[str, str]], fields: Sequence[str]) -> bool:
    if not rows:
        return False
    try:
        for row in rows:
            for field in fields:
                value = float(row[field])
                if not math.isfinite(value):
                    return False
    except (KeyError, TypeError, ValueError):
        return False
    return True


def _stats(values: Sequence[float]) -> Dict[str, float]:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"mean": 0.0, "std": 0.0, "rms": 0.0, "min": 0.0, "max": 0.0}
    mean = statistics.fmean(vals)
    return {
        "mean": float(mean),
        "std": float(statistics.pstdev(vals)),
        "rms": float(math.sqrt(statistics.fmean([v * v for v in vals]))),
        "min": float(min(vals)),
        "max": float(max(vals)),
    }


def _dt_stats(stamps_ns: Sequence[int]) -> Dict[str, float]:
    unique = []
    last = None
    for stamp in stamps_ns:
        if stamp > 0 and stamp != last:
            unique.append(stamp)
            last = stamp
    dts_ms = [(unique[i] - unique[i - 1]) / 1e6 for i in range(1, len(unique))]
    if not dts_ms:
        return {"count": len(unique), "mean_ms": 0.0, "median_ms": 0.0, "p95_ms": 0.0, "p99_ms": 0.0, "max_ms": 0.0, "effective_rate_hz": 0.0}
    mean_ms = statistics.fmean(dts_ms)
    return {
        "count": len(unique),
        "mean_ms": float(mean_ms),
        "median_ms": float(statistics.median(dts_ms)),
        "p95_ms": float(np.percentile(dts_ms, 95.0)),
        "p99_ms": float(np.percentile(dts_ms, 99.0)),
        "max_ms": float(max(dts_ms)),
        "effective_rate_hz": float(1000.0 / mean_ms) if mean_ms > 0.0 else 0.0,
    }


def _monotonic_strict(rows: Sequence[Dict[str, str]], field: str) -> bool:
    stamps = [int(r[field]) for r in rows if int(r.get(field, "0")) > 0]
    return all(stamps[i] > stamps[i - 1] for i in range(1, len(stamps))) if len(stamps) >= 2 else False


def _axis_stats(rows: Sequence[Dict[str, str]], prefix: str) -> Dict[str, Dict[str, float]]:
    return {
        "x": _stats([float(r[f"{prefix}_x"]) for r in rows]),
        "y": _stats([float(r[f"{prefix}_y"]) for r in rows]),
        "z": _stats([float(r[f"{prefix}_z"]) for r in rows]),
    }


def _raw_axis_stats(rows: Sequence[Dict[str, str]], imu_prefix: str) -> Dict[str, Dict[str, float]]:
    return {
        "x": _stats([float(r[f"{imu_prefix}_ax_raw"]) for r in rows]),
        "y": _stats([float(r[f"{imu_prefix}_ay_raw"]) for r in rows]),
        "z": _stats([float(r[f"{imu_prefix}_az_raw"]) for r in rows]),
    }


def _relative_platform_rpy(rows: Sequence[Dict[str, str]]) -> Tuple[List[float], List[float], List[float]]:
    pre = [r for r in rows if r["stage"] == "pre_home"]
    init_rows = pre[-100:] if len(pre) >= 100 else pre
    quats = [(float(r["p_qx"]), float(r["p_qy"]), float(r["p_qz"]), float(r["p_qw"])) for r in init_rows]
    q0 = average_quat_xyzw(quats)
    if q0 is None:
        return [], [], []
    q_pi = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    rolls: List[float] = []
    pitches: List[float] = []
    yaws: List[float] = []
    for r in rows:
        q = normalize_quat_xyzw((float(r["p_qx"]), float(r["p_qy"]), float(r["p_qz"]), float(r["p_qw"])))
        if q is None:
            continue
        q_rel = relative_quat_xyzw(q0, q)
        if q_rel is None:
            continue
        q_p_rel = quat_multiply_xyzw(quat_multiply_xyzw(q_pi, q_rel), quat_conjugate_xyzw(q_pi))
        roll, pitch, yaw = quat_to_rpy_deg_xyzw(q_p_rel)
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)
    return rolls, pitches, yaws


def analyze_rotation_lever_arm_measurement(csv_path: Path, axis: str = "roll", angle_deg: float = 0.5) -> Dict[str, Any]:
    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
    if not rows:
        return {"status": "EMPTY"}

    action_stage = f"action_{axis}_{'pos' if angle_deg >= 0.0 else 'neg'}"
    pre_rows = [r for r in rows if r["stage"] == "pre_home"]
    action_rows = [r for r in rows if r["stage"] == action_stage]
    return_rows = [r for r in rows if r["stage"] == "return_home"]
    post_rows = [r for r in rows if r["stage"] == "post_home"]

    pre_ref = pre_rows[-100:] if len(pre_rows) >= 100 else pre_rows
    settled = action_rows[-100:] if len(action_rows) >= 100 else action_rows

    def mean_field(src: Sequence[Dict[str, str]], field: str) -> float:
        return statistics.fmean(float(r[field]) for r in src) if src else 0.0

    fk_delta = {
        "dx_mm": mean_field(settled, "fk_x_mm") - mean_field(pre_ref, "fk_x_mm"),
        "dy_mm": mean_field(settled, "fk_y_mm") - mean_field(pre_ref, "fk_y_mm"),
        "dz_mm": mean_field(settled, "fk_z_mm") - mean_field(pre_ref, "fk_z_mm"),
        "droll_deg": mean_field(settled, "fk_roll_deg") - mean_field(pre_ref, "fk_roll_deg"),
        "dpitch_deg": mean_field(settled, "fk_pitch_deg") - mean_field(pre_ref, "fk_pitch_deg"),
        "dyaw_deg": mean_field(settled, "fk_yaw_deg") - mean_field(pre_ref, "fk_yaw_deg"),
    }

    plat_rpy_rows = pre_rows + action_rows
    pr, pp, py = _relative_platform_rpy(plat_rpy_rows)
    p_action_gyro = {
        "gx_peak_deg_s": math.degrees(max(abs(float(r["p_gx_rad_s"])) for r in action_rows)) if action_rows else 0.0,
        "gy_peak_deg_s": math.degrees(max(abs(float(r["p_gy_rad_s"])) for r in action_rows)) if action_rows else 0.0,
        "gz_peak_deg_s": math.degrees(max(abs(float(r["p_gz_rad_s"])) for r in action_rows)) if action_rows else 0.0,
    }
    b_action_gyro = {
        "gx_peak_deg_s": math.degrees(max(abs(float(r["b_gx_rad_s"])) for r in action_rows)) if action_rows else 0.0,
        "gy_peak_deg_s": math.degrees(max(abs(float(r["b_gy_rad_s"])) for r in action_rows)) if action_rows else 0.0,
        "gz_peak_deg_s": math.degrees(max(abs(float(r["b_gz_rad_s"])) for r in action_rows)) if action_rows else 0.0,
    }

    p_stamps = [int(r["p_stamp_ns"]) for r in rows]
    b_stamps = [int(r["b_stamp_ns"]) for r in rows]
    host_stamps = [int(r["host_monotonic_time_ns"]) for r in rows]
    latest_base_offsets = [abs(int(r["p_stamp_ns"]) - int(r["b_stamp_ns"])) / 1e6 for r in rows if int(r["p_stamp_ns"]) > 0 and int(r["b_stamp_ns"]) > 0]
    faults = []
    for r in rows:
        raw = r.get("fault_codes", "")
        if raw and raw not in ("[0, 0, 0, 0, 0, 0]", "0"):
            faults.append(raw)

    last_action = settled[-1] if settled else rows[-1]
    completeness = {
        "platform_quaternion_complete": _finite_values(rows, ["p_qx", "p_qy", "p_qz", "p_qw"]),
        "platform_gyro_complete": _finite_values(rows, ["p_gx_rad_s", "p_gy_rad_s", "p_gz_rad_s"]),
        "platform_raw_accel_complete": _finite_values(rows, ["p_ax_raw", "p_ay_raw", "p_az_raw"]),
        "platform_gravity_removed_accel_complete": _finite_values(rows, ["p_aw_x", "p_aw_y", "p_aw_z"]),
        "joint_feedback_complete": _finite_values(rows, [f"joint_fb_{i}" for i in range(1, 7)]),
        "fk_complete": _finite_values(rows, ["fk_x_mm", "fk_y_mm", "fk_z_mm", "fk_roll_deg", "fk_pitch_deg", "fk_yaw_deg"]),
        "host_monotonic_strict": all(host_stamps[i] > host_stamps[i - 1] for i in range(1, len(host_stamps))),
        "platform_stamp_strict": _monotonic_strict(rows, "p_stamp_ns"),
    }

    primary_delta_key = {
        "roll": "droll_deg",
        "pitch": "dpitch_deg",
        "yaw": "dyaw_deg",
    }[axis]

    qualified = bool(
        action_rows
        and all(completeness.values())
        and abs(fk_delta[primary_delta_key]) > 0.05
        and not faults
    )

    return {
        "status": "PASS" if qualified else "CHECK",
        "axis": axis,
        "angle_deg": angle_deg,
        "action_stage": action_stage,
        "frame_contract": {
            "raw_accel_frame": "Platform H30 sensor frame I_P, fields p_ax_raw/p_ay_raw/p_az_raw, units m/s^2 specific force",
            "gyro_frame": "Platform H30 sensor frame I_P, fields p_gx_rad_s/p_gy_rad_s/p_gz_rad_s, units rad/s",
            "gravity_removed_accel_frame": "H30 quaternion-derived world/navigation representation from existing translation harness, fields p_aw_x/p_aw_y/p_aw_z, units m/s^2",
            "platform_transform": R_P_IP_DESCRIPTION,
            "quaternion_convention": "sensor_msgs/Imu orientation xyzw",
        },
        "sample_counts": {
            "total": len(rows),
            "pre_home": len(pre_rows),
            "action": len(action_rows),
            "return_home": len(return_rows),
            "post_home": len(post_rows),
        },
        "telemetry_completeness": completeness,
        "timing": {
            "platform_dt": _dt_stats(p_stamps),
            "base_dt": _dt_stats(b_stamps),
            "host_dt": _dt_stats(host_stamps),
            "online_latest_base_offset_ms": _stats(latest_base_offsets),
        },
        "joint_state": {
            "cmd_steps": [int(last_action[f"joint_cmd_{i}"]) for i in range(1, 7)],
            "fb_steps": [int(last_action[f"joint_fb_{i}"]) for i in range(1, 7)],
        },
        "fk_local_displacement": fk_delta,
        "platform_attitude_response": {
            "roll_deg": _stats(pr),
            "pitch_deg": _stats(pp),
            "yaw_deg": _stats(py),
        },
        "platform_gyro_peaks": p_action_gyro,
        "base_gyro_peaks": b_action_gyro,
        "platform_raw_accel": {
            "pre_home": _raw_axis_stats(pre_rows, "p"),
            "action": _raw_axis_stats(action_rows, "p"),
            "post_home": _raw_axis_stats(post_rows, "p"),
        },
        "platform_gravity_removed_accel": {
            "pre_home": _axis_stats(pre_rows, "p_aw"),
            "action": _axis_stats(action_rows, "p_aw"),
            "post_home": _axis_stats(post_rows, "p_aw"),
        },
        "base_monitor": {
            "raw_accel": {
                "pre_home": _raw_axis_stats(pre_rows, "b"),
                "action": _raw_axis_stats(action_rows, "b"),
                "post_home": _raw_axis_stats(post_rows, "b"),
            },
            "gravity_removed_accel": {
                "pre_home": _axis_stats(pre_rows, "b_aw"),
                "action": _axis_stats(action_rows, "b_aw"),
                "post_home": _axis_stats(post_rows, "b_aw"),
            },
            "gyro_peaks": b_action_gyro,
        },
        "fault_observations": sorted(set(faults)),
        "qualified_for_lever_arm_identification": qualified,
    }


def run_rotation_lever_arm_experiment(
    axis: str = "roll",
    angle_deg: float = 0.5,
    hold_s: float = 8.0,
    quiet_s: float = 5.0,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
    motor_port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    base_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_motor: int = 921600,
    baud_imu: int = 460800,
) -> Dict[str, Any]:
    if axis not in ("roll", "pitch", "yaw"):
        raise ValueError("J9.1-4G rotation measurement axis must be roll, pitch, or yaw")
    if angle_deg == 0.0:
        raise ValueError("J9.1-4G rotation measurement requires a nonzero angle")

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    dir_tag = (f"pos{int(round(abs(angle_deg) * 10)):02d}deg" if angle_deg >= 0.0 else f"neg{int(round(abs(angle_deg) * 10)):02d}deg")
    csv_path = output_dir / f"j9_1_rotation_lever_arm_{axis}_{dir_tag}_{stamp_str}.csv"
    meta_path = output_dir / f"j9_1_rotation_lever_arm_{axis}_{dir_tag}_{stamp_str}_metadata.json"

    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    roll_deg, pitch_deg, yaw_deg = axis_angle_to_rpy(axis, angle_deg)
    continuous, quantized = steps_for_attitude(kin, roll_deg, pitch_deg, yaw_deg)

    print("\n" + "=" * 90)
    print(f"      J9.1-4G DEDICATED ROTATION MEASUREMENT: {axis.capitalize()} {angle_deg:+.3f} deg")
    print("=" * 90)
    print(f"Output CSV: {csv_path}")
    print(f"Frozen IK nominal command: {quantized}")
    print(f"Motion: HOME -> {axis.capitalize()} {angle_deg:+.3f} deg -> HOME; X/Y/Z fixed at OPERATING_HOME")
    print("=" * 90 + "\n")

    print("[PREFLIGHT] Checking motor online status via non-intrusive 0x30 scan...")
    ok, home_steps, audit_msg = z4d.check_motor_read_only_preflight(port=motor_port, baudrate=baud_motor)
    print(f"[PREFLIGHT RESULT] {audit_msg}; readback={home_steps}")
    if not ok:
        raise RuntimeError(f"Motor preflight audit failed: {audit_msg}")
    if max(abs(int(s) - 1000) for s in home_steps) > 20:
        raise RuntimeError(f"Startup is not in OPERATING_HOME band for 4G rotation run: {home_steps}")

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
    harness = RotationLeverArmHarness(csv_path=csv_path, z0=z0, axis=axis, angle_deg=angle_deg)

    executor = MultiThreadedExecutor(num_threads=6)
    for node in (kinematics_node, control_node, motor_driver_node, fk_node, harness):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print(f"\n>>> [Stage 1: pre_home] Holding HOME for {quiet_s:.1f} s <<<")
        harness.set_rotation_stage("pre_home", 0.0, 0.0, 0.0)
        t_end = time.monotonic() + quiet_s
        while time.monotonic() < t_end:
            harness.publish_rotation_target(0.0, 0.0, 0.0)
            time.sleep(0.020)

        action_stage = f"action_{axis}_{'pos' if angle_deg >= 0.0 else 'neg'}"
        print(f"\n>>> [Stage 2: {action_stage}] {axis.capitalize()} {angle_deg:+.3f} deg for {hold_s:.1f} s <<<")
        harness.set_rotation_stage(action_stage, roll_deg, pitch_deg, yaw_deg)
        t_end = time.monotonic() + hold_s
        while time.monotonic() < t_end:
            harness.publish_rotation_target(roll_deg, pitch_deg, yaw_deg)
            time.sleep(0.020)
            if any(f != 0 for f in harness.latest_fault_codes):
                raise RuntimeError(f"Motor fault detected during action: {harness.latest_fault_codes}")

        print("\n>>> [Stage 3: return_home] Returning to HOME for 4.0 s <<<")
        harness.set_rotation_stage("return_home", 0.0, 0.0, 0.0)
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end:
            harness.publish_rotation_target(0.0, 0.0, 0.0)
            time.sleep(0.020)
            if any(f != 0 for f in harness.latest_fault_codes):
                raise RuntimeError(f"Motor fault detected during return: {harness.latest_fault_codes}")

        print(f"\n>>> [Stage 4: post_home] Holding HOME for {quiet_s:.1f} s <<<")
        harness.set_rotation_stage("post_home", 0.0, 0.0, 0.0)
        t_end = time.monotonic() + quiet_s
        while time.monotonic() < t_end:
            harness.publish_rotation_target(0.0, 0.0, 0.0)
            time.sleep(0.020)
    finally:
        print("[SHUTDOWN] Closing telemetry CSV and stopping ROS 2 nodes...")
        harness.close()
        try:
            executor.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        for proc, tag in ((base_proc, "Base H30"), (plat_proc, "Platform H30")):
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=2.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass

    metrics = analyze_rotation_lever_arm_measurement(csv_path, axis=axis, angle_deg=angle_deg)
    final_report = {
        "experiment_name": "J9.1-4G_Rotation_Lever_Arm_Characterization",
        "timestamp": stamp_str,
        "csv_path": str(csv_path),
        "axis": axis,
        "angle_deg": angle_deg,
        "z0_m": z0,
        "frozen_ik_nominal_continuous_steps": continuous,
        "frozen_ik_nominal_command_steps": quantized,
        "frame_contract": metrics.get("frame_contract", {}),
        "metrics": metrics,
        "hardware_write_count": harness.hardware_write_count,
        "final_fault_codes": harness.latest_fault_codes,
        "status": "PASS" if metrics.get("qualified_for_lever_arm_identification") else "CHECK",
        "notes": [
            "This run collects acceleration telemetry for future lever-arm identification.",
            "No r_P_IP estimate is accepted in J9.1-4G first-run acquisition.",
            "Base IMU is recorded only as background monitor; no a_platform - a_base physical subtraction is used.",
        ],
    }
    meta_path.write_text(json.dumps(final_report, indent=2), encoding="utf-8")
    print(f"[REPORT] Metadata written to {meta_path}")
    print(json.dumps({
        "status": final_report["status"],
        "hardware_write_count": final_report["hardware_write_count"],
        "csv_path": str(csv_path),
        "metadata_path": str(meta_path),
    }, indent=2))
    return final_report


def main() -> int:
    parser = argparse.ArgumentParser(description="J9.1-4G dedicated rotation measurement for Platform IMU lever-arm analysis")
    parser.add_argument("--axis", default="roll", choices=["roll", "pitch", "yaw"])
    parser.add_argument("--angle-deg", type=float, default=0.5)
    parser.add_argument("--hold-s", type=float, default=8.0)
    parser.add_argument("--quiet-s", type=float, default=5.0)
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    args = parser.parse_args()

    try:
        report = run_rotation_lever_arm_experiment(
            axis=args.axis,
            angle_deg=args.angle_deg,
            hold_s=args.hold_s,
            quiet_s=args.quiet_s,
            output_dir=Path(args.output_dir),
        )
        return 0 if report.get("status") == "PASS" else 1
    except Exception as exc:
        print(f"[FATAL ERROR] {exc}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
