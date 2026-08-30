#!/usr/bin/env python3
"""J9-E1C-1: Final Platform IMU Orientation Calibration, Run 1.

Estimates ONLY R_P_IP using the canonical frozen J8 ROS2 runtime,
R2 hardened IMU data-quality gates, real H30 0x51 timestamps, and
real FK query timestamps with local SO(3) angular velocity derivation.

Hard Rule:
  Exactly ONE real-hardware calibration run is authorized.
  Never automatically retry or launch a second run.
  Stop and report after Run 1 regardless of PASS/FAIL.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent
WS_ROOT = REPO_ROOT / "bone_ws"
for pkg in ("pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"):
    p = WS_ROOT / "src" / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
scripts_dir = WS_ROOT / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))
imu_scripts_dir = WS_ROOT / "src" / "h30_imu" / "scripts"
if imu_scripts_dir.exists() and str(imu_scripts_dir) not in sys.path:
    sys.path.insert(0, str(imu_scripts_dir))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from h30_imu.msg import TimedImu
import j9_e1a_platform_orientation_calibrator as e1a
from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from run_j7_servo_home_park import run_servo_home_park
from run_j8_c_hardware_readonly_audit import run_j8_c_audit
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, Int32MultiArray

MOTOR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PLATFORM_IMU_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00"
BAUD_MOTOR = 921600
BAUD_IMU = 460800


@dataclass
class CalibrationSegment:
    name: str
    role: str  # "stationary", "calibration", "validation", "home_return"
    duration_s: float
    amp_rpy_deg: tuple[float, float, float]
    freq_rpy_hz: tuple[float, float, float]
    phase_rpy_deg: tuple[float, float, float]

    def target_rpy_rad(self, t_s: float) -> tuple[float, float, float]:
        r = math.radians(self.amp_rpy_deg[0]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[0] * t_s + math.radians(self.phase_rpy_deg[0])) if self.amp_rpy_deg[0] > 0 else 0.0
        p = math.radians(self.amp_rpy_deg[1]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[1] * t_s + math.radians(self.phase_rpy_deg[1])) if self.amp_rpy_deg[1] > 0 else 0.0
        y = math.radians(self.amp_rpy_deg[2]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[2] * t_s + math.radians(self.phase_rpy_deg[2])) if self.amp_rpy_deg[2] > 0 else 0.0
        return r, p, y


def generate_e1c1_segments() -> list[CalibrationSegment]:
    return [
        CalibrationSegment("stationary_preroll", "stationary", 4.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        CalibrationSegment("roll_excitation", "calibration", 8.0, (2.0, 0.0, 0.0), (0.25, 0.0, 0.0), (0.0, 0.0, 0.0)),
        CalibrationSegment("pitch_excitation", "calibration", 8.0, (0.0, 2.0, 0.0), (0.0, 0.25, 0.0), (0.0, 0.0, 0.0)),
        CalibrationSegment("yaw_excitation", "calibration", 8.0, (0.0, 0.0, 3.0), (0.0, 0.0, 0.20), (0.0, 0.0, 0.0)),
        CalibrationSegment("combined_fit_excitation", "calibration", 12.0, (1.5, 1.5, 2.0), (0.23, 0.31, 0.17), (0.0, 45.0, 90.0)),
        CalibrationSegment("withheld_validation_excitation", "validation", 12.0, (1.2, 1.2, 1.8), (0.19, 0.27, 0.13), (30.0, 60.0, 120.0)),
        CalibrationSegment("home_return", "home_return", 4.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ]


class CsvQueueWriter:
    def __init__(self, out_dir: Path, capacity: int = 131072) -> None:
        self.queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue(maxsize=capacity)
        self.out_dir = out_dir
        self.capacity = capacity
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}
        self.written: dict[str, int] = {}
        self.overflow: dict[str, int] = {}
        self.errors = 0
        self.thread = threading.Thread(target=self._run, name="e1c-csv-writer", daemon=True)
        self.thread.start()

    def _open(self, stream: str, row: dict[str, Any]) -> csv.DictWriter:
        if stream not in self.writers:
            f = (self.out_dir / f"{stream}.csv").open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            writer.writeheader()
            self.files[stream] = f
            self.writers[stream] = writer
            self.written[stream] = 0
            self.overflow[stream] = 0
        return self.writers[stream]

    def enqueue(self, stream: str, row: dict[str, Any]) -> bool:
        try:
            self.queue.put_nowait((stream, row))
            return True
        except queue.Full:
            self.overflow[stream] = self.overflow.get(stream, 0) + 1
            return False

    def _run(self) -> None:
        while True:
            item = self.queue.get()
            if item is None:
                self.queue.task_done()
                break
            stream, row = item
            try:
                self._open(stream, row).writerow(row)
                self.written[stream] = self.written.get(stream, 0) + 1
            except Exception:
                self.errors += 1
            self.queue.task_done()

    def stop(self) -> None:
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self.thread.join(timeout=5.0)
        for f in self.files.values():
            f.flush()
            f.close()


class CanonicalObserverNode(Node):
    """Passive recording node subscribing to the canonical J8 ROS2 graph."""

    def __init__(self, out_dir: Path, queue_capacity: int = 131072) -> None:
        super().__init__("j9_e1c1_observer")
        self.writer = CsvQueueWriter(out_dir, queue_capacity)
        
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4096,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(TimedImu, "/pus6/imu_platform/timed", self._on_imu, reliable_qos)
        self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_feedback, sensor_qos)
        self.create_subscription(FkStateStamped, "/pus6/fk/state", self._on_fk, sensor_qos)
        self.create_subscription(PoseStamped, "/pus6/target_cartesian_pose", self._on_target_pose, reliable_qos)
        self.create_subscription(Float64MultiArray, "/pus6/desired_joint_deltas", self._on_ik, reliable_qos)
        self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd, sensor_qos)

    @staticmethod
    def _stamp_ns(msg: Any) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_imu(self, msg: TimedImu) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("imu", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "publisher_sequence": int(msg.publisher_sequence),
            "tid": int(msg.tid),
            "sample_timestamp_us": int(msg.sample_timestamp_us),
            "has_sample_timestamp": int(msg.has_sample_timestamp),
            "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
            "gx_rad_s": float(msg.imu.angular_velocity.x),
            "gy_rad_s": float(msg.imu.angular_velocity.y),
            "gz_rad_s": float(msg.imu.angular_velocity.z),
            "ax_m_s2": float(msg.imu.linear_acceleration.x),
            "ay_m_s2": float(msg.imu.linear_acceleration.y),
            "az_m_s2": float(msg.imu.linear_acceleration.z),
        })

    def _on_feedback(self, msg: TimedJointFeedback) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("joint_feedback", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "snapshot_start_monotonic_ns": int(msg.snapshot_start_monotonic_ns),
            "snapshot_end_monotonic_ns": int(msg.snapshot_end_monotonic_ns),
            "uart_cycle_sequence": int(msg.uart_cycle_sequence),
            "raw_steps": json.dumps([int(v) for v in msg.raw_steps]),
            "currents_ma": json.dumps([int(v) for v in msg.currents_ma]),
            "fault_codes": json.dumps([int(v) for v in msg.fault_codes]),
        })

    def _on_fk(self, msg: FkStateStamped) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("fk", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "query_monotonic_timestamp_ns": int(msg.query_monotonic_timestamp_ns),
            "converged": int(msg.converged),
            "residual_norm_meters": float(msg.residual_norm_meters),
            "compute_time_us": float(msg.compute_time_us),
            "pos_x_m": float(msg.pose.position.x),
            "pos_y_m": float(msg.pose.position.y),
            "pos_z_m": float(msg.pose.position.z),
            "qx": float(msg.pose.orientation.x),
            "qy": float(msg.pose.orientation.y),
            "qz": float(msg.pose.orientation.z),
            "qw": float(msg.pose.orientation.w),
            "quat_xyzw": json.dumps([float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)]),
        })

    def _on_target_pose(self, msg: PoseStamped) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("target_pose", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "pos_x_m": float(msg.pose.position.x),
            "pos_y_m": float(msg.pose.position.y),
            "pos_z_m": float(msg.pose.position.z),
            "quat_xyzw": json.dumps([float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)]),
        })

    def _on_ik(self, msg: Float64MultiArray) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("ik_deltas", {
            "callback_monotonic_ns": t_cb,
            "deltas_m": json.dumps([float(v) for v in msg.data]),
        })

    def _on_cmd(self, msg: Int32MultiArray) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("command_steps", {
            "callback_monotonic_ns": t_cb,
            "steps": json.dumps([int(v) for v in msg.data]),
        })

    def close(self) -> None:
        self.writer.stop()
        (self.writer.out_dir / "recorder_summary.json").write_text(
            json.dumps({
                "capacity": int(self.writer.capacity),
                "written": {k: int(v) for k, v in self.writer.written.items()},
                "overflow": {k: int(v) for k, v in self.writer.overflow.items()},
                "errors": int(self.writer.errors),
            }, indent=2),
            encoding="utf-8",
        )

def shutdown_calibration_pipeline(
    target_pub_node: Node,
    executor: SingleThreadedExecutor,
    spin_thread: threading.Thread,
    observer_node: CanonicalObserverNode,
    kinematics_node: Node,
    control_node: Node,
    motor_driver_node: Node,
    fk_node: Node,
) -> None:
    target_pub_node.destroy_node()
    try:
        executor.shutdown()
    finally:
        if spin_thread.is_alive():
            spin_thread.join(timeout=5.0)

    try:
        observer_node.close()
    finally:
        kinematics_node.destroy_node()
        control_node.destroy_node()
        motor_driver_node.destroy_node()
        fk_node.destroy_node()
        observer_node.destroy_node()



def start_h30_imu(source_path: Path) -> subprocess.Popen[str]:
    cmd = [
        "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
        "-r", "__ns:=/pus6/imu_platform",
        "-p", f"port:={PLATFORM_IMU_PORT}",
        "-p", f"baud_rate:={BAUD_IMU}",
        "-p", "frame_id:=pus6_platform_imu_link",
        "-p", "world_frame:=world",
        "-p", "imu_topic:=/pus6/imu_platform/data",
        "-p", "euler_topic:=/pus6/imu_platform/euler",
        "-p", "temperature_topic:=/pus6/imu_platform/temperature",
        "-p", "timed_topic:=/pus6/imu_platform/timed",
        "-p", "diagnostics_topic:=/pus6/imu_platform/diagnostics",
        "-p", "publish_euler:=true",
        "-p", "publish_tf:=false",
        "-p", "timed_qos_profile:=calibration",
        "-p", "timed_qos_depth:=4096",
        "-p", "timed_qos_reliability:=reliable",
        "-p", "source_capture_enabled:=true",
        "-p", f"source_capture_path:={source_path}",
        "-p", "source_capture_queue_capacity:=131072",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def stop_proc(proc: subprocess.Popen[str] | None, timeout_s: float = 10.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                proc.kill()


def load_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def sequence_stats(values: list[int], expected_step: int = 1) -> dict[str, Any]:
    if len(values) < 2:
        return {"count": len(values), "missing": 0, "duplicates": 0, "out_of_order": 0, "max_delta": None, "p95_delta": None}
    d = np.diff(np.asarray(values, dtype=np.int64))
    return {
        "count": int(len(values)),
        "missing": int(np.sum(np.maximum(0, d - expected_step) // max(1, expected_step))),
        "duplicates": int(np.sum(d == 0)),
        "out_of_order": int(np.sum(d < 0)),
        "max_delta": int(np.max(d)),
        "p95_delta": float(np.percentile(d, 95.0)),
    }


def solve_orientation_calibration(
    imu_rows: list[dict[str, Any]],
    fk_rows: list[dict[str, Any]],
    fit_windows: list[tuple[int, int]],
    val_windows: list[tuple[int, int]],
) -> dict[str, Any]:
    """Solve R_P_IP using affine clock mapping, SO(3) polynomial angular velocity, and Wahba SVD."""
    
    # 1. Parse IMU data
    valid_imu = [r for r in imu_rows if int(r.get("has_sample_timestamp", 1)) == 1]
    if len(valid_imu) < 50:
        raise ValueError("Insufficient valid IMU rows with 0x51 timestamps")

    dev_us = np.asarray([int(r["sample_timestamp_us"]) for r in valid_imu], dtype=float)
    host_ns = np.asarray([int(float(r["host_monotonic_arrival_time_ns"])) for r in valid_imu], dtype=float)
    dev_unwrapped, unwrap_diag = e1a.unwrap_u32_us(dev_us)
    affine_fit = e1a.fit_affine_device_to_host(dev_unwrapped, host_ns)

    imu_stamps_ns = e1a.map_sample_timestamp_to_monotonic_ns(dev_unwrapped, affine_fit["a"], affine_fit["b_us"])
    gyro_data = np.asarray([[float(r["gx_rad_s"]), float(r["gy_rad_s"]), float(r["gz_rad_s"])] for r in valid_imu], dtype=float)

    # 2. Parse FK data
    fk_stamps_ns = np.asarray([int(float(r["query_monotonic_timestamp_ns"])) for r in fk_rows], dtype=np.int64)
    fk_quats = np.asarray([[float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])] for r in fk_rows], dtype=float)
    rotations_b_p = Rotation.from_quat(fk_quats)

    # 3. Derive platform angular velocity from FK using true query timestamps
    omega_p, valid_fk_omega, omega_diag = e1a.local_body_angular_velocity_from_rotations(
        fk_stamps_ns, rotations_b_p, window_s=0.12, poly_degree=2
    )

    # 4. Interpolate gyro to valid FK timestamps
    valid_mask = valid_fk_omega
    fk_times_valid = fk_stamps_ns[valid_mask]
    omega_p_valid = omega_p[valid_mask]

    gyro_interp, gyro_valid = e1a.interpolate_gyro(imu_stamps_ns, gyro_data, fk_times_valid)
    joint_valid = gyro_valid & np.all(np.isfinite(omega_p_valid), axis=1) & np.all(np.isfinite(gyro_interp), axis=1)

    t_eval = fk_times_valid[joint_valid]
    omega_p_eval = omega_p_valid[joint_valid]
    gyro_eval = gyro_interp[joint_valid]

    # 5. Split into Fit and Validation datasets based on monotonic time windows
    fit_mask = np.zeros(len(t_eval), dtype=bool)
    for t_s, t_e in fit_windows:
        fit_mask |= ((t_eval >= t_s) & (t_eval <= t_e))

    val_mask = np.zeros(len(t_eval), dtype=bool)
    for t_s, t_e in val_windows:
        val_mask |= ((t_eval >= t_s) & (t_eval <= t_e))

    # Calibration dataset
    t_fit = t_eval[fit_mask]
    w_p_fit = omega_p_eval[fit_mask]
    w_imu_fit = gyro_eval[fit_mask]

    # Validation dataset
    t_val = t_eval[val_mask]
    w_p_val = omega_p_eval[val_mask]
    w_imu_val = gyro_eval[val_mask]

    if len(w_p_fit) < 30:
        raise ValueError(f"Insufficient calibration samples ({len(w_p_fit)})")
    if len(w_p_val) < 30:
        raise ValueError(f"Insufficient validation samples ({len(w_p_val)})")

    # 6. Observability Matrix and Singular Values: M = sum(w_imu_i * w_imu_i^T)
    M_obs = w_imu_fit.T @ w_imu_fit
    s_vals = np.linalg.svd(M_obs, compute_uv=False)
    cond_num = float(s_vals[0] / max(s_vals[-1], 1e-12))

    # 7. Wahba / SVD / Kabsch solver for R_P_IP:
    # Model: omega_P = R_P_IP * omega_IP
    # Cost: min sum || omega_P_i - R * omega_IP_i ||^2
    # Cross-covariance: H = sum(omega_P_i * omega_IP_i^T)
    H_cross = w_p_fit.T @ w_imu_fit
    U, S, Vt = np.linalg.svd(H_cross)
    # Ensure proper rotation det(R) = +1
    d = np.linalg.det(U @ Vt)
    R_fit_mat = U @ np.diag([1.0, 1.0, d]) @ Vt
    rot_fit = Rotation.from_matrix(R_fit_mat)
    q_fit_xyzw = rot_fit.as_quat().tolist()
    euler_fit_deg = rot_fit.as_euler("xyz", degrees=True).tolist()

    # 8. Calibration residuals
    w_p_pred_fit = (R_fit_mat @ w_imu_fit.T).T
    res_fit = w_p_pred_fit - w_p_fit
    res_fit_norm = np.linalg.norm(res_fit, axis=1)

    # 9. Out-of-Sample Withheld-Validation residuals
    w_p_pred_val = (R_fit_mat @ w_imu_val.T).T
    res_val = w_p_pred_val - w_p_val
    res_val_norm = np.linalg.norm(res_val, axis=1)

    # 10. Refit on Validation dataset to check rotation stability
    H_val_cross = w_p_val.T @ w_imu_val
    U_v, _, Vt_v = np.linalg.svd(H_val_cross)
    d_v = np.linalg.det(U_v @ Vt_v)
    R_val_mat = U_v @ np.diag([1.0, 1.0, d_v]) @ Vt_v
    rot_val = Rotation.from_matrix(R_val_mat)
    
    # Delta rotation between Fit and Validation models
    delta_rot = rot_fit.inv() * rot_val
    delta_rot_angle_deg = float(math.degrees(delta_rot.magnitude()))

    return {
        "candidate_R_P_IP_matrix": R_fit_mat.tolist(),
        "candidate_quaternion_xyzw": q_fit_xyzw,
        "candidate_euler_xyz_deg": euler_fit_deg,
        "affine_clock_sync": affine_fit,
        "observability": {
            "singular_values": s_vals.tolist(),
            "condition_number": cond_num,
            "full_rank": bool(s_vals[-1] > 1e-4),
        },
        "fit_metrics": {
            "sample_count": int(len(w_p_fit)),
            "rms_err_rad_s": float(np.sqrt(np.mean(res_fit_norm**2))),
            "p95_err_rad_s": float(np.percentile(res_fit_norm, 95.0)),
            "max_err_rad_s": float(np.max(res_fit_norm)),
            "per_axis_rms_rad_s": [float(np.sqrt(np.mean(res_fit[:, a]**2))) for a in range(3)],
        },
        "withheld_validation_metrics": {
            "sample_count": int(len(w_p_val)),
            "rms_err_rad_s": float(np.sqrt(np.mean(res_val_norm**2))),
            "p95_err_rad_s": float(np.percentile(res_val_norm, 95.0)),
            "max_err_rad_s": float(np.max(res_val_norm)),
            "per_axis_rms_rad_s": [float(np.sqrt(np.mean(res_val[:, a]**2))) for a in range(3)],
            "validation_refit_euler_deg": rot_val.as_euler("xyz", degrees=True).tolist(),
            "validation_refit_rotation_delta_deg": delta_rot_angle_deg,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="J9-E1C-1: Final Platform IMU Orientation Calibration, Run 1")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "reports" / "j9_e1c1_platform_orientation_calibration_20260828")
    parser.add_argument("--real-hardware", action="store_true", help="Authorize execution on physical hardware")
    parser.add_argument("--confirm-motion", action="store_true", help="Confirm physical platform motion")
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    args = parser.parse_args()

    if not (args.real_hardware and args.confirm_motion):
        print("Dry run mode: --real-hardware and --confirm-motion required for physical calibration.", file=sys.stderr)
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    report_file = args.out_dir / "j9_e1c1_calibration_report.md"
    raw_file = args.out_dir / "j9_e1c1_raw.json"

    print("\n" + "=" * 80)
    print("      6PUS J9-E1C-1: FINAL PLATFORM IMU ORIENTATION CALIBRATION (RUN 1)")
    print("=" * 80)
    print(f"Motor Port:     {MOTOR_PORT} ({BAUD_MOTOR} baud)")
    print(f"Platform IMU:   {PLATFORM_IMU_PORT} ({BAUD_IMU} baud)")
    print(f"FixedLag Delay: {args.fixed_lag_delay_ms:.1f} ms")
    print(f"Output Dir:     {args.out_dir}")
    print("=" * 80 + "\n")

    results: dict[str, Any] = {
        "stage": "J9-E1C-1",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "real_hardware_runs_attempted": 1,
        "run_status": "UNKNOWN",
        "preflight_audit": None,
        "home_result": None,
        "data_quality_evaluation": None,
        "calibration_solution": None,
        "park_result": None,
        "final_readonly_audit": None,
        "error": None,
    }

    # Step 1: Preflight Read-Only Audit
    print("\n--- [Step 1] Preflight READ_ONLY Hardware Audit (4.0s) ---")
    audit_pre = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
    results["preflight_audit"] = audit_pre
    if audit_pre.get("status") != "PASS":
        print(f"[PREFLIGHT FAIL] Initial hardware state is not PASS: {audit_pre}", file=sys.stderr)
        results["run_status"] = "PREFLIGHT_FAIL"
        raw_file.write_text(json.dumps(results, indent=2))
        return 1

    time.sleep(0.5)

    # Step 2: Canonical Transition to OPERATING_HOME
    print("\n--- [Step 2] Canonical J8 Transition to HOME ---")
    home_res = run_servo_home_park("home", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
    results["home_result"] = asdict(home_res)
    if not home_res.passed:
        print("[HOME FAIL] Transition to HOME failed!", file=sys.stderr)
        results["run_status"] = "HOME_FAIL"
        time.sleep(0.5)
        park_res = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
        results["park_result"] = asdict(park_res)
        results["final_readonly_audit"] = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        raw_file.write_text(json.dumps(results, indent=2))
        return 1

    time.sleep(0.5)

    # Step 3: Canonical ROS 2 Execution (EXACTLY ONE RUN)
    print("\n--- [Step 3] Executing Calibration Trajectory on Canonical J8 Path (Single Run) ---")
    source_csv = args.out_dir / "source.csv"
    h30_proc = start_h30_imu(source_csv)

    rclpy.init()
    geo_cfg = load_geometry_config()
    z0 = float(geo_cfg.nominal_height_m)

    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=MOTOR_PORT,
        baudrate=BAUD_MOTOR,
        timeout=0.015,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=args.fixed_lag_delay_ms, query_rate_hz=200.0)
    observer_node = CanonicalObserverNode(args.out_dir)

    executor = SingleThreadedExecutor()
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(observer_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(3.0)  # IMU & FK Discovery pre-roll

    # Target publisher
    target_pub_node = Node("e1c1_target_publisher")
    reliable_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    target_pub = target_pub_node.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", reliable_qos)

    def publish_target(x: float, y: float, z: float, r: float, p: float, y_rad: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = target_pub_node.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        cy = math.cos(y_rad * 0.5)
        sy = math.sin(y_rad * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)
        target_pub.publish(msg)

    segments_log = []
    run_error = None
    try:
        segments = generate_e1c1_segments()
        for seg in segments:
            seg_start_ns = time.monotonic_ns()
            print(f"[CALIB MOTION] Executing segment '{seg.name}' ({seg.duration_s:.1f}s, role={seg.role})...", flush=True)
            n_steps = int(round(seg.duration_s * 50.0))
            for k in range(n_steps):
                t_step_s = k * 0.020
                r, p, y = seg.target_rpy_rad(t_step_s)
                publish_target(0.0, 0.0, z0, r, p, y)
                time.sleep(0.020)
            seg_end_ns = time.monotonic_ns()
            segments_log.append({
                "name": seg.name,
                "role": seg.role,
                "duration_s": seg.duration_s,
                "start_monotonic_ns": seg_start_ns,
                "end_monotonic_ns": seg_end_ns,
            })

        # Post-motion hold: 2.0s hold at HOME to drain FK buffer
        print("[CALIB MOTION] Holding at HOME for 2.0s post-motion to drain FK buffer...", flush=True)
        for _ in range(100):
            publish_target(0.0, 0.0, z0, 0.0, 0.0, 0.0)
            time.sleep(0.020)

        results["run_status"] = "COMPLETED"
    except Exception as exc:
        run_error = f"{type(exc).__name__}: {exc}"
        results["run_status"] = "ERROR"
        results["error"] = run_error
        print(f"[RUN ERROR] {run_error}", file=sys.stderr)
    finally:
        print("\n[TEARDOWN] Shutting down canonical ROS2 pipeline...")
        shutdown_calibration_pipeline(
            target_pub_node,
            executor,
            spin_thread,
            observer_node,
            kinematics_node,
            control_node,
            motor_driver_node,
            fk_node,
        )
        rclpy.shutdown()
        stop_proc(h30_proc)
        print("[TEARDOWN OK] ROS2 nodes stopped.")

    time.sleep(0.5)

    # Step 4: Canonical J8 Return to PARK
    print("\n--- [Step 4] Canonical J8 Transition to PARK ---")
    park_res = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
    results["park_result"] = asdict(park_res)

    time.sleep(0.5)

    # Step 5: Final READ_ONLY Hardware Audit
    print("\n--- [Step 5] Final READ_ONLY Hardware Audit (4.0s) ---")
    audit_post = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
    results["final_readonly_audit"] = audit_post

    # Step 6: Data Quality Gates & Calibration Optimization
    print("\n--- [Step 6] Data Quality Evaluation & Calibration Solution ---")
    source_rows = load_csv(args.out_dir / "source.csv")
    imu_rows = load_csv(args.out_dir / "imu.csv")
    fb_rows = load_csv(args.out_dir / "joint_feedback.csv")
    fk_rows = load_csv(args.out_dir / "fk.csv")

    # Data Quality Gate Evaluation
    gate_cfg = e1a.DataQualityGateConfig(
        require_publisher_sequence=True,
        max_missing_publisher_sequences=0,
        max_duplicate_publisher_sequences=0,
        max_out_of_order_publisher_sequences=0,
        max_missing_sample_timestamps=0,
        max_sample_timestamp_resets=0,
        min_platform_rows=500,
    )
    dq_eval = e1a.evaluate_platform_data_quality(imu_rows, gate_cfg)
    results["data_quality_evaluation"] = dq_eval

    # Identify Fit and Validation Windows
    fit_windows = [(s["start_monotonic_ns"], s["end_monotonic_ns"]) for s in segments_log if s["role"] == "calibration"]
    val_windows = [(s["start_monotonic_ns"], s["end_monotonic_ns"]) for s in segments_log if s["role"] == "validation"]

    try:
        calib_solution = solve_orientation_calibration(imu_rows, fk_rows, fit_windows, val_windows)
        results["calibration_solution"] = calib_solution
        print("\n" + "=" * 80)
        print("                 J9-E1C-1 CALIBRATION CANDIDATE SOLUTION")
        print("=" * 80)
        print(f"Candidate Euler XYZ (deg):   {calib_solution['candidate_euler_xyz_deg']}")
        print(f"Candidate Quaternion (xyzw): {calib_solution['candidate_quaternion_xyzw']}")
        print(f"Condition Number:            {calib_solution['observability']['condition_number']:.2f}")
        print(f"Fit RMS Error (rad/s):       {calib_solution['fit_metrics']['rms_err_rad_s']:.4f}")
        print(f"Validation RMS Err (rad/s):  {calib_solution['withheld_validation_metrics']['rms_err_rad_s']:.4f}")
        print(f"Validation Refit Delta:      {calib_solution['withheld_validation_metrics']['validation_refit_rotation_delta_deg']:.3f} deg")
        print("=" * 80 + "\n")
    except Exception as exc:
        calib_err = f"Calibration solve failed: {type(exc).__name__}: {exc}"
        results["error"] = calib_err
        print(f"[CALIB SOLVE ERROR] {calib_err}", file=sys.stderr)

    raw_file.write_text(json.dumps(results, indent=2))
    print(f"[EVIDENCE SAVED] Raw JSON: {raw_file}")

    # Generate Markdown Report
    sol = results.get("calibration_solution")
    report_md = f"""# J9-E1C-1 Final Platform IMU Orientation Calibration (Run 1) Report

**Execution Timestamp**: {time.strftime("%Y-%m-%d %H:%M:%S")}  
**Robot Extrinsics Status**: `UNCALIBRATED_UNSET` (Candidate evaluation only, NOT frozen)  
**Execution Mode**: Canonical J8 ROS2 Node Graph + Passive J9 Observer  
**Hard Limit**: Exactly ONE real-hardware calibration run attempted.

---

## 1. Hardware State & Safety Summary

- **Preflight Audit Status**: `{audit_pre.get("status")}` (`{audit_pre.get("startup_classification")}`)
- **HOME Transition**: `{"PASS" if home_res.passed else "FAIL"}` ({home_res.total_cycles} cycles)
- **Calibration Motion**: `{results["run_status"]}` (Error: `{results.get("error")}`)
- **PARK Transition**: `{"PASS" if park_res.passed else "FAIL"}` ({park_res.total_cycles} cycles)
- **Final READ_ONLY Audit**: `{audit_post.get("status")}` (`{audit_post.get("startup_classification")}`)
- **Final Positions**: `{audit_post.get("initial_positions_steps")}`

---

## 2. IMU & Telemetry Data Quality Gates

- **Data Quality Status**: `{dq_eval.get("status")}`
- **Source-Side Capture Total Rows**: `{len(source_rows)}`
- **Subscribed IMU Rows**: `{len(imu_rows)}`
- **Publisher Sequence Gaps**: `{dq_eval["publisher_sequence"]["missing"]}`
- **0x51 Sample Timestamp Gaps**: `{dq_eval["sample_timestamp_0x51"]["missing"]}`
- **0x51 Resets / Nonmonotonicity**: `{dq_eval["sample_timestamp_unwrap"]}`
- **TimedJointFeedback Rows**: `{len(fb_rows)}`
- **FK State Reconstruction Rows**: `{len(fk_rows)}`

---

## 3. Orientation Calibration Candidate Results (`R_P_IP`)

"""
    if sol:
        report_md += f"""### Candidate Extrinsic Rotation Matrix $R_P^{{IP}}$
```json
{json.dumps(sol["candidate_R_P_IP_matrix"], indent=2)}
```

### Parametric Representations
- **Quaternion $(x, y, z, w)$**: `{sol["candidate_quaternion_xyzw"]}`
- **Euler XYZ (Roll, Pitch, Yaw in degrees)**: `{sol["candidate_euler_xyz_deg"]}`

### Observability & Clock Synchronization
- **Clock Drift Rate $a$**: `{sol["affine_clock_sync"]["a"]:.8f}` ({sol["affine_clock_sync"]["ppm_equivalent"]:.2f} ppm)
- **Clock Offset $b$**: `{sol["affine_clock_sync"]["b_us"]:.2f} us` (Fit RMS: `{sol["affine_clock_sync"]["residual_rms_us"]:.2f} us`)
- **Observability Singular Values**: `{sol["observability"]["singular_values"]}`
- **Observability Condition Number**: `{sol["observability"]["condition_number"]:.2f}` (Full rank: `{sol["observability"]["full_rank"]}`)

### Residuals & Withheld-Validation
- **Fit Dataset Sample Count**: `{sol["fit_metrics"]["sample_count"]}`
- **Fit RMS Residual**: `{sol["fit_metrics"]["rms_err_rad_s"]:.4f} rad/s` (P95: `{sol["fit_metrics"]["p95_err_rad_s"]:.4f} rad/s`, Max: `{sol["fit_metrics"]["max_err_rad_s"]:.4f} rad/s`)
- **Fit Per-Axis RMS $(X, Y, Z)$**: `{sol["fit_metrics"]["per_axis_rms_rad_s"]}`
- **Withheld-Validation Sample Count**: `{sol["withheld_validation_metrics"]["sample_count"]}`
- **Withheld-Validation RMS Residual**: `{sol["withheld_validation_metrics"]["rms_err_rad_s"]:.4f} rad/s` (P95: `{sol["withheld_validation_metrics"]["p95_err_rad_s"]:.4f} rad/s`)
- **Withheld-Validation Per-Axis RMS $(X, Y, Z)$**: `{sol["withheld_validation_metrics"]["per_axis_rms_rad_s"]}`
- **Validation Refit Rotation Delta**: `{sol["withheld_validation_metrics"]["validation_refit_rotation_delta_deg"]:.4f} deg`

"""
    else:
        report_md += "Calibration solution could not be computed due to runtime error.\n"

    report_md += """
---

## 4. J9 Standing Status

- **Candidate Status**: Reported as candidate only; `R_P_IP` remains **`UNCALIBRATED_UNSET`**.
- **No hardware retries or Run 2 launched.**
- **No Base IMU or lever arm calibration performed.**
- **No fusion or control stabilization active.**
"""
    report_file.write_text(report_md)
    print(f"[REPORT WRITTEN] {report_file}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
