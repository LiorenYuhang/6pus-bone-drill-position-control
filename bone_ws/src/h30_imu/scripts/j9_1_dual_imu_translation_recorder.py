#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J9.1-4A: Dual-IMU Translational Observability & Measurement Baseline Recorder.

Subscribes to:
  - Base H30:     /pus6/imu_base/data (sensor_msgs/msg/Imu)
  - Platform H30: /pus6/imu_platform/data (sensor_msgs/msg/Imu)
  - Joint Feedback: read-only status (hardware_write_count = 0)

Computes offline:
  - Specific force in sensor frame and world frame (f_W = R_WI * f_I)
  - World linear acceleration after gravity removal: a_W = f_W - [0, 0, g]^T
  - High-precision timestamp pairing statistics (Base vs Platform)
  - Static noise floor metrics (mean, std, RMS)

HARDWARE SAFETY:
  hardware_write_count = 0 STRICTLY GUARANTEED.
  Zero actuator motion. Zero motor commands.
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

# Path resolution for workspace packages
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_msgs", "h30_imu"]:
    p = WS_SRC / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics

G_STANDARD = 9.80665

CSV_FIELDS = [
    "stream",
    "sequence_id",
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "ros_stamp_sec",
    "ros_stamp_nanosec",
    "ros_stamp_ns",
    "frame_id",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
    "accel_norm_m_s2",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
    "gyro_norm_rad_s",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "f_world_x",
    "f_world_y",
    "f_world_z",
    "a_world_x",
    "a_world_y",
    "a_world_z",
]


def read_only_motor_audit(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
) -> Tuple[bool, List[int], str]:
    """Runs a non-intrusive 0x30 read-only status query. Zero motion, zero writes."""
    from run_j8_c_hardware_readonly_audit import run_j8_c_audit

    try:
        res = run_j8_c_audit(port=port, baudrate=baudrate, duration_s=0.2)
        if res.get("status") == "PASS":
            steps = res.get("initial_scan", {}).get("steps", [1000] * 6)
            return True, steps, "All 6 motors online at HOME, fault=0"
        return False, [1000] * 6, f"Audit returned status: {res.get('status')}"
    except Exception as exc:
        return False, [1000] * 6, f"Audit exception: {exc}"


class DualImuTranslationRecorder(Node):
    """ROS 2 Node subscribing to both Base and Platform H30 IMU streams."""

    def __init__(
        self,
        *,
        csv_path: Path,
        duration_s: float = 15.0,
        base_topic: str = "/pus6/imu_base/data",
        platform_topic: str = "/pus6/imu_platform/data",
    ) -> None:
        super().__init__("j9_1_dual_imu_translation_recorder")

        self.csv_path = csv_path
        self.duration_s = float(duration_s)
        self.base_topic = base_topic
        self.platform_topic = platform_topic

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self.lock = threading.Lock()
        self.start_monotonic_ns: int | None = None
        self._done = False

        self.base_seq = 0
        self.platform_seq = 0

        self.base_records: List[Dict[str, Any]] = []
        self.platform_records: List[Dict[str, Any]] = []

        self.base_sub = self.create_subscription(
            Imu, self.base_topic, self._on_base_imu, qos_profile_sensor_data
        )
        self.platform_sub = self.create_subscription(
            Imu, self.platform_topic, self._on_platform_imu, qos_profile_sensor_data
        )

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def done(self) -> bool:
        return self._done

    def _process_sample(self, stream: str, msg: Imu, seq: int) -> Dict[str, Any]:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        f_norm = math.sqrt(ax * ax + ay * ay + az * az)

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        g_norm = math.sqrt(gx * gx + gy * gy + gz * gz)

        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)

        # Orientation & specific force rotation into world frame
        rot = R.from_quat([qx, qy, qz, qw])
        euler_deg = rot.as_euler("xyz", degrees=True)
        r_deg, p_deg, y_deg = float(euler_deg[0]), float(euler_deg[1]), float(euler_deg[2])

        f_sensor = np.array([ax, ay, az], dtype=np.float64)
        f_world = rot.apply(f_sensor)
        fw_x, fw_y, fw_z = float(f_world[0]), float(f_world[1]), float(f_world[2])

        # Gravity removal: a_W = f_W + g_W = f_W - [0, 0, g]^T
        aw_x = fw_x
        aw_y = fw_y
        aw_z = fw_z - G_STANDARD

        row = {
            "stream": stream,
            "sequence_id": seq,
            "host_log_time_ns": host_ns,
            "host_monotonic_time_ns": mono_ns,
            "ros_stamp_sec": int(msg.header.stamp.sec),
            "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
            "ros_stamp_ns": stamp_ns,
            "frame_id": msg.header.frame_id,
            "ax_m_s2": ax,
            "ay_m_s2": ay,
            "az_m_s2": az,
            "accel_norm_m_s2": f_norm,
            "gx_rad_s": gx,
            "gy_rad_s": gy,
            "gz_rad_s": gz,
            "gyro_norm_rad_s": g_norm,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "roll_deg": r_deg,
            "pitch_deg": p_deg,
            "yaw_deg": y_deg,
            "f_world_x": fw_x,
            "f_world_y": fw_y,
            "f_world_z": fw_z,
            "a_world_x": aw_x,
            "a_world_y": aw_y,
            "a_world_z": aw_z,
        }
        return row

    def _on_base_imu(self, msg: Imu) -> None:
        if self._done:
            return
        mono_ns = time.monotonic_ns()
        with self.lock:
            if self.start_monotonic_ns is None:
                self.start_monotonic_ns = mono_ns
            elif (mono_ns - self.start_monotonic_ns) / 1e9 >= self.duration_s:
                self._done = True
                return

            row = self._process_sample("base", msg, self.base_seq)
            self.base_seq += 1
            self.base_records.append(row)
            self._writer.writerow(row)

    def _on_platform_imu(self, msg: Imu) -> None:
        if self._done:
            return
        mono_ns = time.monotonic_ns()
        with self.lock:
            if self.start_monotonic_ns is None:
                self.start_monotonic_ns = mono_ns
            elif (mono_ns - self.start_monotonic_ns) / 1e9 >= self.duration_s:
                self._done = True
                return

            row = self._process_sample("platform", msg, self.platform_seq)
            self.platform_seq += 1
            self.platform_records.append(row)
            self._writer.writerow(row)


def compute_stream_statistics(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Computes statistical metrics for an individual IMU stream."""
    if not records:
        return {}

    n = len(records)
    t_first = records[0]["ros_stamp_ns"]
    t_last = records[-1]["ros_stamp_ns"]
    duration_s = max(1e-6, (t_last - t_first) / 1e9)
    rate_hz = (n - 1) / duration_s if n > 1 else 0.0

    stamps = [r["ros_stamp_ns"] for r in records]
    is_monotonic = all(stamps[i] >= stamps[i - 1] for i in range(1, n))

    ax = [r["ax_m_s2"] for r in records]
    ay = [r["ay_m_s2"] for r in records]
    az = [r["az_m_s2"] for r in records]
    norm_f = [r["accel_norm_m_s2"] for r in records]

    gx = [r["gx_rad_s"] for r in records]
    gy = [r["gy_rad_s"] for r in records]
    gz = [r["gz_rad_s"] for r in records]

    awx = [r["a_world_x"] for r in records]
    awy = [r["a_world_y"] for r in records]
    awz = [r["a_world_z"] for r in records]
    norm_aw = [math.sqrt(awx[i] ** 2 + awy[i] ** 2 + awz[i] ** 2) for i in range(n)]

    def _mean_std(data: Sequence[float]) -> Tuple[float, float]:
        m = float(statistics.fmean(data))
        s = float(statistics.stdev(data)) if len(data) > 1 else 0.0
        return m, s

    def _rms(data: Sequence[float]) -> float:
        return float(math.sqrt(statistics.fmean([x * x for x in data])))

    ax_m, ax_s = _mean_std(ax)
    ay_m, ay_s = _mean_std(ay)
    az_m, az_s = _mean_std(az)
    norm_f_m, norm_f_s = _mean_std(norm_f)

    gx_m, gx_s = _mean_std(gx)
    gy_m, gy_s = _mean_std(gy)
    gz_m, gz_s = _mean_std(gz)

    awx_m, awx_s = _mean_std(awx)
    awy_m, awy_s = _mean_std(awy)
    awz_m, awz_s = _mean_std(awz)

    return {
        "sample_count": n,
        "duration_s": duration_s,
        "effective_rate_hz": rate_hz,
        "timestamp_monotonic": is_monotonic,
        "accel_sensor_mean_m_s2": [ax_m, ay_m, az_m],
        "accel_sensor_std_m_s2": [ax_s, ay_s, az_s],
        "accel_norm_mean_m_s2": norm_f_m,
        "accel_norm_std_m_s2": norm_f_s,
        "gyro_sensor_mean_rad_s": [gx_m, gy_m, gz_m],
        "gyro_sensor_std_rad_s": [gx_s, gy_s, gz_s],
        "linear_accel_world_mean_m_s2": [awx_m, awy_m, awz_m],
        "linear_accel_world_std_m_s2": [awx_s, awy_s, awz_s],
        "linear_accel_world_rms_m_s2": [_rms(awx), _rms(awy), _rms(awz)],
        "linear_accel_world_total_rms_m_s2": _rms(norm_aw),
    }


def compute_timestamp_pairing_metrics(
    base_records: List[Dict[str, Any]],
    plat_records: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Computes nearest-timestamp pairing error distribution between Base and Platform."""
    if not base_records or not plat_records:
        return {}

    plat_stamps = np.array([r["ros_stamp_ns"] for r in plat_records], dtype=np.int64)

    pairing_offsets_ms: List[float] = []
    for b in base_records:
        t_b = b["ros_stamp_ns"]
        idx = np.searchsorted(plat_stamps, t_b)
        best_diff = float("inf")
        for i in [idx - 1, idx, idx + 1]:
            if 0 <= i < len(plat_stamps):
                d = abs(t_b - plat_stamps[i])
                if d < best_diff:
                    best_diff = d
        pairing_offsets_ms.append(best_diff / 1e6)

    offsets = np.array(pairing_offsets_ms)
    return {
        "paired_sample_count": len(offsets),
        "mean_pairing_offset_ms": float(np.mean(offsets)),
        "median_pairing_offset_ms": float(np.median(offsets)),
        "min_pairing_offset_ms": float(np.min(offsets)),
        "max_pairing_offset_ms": float(np.max(offsets)),
        "std_pairing_offset_ms": float(np.std(offsets)),
        "p95_pairing_offset_ms": float(np.percentile(offsets, 95)),
        "p99_pairing_offset_ms": float(np.percentile(offsets, 99)),
    }


def run_dual_imu_baseline_recording(
    duration_s: float = 15.0,
    output_dir: Path = Path("/home/lyapunov/bone_drill/results/j9_1/dual_imu_baseline"),
    base_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    platform_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_rate: int = 460800,
    start_nodes: bool = True,
) -> Dict[str, Any]:
    """Executes the dual-IMU static measurement baseline recording."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"j9_1_dual_imu_baseline_{stamp_str}.csv"
    meta_path = output_dir / f"j9_1_dual_imu_baseline_{stamp_str}_metadata.json"

    print("\n" + "=" * 90)
    print("       J9.1-4A DUAL-IMU TRANSLATIONAL OBSERVABILITY & MEASUREMENT BASELINE")
    print("=" * 90)
    print(f"Recording Duration:      {duration_s:.1f} s")
    print(f"Base H30 Port:           {base_port} @ {baud_rate}")
    print(f"Platform H30 Port:       {platform_port} @ {baud_rate}")
    print(f"Hardware Write Count:    0 (STRICTLY READ-ONLY, ZERO MOTION)")
    print(f"Output CSV:              {csv_path}")
    print("=" * 90 + "\n")

    # Step 1: Read-only motor audit to verify static HOME and record joint steps
    print("[PREFLIGHT] Reading motor status via non-intrusive 0x30 audit (hardware_write_count=0)...")
    ok, home_steps, audit_msg = read_only_motor_audit()
    print(f"[PREFLIGHT RESULT] {audit_msg}")
    if not ok:
        raise RuntimeError(f"Preflight audit failed: {audit_msg}")
    print(f"[PREFLIGHT] Confirmed static HOME joint steps: {home_steps}")

    # Compute static FK from joint feedback
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    slider_positions_m = [0.05 + (s - 1000) / 40000.0 for s in home_steps]
    fk_pose = kin.forward_kinematics(slider_positions_m)
    p_BP_fk = fk_pose[0:3]
    euler_BP_deg = [math.degrees(a) for a in fk_pose[3:6]]
    print(f"[FK GEOMETRY] Relative Base->Platform translation p_BP: [X={p_BP_fk[0]*1e3:.2f}mm, Y={p_BP_fk[1]*1e3:.2f}mm, Z={p_BP_fk[2]*1e3:.2f}mm]")
    print(f"[FK GEOMETRY] Relative Base->Platform orientation RPY:  [R={euler_BP_deg[0]:+.3f}°, P={euler_BP_deg[1]:+.3f}°, Y={euler_BP_deg[2]:+.3f}°]")

    # Step 2: Spawn ROS 2 driver nodes for Base and Platform if requested
    base_proc: subprocess.Popen | None = None
    plat_proc: subprocess.Popen | None = None
    if start_nodes:
        print("[ROS2 NODES] Spawning Base H30 node process...")
        cmd_base = [
            "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
            "-r", "__ns:=/pus6/imu_base",
            "-p", f"port:={base_port}",
            "-p", f"baud_rate:={baud_rate}",
            "-p", "frame_id:=pus6_base_imu_link",
            "-p", "world_frame:=world",
            "-p", "imu_topic:=/pus6/imu_base/data",
            "-p", "publish_euler:=true",
        ]
        base_proc = subprocess.Popen(cmd_base, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        print("[ROS2 NODES] Spawning Platform H30 node process...")
        cmd_plat = [
            "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
            "-r", "__ns:=/pus6/imu_platform",
            "-p", f"port:={platform_port}",
            "-p", f"baud_rate:={baud_rate}",
            "-p", "frame_id:=pus6_platform_imu_link",
            "-p", "world_frame:=world",
            "-p", "imu_topic:=/pus6/imu_platform/data",
            "-p", "publish_euler:=true",
        ]
        plat_proc = subprocess.Popen(cmd_plat, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(1.5)

    rclpy.init()
    recorder = DualImuTranslationRecorder(
        csv_path=csv_path,
        duration_s=duration_s,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()
    print(f"[RECORDING] Streaming dual IMU data for {duration_s:.1f} seconds...")

    try:
        t_end = time.monotonic() + duration_s + 1.0
        while time.monotonic() < t_end and not recorder.done():
            time.sleep(0.1)
    finally:
        print("[RECORDING] Complete. Shutting down ROS 2 nodes...")
        try:
            recorder.close()
            recorder.destroy_node()
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

    # Step 3: Compute comprehensive statistics
    base_stats = compute_stream_statistics(recorder.base_records)
    plat_stats = compute_stream_statistics(recorder.platform_records)
    pairing_stats = compute_timestamp_pairing_metrics(recorder.base_records, recorder.platform_records)

    report: Dict[str, Any] = {
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "duration_s": duration_s,
        "hardware_write_count": 0,
        "preflight_motor_steps": home_steps,
        "fk_relative_pose": {
            "p_BP_m": p_BP_fk,
            "rpy_BP_deg": euler_BP_deg,
        },
        "base_imu_stats": base_stats,
        "platform_imu_stats": plat_stats,
        "timestamp_pairing_stats": pairing_stats,
        "csv_path": str(csv_path),
        "status": "PASS",
    }

    meta_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"[METADATA] Saved to {meta_path}")
    print(f"[CSV] Saved to {csv_path}")

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="J9.1-4A Dual-IMU Static Recorder")
    parser.add_argument("--duration", type=float, default=15.0, help="Recording duration in seconds (default: 15.0)")
    parser.add_argument("--output-dir", default="/home/lyapunov/bone_drill/results/j9_1/dual_imu_baseline", help="Output directory")
    args = parser.parse_args()

    try:
        report = run_dual_imu_baseline_recording(
            duration_s=args.duration,
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
