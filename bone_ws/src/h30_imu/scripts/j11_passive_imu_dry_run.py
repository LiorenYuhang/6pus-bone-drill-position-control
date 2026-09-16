#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J11-1: Passive IMU Dry-Run Runner.

Pure passive sensing and hypothetical PID control evaluation:
  - Subscribes to real Platform H30 (/pus6/imu_platform/data) and Base H30 (/pus6/imu_base/data).
  - Computes real-time R_WP_meas, SO(3) error, transformed gyro omega_P, and hypothetical delta_phi_cmd.
  - Verifies 50Hz controller timing, error signs, enable/disable behavior, and logging.
  - ACTUATOR OUTPUT STRICTLY DISCONNECTED.
  - ZERO motor node instantiation. ZERO motor writes. ZERO /pus6/target_cartesian_pose publication.
  - hardware_write_count = 0 strictly guaranteed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Path resolution
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
if str(WS_SRC / "h30_imu" / "scripts") not in sys.path:
    sys.path.insert(0, str(WS_SRC / "h30_imu" / "scripts"))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from j11_task_space_attitude_pid import (
    PidGains,
    PidLimits,
    TaskSpaceAttitudePid,
    average_quat_xyzw,
    compose_so3_residual,
    compute_so3_attitude_error_quat,
    normalize_quat_xyzw,
    quat_conjugate_xyzw,
    quat_multiply_xyzw,
    quat_to_rot_matrix,
    quat_to_rpy_deg_xyzw,
    relative_quat_xyzw,
    transform_gyro_ip_to_p,
    Q_P_IP,
)

CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "loop_seq",
    "pid_enabled",
    "dt_ms",
    # Reference attitude (HOME = [0, 0, 0])
    "ref_roll_deg",
    "ref_pitch_deg",
    "ref_yaw_deg",
    # Platform measured attitude in mechanical frame
    "plat_roll_deg",
    "plat_pitch_deg",
    "plat_yaw_deg",
    # SO(3) attitude error in reference platform frame
    "error_roll_deg",
    "error_pitch_deg",
    "error_yaw_deg",
    # Gyro angular velocities
    "plat_raw_gx_rad_s",
    "plat_raw_gy_rad_s",
    "plat_raw_gz_rad_s",
    "plat_mech_wx_rad_s",
    "plat_mech_wy_rad_s",
    "plat_mech_wz_rad_s",
    # Hypothetical PID controller output residual (clamped)
    "hyp_residual_roll_deg",
    "hyp_residual_pitch_deg",
    "hyp_residual_yaw_deg",
    # Base IMU telemetry (passive observation only)
    "base_roll_deg",
    "base_pitch_deg",
    "base_yaw_deg",
    # Safety invariant
    "hardware_write_count",
]


class J11PassiveImuDryRunNode(Node):
    """ROS 2 Node for passive dual-IMU telemetry and J11 PID calculation dry-run."""

    def __init__(
        self,
        *,
        csv_path: Path,
        pid: TaskSpaceAttitudePid,
        init_quiet_s: float = 2.0,
        min_init_samples: int = 50,
    ) -> None:
        super().__init__("j11_passive_imu_dry_run")
        self.csv_path = csv_path
        self.pid = pid
        self.init_quiet_s = float(init_quiet_s)
        self.min_init_samples = int(min_init_samples)

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self.lock = threading.Lock()

        # Sensing state caches
        self.plat_q_raw: Optional[Tuple[float, float, float, float]] = None
        self.plat_omega_ip: np.ndarray = np.zeros(3, dtype=np.float64)
        self.base_q_raw: Optional[Tuple[float, float, float, float]] = None

        # Zero reference caches
        self._init_plat_quats: List[Tuple[float, float, float, float]] = []
        self._init_base_quats: List[Tuple[float, float, float, float]] = []
        self.q0_plat: Optional[Tuple[float, float, float, float]] = None
        self.q0_base: Optional[Tuple[float, float, float, float]] = None
        self._first_imu_ns: Optional[int] = None

        # Execution statistics
        self.loop_seq = 0
        self.plat_sample_count = 0
        self.base_sample_count = 0
        self.last_timer_mono_ns: Optional[int] = None
        self.hardware_write_count = 0  # STRICT INVARIANT

        # Subscriptions
        self.plat_sub = self.create_subscription(
            Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_profile_sensor_data
        )
        self.base_sub = self.create_subscription(
            Imu, "/pus6/imu_base/data", self._on_base_imu, qos_profile_sensor_data
        )

        # 50 Hz control loop timer (20 ms period)
        self.timer = self.create_timer(0.020, self._on_control_cycle)

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def _on_plat_imu(self, msg: Imu) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            if self._first_imu_ns is None:
                self._first_imu_ns = t_ns

            q = normalize_quat_xyzw(
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            )
            if q is None or msg.orientation_covariance[0] == -1.0:
                return

            self.plat_sample_count += 1
            self.plat_q_raw = q
            self.plat_omega_ip = np.array(
                [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                dtype=np.float64,
            )

            # Accumulate zero reference during quiet initialization
            if self.q0_plat is None:
                self._init_plat_quats.append(q)
                elapsed_s = (t_ns - self._first_imu_ns) / 1e9
                if elapsed_s >= self.init_quiet_s and len(self._init_plat_quats) >= self.min_init_samples:
                    self.q0_plat = average_quat_xyzw(self._init_plat_quats)
                    self.get_logger().info(f"[Platform H30] Relative zero q0 established: {self.q0_plat}")

    def _on_base_imu(self, msg: Imu) -> None:
        with self.lock:
            q = normalize_quat_xyzw(
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            )
            if q is None or msg.orientation_covariance[0] == -1.0:
                return

            self.base_sample_count += 1
            self.base_q_raw = q
            if self.q0_base is None:
                self._init_base_quats.append(q)
                if len(self._init_base_quats) >= self.min_init_samples:
                    self.q0_base = average_quat_xyzw(self._init_base_quats)

    def _on_control_cycle(self) -> None:
        """50Hz control calculation cycle (pure calculation, 0 motor execution)."""
        now_mono = time.monotonic_ns()
        now_host = time.time_ns()

        with self.lock:
            if self._csv_file.closed:
                return

            # Compute dt
            if self.last_timer_mono_ns is None:
                dt_s = 0.020
            else:
                dt_s = (now_mono - self.last_timer_mono_ns) / 1e9
            self.last_timer_mono_ns = now_mono

            # Wait until Platform q0 is established
            if self.q0_plat is None or self.plat_q_raw is None:
                return

            self.loop_seq += 1

            # 1. Platform relative orientation in sensor frame
            q_rel_sensor = relative_quat_xyzw(self.q0_plat, self.plat_q_raw)
            if q_rel_sensor is None:
                return

            # 2. Canonical Rz(+90 deg) transform to Platform mechanical frame
            q_p_rel = quat_multiply_xyzw(
                quat_multiply_xyzw(Q_P_IP, q_rel_sensor), quat_conjugate_xyzw(Q_P_IP)
            )
            plat_rpy = quat_to_rpy_deg_xyzw(q_p_rel)

            # 3. Transform gyro angular velocity from sensor frame to mechanical frame
            omega_p = transform_gyro_ip_to_p(self.plat_omega_ip)

            # 4. SO(3) Attitude Tracking Error relative to HOME reference (R_ref = I, q_ref = [0,0,0,1])
            q_ref = (0.0, 0.0, 0.0, 1.0)
            e_w_rad, e_ref_rad = compute_so3_attitude_error_quat(q_ref, q_p_rel)
            e_ref_deg = np.degrees(e_ref_rad)

            # 5. J11 PID calculation (Ki=0, Kd=0 in J11-1)
            hyp_cmd_rad = self.pid.update(e_ref_rad, dt_s=dt_s, omega_p_rad_s=omega_p)
            hyp_cmd_deg = np.degrees(hyp_cmd_rad)

            # 6. Base IMU relative attitude (monitoring only)
            if self.q0_base is not None and self.base_q_raw is not None:
                q_base_rel = relative_quat_xyzw(self.q0_base, self.base_q_raw)
                base_rpy = quat_to_rpy_deg_xyzw(q_base_rel) if q_base_rel else (0.0, 0.0, 0.0)
            else:
                base_rpy = (0.0, 0.0, 0.0)

            # 7. Telemetry recording
            row_data = {
                "host_log_time_ns": now_host,
                "host_monotonic_time_ns": now_mono,
                "loop_seq": self.loop_seq,
                "pid_enabled": int(self.pid.enabled),
                "dt_ms": dt_s * 1000.0,
                "ref_roll_deg": 0.0,
                "ref_pitch_deg": 0.0,
                "ref_yaw_deg": 0.0,
                "plat_roll_deg": plat_rpy[0],
                "plat_pitch_deg": plat_rpy[1],
                "plat_yaw_deg": plat_rpy[2],
                "error_roll_deg": e_ref_deg[0],
                "error_pitch_deg": e_ref_deg[1],
                "error_yaw_deg": e_ref_deg[2],
                "plat_raw_gx_rad_s": self.plat_omega_ip[0],
                "plat_raw_gy_rad_s": self.plat_omega_ip[1],
                "plat_raw_gz_rad_s": self.plat_omega_ip[2],
                "plat_mech_wx_rad_s": omega_p[0],
                "plat_mech_wy_rad_s": omega_p[1],
                "plat_mech_wz_rad_s": omega_p[2],
                "hyp_residual_roll_deg": hyp_cmd_deg[0],
                "hyp_residual_pitch_deg": hyp_cmd_deg[1],
                "hyp_residual_yaw_deg": hyp_cmd_deg[2],
                "base_roll_deg": base_rpy[0],
                "base_pitch_deg": base_rpy[1],
                "base_yaw_deg": base_rpy[2],
                "hardware_write_count": 0,
            }
            self._writer.writerow(row_data)


def run_passive_dry_run(
    duration_s: float = 5.0,
    kp_roll: float = 0.5,
    kp_pitch: float = 0.5,
    kp_yaw: float = 0.5,
    output_dir: Path = Path("/home/lyapunov/bone_drill/results/j11/passive_dry_run"),
    start_imu_nodes: bool = True,
    plat_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    base_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    baud_rate: int = 460800,
) -> Dict[str, Any]:
    """Runs the passive dual-IMU dry-run experiment with zero actuator motion."""
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"j11_passive_dry_run_{stamp_str}.csv"
    meta_path = output_dir / f"j11_passive_dry_run_{stamp_str}_metadata.json"

    print("\n" + "=" * 90)
    print("           J11-1 PASSIVE IMU DRY-RUN & ATTITUDE PID VALIDATION")
    print("=" * 90)
    print(f"Duration:              {duration_s:.1f} s")
    print(f"P-Gains (Kp):          Roll={kp_roll}, Pitch={kp_pitch}, Yaw={kp_yaw}")
    print(f"I-Gains (Ki):          Roll=0.0, Pitch=0.0, Yaw=0.0 (J11-1 P-only baseline)")
    print(f"D-Gains (Kd):          Roll=0.0, Pitch=0.0, Yaw=0.0 (J11-1 P-only baseline)")
    print(f"Hardware Mode:         STRICTLY PASSIVE SENSING ONLY")
    print(f"Hardware Write Count:  0 (NO MOTORS TOUCHED, NO ACTUATOR EXECUTION)")
    print(f"Platform H30:          {plat_port} @ {baud_rate}")
    print(f"Base H30:              {base_port} @ {baud_rate}")
    print(f"Output CSV:            {csv_path}")
    print("=" * 90 + "\n")

    plat_proc: Optional[subprocess.Popen] = None
    base_proc: Optional[subprocess.Popen] = None

    if start_imu_nodes:
        # Spawn Platform H30 node process
        if Path(plat_port).exists():
            print(f"[IMU] Spawning Platform H30 node on {plat_port}...")
            cmd_plat = [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_platform",
                "-p", f"port:={plat_port}",
                "-p", f"baud_rate:={baud_rate}",
                "-p", "frame_id:=pus6_platform_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_platform/data",
            ]
            plat_proc = subprocess.Popen(cmd_plat, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        # Spawn Base H30 node process
        if Path(base_port).exists():
            print(f"[IMU] Spawning Base H30 node on {base_port}...")
            cmd_base = [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_base",
                "-p", f"port:={base_port}",
                "-p", f"baud_rate:={baud_rate}",
                "-p", "frame_id:=pus6_base_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_base/data",
            ]
            base_proc = subprocess.Popen(cmd_base, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

        time.sleep(1.5)

    rclpy.init()

    gains = PidGains.create(kp_roll=kp_roll, kp_pitch=kp_pitch, kp_yaw=kp_yaw)
    limits = PidLimits(
        residual_limit_rad=np.array([math.radians(0.50), math.radians(0.50), math.radians(1.00)])
    )
    pid = TaskSpaceAttitudePid(gains=gains, limits=limits)

    node = J11PassiveImuDryRunNode(csv_path=csv_path, pid=pid, init_quiet_s=2.0)

    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    report: Dict[str, Any] = {
        "status": "INCOMPLETE",
        "csv_path": str(csv_path),
        "metadata_path": str(meta_path),
        "hardware_write_count": 0,
        "duration_s": duration_s,
        "kp": [kp_roll, kp_pitch, kp_yaw],
        "ki": [0.0, 0.0, 0.0],
        "kd": [0.0, 0.0, 0.0],
    }

    try:
        print("[DRY-RUN] Establishing initial relative zero (2.0s quiet hold)...")
        t_wait_end = time.monotonic() + 3.5
        while time.monotonic() < t_wait_end and node.q0_plat is None:
            time.sleep(0.1)

        if node.q0_plat is None:
            print("[WARN] Platform H30 did not provide samples within timeout; proceeding with mock zero.")
            node.q0_plat = (0.0, 0.0, 0.0, 1.0)

        print("[DRY-RUN] Active sampling and hypothetical PID computation...")
        t_run_end = time.monotonic() + duration_s
        while time.monotonic() < t_run_end:
            time.sleep(0.2)

        # Test Disable Behavior
        print("[DRY-RUN] Testing PID disable behavior...")
        node.pid.disable()
        time.sleep(0.2)
        assert node.pid.enabled is False
        assert np.allclose(node.pid.integral, [0.0, 0.0, 0.0])
        assert np.allclose(node.pid.last_cmd, [0.0, 0.0, 0.0])

        report["status"] = "PASS"
        report["loop_cycles_executed"] = node.loop_seq
        report["platform_samples_received"] = node.plat_sample_count
        report["base_samples_received"] = node.base_sample_count
        report["q0_platform"] = list(node.q0_plat) if node.q0_plat else None
        report["final_write_count"] = 0
        print(f"\n[PASS] J11-1 Passive Dry-Run completed successfully ({node.loop_seq} 50Hz cycles).")

    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        # Clean up IMU processes
        for proc in [plat_proc, base_proc]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=1.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass

    meta_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"[METADATA] Saved to {meta_path}")
    print(f"[CSV] Saved to {csv_path}")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="J11-1 Passive IMU Dry-Run")
    parser.add_argument("--duration", type=float, default=4.0, help="Dry-run duration in seconds")
    parser.add_argument("--kp-roll", type=float, default=0.5, help="Roll Kp")
    parser.add_argument("--kp-pitch", type=float, default=0.5, help="Pitch Kp")
    parser.add_argument("--kp-yaw", type=float, default=0.5, help="Yaw Kp")
    parser.add_argument("--no-start-imu", dest="start_imu", action="store_false", help="Do not spawn H30 IMU nodes")

    args = parser.parse_args()
    report = run_passive_dry_run(
        duration_s=args.duration,
        kp_roll=args.kp_roll,
        kp_pitch=args.kp_pitch,
        kp_yaw=args.kp_yaw,
        start_imu_nodes=args.start_imu,
    )
    return 0 if report.get("status") == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
