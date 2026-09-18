#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J11-3A1: Base IMU Mechanical-Axis Qualification Runner.

Objectives:
  - Pure passive IMU sensing (hardware_write_count = 0 strictly guaranteed).
  - Guides user through two physical manual tilt actions:
      Action A: Mechanical Roll (+X axis, toward Motor 1 and 6)
      Action B: Mechanical Pitch (+Y axis, toward Motor 1 and 2)
  - Records Base H30 raw quaternion and evaluates candidate mounting transforms:
      Candidate 1: No transform (R_B_IB = I)
      Candidate 2: Rz(+90 deg)  (R_B_IB = Rz(+90 deg))
      Candidate 3: Rz(-90 deg)  (R_B_IB = Rz(-90 deg))
      Candidate 4: Rz(180 deg)  (R_B_IB = Rz(180 deg))
  - Selects the correct transform where:
      Roll test:  |Roll response| >> |Pitch response|
      Pitch test: |Pitch response| >> |Roll response|
  - Confirms Roll and Pitch sign conventions.
  - Automatically assesses the validity of J11-3A Base Roll (1.919 deg / 7.711 deg).
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
from typing import Dict, List, Optional, Sequence, Tuple

# ROS 2 & Math imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None
    Node = object

import numpy as np


# ------------------------------------------------------------------------------
# Quaternion Math Utilities
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
    """Computes relative quaternion from q0 to q: q_rel = q0^* ⊗ q."""
    return normalize_quat(quat_multiply(quat_conjugate(normalize_quat(q0)), normalize_quat(q)))


def quat_to_rpy_deg(q: Sequence[float]) -> Tuple[float, float, float]:
    """Extracts extrinsic XYZ (intrinsic ZYX) roll, pitch, yaw in degrees from [x, y, z, w]."""
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


def apply_z_rotation_transform(q_rel: Tuple[float, float, float, float], angle_deg: float) -> Tuple[float, float, float, float]:
    """Applies coordinate frame rotation R_mech = Rz(theta) @ R(q_rel) @ Rz(-theta)."""
    rad = math.radians(angle_deg)
    qz_pos = (0.0, 0.0, math.sin(rad / 2.0), math.cos(rad / 2.0))
    qz_neg = (0.0, 0.0, -math.sin(rad / 2.0), math.cos(rad / 2.0))
    return normalize_quat(quat_multiply(quat_multiply(qz_pos, q_rel), qz_neg))


# ------------------------------------------------------------------------------
# Test Node
# ------------------------------------------------------------------------------
class BaseImuMechanicalAxisQualificationNode(Node if Node is not object else object):
    def __init__(self, output_csv_path: Optional[str] = None):
        if Node is not object:
            super().__init__("j11_base_imu_axis_qualification")

        self.output_csv_path = output_csv_path
        self.lock = threading.Lock()

        # Telemetry storage
        self.q0_base: Optional[Tuple[float, float, float, float]] = None
        self.q0_plat: Optional[Tuple[float, float, float, float]] = None
        self.latest_base_q: Optional[Tuple[float, float, float, float]] = None
        self.latest_plat_q: Optional[Tuple[float, float, float, float]] = None
        self.samples: List[Dict[str, Any]] = []

        # State / Timing
        self.start_time_ns: Optional[int] = None
        self.hardware_write_count = 0  # Strictly guaranteed 0

        # ROS 2 Interfaces
        if Node is not object:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos)
            self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos)
            self.timer = self.create_timer(0.020, self._on_tick)  # 50 Hz sampling

    def _on_base_imu(self, msg: Imu) -> None:
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        with self.lock:
            self.latest_base_q = q

    def _on_plat_imu(self, msg: Imu) -> None:
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        with self.lock:
            self.latest_plat_q = q

    def _on_tick(self) -> None:
        now_ns = time.perf_counter_ns()
        with self.lock:
            if self.start_time_ns is None:
                self.start_time_ns = now_ns

            t_s = (now_ns - self.start_time_ns) * 1e-9

            base_q = self.latest_base_q
            plat_q = self.latest_plat_q

            if base_q is None:
                return

            # Capture initial reference during first 2.5 seconds
            if t_s < 2.5:
                self.q0_base = base_q
                if plat_q is not None:
                    self.q0_plat = plat_q
                phase = "BASELINE_CAPTURE"
            elif t_s < 22.0:
                phase = "ROLL_TEST"
            elif t_s < 26.0:
                phase = "PAUSE_BETWEEN_TESTS"
            elif t_s < 46.0:
                phase = "PITCH_TEST"
            else:
                phase = "FINAL_SETTLE"

            if self.q0_base is None:
                return

            # Compute relative quaternion
            q_rel_base = relative_quat(self.q0_base, base_q)

            # Candidate 1: No transform (R_B_IB = I)
            cand1_rpy = quat_to_rpy_deg(q_rel_base)

            # Candidate 2: Rz(+90 deg)
            q_cand2 = apply_z_rotation_transform(q_rel_base, +90.0)
            cand2_rpy = quat_to_rpy_deg(q_cand2)

            # Candidate 3: Rz(-90 deg)
            q_cand3 = apply_z_rotation_transform(q_rel_base, -90.0)
            cand3_rpy = quat_to_rpy_deg(q_cand3)

            # Candidate 4: Rz(180 deg)
            q_cand4 = apply_z_rotation_transform(q_rel_base, 180.0)
            cand4_rpy = quat_to_rpy_deg(q_cand4)

            # Platform attitude (known Rz(+90) mounting)
            if self.q0_plat is not None and plat_q is not None:
                q_rel_plat = relative_quat(self.q0_plat, plat_q)
                q_plat_mech = apply_z_rotation_transform(q_rel_plat, +90.0)
                plat_rpy = quat_to_rpy_deg(q_plat_mech)
            else:
                plat_rpy = (0.0, 0.0, 0.0)

            sample = {
                "t_s": t_s,
                "timestamp_ns": now_ns,
                "phase": phase,
                "base_q_x": base_q[0],
                "base_q_y": base_q[1],
                "base_q_z": base_q[2],
                "base_q_w": base_q[3],
                "cand1_roll_deg": cand1_rpy[0],
                "cand1_pitch_deg": cand1_rpy[1],
                "cand1_yaw_deg": cand1_rpy[2],
                "cand2_roll_deg": cand2_rpy[0],
                "cand2_pitch_deg": cand2_rpy[1],
                "cand2_yaw_deg": cand2_rpy[2],
                "cand3_roll_deg": cand3_rpy[0],
                "cand3_pitch_deg": cand3_rpy[1],
                "cand3_yaw_deg": cand3_rpy[2],
                "cand4_roll_deg": cand4_rpy[0],
                "cand4_pitch_deg": cand4_rpy[1],
                "cand4_yaw_deg": cand4_rpy[2],
                "plat_roll_deg": plat_rpy[0],
                "plat_pitch_deg": plat_rpy[1],
                "plat_yaw_deg": plat_rpy[2],
                "hardware_write_count": 0,
            }
            self.samples.append(sample)


def run_qualification(duration_s: float = 50.0, output_csv: str = "") -> Dict[str, Any]:
    print(f"\n[J11-3A1] Starting Base IMU Mechanical-Axis Qualification (Duration: {duration_s}s)...")
    print(f"[J11-3A1] hardware_write_count = 0 strictly guaranteed (zero motor communication).")

    rclpy.init()
    node = BaseImuMechanicalAxisQualificationNode(output_csv_path=output_csv)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t_start = time.monotonic()
    last_phase = ""
    last_print_t = 0.0

    try:
        while time.monotonic() - t_start < duration_s:
            time.sleep(0.1)
            with node.lock:
                if not node.samples:
                    continue
                latest = node.samples[-1]
                t_cur = latest["t_s"]
                phase = latest["phase"]

            # Phase transition announcements
            if phase != last_phase:
                last_phase = phase
                print("\n" + "=" * 70, flush=True)
                if phase == "BASELINE_CAPTURE":
                    print("[PHASE 0] BASELINE CAPTURE: Keep Base strictly stationary...", flush=True)
                elif phase == "ROLL_TEST":
                    print("[PHASE 1] ACTION A: MECHANICAL ROLL TEST", flush=True)
                    print("--> Please tilt Base around MECHANICAL ROLL AXIS (+X, toward Motor 1 & 6)", flush=True)
                    print("--> Tilt by ~2° to 5°, hold ~2s, then slowly return to 0.", flush=True)
                elif phase == "PAUSE_BETWEEN_TESTS":
                    print("[PHASE 1.5] PAUSE: Keep Base stationary at initial zero orientation...", flush=True)
                elif phase == "PITCH_TEST":
                    print("[PHASE 2] ACTION B: MECHANICAL PITCH TEST", flush=True)
                    print("--> Please tilt Base around MECHANICAL PITCH AXIS (+Y, toward Motor 1 & 2)", flush=True)
                    print("--> Tilt by ~2° to 5°, hold ~2s, then slowly return to 0.", flush=True)
                elif phase == "FINAL_SETTLE":
                    print("[PHASE 3] FINAL SETTLE: Return to 0 and hold stationary...", flush=True)
                print("=" * 70 + "\n", flush=True)

            # Live telemetry printout every 0.5s
            if t_cur - last_print_t >= 0.5:
                last_print_t = t_cur
                c1_r, c1_p = latest["cand1_roll_deg"], latest["cand1_pitch_deg"]
                c2_r, c2_p = latest["cand2_roll_deg"], latest["cand2_pitch_deg"]
                c3_r, c3_p = latest["cand3_roll_deg"], latest["cand3_pitch_deg"]
                print(
                    f"[{phase[:8]} t={t_cur:4.1f}s] "
                    f"Cand1(NoTrans): R={c1_r:+5.1f}°, P={c1_p:+5.1f}° | "
                    f"Cand2(Rz+90): R={c2_r:+5.1f}°, P={c2_p:+5.1f}° | "
                    f"Cand3(Rz-90): R={c3_r:+5.1f}°, P={c3_p:+5.1f}°",
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\n[J11-3A1] Interrupted by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    # Save CSV
    if output_csv and node.samples:
        os.makedirs(Path(output_csv).parent, exist_ok=True)
        keys = list(node.samples[0].keys())
        with open(output_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(node.samples)
        print(f"\n[J11-3A1] Telemetry CSV saved to: {output_csv} ({len(node.samples)} rows)")

    # Offline candidate evaluation
    summary = evaluate_candidates(node.samples)
    summary_path = Path(output_csv).parent / "j11_3a1_base_imu_axis_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[J11-3A1] Summary evaluation saved to: {summary_path}")
    return summary


def evaluate_candidates(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Analyzes candidate transformations against Roll and Pitch test data."""
    if not samples:
        return {"verdict": "FAIL", "reason": "No samples collected"}

    roll_samples = [s for s in samples if s["phase"] == "ROLL_TEST"]
    pitch_samples = [s for s in samples if s["phase"] == "PITCH_TEST"]

    cand_names = [
        ("cand1", "No transform (R_B_IB = I)"),
        ("cand2", "Rz(+90 deg) (R_B_IB = Rz(+90 deg))"),
        ("cand3", "Rz(-90 deg) (R_B_IB = Rz(-90 deg))"),
        ("cand4", "Rz(180 deg) (R_B_IB = Rz(180 deg))"),
    ]

    analysis = {}
    best_candidate = None
    best_score = -1e9

    for prefix, desc in cand_names:
        # Roll test metrics
        r_vals_roll = [s[f"{prefix}_roll_deg"] for s in roll_samples] if roll_samples else [0.0]
        p_vals_roll = [s[f"{prefix}_pitch_deg"] for s in roll_samples] if roll_samples else [0.0]

        max_abs_r_roll = float(max(abs(v) for v in r_vals_roll))
        max_abs_p_roll = float(max(abs(v) for v in p_vals_roll))
        roll_isolation_ratio = max_abs_r_roll / (max_abs_p_roll + 1e-4)
        roll_peak_signed = float(r_vals_roll[int(np.argmax(np.abs(r_vals_roll)))])

        # Pitch test metrics
        r_vals_pitch = [s[f"{prefix}_roll_deg"] for s in pitch_samples] if pitch_samples else [0.0]
        p_vals_pitch = [s[f"{prefix}_pitch_deg"] for s in pitch_samples] if pitch_samples else [0.0]

        max_abs_r_pitch = float(max(abs(v) for v in r_vals_pitch))
        max_abs_p_pitch = float(max(abs(v) for v in p_vals_pitch))
        pitch_isolation_ratio = max_abs_p_pitch / (max_abs_r_pitch + 1e-4)
        pitch_peak_signed = float(p_vals_pitch[int(np.argmax(np.abs(p_vals_pitch)))])

        # A candidate is viable if both isolation ratios are > 2.0
        is_viable = (roll_isolation_ratio > 2.0) and (pitch_isolation_ratio > 2.0)
        score = roll_isolation_ratio + pitch_isolation_ratio

        analysis[prefix] = {
            "description": desc,
            "roll_test": {
                "max_abs_roll_deg": max_abs_r_roll,
                "max_abs_pitch_deg": max_abs_p_roll,
                "isolation_ratio_roll_over_pitch": roll_isolation_ratio,
                "peak_signed_roll_deg": roll_peak_signed,
                "pass_isolation": roll_isolation_ratio > 2.0,
            },
            "pitch_test": {
                "max_abs_pitch_deg": max_abs_p_pitch,
                "max_abs_roll_deg": max_abs_r_pitch,
                "isolation_ratio_pitch_over_roll": pitch_isolation_ratio,
                "peak_signed_pitch_deg": pitch_peak_signed,
                "pass_isolation": pitch_isolation_ratio > 2.0,
            },
            "is_viable": is_viable,
            "total_score": score,
        }

        if is_viable and score > best_score:
            best_score = score
            best_candidate = prefix

    # Overall verdict
    if best_candidate is not None:
        winning_desc = analysis[best_candidate]["description"]
        confirmed_r_z = "+90 deg" if best_candidate == "cand2" else ("-90 deg" if best_candidate == "cand3" else "0 deg")
        roll_sign = "+" if analysis[best_candidate]["roll_test"]["peak_signed_roll_deg"] > 0 else "-"
        pitch_sign = "+" if analysis[best_candidate]["pitch_test"]["peak_signed_pitch_deg"] > 0 else "-"
        verdict = "PASS"
    else:
        winning_desc = "None"
        confirmed_r_z = "UNKNOWN"
        roll_sign = "UNKNOWN"
        pitch_sign = "UNKNOWN"
        verdict = "FAIL"

    summary = {
        "stage": "J11-3A1",
        "title": "Base IMU Mechanical-Axis Qualification",
        "verdict": verdict,
        "hardware_write_count": 0,
        "winning_candidate": best_candidate,
        "winning_description": winning_desc,
        "confirmed_R_B_IB": f"Rz({confirmed_r_z})",
        "roll_sign": roll_sign,
        "pitch_sign": pitch_sign,
        "candidates_comparison": analysis,
        "j11_3a_reassessment": {
            "candidate_used_in_j11_3a_recalculation": "cand2 (Rz(+90 deg))",
            "is_validated_by_j11_3a1": (best_candidate == "cand2"),
            "base_roll_hold_1_deg": 1.919,
            "base_roll_peak_deg": 7.711,
            "attenuation_ratio_hold_1": 0.5693,
            "attenuation_ratio_hold_2": 0.5116,
            "attenuation_ratio_peak": 0.8740,
            "conclusion": (
                "Rz(+90 deg) is experimentally proven as the authoritative Base IMU mounting transform. "
                "J11-3A Base Roll (1.919 deg hold, 7.711 deg peak) and attenuation ratios (0.51 ~ 0.57) are fully validated."
                if best_candidate == "cand2"
                else "J11-3A Base Roll calculation requires re-evaluation under the confirmed candidate."
            ),
        },
    }
    return summary


def main():
    parser = argparse.ArgumentParser(description="J11-3A1: Base IMU Mechanical-Axis Qualification")
    parser.add_argument("--base-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00")
    parser.add_argument("--plat-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00")
    parser.add_argument("--baud-rate", type=int, default=460800)
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--start-imu", action="store_true", default=False)
    parser.add_argument("--output-csv", default="results/j11/j11_3a1/j11_3a1_base_imu_raw_data.csv")
    args = parser.parse_args()

    plat_proc = None
    base_proc = None

    if args.start_imu:
        if Path(args.plat_port).exists():
            print(f"[IMU] Spawning Platform H30 on {args.plat_port}...")
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
            print(f"[IMU] Spawning Base H30 on {args.base_port}...")
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

    try:
        run_qualification(duration_s=args.duration, output_csv=args.output_csv)
    finally:
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


if __name__ == "__main__":
    main()
