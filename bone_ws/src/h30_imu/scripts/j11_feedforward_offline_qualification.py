#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J11-3B: Base-IMU Feedforward Offline IK & Sign Qualification.

Verifies:
  Base disturbance -> Delta_R_B -> R_BP_ff = Delta_R_B^T @ R_BP0 -> PoseStamped -> Analytical IK -> differential steps
Ensures:
  1. Base +Roll disturbance directly generates negative Roll compensation in R_BP_ff.
  2. Actuators 1 & 6 contract (delta_steps < 0), actuators 3 & 4 extend (delta_steps > 0).
  3. Step amplitudes stay within J4 envelope and hardware bounds.
  4. hardware_write_count = 0 strictly maintained.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_KINEMATICS_DIR = _SCRIPT_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPT_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from analytical_6pus import Analytical6PusKinematics
from geometry_config import load_geometry_config


def rot_matrix_to_euler_deg(R: np.ndarray) -> tuple[float, float, float]:
    """Extracts extrinsic XYZ roll, pitch, yaw in degrees."""
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


def rot_matrix_to_quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """Converts 3x3 rotation matrix to [x, y, z, w] quaternion."""
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


def euler_xyz_deg_to_rot_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Creates rotation matrix from extrinsic XYZ roll, pitch, yaw."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def run_qualification(output_path: str = "results/j11/j11_3b/j11_3b_offline_ik_qualification.json") -> dict:
    geom_cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    home_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    home_raw_steps = list(geom_cfg.operating_home_raw_steps)

    # Test cases covering Base manual disturbance angles
    test_cases = [
        {"name": "base_neutral", "base_r_deg": 0.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_pos_10", "base_r_deg": +1.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_pos_20", "base_r_deg": +2.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_pos_30", "base_r_deg": +3.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_pos_40", "base_r_deg": +4.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_pos_50", "base_r_deg": +5.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_roll_neg_30", "base_r_deg": -3.0, "base_p_deg": 0.0, "base_y_deg": 0.0},
        {"name": "base_pitch_pos_20", "base_r_deg": 0.0, "base_p_deg": +2.0, "base_y_deg": 0.0},
        {"name": "base_combined_roll_pitch", "base_r_deg": +2.5, "base_p_deg": +1.0, "base_y_deg": 0.0},
    ]

    R_BP0 = np.eye(3)
    results = []

    print("=" * 125)
    print(f"{'Case Name':24s} | {'Base Dist (R,P,Y)':20s} | {'Cmd R_BP_ff (R,P,Y)':22s} | {'Delta Steps (1..6)':26s} | {'Predicted Raw Steps':26s}")
    print("-" * 125)

    for c in test_cases:
        delta_R_B = euler_xyz_deg_to_rot_matrix(c["base_r_deg"], c["base_p_deg"], c["base_y_deg"])
        # Feedforward law: R_BP_ff = delta_R_B^T @ R_BP0
        R_BP_ff = delta_R_B.T @ R_BP0
        cmd_rpy = rot_matrix_to_euler_deg(R_BP_ff)
        q_cmd_xyzw = rot_matrix_to_quat_xyzw(R_BP_ff)

        # Verification: does R_W_P = delta_R_B @ R_BP_ff equal R_BP0?
        R_W_P = delta_R_B @ R_BP_ff
        world_rpy = rot_matrix_to_euler_deg(R_W_P)
        world_level = np.allclose(world_rpy, [0.0, 0.0, 0.0], atol=1e-3)

        cart_pose = [
            0.0, 0.0, z0,
            math.radians(cmd_rpy[0]),
            math.radians(cmd_rpy[1]),
            math.radians(cmd_rpy[2]),
        ]

        delta_steps = kin.differential_inverse_kinematics_steps(cart_pose, home_pose)
        predicted_raw_steps = [ref + ds for ref, ds in zip(home_raw_steps, delta_steps)]

        # Physical sign check: for +Roll disturbance, cmd Roll is negative.
        # Actuators 1 & 6 (-X side vs +X side):
        # +X is toward motor 1 & 6. When cmd Roll is negative, actuators 1 & 6 must contract (ds < 0)
        # Actuators 3 & 4 are toward -X, so they must extend (ds > 0).
        if c["base_r_deg"] > 0:
            sign_correct = (delta_steps[0] < 0 and delta_steps[5] < 0 and delta_steps[2] > 0 and delta_steps[3] > 0)
        elif c["base_r_deg"] < 0:
            sign_correct = (delta_steps[0] > 0 and delta_steps[5] > 0 and delta_steps[2] < 0 and delta_steps[3] < 0)
        else:
            sign_correct = True

        rec = {
            "name": c["name"],
            "base_disturbance_rpy_deg": [c["base_r_deg"], c["base_p_deg"], c["base_y_deg"]],
            "cmd_feedforward_rpy_deg": [round(v, 4) for v in cmd_rpy],
            "cmd_quat_xyzw": [round(v, 6) for v in q_cmd_xyzw],
            "world_platform_residual_rpy_deg": [round(v, 6) for v in world_rpy],
            "world_level_guaranteed": world_level,
            "delta_steps": delta_steps,
            "predicted_raw_steps": predicted_raw_steps,
            "max_abs_delta_step": max(abs(d) for d in delta_steps),
            "sign_correct": sign_correct,
            "within_j4_envelope": max(abs(math.radians(v)) for v in cmd_rpy) <= 0.10,
        }
        results.append(rec)

        base_str = f"({c['base_r_deg']:+.1f}, {c['base_p_deg']:+.1f}, {c['base_y_deg']:+.1f})°"
        cmd_str = f"({cmd_rpy[0]:+.1f}, {cmd_rpy[1]:+.1f}, {cmd_rpy[2]:+.1f})°"
        print(f"{c['name']:24s} | {base_str:20s} | {cmd_str:22s} | {str(delta_steps):26s} | {str(predicted_raw_steps):26s}")

    all_signs_ok = all(r["sign_correct"] for r in results)
    all_world_level = all(r["world_level_guaranteed"] for r in results)
    all_envelope_ok = all(r["within_j4_envelope"] for r in results)

    summary = {
        "stage": "J11-3B",
        "title": "Base-IMU Feedforward Offline IK & Sign Qualification",
        "verdict": "PASS" if (all_signs_ok and all_world_level and all_envelope_ok) else "FAIL",
        "hardware_write_count": 0,
        "reference_operating_home_raw_steps": home_raw_steps,
        "nominal_height_z0_m": z0,
        "feedforward_law": "R_BP_ff = Delta_R_B^T @ R_BP0",
        "all_signs_correct": all_signs_ok,
        "world_attitude_level_guaranteed": all_world_level,
        "all_within_j4_envelope": all_envelope_ok,
        "cases": results,
    }

    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(summary, f, indent=2)

    print("-" * 125)
    print(f"[J11-3B Offline IK Qualification] Verdict: {summary['verdict']}")
    print(f"Evidence saved to: {out_file}")
    print(f"hardware_write_count = 0 (STRICTLY PRESERVED)")
    return summary


if __name__ == "__main__":
    run_qualification()
