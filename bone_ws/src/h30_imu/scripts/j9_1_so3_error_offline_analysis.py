#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J9.1-3B: SO(3) Attitude Error Formulation Offline Analysis & Historical Replay.

Pure software analysis tool:
  1. Verifies mathematical frame consistency of the candidate SO(3) error formulation.
  2. Replays historical J9.1-3A real hardware datasets (Roll, Pitch, Yaw).
  3. Evaluates frozen IK continuous and quantized integer steps.
  4. Analyzes synthetic small multi-axis combined cases.
  5. Provides analytical answers to architectural questions Q1-Q5.

DO NOT EXECUTE REAL HARDWARE.
DO NOT MODIFY J8 OR FROZEN RUNNERS.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence, Set, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

# Workspace package path setup
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
if str(WS_SRC / "pus6_kinematics") not in sys.path:
    sys.path.insert(0, str(WS_SRC / "pus6_kinematics"))
if str(WS_SRC / "h30_imu" / "scripts") not in sys.path:
    sys.path.insert(0, str(WS_SRC / "h30_imu" / "scripts"))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_node import quaternion_to_euler


@dataclass
class ControllerEvaluation:
    e_vec_deg: np.ndarray          # 3-element error in deg
    residual_deg: np.ndarray       # 3-element clamped residual in deg
    q_cmd_xyzw: np.ndarray         # 4-element unit quaternion (x, y, z, w)
    r_cmd_matrix: np.ndarray       # 3x3 rotation matrix
    euler_cmd_deg: np.ndarray      # 3-element [roll, pitch, yaw] in deg


def euler_deg_to_rot_matrix(r_deg: float, p_deg: float, y_deg: float) -> np.ndarray:
    """Creates R = Rz(y) * Ry(p) * Rx(r) matching Analytical6PusKinematics convention."""
    return R.from_euler("xyz", [r_deg, p_deg, y_deg], degrees=True).as_matrix()


def rot_matrix_to_euler_deg(rot_mat: np.ndarray) -> np.ndarray:
    """Extracts extrinsic XYZ (intrinsic ZYX) roll, pitch, yaw in degrees."""
    return R.from_matrix(rot_mat).as_euler("xyz", degrees=True)


def so3_angular_distance_rad(r1: np.ndarray, r2: np.ndarray) -> float:
    """Computes the geodesic angular distance ||Log(R1 * R2^T)||_2 on SO(3)."""
    rot_diff = R.from_matrix(r1 @ r2.T)
    return float(np.linalg.norm(rot_diff.as_rotvec()))


def evaluate_euler_channel_controller(
    ref_rpy_deg: Sequence[float],
    meas_rpy_deg: Sequence[float],
    active_axes: Set[str],
    kp: Tuple[float, float, float] = (0.5, 0.5, 0.5),
    limits_deg: Tuple[float, float, float] = (0.25, 0.25, 0.50),
) -> ControllerEvaluation:
    """Evaluates the J9.1-3A frozen Euler-channel P controller with right-multiplied quaternion composition."""
    r_ref, p_ref, y_ref = [float(x) for x in ref_rpy_deg]
    r_meas, p_meas, y_meas = [float(x) for x in meas_rpy_deg]

    e_r = r_ref - r_meas
    e_p = p_ref - p_meas
    e_y = y_ref - y_meas
    e_vec = np.array([e_r, e_p, e_y], dtype=np.float64)

    res_r = np.clip(kp[0] * e_r, -limits_deg[0], limits_deg[0]) if "roll" in active_axes else 0.0
    res_p = np.clip(kp[1] * e_p, -limits_deg[1], limits_deg[1]) if "pitch" in active_axes else 0.0
    res_y = np.clip(kp[2] * e_y, -limits_deg[2], limits_deg[2]) if "yaw" in active_axes else 0.0
    res_vec = np.array([res_r, res_p, res_y], dtype=np.float64)

    rot_ref = R.from_euler("xyz", [r_ref, p_ref, y_ref], degrees=True)
    rot_res = R.from_euler("xyz", [res_r, res_p, res_y], degrees=True)
    rot_cmd = rot_ref * rot_res

    q_cmd = rot_cmd.as_quat()  # [x, y, z, w]
    r_cmd = rot_cmd.as_matrix()
    euler_cmd = rot_matrix_to_euler_deg(r_cmd)

    return ControllerEvaluation(
        e_vec_deg=e_vec,
        residual_deg=res_vec,
        q_cmd_xyzw=q_cmd,
        r_cmd_matrix=r_cmd,
        euler_cmd_deg=euler_cmd,
    )


def evaluate_so3_candidate_controller(
    ref_rpy_deg: Sequence[float],
    meas_rpy_deg: Sequence[float],
    active_axes: Set[str],
    kp: Tuple[float, float, float] = (0.5, 0.5, 0.5),
    limits_deg: Tuple[float, float, float] = (0.25, 0.25, 0.50),
) -> Tuple[ControllerEvaluation, np.ndarray]:
    """
    Evaluates candidate SO(3) attitude error controller:
      E_W = R_ref * R_meas^T
      e_W = Log(E_W)^vee
      e_ref = R_ref^T * e_W   (in reference Platform/body frame)
      active_axes masking on e_ref
      delta_ref = clamp(Kp * e_ref_masked, -limit, +limit)
      R_cmd = R_ref * Exp([delta_ref]_x)
    Returns:
      (evaluation, e_w_deg)
    """
    R_ref = euler_deg_to_rot_matrix(ref_rpy_deg[0], ref_rpy_deg[1], ref_rpy_deg[2])
    R_meas = euler_deg_to_rot_matrix(meas_rpy_deg[0], meas_rpy_deg[1], meas_rpy_deg[2])

    E_W_mat = R_ref @ R_meas.T
    E_W_rot = R.from_matrix(E_W_mat)
    e_w_rad = E_W_rot.as_rotvec()
    e_w_deg = np.degrees(e_w_rad)

    # Frame transport to reference body frame
    e_ref_rad = R_ref.T @ e_w_rad
    e_ref_deg = np.degrees(e_ref_rad)

    # Masking in reference body frame
    axes_map = {"roll": 0, "pitch": 1, "yaw": 2}
    masked_e_ref_deg = np.zeros(3, dtype=np.float64)
    for ax, idx in axes_map.items():
        if ax in active_axes:
            masked_e_ref_deg[idx] = e_ref_deg[idx]

    # Component-wise clamp
    res_vec_deg = np.array([
        np.clip(kp[0] * masked_e_ref_deg[0], -limits_deg[0], limits_deg[0]),
        np.clip(kp[1] * masked_e_ref_deg[1], -limits_deg[1], limits_deg[1]),
        np.clip(kp[2] * masked_e_ref_deg[2], -limits_deg[2], limits_deg[2]),
    ], dtype=np.float64)

    # Right-multiplied command composition on SO(3)
    rot_ref = R.from_matrix(R_ref)
    rot_res = R.from_rotvec(np.radians(res_vec_deg))
    rot_cmd = rot_ref * rot_res

    q_cmd = rot_cmd.as_quat()
    r_cmd = rot_cmd.as_matrix()
    euler_cmd = rot_matrix_to_euler_deg(r_cmd)

    eval_res = ControllerEvaluation(
        e_vec_deg=e_ref_deg,
        residual_deg=res_vec_deg,
        q_cmd_xyzw=q_cmd,
        r_cmd_matrix=r_cmd,
        euler_cmd_deg=euler_cmd,
    )
    return eval_res, e_w_deg


def compute_ik_steps(kin: Analytical6PusKinematics, r_cmd_matrix: np.ndarray, z0: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Runs frozen IK for target orientation at nominal height z0.
    Returns:
      (continuous_steps, integer_steps)
    """
    euler_rad = R.from_matrix(r_cmd_matrix).as_euler("xyz", degrees=False)
    pose = [0.0, 0.0, z0, euler_rad[0], euler_rad[1], euler_rad[2]]

    q_sliders = kin.inverse_kinematics(pose)
    # Convert slider positions (m) to steps relative to 50mm HOME (40000 steps/m)
    # home_slider = 0.05m -> step = 1000 + (q - 0.05) * 40000
    continuous_steps = np.array([1000.0 + (q - 0.05) * 40000.0 for q in q_sliders], dtype=np.float64)
    integer_steps = np.round(continuous_steps).astype(int)
    return continuous_steps, integer_steps


def replay_historical_dataset(csv_path: Path, active_axes: Set[str]) -> Dict[str, Any]:
    """Replays a single recorded CSV dataset through both controllers and frozen IK."""
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height

    rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))

    error_diffs: List[np.ndarray] = []
    so3_distances_deg: List[float] = []
    residual_diffs: List[np.ndarray] = []
    clamp_divergences: int = 0
    sign_divergences: int = 0

    ik_cont_diffs: List[np.ndarray] = []
    int_mismatches: int = 0
    max_int_diff_per_sample: List[int] = []

    for row in rows:
        ref_rpy = (float(row["roll_ref_deg"]), float(row["pitch_ref_deg"]), float(row["yaw_ref_deg"]))
        meas_rpy = (float(row["plat_roll_deg"]), float(row["plat_pitch_deg"]), float(row["plat_yaw_deg"]))

        eval_euler = evaluate_euler_channel_controller(ref_rpy, meas_rpy, active_axes)
        eval_so3, _ = evaluate_so3_candidate_controller(ref_rpy, meas_rpy, active_axes)

        # 1. Error vector difference
        e_diff = np.abs(eval_euler.e_vec_deg - eval_so3.e_vec_deg)
        error_diffs.append(e_diff)

        # 2. Command SO(3) angular distance
        ang_dist_rad = so3_angular_distance_rad(eval_euler.r_cmd_matrix, eval_so3.r_cmd_matrix)
        so3_distances_deg.append(math.degrees(ang_dist_rad))

        # 3. Residual difference
        res_diff = np.abs(eval_euler.residual_deg - eval_so3.residual_deg)
        residual_diffs.append(res_diff)

        # Check clamp decisions
        limits = [0.25, 0.25, 0.50]
        for i in range(3):
            clamped_e = abs(abs(eval_euler.residual_deg[i]) - limits[i]) < 1e-6
            clamped_s = abs(abs(eval_so3.residual_deg[i]) - limits[i]) < 1e-6
            if clamped_e != clamped_s:
                clamp_divergences += 1

        # Check sign consistency
        for i in range(3):
            if abs(eval_euler.residual_deg[i]) > 1e-4 and abs(eval_so3.residual_deg[i]) > 1e-4:
                if np.sign(eval_euler.residual_deg[i]) != np.sign(eval_so3.residual_deg[i]):
                    sign_divergences += 1

        # 4. Frozen IK comparison
        cont_e, int_e = compute_ik_steps(kin, eval_euler.r_cmd_matrix, z0)
        cont_s, int_s = compute_ik_steps(kin, eval_so3.r_cmd_matrix, z0)

        cont_diff = np.abs(cont_e - cont_s)
        ik_cont_diffs.append(cont_diff)

        int_diff = np.abs(int_e - int_s)
        max_d = int(np.max(int_diff))
        max_int_diff_per_sample.append(max_d)
        if max_d > 0:
            int_mismatches += 1

    error_diffs_arr = np.array(error_diffs)
    res_diffs_arr = np.array(residual_diffs)
    ik_cont_arr = np.array(ik_cont_diffs)

    total_samples = len(rows)
    return {
        "csv_path": str(csv_path),
        "total_samples": total_samples,
        "max_error_diff_deg": np.max(error_diffs_arr, axis=0).tolist(),
        "mean_error_diff_deg": np.mean(error_diffs_arr, axis=0).tolist(),
        "max_so3_dist_deg": float(np.max(so3_distances_deg)),
        "mean_so3_dist_deg": float(np.mean(so3_distances_deg)),
        "max_residual_diff_deg": np.max(res_diffs_arr, axis=0).tolist(),
        "mean_residual_diff_deg": np.mean(res_diffs_arr, axis=0).tolist(),
        "clamp_divergences": clamp_divergences,
        "sign_divergences": sign_divergences,
        "max_cont_ik_diff_step": np.max(ik_cont_arr, axis=0).tolist(),
        "mean_cont_ik_diff_step": float(np.mean(ik_cont_arr)),
        "integer_mismatches_count": int_mismatches,
        "integer_mismatch_pct": (int_mismatches / total_samples * 100.0) if total_samples > 0 else 0.0,
        "max_integer_diff_step": int(max(max_int_diff_per_sample)) if max_int_diff_per_sample else 0,
    }


def evaluate_synthetic_case(
    ref_rpy_deg: Tuple[float, float, float],
    meas_rpy_deg: Tuple[float, float, float],
    active_axes: Set[str] = {"roll", "pitch", "yaw"},
) -> Dict[str, Any]:
    """Evaluates a synthetic 3DOF case through both controllers and frozen IK."""
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height

    eval_euler = evaluate_euler_channel_controller(ref_rpy_deg, meas_rpy_deg, active_axes)
    eval_so3, e_w_deg = evaluate_so3_candidate_controller(ref_rpy_deg, meas_rpy_deg, active_axes)

    so3_dist_deg = math.degrees(so3_angular_distance_rad(eval_euler.r_cmd_matrix, eval_so3.r_cmd_matrix))

    cont_e, int_e = compute_ik_steps(kin, eval_euler.r_cmd_matrix, z0)
    cont_s, int_s = compute_ik_steps(kin, eval_so3.r_cmd_matrix, z0)

    cont_diff = np.abs(cont_e - cont_s)
    int_diff = np.abs(int_e - int_s)

    return {
        "ref_rpy_deg": list(ref_rpy_deg),
        "meas_rpy_deg": list(meas_rpy_deg),
        "euler_channel": {
            "e_deg": eval_euler.e_vec_deg.tolist(),
            "res_deg": eval_euler.residual_deg.tolist(),
            "cmd_rpy_deg": eval_euler.euler_cmd_deg.tolist(),
            "cont_steps": np.round(cont_e, 3).tolist(),
            "int_steps": int_e.tolist(),
        },
        "so3_candidate": {
            "e_w_deg": np.round(e_w_deg, 4).tolist(),
            "e_ref_deg": np.round(eval_so3.e_vec_deg, 4).tolist(),
            "res_deg": np.round(eval_so3.residual_deg, 4).tolist(),
            "cmd_rpy_deg": np.round(eval_so3.euler_cmd_deg, 4).tolist(),
            "cont_steps": np.round(cont_s, 3).tolist(),
            "int_steps": int_s.tolist(),
        },
        "so3_cmd_angular_diff_deg": float(so3_dist_deg),
        "max_cont_diff_step": float(np.max(cont_diff)),
        "mean_cont_diff_step": float(np.mean(cont_diff)),
        "max_int_diff_step": int(np.max(int_diff)),
        "int_step_mismatch": bool(np.any(int_diff > 0)),
    }
