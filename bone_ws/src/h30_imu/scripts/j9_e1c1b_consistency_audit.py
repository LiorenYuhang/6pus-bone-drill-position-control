#!/usr/bin/env python3
"""J9-E1C-1B: Offline Estimator Consistency Audit.

Investigates why the fixed extrinsic estimate changes across trajectories
and analyzes the 2.786 deg fit-vs-validation refit delta.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path("/home/lyapunov/bone_drill")
WS_ROOT = REPO_ROOT / "bone_ws"
sys.path.insert(0, str(WS_ROOT / "src" / "h30_imu" / "scripts"))
import j9_e1a_platform_orientation_calibrator as e1a

DATA_DIR = REPO_ROOT / "reports" / "j9_e1c1_platform_orientation_calibration_20260828"


def so3_delta_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    r1 = Rotation.from_matrix(R1)
    r2 = Rotation.from_matrix(R2)
    delta = r1.inv() * r2
    return float(math.degrees(delta.magnitude()))


def load_dataset() -> dict[str, Any]:
    with open(DATA_DIR / "target_pose.csv") as f:
        target_rows = list(csv.DictReader(f))
    t_targets = [int(r["callback_monotonic_ns"]) for r in target_rows]

    segments = {
        "preroll": (t_targets[0], t_targets[199]),
        "roll": (t_targets[200], t_targets[599]),
        "pitch": (t_targets[600], t_targets[999]),
        "yaw": (t_targets[1000], t_targets[1399]),
        "combined": (t_targets[1400], t_targets[1999]),
        "validation": (t_targets[2000], t_targets[2599]),
        "home_return": (t_targets[2600], t_targets[2799]),
    }

    with open(DATA_DIR / "imu.csv") as f:
        imu_rows = list(csv.DictReader(f))
    with open(DATA_DIR / "fk.csv") as f:
        fk_rows = list(csv.DictReader(f))

    # Clock sync
    valid_imu = [r for r in imu_rows if int(r.get("has_sample_timestamp", 1)) == 1]
    dev_us = np.asarray([int(r["sample_timestamp_us"]) for r in valid_imu], dtype=float)
    host_ns = np.asarray([int(float(r["host_monotonic_arrival_time_ns"])) for r in valid_imu], dtype=float)
    dev_unwrapped, _ = e1a.unwrap_u32_us(dev_us)
    affine_fit = e1a.fit_affine_device_to_host(dev_unwrapped, host_ns)

    imu_stamps_ns = e1a.map_sample_timestamp_to_monotonic_ns(dev_unwrapped, affine_fit["a"], affine_fit["b_us"])
    raw_gyro = np.asarray([[float(r["gx_rad_s"]), float(r["gy_rad_s"]), float(r["gz_rad_s"])] for r in valid_imu], dtype=float)

    # Preroll gyro bias
    t_pre_s, t_pre_e = segments["preroll"]
    pre_mask = (imu_stamps_ns >= t_pre_s) & (imu_stamps_ns <= t_pre_e)
    preroll_bias = np.mean(raw_gyro[pre_mask], axis=0)

    # FK data
    fk_stamps_ns = np.asarray([int(float(r["query_monotonic_timestamp_ns"])) for r in fk_rows], dtype=np.int64)
    fk_quats = np.asarray([[float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])] for r in fk_rows], dtype=float)
    rotations_b_p = Rotation.from_quat(fk_quats)

    return {
        "segments": segments,
        "imu_stamps_ns": imu_stamps_ns,
        "raw_gyro": raw_gyro,
        "preroll_bias": preroll_bias,
        "fk_stamps_ns": fk_stamps_ns,
        "rotations_b_p": rotations_b_p,
        "affine_fit": affine_fit,
    }


def extract_paired_data(
    data: dict[str, Any],
    window_s: float = 0.12,
    time_shift_ns: int = 0,
    use_bias: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Derives platform angular velocity and interpolates gyro with time shift."""
    omega_p, valid_fk_omega, _ = e1a.local_body_angular_velocity_from_rotations(
        data["fk_stamps_ns"], data["rotations_b_p"], window_s=window_s, poly_degree=2
    )
    fk_times = data["fk_stamps_ns"][valid_fk_omega]
    w_p = omega_p[valid_fk_omega]

    bias = data["preroll_bias"] if use_bias is None else use_bias
    gyro_corrected = data["raw_gyro"] - bias

    shifted_imu_times = data["imu_stamps_ns"] + time_shift_ns
    gyro_interp, gyro_valid = e1a.interpolate_gyro(shifted_imu_times, gyro_corrected, fk_times)

    joint_valid = gyro_valid & np.all(np.isfinite(w_p), axis=1) & np.all(np.isfinite(gyro_interp), axis=1)
    return fk_times[joint_valid], w_p[joint_valid], gyro_interp[joint_valid]


def filter_windows(times: np.ndarray, w_p: np.ndarray, w_imu: np.ndarray, windows: list[tuple[int, int]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mask = np.zeros(len(times), dtype=bool)
    for t_s, t_e in windows:
        mask |= ((times >= t_s) & (times <= t_e))
    return times[mask], w_p[mask], w_imu[mask]


def solve_kabsch(w_p: np.ndarray, w_imu: np.ndarray) -> np.ndarray:
    H = w_p.T @ w_imu
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(U @ Vt)
    return U @ np.diag([1.0, 1.0, d]) @ Vt


def run_audit() -> dict[str, Any]:
    data = load_dataset()
    seg = data["segments"]

    fit_windows = [seg["roll"], seg["pitch"], seg["yaw"], seg["combined"]]
    val_windows = [seg["validation"]]

    results: dict[str, Any] = {}

    # =========================================================================
    # Check 1: Original E1A alternating estimator vs Bias-Subtracted Kabsch
    # =========================================================================
    print("--- Running Check 1: E1A Alternating vs Bias-Subtracted Kabsch ---")
    t_all, wp_all, g_all = extract_paired_data(data, window_s=0.12, use_bias=np.zeros(3))
    t_fit, wp_fit, g_fit = filter_windows(t_all, wp_all, g_all, fit_windows)
    t_val, wp_val, g_val = filter_windows(t_all, wp_all, g_all, val_windows)

    # E1A Alternating
    e1a_fit_res = e1a.solve_rotation_and_bias(wp_fit, g_fit)
    R_e1a = e1a_fit_res.rotation_p_ip
    b_e1a = e1a_fit_res.gyro_bias_ip_rad_s

    # Kabsch with Preroll Bias
    t_all_p, wp_all_p, g_all_p = extract_paired_data(data, window_s=0.12, use_bias=data["preroll_bias"])
    t_fit_p, wp_fit_p, g_fit_p = filter_windows(t_all_p, wp_all_p, g_all_p, fit_windows)
    t_val_p, wp_val_p, g_val_p = filter_windows(t_all_p, wp_all_p, g_all_p, val_windows)
    R_kabsch = solve_kabsch(wp_fit_p, g_fit_p)

    # Validation refit for both
    e1a_val_res = e1a.solve_rotation_and_bias(wp_val, g_val)
    R_e1a_val = e1a_val_res.rotation_p_ip
    R_kabsch_val = solve_kabsch(wp_val_p, g_val_p)

    delta_e1a = so3_delta_deg(R_e1a, R_e1a_val)
    delta_kabsch = so3_delta_deg(R_kabsch, R_kabsch_val)
    delta_between_estimators = so3_delta_deg(R_e1a, R_kabsch)

    # Residuals
    res_e1a_fit = np.linalg.norm(R_e1a @ (g_fit - b_e1a).T - wp_fit.T, axis=0)
    res_e1a_val = np.linalg.norm(R_e1a @ (g_val - b_e1a).T - wp_val.T, axis=0)
    res_kab_fit = np.linalg.norm(R_kabsch @ g_fit_p.T - wp_fit_p.T, axis=0)
    res_kab_val = np.linalg.norm(R_kabsch @ g_val_p.T - wp_val_p.T, axis=0)

    results["check1_estimator_comparison"] = {
        "e1a_alternating": {
            "euler_xyz_deg": Rotation.from_matrix(R_e1a).as_euler("xyz", degrees=True).tolist(),
            "gyro_bias_rad_s": b_e1a.tolist(),
            "gyro_bias_deg_s": [float(math.degrees(v)) for v in b_e1a],
            "fit_rms_rad_s": float(np.sqrt(np.mean(res_e1a_fit**2))),
            "val_rms_rad_s": float(np.sqrt(np.mean(res_e1a_val**2))),
            "fit_val_refit_delta_deg": delta_e1a,
        },
        "preroll_bias_kabsch": {
            "euler_xyz_deg": Rotation.from_matrix(R_kabsch).as_euler("xyz", degrees=True).tolist(),
            "gyro_bias_rad_s": data["preroll_bias"].tolist(),
            "gyro_bias_deg_s": [float(math.degrees(v)) for v in data["preroll_bias"]],
            "fit_rms_rad_s": float(np.sqrt(np.mean(res_kab_fit**2))),
            "val_rms_rad_s": float(np.sqrt(np.mean(res_kab_val**2))),
            "fit_val_refit_delta_deg": delta_kabsch,
        },
        "delta_between_estimators_deg": delta_between_estimators,
    }

    # =========================================================================
    # Check 2: Single-Trajectory Sub-Estimates and Pairwise SO(3) Deltas
    # =========================================================================
    print("--- Running Check 2: Single-Trajectory Sub-Estimates ---")
    traj_names = ["roll", "pitch", "yaw", "combined", "validation"]
    sub_models: dict[str, np.ndarray] = {}
    sub_eulers: dict[str, list[float]] = {}
    sub_cond: dict[str, float] = {}

    for name in traj_names:
        t_sub, wp_sub, g_sub = filter_windows(t_all_p, wp_all_p, g_all_p, [seg[name]])
        R_sub = solve_kabsch(wp_sub, g_sub)
        sub_models[name] = R_sub
        sub_eulers[name] = Rotation.from_matrix(R_sub).as_euler("xyz", degrees=True).tolist()
        M = g_sub.T @ g_sub
        sv = np.linalg.svd(M, compute_uv=False)
        sub_cond[name] = float(sv[0] / max(sv[-1], 1e-12))

    # Pairwise deltas
    pairwise_matrix = {}
    for n1 in traj_names:
        pairwise_matrix[n1] = {}
        for n2 in traj_names:
            pairwise_matrix[n1][n2] = round(so3_delta_deg(sub_models[n1], sub_models[n2]), 4)

    results["check2_single_trajectory_estimates"] = {
        "trajectory_eulers_deg": sub_eulers,
        "trajectory_condition_numbers": sub_cond,
        "pairwise_so3_deltas_deg": pairwise_matrix,
    }

    # =========================================================================
    # Check 3: Smoothing Window Sensitivity Sweep
    # =========================================================================
    print("--- Running Check 3: Smoothing Window Sensitivity Sweep ---")
    windows = [0.06, 0.08, 0.12, 0.16, 0.20]
    window_sweep = {}
    R_base = None

    for w_s in windows:
        t_w, wp_w, g_w = extract_paired_data(data, window_s=w_s, use_bias=data["preroll_bias"])
        t_fit_w, wp_fit_w, g_fit_w = filter_windows(t_w, wp_w, g_w, fit_windows)
        t_val_w, wp_val_w, g_val_w = filter_windows(t_w, wp_w, g_w, val_windows)

        R_fit_w = solve_kabsch(wp_fit_w, g_fit_w)
        R_val_w = solve_kabsch(wp_val_w, g_val_w)

        if w_s == 0.12:
            R_base = R_fit_w

        res_fit_w = np.linalg.norm(R_fit_w @ g_fit_w.T - wp_fit_w.T, axis=0)
        res_val_w = np.linalg.norm(R_fit_w @ g_val_w.T - wp_val_w.T, axis=0)

        window_sweep[f"{w_s:.2f}s"] = {
            "euler_xyz_deg": Rotation.from_matrix(R_fit_w).as_euler("xyz", degrees=True).tolist(),
            "fit_rms_rad_s": float(np.sqrt(np.mean(res_fit_w**2))),
            "val_rms_rad_s": float(np.sqrt(np.mean(res_val_w**2))),
            "fit_val_refit_delta_deg": so3_delta_deg(R_fit_w, R_val_w),
            "delta_vs_0_12s_baseline_deg": so3_delta_deg(R_fit_w, R_base) if R_base is not None else 0.0,
        }

    results["check3_smoothing_window_sweep"] = window_sweep

    # =========================================================================
    # Check 4: IMU <-> FK Relative Timing Offset Sweep (-20ms to +20ms)
    # =========================================================================
    print("--- Running Check 4: Timing Offset Sweep (-20ms to +20ms) ---")
    shifts_ms = list(range(-20, 21, 2))
    timing_sweep = []

    best_fit_rms = math.inf
    best_shift_fit_rms = 0
    best_val_rms = math.inf
    best_shift_val_rms = 0
    best_refit_delta = math.inf
    best_shift_refit_delta = 0

    for dt_ms in shifts_ms:
        dt_ns = int(dt_ms * 1_000_000)
        t_shift, wp_shift, g_shift = extract_paired_data(data, window_s=0.12, time_shift_ns=dt_ns, use_bias=data["preroll_bias"])
        t_fit_s, wp_fit_s, g_fit_s = filter_windows(t_shift, wp_shift, g_shift, fit_windows)
        t_val_s, wp_val_s, g_val_s = filter_windows(t_shift, wp_shift, g_shift, val_windows)

        R_fit_s = solve_kabsch(wp_fit_s, g_fit_s)
        R_val_s = solve_kabsch(wp_val_s, g_val_s)

        res_fit_s = np.linalg.norm(R_fit_s @ g_fit_s.T - wp_fit_s.T, axis=0)
        res_val_s = np.linalg.norm(R_fit_s @ g_val_s.T - wp_val_s.T, axis=0)
        refit_delta_s = so3_delta_deg(R_fit_s, R_val_s)

        rms_fit = float(np.sqrt(np.mean(res_fit_s**2)))
        rms_val = float(np.sqrt(np.mean(res_val_s**2)))

        if rms_fit < best_fit_rms:
            best_fit_rms = rms_fit
            best_shift_fit_rms = dt_ms
        if rms_val < best_val_rms:
            best_val_rms = rms_val
            best_shift_val_rms = dt_ms
        if refit_delta_s < best_refit_delta:
            best_refit_delta = refit_delta_s
            best_shift_refit_delta = dt_ms

        timing_sweep.append({
            "shift_ms": dt_ms,
            "fit_rms_rad_s": rms_fit,
            "val_rms_rad_s": rms_val,
            "fit_val_refit_delta_deg": refit_delta_s,
            "euler_xyz_deg": Rotation.from_matrix(R_fit_s).as_euler("xyz", degrees=True).tolist(),
        })

    results["check4_timing_offset_sweep"] = {
        "sweep_data": timing_sweep,
        "optimal_for_fit_rms": {"shift_ms": best_shift_fit_rms, "fit_rms_rad_s": best_fit_rms},
        "optimal_for_val_rms": {"shift_ms": best_shift_val_rms, "val_rms_rad_s": best_val_rms},
        "optimal_for_refit_delta": {"shift_ms": best_shift_refit_delta, "refit_delta_deg": best_refit_delta},
    }

    # =========================================================================
    # Check 5: Joint Gyro Bias vs Preroll Bias
    # =========================================================================
    print("--- Running Check 5: Joint Gyro Bias vs Preroll Bias ---")
    results["check5_joint_bias_evaluation"] = {
        "preroll_bias_rad_s": data["preroll_bias"].tolist(),
        "preroll_bias_deg_s": [float(math.degrees(v)) for v in data["preroll_bias"]],
        "joint_estimated_bias_rad_s": b_e1a.tolist(),
        "joint_estimated_bias_deg_s": [float(math.degrees(v)) for v in b_e1a],
        "bias_difference_deg_s": [float(math.degrees(b_e1a[i] - data["preroll_bias"][i])) for i in range(3)],
        "refit_delta_with_preroll_bias_deg": delta_kabsch,
        "refit_delta_with_joint_bias_deg": delta_e1a,
        "improvement_from_joint_bias_deg": delta_kabsch - delta_e1a,
    }

    return results


if __name__ == "__main__":
    audit_res = run_audit()
    out_file = DATA_DIR / "j9_e1c1b_consistency_audit_results.json"
    out_file.write_text(json.dumps(audit_res, indent=2))
    print(f"\nAudit complete. Results written to {out_file}")
    print(json.dumps(audit_res, indent=2))
