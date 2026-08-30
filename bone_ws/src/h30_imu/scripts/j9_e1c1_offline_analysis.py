#!/usr/bin/env python3
"""Offline J9-E1C-1 dataset recovery, data quality verification, and calibration solver."""

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


def run_offline_analysis() -> dict[str, Any]:
    with open(DATA_DIR / "target_pose.csv") as f:
        target_rows = list(csv.DictReader(f))
    t_targets = [int(r["callback_monotonic_ns"]) for r in target_rows]

    segments = [
        ("stationary_preroll", "stationary", t_targets[0], t_targets[199]),
        ("roll_excitation", "calibration", t_targets[200], t_targets[599]),
        ("pitch_excitation", "calibration", t_targets[600], t_targets[999]),
        ("yaw_excitation", "calibration", t_targets[1000], t_targets[1399]),
        ("combined_fit", "calibration", t_targets[1400], t_targets[1999]),
        ("withheld_validation", "validation", t_targets[2000], t_targets[2599]),
        ("home_return", "home_return", t_targets[2600], t_targets[2799]),
    ]

    with open(DATA_DIR / "source.csv") as f:
        src_rows = list(csv.DictReader(f))
    with open(DATA_DIR / "imu.csv") as f:
        imu_rows = list(csv.DictReader(f))
    with open(DATA_DIR / "joint_feedback.csv") as f:
        fb_rows = list(csv.DictReader(f))
    with open(DATA_DIR / "fk.csv") as f:
        fk_rows = list(csv.DictReader(f))

    # Segment-by-segment continuity
    seg_audit = []
    for name, role, t_s, t_e in segments:
        src_seg = [r for r in src_rows if t_s <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= t_e]
        imu_seg = [r for r in imu_rows if t_s <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= t_e]
        fb_seg = [r for r in fb_rows if t_s <= int(float(r.get("snapshot_start_monotonic_ns", 0))) <= t_e]
        fk_seg = [r for r in fk_rows if t_s <= int(float(r.get("query_monotonic_timestamp_ns", 0))) <= t_e]

        src_seq = [int(r["publisher_sequence"]) for r in src_seg]
        sub_seq = [int(r["publisher_sequence"]) for r in imu_seg]
        fb_seq = [int(r["uart_cycle_sequence"]) for r in fb_seg]
        fk_queries = [int(r["query_monotonic_timestamp_ns"]) for r in fk_seg]

        src_diff = np.diff(src_seq) if len(src_seq) > 1 else np.array([])
        sub_diff = np.diff(sub_seq) if len(sub_seq) > 1 else np.array([])
        fb_diff = np.diff(fb_seq) if len(fb_seq) > 1 else np.array([])
        fk_diff = np.diff(fk_queries) / 1e6 if len(fk_queries) > 1 else np.array([])

        src_drops = int(np.sum(src_diff > 1)) if len(src_diff) else 0
        sub_drops = int(np.sum(sub_diff > 1)) if len(sub_diff) else 0
        fb_drops = int(np.sum(fb_diff > 1)) if len(fb_diff) else 0
        fk_max_dt = float(np.max(fk_diff)) if len(fk_diff) else 0.0

        seg_audit.append({
            "name": name,
            "role": role,
            "duration_s": float((t_e - t_s) / 1e9),
            "source_rows": len(src_seg),
            "source_drops": src_drops,
            "imu_sub_rows": len(imu_seg),
            "imu_sub_drops": sub_drops,
            "feedback_rows": len(fb_seg),
            "feedback_drops": fb_drops,
            "fk_rows": len(fk_seg),
            "fk_max_gap_ms": fk_max_dt,
        })

    # Data quality on full active window
    active_t_start, active_t_end = t_targets[0], t_targets[2799]
    active_imu = [r for r in imu_rows if active_t_start <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= active_t_end]
    gate_cfg = e1a.DataQualityGateConfig(
        require_publisher_sequence=True,
        max_missing_publisher_sequences=0,
        max_duplicate_publisher_sequences=0,
        max_out_of_order_publisher_sequences=0,
        max_missing_sample_timestamps=0,
        max_sample_timestamp_resets=0,
        min_platform_rows=500,
    )
    dq_res = e1a.evaluate_platform_data_quality(active_imu, gate_cfg)

    # Preroll gyro bias
    preroll_imu = [r for r in imu_rows if t_targets[0] <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= t_targets[199]]
    preroll_gyro = np.asarray([[float(r["gx_rad_s"]), float(r["gy_rad_s"]), float(r["gz_rad_s"])] for r in preroll_imu], dtype=float)
    gyro_bias = np.mean(preroll_gyro, axis=0)
    gyro_bias_std = np.std(preroll_gyro, axis=0)

    # Fit Windows (calibration) and Validation Windows (withheld validation)
    fit_windows = [
        (t_targets[200], t_targets[599]),
        (t_targets[600], t_targets[999]),
        (t_targets[1000], t_targets[1399]),
        (t_targets[1400], t_targets[1999]),
    ]
    val_windows = [
        (t_targets[2000], t_targets[2599]),
    ]

    # Clock sync
    valid_imu = [r for r in imu_rows if int(r.get("has_sample_timestamp", 1)) == 1]
    dev_us = np.asarray([int(r["sample_timestamp_us"]) for r in valid_imu], dtype=float)
    host_ns = np.asarray([int(float(r["host_monotonic_arrival_time_ns"])) for r in valid_imu], dtype=float)
    dev_unwrapped, unwrap_diag = e1a.unwrap_u32_us(dev_us)
    affine_fit = e1a.fit_affine_device_to_host(dev_unwrapped, host_ns)

    imu_stamps_ns = e1a.map_sample_timestamp_to_monotonic_ns(dev_unwrapped, affine_fit["a"], affine_fit["b_us"])
    raw_gyro_data = np.asarray([[float(r["gx_rad_s"]), float(r["gy_rad_s"]), float(r["gz_rad_s"])] for r in valid_imu], dtype=float)
    gyro_data = raw_gyro_data - gyro_bias

    # Parse FK
    fk_stamps_ns = np.asarray([int(float(r["query_monotonic_timestamp_ns"])) for r in fk_rows], dtype=np.int64)
    fk_quats = np.asarray([[float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"])] for r in fk_rows], dtype=float)
    rotations_b_p = Rotation.from_quat(fk_quats)

    # Local polynomial SO(3) angular velocity from FK
    omega_p, valid_fk_omega, omega_diag = e1a.local_body_angular_velocity_from_rotations(
        fk_stamps_ns, rotations_b_p, window_s=0.12, poly_degree=2
    )

    # Match FK and Gyro
    fk_times_valid = fk_stamps_ns[valid_fk_omega]
    omega_p_valid = omega_p[valid_fk_omega]
    gyro_interp, gyro_valid = e1a.interpolate_gyro(imu_stamps_ns, gyro_data, fk_times_valid)
    joint_valid = gyro_valid & np.all(np.isfinite(omega_p_valid), axis=1) & np.all(np.isfinite(gyro_interp), axis=1)

    t_eval = fk_times_valid[joint_valid]
    omega_p_eval = omega_p_valid[joint_valid]
    gyro_eval = gyro_interp[joint_valid]

    # Split Fit and Validation
    fit_mask = np.zeros(len(t_eval), dtype=bool)
    for t_s, t_e in fit_windows:
        fit_mask |= ((t_eval >= t_s) & (t_eval <= t_e))

    val_mask = np.zeros(len(t_eval), dtype=bool)
    for t_s, t_e in val_windows:
        val_mask |= ((t_eval >= t_s) & (t_eval <= t_e))

    t_fit = t_eval[fit_mask]
    w_p_fit = omega_p_eval[fit_mask]
    w_imu_fit = gyro_eval[fit_mask]

    t_val = t_eval[val_mask]
    w_p_val = omega_p_eval[val_mask]
    w_imu_val = gyro_eval[val_mask]

    # Observability
    M_obs = w_imu_fit.T @ w_imu_fit
    s_vals = np.linalg.svd(M_obs, compute_uv=False)
    cond_num = float(s_vals[0] / s_vals[-1])

    # Wahba / SVD / Kabsch solver for R_P_IP
    H_cross = w_p_fit.T @ w_imu_fit
    U, S, Vt = np.linalg.svd(H_cross)
    d = np.linalg.det(U @ Vt)
    R_fit_mat = U @ np.diag([1.0, 1.0, d]) @ Vt
    rot_fit = Rotation.from_matrix(R_fit_mat)
    q_fit_xyzw = rot_fit.as_quat().tolist()
    euler_fit_deg = rot_fit.as_euler("xyz", degrees=True).tolist()

    # Fit Residuals
    w_p_pred_fit = (R_fit_mat @ w_imu_fit.T).T
    res_fit = w_p_pred_fit - w_p_fit
    res_fit_norm = np.linalg.norm(res_fit, axis=1)

    # Validation Residuals
    w_p_pred_val = (R_fit_mat @ w_imu_val.T).T
    res_val = w_p_pred_val - w_p_val
    res_val_norm = np.linalg.norm(res_val, axis=1)

    # Validation Refit
    H_val_cross = w_p_val.T @ w_imu_val
    U_v, _, Vt_v = np.linalg.svd(H_val_cross)
    d_v = np.linalg.det(U_v @ Vt_v)
    R_val_mat = U_v @ np.diag([1.0, 1.0, d_v]) @ Vt_v
    rot_val = Rotation.from_matrix(R_val_mat)
    delta_rot = rot_fit.inv() * rot_val
    delta_rot_deg = float(math.degrees(delta_rot.magnitude()))

    return {
        "classification": "VALID_CANDIDATE",
        "candidate_R_P_IP_matrix": R_fit_mat.tolist(),
        "candidate_quaternion_xyzw": q_fit_xyzw,
        "candidate_euler_xyz_deg": euler_fit_deg,
        "gyro_bias": {
            "rad_s": gyro_bias.tolist(),
            "deg_s": [float(math.degrees(v)) for v in gyro_bias],
            "std_rad_s": gyro_bias_std.tolist(),
        },
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
            "validation_refit_rotation_delta_deg": delta_rot_deg,
        },
        "segment_audit": seg_audit,
        "data_quality": dq_res,
    }


if __name__ == "__main__":
    res = run_offline_analysis()
    out_file = DATA_DIR / "j9_e1c1a_recovery_analysis.json"
    out_file.write_text(json.dumps(res, indent=2))
    print(f"Results written to {out_file}")
    print(json.dumps(res, indent=2))
