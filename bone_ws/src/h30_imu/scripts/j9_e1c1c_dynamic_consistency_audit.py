#!/usr/bin/env python3
"""J9-E1C-1C: High-SNR / Dynamic-Consistency Offline Audit.

Performs deep data-driven investigation on:
1. Measured angular rates (FK and IMU) across segments
2. High-|omega| thresholded sub-estimations
3. Direction-dependent velocity splitting (backlash/hysteresis)
4. Dynamic loading splitting (angular acceleration |alpha|)
5. Excitation covariance and eigenvector subspace alignment
6. Integrated finite-window relative rotation (Hand-Eye) estimator
7. E1C-2 trajectory kinematic/stroke envelope offline verification
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
sys.path.insert(0, str(WS_ROOT / "src" / "pus6_kinematics"))
import j9_e1a_platform_orientation_calibrator as e1a
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics

DATA_DIR = REPO_ROOT / "reports" / "j9_e1c1_platform_orientation_calibration_20260828"


def so3_delta_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    r1 = Rotation.from_matrix(R1)
    r2 = Rotation.from_matrix(R2)
    delta = r1.inv() * r2
    return float(math.degrees(delta.magnitude()))


def solve_kabsch(w_p: np.ndarray, w_imu: np.ndarray) -> np.ndarray:
    H = w_p.T @ w_imu
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(U @ Vt)
    return U @ np.diag([1.0, 1.0, d]) @ Vt


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


def compute_all_checks() -> dict[str, Any]:
    data = load_dataset()
    seg = data["segments"]

    fit_windows = [seg["roll"], seg["pitch"], seg["yaw"], seg["combined"]]
    val_windows = [seg["validation"]]

    # 1. Derive FK angular velocity
    omega_p, valid_fk_omega, _ = e1a.local_body_angular_velocity_from_rotations(
        data["fk_stamps_ns"], data["rotations_b_p"], window_s=0.12, poly_degree=2
    )
    fk_times = data["fk_stamps_ns"][valid_fk_omega]
    w_p = omega_p[valid_fk_omega]
    fk_rotations = data["rotations_b_p"][valid_fk_omega]

    # Bias-corrected gyro
    gyro_corrected = data["raw_gyro"] - data["preroll_bias"]
    gyro_interp, gyro_valid = e1a.interpolate_gyro(data["imu_stamps_ns"], gyro_corrected, fk_times)

    joint_valid = gyro_valid & np.all(np.isfinite(w_p), axis=1) & np.all(np.isfinite(gyro_interp), axis=1)
    t_eval = fk_times[joint_valid]
    w_p_eval = w_p[joint_valid]
    g_eval = gyro_interp[joint_valid]
    rot_eval = fk_rotations[joint_valid]

    # Helpers to extract window slices
    def get_window_mask(windows: list[tuple[int, int]]) -> np.ndarray:
        m = np.zeros(len(t_eval), dtype=bool)
        for t_s, t_e in windows:
            m |= ((t_eval >= t_s) & (t_eval <= t_e))
        return m

    fit_mask = get_window_mask(fit_windows)
    val_mask = get_window_mask(val_windows)

    results: dict[str, Any] = {}

    # =========================================================================
    # Check 1: Actual Measured Angular Rate Profiles (FK & IMU) per segment
    # =========================================================================
    print("--- Check 1: Actual Measured Angular Rates ---")
    seg_rates = {}
    for s_name, (t_s, t_e) in seg.items():
        m = (t_eval >= t_s) & (t_eval <= t_e)
        if np.sum(m) == 0:
            continue
        wp_s = w_p_eval[m]
        g_s = g_eval[m]
        wp_mag = np.linalg.norm(wp_s, axis=1)
        g_mag = np.linalg.norm(g_s, axis=1)

        seg_rates[s_name] = {
            "samples": int(np.sum(m)),
            "fk_omega_deg_s": {
                "mean": float(math.degrees(np.mean(wp_mag))),
                "rms": float(math.degrees(np.sqrt(np.mean(wp_mag**2)))),
                "p95": float(math.degrees(np.percentile(wp_mag, 95.0))),
                "peak": float(math.degrees(np.max(wp_mag))),
                "per_axis_peak": [float(math.degrees(np.max(np.abs(wp_s[:, a])))) for a in range(3)],
            },
            "imu_gyro_deg_s": {
                "mean": float(math.degrees(np.mean(g_mag))),
                "rms": float(math.degrees(np.sqrt(np.mean(g_mag**2)))),
                "p95": float(math.degrees(np.percentile(g_mag, 95.0))),
                "peak": float(math.degrees(np.max(g_mag))),
                "per_axis_peak": [float(math.degrees(np.max(np.abs(g_s[:, a])))) for a in range(3)],
            },
        }

    results["check1_actual_measured_angular_rates"] = seg_rates

    # =========================================================================
    # Check 2: Progressively higher-|omega| subset calibration
    # =========================================================================
    print("--- Check 2: Thresholded High-|omega| Subsets ---")
    w_fit_mag = np.linalg.norm(w_p_eval[fit_mask], axis=1)
    w_val_mag = np.linalg.norm(w_p_eval[val_mask], axis=1)

    threshold_sweep = []
    # Test percentiles: 0 (all), 25, 50, 70, 80, 85, 90
    percentiles = [0, 25, 50, 65, 75, 85]
    for p in percentiles:
        th = float(np.percentile(w_fit_mag, p))
        m_fit_th = fit_mask & (np.linalg.norm(w_p_eval, axis=1) >= th)
        m_val_th = val_mask & (np.linalg.norm(w_p_eval, axis=1) >= th)

        n_fit = int(np.sum(m_fit_th))
        n_val = int(np.sum(m_val_th))
        if n_fit < 20 or n_val < 20:
            continue

        R_fit_th = solve_kabsch(w_p_eval[m_fit_th], g_eval[m_fit_th])
        R_val_th = solve_kabsch(w_p_eval[m_val_th], g_eval[m_val_th])

        res_fit = np.linalg.norm(R_fit_th @ g_eval[m_fit_th].T - w_p_eval[m_fit_th].T, axis=0)
        res_val = np.linalg.norm(R_fit_th @ g_eval[m_val_th].T - w_p_eval[m_val_th].T, axis=0)
        delta_th = so3_delta_deg(R_fit_th, R_val_th)

        threshold_sweep.append({
            "percentile": p,
            "threshold_rad_s": th,
            "threshold_deg_s": float(math.degrees(th)),
            "fit_samples": n_fit,
            "val_samples": n_val,
            "euler_xyz_deg": Rotation.from_matrix(R_fit_th).as_euler("xyz", degrees=True).tolist(),
            "fit_rms_rad_s": float(np.sqrt(np.mean(res_fit**2))),
            "val_rms_rad_s": float(np.sqrt(np.mean(res_val**2))),
            "fit_val_refit_delta_deg": delta_th,
        })

    results["check2_thresholded_omega_subsets"] = threshold_sweep

    # =========================================================================
    # Check 3: Direction-dependent velocity splitting (Hysteresis / Backlash)
    # =========================================================================
    print("--- Check 3: Positive vs Negative Angular Velocity Splitting ---")
    wp_fit = w_p_eval[fit_mask]
    g_fit = g_eval[fit_mask]

    # Baseline combined fit
    R_fit_base = solve_kabsch(wp_fit, g_fit)

    direction_splits = {}
    for axis_idx, axis_name in enumerate(["X_roll", "Y_pitch", "Z_yaw"]):
        m_pos = fit_mask & (w_p_eval[:, axis_idx] > 0.005)
        m_neg = fit_mask & (w_p_eval[:, axis_idx] < -0.005)

        n_pos = int(np.sum(m_pos))
        n_neg = int(np.sum(m_neg))

        if n_pos >= 30 and n_neg >= 30:
            R_pos = solve_kabsch(w_p_eval[m_pos], g_eval[m_pos])
            R_neg = solve_kabsch(w_p_eval[m_neg], g_eval[m_neg])
            delta_pos_neg = so3_delta_deg(R_pos, R_neg)
            delta_pos_base = so3_delta_deg(R_pos, R_fit_base)
            delta_neg_base = so3_delta_deg(R_neg, R_fit_base)

            direction_splits[axis_name] = {
                "pos_samples": n_pos,
                "neg_samples": n_neg,
                "euler_pos_deg": Rotation.from_matrix(R_pos).as_euler("xyz", degrees=True).tolist(),
                "euler_neg_deg": Rotation.from_matrix(R_neg).as_euler("xyz", degrees=True).tolist(),
                "pos_vs_neg_so3_delta_deg": delta_pos_neg,
                "pos_vs_base_so3_delta_deg": delta_pos_base,
                "neg_vs_base_so3_delta_deg": delta_neg_base,
            }

    results["check3_direction_dependent_hysteresis"] = direction_splits

    # =========================================================================
    # Check 4: Angular acceleration |alpha| splitting (Dynamic Loading / Compliance)
    # =========================================================================
    print("--- Check 4: Angular Acceleration |alpha| Splitting ---")
    # Estimate angular acceleration from w_p_eval via central difference
    dt_eval = np.gradient(t_eval) * 1e-9
    alpha_p = np.zeros_like(w_p_eval)
    for a in range(3):
        alpha_p[:, a] = np.gradient(w_p_eval[:, a], dt_eval)
    alpha_mag = np.linalg.norm(alpha_p, axis=1)

    alpha_fit = alpha_mag[fit_mask]
    med_alpha = float(np.median(alpha_fit))

    m_low_alpha = fit_mask & (alpha_mag <= med_alpha)
    m_high_alpha = fit_mask & (alpha_mag > med_alpha)

    R_low_alpha = solve_kabsch(w_p_eval[m_low_alpha], g_eval[m_low_alpha])
    R_high_alpha = solve_kabsch(w_p_eval[m_high_alpha], g_eval[m_high_alpha])
    delta_alpha_split = so3_delta_deg(R_low_alpha, R_high_alpha)

    results["check4_angular_acceleration_dynamic_loading"] = {
        "median_alpha_fit_rad_s2": med_alpha,
        "median_alpha_fit_deg_s2": float(math.degrees(med_alpha)),
        "low_alpha_samples": int(np.sum(m_low_alpha)),
        "high_alpha_samples": int(np.sum(m_high_alpha)),
        "euler_low_alpha_deg": Rotation.from_matrix(R_low_alpha).as_euler("xyz", degrees=True).tolist(),
        "euler_high_alpha_deg": Rotation.from_matrix(R_high_alpha).as_euler("xyz", degrees=True).tolist(),
        "low_vs_high_alpha_so3_delta_deg": delta_alpha_split,
        "low_vs_base_so3_delta_deg": so3_delta_deg(R_low_alpha, R_fit_base),
        "high_vs_base_so3_delta_deg": so3_delta_deg(R_high_alpha, R_fit_base),
    }

    # =========================================================================
    # Check 5: Excitation Covariance and Eigenvector Geometry
    # =========================================================================
    print("--- Check 5: Covariance & Eigenvector Geometry ---")
    wp_fit_mat = w_p_eval[fit_mask]
    wp_val_mat = w_p_eval[val_mask]

    cov_fit = (wp_fit_mat.T @ wp_fit_mat) / len(wp_fit_mat)
    cov_val = (wp_val_mat.T @ wp_val_mat) / len(wp_val_mat)

    evals_fit, evecs_fit = np.linalg.eigh(cov_fit)
    evals_val, evecs_val = np.linalg.eigh(cov_val)

    # Sort descending
    idx_f = np.argsort(evals_fit)[::-1]
    idx_v = np.argsort(evals_val)[::-1]

    evals_fit = evals_fit[idx_f]
    evecs_fit = evecs_fit[:, idx_f]
    evals_val = evals_val[idx_v]
    evecs_val = evecs_val[:, idx_v]

    # Subspace alignment: angle between corresponding principal axes
    principal_angles_deg = []
    for k in range(3):
        cos_ang = np.clip(np.abs(np.dot(evecs_fit[:, k], evecs_val[:, k])), 0.0, 1.0)
        principal_angles_deg.append(float(math.degrees(math.acos(cos_ang))))

    results["check5_covariance_eigenvector_geometry"] = {
        "fit_eigenvalues_rad2_s2": evals_fit.tolist(),
        "fit_eigenvalues_deg2_s2": [float(v * (180/math.pi)**2) for v in evals_fit],
        "fit_eigenvectors": evecs_fit.tolist(),
        "val_eigenvalues_rad2_s2": evals_val.tolist(),
        "val_eigenvalues_deg2_s2": [float(v * (180/math.pi)**2) for v in evals_val],
        "val_eigenvectors": evecs_val.tolist(),
        "principal_axis_misalignment_angles_deg": principal_angles_deg,
    }

    # =========================================================================
    # Check 6: Finite-Window Relative Rotation (Hand-Eye) Estimator
    # =========================================================================
    print("--- Check 6: Finite-Window Relative Rotation Hand-Eye Estimator ---")
    # For windows of dt_steps = 20 (0.10s) or 40 (0.20s):
    # Delta R_FK(i, i+k) = R_FK(i)^-1 * R_FK(i+k)
    # Delta R_IMU(i, i+k) = Exp( \int_{t_i}^{t_{i+k}} gyro dt )
    # Relation: Delta R_FK = R_P_IP * Delta R_IMU * (R_P_IP)^T
    # Or in rotation vectors: r_FK = R_P_IP * r_IMU

    dt_step = 20  # ~100 ms interval
    
    def extract_relative_rotations(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        indices = np.flatnonzero(mask)
        r_fk_list = []
        r_imu_list = []
        for i_start in range(0, len(indices) - dt_step, dt_step // 2):
            i0 = indices[i_start]
            i1 = indices[i_start + dt_step]
            # Ensure consecutive in time
            if (t_eval[i1] - t_eval[i0]) > 0.20e9:
                continue
            # FK relative rotation
            R_rel_fk = rot_eval[i0].inv() * rot_eval[i1]
            rvec_fk = R_rel_fk.as_rotvec()

            # Integrated IMU gyro
            t0_ns = t_eval[i0]
            t1_ns = t_eval[i1]
            m_imu = (data["imu_stamps_ns"] >= t0_ns) & (data["imu_stamps_ns"] <= t1_ns)
            if np.sum(m_imu) < 5:
                continue
            t_imu_seg = data["imu_stamps_ns"][m_imu]
            g_imu_seg = gyro_corrected[m_imu]
            dt_imu = np.diff(t_imu_seg) * 1e-9
            # Trapezoidal integration of gyro
            angle_inc = 0.5 * (g_imu_seg[:-1] + g_imu_seg[1:]) * dt_imu[:, None]
            tot_rotvec_imu = np.sum(angle_inc, axis=0)

            if np.linalg.norm(rvec_fk) > 1e-4 and np.linalg.norm(tot_rotvec_imu) > 1e-4:
                r_fk_list.append(rvec_fk)
                r_imu_list.append(tot_rotvec_imu)

        return np.asarray(r_fk_list), np.asarray(r_imu_list)

    r_fk_fit, r_imu_fit = extract_relative_rotations(fit_mask)
    r_fk_val, r_imu_val = extract_relative_rotations(val_mask)

    R_handeye_fit = solve_kabsch(r_fk_fit, r_imu_fit)
    R_handeye_val = solve_kabsch(r_fk_val, r_imu_val)

    delta_handeye_refit = so3_delta_deg(R_handeye_fit, R_handeye_val)
    delta_handeye_vs_diff = so3_delta_deg(R_handeye_fit, R_fit_base)

    results["check6_finite_window_handeye_estimator"] = {
        "dt_window_ms": 100.0,
        "fit_pairs_count": len(r_fk_fit),
        "val_pairs_count": len(r_fk_val),
        "handeye_fit_euler_deg": Rotation.from_matrix(R_handeye_fit).as_euler("xyz", degrees=True).tolist(),
        "handeye_val_euler_deg": Rotation.from_matrix(R_handeye_val).as_euler("xyz", degrees=True).tolist(),
        "handeye_fit_vs_val_delta_deg": delta_handeye_refit,
        "handeye_vs_differentiated_omega_delta_deg": delta_handeye_vs_diff,
    }

    # =========================================================================
    # Check 7: Offline Kinematic Envelope Verification for E1C-2 Candidate
    # =========================================================================
    print("--- Check 7: Kinematic Envelope Verification for E1C-2 ---")
    kin = Analytical6PusKinematics()
    z0 = float(kin.nominal_height)

    # Proposed conservative excitation:
    # Roll +/-3.5 deg (0.061 rad), Pitch +/-3.5 deg (0.061 rad), Yaw +/-4.0 deg (0.070 rad)
    # Peak rotation norm: sqrt(3.5^2 + 3.5^2 + 4.0^2) = 6.36 deg = 0.111 rad (single axis <= 4 deg = 0.070 rad < 0.10 rad safety limit)
    # Multi-axis combined: Roll +/-2.5 deg, Pitch +/-2.5 deg, Yaw +/-3.0 deg @ 0.30 Hz
    # Peak multi-axis rotation = sqrt(2.5^2 + 2.5^2 + 3.0^2) = 4.64 deg = 0.081 rad < 0.10 rad envelope!

    test_trajectories = [
        {"name": "E1C-2 Single-Axis Roll", "r_deg": 3.5, "p_deg": 0.0, "y_deg": 0.0, "f_hz": 0.25},
        {"name": "E1C-2 Single-Axis Pitch", "r_deg": 0.0, "p_deg": 3.5, "y_deg": 0.0, "f_hz": 0.25},
        {"name": "E1C-2 Single-Axis Yaw", "r_deg": 0.0, "p_deg": 0.0, "y_deg": 4.0, "f_hz": 0.20},
        {"name": "E1C-2 Combined Multi-Axis", "r_deg": 2.5, "p_deg": 2.5, "y_deg": 3.0, "f_hz": 0.25},
    ]

    kin_verification = []
    for traj in test_trajectories:
        r_rad = math.radians(traj["r_deg"])
        p_rad = math.radians(traj["p_deg"])
        y_rad = math.radians(traj["y_deg"])
        rot_norm_rad = math.sqrt(r_rad**2 + p_rad**2 + y_rad**2)

        # Discretize 1 period at 100 steps
        t_vals = np.linspace(0, 1.0 / traj["f_hz"], 200)
        slider_positions = []
        slider_speeds = []
        dt = t_vals[1] - t_vals[0]

        for t in t_vals:
            r = r_rad * math.sin(2.0 * math.pi * traj["f_hz"] * t)
            p = p_rad * math.cos(2.0 * math.pi * traj["f_hz"] * t) if traj["p_deg"] > 0 else 0.0
            y = y_rad * math.sin(2.0 * math.pi * traj["f_hz"] * t + math.pi/4) if traj["y_deg"] > 0 else 0.0
            pose = [0.0, 0.0, z0, r, p, y]
            q = kin.inverse_kinematics(pose)
            slider_positions.append(q)

        slider_arr = np.asarray(slider_positions)  # Shape (200, 6)
        slider_steps = (slider_arr - 0.025) * 40000.0  # from 0 to 1000 relative to park
        slider_vel_mm_s = np.gradient(slider_arr * 1000.0, dt, axis=0)

        min_q_m = float(np.min(slider_arr))
        max_q_m = float(np.max(slider_arr))
        min_steps = float(np.min(slider_steps))
        max_steps = float(np.max(slider_steps))
        peak_speed_mm_s = float(np.max(np.abs(slider_vel_mm_s)))

        # Envelope checks
        envelope_rot_pass = rot_norm_rad < 0.10  # 0.10 rad safety limit
        stroke_pass = (min_q_m >= 0.0) and (max_q_m <= 0.050)
        speed_pass = peak_speed_mm_s <= 5.0  # 5.0 mm/s limit

        kin_verification.append({
            "trajectory": traj["name"],
            "max_rotation_norm_deg": float(math.degrees(rot_norm_rad)),
            "max_rotation_norm_rad": rot_norm_rad,
            "envelope_pass_0_10_rad": bool(envelope_rot_pass),
            "min_slider_m": min_q_m,
            "max_slider_m": max_q_m,
            "slider_steps_range": [round(min_steps, 1), round(max_steps, 1)],
            "stroke_pass_0_to_50mm": bool(stroke_pass),
            "peak_slider_speed_mm_s": peak_speed_mm_s,
            "speed_pass_5mm_s": bool(speed_pass),
            "all_safety_passed": bool(envelope_rot_pass and stroke_pass and speed_pass),
        })

    results["check7_e1c2_kinematic_safety_verification"] = kin_verification

    return results


if __name__ == "__main__":
    audit = compute_all_checks()
    out_file = DATA_DIR / "j9_e1c1c_dynamic_consistency_audit.json"
    out_file.write_text(json.dumps(audit, indent=2))
    print(f"Audit results written to {out_file}")
    print(json.dumps(audit, indent=2))
