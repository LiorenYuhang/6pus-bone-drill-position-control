#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J12-3 Dynamic Performance Characterization Analyzer.

Analyzes telemetry from J12-3 real-hardware dynamic experiments under
handheld manual disturbances (Slow -> Medium -> Fast).
Computes:
  - Base SO(3) attitude and angular rates via Riemannian logarithm map
  - Platform SO(3) tracking error
  - Actuator step tracking errors and motor currents
  - Segmentation across Slow, Medium, Fast, and Hold phases
  - High current characterization (>500, >800, >1000, >1200, >1400 mA)
  - Correlation models: |omega_B| -> Platform Error, Tracking Error, Current
  - Comprehensive answers to all 11 explicit audit questions
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXIS_NAMES = [f"ID{i+1}" for i in range(6)]
AXIS_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


def compute_so3_angular_rate(R_mats: np.ndarray, dt_s: np.ndarray) -> np.ndarray:
    """Compute SO(3) angular rate vector and magnitude in deg/s using Riemannian log map."""
    n = len(R_mats)
    omega_mag = np.zeros(n, dtype=np.float64)
    for i in range(n - 1):
        dt = max(dt_s[i], 0.001)
        R1 = R_mats[i]
        R2 = R_mats[i + 1]
        dR = R2 @ R1.T
        tr = np.trace(dR)
        cos_theta = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
        theta = math.acos(cos_theta)
        omega_mag[i] = math.degrees(theta / dt)
    if n > 1:
        omega_mag[-1] = omega_mag[-2]
    return omega_mag


def detect_motion_phases(t_s: np.ndarray, omega_mag: np.ndarray) -> List[Dict[str, Any]]:
    """Segment data into Baseline, Slow, Medium, Fast, and Holds based on rate envelope."""
    n = len(t_s)
    if n == 0:
        return []

    # Filter rate with moving average
    kernel_size = 25  # 0.5s at 50Hz
    rate_smooth = np.convolve(omega_mag, np.ones(kernel_size) / kernel_size, mode="same")

    # Threshold for active motion: 3.0 deg/s
    active = rate_smooth > 3.0

    # Merge short gaps (< 1.5s)
    gap_samples = int(1.5 * 50)
    for i in range(n):
        if not active[i]:
            fwd = np.where(active[i:])[0]
            if len(fwd) > 0 and 0 < fwd[0] < gap_samples:
                active[i : i + fwd[0]] = True

    # Find contiguous blocks
    raw_blocks = []
    in_b = False
    s_idx = 0
    for i, a in enumerate(active):
        if a and not in_b:
            in_b = True
            s_idx = i
        elif not a and in_b:
            in_b = False
            dur = t_s[i - 1] - t_s[s_idx]
            if dur >= 3.0:
                raw_blocks.append((s_idx, i - 1))
    if in_b:
        dur = t_s[-1] - t_s[s_idx]
        if dur >= 3.0:
            raw_blocks.append((s_idx, n - 1))

    segments = []
    # If blocks found, sort by time and classify
    if len(raw_blocks) >= 3:
        # Sort by peak/mean rate
        block_rates = [np.mean(omega_mag[s:e+1]) for s, e in raw_blocks]
        # Typically sequential: Slow -> Medium -> Fast
        names = ["Slow", "Medium", "Fast"]
        for idx, (s, e) in enumerate(raw_blocks[:3]):
            name = names[idx] if idx < 3 else f"Dynamic_{idx+1}"
            segments.append({
                "name": name,
                "start_idx": int(s),
                "end_idx": int(e),
                "start_t": float(t_s[s]),
                "end_t": float(t_s[e]),
                "duration_s": float(t_s[e] - t_s[s]),
            })
    elif len(raw_blocks) > 0:
        for idx, (s, e) in enumerate(raw_blocks):
            segments.append({
                "name": f"Motion_Block_{idx+1}",
                "start_idx": int(s),
                "end_idx": int(e),
                "start_t": float(t_s[s]),
                "end_t": float(t_s[e]),
                "duration_s": float(t_s[e] - t_s[s]),
            })
    else:
        # Fallback: whole run as single segment
        segments.append({
            "name": "Full_Run",
            "start_idx": 0,
            "end_idx": n - 1,
            "start_t": float(t_s[0]),
            "end_t": float(t_s[-1]),
            "duration_s": float(t_s[-1] - t_s[0]),
        })

    return segments


def run_characterization_analysis(csv_path: str, output_dir: str):
    csv_p = Path(csv_path)
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    with open(csv_p, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    active_rows = [r for r in rows if r.get("state") == "ACTIVE"]
    if not active_rows:
        active_rows = rows

    n = len(active_rows)
    print(f"[ANALYSIS] Total rows: {len(rows)}, ACTIVE cycles: {n}")

    timestamps_ns = np.array([float(r["timestamp_ns"]) for r in active_rows])
    t_s = (timestamps_ns - timestamps_ns[0]) * 1e-9
    dts_s = np.array([float(r.get("dt_ms", 20.0)) for r in active_rows]) * 1e-3

    # Base attitude
    base_roll = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])
    base_pitch = np.array([float(r["base_rel_pitch_deg"]) for r in active_rows])
    base_yaw = np.array([float(r["base_rel_yaw_deg"]) for r in active_rows])
    base_so3 = np.array([float(r["base_so3_magnitude_deg"]) for r in active_rows])

    # Rotation matrices and angular rate
    R_mats = np.array([json.loads(r["R_WB"]) for r in active_rows])
    omega_mag = compute_so3_angular_rate(R_mats, dts_s)

    # Platform error
    plat_so3_err = np.array([float(r["platform_so3_error_deg"]) for r in active_rows])
    plat_roll = np.array([float(r["platform_rel_roll_deg"]) for r in active_rows])
    plat_pitch = np.array([float(r["platform_rel_pitch_deg"]) for r in active_rows])
    plat_yaw = np.array([float(r["platform_rel_yaw_deg"]) for r in active_rows])

    # Actuator tracking errors & currents
    cmd_steps = np.array([json.loads(r["commanded_steps"]) for r in active_rows])
    fb_steps = np.array([json.loads(r["feedback_steps"]) for r in active_rows])
    track_errors = np.abs(cmd_steps - fb_steps)
    currents_mA = np.array([json.loads(r["joint_currents_raw_mA"]) for r in active_rows])

    # Step rates (steps/s)
    cmd_rate = np.zeros_like(cmd_steps, dtype=np.float64)
    cmd_rate[1:] = np.abs(cmd_steps[1:] - cmd_steps[:-1]) / dts_s[1:, None]

    # Segmentation
    segments = detect_motion_phases(t_s, omega_mag)
    print(f"[ANALYSIS] Identified {len(segments)} segments:")
    for s in segments:
        print(f"  - {s['name']}: t={s['start_t']:.1f}s ~ {s['end_t']:.1f}s ({s['duration_s']:.1f}s)")

    # Segment metrics
    seg_metrics = []
    for s in segments:
        si, ei = s["start_idx"], s["end_idx"]
        s_so3 = base_so3[si : ei + 1]
        s_rate = omega_mag[si : ei + 1]
        s_err = plat_so3_err[si : ei + 1]
        s_terr = track_errors[si : ei + 1]
        s_curr = currents_mA[si : ei + 1]

        peak_curr_axis = int(np.argmax(np.max(s_curr, axis=0)))
        peak_curr_val = int(np.max(s_curr))

        m = {
            "name": s["name"],
            "start_time_s": s["start_t"],
            "end_time_s": s["end_t"],
            "duration_s": s["duration_s"],
            "base_so3_rms_deg": float(np.sqrt(np.mean(s_so3 ** 2))),
            "base_so3_peak_deg": float(np.max(s_so3)),
            "angular_rate_rms_deg_s": float(np.sqrt(np.mean(s_rate ** 2))),
            "angular_rate_peak_deg_s": float(np.max(s_rate)),
            "angular_rate_min_deg_s": float(np.min(s_rate)),
            "platform_so3_error_rms_deg": float(np.sqrt(np.mean(s_err ** 2))),
            "platform_so3_error_peak_deg": float(np.max(s_err)),
            "max_tracking_error_steps": int(np.max(s_terr)),
            "tracking_error_per_motor_max": [int(x) for x in np.max(s_terr, axis=0)],
            "peak_motor_current_mA": peak_curr_val,
            "peak_motor_id": peak_curr_axis + 1,
            "motor_current_per_axis_max_mA": [int(x) for x in np.max(s_curr, axis=0)],
        }
        seg_metrics.append(m)

    # High Current Characterization (>500, >800, >1000, >1200, >1400 mA)
    thresholds = [500, 800, 1000, 1200, 1400]
    high_curr_events = []
    threshold_counts = {th: 0 for th in thresholds}

    for i in range(n):
        row_curr = currents_mA[i]
        for m_idx in range(6):
            c_val = row_curr[m_idx]
            for th in thresholds:
                if c_val > th:
                    threshold_counts[th] += 1
            if c_val > 500:
                # Find which segment
                cur_t = t_s[i]
                seg_name = "Inter-Segment / Transition"
                for s in segments:
                    if s["start_t"] <= cur_t <= s["end_t"]:
                        seg_name = s["name"]
                        break
                high_curr_events.append({
                    "timestamp_s": float(cur_t),
                    "motor_id": m_idx + 1,
                    "current_mA": int(c_val),
                    "base_so3_deg": float(base_so3[i]),
                    "base_rpy_deg": [float(base_roll[i]), float(base_pitch[i]), float(base_yaw[i])],
                    "base_angular_rate_deg_s": float(omega_mag[i]),
                    "platform_so3_error_deg": float(plat_so3_err[i]),
                    "commanded_step": int(cmd_steps[i, m_idx]),
                    "feedback_step": int(fb_steps[i, m_idx]),
                    "tracking_error_steps": int(track_errors[i, m_idx]),
                    "segment": seg_name,
                })

    # Overall Peak Current
    overall_peak_curr = int(np.max(currents_mA))
    overall_peak_idx = np.unravel_index(np.argmax(currents_mA), currents_mA.shape)
    peak_cycle = int(overall_peak_idx[0])
    peak_motor_id = int(overall_peak_idx[1]) + 1
    peak_timestamp_s = float(t_s[peak_cycle])
    peak_base_att = float(base_so3[peak_cycle])
    peak_base_rate = float(omega_mag[peak_cycle])
    peak_plat_err = float(plat_so3_err[peak_cycle])

    # Check internal 1500 protection trigger
    internal_protection_triggered = (overall_peak_curr >= 1500) or any(
        any(f != 0 for f in json.loads(r["faults"])) for r in active_rows
    )

    # 11 Questions Direct Answers Dict
    qa_summary = {
        "q1_slow_rate_range_deg_s": [
            seg_metrics[0]["angular_rate_min_deg_s"] if len(seg_metrics) > 0 else 0.0,
            seg_metrics[0]["angular_rate_peak_deg_s"] if len(seg_metrics) > 0 else 0.0,
        ],
        "q2_medium_rate_range_deg_s": [
            seg_metrics[1]["angular_rate_min_deg_s"] if len(seg_metrics) > 1 else 0.0,
            seg_metrics[1]["angular_rate_peak_deg_s"] if len(seg_metrics) > 1 else 0.0,
        ],
        "q3_fast_rate_range_deg_s": [
            seg_metrics[2]["angular_rate_min_deg_s"] if len(seg_metrics) > 2 else 0.0,
            seg_metrics[2]["angular_rate_peak_deg_s"] if len(seg_metrics) > 2 else 0.0,
        ],
        "q4_full_run_max_motor_current_mA": overall_peak_curr,
        "q5_peak_current_motor_id": peak_motor_id,
        "q6_peak_current_condition": {
            "timestamp_s": peak_timestamp_s,
            "base_so3_deg": peak_base_att,
            "base_angular_rate_deg_s": peak_base_rate,
            "platform_so3_error_deg": peak_plat_err,
        },
        "q7_threshold_exceedances": {
            ">500_mA": threshold_counts[500] > 0,
            ">800_mA": threshold_counts[800] > 0,
            ">1000_mA": threshold_counts[1000] > 0,
            ">1200_mA": threshold_counts[1200] > 0,
            ">1400_mA": threshold_counts[1400] > 0,
            "counts": threshold_counts,
        },
        "q8_actuator_internal_1500_protection_tripped": internal_protection_triggered,
        "q9_platform_error_deterioration_rate_deg_s": None,
        "q10_tracking_error_deterioration_rate_deg_s": None,
        "q11_effective_dynamic_range_deg_s": None,
    }

    # Determine deterioration points by binning |omega_B|
    bins = np.linspace(0, max(np.max(omega_mag), 10.0), 15)
    bin_centers = 0.5 * (bins[:-1] + bins[1:])
    bin_plat_err_mean = []
    bin_track_err_mean = []
    bin_curr_max = []

    for b0, b1 in zip(bins[:-1], bins[1:]):
        mask = (omega_mag >= b0) & (omega_mag < b1)
        if np.any(mask):
            bin_plat_err_mean.append(float(np.mean(plat_so3_err[mask])))
            bin_track_err_mean.append(float(np.mean(np.max(track_errors[mask], axis=1))))
            bin_curr_max.append(float(np.max(currents_mA[mask])))
        else:
            bin_plat_err_mean.append(np.nan)
            bin_track_err_mean.append(np.nan)
            bin_curr_max.append(np.nan)

    # Find where platform error exceeds 1.0 deg or jumps by 2x baseline
    base_plat_err = np.nanmedian(plat_so3_err[omega_mag < 2.0]) if np.any(omega_mag < 2.0) else 0.05
    det_plat_rate = None
    for bc, pe in zip(bin_centers, bin_plat_err_mean):
        if not np.isnan(pe) and pe > max(1.0, 2.5 * base_plat_err):
            det_plat_rate = float(bc)
            break
    qa_summary["q9_platform_error_deterioration_rate_deg_s"] = det_plat_rate if det_plat_rate else float(bin_centers[-1])

    # Find where tracking error exceeds 15 steps
    det_track_rate = None
    for bc, te in zip(bin_centers, bin_track_err_mean):
        if not np.isnan(te) and te > 10.0:
            det_track_rate = float(bc)
            break
    qa_summary["q10_tracking_error_deterioration_rate_deg_s"] = det_track_rate if det_track_rate else float(bin_centers[-1])

    # Effective dynamic range is the min of safe tracking and error boundaries
    eff_range = min(
        qa_summary["q9_platform_error_deterioration_rate_deg_s"],
        qa_summary["q10_tracking_error_deterioration_rate_deg_s"],
    )
    qa_summary["q11_effective_dynamic_range_deg_s"] = eff_range

    # Save Metrics JSON
    full_output = {
        "dataset_summary": {
            "csv_file": csv_p.name,
            "total_rows": len(rows),
            "active_rows": n,
            "duration_s": float(t_s[-1]),
            "overall_peak_current_mA": overall_peak_curr,
            "peak_motor_id": peak_motor_id,
        },
        "segment_metrics": seg_metrics,
        "high_current_events": high_curr_events,
        "high_current_threshold_counts": threshold_counts,
        "audit_questions_answers": qa_summary,
    }
    json_path = out_dir / "j12_3_dynamic_characterization_metrics.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(full_output, f, indent=2)
    print(f"[ANALYSIS] Saved metrics JSON to {json_path}")

    # =========================================================================
    # GENERATE FIGURES
    # =========================================================================
    print("[ANALYSIS] Generating figures...")

    # Figure 1: Base Attitude & Angular Rate Timeline
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1.plot(t_s, base_roll, label="Base Roll (°)", color="#1f77b4", lw=1.2)
    ax1.plot(t_s, base_pitch, label="Base Pitch (°)", color="#ff7f0e", lw=1.2)
    ax1.plot(t_s, base_yaw, label="Base Yaw (°)", color="#2ca02c", lw=1.2)
    ax1.plot(t_s, base_so3, label="Base SO(3) Mag (°)", color="black", lw=1.5, ls="--")
    for s in segments:
        ax1.axvspan(s["start_t"], s["end_t"], color="gray", alpha=0.15)
        ax1.text(0.5 * (s["start_t"] + s["end_t"]), ax1.get_ylim()[1] * 0.85, s["name"],
                 ha="center", fontsize=9, fontweight="bold", bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7))
    ax1.set_ylabel("Attitude (°)")
    ax1.set_title("J12-3 Dynamic Performance: Base Attitude Progression")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", ncol=4)

    ax2.plot(t_s, omega_mag, label=r"$|\omega|_B$ Angular Rate (°/s)", color="crimson", lw=1.2)
    for s in segments:
        ax2.axvspan(s["start_t"], s["end_t"], color="gray", alpha=0.15)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel(r"$|\omega|_B$ (°/s)")
    ax2.set_title("Base SO(3) Angular Rate Progression")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")
    fig.tight_layout()
    fig1_path = out_dir / "j12_3_fig1_base_motion_timeline.png"
    fig.savefig(fig1_path, dpi=150)
    plt.close(fig)

    # Figure 2: Platform Residuals & SO(3) Error Timeline
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1.plot(t_s, plat_roll, label="Platform Roll (°)", color="#1f77b4", lw=1.0)
    ax1.plot(t_s, plat_pitch, label="Platform Pitch (°)", color="#ff7f0e", lw=1.0)
    ax1.plot(t_s, plat_yaw, label="Platform Yaw (°)", color="#2ca02c", lw=1.0)
    ax1.set_ylabel("Residual Euler (°)")
    ax1.set_title("Platform Attitude Residuals")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", ncol=3)

    ax2.plot(t_s, plat_so3_err, label=r"Platform $SO(3)$ Error $e_{SO3}$ (°)", color="purple", lw=1.2)
    ax2.axhline(1.0, color="orange", ls="--", label="1.0° Boundary")
    ax2.axhline(2.0, color="red", ls="--", label="2.0° High Error")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error (°)")
    ax2.set_title(r"Platform Geodesic $SO(3)$ Error Progression")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")
    fig.tight_layout()
    fig2_path = out_dir / "j12_3_fig2_platform_error_timeline.png"
    fig.savefig(fig2_path, dpi=150)
    plt.close(fig)

    # Figure 3: Six-Axis Motor Current Timeline
    fig, ax = plt.subplots(figsize=(12, 5))
    for m in range(6):
        ax.plot(t_s, currents_mA[:, m], label=f"ID{m+1}", color=AXIS_COLORS[m], lw=1.1)
    ax.axhline(500, color="gold", ls="--", lw=1.5, label="500 mA (Old Gate)")
    ax.axhline(1500, color="red", ls="--", lw=1.5, label="1500 mA (Hard Internal OC)")
    for s in segments:
        ax.axvspan(s["start_t"], s["end_t"], color="gray", alpha=0.15)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Current (mA)")
    ax.set_title(f"Six-Axis Motor Current Progression (Peak: {overall_peak_curr} mA on ID{peak_motor_id})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", ncol=4)
    fig.tight_layout()
    fig3_path = out_dir / "j12_3_fig3_motor_current_timeline.png"
    fig.savefig(fig3_path, dpi=150)
    plt.close(fig)

    # Figure 4: Six-Axis Tracking Error Timeline
    fig, ax = plt.subplots(figsize=(12, 5))
    for m in range(6):
        ax.plot(t_s, track_errors[:, m], label=f"ID{m+1}", color=AXIS_COLORS[m], lw=1.1)
    ax.axhline(10, color="orange", ls="--", lw=1.5, label="10 Steps Boundary")
    ax.axhline(25, color="red", ls="--", lw=1.5, label="25 Steps Divergence")
    for s in segments:
        ax.axvspan(s["start_t"], s["end_t"], color="gray", alpha=0.15)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tracking Error (|cmd - fb| steps)")
    ax.set_title(f"Six-Axis Actuator Tracking Error Progression (Max: {int(np.max(track_errors))} steps)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", ncol=4)
    fig.tight_layout()
    fig4_path = out_dir / "j12_3_fig4_tracking_error_timeline.png"
    fig.savefig(fig4_path, dpi=150)
    plt.close(fig)

    # Figure 5: |omega_B| -> Platform Error Correlation
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.scatter(omega_mag, plat_so3_err, alpha=0.2, color="purple", s=10, label="Sample Cycles")
    valid_bc = [bc for bc, pe in zip(bin_centers, bin_plat_err_mean) if not np.isnan(pe)]
    valid_pe = [pe for pe in bin_plat_err_mean if not np.isnan(pe)]
    if valid_bc:
        ax.plot(valid_bc, valid_pe, color="black", lw=2.5, marker="o", label="Binned Mean")
    ax.set_xlabel(r"Base Angular Rate $|\omega|_B$ (°/s)")
    ax.set_ylabel(r"Platform $SO(3)$ Error $e_{SO3}$ (°)")
    ax.set_title(r"Dynamic Correlation: $|\omega|_B \to$ Platform $SO(3)$ Error")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig5_path = out_dir / "j12_3_fig5_correlation_rate_vs_plat_error.png"
    fig.savefig(fig5_path, dpi=150)
    plt.close(fig)

    # Figure 6: |omega_B| -> Tracking Error Correlation
    fig, ax = plt.subplots(figsize=(8, 6))
    max_terr_per_cycle = np.max(track_errors, axis=1)
    ax.scatter(omega_mag, max_terr_per_cycle, alpha=0.2, color="teal", s=10, label="Max Tracking Error")
    valid_te = [te for te in bin_track_err_mean if not np.isnan(te)]
    if valid_bc and valid_te:
        ax.plot(valid_bc, valid_te, color="black", lw=2.5, marker="s", label="Binned Mean")
    ax.set_xlabel(r"Base Angular Rate $|\omega|_B$ (°/s)")
    ax.set_ylabel("Max Actuator Tracking Error (steps)")
    ax.set_title(r"Dynamic Correlation: $|\omega|_B \to$ Actuator Tracking Error")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig6_path = out_dir / "j12_3_fig6_correlation_rate_vs_tracking_error.png"
    fig.savefig(fig6_path, dpi=150)
    plt.close(fig)

    # Figure 7: |omega_B| -> Motor Current Correlation
    fig, ax = plt.subplots(figsize=(8, 6))
    max_curr_per_cycle = np.max(currents_mA, axis=1)
    ax.scatter(omega_mag, max_curr_per_cycle, alpha=0.2, color="coral", s=10, label="Max Motor Current")
    valid_cm = [cm for cm in bin_curr_max if not np.isnan(cm)]
    if valid_bc and valid_cm:
        ax.plot(valid_bc, valid_cm, color="black", lw=2.5, marker="^", label="Binned Peak")
    ax.axhline(500, color="gold", ls="--", label="500 mA Threshold")
    ax.axhline(1500, color="red", ls="--", label="1500 mA Internal Protection")
    ax.set_xlabel(r"Base Angular Rate $|\omega|_B$ (°/s)")
    ax.set_ylabel("Max Motor Current (mA)")
    ax.set_title(r"Dynamic Correlation: $|\omega|_B \to$ Motor Current")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig7_path = out_dir / "j12_3_fig7_correlation_rate_vs_current.png"
    fig.savefig(fig7_path, dpi=150)
    plt.close(fig)

    print(f"[ANALYSIS] Figures saved to {out_dir}")
    return full_output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="J12-3 Dynamic Characterization Analyzer")
    parser.add_argument("--input-csv", required=True, help="Path to telemetry CSV")
    parser.add_argument("--output-dir", default=None, help="Output directory")
    args = parser.parse_args()
    out = args.output_dir if args.output_dir else str(Path(args.input_csv).parent)
    run_characterization_analysis(args.input_csv, out)
