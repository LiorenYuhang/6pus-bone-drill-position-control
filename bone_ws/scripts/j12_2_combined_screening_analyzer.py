#!/usr/bin/env python3
"""Stage J12-2: Combined Quasi-Static Attitude Screening Offline Analyzer.

Processes telemetry from Feedforward-only combined attitude screening runs:
Cases:
  - Baseline 0: Roll ≈ 0°, Pitch ≈ 0°, Yaw ≈ 0°
  - Case A: Base Roll ≈ +15°, Pitch ≈ +10°, Yaw ≈ +5°
  - Return 0A: Roll ≈ 0°, Pitch ≈ 0°, Yaw ≈ 0°
  - Case B: Base Roll ≈ +15°, Pitch ≈ -10°, Yaw ≈ -5°
  - Case C: Base Roll ≈ -15°, Pitch ≈ +10°, Yaw ≈ -5°
  - Case D: Base Roll ≈ -15°, Pitch ≈ -10°, Yaw ≈ +5°
  - Return 0D: Roll ≈ 0°, Pitch ≈ 0°, Yaw ≈ 0°

Generates 7 standard figures:
  Figure 1: 3-DOF Base Attitude Progression (Roll, Pitch, Yaw, SO3) with Plateau Labels
  Figure 2: 3-DOF Platform Attitude Tracking Error Progression
  Figure 3: Six-Axis Actuator Feedback Steps Progression
  Figure 4: Six-Axis Tracking Error Progression (with per-case max annotations)
  Figure 5: Six-Axis Motor Current Progression (with Global Peak annotation & 400/500mA limits)
  Figure 6: Per-Case Steady-State Current Comparison
  Figure 7: Per-Case Actuator Stroke & Tracking Error Envelope
"""

import argparse
import csv
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


AXIS_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
AXIS_NAMES = [f"ID{i+1}" for i in range(6)]

STANDARD_PRESET_INTERVALS = [
    ("Baseline 0", 0.5, 3.0),
    ("Case A (+, +, +)", 15.5, 17.5),
    ("Return 0A", 23.0, 24.5),
    ("Case B (+, -, -)", 32.0, 34.5),
    ("Case C (-, +, -)", 46.0, 49.0),
    ("Case D (-, -, +)", 54.5, 58.5),
    ("Return 0D", 65.0, 69.5),
]


def parse_telemetry_csv(csv_path: str) -> List[Dict[str, Any]]:
    rows = []
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def classify_plateau(mean_r: float, mean_p: float, mean_y: float) -> str:
    """Classify plateau into Case A/B/C/D or Baseline based on angles."""
    # Check baseline (near zero)
    if abs(mean_r) < 6.0 and abs(mean_p) < 6.0 and abs(mean_y) < 6.0:
        return "Baseline / Return 0"
    
    # Check Case A (+ / + / +)
    if mean_r > 5.0 and mean_p > 3.0:
        return "Case A (+15°, +10°, +5°)"
    # Check Case B (+ / - / -)
    if mean_r > 5.0 and mean_p < -3.0:
        return "Case B (+15°, -10°, -5°)"
    # Check Case C (- / + / -)
    if mean_r < -5.0 and mean_p > 3.0:
        return "Case C (-15°, +10°, -5°)"
    # Check Case D (- / - / +)
    if mean_r < -5.0 and mean_p < -3.0:
        return "Case D (-15°, -10°, +5°)"
    
    return f"Plateau (R={mean_r:+.1f}°, P={mean_p:+.1f}°, Y={mean_y:+.1f}°)"


def detect_plateaus_preset(
    active_rows: List[Dict[str, Any]],
    t_s: np.ndarray,
) -> List[Dict[str, Any]]:
    """Segment telemetry using known validated physical screening windows."""
    segmented = []
    for p_idx, (name, t_start, t_end) in enumerate(STANDARD_PRESET_INTERVALS):
        # find matching rows
        idx_mask = (t_s >= t_start) & (t_s <= t_end)
        indices = np.where(idx_mask)[0]
        if len(indices) == 0:
            continue
        s_idx, e_idx = int(indices[0]), int(indices[-1])
        p_rows = active_rows[s_idx : e_idx + 1]
        
        rolls = [float(r["base_rel_roll_deg"]) for r in p_rows]
        pitches = [float(r["base_rel_pitch_deg"]) for r in p_rows]
        yaws = [float(r["base_rel_yaw_deg"]) for r in p_rows]

        segmented.append({
            "plateau_index": p_idx + 1,
            "case_name": name,
            "start_time_s": float(t_s[s_idx]),
            "end_time_s": float(t_s[e_idx]),
            "start_idx": s_idx,
            "end_idx": e_idx,
            "duration_s": float(t_s[e_idx] - t_s[s_idx]),
            "sample_count": len(p_rows),
            "mean_roll_deg": round(float(np.mean(rolls)), 2),
            "mean_pitch_deg": round(float(np.mean(pitches)), 2),
            "mean_yaw_deg": round(float(np.mean(yaws)), 2),
            "rows": p_rows,
        })
    return segmented


def detect_plateaus_auto(
    active_rows: List[Dict[str, Any]],
    t_s: np.ndarray,
    min_duration_s: float = 1.8,
    rate_threshold_deg_s: float = 3.0,
) -> List[Dict[str, Any]]:
    """Segment telemetry into quasi-static plateaus using 3-DOF angular rate."""
    rolls = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])
    pitches = np.array([float(r["base_rel_pitch_deg"]) for r in active_rows])
    yaws = np.array([float(r["base_rel_yaw_deg"]) for r in active_rows])

    dt = np.diff(t_s)
    dt[dt <= 0] = 0.020
    
    # Compute 3-DOF rate magnitude: sqrt(rdot^2 + pdot^2 + ydot^2)
    dr_dt = np.diff(rolls) / dt
    dp_dt = np.diff(pitches) / dt
    dy_dt = np.diff(yaws) / dt
    rate_3d = np.sqrt(dr_dt**2 + dp_dt**2 + dy_dt**2)
    rate_3d = np.append(rate_3d, rate_3d[-1])

    # Moving average filter
    kernel_size = 5
    rate_smooth = np.convolve(rate_3d, np.ones(kernel_size) / kernel_size, mode="same")
    is_static = rate_smooth < rate_threshold_deg_s

    # Merge short gaps < 0.35s caused by minor tremor
    for i in range(len(is_static)):
        if not is_static[i]:
            fwd = np.where(is_static[i:])[0]
            if len(fwd) > 0 and fwd[0] > 0:
                gap_dur = t_s[i + fwd[0]] - t_s[i]
                if gap_dur < 0.35:
                    is_static[i : i + fwd[0]] = True

    plateaus = []
    in_plateau = False
    start_idx = 0

    for i, static in enumerate(is_static):
        if static and not in_plateau:
            in_plateau = True
            start_idx = i
        elif not static and in_plateau:
            in_plateau = False
            dur = t_s[i - 1] - t_s[start_idx]
            if dur >= min_duration_s:
                plateaus.append((start_idx, i - 1, dur))

    if in_plateau:
        dur = t_s[-1] - t_s[start_idx]
        if dur >= min_duration_s:
            plateaus.append((start_idx, len(is_static) - 1, dur))

    segmented_results = []
    for p_idx, (s_idx, e_idx, dur) in enumerate(plateaus):
        p_rows = active_rows[s_idx : e_idx + 1]
        p_rolls = [float(r["base_rel_roll_deg"]) for r in p_rows]
        p_pitches = [float(r["base_rel_pitch_deg"]) for r in p_rows]
        p_yaws = [float(r["base_rel_yaw_deg"]) for r in p_rows]

        mean_r = float(np.mean(p_rolls))
        mean_p = float(np.mean(p_pitches))
        mean_y = float(np.mean(p_yaws))
        case_name = classify_plateau(mean_r, mean_p, mean_y)

        segmented_results.append({
            "plateau_index": p_idx + 1,
            "case_name": case_name,
            "start_time_s": float(t_s[s_idx]),
            "end_time_s": float(t_s[e_idx]),
            "start_idx": s_idx,
            "end_idx": e_idx,
            "duration_s": float(dur),
            "sample_count": len(p_rows),
            "mean_roll_deg": round(mean_r, 2),
            "mean_pitch_deg": round(mean_p, 2),
            "mean_yaw_deg": round(mean_y, 2),
            "rows": p_rows,
        })

    return segmented_results


def compute_plateau_metrics(p_data: Dict[str, Any]) -> Dict[str, Any]:
    rows = p_data["rows"]
    n = len(rows)

    base_r = np.array([float(r["base_rel_roll_deg"]) for r in rows])
    base_p = np.array([float(r["base_rel_pitch_deg"]) for r in rows])
    base_y = np.array([float(r["base_rel_yaw_deg"]) for r in rows])
    base_so3 = np.array([float(r.get("base_so3_magnitude_deg", 0.0)) for r in rows])

    cmd_r = np.array([float(r.get("cmd_roll_deg", 0.0)) for r in rows])
    cmd_p = np.array([float(r.get("cmd_pitch_deg", 0.0)) for r in rows])
    cmd_y = np.array([float(r.get("cmd_yaw_deg", 0.0)) for r in rows])

    plat_r = np.array([float(r.get("platform_rel_roll_deg", 0.0)) for r in rows])
    plat_p = np.array([float(r.get("platform_rel_pitch_deg", 0.0)) for r in rows])
    plat_y = np.array([float(r.get("platform_rel_yaw_deg", 0.0)) for r in rows])
    plat_so3 = np.array([float(r.get("platform_so3_error_deg", 0.0)) for r in rows])
    plat_tilt = np.array([float(r.get("platform_tilt_error_deg", 0.0)) for r in rows])

    cmd_steps = np.array([json.loads(r["commanded_steps"]) for r in rows])
    fb_steps = np.array([json.loads(r["feedback_steps"]) for r in rows])
    err_steps = np.array([json.loads(r["command_feedback_errors"]) for r in rows])
    currents = np.array([json.loads(r["joint_currents_raw_mA"]) for r in rows])
    dts = np.array([float(r.get("loop_dt_ms", 20.0)) for r in rows])
    fb_ages = np.array([float(r.get("driver_feedback_age_ms", 0.0)) for r in rows])

    axis_metrics = {}
    for i in range(6):
        c = currents[:, i]
        e = err_steps[:, i]
        s_cmd = cmd_steps[:, i]
        s_fb = fb_steps[:, i]

        axis_metrics[f"ID{i+1}"] = {
            "cmd_steps_mean": float(np.mean(s_cmd)),
            "cmd_steps_min": int(np.min(s_cmd)),
            "cmd_steps_max": int(np.max(s_cmd)),
            "fb_steps_mean": float(np.mean(s_fb)),
            "fb_steps_min": int(np.min(s_fb)),
            "fb_steps_max": int(np.max(s_fb)),
            "tracking_error_mean": float(np.mean(e)),
            "tracking_error_max": int(np.max(np.abs(e))),
            "current_instantaneous_mA": int(c[-1]),
            "current_mean_mA": float(np.mean(c)),
            "current_median_mA": float(np.median(c)),
            "current_max_mA": int(np.max(c)),
            "current_p95_mA": float(np.percentile(c, 95)),
            "current_p99_mA": float(np.percentile(c, 99)),
        }

    return {
        "plateau_index": p_data["plateau_index"],
        "case_name": p_data["case_name"],
        "start_time_s": p_data["start_time_s"],
        "end_time_s": p_data["end_time_s"],
        "duration_s": p_data["duration_s"],
        "sample_count": n,
        "base_attitude": {
            "roll_mean_deg": float(np.mean(base_r)),
            "roll_std_deg": float(np.std(base_r)),
            "pitch_mean_deg": float(np.mean(base_p)),
            "pitch_std_deg": float(np.std(base_p)),
            "yaw_mean_deg": float(np.mean(base_y)),
            "yaw_std_deg": float(np.std(base_y)),
            "so3_mean_deg": float(np.mean(base_so3)),
            "so3_max_deg": float(np.max(base_so3)),
        },
        "feedforward_command": {
            "roll_mean_deg": float(np.mean(cmd_r)),
            "pitch_mean_deg": float(np.mean(cmd_p)),
            "yaw_mean_deg": float(np.mean(cmd_y)),
        },
        "platform_attitude": {
            "roll_error_mean_deg": float(np.mean(plat_r)),
            "pitch_error_mean_deg": float(np.mean(plat_p)),
            "yaw_error_mean_deg": float(np.mean(plat_y)),
            "so3_error_mean_deg": float(np.mean(plat_so3)),
            "so3_error_max_deg": float(np.max(plat_so3)),
            "tilt_error_mean_deg": float(np.mean(plat_tilt)),
        },
        "axis_metrics": axis_metrics,
        "timing": {
            "dt_mean_ms": float(np.mean(dts)),
            "dt_max_ms": float(np.max(dts)),
            "feedback_age_mean_ms": float(np.mean(fb_ages)),
            "feedback_age_max_ms": float(np.max(fb_ages)),
        },
    }


def generate_all_figures(
    rows: List[Dict[str, Any]],
    plateaus: List[Dict[str, Any]],
    metrics_list: List[Dict[str, Any]],
    out_dir: Path,
) -> None:
    """Generate Figures 1 through 7."""
    active_rows = [r for r in rows if r.get("state") == "ACTIVE"]
    if not active_rows:
        active_rows = rows

    timestamps = np.array([float(r["timestamp_ns"]) for r in active_rows])
    t_s = (timestamps - timestamps[0]) * 1e-9

    base_r = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])
    base_p = np.array([float(r["base_rel_pitch_deg"]) for r in active_rows])
    base_y = np.array([float(r["base_rel_yaw_deg"]) for r in active_rows])
    base_so3 = np.array([float(r.get("base_so3_magnitude_deg", 0.0)) for r in active_rows])

    plat_r = np.array([float(r.get("platform_rel_roll_deg", 0.0)) for r in active_rows])
    plat_p = np.array([float(r.get("platform_rel_pitch_deg", 0.0)) for r in active_rows])
    plat_y = np.array([float(r.get("platform_rel_yaw_deg", 0.0)) for r in active_rows])
    plat_so3 = np.array([float(r.get("platform_so3_error_deg", 0.0)) for r in active_rows])

    fb_steps = np.array([json.loads(r["feedback_steps"]) for r in active_rows])
    err_steps = np.array([json.loads(r["command_feedback_errors"]) for r in active_rows])
    currents = np.array([json.loads(r["joint_currents_raw_mA"]) for r in active_rows])

    plt.rcParams["font.sans-serif"] = "DejaVu Sans"
    plt.rcParams["axes.edgecolor"] = "#333333"

    def add_plateau_shading(ax, y_offset_ratio=0.10):
        for p in plateaus:
            t0, t1 = p["start_time_s"], p["end_time_s"]
            name = p["case_name"]
            c = "lightgoldenrodyellow" if "Baseline" in name or "Return" in name else "lightcyan"
            ax.axvspan(t0, t1, color=c, alpha=0.35)
            mid = (t0 + t1) / 2.0
            y_lim = ax.get_ylim()
            y_pos = y_lim[1] - y_offset_ratio * (y_lim[1] - y_lim[0])
            lbl = name.split()[0]
            ax.text(mid, y_pos, lbl, ha="center", va="top", fontsize=8, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.75, edgecolor="gray", lw=0.6))

    # ==========================================================
    # Figure 1: 3-DOF Base Attitude Progression
    # ==========================================================
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.8), sharex=True, dpi=150)
    axs[0].plot(t_s, base_r, color="#1f77b4", lw=1.6, label="Base Roll")
    axs[0].set_ylabel("Roll (°)", fontsize=10)
    axs[0].grid(True, ls="--", alpha=0.5)
    axs[0].legend(loc="upper left")

    axs[1].plot(t_s, base_p, color="#ff7f0e", lw=1.6, label="Base Pitch")
    axs[1].set_ylabel("Pitch (°)", fontsize=10)
    axs[1].grid(True, ls="--", alpha=0.5)
    axs[1].legend(loc="upper left")

    axs[2].plot(t_s, base_y, color="#2ca02c", lw=1.6, label="Base Yaw")
    axs[2].set_ylabel("Yaw (°)", fontsize=10)
    axs[2].grid(True, ls="--", alpha=0.5)
    axs[2].legend(loc="upper left")

    axs[3].plot(t_s, base_so3, color="#d62728", lw=1.6, label="Base SO(3)")
    axs[3].set_ylabel("SO(3) (°)", fontsize=10)
    axs[3].set_xlabel("Time (s)", fontsize=10)
    axs[3].grid(True, ls="--", alpha=0.5)
    axs[3].legend(loc="upper left")

    for ax in axs:
        add_plateau_shading(ax, y_offset_ratio=0.15)

    fig.suptitle("Figure 1: 3-DOF Base Attitude Progression (Quasi-Static Screening)", fontsize=13, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_2_figure_1_base_attitude_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 2: 3-DOF Platform Attitude Tracking Error Progression
    # ==========================================================
    fig, axs = plt.subplots(4, 1, figsize=(11, 8.8), sharex=True, dpi=150)
    axs[0].plot(t_s, plat_r, color="#1f77b4", lw=1.6, label="Platform Roll Error")
    axs[0].axhline(0, color="gray", ls=":")
    axs[0].set_ylabel("Roll Err (°)", fontsize=10)
    axs[0].grid(True, ls="--", alpha=0.5)
    axs[0].legend(loc="upper left")

    axs[1].plot(t_s, plat_p, color="#ff7f0e", lw=1.6, label="Platform Pitch Error")
    axs[1].axhline(0, color="gray", ls=":")
    axs[1].set_ylabel("Pitch Err (°)", fontsize=10)
    axs[1].grid(True, ls="--", alpha=0.5)
    axs[1].legend(loc="upper left")

    axs[2].plot(t_s, plat_y, color="#2ca02c", lw=1.6, label="Platform Yaw Error")
    axs[2].axhline(0, color="gray", ls=":")
    axs[2].set_ylabel("Yaw Err (°)", fontsize=10)
    axs[2].grid(True, ls="--", alpha=0.5)
    axs[2].legend(loc="upper left")

    axs[3].plot(t_s, plat_so3, color="#d62728", lw=1.6, label="Platform SO(3) Error")
    axs[3].set_ylabel("SO(3) Err (°)", fontsize=10)
    axs[3].set_xlabel("Time (s)", fontsize=10)
    axs[3].grid(True, ls="--", alpha=0.5)
    axs[3].legend(loc="upper left")

    for ax in axs:
        add_plateau_shading(ax, y_offset_ratio=0.15)

    fig.suptitle("Figure 2: 3-DOF Platform Attitude Residual / Tracking Error Progression", fontsize=13, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_2_figure_2_platform_error_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 3: Six-Axis Actuator Feedback Steps Progression
    # ==========================================================
    fig, ax = plt.subplots(figsize=(11, 5.5), dpi=150)
    for i in range(6):
        ax.plot(t_s, fb_steps[:, i], color=AXIS_COLORS[i], lw=1.5, label=f"ID{i+1}")
    ax.axhline(1000, color="black", ls=":", lw=1.2, label="Home (1000)")
    ax.axhline(500, color="red", ls="--", lw=1.2, alpha=0.8, label="Compression Warning Limit (500)")
    add_plateau_shading(ax, y_offset_ratio=0.08)
    ax.set_title("Figure 3: Six-Axis Actuator Feedback Steps Progression", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Actuator Steps", fontsize=10)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper left", ncol=4, framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_2_figure_3_actuator_steps_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 4: Six-Axis Tracking Error Progression with Max Annotation
    # ==========================================================
    fig, ax = plt.subplots(figsize=(11, 5.8), dpi=150)
    for i in range(6):
        ax.plot(t_s, err_steps[:, i], color=AXIS_COLORS[i], lw=1.4, label=f"ID{i+1} Err")
    ax.axhline(15, color="red", ls=":", lw=1.0, label="±15 Step Normal Envelope")
    ax.axhline(-15, color="red", ls=":", lw=1.0)
    ax.axhline(0, color="gray", ls=":")
    add_plateau_shading(ax, y_offset_ratio=0.10)

    # Annotate max error per plateau
    for p in plateaus:
        t_mid = (p["start_time_s"] + p["end_time_s"]) / 2.0
        p_sub = [r for r in active_rows if p["start_time_s"] <= (float(r["timestamp_ns"]) - timestamps[0])*1e-9 <= p["end_time_s"]]
        if p_sub:
            sub_errs = np.array([json.loads(r["command_feedback_errors"]) for r in p_sub])
            p_max_err = int(np.max(np.abs(sub_errs)))
            ax.text(t_mid, -30, f"Max {p_max_err}st", ha="center", va="center", fontsize=7.5,
                    bbox=dict(boxstyle="square,pad=0.2", facecolor="white", alpha=0.8, edgecolor="blue", lw=0.5))

    ax.set_title("Figure 4: Six-Axis Step Tracking Error Progression (with Per-Case Max Annotations)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Tracking Error (Steps)", fontsize=10)
    ax.set_ylim(-38, 65)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper right", ncol=4, framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_2_figure_4_tracking_error_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 5: Six-Axis Motor Current Progression with Global Peak Annotation
    # ==========================================================
    fig, ax = plt.subplots(figsize=(11, 5.8), dpi=150)
    for i in range(6):
        lw = 2.0 if i in [2, 3, 4] else 1.2
        ax.plot(t_s, currents[:, i], color=AXIS_COLORS[i], lw=lw, label=f"ID{i+1}")
    ax.axhline(500, color="red", ls="--", lw=2.0, label="Safety Trip Limit (500 mA)")
    ax.axhline(400, color="orange", ls=":", lw=1.5, label="Early Warning Limit (400 mA)")
    add_plateau_shading(ax, y_offset_ratio=0.10)

    # Find and annotate whole-run peak
    max_c_val = np.max(currents)
    max_c_pos = np.unravel_index(np.argmax(currents), currents.shape)
    peak_t = t_s[max_c_pos[0]]
    peak_id = f"ID{max_c_pos[1]+1}"

    ax.annotate(
        f"Global Run Peak: {peak_id} = {max_c_val} mA\n(t = {peak_t:.2f} s)",
        xy=(peak_t, max_c_val),
        xytext=(peak_t + 5.0, max_c_val + 70.0),
        arrowprops=dict(facecolor="red", edgecolor="black", shrink=0.08, width=1.5, headwidth=6),
        fontsize=9,
        fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.85, edgecolor="red"),
    )

    ax.set_title("Figure 5: Six-Axis Motor Current Progression (mA) with Global Peak Annotation", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Motor Current (mA)", fontsize=10)
    ax.set_ylim(-10, 530)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper right", ncol=4, framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_2_figure_5_motor_current_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 6: Per-Case Steady-State Current Comparison
    # ==========================================================
    if metrics_list:
        fig, ax = plt.subplots(figsize=(11, 5.5), dpi=150)
        n_cases = len(metrics_list)
        bar_width = 0.12
        x_indices = np.arange(n_cases)

        for i in range(6):
            means = [m["axis_metrics"][f"ID{i+1}"]["current_mean_mA"] for m in metrics_list]
            maxs = [m["axis_metrics"][f"ID{i+1}"]["current_max_mA"] for m in metrics_list]
            err_up = [max(0, mx - mn) for mx, mn in zip(maxs, means)]
            offset = (i - 2.5) * bar_width
            ax.bar(
                x_indices + offset,
                means,
                width=bar_width,
                color=AXIS_COLORS[i],
                label=f"ID{i+1}",
                yerr=[np.zeros(len(means)), err_up],
                capsize=3,
                alpha=0.88,
            )

        ax.axhline(500, color="red", ls="--", lw=1.8, label="500 mA Trip")
        ax.axhline(400, color="orange", ls=":", lw=1.4, label="400 mA Warning")
        ax.set_xticks(x_indices)
        labels = [f"P{m['plateau_index']}: {m['case_name'].split()[0]}" for m in metrics_list]
        ax.set_xticklabels(labels, fontsize=9)
        ax.set_title("Figure 6: Per-Case Steady-State Motor Current Comparison (Mean + Max Errorbar)", fontsize=12, fontweight="bold")
        ax.set_ylabel("Motor Current (mA)", fontsize=10)
        ax.set_ylim(0, 160)
        ax.grid(True, ls="--", alpha=0.5, axis="y")
        ax.legend(loc="upper left", ncol=4, framealpha=0.9)
        plt.tight_layout()
        fig.savefig(out_dir / "j12_2_figure_6_per_case_current_comparison.png")
        plt.close(fig)

    # ==========================================================
    # Figure 7: Per-Case Stroke Envelope & Max Tracking Error
    # ==========================================================
    if metrics_list:
        fig, (ax_strk, ax_err) = plt.subplots(1, 2, figsize=(12, 5.2), dpi=150)
        
        # Subplot 1: Min and Max Steps across cases for each ID
        for i in range(6):
            ax_id = f"ID{i+1}"
            min_steps = [m["axis_metrics"][ax_id]["fb_steps_min"] for m in metrics_list]
            max_steps = [m["axis_metrics"][ax_id]["fb_steps_max"] for m in metrics_list]
            means = [m["axis_metrics"][ax_id]["fb_steps_mean"] for m in metrics_list]
            ax_strk.plot(range(len(metrics_list)), means, marker="o", color=AXIS_COLORS[i], label=ax_id)
            ax_strk.fill_between(range(len(metrics_list)), min_steps, max_steps, color=AXIS_COLORS[i], alpha=0.15)
        
        ax_strk.axhline(1000, color="black", ls=":", label="Home (1000)")
        ax_strk.axhline(500, color="red", ls="--", lw=1.2, label="Compression Warning (500)")
        ax_strk.set_xticks(range(len(metrics_list)))
        labels = [f"P{m['plateau_index']}: {m['case_name'].split()[0]}" for m in metrics_list]
        ax_strk.set_xticklabels(labels, fontsize=8, rotation=15)
        ax_strk.set_title("Actuator Stroke Envelope per Case", fontsize=11, fontweight="bold")
        ax_strk.set_ylabel("Steps", fontsize=10)
        ax_strk.set_ylim(450, 1500)
        ax_strk.grid(True, ls="--", alpha=0.5)
        ax_strk.legend(loc="best", fontsize=8, ncol=2)

        # Subplot 2: Max tracking error per case
        for i in range(6):
            ax_id = f"ID{i+1}"
            max_errs = [m["axis_metrics"][ax_id]["tracking_error_max"] for m in metrics_list]
            ax_err.plot(range(len(metrics_list)), max_errs, marker="^", color=AXIS_COLORS[i], label=ax_id)
        
        ax_err.axhline(15, color="red", ls=":", label="15-Step Gate")
        ax_err.set_xticks(range(len(metrics_list)))
        ax_err.set_xticklabels(labels, fontsize=8, rotation=15)
        ax_err.set_title("Max Tracking Error per Case", fontsize=11, fontweight="bold")
        ax_err.set_ylabel("Max Error (Steps)", fontsize=10)
        ax_err.set_ylim(0, 35)
        ax_err.grid(True, ls="--", alpha=0.5)
        ax_err.legend(loc="best", fontsize=8, ncol=2)

        fig.suptitle("Figure 7: Per-Case Kinematic Envelope & Tracking Integrity", fontsize=13, fontweight="bold")
        plt.tight_layout()
        fig.savefig(out_dir / "j12_2_figure_7_stroke_and_error_envelope.png")
        plt.close(fig)

    print(f"[ANALYZER] Generated Figures 1 through 7 in {out_dir}.")


def generate_markdown_report(metrics_list: List[Dict[str, Any]], out_dir: Path) -> None:
    report_path = out_dir / "j12_2_combined_screening_report.md"
    lines = [
        "# J12-2 Combined Quasi-Static Attitude Screening Report",
        "",
        f"**Run Directory**: `{Path(out_dir).name}`",
        f"**Detected Plateaus**: {len(metrics_list)}",
        "",
        "## 1. Case Screening Summary Table",
        "",
        "| Plateau | Case / Nom Target | Base R/P/Y (°) | Base SO3 | Plat Err SO3 | ID1 (mA) | ID2 (mA) | ID3 (mA) | ID4 (mA) | ID5 (mA) | ID6 (mA) | Max Err (st) | Min Stroke |",
        "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    for m in metrics_list:
        p_id = m["plateau_index"]
        c_name = m["case_name"]
        b_r = m["base_attitude"]["roll_mean_deg"]
        b_p = m["base_attitude"]["pitch_mean_deg"]
        b_y = m["base_attitude"]["yaw_mean_deg"]
        b_so3 = m["base_attitude"]["so3_mean_deg"]
        p_so3_err = m["platform_attitude"]["so3_error_mean_deg"]

        currs = [m["axis_metrics"][f"ID{i+1}"]["current_mean_mA"] for i in range(6)]
        c_strs = [f"{c:.1f}" for c in currs]
        # Highlight ID3, ID4, ID5
        c_strs[2] = f"**{c_strs[2]}**"
        c_strs[3] = f"**{c_strs[3]}**"
        c_strs[4] = f"**{c_strs[4]}**"

        max_err = max(m["axis_metrics"][f"ID{i+1}"]["tracking_error_max"] for i in range(6))
        min_strk = min(m["axis_metrics"][f"ID{i+1}"]["fb_steps_min"] for i in range(6))

        lines.append(
            f"| P{p_id} | {c_name} | {b_r:+.1f} / {b_p:+.1f} / {b_y:+.1f} | {b_so3:.1f}° | {p_so3_err:.2f}° | "
            f"{c_strs[0]} | {c_strs[1]} | {c_strs[2]} | {c_strs[3]} | {c_strs[4]} | {c_strs[5]} | {max_err} | {min_strk} |"
        )

    lines.extend([
        "",
        "## 2. Multi-Axis Kinematic Coupling & Load Qualification",
        "",
    ])

    # Assess safety gates across all detected plateaus
    any_over_400 = False
    any_over_500 = False
    any_large_error = False
    min_stroke_global = 9999

    for m in metrics_list:
        for i in range(6):
            ax_id = f"ID{i+1}"
            c_max = m["axis_metrics"][ax_id]["current_max_mA"]
            err_max = m["axis_metrics"][ax_id]["tracking_error_max"]
            s_min = m["axis_metrics"][ax_id]["fb_steps_min"]

            if c_max >= 500:
                any_over_500 = True
            elif c_max >= 400:
                any_over_400 = True
            if err_max > 25:
                any_large_error = True
            if s_min < min_stroke_global:
                min_stroke_global = s_min

    lines.append(f"- **Global Minimum Actuator Stroke in Plateaus**: {min_stroke_global} steps (margin to 500 threshold: +{min_stroke_global - 500} steps)")
    lines.append(f"- **Current Safety Limit (500 mA)**: {'VIOLATED' if any_over_500 else 'PASS (No axis exceeded 500 mA)'}")
    lines.append(f"- **Current Warning Limit (400 mA)**: {'EXCEEDED' if any_over_400 else 'PASS (All axes below 400 mA)'}")
    lines.append(f"- **Tracking Error Boundary (25 steps)**: {'DIVERGED / SATURATED' if any_large_error else 'PASS (All axes tracking tightly)'}")

    if not any_over_400 and not any_large_error and min_stroke_global > 600:
        lines.append("\n**Screening Verdict**: **PASS** - 3-DOF combined attitude envelope is kinematically clean, loads are evenly distributed, and no non-linear current surge or tracking saturation occurred.")
    else:
        lines.append("\n**Screening Verdict**: **ATTENTION REQUIRED** - Kinematic or load anomalies observed during screening; review individual axes above.")

    lines.extend([
        "",
        "## 3. Generated Figures",
        "",
        "- [Figure 1: 3-DOF Base Attitude Progression](j12_2_figure_1_base_attitude_progression.png)",
        "- [Figure 2: 3-DOF Platform Tracking Residual](j12_2_figure_2_platform_error_progression.png)",
        "- [Figure 3: Six-Axis Actuator Feedback Steps](j12_2_figure_3_actuator_steps_progression.png)",
        "- [Figure 4: Six-Axis Tracking Error Progression](j12_2_figure_4_tracking_error_progression.png)",
        "- [Figure 5: Six-Axis Motor Current Progression](j12_2_figure_5_motor_current_progression.png)",
        "- [Figure 6: Per-Case Steady-State Current Comparison](j12_2_figure_6_per_case_current_comparison.png)",
        "- [Figure 7: Per-Case Actuator Stroke & Error Envelope](j12_2_figure_7_stroke_and_error_envelope.png)",
        "",
    ])

    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[ANALYZER] Wrote markdown report to {report_path}")


def main():
    parser = argparse.ArgumentParser(description="J12-2 Combined Quasi-Static Attitude Screening Analyzer")
    parser.add_argument("--input-csv", required=True, help="Path to telemetry CSV")
    parser.add_argument("--output-dir", default=None, help="Directory to save figures and reports")
    parser.add_argument("--segmentation-mode", default="preset", choices=["preset", "auto"],
                        help="Segmentation mode: 'preset' (default, uses verified test windows) or 'auto' (detects via rate)")
    parser.add_argument("--min-duration", type=float, default=1.8, help="Minimum plateau duration in seconds (default: 1.8)")
    parser.add_argument("--rate-threshold", type=float, default=3.0, help="Max 3-DOF angular rate in deg/s for plateau (default: 3.0)")
    args = parser.parse_args()

    csv_path = Path(args.input_csv)
    if not csv_path.exists():
        print(f"[ERROR] Input CSV not found: {csv_path}")
        sys.exit(1)

    out_dir = Path(args.output_dir) if args.output_dir else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = parse_telemetry_csv(str(csv_path))
    active_rows = [r for r in rows if r.get("state") == "ACTIVE"]
    if not active_rows:
        active_rows = rows
    timestamps = np.array([float(r["timestamp_ns"]) for r in active_rows])
    t_s = (timestamps - timestamps[0]) * 1e-9

    print(f"[ANALYZER] Loaded {len(rows)} telemetry rows from {csv_path.name}")

    if args.segmentation_mode == "preset":
        plateaus = detect_plateaus_preset(active_rows, t_s)
    else:
        plateaus = detect_plateaus_auto(
            active_rows,
            t_s,
            min_duration_s=args.min_duration,
            rate_threshold_deg_s=args.rate_threshold,
        )

    print(f"[ANALYZER] Segmented {len(plateaus)} quasi-static plateaus:")
    for p in plateaus:
        print(f"  - P{p['plateau_index']}: {p['case_name']} [{p['start_time_s']:.1f}s - {p['end_time_s']:.1f}s, dur={p['duration_s']:.1f}s] (R={p['mean_roll_deg']:+.1f}, P={p['mean_pitch_deg']:+.1f}, Y={p['mean_yaw_deg']:+.1f})")

    metrics_list = [compute_plateau_metrics(p) for p in plateaus]

    # Save metrics JSON
    json_path = out_dir / "j12_2_combined_screening_metrics.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metrics_list, f, indent=2)
    print(f"[ANALYZER] Saved screening metrics JSON to {json_path}")

    # Generate Figures and Report
    generate_all_figures(rows, plateaus, metrics_list, out_dir)
    generate_markdown_report(metrics_list, out_dir)


if __name__ == "__main__":
    main()
