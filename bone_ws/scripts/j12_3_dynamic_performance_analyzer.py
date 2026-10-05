#!/usr/bin/env python3
"""Stage J12-3: Small-Angle Dynamic Performance Offline Analyzer.

Processes telemetry from Feedforward + P/PD dual-IMU dynamic screening runs:
Segments:
  - Roll: Slow (~0.25 Hz), Medium (~0.5 Hz), Fast (~1.0 Hz)
  - Pitch: Slow (~0.25 Hz), Medium (~0.5 Hz), Fast (~1.0 Hz)
  - Yaw: Slow (~0.25 Hz), Medium (~0.5 Hz), Fast (~1.0 Hz)
  - Combined Dynamic (~0.5 Hz, 3-axis coupled ±5°)
  - Inter-segment zero-holds

Generates 9 standard figures:
  Figure 1: Base R/P/Y Progression with Dynamic Segments Annotated
  Figure 2: Base SO(3) Angular Rate Magnitude Progression (|omega|_B in deg/s)
  Figure 3: Platform Attitude Residuals (R/P/Y) and SO(3) Error Progression
  Figure 4: Base Angular Rate vs Platform SO(3) Error (|omega|_B -> e_SO3)
  Figure 5: Task-Space PID Residuals (P, D, Total) vs Time (with ±0.5° Saturation Lines)
  Figure 6: Six-Axis Actuator Command & Feedback Step Rates (steps/s)
  Figure 7: Six-Axis Tracking Error vs Base Angular Rate
  Figure 8: Six-Axis Motor Current vs Base Angular Rate
  Figure 9: Multi-Axis Disturbance Rejection Comparison (Attenuation dB & Amplitude Ratio)
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


AXIS_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
AXIS_NAMES = [f"ID{i+1}" for i in range(6)]


def parse_telemetry_csv(csv_path: str) -> List[Dict[str, Any]]:
    rows = []
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def compute_so3_angular_rate(R_mats: np.ndarray, dt_s: np.ndarray) -> np.ndarray:
    """Compute SO(3) angular rate vector and magnitude in deg/s.
    
    omega_k = Log(R_{k+1} R_k^T)^vee / dt
    """
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
        
        # Rate magnitude in rad/s -> deg/s
        omega_mag[i] = math.degrees(theta / dt)
        
    omega_mag[-1] = omega_mag[-2] if n > 1 else 0.0
    return omega_mag


def segment_dynamic_runs(
    t_s: np.ndarray,
    r: np.ndarray,
    p: np.ndarray,
    y: np.ndarray,
    omega_mag: np.ndarray,
    custom_intervals: Optional[List[Tuple[str, float, float]]] = None,
) -> List[Dict[str, Any]]:
    """Segment dynamic testing into Roll S/M/F, Pitch S/M/F, Yaw S/M/F, Combined."""
    if custom_intervals:
        segments = []
        for idx, (name, t0, t1) in enumerate(custom_intervals):
            mask = (t_s >= t0) & (t_s <= t1)
            indices = np.where(mask)[0]
            if len(indices) == 0:
                continue
            segments.append({
                "segment_index": idx + 1,
                "name": name,
                "start_time_s": float(t0),
                "end_time_s": float(t1),
                "start_idx": int(indices[0]),
                "end_idx": int(indices[-1]),
                "duration_s": float(t1 - t0),
            })
        return segments

    # Automatic segmentation based on angular rate threshold
    # Active motion when smoothed rate > 3.0 deg/s
    kernel = 15
    rate_smooth = np.convolve(omega_mag, np.ones(kernel)/kernel, mode="same")
    is_active = rate_smooth > 3.5

    # Merge gaps < 1.0s
    for i in range(len(is_active)):
        if not is_active[i]:
            fwd = np.where(is_active[i:])[0]
            if len(fwd) > 0 and fwd[0] > 0:
                gap = t_s[i + fwd[0]] - t_s[i]
                if gap < 1.0:
                    is_active[i : i + fwd[0]] = True

    # Identify blocks >= 3.0s
    blocks = []
    in_block = False
    s_idx = 0
    for i, act in enumerate(is_active):
        if act and not in_block:
            in_block = True
            s_idx = i
        elif not act and in_block:
            in_block = False
            dur = t_s[i - 1] - t_s[s_idx]
            if dur >= 3.0:
                blocks.append((s_idx, i - 1, dur))
    if in_block:
        dur = t_s[-1] - t_s[s_idx]
        if dur >= 3.0:
            blocks.append((s_idx, len(is_active) - 1, dur))

    # Classify blocks by dominant axis
    classified_segments = []
    for b_idx, (si, ei, dur) in enumerate(blocks):
        sub_r = r[si : ei + 1]
        sub_p = p[si : ei + 1]
        sub_y = y[si : ei + 1]
        
        amp_r = (np.max(sub_r) - np.min(sub_r)) / 2.0
        amp_p = (np.max(sub_p) - np.min(sub_p)) / 2.0
        amp_y = (np.max(sub_y) - np.min(sub_y)) / 2.0
        
        if amp_r > 1.8 * max(amp_p, amp_y):
            axis = "Roll"
        elif amp_p > 1.8 * max(amp_r, amp_y):
            axis = "Pitch"
        elif amp_y > 1.8 * max(amp_r, amp_p):
            axis = "Yaw"
        else:
            axis = "Combined"

        mean_rate = np.mean(omega_mag[si : ei + 1])
        classified_segments.append({
            "axis": axis,
            "mean_rate": mean_rate,
            "start_time_s": float(t_s[si]),
            "end_time_s": float(t_s[ei]),
            "start_idx": si,
            "end_idx": ei,
            "duration_s": float(dur),
        })

    # Group by axis and assign Slow / Medium / Fast
    results = []
    seg_counter = 1
    for ax in ["Roll", "Pitch", "Yaw"]:
        ax_blocks = [b for b in classified_segments if b["axis"] == ax]
        ax_blocks.sort(key=lambda x: x["start_time_s"])
        levels = ["Slow", "Medium", "Fast"]
        for idx, b in enumerate(ax_blocks):
            lvl = levels[min(idx, len(levels) - 1)] if len(ax_blocks) > 1 else "Dynamic"
            results.append({
                "segment_index": seg_counter,
                "name": f"{ax}_{lvl}",
                "axis": ax,
                "level": lvl,
                "start_time_s": b["start_time_s"],
                "end_time_s": b["end_time_s"],
                "start_idx": b["start_idx"],
                "end_idx": b["end_idx"],
                "duration_s": b["duration_s"],
            })
            seg_counter += 1

    combined_blocks = [b for b in classified_segments if b["axis"] == "Combined"]
    for idx, b in enumerate(combined_blocks):
        results.append({
            "segment_index": seg_counter,
            "name": f"Combined_{idx+1}",
            "axis": "Combined",
            "level": "Combined",
            "start_time_s": b["start_time_s"],
            "end_time_s": b["end_time_s"],
            "start_idx": b["start_idx"],
            "end_idx": b["end_idx"],
            "duration_s": b["duration_s"],
        })
        seg_counter += 1

    results.sort(key=lambda x: x["start_time_s"])
    for i, r_item in enumerate(results):
        r_item["segment_index"] = i + 1
    return results


def analyze_segment_metrics(
    seg: Dict[str, Any],
    active_rows: List[Dict[str, Any]],
    t_s: np.ndarray,
    omega_mag: np.ndarray,
) -> Dict[str, Any]:
    """Compute detailed dynamic metrics for one segment."""
    si, ei = seg["start_idx"], seg["end_idx"]
    sub_rows = active_rows[si : ei + 1]
    n = len(sub_rows)

    sub_t = t_s[si : ei + 1]
    sub_omega = omega_mag[si : ei + 1]

    sub_base_r = np.array([float(r["base_rel_roll_deg"]) for r in sub_rows])
    sub_base_p = np.array([float(r["base_rel_pitch_deg"]) for r in sub_rows])
    sub_base_y = np.array([float(r["base_rel_yaw_deg"]) for r in sub_rows])
    sub_base_so3 = np.array([float(r.get("base_so3_magnitude_deg", 0.0)) for r in sub_rows])

    sub_plat_r = np.array([float(r.get("platform_rel_roll_deg", 0.0)) for r in sub_rows])
    sub_plat_p = np.array([float(r.get("platform_rel_pitch_deg", 0.0)) for r in sub_rows])
    sub_plat_y = np.array([float(r.get("platform_rel_yaw_deg", 0.0)) for r in sub_rows])
    sub_plat_so3 = np.array([float(r.get("platform_so3_error_deg", 0.0)) for r in sub_rows])

    # Amplitudes
    amp_base_r = (np.max(sub_base_r) - np.min(sub_base_r)) / 2.0
    amp_base_p = (np.max(sub_base_p) - np.min(sub_base_p)) / 2.0
    amp_base_y = (np.max(sub_base_y) - np.min(sub_base_y)) / 2.0

    amp_plat_r = (np.max(sub_plat_r) - np.min(sub_plat_r)) / 2.0
    amp_plat_p = (np.max(sub_plat_p) - np.min(sub_plat_p)) / 2.0
    amp_plat_y = (np.max(sub_plat_y) - np.min(sub_plat_y)) / 2.0

    axis = seg.get("axis", "Roll")
    if "Roll" in seg["name"]:
        a_base = amp_base_r
        a_plat = amp_plat_r
    elif "Pitch" in seg["name"]:
        a_base = amp_base_p
        a_plat = amp_plat_p
    elif "Yaw" in seg["name"]:
        a_base = amp_base_y
        a_plat = amp_plat_y
    else:
        a_base = max(amp_base_r, amp_base_p, amp_base_y)
        a_plat = max(amp_plat_r, amp_plat_p, amp_plat_y)

    ratio = (a_plat / a_base) if a_base > 0.1 else 1.0
    attenuation_db = 20.0 * math.log10(max(ratio, 1e-4))

    # Dominant frequency estimation via zero-crossings
    sig = sub_base_r if "Roll" in seg["name"] else (sub_base_p if "Pitch" in seg["name"] else sub_base_y)
    sig_detrend = sig - np.mean(sig)
    zc = np.where(np.diff(np.sign(sig_detrend)))[0]
    dur = sub_t[-1] - sub_t[0]
    freq_est = (len(zc) / (2.0 * max(dur, 0.1))) if len(zc) >= 2 else 0.0

    # PID Residuals
    p_deg = np.array([json.loads(r["pid_p_deg"]) for r in sub_rows])
    d_deg = np.array([json.loads(r["pid_d_deg"]) for r in sub_rows])
    tot_deg = np.array([json.loads(r["pid_total_residual_deg"]) for r in sub_rows])
    sat_axes = np.array([json.loads(r["pid_saturated_axes"]) for r in sub_rows])

    sat_counts = [int(np.sum(sat_axes[:, i])) for i in range(3)]
    sat_ratios = [float(np.mean(sat_axes[:, i])) * 100.0 for i in range(3)]

    # Actuators
    cmd_steps = np.array([json.loads(r["commanded_steps"]) for r in sub_rows])
    fb_steps = np.array([json.loads(r["feedback_steps"]) for r in sub_rows])
    err_steps = np.array([json.loads(r["command_feedback_errors"]) for r in sub_rows])
    currents = np.array([json.loads(r["joint_currents_raw_mA"]) for r in sub_rows])
    dts = np.array([float(r.get("dt_ms", 20.0)) for r in sub_rows])

    # Step rates (steps / s)
    dt_arr = np.clip(dts * 1e-3, 0.010, 0.100)
    cmd_rates = np.abs(np.diff(cmd_steps, axis=0)) / dt_arr[:-1, None]
    fb_rates = np.abs(np.diff(fb_steps, axis=0)) / dt_arr[:-1, None]

    axis_metrics = {}
    for i in range(6):
        c = currents[:, i]
        e = err_steps[:, i]
        axis_metrics[f"ID{i+1}"] = {
            "peak_cmd_rate_steps_s": float(np.max(cmd_rates[:, i])) if len(cmd_rates) > 0 else 0.0,
            "peak_fb_rate_steps_s": float(np.max(fb_rates[:, i])) if len(fb_rates) > 0 else 0.0,
            "tracking_error_rms": float(np.sqrt(np.mean(e**2))),
            "tracking_error_peak": int(np.max(np.abs(e))),
            "current_max_mA": int(np.max(c)),
            "current_p95_mA": float(np.percentile(c, 95)),
        }

    return {
        "segment_index": seg["segment_index"],
        "name": seg["name"],
        "start_time_s": seg["start_time_s"],
        "end_time_s": seg["end_time_s"],
        "duration_s": seg["duration_s"],
        "sample_count": n,
        "input_disturbance": {
            "amplitude_base_deg": float(a_base),
            "dominant_frequency_hz": float(freq_est),
            "peak_angular_rate_deg_s": float(np.max(sub_omega)),
            "rms_angular_rate_deg_s": float(np.sqrt(np.mean(sub_omega**2))),
        },
        "platform_stabilization": {
            "amplitude_plat_deg": float(a_plat),
            "so3_error_rms_deg": float(np.sqrt(np.mean(sub_plat_so3**2))),
            "so3_error_peak_deg": float(np.max(sub_plat_so3)),
            "so3_error_p95_deg": float(np.percentile(sub_plat_so3, 95)),
            "attenuation_ratio": float(ratio),
            "attenuation_db": float(attenuation_db),
        },
        "controller": {
            "p_residual_rms_deg": [float(np.sqrt(np.mean(p_deg[:, i]**2))) for i in range(3)],
            "p_residual_peak_deg": [float(np.max(np.abs(p_deg[:, i]))) for i in range(3)],
            "d_residual_rms_deg": [float(np.sqrt(np.mean(d_deg[:, i]**2))) for i in range(3)],
            "d_residual_peak_deg": [float(np.max(np.abs(d_deg[:, i]))) for i in range(3)],
            "total_residual_rms_deg": [float(np.sqrt(np.mean(tot_deg[:, i]**2))) for i in range(3)],
            "total_residual_peak_deg": [float(np.max(np.abs(tot_deg[:, i]))) for i in range(3)],
            "saturation_counts": sat_counts,
            "saturation_ratios_pct": sat_ratios,
        },
        "actuators": axis_metrics,
    }


def generate_figures(
    active_rows: List[Dict[str, Any]],
    segments: List[Dict[str, Any]],
    metrics_list: List[Dict[str, Any]],
    out_dir: Path,
    t_s: np.ndarray,
    omega_mag: np.ndarray,
) -> None:
    """Generate Figures 1 through 9."""
    plt.rcParams["font.sans-serif"] = "DejaVu Sans"
    plt.rcParams["axes.edgecolor"] = "#333333"

    base_r = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])
    base_p = np.array([float(r["base_rel_pitch_deg"]) for r in active_rows])
    base_y = np.array([float(r["base_rel_yaw_deg"]) for r in active_rows])

    plat_r = np.array([float(r.get("platform_rel_roll_deg", 0.0)) for r in active_rows])
    plat_p = np.array([float(r.get("platform_rel_pitch_deg", 0.0)) for r in active_rows])
    plat_y = np.array([float(r.get("platform_rel_yaw_deg", 0.0)) for r in active_rows])
    plat_so3 = np.array([float(r.get("platform_so3_error_deg", 0.0)) for r in active_rows])

    p_deg = np.array([json.loads(r["pid_p_deg"]) for r in active_rows])
    d_deg = np.array([json.loads(r["pid_d_deg"]) for r in active_rows])
    tot_deg = np.array([json.loads(r["pid_total_residual_deg"]) for r in active_rows])

    cmd_steps = np.array([json.loads(r["commanded_steps"]) for r in active_rows])
    fb_steps = np.array([json.loads(r["feedback_steps"]) for r in active_rows])
    err_steps = np.array([json.loads(r["command_feedback_errors"]) for r in active_rows])
    currents = np.array([json.loads(r["joint_currents_raw_mA"]) for r in active_rows])
    dts = np.array([float(r.get("dt_ms", 20.0)) for r in active_rows])

    dt_arr = np.clip(dts * 1e-3, 0.010, 0.100)
    cmd_rates = np.abs(np.diff(cmd_steps, axis=0)) / dt_arr[:-1, None]
    fb_rates = np.abs(np.diff(fb_steps, axis=0)) / dt_arr[:-1, None]

    def add_segment_shading(ax, y_offset_ratio=0.10):
        for s in segments:
            t0, t1 = s["start_time_s"], s["end_time_s"]
            name = s["name"]
            color = "#e6f2ff" if "Roll" in name else ("#fff2e6" if "Pitch" in name else ("#e6ffe6" if "Yaw" in name else "#f9e6ff"))
            ax.axvspan(t0, t1, color=color, alpha=0.55)
            mid = (t0 + t1) / 2.0
            y_lim = ax.get_ylim()
            y_pos = y_lim[1] - y_offset_ratio * (y_lim[1] - y_lim[0])
            lbl = name.replace("_", "\n")
            ax.text(mid, y_pos, lbl, ha="center", va="top", fontsize=7, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.75, edgecolor="gray", lw=0.5))

    # ==========================================================
    # Figure 1: Base R/P/Y vs Time with Dynamic Segments
    # ==========================================================
    fig, axs = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True, dpi=150)
    axs[0].plot(t_s, base_r, color="#1f77b4", lw=1.5, label="Base Roll")
    axs[0].set_ylabel("Roll (°)", fontsize=10)
    axs[0].grid(True, ls="--", alpha=0.5)
    axs[0].legend(loc="upper left")

    axs[1].plot(t_s, base_p, color="#ff7f0e", lw=1.5, label="Base Pitch")
    axs[1].set_ylabel("Pitch (°)", fontsize=10)
    axs[1].grid(True, ls="--", alpha=0.5)
    axs[1].legend(loc="upper left")

    axs[2].plot(t_s, base_y, color="#2ca02c", lw=1.5, label="Base Yaw")
    axs[2].set_ylabel("Yaw (°)", fontsize=10)
    axs[2].set_xlabel("Time (s)", fontsize=10)
    axs[2].grid(True, ls="--", alpha=0.5)
    axs[2].legend(loc="upper left")

    for ax in axs:
        add_segment_shading(ax, y_offset_ratio=0.15)
    fig.suptitle("Figure 1: Base Attitude Progression (Small-Angle Dynamic Screening)", fontsize=12, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_1_base_attitude_progression.png")
    plt.close(fig)

    # ==========================================================
    # Figure 2: Base SO(3) Angular Rate Magnitude (|omega|_B)
    # ==========================================================
    fig, ax = plt.subplots(figsize=(12, 5.0), dpi=150)
    ax.plot(t_s, omega_mag, color="#d62728", lw=1.5, label=r"$|\omega|_B$ SO(3) Rate")
    ax.axhline(7.9, color="blue", ls=":", lw=1.0, label="Slow Nom Peak (7.9°/s)")
    ax.axhline(15.7, color="orange", ls=":", lw=1.0, label="Medium Nom Peak (15.7°/s)")
    ax.axhline(31.4, color="red", ls=":", lw=1.0, label="Fast Nom Peak (31.4°/s)")
    add_segment_shading(ax, y_offset_ratio=0.12)
    ax.set_title(r"Figure 2: Base SO(3) Angular Rate Magnitude $|\omega|_B$ Progression", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)", fontsize=10)
    ax.set_ylabel("Angular Rate (°/s)", fontsize=10)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper right", framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_2_base_angular_rate.png")
    plt.close(fig)

    # ==========================================================
    # Figure 3: Platform Attitude Residuals & SO(3) Error
    # ==========================================================
    fig, axs = plt.subplots(4, 1, figsize=(12, 8.5), sharex=True, dpi=150)
    axs[0].plot(t_s, plat_r, color="#1f77b4", lw=1.4, label="Platform Roll Error")
    axs[0].set_ylabel("Roll Err (°)", fontsize=9)
    axs[0].grid(True, ls="--", alpha=0.5)
    axs[0].legend(loc="upper left")

    axs[1].plot(t_s, plat_p, color="#ff7f0e", lw=1.4, label="Platform Pitch Error")
    axs[1].set_ylabel("Pitch Err (°)", fontsize=9)
    axs[1].grid(True, ls="--", alpha=0.5)
    axs[1].legend(loc="upper left")

    axs[2].plot(t_s, plat_y, color="#2ca02c", lw=1.4, label="Platform Yaw Error")
    axs[2].set_ylabel("Yaw Err (°)", fontsize=9)
    axs[2].grid(True, ls="--", alpha=0.5)
    axs[2].legend(loc="upper left")

    axs[3].plot(t_s, plat_so3, color="#d62728", lw=1.4, label="Platform SO(3) Error")
    axs[3].set_ylabel("SO(3) Err (°)", fontsize=9)
    axs[3].set_xlabel("Time (s)", fontsize=10)
    axs[3].grid(True, ls="--", alpha=0.5)
    axs[3].legend(loc="upper left")

    for ax in axs:
        add_segment_shading(ax, y_offset_ratio=0.15)
    fig.suptitle("Figure 3: Platform Attitude Residuals & SO(3) Error Progression", fontsize=12, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_3_platform_attitude_residuals.png")
    plt.close(fig)

    # ==========================================================
    # Figure 4: Base Angular Rate vs Platform SO(3) Error
    # ==========================================================
    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)
    ax.scatter(omega_mag, plat_so3, alpha=0.25, s=12, color="#1f77b4", label="Cycle Samples (50Hz)")

    # Compute binned mean/RMS curve
    bins = np.linspace(0, max(np.max(omega_mag), 10.0), 16)
    bin_centers = (bins[:-1] + bins[1:]) / 2.0
    binned_rms = []
    for bi in range(len(bins) - 1):
        idx_b = (omega_mag >= bins[bi]) & (omega_mag < bins[bi + 1])
        if np.sum(idx_b) > 5:
            binned_rms.append(np.sqrt(np.mean(plat_so3[idx_b]**2)))
        else:
            binned_rms.append(np.nan)
    ax.plot(bin_centers, binned_rms, "r-o", lw=2.2, label=r"Binned $e_{SO3}$ RMS Trend")

    ax.set_title(r"Figure 4: Base Angular Rate $|\omega|_B$ vs. Platform SO(3) Error", fontsize=12, fontweight="bold")
    ax.set_xlabel(r"Base SO(3) Angular Rate $|\omega|_B$ (°/s)", fontsize=10)
    ax.set_ylabel("Platform SO(3) Error (°)", fontsize=10)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper left", framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_4_angular_rate_vs_so3_error.png")
    plt.close(fig)

    # ==========================================================
    # Figure 5: P / D / Total Residual vs Time (with ±0.5° Saturation)
    # ==========================================================
    fig, axs = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True, dpi=150)
    ch_names = ["Roll P/PD", "Pitch PD", "Yaw PD"]
    for ch_idx in range(3):
        axs[ch_idx].plot(t_s, p_deg[:, ch_idx], label=f"{ch_names[ch_idx]} P Term", lw=1.2, color="#1f77b4")
        axs[ch_idx].plot(t_s, d_deg[:, ch_idx], label=f"{ch_names[ch_idx]} D Term", lw=1.2, color="#2ca02c")
        axs[ch_idx].plot(t_s, tot_deg[:, ch_idx], label=f"{ch_names[ch_idx]} Total", lw=1.5, color="#d62728")
        axs[ch_idx].axhline(0.5, color="red", ls="--", lw=1.0, label="±0.5° Saturation Limit")
        axs[ch_idx].axhline(-0.5, color="red", ls="--", lw=1.0)
        axs[ch_idx].set_ylabel(f"{ch_names[ch_idx]} (°)", fontsize=9)
        axs[ch_idx].set_ylim(-0.7, 0.7)
        axs[ch_idx].grid(True, ls="--", alpha=0.5)
        axs[ch_idx].legend(loc="upper right", ncol=4, fontsize=8)
        add_segment_shading(axs[ch_idx], y_offset_ratio=0.15)
    axs[2].set_xlabel("Time (s)", fontsize=10)
    fig.suptitle("Figure 5: Task-Space PID Residuals & Saturation Boundary (±0.5°)", fontsize=12, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_5_pid_residuals_and_saturation.png")
    plt.close(fig)

    # ==========================================================
    # Figure 6: Six-Axis Actuator Command & Feedback Step Rates
    # ==========================================================
    fig, (ax_cmd, ax_fb) = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True, dpi=150)
    t_rate = t_s[:-1]
    for i in range(6):
        ax_cmd.plot(t_rate, cmd_rates[:, i], color=AXIS_COLORS[i], lw=1.2, label=f"ID{i+1}")
        ax_fb.plot(t_rate, fb_rates[:, i], color=AXIS_COLORS[i], lw=1.2, label=f"ID{i+1}")
    ax_cmd.set_title("Command Step Rate (steps/s)", fontsize=10, fontweight="bold")
    ax_cmd.set_ylabel("Cmd Rate", fontsize=9)
    ax_cmd.grid(True, ls="--", alpha=0.5)
    ax_cmd.legend(loc="upper right", ncol=6, fontsize=8)
    add_segment_shading(ax_cmd, y_offset_ratio=0.15)

    ax_fb.set_title("Feedback Step Rate (steps/s)", fontsize=10, fontweight="bold")
    ax_fb.set_ylabel("Fb Rate", fontsize=9)
    ax_fb.set_xlabel("Time (s)", fontsize=10)
    ax_fb.grid(True, ls="--", alpha=0.5)
    ax_fb.legend(loc="upper right", ncol=6, fontsize=8)
    add_segment_shading(ax_fb, y_offset_ratio=0.15)

    fig.suptitle("Figure 6: Six-Axis Actuator Command vs Feedback Step Rates", fontsize=12, fontweight="bold")
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_6_actuator_step_rates.png")
    plt.close(fig)

    # ==========================================================
    # Figure 7: Six-Axis Tracking Error vs Base Angular Rate
    # ==========================================================
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=150)
    for i in range(6):
        ax.scatter(omega_mag, np.abs(err_steps[:, i]), alpha=0.3, s=10, color=AXIS_COLORS[i], label=f"ID{i+1}")
    ax.axhline(15, color="red", ls=":", lw=1.2, label="±15 Step Gate")
    ax.set_title(r"Figure 7: Six-Axis Absolute Tracking Error vs Base Angular Rate $|\omega|_B$", fontsize=12, fontweight="bold")
    ax.set_xlabel(r"Base SO(3) Angular Rate $|\omega|_B$ (°/s)", fontsize=10)
    ax.set_ylabel("Absolute Tracking Error (steps)", fontsize=10)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper left", ncol=4, framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_7_tracking_error_vs_rate.png")
    plt.close(fig)

    # ==========================================================
    # Figure 8: Motor Current vs Base Angular Rate
    # ==========================================================
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=150)
    for i in range(6):
        lw = 2.0 if i in [2, 3, 4] else 1.2
        ax.scatter(omega_mag, currents[:, i], alpha=0.3, s=10, color=AXIS_COLORS[i], label=f"ID{i+1}")
    ax.axhline(400, color="orange", ls=":", lw=1.5, label="400 mA Warning")
    ax.axhline(500, color="red", ls="--", lw=1.8, label="500 mA Safety Limit")
    ax.set_title(r"Figure 8: Six-Axis Motor Current vs Base Angular Rate $|\omega|_B$", fontsize=12, fontweight="bold")
    ax.set_xlabel(r"Base SO(3) Angular Rate $|\omega|_B$ (°/s)", fontsize=10)
    ax.set_ylabel("Motor Current (mA)", fontsize=10)
    ax.set_ylim(-10, 530)
    ax.grid(True, ls="--", alpha=0.5)
    ax.legend(loc="upper left", ncol=4, framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_3_figure_8_motor_current_vs_rate.png")
    plt.close(fig)

    # ==========================================================
    # Figure 9: Disturbance Rejection Comparison (Bar Chart)
    # ==========================================================
    if metrics_list:
        fig, (ax_ratio, ax_db) = plt.subplots(1, 2, figsize=(12, 5.0), dpi=150)
        seg_names = [m["name"] for m in metrics_list]
        ratios = [m["platform_stabilization"]["attenuation_ratio"] for m in metrics_list]
        dbs = [m["platform_stabilization"]["attenuation_db"] for m in metrics_list]
        x_idx = np.arange(len(seg_names))

        colors = ["#1f77b4" if "Roll" in n else ("#ff7f0e" if "Pitch" in n else ("#2ca02c" if "Yaw" in n else "#9467bd")) for n in seg_names]

        ax_ratio.bar(x_idx, ratios, color=colors, alpha=0.85)
        ax_ratio.axhline(1.0, color="red", ls="--", lw=1.2, label="Unity (No Attenuation)")
        ax_ratio.set_xticks(x_idx)
        ax_ratio.set_xticklabels(seg_names, rotation=35, ha="right", fontsize=8)
        ax_ratio.set_ylabel(r"$A_{\rm platform} / A_{\rm base}$", fontsize=10)
        ax_ratio.set_title("Amplitude Transmission Ratio", fontsize=11, fontweight="bold")
        ax_ratio.grid(True, ls="--", alpha=0.5, axis="y")
        ax_ratio.legend(loc="upper left")

        ax_db.bar(x_idx, dbs, color=colors, alpha=0.85)
        ax_db.axhline(0.0, color="red", ls="--", lw=1.2, label="0 dB (Unity)")
        ax_db.set_xticks(x_idx)
        ax_db.set_xticklabels(seg_names, rotation=35, ha="right", fontsize=8)
        ax_db.set_ylabel("Attenuation (dB)", fontsize=10)
        ax_db.set_title("Disturbance Rejection (dB)", fontsize=11, fontweight="bold")
        ax_db.grid(True, ls="--", alpha=0.5, axis="y")
        ax_db.legend(loc="best")

        fig.suptitle("Figure 9: Small-Angle Dynamic Disturbance Rejection Across Segments", fontsize=12, fontweight="bold")
        plt.tight_layout()
        fig.savefig(out_dir / "j12_3_figure_9_disturbance_rejection_comparison.png")
        plt.close(fig)

    print(f"[ANALYZER] Successfully generated Figures 1 through 9 in {out_dir}.")


def generate_markdown_report(metrics_list: List[Dict[str, Any]], out_dir: Path) -> None:
    report_path = out_dir / "j12_3_dynamic_performance_report.md"
    lines = [
        "# J12-3 Small-Angle Dynamic Performance Report",
        "",
        f"**Run Directory**: `{Path(out_dir).name}`",
        f"**Dynamic Segments Evaluated**: {len(metrics_list)}",
        "",
        "## 1. Dynamic Performance Summary Table",
        "",
        "| Segment | Freq (Hz) | Peak Rate (°/s) | Base Amp (°) | Plat SO3 RMS (°) | Plat SO3 Peak (°) | Attenuation (dB) | P/D Sat Ratio (%) | Max Err (st) | Max Curr (mA) |",
        "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    for m in metrics_list:
        name = m["name"]
        inp = m["input_disturbance"]
        plat = m["platform_stabilization"]
        ctrl = m["controller"]
        act = m["actuators"]

        freq = inp["dominant_frequency_hz"]
        pk_rate = inp["peak_angular_rate_deg_s"]
        b_amp = inp["amplitude_base_deg"]
        so3_rms = plat["so3_error_rms_deg"]
        so3_pk = plat["so3_error_peak_deg"]
        att_db = plat["attenuation_db"]
        sat_max = max(ctrl["saturation_ratios_pct"])
        max_err = max(act[ax]["tracking_error_peak"] for ax in AXIS_NAMES)
        max_curr = max(act[ax]["current_max_mA"] for ax in AXIS_NAMES)

        lines.append(
            f"| **{name}** | {freq:.2f} | {pk_rate:.1f} | {b_amp:.2f}° | {so3_rms:.2f}° | {so3_pk:.2f}° | "
            f"**{att_db:+.1f} dB** | {sat_max:.1f}% | {max_err} st | {max_curr} mA |"
        )

    lines.extend([
        "",
        "## 2. Dynamic Performance & Classical Bottleneck Audit",
        "",
    ])

    # Check saturation, tracking, and current across segments
    any_high_sat = any(max(m["controller"]["saturation_ratios_pct"]) > 20.0 for m in metrics_list)
    any_curr_warn = any(max(m["actuators"][ax]["current_max_mA"] for ax in AXIS_NAMES) > 400 for m in metrics_list)
    max_rate_all = max(m["input_disturbance"]["peak_angular_rate_deg_s"] for m in metrics_list) if metrics_list else 0.0

    lines.append(f"- **Max Base Angular Rate Tested**: {max_rate_all:.1f}°/s")
    lines.append(f"- **Residual Saturation (±0.5°)**: {'FREQUENT SATURATION (>20% cycles)' if any_high_sat else 'NOMINAL / LINEAR (Rarely saturated)'}")
    lines.append(f"- **Motor Current Boundary (400 mA / 500 mA)**: {'OVERCURRENT WARNING' if any_curr_warn else 'SAFE (All axes well below 400 mA)'}")

    lines.extend([
        "",
        "## 3. Generated Figures",
        "",
        "- [Figure 1: Base Attitude Progression](j12_3_figure_1_base_attitude_progression.png)",
        "- [Figure 2: Base SO(3) Angular Rate Progression](j12_3_figure_2_base_angular_rate.png)",
        "- [Figure 3: Platform Attitude Residuals](j12_3_figure_3_platform_attitude_residuals.png)",
        "- [Figure 4: Base Rate vs Platform SO(3) Error](j12_3_figure_4_angular_rate_vs_so3_error.png)",
        "- [Figure 5: PID Residuals & Saturation Boundary](j12_3_figure_5_pid_residuals_and_saturation.png)",
        "- [Figure 6: Six-Axis Actuator Step Rates](j12_3_figure_6_actuator_step_rates.png)",
        "- [Figure 7: Six-Axis Tracking Error vs Rate](j12_3_figure_7_tracking_error_vs_rate.png)",
        "- [Figure 8: Motor Current vs Rate](j12_3_figure_8_motor_current_vs_rate.png)",
        "- [Figure 9: Disturbance Rejection Comparison](j12_3_figure_9_disturbance_rejection_comparison.png)",
        "",
    ])

    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[ANALYZER] Wrote markdown report to {report_path}")


def main():
    parser = argparse.ArgumentParser(description="J12-3 Small-Angle Dynamic Performance Analyzer")
    parser.add_argument("--input-csv", required=True, help="Path to telemetry CSV")
    parser.add_argument("--output-dir", default=None, help="Directory to save figures and reports")
    parser.add_argument("--custom-segments", default=None,
                        help="Optional custom segments format: 'Name1:t0-t1,Name2:t0-t1'")
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
    n = len(active_rows)
    print(f"[ANALYZER] Loaded {len(rows)} telemetry rows ({n} ACTIVE) from {csv_path.name}")

    timestamps = np.array([float(r["timestamp_ns"]) for r in active_rows])
    t_s = (timestamps - timestamps[0]) * 1e-9

    rolls = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])
    pitches = np.array([float(r["base_rel_pitch_deg"]) for r in active_rows])
    yaws = np.array([float(r["base_rel_yaw_deg"]) for r in active_rows])
    dts = np.array([float(r.get("dt_ms", 20.0)) for r in active_rows]) * 1e-3

    # Extract rotation matrices
    R_mats = np.array([json.loads(r["R_WB"]) for r in active_rows])
    omega_mag = compute_so3_angular_rate(R_mats, dts)

    custom_intervals = None
    if args.custom_segments:
        custom_intervals = []
        for item in args.custom_segments.split(","):
            part_name, part_time = item.strip().split(":")
            t0_str, t1_str = part_time.split("-")
            custom_intervals.append((part_name.strip(), float(t0_str), float(t1_str)))

    segments = segment_dynamic_runs(t_s, rolls, pitches, yaws, omega_mag, custom_intervals)
    print(f"[ANALYZER] Identified {len(segments)} dynamic segments:")
    for s in segments:
        print(f"  - {s['name']}: [{s['start_time_s']:.1f}s - {s['end_time_s']:.1f}s, dur={s['duration_s']:.1f}s]")

    metrics_list = [analyze_segment_metrics(s, active_rows, t_s, omega_mag) for s in segments]

    # Save metrics JSON
    json_path = out_dir / "j12_3_dynamic_performance_metrics.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metrics_list, f, indent=2)
    print(f"[ANALYZER] Saved dynamic metrics JSON to {json_path}")

    # Generate Figures and Report
    generate_figures(active_rows, segments, metrics_list, out_dir, t_s, omega_mag)
    generate_markdown_report(metrics_list, out_dir)


if __name__ == "__main__":
    main()
