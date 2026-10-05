#!/usr/bin/env python3
"""Stage J12-1: Large-Angle Static Load Boundary Qualification Offline Analyzer.

Processes telemetry from Feedforward-only static staircase runs (e.g. Base Roll at
-25.0°, -27.5°, -30.0°, -32.5° with 3s plateaus).

Extracts:
1. Plateau segmentation (detects quasi-static intervals with |dRoll/dt| < threshold)
2. Statistical metrics per plateau:
   - Base R/P/Y and SO(3)
   - Feedforward command R/P/Y
   - Platform R/P/Y and SO(3) error
   - Six-axis commanded, feedback, tracking error
   - Six-axis currents: mean, median, max, P95, P99
   - Target and feedback step rates
   - Real-time dt and feedback age
3. Output Figures:
   - Figure A: Roll angle -> ID1-ID6 actuator position
   - Figure B: Roll angle -> ID1-ID6 steady-state current (inflection analysis)
   - Figure C: Roll angle -> ID1-ID6 tracking error
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


def parse_telemetry_csv(csv_path: str) -> List[Dict[str, Any]]:
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def detect_plateaus(
    rows: List[Dict[str, Any]],
    min_duration_s: float = 2.0,
    rate_threshold_deg_s: float = 1.0,
    target_angles_deg: Optional[List[float]] = None,
) -> List[Dict[str, Any]]:
    """Segment telemetry into quasi-static plateaus."""
    active_rows = [r for r in rows if r.get("state") == "ACTIVE"]
    if not active_rows:
        active_rows = rows

    timestamps = np.array([float(r["timestamp_ns"]) for r in active_rows])
    t_s = (timestamps - timestamps[0]) * 1e-9
    rolls = np.array([float(r["base_rel_roll_deg"]) for r in active_rows])

    dt = np.diff(t_s)
    dt[dt <= 0] = 0.020
    roll_rates = np.abs(np.diff(rolls) / dt)
    roll_rates = np.append(roll_rates, roll_rates[-1])

    is_static = roll_rates < rate_threshold_deg_s

    # Find contiguous static intervals
    plateaus = []
    in_plateau = False
    start_idx = 0

    for i, static in enumerate(is_static):
        if static and not in_plateau:
            in_plateau = True
            start_idx = i
        elif not static and in_plateau:
            in_plateau = False
            duration = t_s[i - 1] - t_s[start_idx]
            if duration >= min_duration_s:
                plateaus.append((start_idx, i - 1, duration))

    if in_plateau:
        duration = t_s[-1] - t_s[start_idx]
        if duration >= min_duration_s:
            plateaus.append((start_idx, len(is_static) - 1, duration))

    # Match each interval to candidate target angle or mean roll
    segmented_results = []
    for p_idx, (s_idx, e_idx, dur) in enumerate(plateaus):
        p_rows = active_rows[s_idx : e_idx + 1]
        p_rolls = [float(r["base_rel_roll_deg"]) for r in p_rows]
        mean_roll = float(np.mean(p_rolls))

        segmented_results.append({
            "plateau_index": p_idx + 1,
            "start_time_s": float(t_s[s_idx]),
            "end_time_s": float(t_s[e_idx]),
            "duration_s": float(dur),
            "sample_count": len(p_rows),
            "nominal_roll_deg": round(mean_roll, 1),
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
            "fb_steps_mean": float(np.mean(s_fb)),
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
        "nominal_roll_deg": p_data["nominal_roll_deg"],
        "duration_s": p_data["duration_s"],
        "sample_count": n,
        "base_attitude": {
            "roll_mean_deg": float(np.mean(base_r)),
            "roll_std_deg": float(np.std(base_r)),
            "pitch_mean_deg": float(np.mean(base_p)),
            "yaw_mean_deg": float(np.mean(base_y)),
            "so3_mean_deg": float(np.mean(base_so3)),
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
        },
        "axis_metrics": axis_metrics,
        "timing": {
            "dt_mean_ms": float(np.mean(dts)),
            "dt_max_ms": float(np.max(dts)),
            "feedback_age_mean_ms": float(np.mean(fb_ages)),
            "feedback_age_max_ms": float(np.max(fb_ages)),
        },
    }


def generate_figures(metrics_list: List[Dict[str, Any]], out_dir: Path) -> None:
    if not metrics_list:
        print("[ANALYZER] No plateaus detected to generate figures.")
        return

    rolls = [m["nominal_roll_deg"] for m in metrics_list]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    plt.rcParams["font.sans-serif"] = "DejaVu Sans"
    plt.rcParams["axes.edgecolor"] = "#333333"

    # Figure A: Roll angle -> ID1-ID6 actuator position
    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)
    for i in range(6):
        ax_id = f"ID{i+1}"
        positions = [m["axis_metrics"][ax_id]["fb_steps_mean"] for m in metrics_list]
        ax.plot(rolls, positions, marker="o", lw=1.8, color=colors[i], label=f"{ax_id} Steps")
    ax.axhline(1000, color="gray", ls=":", label="Operating Home (1000)")
    ax.set_title("Figure A: Base Roll Angle vs. Six-Axis Actuator Positions", fontsize=12, fontweight="bold")
    ax.set_xlabel("Nominal Base Roll Angle (°)", fontsize=10)
    ax.set_ylabel("Feedback Steps", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="best", framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_1_figure_a_roll_vs_actuator_position.png")
    plt.close(fig)

    # Figure B: Roll angle -> ID1-ID6 steady-state current
    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)
    for i in range(6):
        ax_id = f"ID{i+1}"
        currents_mean = [m["axis_metrics"][ax_id]["current_mean_mA"] for m in metrics_list]
        lw = 2.2 if ax_id in ["ID3", "ID4", "ID5"] else 1.2
        ax.plot(rolls, currents_mean, marker="s", lw=lw, color=colors[i], label=f"{ax_id} Mean Current")
    ax.axhline(500, color="red", ls="--", lw=2.0, label="Overcurrent Safety Limit (500 mA)")
    ax.axhline(400, color="orange", ls=":", lw=1.5, label="Early Warning Gate (400 mA)")
    ax.set_title("Figure B: Base Roll Angle vs. Six-Axis Steady-State Current", fontsize=12, fontweight="bold")
    ax.set_xlabel("Nominal Base Roll Angle (°)", fontsize=10)
    ax.set_ylabel("Motor Current (mA)", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper left", framealpha=0.9, ncol=2)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_1_figure_b_roll_vs_steady_state_current.png")
    plt.close(fig)

    # Figure C: Roll angle -> ID1-ID6 tracking error
    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)
    for i in range(6):
        ax_id = f"ID{i+1}"
        errors = [m["axis_metrics"][ax_id]["tracking_error_mean"] for m in metrics_list]
        ax.plot(rolls, errors, marker="^", lw=1.5, color=colors[i], label=f"{ax_id} Error")
    ax.set_title("Figure C: Base Roll Angle vs. Six-Axis Tracking Error", fontsize=12, fontweight="bold")
    ax.set_xlabel("Nominal Base Roll Angle (°)", fontsize=10)
    ax.set_ylabel("Tracking Error (Steps)", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="best", framealpha=0.9)
    plt.tight_layout()
    fig.savefig(out_dir / "j12_1_figure_c_roll_vs_tracking_error.png")
    plt.close(fig)

    print(f"[ANALYZER] Generated Figures A, B, and C in {out_dir}.")


def generate_markdown_report(metrics_list: List[Dict[str, Any]], out_dir: Path) -> None:
    report_path = out_dir / "j12_1_static_load_plateau_report.md"
    lines = [
        "# J12-1 Large-Angle Static Load Boundary Qualification Report",
        "",
        f"**Run Date**: {Path(out_dir).name}",
        f"**Detected Plateaus**: {len(metrics_list)}",
        "",
        "## 1. Plateau Summary Table",
        "",
        "| Plateau | Nom Base Roll | Dur (s) | Cmd Roll (°) | ID1 curr (mA) | ID2 curr (mA) | ID3 curr (mA) | ID4 curr (mA) | ID5 curr (mA) | ID6 curr (mA) | Max Err (steps) |",
        "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]

    for m in metrics_list:
        p_id = m["plateau_index"]
        b_roll = m["nominal_roll_deg"]
        dur = round(m["duration_s"], 1)
        c_roll = round(m["feedforward_command"]["roll_mean_deg"], 1)
        c1 = round(m["axis_metrics"]["ID1"]["current_mean_mA"], 1)
        c2 = round(m["axis_metrics"]["ID2"]["current_mean_mA"], 1)
        c3 = round(m["axis_metrics"]["ID3"]["current_mean_mA"], 1)
        c4 = round(m["axis_metrics"]["ID4"]["current_mean_mA"], 1)
        c5 = round(m["axis_metrics"]["ID5"]["current_mean_mA"], 1)
        c6 = round(m["axis_metrics"]["ID6"]["current_mean_mA"], 1)
        max_err = max(m["axis_metrics"][f"ID{i+1}"]["tracking_error_max"] for i in range(6))
        lines.append(f"| P{p_id} | {b_roll:+.1f}° | {dur}s | {c_roll:+.1f}° | {c1} | {c2} | **{c3}** | **{c4}** | **{c5}** | {c6} | {max_err} |")

    lines.extend([
        "",
        "## 2. Multi-Axis Current Escalation & Inflection Analysis",
        "",
    ])

    if len(metrics_list) >= 2:
        rolls = [m["nominal_roll_deg"] for m in metrics_list]
        id5_currs = [m["axis_metrics"]["ID5"]["current_mean_mA"] for m in metrics_list]
        id3_currs = [m["axis_metrics"]["ID3"]["current_mean_mA"] for m in metrics_list]

        lines.append(f"- **Roll Progression**: {rolls}")
        lines.append(f"- **ID5 Current Trend**: {id5_currs} mA")
        lines.append(f"- **ID3 Current Trend**: {id3_currs} mA")

        # Check for rapid acceleration
        if len(id5_currs) >= 3:
            d_curr = np.diff(id5_currs)
            d2_curr = np.diff(d_curr)
            if any(d2 > 20.0 for d2 in d2_curr):
                lines.append("- **Inflection Verdict**: **NON-LINEAR RAPID ESCALATION DETECTED** (second difference > 20 mA).")
            else:
                lines.append("- **Inflection Verdict**: **APPROXIMATELY SMOOTH / LINEAR TREND**.")
    else:
        lines.append("- Insufficient plateaus (<2) to evaluate slope or inflection.")

    lines.extend([
        "",
        "## 3. Generated Figures",
        "",
        "- [Figure A: Roll Angle vs Actuator Position](j12_1_figure_a_roll_vs_actuator_position.png)",
        "- [Figure B: Roll Angle vs Steady-State Current](j12_1_figure_b_roll_vs_steady_state_current.png)",
        "- [Figure C: Roll Angle vs Tracking Error](j12_1_figure_c_roll_vs_tracking_error.png)",
        "",
    ])

    report_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[ANALYZER] Wrote markdown report to {report_path}")


def main():
    parser = argparse.ArgumentParser(description="J12-1 Static Load Plateau Analyzer")
    parser.add_argument("--input-csv", required=True, help="Path to telemetry CSV")
    parser.add_argument("--output-dir", default=None, help="Directory to save figures and reports")
    parser.add_argument("--min-duration", type=float, default=2.0, help="Minimum plateau duration in seconds")
    parser.add_argument("--rate-threshold", type=float, default=1.0, help="Max base roll angular rate in deg/s for plateau")
    args = parser.parse_args()

    csv_path = Path(args.input_csv)
    if not csv_path.exists():
        print(f"[ERROR] Input CSV not found: {csv_path}")
        sys.exit(1)

    out_dir = Path(args.output_dir) if args.output_dir else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = parse_telemetry_csv(str(csv_path))
    print(f"[ANALYZER] Loaded {len(rows)} telemetry rows from {csv_path.name}")

    plateaus = detect_plateaus(rows, min_duration_s=args.min_duration, rate_threshold_deg_s=args.rate_threshold)
    print(f"[ANALYZER] Detected {len(plateaus)} quasi-static plateaus.")

    metrics_list = [compute_plateau_metrics(p) for p in plateaus]

    # Save metrics JSON
    json_path = out_dir / "j12_1_static_load_plateau_metrics.json"
    with open(json_path, "w") as f:
        json.dump(metrics_list, f, indent=2)
    print(f"[ANALYZER] Saved plateau metrics JSON to {json_path}")

    # Generate Figures and Report
    generate_figures(metrics_list, out_dir)
    generate_markdown_report(metrics_list, out_dir)


if __name__ == "__main__":
    main()
