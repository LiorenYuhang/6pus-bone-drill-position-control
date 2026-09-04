#!/usr/bin/env python3
"""J9.1-4E offline XYZ translation coupling audit.

This script reads the six frozen J9.1-4B/4C/4D translation characterization
artifacts and computes actuator-feedback-based FK response matrices plus
Platform-IMU measurement-point dynamic response matrices.

It is strictly offline: no ROS nodes, no motor drivers, no hardware access.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_INPUT_DIR = REPO_ROOT / "results" / "j9_1" / "translation_characterization"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "results" / "j9_1" / "translation_coupling_analysis"
COMMAND_AMPLITUDE_MM = 0.5
RESAMPLE_RATE_HZ = 200.0
ONSET_WINDOW_S = 1.0
AXES = ("x", "y", "z")
ROT_AXES = ("roll", "pitch", "yaw")

FORMAL_RUNS = {
    ("x", "+"): "j9_1_x_translation_pos05mm_20260903_203327",
    ("x", "-"): "j9_1_x_translation_neg05mm_20260903_204626",
    ("y", "+"): "j9_1_y_translation_pos05mm_20260904_101138",
    ("y", "-"): "j9_1_y_translation_neg05mm_20260904_120930",
    ("z", "+"): "j9_1_z_translation_pos05mm_20260904_123707",
    ("z", "-"): "j9_1_z_translation_neg05mm_20260904_131517",
}


@dataclass(frozen=True)
class RunSpec:
    command_axis: str
    sign: str
    stem: str
    csv_path: Path
    metadata_path: Path


def rms(values: np.ndarray) -> float:
    return float(math.sqrt(float(np.mean(values * values)))) if values.size else 0.0


def l2_norm(values: np.ndarray, dt_s: float) -> float:
    return float(math.sqrt(float(np.sum(values * values) * dt_s))) if values.size else 0.0


def peak_to_peak(values: np.ndarray) -> float:
    return float(np.max(values) - np.min(values)) if values.size else 0.0


def scalar_stats(values: Sequence[float]) -> Dict[str, float]:
    vals = [float(v) for v in values]
    if not vals:
        return {"mean": 0.0, "std": 0.0, "rms": 0.0, "min": 0.0, "max": 0.0}
    mean = statistics.fmean(vals)
    return {
        "mean": float(mean),
        "std": float(statistics.pstdev(vals)),
        "rms": float(math.sqrt(statistics.fmean([v * v for v in vals]))),
        "min": float(min(vals)),
        "max": float(max(vals)),
    }


def matrix_to_dict(matrix: np.ndarray, rows: Sequence[str], cols: Sequence[str]) -> Dict[str, Dict[str, float]]:
    return {
        row: {col: float(matrix[i, j]) for j, col in enumerate(cols)}
        for i, row in enumerate(rows)
    }


def load_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def load_metadata(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def specs(input_dir: Path) -> List[RunSpec]:
    return [
        RunSpec(axis, sign, stem, input_dir / f"{stem}.csv", input_dir / f"{stem}_metadata.json")
        for (axis, sign), stem in FORMAL_RUNS.items()
    ]


def require_formal_inputs(input_dir: Path) -> List[RunSpec]:
    run_specs = specs(input_dir)
    missing = [str(p) for spec in run_specs for p in (spec.csv_path, spec.metadata_path) if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing formal J9.1 translation artifact(s): " + ", ".join(missing))
    return run_specs


def fk_vector(meta: Dict[str, Any]) -> np.ndarray:
    fk = meta["metrics"]["fk_local_displacement"]
    return np.array([fk["dx_mm"], fk["dy_mm"], fk["dz_mm"]], dtype=float)


def rpy_vector(meta: Dict[str, Any]) -> np.ndarray:
    fk = meta["metrics"]["fk_local_displacement"]
    return np.array([fk["droll_deg"], fk["dpitch_deg"], fk["dyaw_deg"]], dtype=float)


def compute_odd_matrix(
    metas: Dict[Tuple[str, str], Dict[str, Any]],
    vector_fn,
    output_units_per_mm: bool = True,
) -> np.ndarray:
    cols = []
    denom = 2.0 * COMMAND_AMPLITUDE_MM if output_units_per_mm else 2.0
    for axis in AXES:
        pos = vector_fn(metas[(axis, "+")])
        neg = vector_fn(metas[(axis, "-")])
        cols.append((pos - neg) / denom)
    return np.column_stack(cols)


def condition_number(matrix: np.ndarray) -> float:
    return float(np.linalg.cond(matrix))


def resample_run(rows: List[Dict[str, str]], command_axis: str, sign: str, window_s: float, rate_hz: float) -> Dict[str, Any]:
    action_prefix = f"action_{command_axis}_{'pos' if sign == '+' else 'neg'}"
    pre_rows = [r for r in rows if r["stage"] == "pre_home"]
    action_rows = [r for r in rows if r["stage"] == action_prefix]
    post_rows = [r for r in rows if r["stage"] == "post_home"]
    if not action_rows:
        raise ValueError(f"No rows found for stage {action_prefix}")

    def mean_field(field: str, src_rows: List[Dict[str, str]]) -> float:
        return statistics.fmean(float(r[field]) for r in src_rows) if src_rows else 0.0

    baseline_rows = pre_rows[-100:] if len(pre_rows) >= 100 else pre_rows
    baselines = {
        field: mean_field(field, baseline_rows)
        for field in (
            "p_aw_x", "p_aw_y", "p_aw_z", "p_ax_raw", "p_ay_raw", "p_az_raw",
            "p_gx_rad_s", "p_gy_rad_s", "p_gz_rad_s",
        )
    }
    base_baselines = {
        field: mean_field(field, baseline_rows)
        for field in ("b_aw_x", "b_aw_y", "b_aw_z", "b_ax_raw", "b_ay_raw", "b_az_raw")
    }

    t0 = float(action_rows[0]["host_monotonic_time_ns"])
    t = np.array([(float(r["host_monotonic_time_ns"]) - t0) / 1e9 for r in action_rows], dtype=float)
    order = np.argsort(t)
    t = t[order]
    grid = np.arange(0.0, window_s + 0.5 / rate_hz, 1.0 / rate_hz, dtype=float)
    if t.size < 2:
        raise ValueError(f"Not enough samples for {action_prefix}")

    def interp_field(field: str, baseline: float = 0.0, scale: float = 1.0) -> np.ndarray:
        values = np.array([(float(r[field]) - baseline) * scale for r in action_rows], dtype=float)[order]
        return np.interp(grid, t, values, left=values[0], right=values[-1])

    accel = np.vstack([
        interp_field("p_aw_x", baselines["p_aw_x"]),
        interp_field("p_aw_y", baselines["p_aw_y"]),
        interp_field("p_aw_z", baselines["p_aw_z"]),
    ])
    gyro_deg_s = np.vstack([
        interp_field("p_gx_rad_s", baselines["p_gx_rad_s"], 180.0 / math.pi),
        interp_field("p_gy_rad_s", baselines["p_gy_rad_s"], 180.0 / math.pi),
        interp_field("p_gz_rad_s", baselines["p_gz_rad_s"], 180.0 / math.pi),
    ])
    raw_z_dev = interp_field("p_az_raw", baselines["p_az_raw"])

    def dt_stats(prefix: str) -> Dict[str, float]:
        stamps = []
        last = None
        for r in rows:
            stamp = int(r[prefix])
            if stamp > 0 and stamp != last:
                stamps.append(stamp)
                last = stamp
        dts = [(stamps[i] - stamps[i - 1]) / 1e6 for i in range(1, len(stamps))]
        stats = scalar_stats(dts)
        stats["rate_hz"] = 1000.0 / stats["mean"] if stats["mean"] > 0.0 else 0.0
        return stats

    offsets = [abs(int(r["p_stamp_ns"]) - int(r["b_stamp_ns"])) / 1e6 for r in rows if int(r["p_stamp_ns"]) > 0 and int(r["b_stamp_ns"]) > 0]

    static_z = {
        "platform_pre_aw_z": scalar_stats([float(r["p_aw_z"]) for r in pre_rows]),
        "platform_post_aw_z": scalar_stats([float(r["p_aw_z"]) for r in post_rows]),
        "platform_pre_raw_z": scalar_stats([float(r["p_az_raw"]) for r in pre_rows]),
        "platform_post_raw_z": scalar_stats([float(r["p_az_raw"]) for r in post_rows]),
        "base_pre_aw_z": scalar_stats([float(r["b_aw_z"]) for r in pre_rows]),
        "base_post_aw_z": scalar_stats([float(r["b_aw_z"]) for r in post_rows]),
        "base_pre_raw_z": scalar_stats([float(r["b_az_raw"]) for r in pre_rows]),
        "base_post_raw_z": scalar_stats([float(r["b_az_raw"]) for r in post_rows]),
    }

    return {
        "stage": action_prefix,
        "grid_s": grid,
        "accel_m_s2": accel,
        "gyro_deg_s": gyro_deg_s,
        "raw_z_deviation_m_s2": raw_z_dev,
        "platform_dt_ms": dt_stats("p_stamp_ns"),
        "base_dt_ms": dt_stats("b_stamp_ns"),
        "online_latest_base_offset_ms": scalar_stats(offsets),
        "static_z": static_z,
        "samples": {"pre_home": len(pre_rows), "action": len(action_rows), "post_home": len(post_rows), "total": len(rows)},
    }


def pair_dynamic_metrics(pos: Dict[str, Any], neg: Dict[str, Any], signal_key: str, rows: Sequence[str], dt_s: float) -> Dict[str, Any]:
    plus = pos[signal_key]
    minus = neg[signal_key]
    odd = (plus - minus) / 2.0
    even = (plus + minus) / 2.0

    metrics: Dict[str, Any] = {
        "odd_rms": {},
        "odd_peak_to_peak": {},
        "odd_l2": {},
        "even_rms": {},
        "even_l2": {},
        "even_to_odd_l2_ratio": {},
    }
    for i, row in enumerate(rows):
        metrics["odd_rms"][row] = rms(odd[i])
        metrics["odd_peak_to_peak"][row] = peak_to_peak(odd[i])
        metrics["odd_l2"][row] = l2_norm(odd[i], dt_s)
        metrics["even_rms"][row] = rms(even[i])
        metrics["even_l2"][row] = l2_norm(even[i], dt_s)
        denom = metrics["odd_l2"][row]
        metrics["even_to_odd_l2_ratio"][row] = metrics["even_l2"][row] / denom if denom > 1e-12 else math.inf
    return metrics


def build_dynamic_matrices(dynamic: Dict[Tuple[str, str], Dict[str, Any]]) -> Dict[str, Any]:
    dt_s = 1.0 / RESAMPLE_RATE_HZ
    accel_metrics = {}
    gyro_metrics = {}
    accel_matrix = np.zeros((3, 3), dtype=float)
    accel_ptp_matrix = np.zeros((3, 3), dtype=float)
    gyro_matrix = np.zeros((3, 3), dtype=float)

    for j, axis in enumerate(AXES):
        accel_metrics[axis] = pair_dynamic_metrics(dynamic[(axis, "+")], dynamic[(axis, "-")], "accel_m_s2", AXES, dt_s)
        gyro_metrics[axis] = pair_dynamic_metrics(dynamic[(axis, "+")], dynamic[(axis, "-")], "gyro_deg_s", ("gx", "gy", "gz"), dt_s)
        for i, row in enumerate(AXES):
            accel_matrix[i, j] = accel_metrics[axis]["odd_rms"][row]
            accel_ptp_matrix[i, j] = accel_metrics[axis]["odd_peak_to_peak"][row]
        for i, row in enumerate(("gx", "gy", "gz")):
            gyro_matrix[i, j] = gyro_metrics[axis]["odd_rms"][row]

    return {
        "M_accel_odd_rms_m_s2": matrix_to_dict(accel_matrix, AXES, AXES),
        "M_accel_odd_peak_to_peak_m_s2": matrix_to_dict(accel_ptp_matrix, AXES, AXES),
        "M_gyro_odd_rms_deg_s": matrix_to_dict(gyro_matrix, ("gx", "gy", "gz"), AXES),
        "accel_pair_metrics": accel_metrics,
        "gyro_pair_metrics": gyro_metrics,
    }


def max_offdiag_entries(matrix: Dict[str, Dict[str, float]], diagonal_rows: Sequence[str], limit: int = 6) -> List[Dict[str, Any]]:
    entries = []
    for row in matrix:
        for col, value in matrix[row].items():
            if row != col:
                entries.append({"row": row, "command": col, "value": float(value)})
    entries.sort(key=lambda item: abs(item["value"]), reverse=True)
    return entries[:limit]


def write_matrix_csv(path: Path, matrix: Dict[str, Dict[str, float]]) -> None:
    cols = list(next(iter(matrix.values())).keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["row"] + cols)
        for row, values in matrix.items():
            writer.writerow([row] + [values[col] for col in cols])


def render_markdown(report: Dict[str, Any]) -> str:
    lines = [
        "# J9.1-4E Translational Coupling Audit",
        "",
        "Scope: offline analysis of frozen formal +/-X, +/-Y, +/-Z artifacts only. No hardware execution.",
        "FK matrices are actuator-feedback-based empirical geometric responses, not ground truth Jacobians.",
        "Platform IMU matrices are measurement-point empirical dynamic responses, not platform-center plant matrices.",
        "",
        "## Decision",
        report["architecture_decision"]["recommendation"],
        "",
        "## M_FK mm_per_mm",
    ]
    for row, vals in report["M_FK_mm_per_mm"].items():
        lines.append(f"- {row}: " + ", ".join(f"cmd_{c}={v:.4f}" for c, v in vals.items()))
    lines.extend(["", "## M_accel odd RMS m_s2"])
    for row, vals in report["dynamic"]["M_accel_odd_rms_m_s2"].items():
        lines.append(f"- {row}: " + ", ".join(f"cmd_{c}={v:.4f}" for c, v in vals.items()))
    lines.extend(["", "## M_gyro odd RMS deg_s"])
    for row, vals in report["dynamic"]["M_gyro_odd_rms_deg_s"].items():
        lines.append(f"- {row}: " + ", ".join(f"cmd_{c}={v:.4f}" for c, v in vals.items()))
    lines.extend(["", "## Notes"])
    for note in report["architecture_decision"]["evidence_notes"]:
        lines.append(f"- {note}")
    return "\n".join(lines) + "\n"


def analyze(input_dir: Path = DEFAULT_INPUT_DIR, output_dir: Path = DEFAULT_OUTPUT_DIR) -> Dict[str, Any]:
    run_specs = require_formal_inputs(input_dir)
    metas: Dict[Tuple[str, str], Dict[str, Any]] = {}
    rows_by_run: Dict[Tuple[str, str], List[Dict[str, str]]] = {}
    for spec in run_specs:
        key = (spec.command_axis, spec.sign)
        metas[key] = load_metadata(spec.metadata_path)
        rows_by_run[key] = load_rows(spec.csv_path)
        if metas[key].get("status") != "PASS" or metas[key].get("metrics", {}).get("status") != "PASS":
            raise ValueError(f"Formal run {spec.stem} is not PASS")

    m_fk = compute_odd_matrix(metas, fk_vector)
    m_rot = compute_odd_matrix(metas, rpy_vector)
    fk_diag = np.abs(np.diag(m_fk))
    fk_offdiag = np.abs(m_fk.copy())
    np.fill_diagonal(fk_offdiag, 0.0)
    fk_cross_ratio = float(np.max(fk_offdiag) / np.min(fk_diag)) if np.min(fk_diag) > 1e-12 else math.inf

    dynamic: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for spec in run_specs:
        dynamic[(spec.command_axis, spec.sign)] = resample_run(
            rows_by_run[(spec.command_axis, spec.sign)],
            spec.command_axis,
            spec.sign,
            ONSET_WINDOW_S,
            RESAMPLE_RATE_HZ,
        )
    dynamic_summary = build_dynamic_matrices(dynamic)

    accel_matrix = np.array([[dynamic_summary["M_accel_odd_rms_m_s2"][r][c] for c in AXES] for r in AXES])
    gyro_matrix = np.array([[dynamic_summary["M_gyro_odd_rms_deg_s"][r][c] for c in AXES] for r in ("gx", "gy", "gz")])

    architecture_decision = {
        "recommendation": "C now, then hybrid control architecture: refine measurement/frame/lever-arm separation before committing controller structure; use FK low-frequency centering plus Platform dynamic feedback once measurement channels are separated; keep future Base IMU disturbance feedforward pending Base disturbance characterization.",
        "selected_option": "C_measurement_refinement_before_controller_selection",
        "evidence_notes": [
            "M_FK diagonal terms dominate and indicate near-decoupled actuator-feedback-based low-frequency geometry.",
            "Platform IMU dynamic matrices show large off-axis acceleration responses, especially during Y and Z commands.",
            "Large acceleration cross-axis responses co-occur with large gyro responses, so they should not be directly interpreted as translational plant coupling before frame/lever-arm refinement.",
            "3 independent SISO loops are not yet justified from Platform IMU dynamics; full 3DOF MIMO translational control is also premature because the measurement is not proven platform-center translational acceleration.",
        ],
    }

    report = {
        "analysis_name": "J9.1-4E_XYZ_Translation_Coupling_Audit",
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
        "scope": "offline_only_frozen_formal_translation_artifacts",
        "hardware_write_count": 0,
        "input_artifacts": {f"{axis}{sign}": str((input_dir / f"{stem}.csv")) for (axis, sign), stem in FORMAL_RUNS.items()},
        "resampling": {
            "method": "linear interpolation on host_monotonic_time_ns after HOME-to-ACTION stage onset; pre_home last-100-sample bias subtraction; no original CSV modification",
            "rate_hz": RESAMPLE_RATE_HZ,
            "window_s": ONSET_WINDOW_S,
        },
        "M_FK_mm_per_mm": matrix_to_dict(m_fk, AXES, AXES),
        "M_FK_condition_number": condition_number(m_fk),
        "M_FK_max_offdiag_over_min_diag": fk_cross_ratio,
        "M_rot_deg_per_mm": matrix_to_dict(m_rot, ROT_AXES, AXES),
        "M_rot_max_abs_by_command": {
            axis: {"max_abs_deg_per_mm": float(np.max(np.abs(m_rot[:, j]))), "dominant_axis": ROT_AXES[int(np.argmax(np.abs(m_rot[:, j])))]}
            for j, axis in enumerate(AXES)
        },
        "dynamic": dynamic_summary,
        "largest_accel_offdiag_odd_rms": max_offdiag_entries(dynamic_summary["M_accel_odd_rms_m_s2"], AXES),
        "largest_gyro_odd_rms": [
            {"row": row, "command": col, "value": float(gyro_matrix[i, j])}
            for i, row in enumerate(("gx", "gy", "gz"))
            for j, col in enumerate(AXES)
        ],
        "per_run_timing_and_static_z": {
            f"{axis}{sign}": {
                "samples": dynamic[(axis, sign)]["samples"],
                "platform_dt_ms": dynamic[(axis, sign)]["platform_dt_ms"],
                "base_dt_ms": dynamic[(axis, sign)]["base_dt_ms"],
                "online_latest_base_offset_ms": dynamic[(axis, sign)]["online_latest_base_offset_ms"],
                "static_z": dynamic[(axis, sign)]["static_z"],
            }
            for axis in AXES
            for sign in ("+", "-")
        },
        "architecture_decision": architecture_decision,
        "prohibited_interpretations": [
            "FK is not ground truth displacement.",
            "Platform IMU XYZ is not proven platform-center true translational acceleration.",
            "No Base/Platform acceleration subtraction is interpreted as true relative acceleration.",
            "No coupling mechanism is identified in this stage.",
        ],
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "j9_1_4e_xyz_translation_coupling_analysis.json"
    md_path = output_dir / "j9_1_4e_xyz_translation_coupling_analysis.md"
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")
    write_matrix_csv(output_dir / "j9_1_4e_M_FK_mm_per_mm.csv", report["M_FK_mm_per_mm"])
    write_matrix_csv(output_dir / "j9_1_4e_M_accel_odd_rms_m_s2.csv", report["dynamic"]["M_accel_odd_rms_m_s2"])
    write_matrix_csv(output_dir / "j9_1_4e_M_gyro_odd_rms_deg_s.csv", report["dynamic"]["M_gyro_odd_rms_deg_s"])
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="J9.1-4E offline XYZ translation coupling audit")
    parser.add_argument("--input-dir", default=str(DEFAULT_INPUT_DIR))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    args = parser.parse_args()
    report = analyze(Path(args.input_dir), Path(args.output_dir))
    print(json.dumps({
        "analysis_name": report["analysis_name"],
        "hardware_write_count": report["hardware_write_count"],
        "M_FK_condition_number": report["M_FK_condition_number"],
        "M_FK_max_offdiag_over_min_diag": report["M_FK_max_offdiag_over_min_diag"],
        "recommendation": report["architecture_decision"]["selected_option"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
