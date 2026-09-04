#!/usr/bin/env python3
"""J9.1-4F Platform IMU lever-arm identifiability audit.

This is an offline-only audit. It reads frozen J9.1 attitude artifacts and
checks whether they contain enough telemetry to identify the Platform H30
lever arm r_P_IP from the rigid-body model:

    a_I - a_P = alpha x r + omega x (omega x r)

No ROS nodes, motor drivers, hardware interfaces, acceleration integration,
Kalman filters, or controller code are used.
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
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_ATTITUDE_DIRS = (
    REPO_ROOT / "results" / "j9_1" / "open_loop_attitude",
    REPO_ROOT / "results" / "j9_1" / "closed_loop_attitude",
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "results" / "j9_1" / "platform_lever_arm_analysis"

CORE_FIELDS = {
    "host_monotonic_time_ns",
    "stage",
    "qx",
    "qy",
    "qz",
    "qw",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
}
JOINT_FEEDBACK_FIELDS = {f"joint_fb_{i}" for i in range(1, 7)}
JOINT_COMMAND_FIELDS = {f"joint_cmd_{i}" for i in range(1, 7)}
REFERENCE_FIELD_SETS = (
    {"command_roll_deg", "command_pitch_deg", "command_yaw_deg"},
    {"roll_ref_deg", "pitch_ref_deg", "yaw_ref_deg"},
)
PLATFORM_ACCEL_FIELD_SETS = (
    {"p_aw_x", "p_aw_y", "p_aw_z"},
    {"platform_aw_x", "platform_aw_y", "platform_aw_z"},
    {"ax_m_s2", "ay_m_s2", "az_m_s2"},
    {"linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"},
)

R_P_IP = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=float,
)
FILTER_WINDOWS = (5, 11, 21)


@dataclass(frozen=True)
class DatasetSpec:
    stem: str
    csv_path: Path
    metadata_path: Path
    family: str


def load_metadata(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_header(path: Path) -> List[str]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        return next(reader)


def load_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def discover_attitude_datasets(attitude_dirs: Sequence[Path]) -> List[DatasetSpec]:
    specs: List[DatasetSpec] = []
    for directory in attitude_dirs:
        if not directory.exists():
            continue
        for csv_path in sorted(directory.glob("*.csv")):
            meta_path = csv_path.with_name(f"{csv_path.stem}_metadata.json")
            if not meta_path.exists():
                continue
            specs.append(DatasetSpec(csv_path.stem, csv_path, meta_path, directory.name))
    return specs


def has_any_field_set(fields: set[str], choices: Sequence[set[str]]) -> bool:
    return any(choice.issubset(fields) for choice in choices)


def classify_dataset(spec: DatasetSpec) -> Dict[str, Any]:
    header = load_header(spec.csv_path)
    fields = set(header)
    meta = load_metadata(spec.metadata_path)
    stages = meta.get("stages_executed") or []
    if not stages:
        try:
            stages = sorted({row["stage"] for row in load_rows(spec.csv_path) if row.get("stage")})
        except Exception:
            stages = []

    has_core = CORE_FIELDS.issubset(fields)
    has_accel = has_any_field_set(fields, PLATFORM_ACCEL_FIELD_SETS)
    has_reference = has_any_field_set(fields, REFERENCE_FIELD_SETS)
    has_joint_feedback = JOINT_FEEDBACK_FIELDS.issubset(fields)
    has_joint_command = JOINT_COMMAND_FIELDS.issubset(fields)
    status = meta.get("status", "UNKNOWN")
    hardware_mode = meta.get("hardware_mode", "UNKNOWN")
    usable_for_fit = bool(status == "PASS" and hardware_mode == "active" and has_core and has_accel)

    missing_for_fit = []
    if status != "PASS":
        missing_for_fit.append("metadata status is not PASS")
    if hardware_mode != "active":
        missing_for_fit.append("metadata hardware_mode is not active")
    if not has_core:
        missing_for_fit.append("missing quaternion/gyro/timestamp/stage core fields")
    if not has_accel:
        missing_for_fit.append("missing Platform acceleration telemetry required for b = A r")

    return {
        "stem": spec.stem,
        "family": spec.family,
        "csv_path": str(spec.csv_path),
        "metadata_path": str(spec.metadata_path),
        "status": status,
        "hardware_mode": hardware_mode,
        "total_recorded_samples": meta.get("total_recorded_samples"),
        "final_write_count": meta.get("final_write_count"),
        "stages_executed": stages,
        "has_quaternion": {"qx", "qy", "qz", "qw"}.issubset(fields),
        "has_gyro_xyz": {"gx_rad_s", "gy_rad_s", "gz_rad_s"}.issubset(fields),
        "has_platform_accel_xyz": has_accel,
        "has_timestamp": "host_monotonic_time_ns" in fields,
        "has_stage_labels": "stage" in fields,
        "has_cartesian_reference": has_reference,
        "has_joint_feedback": has_joint_feedback,
        "has_joint_command": has_joint_command,
        "has_fk_origin_translation": False,
        "usable_for_lever_arm_fit": usable_for_fit,
        "missing_for_lever_arm_fit": missing_for_fit,
    }


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        raise ValueError("moving average window must be odd")
    pad = window // 2
    padded = np.pad(values, ((pad, pad), (0, 0)), mode="edge")
    kernel = np.ones(window, dtype=float) / float(window)
    return np.vstack([np.convolve(padded[:, i], kernel, mode="valid") for i in range(values.shape[1])]).T


def skew(v: Sequence[float]) -> np.ndarray:
    x, y, z = v
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def scalar_stats(values: Sequence[float]) -> Dict[str, float]:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"mean": 0.0, "median": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
    sorted_vals = sorted(vals)
    return {
        "mean": float(statistics.fmean(vals)),
        "median": float(statistics.median(vals)),
        "p95": float(np.percentile(sorted_vals, 95.0)),
        "p99": float(np.percentile(sorted_vals, 99.0)),
        "max": float(max(vals)),
    }


def action_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, str]]:
    selected = []
    for row in rows:
        stage = row.get("stage", "")
        if "ret" in stage or "home" in stage or stage == "pre_home" or stage == "post_home":
            continue
        if stage:
            selected.append(row)
    return selected


def excitation_audit(spec: DatasetSpec, filter_window: int) -> Dict[str, Any]:
    rows = action_rows(load_rows(spec.csv_path))
    if len(rows) < max(filter_window + 2, 8):
        return {
            "stem": spec.stem,
            "filter_window_samples": filter_window,
            "sample_count": len(rows),
            "rank": 0,
            "singular_values": [],
            "condition_number": math.inf,
            "dt_ms": scalar_stats([]),
        }

    t_ns = np.array([float(r["host_monotonic_time_ns"]) for r in rows], dtype=float)
    t_s = (t_ns - t_ns[0]) / 1e9
    omega_i = np.array(
        [[float(r["gx_rad_s"]), float(r["gy_rad_s"]), float(r["gz_rad_s"])] for r in rows],
        dtype=float,
    )
    omega_p = (R_P_IP @ omega_i.T).T
    omega_f = moving_average(omega_p, filter_window)
    alpha_p = np.zeros_like(omega_f)
    for axis in range(3):
        alpha_p[:, axis] = np.gradient(omega_f[:, axis], t_s, edge_order=1)

    matrices = []
    for w, a in zip(omega_f, alpha_p):
        matrices.append(skew(a) + skew(w) @ skew(w))
    a_stack = np.vstack(matrices)
    singular_values = np.linalg.svd(a_stack, compute_uv=False)
    rank = int(np.linalg.matrix_rank(a_stack, tol=max(a_stack.shape) * np.finfo(float).eps * singular_values[0])) if singular_values.size else 0
    cond = float(singular_values[0] / singular_values[-1]) if singular_values.size and singular_values[-1] > 1e-12 else math.inf
    dts_ms = np.diff(t_s) * 1000.0
    return {
        "stem": spec.stem,
        "filter_window_samples": filter_window,
        "sample_count": len(rows),
        "rank": rank,
        "singular_values": [float(v) for v in singular_values],
        "condition_number": cond,
        "dt_ms": scalar_stats(dts_ms.tolist()),
    }


def summarize_by_family(classified: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    usable = [d for d in classified if d["usable_for_lever_arm_fit"]]
    accel_missing = [d["stem"] for d in classified if not d["has_platform_accel_xyz"]]
    return {
        "candidate_count": len(classified),
        "usable_for_lever_arm_fit_count": len(usable),
        "all_candidate_stems_missing_platform_accel": len(accel_missing) == len(classified) if classified else False,
        "stems_missing_platform_accel": accel_missing,
    }


def render_markdown(report: Dict[str, Any]) -> str:
    lines = [
        "# J9.1-4F Platform IMU Lever-Arm Identifiability Audit",
        "",
        "Scope: offline audit of frozen J9.1 attitude artifacts only. No hardware execution.",
        "R_P_IP is frozen as Rz(+90 deg); Base IMU geometry is out of scope.",
        "",
        "## Decision",
        report["decision"]["verdict"],
        report["decision"]["summary"],
        "",
        "## Dataset Availability",
    ]
    for dataset in report["datasets"]:
        usable = "yes" if dataset["usable_for_lever_arm_fit"] else "no"
        accel = "yes" if dataset["has_platform_accel_xyz"] else "no"
        lines.append(f"- {dataset['stem']}: usable_for_fit={usable}, accel={accel}, stages={','.join(dataset['stages_executed'])}")
    lines.extend([
        "",
        "## Estimates",
        "No lever-arm least-squares estimate was accepted because b = a_I - a_P cannot be formed from these attitude artifacts.",
        "",
        "## Next Step",
        report["next_step"]["recommendation"],
    ])
    return "\n".join(lines) + "\n"


def write_dataset_audit_csv(path: Path, datasets: Sequence[Dict[str, Any]]) -> None:
    fields = [
        "stem",
        "family",
        "status",
        "hardware_mode",
        "has_quaternion",
        "has_gyro_xyz",
        "has_platform_accel_xyz",
        "has_timestamp",
        "has_stage_labels",
        "has_cartesian_reference",
        "has_joint_feedback",
        "has_joint_command",
        "usable_for_lever_arm_fit",
        "missing_for_lever_arm_fit",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in datasets:
            out = {field: row.get(field) for field in fields}
            out["missing_for_lever_arm_fit"] = "; ".join(row.get("missing_for_lever_arm_fit", []))
            writer.writerow(out)


def analyze(
    attitude_dirs: Sequence[Path] = DEFAULT_ATTITUDE_DIRS,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
) -> Dict[str, Any]:
    specs = discover_attitude_datasets(attitude_dirs)
    classified = [classify_dataset(spec) for spec in specs]

    excitation: Dict[str, Dict[str, Any]] = {}
    for window in FILTER_WINDOWS:
        excitation[str(window)] = {spec.stem: excitation_audit(spec, window) for spec in specs}

    availability = summarize_by_family(classified)
    usable = availability["usable_for_lever_arm_fit_count"]
    verdict = "NOT_RELIABLY_IDENTIFIABLE" if usable == 0 else "PARTIALLY_IDENTIFIABLE"

    estimates = {
        "roll_mm": None,
        "pitch_mm": None,
        "yaw_mm": None,
        "combined_mm": None,
        "reason": "No accepted estimate: frozen attitude artifacts do not include Platform acceleration telemetry, so b = a_I - a_P cannot be formed. FK origin acceleration is also unavailable as reliable second-derivative evidence.",
    }
    residual_reduction = {
        "attitude_residual_reduction_available": False,
        "xyz_m_accel_center_candidate_available": False,
        "reason": "No reliable candidate r_P_IP was identified; lever-arm correction was not applied to frozen XYZ artifacts.",
    }

    report = {
        "analysis_name": "J9.1-4F_Platform_IMU_Lever_Arm_Identifiability_Audit",
        "timestamp": time.strftime("%Y%m%d_%H%M%S"),
        "scope": "offline_only_frozen_attitude_artifacts",
        "hardware_write_count": 0,
        "frame_convention": {
            "r_definition": "r_P_IP is from Platform origin P to Platform H30 measurement point I_P, expressed in Platform frame P.",
            "R_P_IP": "Rz(+90 deg), frozen from J9.1; vector transform v_P = R_P_IP * v_IP.",
            "base_imu": "out_of_scope; R_B_IB remains UNKNOWN",
        },
        "gyro_filtering_alpha_method": {
            "method": "offline moving-average smoothing of gyro in Platform frame followed by numpy.gradient central finite difference on host_monotonic_time_ns",
            "primary_filter_window_samples": 11,
            "sensitivity_filter_window_samples": list(FILTER_WINDOWS),
            "no_parameter_tuning_against_lever_arm_result": True,
        },
        "datasets": classified,
        "availability_summary": availability,
        "angular_excitation_identifiability_audit": excitation,
        "lever_arm_estimates_mm": estimates,
        "filter_and_window_sensitivity": {
            "result": "Only A-matrix rank/conditioning sensitivity can be audited. Lever-arm estimate sensitivity is unavailable because no b vector exists.",
            "filter_windows_samples": list(FILTER_WINDOWS),
        },
        "residual_reduction_test": residual_reduction,
        "xyz_m_accel_correction_test": residual_reduction,
        "decision": {
            "verdict": verdict,
            "summary": "Current frozen attitude datasets cannot separate Platform IMU lever-arm acceleration from Platform-origin translation because they lack acceleration telemetry. Candidate r_P_IP is not reliable and was not used to correct XYZ M_accel.",
            "allowed_interpretation": "Large XYZ cross-axis acceleration may be partly consistent with measurement-point rotational contamination, but existing data do not identify the lever arm or prove causality.",
        },
        "next_step": {
            "recommendation": "Run a minimal dedicated pure-rotation characterization only after explicit hardware authorization: Roll +/-0.5 deg, Pitch +/-0.5 deg, Yaw +/-2 deg, logging Platform quaternion, gyro, raw accel, gravity-removed accel, stages, joint feedback, and FK. Then repeat identifiability before Platform-center acceleration reconstruction.",
            "do_not_do_now": ["controller", "PID tuning", "MIMO", "Kalman/ESKF", "acceleration integration", "Base disturbance experiment"],
        },
        "prohibited_interpretations": [
            "No accepted r_P_IP calibration is produced.",
            "No lever-arm effect is proven as the cause of XYZ cross-axis acceleration.",
            "No Platform-center acceleration reconstruction is validated.",
        ],
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "j9_1_4f_platform_lever_arm_identifiability.json"
    md_path = output_dir / "j9_1_4f_platform_lever_arm_identifiability.md"
    csv_path = output_dir / "j9_1_4f_attitude_dataset_availability.csv"
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")
    write_dataset_audit_csv(csv_path, classified)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="J9.1-4F offline Platform IMU lever-arm identifiability audit")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    args = parser.parse_args()
    report = analyze(output_dir=Path(args.output_dir))
    print(json.dumps({
        "analysis_name": report["analysis_name"],
        "hardware_write_count": report["hardware_write_count"],
        "candidate_count": report["availability_summary"]["candidate_count"],
        "usable_for_lever_arm_fit_count": report["availability_summary"]["usable_for_lever_arm_fit_count"],
        "verdict": report["decision"]["verdict"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
