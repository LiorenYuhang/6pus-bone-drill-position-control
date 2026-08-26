#!/usr/bin/env python3
"""Validate a measured geometry record and write a non-production candidate YAML."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
KINEMATICS_ROOT = WORKSPACE_ROOT / "src" / "pus6_kinematics"
for path in (SCRIPT_ROOT, KINEMATICS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from j6_geometry_common import (  # noqa: E402
    build_candidate_yaml,
    default_config_path,
    load_raw_yaml,
    measurement_means,
    reject_production_overwrite,
    sha256_file,
    write_yaml_once,
)
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics  # noqa: E402
from pus6_kinematics.geometry_config import load_geometry_config  # noqa: E402


def validate_geometry_candidate(
    measurement_report: dict[str, Any],
    *,
    base_config_path: str | Path | None = None,
    candidate_yaml_path: str | Path | None = None,
    measurement_source: str | None = None,
) -> dict[str, Any]:
    base_path = Path(base_config_path or default_config_path()).expanduser()
    candidate_path = Path(candidate_yaml_path).expanduser() if candidate_yaml_path else None
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase6_geometry_validation",
        "phase": "J6_PHASE6",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "hardware_mode": "offline",
        "port": None,
        "baudrate": None,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "source_report_path": measurement_source,
        "base_config_path": str(base_path),
        "candidate_config_path": str(candidate_path) if candidate_path else None,
        "pass_conditions": {},
        "failure_reasons": [],
    }
    source_passed = measurement_report.get("pass") is True
    report["pass_conditions"]["measurement_record_passed"] = source_passed
    if not source_passed:
        report["failure_reasons"].append("measurement report is not PASS")
        report["pass_conditions"].update({
            "base_geometry_yaml_loads": False,
            "candidate_created_without_overwrite": False,
            "candidate_geometry_yaml_loads": False,
            "production_geometry_not_overwritten": True,
            "zero_hardware_writes": True,
        })
        report["pass"] = False
        report["outcome"] = "FAIL"
        return report
    try:
        base_raw = load_raw_yaml(base_path)
        report["base_config_sha256"] = sha256_file(base_path)
        report["pass_conditions"]["base_geometry_yaml_loads"] = True
        if candidate_path is None:
            raise ValueError("candidate_yaml_path is required to create a validation candidate")
        reject_production_overwrite(base_path, candidate_path)
        candidate_raw = build_candidate_yaml(
            base_path,
            measurement_report,
            measurement_source=measurement_source,
        )
        write_yaml_once(candidate_raw, candidate_path)
        report["pass_conditions"]["candidate_created_without_overwrite"] = True
        candidate_config = load_geometry_config(candidate_path)
        report["pass_conditions"]["candidate_geometry_yaml_loads"] = True
        candidate_kin = Analytical6PusKinematics(config_path=candidate_path)
        means = measurement_means(measurement_report)
        report["measured_means"] = means
        report["candidate_geometry"] = {
            "base_radius_m": candidate_config.base_radius_m,
            "platform_radius_m": candidate_config.platform_radius_m,
            "strut_length_m": candidate_config.strut_length_m,
            "lambda_rad": candidate_config.lambda_rad,
            "mu_rad": candidate_config.mu_rad,
            "nominal_height_m": candidate_config.nominal_height_m,
        }
        report["pass_conditions"]["candidate_matches_measurement"] = all(
            abs(float(getattr(candidate_config, key)) - value) <= 1e-12
            for key, value in {
                "base_radius_m": means["base_radius_m"],
                "platform_radius_m": means["platform_radius_m"],
                "strut_length_m": means["strut_lengths_m"],
                "lambda_rad": means["lambda_rad"],
                "mu_rad": means["mu_rad"],
                "nominal_height_m": means["nominal_height_m"],
            }.items()
        )
        home_pose = [0.0, 0.0, candidate_kin.nominal_height, 0.0, 0.0, 0.0]
        home_sliders = candidate_kin.inverse_kinematics(home_pose)
        home_fk = candidate_kin.forward_kinematics_with_metrics(home_sliders)
        report["home_ik_sliders_m"] = home_sliders
        home_tolerance_m = 1.0 / candidate_config.steps_per_meter
        report["home_ik_tolerance_m"] = home_tolerance_m
        report["home_fk"] = {
            "pose": home_fk.pose,
            "converged": home_fk.converged,
            "residual_norm_m": home_fk.residual_norm,
            "iterations": home_fk.iterations,
        }
        report["pass_conditions"]["home_ik_is_midstroke"] = all(
            abs(value - 0.05) <= home_tolerance_m for value in home_sliders
        )
        report["pass_conditions"]["home_fk_converged"] = home_fk.converged
        report["pass_conditions"]["home_fk_residual_within_limit"] = home_fk.residual_norm <= 1e-9
        report["candidate_sha256"] = sha256_file(candidate_path)
    except Exception as exc:
        report["failure_reasons"].append(f"candidate validation failed: {exc}")
        report["pass_conditions"].setdefault("base_geometry_yaml_loads", False)
        report["pass_conditions"].setdefault("candidate_created_without_overwrite", False)
        report["pass_conditions"].setdefault("candidate_geometry_yaml_loads", False)

    report["pass_conditions"]["production_geometry_not_overwritten"] = True
    report["pass_conditions"]["zero_hardware_writes"] = report["hardware_write_count"] == 0
    report["pass"] = not report["failure_reasons"] and all(report["pass_conditions"].values())
    report["outcome"] = "PASS" if report["pass"] else "FAIL"
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate a J6 measured geometry candidate")
    parser.add_argument("--measurement-report", type=Path, required=True)
    parser.add_argument("--base-config", type=Path, default=default_config_path())
    parser.add_argument("--candidate-yaml", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    measurement = json.loads(args.measurement_report.read_text(encoding="utf-8"))
    if not isinstance(measurement, dict):
        raise SystemExit("measurement report must be a JSON object")
    report = validate_geometry_candidate(
        measurement,
        base_config_path=args.base_config,
        candidate_yaml_path=args.candidate_yaml,
        measurement_source=str(args.measurement_report),
    )
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase6_geometry_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
