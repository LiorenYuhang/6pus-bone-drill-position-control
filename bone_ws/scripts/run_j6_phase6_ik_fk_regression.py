#!/usr/bin/env python3
"""Offline deterministic IK/FK regression wrapper for a Phase 6 candidate."""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
KINEMATICS_ROOT = WORKSPACE_ROOT / "src" / "pus6_kinematics"
if str(KINEMATICS_ROOT) not in sys.path:
    sys.path.insert(0, str(KINEMATICS_ROOT))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics  # noqa: E402
from pus6_kinematics.geometry_config import load_geometry_config  # noqa: E402


def _rotation_error_rad(first: list[list[float]], second: list[list[float]]) -> float:
    trace = sum(first[row][col] * second[row][col] for row in range(3) for col in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)


def run_ik_fk_regression(
    *,
    config_path: str | Path,
    sample_count: int = 1000,
    seed: int = 42,
    translation_tolerance_m: float = 1e-7,
    rotation_tolerance_rad: float = math.radians(1e-4),
    residual_tolerance_m: float = 1e-9,
) -> dict[str, Any]:
    if sample_count < 1:
        raise ValueError("sample_count must be >= 1")
    config = load_geometry_config(config_path)
    kin = Analytical6PusKinematics(config_path=config.path)
    rng = random.Random(seed)
    failures: list[dict[str, Any]] = []
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    residuals: list[float] = []
    converged_count = 0
    z_nom = kin.nominal_height

    for index in range(sample_count):
        pose = [
            rng.uniform(-0.004, 0.004),
            rng.uniform(-0.004, 0.004),
            z_nom + rng.uniform(-0.008, 0.008),
            rng.uniform(-0.035, 0.035),
            rng.uniform(-0.035, 0.035),
            rng.uniform(-0.035, 0.035),
        ]
        try:
            sliders = kin.inverse_kinematics(pose)
            result = kin.forward_kinematics_with_metrics(sliders)
            if result.converged:
                converged_count += 1
            translation_error = math.sqrt(
                sum((result.pose[offset] - pose[offset]) ** 2 for offset in range(3))
            )
            rotation_error = _rotation_error_rad(
                result.rotation_matrix,
                kin._rotation_matrix_zyx(pose[3], pose[4], pose[5]),
            )
            translation_errors.append(translation_error)
            rotation_errors.append(rotation_error)
            residuals.append(result.residual_norm)
            if (
                not result.converged
                or translation_error > translation_tolerance_m
                or rotation_error > rotation_tolerance_rad
                or result.residual_norm > residual_tolerance_m
            ) and len(failures) < 20:
                failures.append({
                    "sample": index,
                    "converged": result.converged,
                    "translation_error_m": translation_error,
                    "rotation_error_rad": rotation_error,
                    "residual_m": result.residual_norm,
                })
        except Exception as exc:
            if len(failures) < 20:
                failures.append({"sample": index, "error": str(exc)})

    max_translation = max(translation_errors, default=float("inf"))
    max_rotation = max(rotation_errors, default=float("inf"))
    max_residual = max(residuals, default=float("inf"))
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase6_ik_fk_regression",
        "phase": "J6_PHASE6",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "hardware_mode": "offline",
        "port": None,
        "baudrate": None,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "config_path": str(config.path),
        "seed": seed,
        "sample_count_requested": sample_count,
        "sample_count_converged": converged_count,
        "metrics": {
            "max_translation_error_m": max_translation,
            "max_rotation_error_rad": max_rotation,
            "max_residual_m": max_residual,
            "mean_translation_error_m": sum(translation_errors) / len(translation_errors) if translation_errors else float("inf"),
            "mean_rotation_error_rad": sum(rotation_errors) / len(rotation_errors) if rotation_errors else float("inf"),
        },
        "failure_samples": failures,
        "pass_conditions": {
            "geometry_loads": True,
            "all_samples_converged": converged_count == sample_count,
            "translation_error_within_tolerance": max_translation <= translation_tolerance_m,
            "rotation_error_within_tolerance": max_rotation <= rotation_tolerance_rad,
            "residual_within_tolerance": max_residual <= residual_tolerance_m,
            "zero_hardware_writes": True,
        },
        "failure_reasons": [],
    }
    report["failure_reasons"].extend(
        key for key, value in report["pass_conditions"].items() if not value
    )
    report["pass"] = not report["failure_reasons"]
    report["outcome"] = "PASS" if report["pass"] else "FAIL"
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run offline Phase 6 IK/FK regression")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    report = run_ik_fk_regression(config_path=args.config, sample_count=args.samples, seed=args.seed)
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase6_ik_fk_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
