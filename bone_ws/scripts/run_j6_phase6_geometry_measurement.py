#!/usr/bin/env python3
"""Record measured post-assembly geometry without changing production YAML."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
KINEMATICS_ROOT = WORKSPACE_ROOT / "src" / "pus6_kinematics"
for path in (SCRIPT_ROOT, KINEMATICS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from j6_geometry_common import normalize_measurements  # noqa: E402

MAX_STRUT_SPREAD_M = 0.00005


def record_measurements(
    measurements: Mapping[str, Any],
    *,
    operator: str | None = None,
    instrument: str | None = None,
    session_id: str | None = None,
    max_strut_spread_m: float = MAX_STRUT_SPREAD_M,
) -> dict[str, Any]:
    normalized = normalize_measurements(measurements)
    if max_strut_spread_m < 0.0:
        raise ValueError("max_strut_spread_m must be non-negative")
    strut_spread = normalized["strut_lengths_m"]["spread"]
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase6_geometry_measurement",
        "phase": "J6_PHASE6",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "session_id": session_id,
        "hardware_mode": "offline",
        "port": None,
        "baudrate": None,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "measurements": normalized,
        "units": {"length": "m", "angle": "rad", "raw_position": "step"},
        "operator": operator,
        "instrument": instrument,
        "max_strut_spread_m": max_strut_spread_m,
        "pass_conditions": {
            "required_measurements_complete": True,
            "six_strut_measurements_present": len(normalized["strut_lengths_m"]["samples"]) >= 6,
            "strut_repeatability_within_limit": strut_spread <= max_strut_spread_m,
            "no_hardware_writes": True,
        },
        "failure_reasons": [],
        "evidence": {"measurement_record": "operator_supplied_samples"},
    }
    if not report["pass_conditions"]["strut_repeatability_within_limit"]:
        report["failure_reasons"].append(
            f"strut length spread {strut_spread:.9g}m exceeds {max_strut_spread_m:.9g}m"
        )
    report["pass"] = not report["failure_reasons"] and all(report["pass_conditions"].values())
    report["outcome"] = "PASS" if report["pass"] else "FAIL"
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Record J6 Phase 6 geometry measurements")
    parser.add_argument("--input", type=Path, required=True, help="JSON measurement mapping")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--operator")
    parser.add_argument("--instrument")
    parser.add_argument("--session-id")
    parser.add_argument("--max-strut-spread-m", type=float, default=MAX_STRUT_SPREAD_M)
    args = parser.parse_args(argv)
    raw = json.loads(args.input.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise SystemExit("measurement input must be a JSON object")
    report = record_measurements(
        raw,
        operator=args.operator,
        instrument=args.instrument,
        session_id=args.session_id,
        max_strut_spread_m=args.max_strut_spread_m,
    )
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase6_measurement_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
