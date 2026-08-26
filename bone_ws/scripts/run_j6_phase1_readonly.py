#!/usr/bin/env python3
"""J6 Phase 1 READ_ONLY wrapper and commissioning report generator.

The frozen J2-A auditor remains the only implementation that owns the ROS 2
driver/auditor lifecycle.  This wrapper supplies the J6 duration and transport
arguments, then normalizes its generated JSON into a J6-specific report.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
MIN_PHASE1_DURATION_S = 30.0
Delegate = Callable[..., int]


def _json_load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"source report must contain a JSON object: {path}")
    return value


def _find_source_report(
    log_dir: Path,
    before: set[Path],
    started_ns: int,
    alternate_dir: Path | None = None,
) -> Path | None:
    search_dirs = [log_dir]
    if alternate_dir is not None and alternate_dir != log_dir:
        search_dirs.append(alternate_dir)
    candidates = [
        path
        for directory in search_dirs
        for path in directory.glob("j2a_readonly_*.json")
        if path not in before and path.stat().st_mtime_ns >= started_ns
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def _source_pass_conditions(source: dict[str, Any]) -> dict[str, bool]:
    write_audit = source.get("write_audit", {})
    per_motor = source.get("per_motor_stats", {})
    faults_zero = len(per_motor) == 6 and all(
        item.get("fault_set") == ["0x00"]
        for item in per_motor.values()
        if isinstance(item, dict)
    )
    source_conditions = source.get("pass_conditions", {})
    all_source_conditions = bool(source_conditions) and all(
        bool(value) for value in source_conditions.values()
    )
    return {
        "source_returned_pass": source.get("verdict") == "J2-A COMPLETE / PASS",
        "read_only_mode": source.get("hardware_mode") == "read_only",
        "duration_met": float(source.get("runtime_duration_s", 0.0))
        >= float(source.get("target_duration_s", MIN_PHASE1_DURATION_S)) - 1.0,
        "six_axis_faults_zero": faults_zero,
        "source_pass_conditions_all_true": all_source_conditions,
        "startup_writes_zero": int(write_audit.get("startup_writes", -1)) == 0,
        "runtime_writes_zero": int(write_audit.get("runtime_writes", -1)) == 0,
        "shutdown_writes_zero": int(write_audit.get("shutdown_writes", -1)) == 0,
        "total_writes_zero": int(write_audit.get("total_hardware_writes", -1)) == 0,
    }


def _write_outputs(report: dict[str, Any], output_prefix: Path) -> tuple[Path, Path]:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = output_prefix.with_suffix(".json")
    log_path = output_prefix.with_suffix(".log")
    if json_path.exists() or log_path.exists():
        raise FileExistsError(f"refusing to overwrite existing J6 report: {output_prefix}")
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    conditions = report["pass_conditions"]
    lines = [
        "J6 PHASE 1 READ_ONLY COMMISSIONING REPORT",
        f"Verdict: {'PASS' if report['pass'] else 'FAIL'}",
        f"Source report: {report.get('source_report_path')}",
        f"Hardware mode: {report.get('hardware_mode')}",
        f"Hardware writes: {report.get('hardware_write_count')}",
        "Pass conditions:",
    ]
    lines.extend(f"  {key}: {'PASS' if value else 'FAIL'}" for key, value in conditions.items())
    if report.get("failure_reasons"):
        lines.append("Failure reasons:")
        lines.extend(f"  - {reason}" for reason in report["failure_reasons"])
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, log_path


def run_phase1(
    *,
    duration_s: float = MIN_PHASE1_DURATION_S,
    port: str = DEFAULT_PORT,
    baudrate: int = 921600,
    source_log_dir: Path | None = None,
    output_prefix: Path | None = None,
    delegate: Delegate | None = None,
) -> dict[str, Any]:
    """Delegate one J2-A READ_ONLY run and create a J6 report.

    ``delegate`` is injectable for tests.  Production callers use the frozen
    ``run_j2a_verification`` function without changing that file.
    """

    if duration_s < MIN_PHASE1_DURATION_S:
        raise ValueError(f"duration_s must be >= {MIN_PHASE1_DURATION_S}")
    source_dir = source_log_dir or (WORKSPACE_ROOT / "log")
    source_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_prefix or (
        WORKSPACE_ROOT
        / "log"
        / f"j6_phase1_readonly_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    before = set(source_dir.glob("j2a_readonly_*.json"))
    started_ns = time.time_ns()
    delegate_result = 1
    delegate_error: str | None = None

    try:
        if delegate is None:
            from run_j2a_readonly_verification import (  # type: ignore
                run_j2a_verification,
            )

            delegate = run_j2a_verification
        delegate_result = int(
            delegate(duration_sec=duration_s, port=port, baudrate=baudrate)
        )
    except Exception as exc:
        delegate_error = str(exc)

    # The frozen J2-A implementation currently owns a fixed ``bone_ws/log``
    # output directory.  Keep ``source_log_dir`` injectable for tests while
    # also searching that production directory without modifying J2-A.
    source_path = _find_source_report(
        source_dir,
        before,
        started_ns,
        WORKSPACE_ROOT / "log",
    )
    source: dict[str, Any] = {}
    failures: list[str] = []
    if delegate_error is not None:
        failures.append(f"J2-A delegate failed: {delegate_error}")
    if source_path is None:
        failures.append("J2-A source JSON report was not generated")
    else:
        try:
            source = _json_load(source_path)
        except Exception as exc:
            failures.append(f"J2-A source report invalid: {exc}")

    source_conditions = _source_pass_conditions(source) if source else {
        "source_returned_pass": False,
        "read_only_mode": False,
        "duration_met": False,
        "six_axis_faults_zero": False,
        "source_pass_conditions_all_true": False,
        "startup_writes_zero": False,
        "runtime_writes_zero": False,
        "shutdown_writes_zero": False,
        "total_writes_zero": False,
    }
    failures.extend(key for key, value in source_conditions.items() if not value)
    write_audit = source.get("write_audit", {})
    hardware_writes = int(write_audit.get("total_hardware_writes", 0))
    report = {
        "schema_version": 1,
        "tool": "j6_phase1_readonly_wrapper",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "phase": "J6_PHASE1",
        "source_tool": "run_j2a_readonly_verification",
        "source_report_path": str(source_path) if source_path else None,
        "delegate_return_code": delegate_result,
        "hardware_mode": source.get("hardware_mode", "unknown"),
        "port": port,
        "baudrate": baudrate,
        "requested_duration_s": duration_s,
        "actual_duration_s": source.get("runtime_duration_s"),
        "hardware_motion_executed": False,
        "hardware_write_count": hardware_writes,
        "source_write_audit": write_audit,
        "source_pass_conditions": source.get("pass_conditions", {}),
        "pass_conditions": source_conditions,
        "failure_reasons": failures,
    }
    report["pass"] = not failures and delegate_result == 0 and all(source_conditions.values())
    _write_outputs(report, prefix)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J6 Phase 1 READ_ONLY wrapper")
    parser.add_argument(
        "--duration-s",
        "--duration",
        dest="duration_s",
        type=float,
        default=MIN_PHASE1_DURATION_S,
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=None,
        help="JSON/log prefix (default: bone_ws/log/j6_phase1_readonly_<timestamp>)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        report = run_phase1(
            duration_s=args.duration_s,
            port=args.port,
            baudrate=args.baudrate,
            output_prefix=args.output_prefix,
        )
    except (ValueError, FileExistsError) as exc:
        print(f"J6 Phase 1 wrapper rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"pass": report["pass"], "phase": report["phase"]}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
