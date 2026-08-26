#!/usr/bin/env python3
"""Offline Phase 7 FK acceptance wrapper.

This tool deliberately reuses the frozen J5-C2 timed-feedback pipeline.  It
does not create a second synchronizer or FK solver, and it never opens a UART
or sends a motor command.  A successful report is therefore a *numerical*
FK/replay result only; physical mechanism acceptance remains out of scope.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_j5_c2_hw_acceptance import (  # noqa: E402
    C2HwAcceptanceRunner,
    TimedFeedbackFrame,
    _load_replay,
    _run_frames,
    _synthetic_frames,
)
from pus6_kinematics.geometry_config import load_geometry_config  # noqa: E402


TOOL_NAME = "j6_phase7_fk_acceptance"
PHASE_NAME = "J6_PHASE7"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _csv_value(value: Any) -> Any:
    if isinstance(value, (list, dict, tuple)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return value


class Phase7FkAcceptanceRunner:
    """Adapter that exposes the J5-C2 pipeline as a J6 Phase 7 report.

    ``C2HwAcceptanceRunner`` remains the sole owner of TimedJointFeedback
    ingestion, FixedLagJointSynchronizer use, and the FK implementation.  The
    adapter only maps its diagnostics into the J6 report contract and applies
    the Phase 7 numerical-vs-physical acceptance boundary.
    """

    def __init__(
        self,
        *,
        config_path: str | Path | None = None,
        fixed_lag_ms: float = 50.0,
        max_residual_m: float = 1e-4,
        max_pos_jump_m: float = 0.015,
        max_ang_jump_rad: float = 0.15,
        query_rate_hz: float = 200.0,
    ) -> None:
        config = load_geometry_config(config_path)
        self.c2_runner = C2HwAcceptanceRunner(
            config=config,
            fixed_lag_delay_ms=fixed_lag_ms,
            max_residual_m=max_residual_m,
            max_pos_jump_m=max_pos_jump_m,
            max_ang_jump_rad=max_ang_jump_rad,
        )
        self.fixed_lag_ms = float(fixed_lag_ms)
        self.query_rate_hz = float(query_rate_hz)
        self.input_frame_count = 0
        self.input_error: str | None = None

    def run_frames(self, frames: Iterable[TimedFeedbackFrame]) -> None:
        """Run an offline/replay frame sequence through the existing pipeline."""
        frame_list = list(frames)
        self.input_frame_count = len(frame_list)
        try:
            _run_frames(
                self.c2_runner,
                frame_list,
                query_rate_hz=self.query_rate_hz,
                fixed_lag_ms=self.fixed_lag_ms,
            )
        except (TypeError, ValueError, KeyError, IndexError, ArithmeticError) as exc:
            self.input_error = f"replay validation failed: {type(exc).__name__}: {exc}"

    def report(
        self,
        *,
        input_mode: str,
        source_path: str | Path | None = None,
        external_ground_truth: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Return a unified J6 report without asserting physical accuracy."""
        legacy = self.c2_runner.report(input_mode)
        summary = legacy["summary"]
        result_counts = summary["result_counts"]
        uart = summary["uart"]
        fk_ok_count = int(result_counts.get("FK_OK", 0))
        drop_count = int(result_counts.get("DROP", 0))
        source_exists = source_path is None or Path(source_path).is_file()

        pass_conditions: dict[str, bool] = {
            "input_frames_available": self.input_frame_count >= 2,
            "timed_feedback_pipeline_valid": self.input_error is None and self.input_frame_count >= 2,
            "fixed_lag_sync_observed": fk_ok_count > 0,
            "fk_convergence_and_residual": fk_ok_count > 0 and drop_count == 0,
            "branch_continuity": int(summary.get("branch_jump_count", 0)) == 0,
            "uart_sequence_integrity": (
                int(uart.get("duplicate_count", 0)) == 0
                and int(uart.get("gap_count", 0)) == 0
                and int(uart.get("out_of_order_count", 0)) == 0
            ),
            "timed_feedback_timestamps_valid": int(uart.get("timestamp_violation_count", 0)) == 0,
            "fault_free_feedback": int(uart.get("fault_frame_count", 0)) == 0,
            "source_replay_available": source_exists,
            "zero_hardware_writes": True,
        }
        failure_reasons = [name for name, passed in pass_conditions.items() if not passed]
        if self.input_error:
            failure_reasons.append(self.input_error)

        numerical_pass = not failure_reasons
        safety_stop = bool(
            summary.get("branch_jump_count", 0)
            or uart.get("duplicate_count", 0)
            or uart.get("gap_count", 0)
            or uart.get("out_of_order_count", 0)
            or uart.get("timestamp_violation_count", 0)
            or uart.get("fault_frame_count", 0)
        )
        source_value = str(source_path) if source_path is not None else None
        evidence: dict[str, Any] = {
            "input_mode": input_mode,
            "source_frame_count": self.input_frame_count,
            "source_report_path": source_value,
            "j5_c2_tool": "run_j5_c2_hw_acceptance.py",
            "j5_c2_summary": summary,
            "external_ground_truth": dict(external_ground_truth or {}),
        }
        if source_path is not None and Path(source_path).is_file():
            evidence["source_sha256"] = _sha256(Path(source_path))

        return {
            "schema_version": 1,
            "tool": TOOL_NAME,
            "phase": PHASE_NAME,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "session_id": None,
            "hardware_mode": input_mode if input_mode in {"offline", "mock"} else "offline",
            "port": None,
            "baudrate": None,
            "hardware_motion_executed": False,
            "hardware_write_count": 0,
            "pass": numerical_pass,
            "outcome": "STOP" if safety_stop else ("PASS" if numerical_pass else "FAIL"),
            "pass_conditions": pass_conditions,
            "failure_reasons": failure_reasons,
            "evidence": evidence,
            "observations": {
                "acceptance_scope": "numerical_fk_validation_only",
                "physical_accuracy": "not assessed by offline/mock input",
                "external_ground_truth_required_for_hardware_acceptance": True,
            },
            "metrics": {
                "frame_count": self.input_frame_count,
                "query_count": int(summary.get("query_count", 0)),
                "fk_ok_count": fk_ok_count,
                "fk_drop_count": drop_count,
                "wait_count": int(result_counts.get("WAIT", 0)),
                "fixed_lag_delay_ms": self.fixed_lag_ms,
                "query_rate_hz": self.query_rate_hz,
                "j5_c2_summary": summary,
            },
            "source_report_path": source_value,
            "source_reports": [source_value] if source_value is not None else [],
            "numerical_fk_validation_passed": numerical_pass,
            "hardware_acceptance_passed": False,
            "hardware_acceptance_status": "NOT_EVALUATED",
            "physical_accuracy_claimed": False,
            "records": list(legacy["records"]),
        }


def run_phase7_fk_acceptance(
    *,
    input_path: str | Path | None = None,
    config_path: str | Path | None = None,
    duration_s: float = 3.0,
    feedback_hz: float = 30.0,
    query_rate_hz: float = 200.0,
    fixed_lag_ms: float = 50.0,
    inject_branch_jump: bool = False,
    external_ground_truth: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Run a replay or deterministic synthetic sequence and return its report."""
    runner = Phase7FkAcceptanceRunner(
        config_path=config_path,
        fixed_lag_ms=fixed_lag_ms,
        query_rate_hz=query_rate_hz,
    )
    if input_path is not None:
        source = Path(input_path).expanduser()
        try:
            frames = _load_replay(source)
        except (OSError, ValueError, TypeError, KeyError, IndexError, json.JSONDecodeError) as exc:
            runner.input_error = f"unable to load replay: {type(exc).__name__}: {exc}"
            runner.input_frame_count = 0
        else:
            runner.run_frames(frames)
        return runner.report(
            input_mode="offline",
            source_path=source,
            external_ground_truth=external_ground_truth,
        )

    try:
        if duration_s <= 0.0 or feedback_hz <= 0.0 or query_rate_hz <= 0.0:
            raise ValueError("duration_s, feedback_hz, and query_rate_hz must be positive")
        frames = _synthetic_frames(
            runner.c2_runner,
            duration_s=duration_s,
            feedback_hz=feedback_hz,
            inject_jump=inject_branch_jump,
        )
        runner.run_frames(frames)
    except (TypeError, ValueError, ArithmeticError) as exc:
        runner.input_error = f"synthetic input validation failed: {type(exc).__name__}: {exc}"
    return runner.report(
        input_mode="mock",
        source_path=None,
        external_ground_truth=external_ground_truth,
    )


def write_phase7_reports(report: dict[str, Any], output: str | Path) -> tuple[Path, Path]:
    """Write JSON and CSV reports once; never overwrite an existing report."""
    json_path = Path(output).expanduser()
    csv_path = json_path.with_suffix(".csv")
    if json_path.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {json_path}")
    if csv_path.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {csv_path}")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    records = report.get("records", [])
    fields = sorted({key for record in records for key in record}) if records else ["query_timestamp_ns"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            writer.writerow({key: _csv_value(record.get(key)) for key in fields})
    return json_path, csv_path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline/mock J6 Phase 7 FK acceptance")
    parser.add_argument("--input", type=Path, help="TimedJointFeedback JSON array or JSONL replay")
    parser.add_argument("--config", type=Path, help="Geometry YAML used by the existing FK implementation")
    parser.add_argument("--output", type=Path, default=Path("/tmp/j6_phase7_fk_acceptance.json"))
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--feedback-hz", type=float, default=30.0)
    parser.add_argument("--query-rate-hz", type=float, default=200.0)
    parser.add_argument("--fixed-lag-ms", type=float, default=50.0)
    parser.add_argument("--inject-branch-jump", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    report = run_phase7_fk_acceptance(
        input_path=args.input,
        config_path=args.config,
        duration_s=args.duration_s,
        feedback_hz=args.feedback_hz,
        query_rate_hz=args.query_rate_hz,
        fixed_lag_ms=args.fixed_lag_ms,
        inject_branch_jump=args.inject_branch_jump,
    )
    json_path, csv_path = write_phase7_reports(report, args.output)
    print(json.dumps({
        "phase": report["phase"],
        "pass": report["pass"],
        "outcome": report["outcome"],
        "json": str(json_path),
        "csv": str(csv_path),
        "hardware_write_count": report["hardware_write_count"],
    }, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
