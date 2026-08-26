#!/usr/bin/env python3
"""J6 Phase 2 axis-ID mapping and static-feedback commissioning tool.

The software can read six-axis status and compare an operator-recorded physical
ID order with the expected anchor order.  It cannot observe physical labels or
the mechanical bottom position; those are supplied as explicit evidence.
The CLI is read-only by construction.  Tests inject a FakeBus controller.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
if str(DRIVER_ROOT) not in sys.path:
    sys.path.insert(0, str(DRIVER_ROOT))

from pus6_motor_driver.safety import (  # noqa: E402
    DEFAULT_SERIAL_PORT,
    HardwareMode,
    MAX_CURRENT_MA,
    MOTOR_IDS,
)
from pus6_motor_driver.six_motor_controller import (  # noqa: E402
    SixAxisStateSnapshot,
    SixMotorController,
)


def _snapshot_dict(snapshot: SixAxisStateSnapshot) -> dict[str, Any]:
    return {
        "timestamp_start_ns": int(snapshot.timestamp_start_ns),
        "timestamp_end_ns": int(snapshot.timestamp_end_ns),
        "positions_steps": [int(value) for value in snapshot.positions_steps],
        "positions_meters": [float(value) for value in snapshot.positions_meters],
        "currents_mA": [int(value) for value in snapshot.currents_mA],
        "temperatures_C": [int(value) for value in snapshot.temperatures_C],
        "fault_codes": [int(value) for value in snapshot.fault_codes],
        "per_axis_timestamps_ns": {
            str(mid): int(stamp)
            for mid, stamp in snapshot.per_axis_timestamps_ns.items()
        },
    }


def _write_records(controller: SixMotorController) -> list[dict[str, Any]]:
    bus = controller.bus
    raw_audit = getattr(bus, "write_audit", None)
    if raw_audit is not None:
        return [dict(record) for record in raw_audit]
    return []


def _valid_id_order(values: Iterable[int]) -> bool:
    ordered = list(values)
    return len(ordered) == len(MOTOR_IDS) and sorted(ordered) == list(MOTOR_IDS)


def run_phase2_axis_mapping(
    controller: SixMotorController,
    *,
    observed_mapping: list[int],
    expected_mapping: list[int] | None = None,
    bottom_reference_steps: list[int] | None = None,
    zero_tolerance_steps: int = 2,
    sample_count: int = 1,
    sample_interval_s: float = 0.0,
) -> dict[str, Any]:
    """Collect static status and produce a Phase 2 report.

    ``observed_mapping`` is the operator's physical ID order for anchors
    A1..A6.  ``bottom_reference_steps`` is a measured q_bottom vector; no
    software component infers either value from geometry or feedback.
    """

    if expected_mapping is None:
        expected_mapping = list(MOTOR_IDS)
    if not _valid_id_order(observed_mapping):
        raise ValueError("observed_mapping must be a permutation of motor IDs 1..6")
    if not _valid_id_order(expected_mapping):
        raise ValueError("expected_mapping must be a permutation of motor IDs 1..6")
    if bottom_reference_steps is not None and len(bottom_reference_steps) != len(MOTOR_IDS):
        raise ValueError("bottom_reference_steps must contain six integer values")
    if bottom_reference_steps is not None and any(
        not isinstance(value, int) or isinstance(value, bool)
        for value in bottom_reference_steps
    ):
        raise ValueError("bottom_reference_steps must contain integers")
    if zero_tolerance_steps < 0 or sample_count < 1 or sample_interval_s < 0:
        raise ValueError("zero_tolerance_steps >= 0, sample_count >= 1, interval >= 0 required")

    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase2_axis_mapping",
        "phase": "J6_PHASE2",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "hardware_mode": controller.hardware_mode.value,
        "port": controller.port,
        "baudrate": controller.baudrate,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "mock_write_count": 0,
        "expected_mapping_a1_to_a6": list(expected_mapping),
        "observed_mapping_a1_to_a6": list(observed_mapping),
        "mapping_evidence_source": "operator_recorded_physical_labels",
        "bottom_reference_steps": list(bottom_reference_steps) if bottom_reference_steps is not None else None,
        "zero_tolerance_steps": zero_tolerance_steps,
        "snapshots": [],
        "pass_conditions": {},
        "failure_reasons": [],
    }

    if controller.hardware_mode == HardwareMode.ACTIVE:
        report["failure_reasons"].append("ACTIVE hardware mode is forbidden for Phase 2 read-only audit")
        report["outcome"] = "STOP"
        report["pass"] = False
        return report

    try:
        controller.connect()
        if controller.hardware_mode == HardwareMode.READ_ONLY:
            controller.read_limits()
        for index in range(sample_count):
            snapshot = controller.read_state_snapshot()
            report["snapshots"].append(_snapshot_dict(snapshot))
            if index + 1 < sample_count and sample_interval_s:
                time.sleep(sample_interval_s)
    except Exception as exc:
        report["failure_reasons"].append(f"static feedback read failed: {exc}")
    finally:
        report["hardware_write_count"] = int(controller.write_count)
        report["mock_write_count"] = int(controller.write_count) if controller.hardware_mode == HardwareMode.MOCK else 0
        report["write_audit"] = _write_records(controller)
        try:
            controller.close()
        except Exception as exc:
            report["failure_reasons"].append(f"close failure: {exc}")

    snapshots = report["snapshots"]
    feedback_complete = bool(snapshots) and all(
        len(item["positions_steps"]) == len(MOTOR_IDS)
        and len(item["fault_codes"]) == len(MOTOR_IDS)
        for item in snapshots
    )
    faults_zero = feedback_complete and all(
        all(code == 0 for code in item["fault_codes"]) for item in snapshots
    )
    currents_safe = feedback_complete and all(
        all(current <= MAX_CURRENT_MA for current in item["currents_mA"])
        for item in snapshots
    )
    mapping_verified = list(observed_mapping) == list(expected_mapping)
    if not mapping_verified:
        report["failure_reasons"].append("observed physical ID order does not match expected A1..A6 order")
    zero_verified = False
    if bottom_reference_steps is None:
        report["failure_reasons"].append("measured bottom_reference_steps were not supplied")
    elif snapshots:
        actual = snapshots[-1]["positions_steps"]
        errors = [abs(actual[index] - bottom_reference_steps[index]) for index in range(len(MOTOR_IDS))]
        report["bottom_reference_errors_steps"] = errors
        zero_verified = max(errors, default=0) <= zero_tolerance_steps
        if not zero_verified:
            report["failure_reasons"].append(
                f"static feedback differs from measured bottom reference by > {zero_tolerance_steps} steps"
            )

    zero_writes = report["hardware_write_count"] == 0 and not report["write_audit"]
    report["pass_conditions"] = {
        "read_only_or_mock_mode": controller.hardware_mode in (HardwareMode.READ_ONLY, HardwareMode.MOCK),
        "sample_count_met": len(snapshots) == sample_count,
        "six_axis_feedback_complete": feedback_complete,
        "all_faults_zero": faults_zero,
        "currents_within_threshold": currents_safe,
        "physical_mapping_matches_expected": mapping_verified,
        "bottom_reference_matches_feedback": zero_verified,
        "zero_hardware_writes": zero_writes,
    }
    report["pass"] = not report["failure_reasons"] and all(report["pass_conditions"].values())
    report["outcome"] = "PASS" if report["pass"] else "FAIL"
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def _parse_six_ints(value: str, name: str) -> list[int]:
    try:
        values = [int(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be six comma-separated integers") from exc
    if len(values) != len(MOTOR_IDS):
        raise argparse.ArgumentTypeError(f"{name} must contain six comma-separated integers")
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J6 Phase 2 axis mapping/static feedback audit")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--sample-interval-s", type=float, default=0.0)
    parser.add_argument("--observed-mapping", required=True, type=lambda value: _parse_six_ints(value, "observed-mapping"), help="physical ID order A1..A6")
    parser.add_argument("--expected-mapping", default="1,2,3,4,5,6", type=lambda value: _parse_six_ints(value, "expected-mapping"))
    parser.add_argument("--bottom-reference", required=True, type=lambda value: _parse_six_ints(value, "bottom-reference"), help="measured q_bottom ID1..ID6")
    parser.add_argument("--zero-tolerance-steps", type=int, default=2)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase2_axis_mapping_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    controller = SixMotorController(port=args.port, baudrate=args.baudrate, hardware_mode=HardwareMode.READ_ONLY)
    report = run_phase2_axis_mapping(
        controller,
        observed_mapping=args.observed_mapping,
        expected_mapping=args.expected_mapping,
        bottom_reference_steps=args.bottom_reference,
        zero_tolerance_steps=args.zero_tolerance_steps,
        sample_count=args.samples,
        sample_interval_s=args.sample_interval_s,
    )
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
