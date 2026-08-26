#!/usr/bin/env python3
"""J6 Phase 1 connection and static-health checker.

This tool is intentionally READ_ONLY-only.  It performs register/status reads
through the frozen ``SixMotorController`` API and writes a machine-readable
report.  It does not configure, command, or pause any motor.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


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
        "positions_steps": [int(v) for v in snapshot.positions_steps],
        "positions_meters": [float(v) for v in snapshot.positions_meters],
        "currents_mA": [int(v) for v in snapshot.currents_mA],
        "temperatures_C": [int(v) for v in snapshot.temperatures_C],
        "fault_codes": [int(v) for v in snapshot.fault_codes],
        "per_axis_timestamps_ns": {
            str(mid): int(stamp)
            for mid, stamp in snapshot.per_axis_timestamps_ns.items()
        },
    }


def _bus_write_records(controller: SixMotorController) -> list[dict[str, Any]]:
    bus = controller.bus
    raw_audit = getattr(bus, "write_audit", None)
    if raw_audit is not None:
        return [dict(record) for record in raw_audit]
    events = getattr(bus, "events", [])
    return [
        {
            "motor_id": int(event[0]),
            "register_address": int(event[2]),
            "payload": list(event[3]),
        }
        for event in events
        if len(event) >= 4 and event[1] == 0x32
    ]


def run_connection_check(
    controller: SixMotorController,
    *,
    sample_count: int = 1,
    sample_interval_s: float = 0.0,
) -> dict[str, Any]:
    """Run a static six-axis READ_ONLY check against an injected controller.

    The injected-controller form keeps all tests on FakeBus while the CLI
    constructs the real-port controller in READ_ONLY mode.
    """

    if sample_count < 1:
        raise ValueError("sample_count must be >= 1")
    if sample_interval_s < 0:
        raise ValueError("sample_interval_s must be >= 0")

    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_hardware_connection_checker",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "hardware_mode": controller.hardware_mode.value,
        "port": controller.port,
        "baudrate": controller.baudrate,
        "requested_sample_count": sample_count,
        "sample_interval_s": sample_interval_s,
        "connected": False,
        "limits": {},
        "snapshots": [],
        "pass_conditions": {},
        "failure_reasons": [],
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
    }

    if controller.hardware_mode != HardwareMode.READ_ONLY:
        report["failure_reasons"].append("controller is not in READ_ONLY mode")
        report["pass"] = False
        return report

    try:
        writes_before_connect = controller.write_count
        controller.connect()
        report["connected"] = controller.is_connected
        limits = controller.read_limits()
        report["limits"] = {
            str(mid): [int(pair[0]), int(pair[1])] for mid, pair in limits.items()
        }

        invalid_limits = [
            mid
            for mid in MOTOR_IDS
            if mid not in limits
            or len(limits[mid]) != 2
            or limits[mid][0] > limits[mid][1]
        ]
        if invalid_limits:
            report["failure_reasons"].append(
                f"invalid hardware limits for axes {invalid_limits}"
            )

        for index in range(sample_count):
            snapshot = controller.read_state_snapshot()
            report["snapshots"].append(_snapshot_dict(snapshot))
            if index + 1 < sample_count and sample_interval_s:
                time.sleep(sample_interval_s)

        if writes_before_connect != 0:
            report["failure_reasons"].append(
                f"controller had {writes_before_connect} writes before connect"
            )
    except Exception as exc:  # report failures without hiding cleanup
        report["failure_reasons"].append(f"connection/read failure: {exc}")
    finally:
        writes_after_reads = controller.write_count
        report["hardware_write_count"] = int(writes_after_reads)
        report["write_audit"] = _bus_write_records(controller)
        try:
            controller.close()
        except Exception as exc:
            report["failure_reasons"].append(f"close failure: {exc}")

    snapshots = report["snapshots"]
    all_faults_zero = bool(snapshots) and all(
        all(int(code) == 0 for code in snap["fault_codes"]) for snap in snapshots
    )
    currents_safe = bool(snapshots) and all(
        all(int(current) <= MAX_CURRENT_MA for current in snap["currents_mA"])
        for snap in snapshots
    )
    complete_axes = bool(snapshots) and all(
        len(snap["positions_steps"]) == len(MOTOR_IDS) for snap in snapshots
    )
    writes_zero = report["hardware_write_count"] == 0 and not report["write_audit"]
    report["pass_conditions"] = {
        "read_only_mode": controller.hardware_mode == HardwareMode.READ_ONLY,
        "connected": report["connected"],
        "sample_count_met": len(snapshots) == sample_count,
        "six_axis_feedback_complete": complete_axes,
        "all_faults_zero": all_faults_zero,
        "currents_within_threshold": currents_safe,
        "zero_hardware_writes": writes_zero,
    }
    report["pass"] = not report["failure_reasons"] and all(
        report["pass_conditions"].values()
    )
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J6 READ_ONLY hardware connection checker")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--sample-interval-s", type=float, default=0.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="JSON report path (default: bone_ws/log/j6_connection_<timestamp>.json)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.samples < 1 or args.sample_interval_s < 0 or args.retries < 0:
        raise SystemExit("samples, sample interval, and retries must be non-negative; samples >= 1")
    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = WORKSPACE_ROOT / "log" / f"j6_connection_{stamp}.json"

    # The CLI has no ACTIVE mode by construction.  The only controller created
    # here is the frozen driver's READ_ONLY transport.
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        retries=args.retries,
        hardware_mode=HardwareMode.READ_ONLY,
    )
    report = run_connection_check(
        controller,
        sample_count=args.samples,
        sample_interval_s=args.sample_interval_s,
    )
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
