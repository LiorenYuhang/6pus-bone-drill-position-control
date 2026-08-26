#!/usr/bin/env python3
"""J6 Phase 3 single-axis micro-jog commissioning tool.

The tool is Mock by default.  An ACTIVE run requires all explicit motion
authorization flags and is limited to one selected axis, one 20--40 step
signed jog, and a return to the original baseline.
"""

from __future__ import annotations

import argparse
import json
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
    DEFAULT_SPEED,
    HardwareMode,
    MAX_CURRENT_MA,
    MOTOR_IDS,
    SOFTWARE_LOWER_LIMIT,
    SOFTWARE_UPPER_LIMIT,
    SafetyViolation,
    TargetValidationError,
    validate_status_feedback,
)
from pus6_motor_driver.six_motor_controller import (  # noqa: E402
    SixAxisStateSnapshot,
    SixMotorController,
)


MIN_MICRO_JOG_STEPS = 20
MAX_MICRO_JOG_STEPS = 40
RETURN_TOLERANCE_STEPS = 1
ARRIVAL_SAMPLES = 2


def _snapshot_dict(snapshot: SixAxisStateSnapshot) -> dict[str, Any]:
    return {
        "timestamp_end_ns": int(snapshot.timestamp_end_ns),
        "positions_steps": [int(value) for value in snapshot.positions_steps],
        "currents_mA": [int(value) for value in snapshot.currents_mA],
        "temperatures_C": [int(value) for value in snapshot.temperatures_C],
        "fault_codes": [int(value) for value in snapshot.fault_codes],
    }


def _write_records(controller: SixMotorController) -> list[dict[str, Any]]:
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


def _write_index(controller: SixMotorController) -> int:
    return len(_write_records(controller))


def _axis_write_ids(records: list[dict[str, Any]], start: int) -> set[int]:
    return {
        int(record["motor_id"])
        for record in records[start:]
        if int(record["register_address"]) == 0x1A
    }


def _validate_static(snapshot: SixAxisStateSnapshot) -> None:
    for mid, status in zip(MOTOR_IDS, snapshot.states):
        if status.error_code != 0:
            raise SafetyViolation(f"ID{mid} fault is 0x{status.error_code:02X}")
        if status.current_mA != 0:
            raise SafetyViolation(f"ID{mid} current is {status.current_mA}mA, expected 0mA")
        validate_status_feedback(status, mid, None)


def _validate_runtime(snapshot: SixAxisStateSnapshot) -> None:
    for mid, status in zip(MOTOR_IDS, snapshot.states):
        if status.error_code != 0:
            raise SafetyViolation(f"ID{mid} fault is 0x{status.error_code:02X}")
        if status.current_mA > MAX_CURRENT_MA:
            raise SafetyViolation(
                f"ID{mid} current {status.current_mA}mA exceeds {MAX_CURRENT_MA}mA"
            )


def _monotonic_toward(values: list[int], start: int, target: int) -> bool:
    if not values:
        return False
    if target == start:
        return all(value == start for value in values)
    if target > start:
        return all(current >= previous for previous, current in zip(values, values[1:]))
    return all(current <= previous for previous, current in zip(values, values[1:]))


def _pause_all_after_failure(
    controller: SixMotorController,
    *,
    write_start: int,
) -> dict[str, Any]:
    """Pause every axis once, avoiding a duplicate target-axis pause.

    ``command_single_axis_with_metrics`` already attempts a target-axis pause
    when its target write raises.  The audit is checked so the remaining axes
    receive exactly one pause without retrying the failed motion command.
    """

    records_before = _write_records(controller)
    already_paused = _axis_write_ids(records_before, write_start)
    pause_results: dict[str, Any] = {}
    for mid in MOTOR_IDS:
        if mid in already_paused:
            pause_results[str(mid)] = {"attempted": False, "already_paused": True}
            continue
        try:
            status = controller.pause_single_axis(mid)
            pause_results[str(mid)] = {
                "attempted": True,
                "success": True,
                "current_mA": int(status.current_mA),
                "fault": int(status.error_code),
            }
        except Exception as exc:
            pause_results[str(mid)] = {
                "attempted": True,
                "success": False,
                "error": str(exc),
            }
    try:
        stopped = controller.read_state_snapshot()
        stopped_ok = all(
            status.current_mA == 0 and status.error_code == 0
            for status in stopped.states
        )
    except Exception as exc:
        stopped_ok = False
        pause_results["confirmation_error"] = str(exc)
    pause_results["confirmed"] = stopped_ok
    pause_results["hard_stop_required"] = not stopped_ok
    return pause_results


def _poll_leg(
    controller: SixMotorController,
    *,
    axis: int,
    start_position: int,
    target_position: int,
    timeout_s: float,
    poll_period_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    samples: list[dict[str, Any]] = []
    axis_positions: list[int] = []
    consecutive_arrivals = 0
    while time.monotonic() < deadline:
        snapshot = controller.read_state_snapshot()
        _validate_runtime(snapshot)
        status = snapshot.states[axis - 1]
        axis_positions.append(int(status.current_position))
        samples.append(_snapshot_dict(snapshot))
        if abs(status.current_position - target_position) <= RETURN_TOLERANCE_STEPS:
            consecutive_arrivals += 1
            if consecutive_arrivals >= ARRIVAL_SAMPLES:
                return {
                    "target_position": target_position,
                    "samples": samples,
                    "axis_positions": axis_positions,
                    "arrived": True,
                    "final_error_steps": int(status.current_position - target_position),
                    "monotonic_toward_target": _monotonic_toward(
                        axis_positions, start_position, target_position
                    ),
                }
        else:
            consecutive_arrivals = 0
        time.sleep(poll_period_s)
    return {
        "target_position": target_position,
        "samples": samples,
        "axis_positions": axis_positions,
        "arrived": False,
        "final_error_steps": (
            int(axis_positions[-1] - target_position) if axis_positions else None
        ),
        "monotonic_toward_target": _monotonic_toward(
            axis_positions, start_position, target_position
        ),
    }


def _pause_and_confirm_selected(
    controller: SixMotorController,
    *,
    axis: int,
    baseline_target: int,
    poll_period_s: float,
) -> dict[str, Any]:
    status = controller.pause_single_axis(axis)
    confirmations: list[dict[str, Any]] = []
    previous_position: int | None = None
    stable = False
    for _ in range(ARRIVAL_SAMPLES):
        snapshot = controller.read_state_snapshot()
        _validate_static(snapshot)
        selected = snapshot.states[axis - 1]
        confirmations.append(_snapshot_dict(snapshot))
        if previous_position is not None:
            stable = stable or selected.current_position == previous_position
        previous_position = selected.current_position
        if len(confirmations) < ARRIVAL_SAMPLES:
            time.sleep(poll_period_s)
    selected_final = confirmations[-1]["positions_steps"][axis - 1]
    confirmed = (
        stable
        and abs(selected_final - baseline_target) <= RETURN_TOLERANCE_STEPS
        and all(
            current == 0 and fault == 0
            for current, fault in zip(
                confirmations[-1]["currents_mA"], confirmations[-1]["fault_codes"]
            )
        )
    )
    return {
        "axis": axis,
        "pause_status": {
            "current_mA": int(status.current_mA),
            "fault": int(status.error_code),
        },
        "confirmations": confirmations,
        "confirmed": confirmed,
        "hard_stop_required": not confirmed,
    }


def run_micro_jog(
    controller: SixMotorController,
    *,
    axis: int,
    displacement_steps: int,
    cycles: int = 1,
    speed: int = DEFAULT_SPEED,
    timeout_s: float = 3.0,
    poll_period_s: float = 0.05,
    authorized: bool = False,
    physical_direction_observed: str = "unrecorded",
    clearance_observed: str = "unrecorded",
) -> dict[str, Any]:
    """Execute one-axis out-and-back micro-jogs using existing APIs."""

    if axis not in MOTOR_IDS:
        raise TargetValidationError(f"axis must be one of {MOTOR_IDS}")
    if not isinstance(displacement_steps, int) or isinstance(displacement_steps, bool):
        raise TargetValidationError("displacement_steps must be an integer")
    if not MIN_MICRO_JOG_STEPS <= abs(displacement_steps) <= MAX_MICRO_JOG_STEPS:
        raise TargetValidationError(
            f"absolute displacement must be {MIN_MICRO_JOG_STEPS}..{MAX_MICRO_JOG_STEPS} steps"
        )
    if speed != DEFAULT_SPEED:
        raise TargetValidationError(
            f"first J6 micro-jog implementation requires speed={DEFAULT_SPEED} step/s"
        )
    if cycles < 1 or timeout_s <= 0 or poll_period_s <= 0:
        raise ValueError("cycles, timeout_s, and poll_period_s must be positive")
    if controller.hardware_mode == HardwareMode.READ_ONLY:
        raise TargetValidationError("micro-jog requires MOCK or explicitly authorized ACTIVE mode")
    if controller.hardware_mode == HardwareMode.ACTIVE and not authorized:
        raise TargetValidationError(
            "ACTIVE micro-jog requires real-hardware, confirm-motion, validated-config, "
            "and confirm-micro-jog authorization"
        )

    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase3_single_axis_micro_jog",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "phase": "J6_PHASE3",
        "hardware_mode": controller.hardware_mode.value,
        "axis": axis,
        "displacement_steps": displacement_steps,
        "speed_step_s": speed,
        "cycles_requested": cycles,
        "timeout_s": timeout_s,
        "poll_period_s": poll_period_s,
        "physical_direction_observed": physical_direction_observed,
        "clearance_observed": clearance_observed,
        "baseline_positions_steps": None,
        "legs": [],
        "failure_reasons": [],
        "outcome": "NOT_STARTED",
        "hardware_motion_executed": controller.hardware_mode == HardwareMode.ACTIVE,
    }
    failure_write_start = 0
    try:
        controller.connect()
        limits = controller.read_limits()
        preflight = controller.read_state_snapshot()
        _validate_static(preflight)
        report["baseline_positions_steps"] = list(preflight.positions_steps)
        lower = max(int(limits[axis][0]), SOFTWARE_LOWER_LIMIT)
        upper = min(int(limits[axis][1]), SOFTWARE_UPPER_LIMIT)
        baseline = int(preflight.positions_steps[axis - 1])
        target = baseline + displacement_steps
        if not lower <= target <= upper:
            raise TargetValidationError(
                f"axis {axis} target {target} outside safe bounds [{lower}, {upper}]"
            )

        # Verify the selected axis before the one validated 0x25 setup write.
        preflight_config = controller.read_single_axis_preflight(axis)
        report["preflight_configuration"] = dict(preflight_config)
        failure_write_start = _write_index(controller)
        configuration = controller.configure_single_axis_for_bringup(
            axis,
            baseline,
            speed=speed,
            baseline_band=(baseline, baseline),
            target_band=(baseline, baseline),
        )
        report["configuration"] = dict(configuration)

        for cycle in range(1, cycles + 1):
            cycle_report: dict[str, Any] = {
                "cycle": cycle,
                "forward": None,
                "return": None,
                "pass": False,
            }
            forward_start = _write_index(controller)
            failure_write_start = forward_start
            command = controller.command_single_axis_with_metrics(axis, target)
            forward = _poll_leg(
                controller,
                axis=axis,
                start_position=baseline,
                target_position=target,
                timeout_s=timeout_s,
                poll_period_s=poll_period_s,
            )
            forward["command_response_position"] = int(command.response.current_position)
            forward["pause_confirmation"] = _pause_and_confirm_selected(
                controller,
                axis=axis,
                baseline_target=target,
                poll_period_s=poll_period_s,
            )
            forward["write_records"] = _write_records(controller)[forward_start:]
            cycle_report["forward"] = forward
            if not (
                forward["arrived"]
                and forward["monotonic_toward_target"]
                and forward["pause_confirmation"]["confirmed"]
            ):
                raise SafetyViolation("forward micro-jog did not meet arrival/stop gates")

            return_start = _write_index(controller)
            failure_write_start = return_start
            return_command = controller.command_single_axis_with_metrics(axis, baseline)
            returned = _poll_leg(
                controller,
                axis=axis,
                start_position=target,
                target_position=baseline,
                timeout_s=timeout_s,
                poll_period_s=poll_period_s,
            )
            returned["command_response_position"] = int(return_command.response.current_position)
            returned["pause_confirmation"] = _pause_and_confirm_selected(
                controller,
                axis=axis,
                baseline_target=baseline,
                poll_period_s=poll_period_s,
            )
            returned["write_records"] = _write_records(controller)[return_start:]
            cycle_report["return"] = returned
            cycle_report["pass"] = (
                returned["arrived"]
                and returned["monotonic_toward_target"]
                and abs(int(returned["final_error_steps"])) <= RETURN_TOLERANCE_STEPS
                and returned["pause_confirmation"]["confirmed"]
            )
            if not cycle_report["pass"]:
                raise SafetyViolation("return micro-jog did not meet arrival/stop gates")
            report["legs"].append(cycle_report)

        report["outcome"] = "PASS"
    except Exception as exc:
        report["failure_reasons"].append(str(exc))
        report["outcome"] = "STOP" if report["legs"] else "FAIL"
        if controller.write_count > 0 or _write_records(controller):
            try:
                report["failure_stop"] = _pause_all_after_failure(
                    controller,
                    write_start=failure_write_start,
                )
            except Exception as stop_exc:
                report["failure_stop"] = {
                    "confirmed": False,
                    "hard_stop_required": True,
                    "error": str(stop_exc),
                }
    finally:
        report["write_audit"] = _write_records(controller)
        report["controller_write_count"] = int(controller.write_count)
        report["hardware_write_count"] = (
            int(controller.write_count)
            if controller.hardware_mode == HardwareMode.ACTIVE
            else 0
        )
        report["mock_write_count"] = (
            int(controller.write_count)
            if controller.hardware_mode == HardwareMode.MOCK
            else 0
        )
        report["per_axis_write_counts"] = {
            str(mid): sum(
                int(record["motor_id"]) == mid for record in report["write_audit"]
            )
            for mid in MOTOR_IDS
        }
        report["observer_zero_writes"] = all(
            report["per_axis_write_counts"][str(mid)] == 0
            for mid in MOTOR_IDS
            if mid != axis
        )
        report["software_pass"] = report["outcome"] == "PASS" and report["observer_zero_writes"]
        try:
            controller.close()
        except Exception as exc:
            report["failure_reasons"].append(f"close failure: {exc}")
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J6 Phase 3 single-axis micro-jog")
    parser.add_argument("--axis", type=int, required=True, choices=MOTOR_IDS)
    parser.add_argument(
        "--displacement-steps",
        "--displacement",
        dest="displacement_steps",
        type=int,
        required=True,
    )
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--poll-period-s", type=float, default=0.05)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--confirm-micro-jog", action="store_true")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument(
        "--physical-direction",
        choices=("up", "down", "unknown"),
        default="unknown",
        help="Operator annotation; software cannot infer physical +Z direction",
    )
    parser.add_argument(
        "--clearance-observed",
        choices=("pass", "fail", "unknown"),
        default="unknown",
        help="Operator annotation for mechanical clearance",
    )
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.real_hardware and not (
        args.confirm_motion and args.validated_config and args.confirm_micro_jog
    ):
        print(
            "real hardware requires --confirm-motion, --validated-config, and "
            "--confirm-micro-jog",
            file=sys.stderr,
        )
        return 2
    if not args.real_hardware and (args.confirm_motion or args.validated_config or args.confirm_micro_jog):
        print("motion authorization flags require --real-hardware", file=sys.stderr)
        return 2
    mode = HardwareMode.ACTIVE if args.real_hardware else HardwareMode.MOCK
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        retries=0,
        hardware_mode=mode,
    )
    try:
        report = run_micro_jog(
            controller,
            axis=args.axis,
            displacement_steps=args.displacement_steps,
            cycles=args.cycles,
            speed=args.speed,
            timeout_s=args.timeout_s,
            poll_period_s=args.poll_period_s,
            authorized=(
                args.confirm_motion and args.validated_config and args.confirm_micro_jog
            ),
            physical_direction_observed=args.physical_direction,
            clearance_observed=args.clearance_observed,
        )
        output = args.output
        if output is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = WORKSPACE_ROOT / "log" / f"j6_phase3_micro_jog_{stamp}.json"
        _write_report(report, output)
    except (TargetValidationError, ValueError, FileExistsError) as exc:
        print(f"J6 Phase 3 rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"outcome": report["outcome"], "report": str(output)}, indent=2))
    return 0 if report["software_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
