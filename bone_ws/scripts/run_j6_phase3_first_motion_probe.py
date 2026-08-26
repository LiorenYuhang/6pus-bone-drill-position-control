#!/usr/bin/env python3
"""J6 Phase 3 first closed-chain motion probe.

This is an observation state machine, not a normal jog.  It records a static
baseline, performs exactly one bounded target perturbation, holds for explicit
observation samples, and returns only when return authorization is supplied.
The script does not issue HOME, PARK, Cartesian, or multi-axis commands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Mapping


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
if str(DRIVER_ROOT) not in sys.path:
    sys.path.insert(0, str(DRIVER_ROOT))

from pus6_motor_driver.safety import (  # noqa: E402
    DEFAULT_SERIAL_PORT,
    HardwareMode,
    MAX_CURRENT_MA,
    MOTOR_IDS,
    PAUSE_ADDR,
    SOFTWARE_LOWER_LIMIT,
    SOFTWARE_UPPER_LIMIT,
    TARGET_POSITION_ADDR,
    SafetyViolation,
    TargetValidationError,
)
from pus6_motor_driver.six_motor_controller import (  # noqa: E402
    SixAxisStateSnapshot,
    SixMotorController,
)


ALLOWED_DISPLACEMENTS: tuple[int, ...] = (2, 5, 10)
DEFAULT_DISPLACEMENT_STEPS = 2
RETURN_TOLERANCE_STEPS = 1
DEFAULT_HOLD_SAMPLES = 2
DEFAULT_HOLD_INTERVAL_S = 0.05

OBSERVATION_DEFAULTS: dict[str, str] = {
    "platform_motion": "unrecorded",
    "ball_joint_condition": "unrecorded",
    "interference": "unrecorded",
    "abnormal_sound": "unrecorded",
}
OBSERVATION_VALUES: dict[str, set[str]] = {
    "platform_motion": {"unrecorded", "expected", "unexpected"},
    "ball_joint_condition": {"unrecorded", "clear", "abnormal"},
    "interference": {"unrecorded", "none", "present"},
    "abnormal_sound": {"unrecorded", "none", "present"},
}


class ProbeState(str, Enum):
    CREATED = "CREATED"
    BASELINE_RECORDED = "BASELINE_RECORDED"
    FORWARD_COMPLETE = "FORWARD_COMPLETE"
    OBSERVATION_HOLD = "OBSERVATION_HOLD"
    RETURN_COMPLETE = "RETURN_COMPLETE"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


def _normalise_observations(observations: Mapping[str, str] | None) -> dict[str, str]:
    values = dict(OBSERVATION_DEFAULTS)
    if observations is not None:
        for key, value in observations.items():
            if key not in OBSERVATION_VALUES:
                raise TargetValidationError(f"unknown operator observation field: {key}")
            if not isinstance(value, str):
                raise TargetValidationError(f"operator observation {key} must be a string")
            if value not in OBSERVATION_VALUES[key]:
                allowed = ", ".join(sorted(OBSERVATION_VALUES[key]))
                raise TargetValidationError(
                    f"operator observation {key} must be one of: {allowed}"
                )
            values[key] = value
    return values


def _observations_complete(observations: Mapping[str, str]) -> bool:
    return all(value != "unrecorded" for value in observations.values())


def _observations_safe(observations: Mapping[str, str]) -> bool:
    return (
        observations.get("platform_motion") == "expected"
        and observations.get("ball_joint_condition") == "clear"
        and observations.get("interference") == "none"
        and observations.get("abnormal_sound") == "none"
    )


def _snapshot_dict(snapshot: SixAxisStateSnapshot) -> dict[str, Any]:
    """Serialize the complete ordered six-axis feedback snapshot."""

    return {
        "timestamp_start_ns": int(snapshot.timestamp_start_ns),
        "timestamp_end_ns": int(snapshot.timestamp_end_ns),
        "per_axis_timestamps_ns": {
            str(mid): int(stamp)
            for mid, stamp in snapshot.per_axis_timestamps_ns.items()
        },
        "target_positions_steps": [
            int(status.target_position) for status in snapshot.states
        ],
        "actual_positions_steps": [int(value) for value in snapshot.positions_steps],
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
            "command": int(event[1]),
            "register_address": int(event[2]),
            "payload": list(event[3]),
        }
        for event in events
        if len(event) >= 4 and event[1] == 0x32
    ]


def _validate_snapshot_health(snapshot: SixAxisStateSnapshot, *, phase: str) -> list[str]:
    failures: list[str] = []
    if len(snapshot.states) != len(MOTOR_IDS):
        failures.append(f"{phase}: six-axis status count is {len(snapshot.states)}")
        return failures
    for index, status in enumerate(snapshot.states, start=1):
        if status.error_code != 0:
            failures.append(f"{phase}: ID{index} fault=0x{status.error_code:02X}")
        if status.current_mA > MAX_CURRENT_MA:
            failures.append(
                f"{phase}: ID{index} current={status.current_mA}mA exceeds {MAX_CURRENT_MA}mA"
            )
    return failures


class FirstMotionProbeSession:
    """Explicit baseline -> forward -> hold -> authorized return state machine."""

    def __init__(
        self,
        controller: SixMotorController,
        *,
        axis: int,
        displacement_steps: int = DEFAULT_DISPLACEMENT_STEPS,
        hold_samples: int = DEFAULT_HOLD_SAMPLES,
        hold_interval_s: float = DEFAULT_HOLD_INTERVAL_S,
        observations: Mapping[str, str] | None = None,
        authorized: bool = False,
        session_id: str | None = None,
    ) -> None:
        if axis not in MOTOR_IDS:
            raise TargetValidationError(f"axis must be one of {MOTOR_IDS}")
        if not isinstance(displacement_steps, int) or isinstance(displacement_steps, bool):
            raise TargetValidationError("displacement_steps must be an integer")
        if abs(displacement_steps) not in ALLOWED_DISPLACEMENTS:
            raise TargetValidationError(
                f"absolute displacement must be one of {ALLOWED_DISPLACEMENTS} steps"
            )
        if hold_samples < 1 or hold_interval_s < 0:
            raise ValueError("hold_samples must be >= 1 and hold_interval_s must be >= 0")
        if controller.hardware_mode == HardwareMode.READ_ONLY:
            raise TargetValidationError(
                "first-motion probe requires MOCK or explicitly authorized ACTIVE mode"
            )
        if controller.hardware_mode == HardwareMode.ACTIVE and not authorized:
            raise TargetValidationError(
                "ACTIVE first-motion probe requires explicit motion authorization"
            )
        if getattr(controller, "retries", 0) != 0:
            raise TargetValidationError(
                "first-motion probe requires controller retries=0; motion retries are forbidden"
            )

        self.controller = controller
        self.axis = axis
        self.displacement_steps = displacement_steps
        self.hold_samples = hold_samples
        self.hold_interval_s = hold_interval_s
        self.observations = _normalise_observations(observations)
        self.state = ProbeState.CREATED
        self.baseline_snapshot: SixAxisStateSnapshot | None = None
        self.forward_target: int | None = None
        self.limits: dict[int, tuple[int, int]] = {}
        self.failures: list[str] = []
        self._return_authorized = False
        self._return_executed = False
        self._expected_target_writes = 0
        self._snapshots_logged = 0
        self.session_id = session_id
        self.report: dict[str, Any] = {
            "schema_version": 1,
            "tool": "j6_phase3_first_motion_probe",
            "phase": "J6_PHASE3",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "session_id": session_id,
            "hardware_mode": controller.hardware_mode.value,
            "port": controller.port,
            "baudrate": controller.baudrate,
            "axis": axis,
            "displacement_steps": displacement_steps,
            "displacement_mm": float(displacement_steps) * 0.025,
            "allowed_displacements_steps": list(ALLOWED_DISPLACEMENTS),
            "hold_samples_requested": hold_samples,
            "hold_interval_s": hold_interval_s,
            "hardware_motion_executed": False,
            "hardware_write_count": 0,
            "mock_write_count": 0,
            "state": self.state.value,
            "stages": {
                "baseline_recording": None,
                "forward_perturbation": None,
                "observation_hold": None,
                "return_authorization": {
                    "authorized": False,
                    "executed": False,
                    "feedback": None,
                },
            },
            "operator_observations": dict(self.observations),
            "pass_conditions": {},
            "failure_reasons": self.failures,
            "evidence": {
                "motion_policy": "J6_PHASE3_FIRST_MOTION_SAFETY_REVIEW_CN.md",
                "home_executed": False,
                "park_executed": False,
                "cartesian_motion_executed": False,
                "automatic_retry": False,
            },
        }

    def _set_state(self, state: ProbeState) -> None:
        self.state = state
        self.report["state"] = state.value

    def _record_failure(self, message: str, *, stopped: bool = False) -> None:
        self.failures.append(message)
        self._set_state(ProbeState.STOPPED if stopped else ProbeState.FAILED)

    def _safe_target_bounds(self) -> tuple[int, int]:
        lower, upper = self.limits[self.axis]
        return max(int(lower), SOFTWARE_LOWER_LIMIT), min(
            int(upper), SOFTWARE_UPPER_LIMIT
        )

    def record_baseline(self) -> dict[str, Any]:
        """Record six-axis feedback and limits without any write."""

        if self.state != ProbeState.CREATED:
            raise SafetyViolation("baseline recording must be the first probe stage")
        self.controller.connect()
        snapshot = self.controller.read_state_snapshot()
        self.limits = self.controller.read_limits()
        health_failures = _validate_snapshot_health(snapshot, phase="baseline")
        positions = list(snapshot.positions_steps)
        envelope_failures = [
            f"baseline: ID{mid} actual={positions[mid - 1]} outside software envelope "
            f"[{SOFTWARE_LOWER_LIMIT},{SOFTWARE_UPPER_LIMIT}]"
            for mid in MOTOR_IDS
            if not SOFTWARE_LOWER_LIMIT <= positions[mid - 1] <= SOFTWARE_UPPER_LIMIT
        ]
        self.failures.extend(health_failures + envelope_failures)
        self.baseline_snapshot = snapshot
        self.report["stages"]["baseline_recording"] = {
            "feedback": _snapshot_dict(snapshot),
            "hardware_limits_steps": {
                str(mid): [int(lower), int(upper)]
                for mid, (lower, upper) in self.limits.items()
            },
            "health_pass": not health_failures,
            "software_envelope_pass": not envelope_failures,
            "write_count_at_stage": int(self.controller.write_count),
        }
        if health_failures or envelope_failures:
            self._set_state(ProbeState.FAILED)
            raise SafetyViolation("baseline safety gate failed")
        self._set_state(ProbeState.BASELINE_RECORDED)
        self._snapshots_logged += 1
        return self.report["stages"]["baseline_recording"]

    def forward_perturbation(self) -> dict[str, Any]:
        """Issue exactly one selected-axis target write after baseline recording."""

        if self.state != ProbeState.BASELINE_RECORDED or self.baseline_snapshot is None:
            raise SafetyViolation("forward perturbation requires a recorded baseline")
        baseline = int(self.baseline_snapshot.positions_steps[self.axis - 1])
        target = baseline + self.displacement_steps
        lower, upper = self._safe_target_bounds()
        if not lower <= target <= upper:
            self._record_failure(
                f"forward target {target} outside safe bounds [{lower},{upper}]"
            )
            raise TargetValidationError(self.failures[-1])
        self.forward_target = target
        self._expected_target_writes = 1
        command = self.controller.command_single_axis_with_metrics(self.axis, target)
        post_command = self.controller.read_state_snapshot()
        health_failures = _validate_snapshot_health(post_command, phase="forward")
        self.failures.extend(health_failures)
        self.report["stages"]["forward_perturbation"] = {
            "motor_id": self.axis,
            "baseline_position_steps": baseline,
            "target_position_steps": target,
            "commanded_target_steps": int(command.commanded_target),
            "command_timestamp": float(command.timestamp_write),
            "feedback": _snapshot_dict(post_command),
            "health_pass": not health_failures,
            "write_count_at_stage": int(self.controller.write_count),
        }
        self._snapshots_logged += 1
        if health_failures:
            self._record_failure("forward feedback safety gate failed", stopped=True)
            raise SafetyViolation(self.failures[-1])
        self._set_state(ProbeState.FORWARD_COMPLETE)
        return self.report["stages"]["forward_perturbation"]

    def observation_hold(
        self,
        observations: Mapping[str, str] | None = None,
    ) -> dict[str, Any]:
        """Collect read-only hold samples; this stage never returns automatically."""

        if self.state != ProbeState.FORWARD_COMPLETE:
            raise SafetyViolation("observation hold requires a completed forward perturbation")
        if observations is not None:
            self.observations = _normalise_observations(observations)
            self.report["operator_observations"] = dict(self.observations)
        samples: list[dict[str, Any]] = []
        hold_failures: list[str] = []
        for index in range(self.hold_samples):
            snapshot = self.controller.read_state_snapshot()
            samples.append(_snapshot_dict(snapshot))
            self._snapshots_logged += 1
            hold_failures.extend(
                _validate_snapshot_health(snapshot, phase=f"hold[{index}]")
            )
            if index + 1 < self.hold_samples and self.hold_interval_s:
                time.sleep(self.hold_interval_s)
        self.failures.extend(hold_failures)
        observations_complete = _observations_complete(self.observations)
        observations_safe = _observations_safe(self.observations)
        self.report["stages"]["observation_hold"] = {
            "samples": samples,
            "sample_count": len(samples),
            "health_pass": not hold_failures,
            "operator_observations_complete": observations_complete,
            "operator_observations_safe": observations_safe,
            "automatic_return": False,
            "write_count_at_stage": int(self.controller.write_count),
        }
        if hold_failures or (observations_complete and not observations_safe):
            self._record_failure("observation hold safety gate failed", stopped=True)
            raise SafetyViolation(self.failures[-1])
        self._set_state(ProbeState.OBSERVATION_HOLD)
        return self.report["stages"]["observation_hold"]

    def authorize_return(self) -> dict[str, Any]:
        """Return once after an explicit post-hold authorization."""

        if self.state != ProbeState.OBSERVATION_HOLD:
            raise SafetyViolation(
                "return authorization requires completed observation hold; no automatic return"
            )
        if not _observations_complete(self.observations) or not _observations_safe(
            self.observations
        ):
            raise SafetyViolation(
                "return authorization requires complete, safe operator observations"
            )
        if self.baseline_snapshot is None:
            raise SafetyViolation("baseline is missing for return")
        baseline = int(self.baseline_snapshot.positions_steps[self.axis - 1])
        lower, upper = self._safe_target_bounds()
        if not lower <= baseline <= upper:
            raise TargetValidationError(
                f"return baseline {baseline} outside safe bounds [{lower},{upper}]"
            )
        self._return_authorized = True
        self.report["stages"]["return_authorization"]["authorized"] = True
        self._expected_target_writes = 2
        command = self.controller.command_single_axis_with_metrics(self.axis, baseline)
        feedback = self.controller.read_state_snapshot()
        health_failures = _validate_snapshot_health(feedback, phase="return")
        final_position = int(feedback.positions_steps[self.axis - 1])
        final_error = final_position - baseline
        return_pass = not health_failures and abs(final_error) <= RETURN_TOLERANCE_STEPS
        self.failures.extend(health_failures)
        if not return_pass and not health_failures:
            self.failures.append(
                f"return residual {final_error} steps exceeds {RETURN_TOLERANCE_STEPS} step tolerance"
            )
        self._return_executed = True
        self._snapshots_logged += 1
        self.report["stages"]["return_authorization"].update(
            {
                "executed": True,
                "baseline_position_steps": baseline,
                "commanded_target_steps": int(command.commanded_target),
                "feedback": _snapshot_dict(feedback),
                "final_error_steps": final_error,
                "pass": return_pass,
                "write_count_at_stage": int(self.controller.write_count),
            }
        )
        if not return_pass:
            self._record_failure("return feedback safety gate failed", stopped=True)
            raise SafetyViolation(self.failures[-1])
        self._set_state(ProbeState.RETURN_COMPLETE)
        return self.report["stages"]["return_authorization"]

    def _audit_writes(self) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        records = _write_records(self.controller)
        unexpected: list[dict[str, Any]] = []
        target_records = 0
        for record in records:
            motor_id = int(record.get("motor_id", -1))
            register = int(record.get("register_address", -1))
            if motor_id == self.axis and register == TARGET_POSITION_ADDR:
                target_records += 1
            elif motor_id == self.axis and register == PAUSE_ADDR:
                # A selected-axis pause is an allowed safety stop record, not a retry.
                continue
            else:
                unexpected.append(record)
        expected_target_records = self._expected_target_writes
        if target_records != expected_target_records:
            self.failures.append(
                f"target write count {target_records} != expected {expected_target_records}"
            )
        if target_records > expected_target_records:
            self.failures.append("duplicate target write detected; automatic retry is not allowed")
        return records, unexpected

    def finalise(self) -> dict[str, Any]:
        records, unexpected = self._audit_writes()
        stage_baseline = self.report["stages"]["baseline_recording"]
        stage_forward = self.report["stages"]["forward_perturbation"]
        stage_hold = self.report["stages"]["observation_hold"]
        return_stage = self.report["stages"]["return_authorization"]
        no_retry = (
            getattr(self.controller, "retries", 0) == 0
            and not any(
                "duplicate target write" in reason for reason in self.failures
            )
        )
        all_feedback = self._snapshots_logged > 0 and all(
            isinstance(stage, dict)
            and isinstance(stage.get("feedback", stage.get("samples")), (dict, list))
            for stage in (stage_baseline, stage_forward)
            if stage is not None
        )
        hold_logged = isinstance(stage_hold, dict) and int(stage_hold.get("sample_count", 0)) >= 1
        observations_complete = _observations_complete(self.observations)
        observations_safe = _observations_safe(self.observations)
        return_pass = bool(return_stage.get("pass", False))
        no_unexpected = not unexpected
        pass_conditions = {
            "allowed_displacement": abs(self.displacement_steps) in ALLOWED_DISPLACEMENTS,
            "baseline_recorded": stage_baseline is not None,
            "forward_perturbation_recorded": stage_forward is not None,
            "observation_hold_recorded": hold_logged,
            "full_six_axis_feedback_logged": all_feedback and self._snapshots_logged >= 3,
            "operator_observations_complete": observations_complete,
            "operator_observations_safe": observations_safe,
            "return_explicitly_authorized": self._return_authorized,
            "return_feedback_within_tolerance": return_pass,
            "no_automatic_retry": no_retry,
            "no_unexpected_writes": no_unexpected,
            "home_park_cartesian_forbidden": True,
        }
        successful_probe = all(pass_conditions.values()) and not self.failures
        if self.state == ProbeState.RETURN_COMPLETE and successful_probe:
            outcome = "PASS"
        elif self.state in (ProbeState.FORWARD_COMPLETE, ProbeState.OBSERVATION_HOLD):
            outcome = "HOLD"
        elif self.state == ProbeState.STOPPED:
            outcome = "STOP"
        else:
            outcome = "FAIL"
        self.report.update(
            {
                "state": self.state.value,
                "outcome": outcome,
                "pass": successful_probe,
                "hardware_motion_executed": self.controller.hardware_mode == HardwareMode.ACTIVE
                and self._expected_target_writes > 0,
                "hardware_write_count": int(self.controller.write_count)
                if self.controller.hardware_mode == HardwareMode.ACTIVE
                else 0,
                "mock_write_count": int(self.controller.write_count)
                if self.controller.hardware_mode == HardwareMode.MOCK
                else 0,
                "write_audit": records,
                "unexpected_writes": unexpected,
                "pass_conditions": pass_conditions,
                "failure_reasons": list(dict.fromkeys(self.failures)),
                "metrics": {
                    "snapshots_logged": self._snapshots_logged,
                    "expected_target_write_count": self._expected_target_writes,
                    "target_write_count": sum(
                        int(record.get("motor_id", -1)) == self.axis
                        and int(record.get("register_address", -1)) == TARGET_POSITION_ADDR
                        for record in records
                    ),
                    "unexpected_write_count": len(unexpected),
                },
            }
        )
        return self.report


def run_first_motion_probe(
    controller: SixMotorController,
    *,
    axis: int,
    displacement_steps: int = DEFAULT_DISPLACEMENT_STEPS,
    hold_samples: int = DEFAULT_HOLD_SAMPLES,
    hold_interval_s: float = DEFAULT_HOLD_INTERVAL_S,
    observations: Mapping[str, str] | None = None,
    return_authorized: bool = False,
    authorized: bool = False,
    session_id: str | None = None,
) -> dict[str, Any]:
    """Run the probe through hold, returning only with explicit authorization."""

    session = FirstMotionProbeSession(
        controller,
        axis=axis,
        displacement_steps=displacement_steps,
        hold_samples=hold_samples,
        hold_interval_s=hold_interval_s,
        observations=observations,
        authorized=authorized,
        session_id=session_id,
    )
    try:
        session.record_baseline()
        session.forward_perturbation()
        session.observation_hold()
        if return_authorized:
            session.authorize_return()
    except Exception as exc:
        if not session.failures or session.failures[-1] != str(exc):
            session.failures.append(str(exc))
        if session.state in (
            ProbeState.FORWARD_COMPLETE,
            ProbeState.OBSERVATION_HOLD,
        ):
            session._set_state(ProbeState.STOPPED)
        elif session.state == ProbeState.CREATED:
            session._set_state(ProbeState.FAILED)
    finally:
        report = session.finalise()
        try:
            controller.close()
        except Exception as exc:
            report["failure_reasons"].append(f"close failure: {exc}")
            report["pass"] = False
            report["outcome"] = "FAIL"
    return report


def _parse_displacement(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("displacement must be an integer") from exc
    if abs(parsed) not in ALLOWED_DISPLACEMENTS:
        raise argparse.ArgumentTypeError(
            f"absolute displacement must be one of {ALLOWED_DISPLACEMENTS} steps"
        )
    return parsed


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J6 Phase 3 first motion observation probe")
    parser.add_argument("--axis", type=int, required=True, choices=MOTOR_IDS)
    parser.add_argument(
        "--displacement-steps",
        "--displacement",
        type=_parse_displacement,
        default=DEFAULT_DISPLACEMENT_STEPS,
    )
    parser.add_argument("--hold-samples", type=int, default=DEFAULT_HOLD_SAMPLES)
    parser.add_argument("--hold-interval-s", type=float, default=DEFAULT_HOLD_INTERVAL_S)
    parser.add_argument("--authorize-return", action="store_true")
    parser.add_argument("--confirm-return", action="store_true")
    parser.add_argument("--platform-motion", default="unrecorded")
    parser.add_argument("--ball-joint-condition", default="unrecorded")
    parser.add_argument("--interference", default="unrecorded")
    parser.add_argument("--abnormal-sound", default="unrecorded")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--confirm-first-motion-probe", action="store_true")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    confirmations = (
        args.confirm_motion,
        args.validated_config,
        args.confirm_first_motion_probe,
    )
    if args.real_hardware and not all(confirmations):
        print(
            "real hardware requires --confirm-motion, --validated-config, and "
            "--confirm-first-motion-probe",
            file=sys.stderr,
        )
        return 2
    if not args.real_hardware and any(confirmations):
        print("motion authorization flags require --real-hardware", file=sys.stderr)
        return 2
    if args.authorize_return and not args.confirm_return:
        print("--authorize-return requires --confirm-return", file=sys.stderr)
        return 2
    if not args.authorize_return and args.confirm_return:
        print("--confirm-return requires --authorize-return", file=sys.stderr)
        return 2
    mode = HardwareMode.ACTIVE if args.real_hardware else HardwareMode.MOCK
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        retries=0,
        hardware_mode=mode,
    )
    observations = {
        "platform_motion": args.platform_motion,
        "ball_joint_condition": args.ball_joint_condition,
        "interference": args.interference,
        "abnormal_sound": args.abnormal_sound,
    }
    try:
        report = run_first_motion_probe(
            controller,
            axis=args.axis,
            displacement_steps=args.displacement_steps,
            hold_samples=args.hold_samples,
            hold_interval_s=args.hold_interval_s,
            observations=observations,
            return_authorized=args.authorize_return,
            authorized=all(confirmations),
        )
        output = args.output or (
            WORKSPACE_ROOT
            / "log"
            / f"j6_phase3_first_motion_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        _write_report(report, output)
    except (TargetValidationError, ValueError, FileExistsError) as exc:
        print(f"J6 Phase 3 first-motion probe rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"outcome": report["outcome"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
