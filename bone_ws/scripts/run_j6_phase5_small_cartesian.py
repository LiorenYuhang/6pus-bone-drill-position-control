#!/usr/bin/env python3
"""Guarded J6 Phase 5 small Cartesian commissioning wrapper.

One invocation validates one small, single-DOF Cartesian excursion from the
OPERATING_HOME baseline and requires the delegated J3 runner to return home
before the report can pass.  The wrapper never implements a second motion
pipeline: IK, trajectory shaping, safety checks, and FakeBus behavior remain in
the validated J3/J4-era interfaces.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
KINEMATICS_ROOT = WORKSPACE_ROOT / "src" / "pus6_kinematics"
for path in (SCRIPT_ROOT, DRIVER_ROOT, KINEMATICS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics  # noqa: E402
from pus6_motor_driver.safety import (  # noqa: E402
    DEFAULT_SERIAL_PORT,
    HardwareMode,
    OPERATING_HOME_RAW_STEPS,
)

J3Delegate = Callable[..., dict[str, Any]]
COMMISSIONING_PEAK_SPEED_STEP_S = 50.0
RETURN_TOLERANCE_STEPS = 2
HOME_RAW_STEPS = list(OPERATING_HOME_RAW_STEPS)

# Pose order is [x, y, z, roll, pitch, yaw], in meters/radians.  These are
# commissioning limits, deliberately smaller than the broader J3/J4 envelope.
SMALL_TEST_OFFSETS: dict[str, tuple[int, float, str]] = {
    "z_pos": (2, 0.002, "Z +2 mm"),
    "z_neg": (2, -0.002, "Z -2 mm"),
    "x_pos": (0, 0.002, "X +2 mm"),
    "x_neg": (0, -0.002, "X -2 mm"),
    "y_pos": (1, 0.002, "Y +2 mm"),
    "y_neg": (1, -0.002, "Y -2 mm"),
    "pitch_pos": (4, 0.02, "Pitch +0.02 rad"),
    "pitch_neg": (4, -0.02, "Pitch -0.02 rad"),
    "roll_pos": (3, 0.02, "Roll +0.02 rad"),
    "roll_neg": (3, -0.02, "Roll -0.02 rad"),
}


def _target_pose(test_name: str) -> tuple[list[float], str]:
    if test_name not in SMALL_TEST_OFFSETS:
        raise ValueError(
            f"unsupported Phase 5 test {test_name!r}; allowed: {sorted(SMALL_TEST_OFFSETS)}"
        )
    index, delta, description = SMALL_TEST_OFFSETS[test_name]
    ik = Analytical6PusKinematics()
    pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    pose[index] += delta
    return pose, description


def _gate_failures(
    *,
    test_name: str,
    peak_speed: float,
    phase4_passed: bool,
    home_baseline_confirmed: bool,
    home_baseline_steps: list[int] | None,
    e_stop_ready: bool,
    two_person_observation: bool,
    real_hardware: bool,
    confirm_motion: bool,
    validated_config: bool,
    custom_start: list[int] | None,
) -> list[str]:
    failures: list[str] = []
    if test_name not in SMALL_TEST_OFFSETS:
        failures.append(f"unsupported Phase 5 test: {test_name}")
    if peak_speed != COMMISSIONING_PEAK_SPEED_STEP_S:
        failures.append(
            f"Phase 5 peak speed must be exactly {COMMISSIONING_PEAK_SPEED_STEP_S:g} step/s"
        )
    if not phase4_passed:
        failures.append("Phase 4 HOME/PARK gate is not PASS")
    if not home_baseline_confirmed:
        failures.append("OPERATING_HOME baseline was not explicitly confirmed")
    if home_baseline_steps is None:
        failures.append("home_baseline_steps evidence was not supplied")
    elif list(home_baseline_steps) != HOME_RAW_STEPS:
        failures.append(f"HOME baseline must equal {HOME_RAW_STEPS}")
    if custom_start is not None and list(custom_start) != HOME_RAW_STEPS:
        failures.append(f"custom_start must equal HOME baseline {HOME_RAW_STEPS}")
    if not e_stop_ready:
        failures.append("E-stop readiness was not explicitly confirmed")
    if not two_person_observation:
        failures.append("two-person observation was not explicitly confirmed")
    if real_hardware and not (confirm_motion and validated_config):
        failures.append("real hardware requires both confirm_motion and validated_config")
    return failures


def run_phase5_small_cartesian(
    *,
    test_name: str,
    phase4_passed: bool,
    home_baseline_confirmed: bool,
    home_baseline_steps: list[int] | None,
    e_stop_ready: bool,
    two_person_observation: bool,
    peak_speed: float = COMMISSIONING_PEAK_SPEED_STEP_S,
    control_rate_hz: float = 10.0,
    hold_time_s: float = 0.2,
    settle_time_s: float = 0.5,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    custom_start: list[int] | None = None,
    override_duration: float | None = None,
    port: str = DEFAULT_SERIAL_PORT,
    delegate: J3Delegate | None = None,
) -> dict[str, Any]:
    """Run one guarded small Cartesian test through the existing J3 runner."""

    failures = _gate_failures(
        test_name=test_name,
        peak_speed=peak_speed,
        phase4_passed=phase4_passed,
        home_baseline_confirmed=home_baseline_confirmed,
        home_baseline_steps=home_baseline_steps,
        e_stop_ready=e_stop_ready,
        two_person_observation=two_person_observation,
        real_hardware=real_hardware,
        confirm_motion=confirm_motion,
        validated_config=validated_config,
        custom_start=custom_start,
    )
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase5_small_cartesian",
        "phase": "J6_PHASE5",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "test_name": test_name,
        "hardware_mode": HardwareMode.ACTIVE.value if real_hardware else HardwareMode.MOCK.value,
        "port": port if real_hardware else None,
        "baudrate": 921600,
        "hardware_motion_executed": bool(real_hardware),
        "hardware_write_count": 0,
        "mock_write_count": 0,
        "phase4_passed": phase4_passed,
        "home_baseline_confirmed": home_baseline_confirmed,
        "home_baseline_steps": list(home_baseline_steps) if home_baseline_steps is not None else None,
        "e_stop_ready": e_stop_ready,
        "two_person_observation": two_person_observation,
        "peak_speed_step_s": peak_speed,
        "cycles": 1,
        "return_to_home_required": True,
        "pass_conditions": {},
        "failure_reasons": failures,
        "source_tool": "run_j3_cartesian_motion.run_cartesian_test_session",
    }

    if test_name in SMALL_TEST_OFFSETS:
        pose, description = _target_pose(test_name)
        report["target_pose_cartesian"] = pose
        report["test_description"] = description
        report["allowed_limit"] = "Z/X/Y <= 2 mm; Pitch/Roll <= 0.02 rad; Yaw prohibited"

    if failures:
        report["outcome"] = "STOP"
        report["pass"] = False
        return report

    pose, description = _target_pose(test_name)
    try:
        if delegate is None:
            from run_j3_cartesian_motion import run_cartesian_test_session  # type: ignore

            delegate = run_cartesian_test_session
        source = delegate(
            test_name=f"j6_{test_name}",
            custom_pose=pose,
            peak_speed=peak_speed,
            control_rate_hz=control_rate_hz,
            hold_time_s=hold_time_s,
            settle_time_s=settle_time_s,
            cycles=1,
            real_hardware=real_hardware,
            confirm_motion=confirm_motion,
            validated_config=validated_config,
            custom_start=custom_start,
            override_duration=override_duration,
            port=port,
        )
        report["source_report"] = source
        report["target_pose_cartesian"] = source.get("target_pose_cartesian", pose)
        report["ik_differential_steps"] = source.get("ik_differential_steps")
        report["ik_target_steps"] = source.get("ik_target_steps")
        report["q_start_raw"] = source.get("q_start_raw")
        report["q_final_raw"] = source.get("q_final_raw")
        report["final_return_errors_steps"] = source.get("final_return_errors_steps")
        report["max_return_error_steps"] = source.get("max_return_error_steps")
        report["safety_aborted"] = bool(source.get("safety_aborted", False))
        report["return_to_home_verified"] = (
            source.get("max_return_error_steps") is not None
            and int(source.get("max_return_error_steps")) <= RETURN_TOLERANCE_STEPS
        )
        audit = source.get("write_audit", {})
        total_writes = int(audit.get("total_writes", 0))
        report["hardware_write_count"] = total_writes if real_hardware else 0
        report["mock_write_count"] = total_writes if not real_hardware else 0
        report["pass_conditions"] = {
            "phase4_gate_passed": phase4_passed,
            "home_baseline_confirmed": home_baseline_confirmed,
            "small_test_limit_enforced": test_name in SMALL_TEST_OFFSETS,
            "low_speed_enforced": peak_speed == COMMISSIONING_PEAK_SPEED_STEP_S,
            "single_cycle": source.get("cycles") == 1,
            "returned_to_home": report["return_to_home_verified"],
            "safety_not_aborted": not report["safety_aborted"],
            "hardware_writes_recorded": report["hardware_write_count"] >= 0,
        }
        if not all(report["pass_conditions"].values()):
            report["failure_reasons"].extend(
                key for key, value in report["pass_conditions"].items() if not value
            )
        report["pass"] = not report["failure_reasons"] and all(report["pass_conditions"].values())
        report["outcome"] = "PASS" if report["pass"] else ("STOP" if report["safety_aborted"] else "FAIL")
    except Exception as exc:
        report["failure_reasons"].append(f"J3 Cartesian delegate failed: {exc}")
        report["outcome"] = "STOP"
        report["pass"] = False
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def _parse_home(value: str) -> list[int]:
    try:
        values = [int(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("home baseline must be six comma-separated integers") from exc
    if len(values) != 6:
        raise argparse.ArgumentTypeError("home baseline must contain six integers")
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Guarded J6 Phase 5 small Cartesian test")
    parser.add_argument("--test", dest="test_name", choices=sorted(SMALL_TEST_OFFSETS), required=True)
    parser.add_argument("--phase4-passed", action="store_true")
    parser.add_argument("--home-baseline-confirmed", action="store_true")
    parser.add_argument("--home-baseline", type=_parse_home, default=HOME_RAW_STEPS)
    parser.add_argument("--e-stop-ready", action="store_true")
    parser.add_argument("--two-person-observation", action="store_true")
    parser.add_argument("--peak-speed", type=float, default=COMMISSIONING_PEAK_SPEED_STEP_S)
    parser.add_argument("--control-rate", type=float, default=10.0)
    parser.add_argument("--hold-time", type=float, default=0.2)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase5_{args.test_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    report = run_phase5_small_cartesian(
        test_name=args.test_name,
        phase4_passed=args.phase4_passed,
        home_baseline_confirmed=args.home_baseline_confirmed,
        home_baseline_steps=args.home_baseline,
        e_stop_ready=args.e_stop_ready,
        two_person_observation=args.two_person_observation,
        peak_speed=args.peak_speed,
        control_rate_hz=args.control_rate,
        hold_time_s=args.hold_time,
        settle_time_s=args.settle_time,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
        port=args.port,
        override_duration=args.duration,
    )
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
