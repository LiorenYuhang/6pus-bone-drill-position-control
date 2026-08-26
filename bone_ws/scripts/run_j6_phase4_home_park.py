#!/usr/bin/env python3
"""Guarded J6 Phase 4 HOME/PARK commissioning wrapper.

This wrapper delegates trajectory generation and execution to the existing J3
startup homing runner only after the J6 commissioning gates are explicit and
valid.  The default mode is Mock; no wrapper option silently authorizes motion.
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
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))
if str(DRIVER_ROOT) not in sys.path:
    sys.path.insert(0, str(DRIVER_ROOT))

from pus6_motor_driver.safety import DEFAULT_SERIAL_PORT  # noqa: E402

J3Delegate = Callable[..., dict[str, Any]]
COMMISSIONING_SPEED_STEP_S = 50


def _gate_failures(
    *,
    action: str,
    speed: int,
    phase2_passed: bool,
    phase3_passed: bool,
    e_stop_ready: bool,
    two_person_observation: bool,
    maintenance_authorized: bool,
    real_hardware: bool,
    confirm_motion: bool,
    validated_config: bool,
) -> list[str]:
    failures: list[str] = []
    if action not in {"home", "park"}:
        failures.append("action must be home or park")
    if speed != COMMISSIONING_SPEED_STEP_S:
        failures.append(f"Phase 4 speed must be exactly {COMMISSIONING_SPEED_STEP_S} step/s")
    if not phase2_passed:
        failures.append("Phase 2 axis mapping/static feedback gate is not PASS")
    if not phase3_passed:
        failures.append("Phase 3 single-axis direction/clearance gate is not PASS")
    if not e_stop_ready:
        failures.append("E-stop readiness was not explicitly confirmed")
    if not two_person_observation:
        failures.append("two-person HOME/PARK observation was not explicitly confirmed")
    if action == "park" and not maintenance_authorized:
        failures.append("STORAGE_PARK requires explicit maintenance authorization")
    if real_hardware and not (confirm_motion and validated_config):
        failures.append("real hardware requires both confirm_motion and validated_config")
    return failures


def run_phase4_home_park(
    *,
    action: str = "home",
    speed: int = COMMISSIONING_SPEED_STEP_S,
    control_rate_hz: float = 10.0,
    phase2_passed: bool,
    phase3_passed: bool,
    e_stop_ready: bool,
    two_person_observation: bool,
    maintenance_authorized: bool = False,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    custom_start: list[int] | None = None,
    duration: float | None = None,
    settle_time: float = 2.0,
    port: str = DEFAULT_SERIAL_PORT,
    delegate: J3Delegate | None = None,
) -> dict[str, Any]:
    """Run guarded J3 HOME/PARK and normalize its result to J6 schema."""

    failures = _gate_failures(
        action=action,
        speed=speed,
        phase2_passed=phase2_passed,
        phase3_passed=phase3_passed,
        e_stop_ready=e_stop_ready,
        two_person_observation=two_person_observation,
        maintenance_authorized=maintenance_authorized,
        real_hardware=real_hardware,
        confirm_motion=confirm_motion,
        validated_config=validated_config,
    )
    report: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_phase4_home_park_wrapper",
        "phase": "J6_PHASE4",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "action": action,
        "hardware_mode": "active" if real_hardware else "mock",
        "port": port if real_hardware else None,
        "baudrate": 921600,
        "hardware_motion_executed": bool(real_hardware),
        "hardware_write_count": 0,
        "mock_write_count": 0,
        "write_audit": {},
        "expected_configuration_writes": [],
        "expected_motion_writes": [],
        "expected_stop_writes": [],
        "unexpected_writes": [],
        "stop_result": None,
        "phase2_passed": phase2_passed,
        "phase3_passed": phase3_passed,
        "e_stop_ready": e_stop_ready,
        "two_person_observation": two_person_observation,
        "maintenance_authorized": maintenance_authorized,
        "speed_step_s": speed,
        "pass_conditions": {},
        "failure_reasons": failures,
        "source_tool": "run_j3_startup_homing.run_homing_session",
    }
    if failures:
        report["outcome"] = "STOP"
        report["pass"] = False
        return report

    try:
        if delegate is None:
            from run_j3_startup_homing import run_homing_session  # type: ignore

            delegate = run_homing_session
        j3_report = delegate(
            action=action,
            speed=speed,
            control_rate_hz=control_rate_hz,
            real_hardware=real_hardware,
            confirm_motion=confirm_motion,
            validated_config=validated_config,
            custom_start=custom_start,
            duration=duration,
            settle_time=settle_time,
            port=port,
        )
        report["source_report"] = j3_report
        report["q_start_raw"] = j3_report.get("q_start_raw")
        report["trajectory_start_raw"] = j3_report.get("trajectory_start_raw")
        report["park_baseline_deviation_steps"] = j3_report.get("park_baseline_deviation_steps")
        report["park_baseline_within_tolerance"] = j3_report.get("park_baseline_within_tolerance")
        report["park_baseline_tolerance_steps"] = j3_report.get("park_baseline_tolerance_steps")
        report["q_target_raw"] = j3_report.get("q_target_raw")
        report["q_final_raw"] = j3_report.get("q_final_raw")
        report["final_errors_steps"] = j3_report.get("final_errors_steps")
        report["max_error_steps"] = j3_report.get("max_error_steps")
        report["tracking_tolerable"] = bool(j3_report.get("tracking_tolerable"))
        report["safety_aborted"] = bool(j3_report.get("safety_aborted"))
        report["stop_result"] = j3_report.get("stop_result")
        expected_target = [1000] * 6 if action == "home" else [0] * 6
        report["target_matches_action"] = j3_report.get("q_target_raw") == expected_target
        audit = j3_report.get("write_audit", {})
        report["write_audit"] = audit
        for key in (
            "expected_configuration_writes",
            "expected_motion_writes",
            "expected_stop_writes",
            "unexpected_writes",
        ):
            report[key] = audit.get(key, [])
        total_writes = int(audit.get("total_writes", 0))
        report["hardware_write_count"] = total_writes if real_hardware else 0
        report["mock_write_count"] = total_writes if not real_hardware else 0
        audit_categories = {
            "expected_configuration_writes",
            "expected_motion_writes",
            "expected_stop_writes",
            "unexpected_writes",
        }
        audit_complete = (
            audit.get("unexpected_writes_checked") is True
            and audit_categories.issubset(audit)
        )
        stop_confirmed = bool((j3_report.get("stop_result") or {}).get("verified_stopped"))
        report["pass_conditions"] = {
            "phase2_gate_passed": phase2_passed,
            "phase3_gate_passed": phase3_passed,
            "speed_limit_enforced": speed == COMMISSIONING_SPEED_STEP_S,
            "target_matches_action": report["target_matches_action"],
            "tracking_within_two_steps": report["tracking_tolerable"],
            "audit_classification_complete": audit_complete,
            "unexpected_writes_zero": audit_complete and not report["unexpected_writes"],
            "motion_audit_matches": bool(audit.get("motion_sequence_matches", False)),
            "configuration_audit_matches": bool(audit.get("configuration_sequence_matches", False)),
            "stop_confirmation_recorded": stop_confirmed,
            "hardware_writes_recorded": total_writes == int(audit.get("classified_write_count", -1)),
            "safety_not_aborted": not report["safety_aborted"],
        }
        if not all(report["pass_conditions"].values()):
            report["failure_reasons"].extend(
                key for key, value in report["pass_conditions"].items() if not value
            )
        report["pass"] = not report["failure_reasons"] and all(report["pass_conditions"].values())
        report["outcome"] = "PASS" if report["pass"] else ("STOP" if report["safety_aborted"] else "FAIL")
    except Exception as exc:
        report["failure_reasons"].append(f"J3 homing delegate failed: {exc}")
        report["outcome"] = "STOP"
        report["pass"] = False
    except Exception as exc:
        report["failure_reasons"].append(f"J3 homing delegate failed: {exc}")
        report["outcome"] = "STOP"
        report["pass"] = False
    return report


def _write_report(report: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Guarded J6 Phase 4 HOME/PARK wrapper")
    parser.add_argument("--action", choices=["home", "park"], default="home")
    parser.add_argument("--speed", type=int, default=COMMISSIONING_SPEED_STEP_S)
    parser.add_argument("--control-rate", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--phase2-passed", action="store_true")
    parser.add_argument("--phase3-passed", action="store_true")
    parser.add_argument("--e-stop-ready", action="store_true")
    parser.add_argument("--two-person-observation", action="store_true")
    parser.add_argument("--maintenance-authorized", action="store_true")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase4_{args.action}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    report = run_phase4_home_park(
        action=args.action,
        speed=args.speed,
        control_rate_hz=args.control_rate,
        phase2_passed=args.phase2_passed,
        phase3_passed=args.phase3_passed,
        e_stop_ready=args.e_stop_ready,
        two_person_observation=args.two_person_observation,
        maintenance_authorized=args.maintenance_authorized,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
        duration=args.duration,
        settle_time=args.settle_time,
        port=args.port,
    )
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "phase": report["phase"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
