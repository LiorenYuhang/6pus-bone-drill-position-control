#!/usr/bin/env python3
"""Runner for J3 6-Axis Coordinated Startup Homing and Controlled Parking.

Executes synchronized 6-axis S-curve motion between arbitrary startup positions
and OPERATING_HOME ([1000]*6) or STORAGE_PARK ([0]*6).

Strictly follows Safety STOP / Performance Result separation.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Sequence

# Ensure workspace source packages are on path
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

from pus6_control.joint_trajectory_controller import compute_synchronized_step_point, quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    CONTROL_MODE_ADDR,
    DEFAULT_SPEED,
    DEFAULT_SERIAL_PORT,
    TargetValidationError,
    MAX_CURRENT_MA,
    MOTOR_IDS,
    NUM_MOTORS,
    OPERATING_HOME_RAW_STEPS,
    PAUSE_ADDR,
    STORAGE_PARK_RAW_STEPS,
    TARGET_POSITION_ADDR,
    HardwareMode,
    raw_to_relative_steps,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController
from pus6_motor_driver.uart_bus import UartBus

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("j3_startup_homing")

QUINTIC_PEAK_DERIVATIVE = 1.875
POSITIONING_MODE = 0
TRACKING_FAILURE_TOLERANCE_STEPS = 10
TRACKING_FAILURE_DURATION_S = 2.0
PARK_BASELINE_TOLERANCE_STEPS = 5
STOP_VERIFY_INTERVAL_S = 0.05


def _write_audit_records(controller: SixMotorController) -> list[dict[str, Any]]:
    bus = getattr(controller, "bus", None)
    records = getattr(bus, "write_audit", None)
    if records is None:
        return []
    return [dict(record) for record in records]


def _pause_write_ids(records: Sequence[dict[str, Any]]) -> set[int]:
    return {
        int(record.get("motor_id"))
        for record in records
        if int(record.get("register_address", -1)) == PAUSE_ADDR
    }


def _pause_and_verify(
    controller: SixMotorController,
    *,
    reason: str,
    sleep_fn: Callable[[float], None],
    verify_interval_s: float = STOP_VERIFY_INTERVAL_S,
    pause_already_attempted: bool = False,
    safety_triggered: bool = True,
) -> dict[str, Any]:
    """Pause immediately, then verify stable position feedback."""
    result: dict[str, Any] = {
        "triggered": safety_triggered,
        "reason": reason,
        "stop_requested": safety_triggered,
        "stop_request_timestamp_ns": time.perf_counter_ns(),
        "pause_already_attempted": pause_already_attempted,
        "pause_results": {},
        "pause_error": None,
        "verified_stopped": False,
        "verification": None,
    }
    audit_before = _write_audit_records(controller)
    try:
        if pause_already_attempted:
            # command_positions() already attempted its one allowed pause path.
            # Do not infer success merely from audit records: a failed UART write
            # is audited before transmission and must remain a failed stop.
            previous = getattr(controller, "last_pause_results", {})
            if not previous:
                result["pause_error"] = "pause result unavailable after prior pause attempt"
            result["pause_results"] = {
                str(mid): bool(previous.get(mid, False)) for mid in MOTOR_IDS
            }
        else:
            pause_results = controller.pause_all()
            result["pause_results"] = {
                str(mid): bool(pause_results.get(mid, False)) for mid in MOTOR_IDS
            }
    except Exception as exc:
        result["pause_error"] = f"{type(exc).__name__}: {exc}"
    audit_after = _write_audit_records(controller)
    result["pause_write_records"] = audit_after[len(audit_before):]
    try:
        first = controller.read_positions_snapshot()
        if verify_interval_s > 0:
            sleep_fn(verify_interval_s)
        second = controller.read_positions_snapshot()
        stable = list(first.positions_steps) == list(second.positions_steps)
        pause_ok = (
            result["pause_error"] is None
            and len(result["pause_results"]) == NUM_MOTORS
            and all(result["pause_results"].values())
        )
        result["verification"] = {
            "first_timestamp_start_ns": int(first.timestamp_start_ns),
            "first_timestamp_end_ns": int(first.timestamp_end_ns),
            "second_timestamp_start_ns": int(second.timestamp_start_ns),
            "second_timestamp_end_ns": int(second.timestamp_end_ns),
            "first_positions_steps": list(first.positions_steps),
            "second_positions_steps": list(second.positions_steps),
            "positions_stable": stable,
        }
        result["verified_stopped"] = bool(pause_ok and stable)
    except Exception as exc:
        result["verification"] = {"read_error": f"{type(exc).__name__}: {exc}", "positions_stable": False}
    result["stop_confirmation"] = "PASS" if result["verified_stopped"] else "FAIL"
    return result


def _exception_stop_reason(exc: Exception, stage: str) -> str:
    message = str(exc)
    lowered = message.lower()
    if isinstance(exc, TargetValidationError) or "safe boundary" in lowered:
        return f"Safety limit violation during {stage}: {message}"
    if "fault" in lowered:
        return f"Motor hardware fault during {stage}: {message}"
    if "current" in lowered or "overcurrent" in lowered:
        return f"Overcurrent during {stage}: {message}"
    return f"Communication failure during {stage}: {type(exc).__name__}: {message}"


def classify_write_audit(
    controller: SixMotorController,
    *,
    expected_motion_targets: Sequence[Sequence[int]],
    expected_configuration_count: int = NUM_MOTORS,
) -> dict[str, Any]:
    records = _write_audit_records(controller)
    configuration = [r for r in records if int(r.get("register_address", -1)) == CONTROL_MODE_ADDR]
    motion = [r for r in records if int(r.get("register_address", -1)) == TARGET_POSITION_ADDR]
    stop = [r for r in records if int(r.get("register_address", -1)) == PAUSE_ADDR]
    stop_ids = [int(r.get("motor_id", -1)) for r in stop]
    stop_payloads_valid = all(list(r.get("payload") or []) == [1] for r in stop)
    unexpected = [
        r for r in records
        if int(r.get("register_address", -1)) not in {CONTROL_MODE_ADDR, TARGET_POSITION_ADDR, PAUSE_ADDR}
    ]
    expected_motion = [
        {"motor_id": mid, "target": int(target)}
        for vector in expected_motion_targets
        for mid, target in zip(MOTOR_IDS, vector)
    ]
    observed_motion = []
    for record in motion:
        payload = list(record.get("payload") or [])
        try:
            target = int(payload[0]) if payload else None
        except (TypeError, ValueError):
            target = None
        observed_motion.append({"motor_id": int(record.get("motor_id", -1)), "target": target})
    motion_match = observed_motion == expected_motion
    if not motion_match:
        unexpected.append({"category": "motion_sequence_mismatch", "expected": expected_motion, "observed": observed_motion})
    config_ids = [int(r.get("motor_id")) for r in configuration]
    config_match = config_ids == list(MOTOR_IDS) and len(configuration) == expected_configuration_count
    if not config_match:
        unexpected.append({
            "category": "configuration_sequence_mismatch",
            "expected_count": expected_configuration_count,
            "observed_count": len(configuration),
            "expected_motor_ids": list(MOTOR_IDS),
            "observed_motor_ids": config_ids,
        })
    if any(len(list(r.get("payload") or [])) != 4 for r in configuration):
        unexpected.append({"category": "configuration_payload_mismatch", "observed": configuration})
    if stop and (stop_ids != list(MOTOR_IDS) or not stop_payloads_valid):
        unexpected.append({
            "category": "stop_sequence_mismatch",
            "expected_motor_ids": list(MOTOR_IDS),
            "observed_motor_ids": stop_ids,
            "payloads_valid": stop_payloads_valid,
        })
    return {
        "records": records,
        "expected_configuration_writes": configuration,
        "expected_motion_writes": motion,
        "expected_stop_writes": stop,
        "unexpected_writes": unexpected,
        "unexpected_writes_checked": True,
        "configuration_write_count": len(configuration),
        "motion_write_count": len(motion),
        "stop_write_count": len(stop),
        "classified_write_count": len(records),
        "total_writes": len(records),
        "configuration_sequence_matches": config_match,
        "motion_sequence_matches": motion_match,
        "unexpected_write_count": len(unexpected),
        "stop_sequence_matches": (not stop) or (stop_ids == list(MOTOR_IDS) and stop_payloads_valid),
        "audit_pass": not unexpected,
    }


def _select_trajectory_start(action: str, feedback_steps: Sequence[int]) -> tuple[list[int], list[int], bool]:
    """Separate signed feedback baseline from safe commanded trajectory start."""
    park_reference = list(STORAGE_PARK_RAW_STEPS)
    deviations = [int(value) - int(reference) for value, reference in zip(feedback_steps, park_reference)]
    within_tolerance = all(abs(delta) <= PARK_BASELINE_TOLERANCE_STEPS for delta in deviations)
    if within_tolerance:
        return park_reference, deviations, True
    return [int(value) for value in feedback_steps], deviations, False


def run_homing_session(
    action: str = "home",
    speed: int = DEFAULT_SPEED,
    control_rate_hz: float = 10.0,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    custom_start: list[int] | None = None,
    duration: float | None = None,
    settle_time: float = 2.0,
    port: str = DEFAULT_SERIAL_PORT,
    sleep_fn: Callable[[float], None] = time.sleep,
    stop_verify_interval_s: float = STOP_VERIFY_INTERVAL_S,
    tracking_failure_tolerance_steps: int = TRACKING_FAILURE_TOLERANCE_STEPS,
    controller: SixMotorController | None = None,
) -> dict[str, Any]:
    """Execute synchronized 6-axis homing or parking motion."""
    # Safety mode gating
    if real_hardware and confirm_motion and validated_config:
        hw_mode = HardwareMode.ACTIVE
        logger.info("⚡ GATED ACTIVE MODE: REAL HARDWARE WRITE AUTHORIZED")
    else:
        hw_mode = HardwareMode.MOCK
        logger.info("🛡️  MOCK MODE: Simulating 6-axis controller")

    # Target selection
    if action == "home":
        target_steps = list(OPERATING_HOME_RAW_STEPS)
        target_name = "OPERATING_HOME ([1000]*6)"
    elif action == "park":
        target_steps = list(STORAGE_PARK_RAW_STEPS)
        target_name = "STORAGE_PARK ([0]*6)"
    else:
        raise ValueError(f"Unknown action '{action}'. Must be 'home' or 'park'.")

    # Hardware connection
    if controller is None:
        if hw_mode == HardwareMode.MOCK:
            bus = FakeBus()
            if custom_start is not None:
                assert len(custom_start) == NUM_MOTORS
                bus.positions = {mid: custom_start[idx] for idx, mid in enumerate(MOTOR_IDS)}
                bus.targets = dict(bus.positions)
            controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
        else:
            controller = SixMotorController(hardware_mode=HardwareMode.ACTIVE, port=port)
    elif custom_start is not None:
        bus = getattr(controller, "bus", None)
        if hasattr(bus, "positions"):
            assert len(custom_start) == NUM_MOTORS
            bus.positions = {mid: custom_start[idx] for idx, mid in enumerate(MOTOR_IDS)}
            if hasattr(bus, "targets"):
                bus.targets = dict(bus.positions)

    controller.connect()

    # Pre-motion read-only baseline
    logger.info("Phase 1: Reading initial 6-axis starting positions...")
    initial_snapshot = controller.read_positions_snapshot()
    start_steps = list(initial_snapshot.positions_steps)
    logger.info(f"Initial positions (q_start): {start_steps}")
    logger.info(f"Target positions ({action}): {target_steps} -> {target_name}")
    trajectory_start_steps, park_baseline_deviation_steps, park_baseline_within_tolerance = _select_trajectory_start(action, start_steps)
    logger.info(f"Trajectory start reference: {trajectory_start_steps}; baseline deviation: {park_baseline_deviation_steps}; within tolerance: {park_baseline_within_tolerance}")

    # Validate target vector against safety boundaries
    validated_target = validate_target_vector(target_steps)

    # Compute duration based on maximum axis displacement
    displacements = [abs(t - s) for t, s in zip(validated_target, trajectory_start_steps)]
    max_disp = max(displacements)

    # For quintic polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5:
    # ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4, with exact peak derivative = 1.875 at tau = 0.5.
    # Therefore, to constrain maximum planned trajectory velocity <= speed (e.g. 50 step/s):
    # v_peak = D_max * 1.875 / T <= speed ==> T >= 1.875 * D_max / speed
    QUINTIC_PEAK_DERIVATIVE = 1.875

    if duration is not None:
        total_duration = max(0.01, duration)
    elif max_disp == 0:
        logger.info("All axes already at target. Motion skipped.")
        total_duration = 0.5
    else:
        peak_speed = max(1.0, float(speed))
        total_duration = max(2.0, (QUINTIC_PEAK_DERIVATIVE * float(max_disp)) / peak_speed)

    planned_peak_speed = (QUINTIC_PEAK_DERIVATIVE * float(max_disp)) / total_duration if total_duration > 0 else 0.0

    logger.info(
        f"Trajectory plan: max_disp={max_disp} steps ({max_disp * 0.025:.3f} mm), "
        f"target_peak_speed={planned_peak_speed:.2f} step/s ({planned_peak_speed * 0.025:.3f} mm/s), "
        f"duration T={total_duration:.2f} s (1.875 factor applied)"
    )

    # Configure motor speeds and operating mode
    # Set motor hardware velocity limit register (0x28) to at least the peak speed
    motor_reg_speed = max(50, int(round(planned_peak_speed * 1.2)))
    logger.info(
        f"Phase 2: Configuring motor parameters "
        f"(mode={POSITIONING_MODE}, speed_reg={motor_reg_speed})..."
    )
    stop_result: dict[str, Any] | None = None
    safety_aborted = False
    abort_reason = ""
    expected_motion_targets: list[list[int]] = []

    # Configuration is a write boundary. A partial configuration must stop
    # immediately; there is no settle window before the pause path.
    try:
        controller.configure_all(mode=POSITIONING_MODE, speed=motor_reg_speed)
    except Exception as exc:
        safety_aborted = True
        abort_reason = _exception_stop_reason(exc, "motor configuration")
        logger.error("🛑 SAFETY STOP: %s", abort_reason)
        stop_result = _pause_and_verify(
            controller,
            reason=abort_reason,
            sleep_fn=sleep_fn,
            verify_interval_s=stop_verify_interval_s,
            pause_already_attempted=False,
        )

    # Coordinated S-curve trajectory execution loop
    logger.info(f"Phase 3: Dispatching coordinated S-curve trajectory @ {control_rate_hz:.1f} Hz...")
    start_time = time.time()
    dt = 1.0 / control_rate_hz
    trajectory_log: list[dict[str, Any]] = []
    last_dispatched_cmd: list[int] | None = None
    write_frame_count = 0

    current_t = 0.0
    tracking_violation_count = 0
    tracking_violation_sample_limit = max(
        1, int(math.ceil(TRACKING_FAILURE_DURATION_S * control_rate_hz))
    )
    logger.info(
        "Tracking watchdog: limit=%d steps, consecutive_samples=%d (~%.2fs)",
        tracking_failure_tolerance_steps,
        tracking_violation_sample_limit,
        tracking_violation_sample_limit / max(0.001, control_rate_hz),
    )
    while not safety_aborted and current_t <= total_duration + 0.05:
        loop_start = time.time()
        tau = min(1.0, current_t / max(0.001, total_duration))
        s_prog = quintic_progress(tau)

        # Interpolated command coordinate: q_i(t) = trajectory_start_i + s(t) * (q_target_i - trajectory_start_i)
        step_cmd = [
            int(round(s_i + s_prog * (t_i - s_i)))
            for s_i, t_i in zip(trajectory_start_steps, validated_target)
        ]

        # Dispatch if command changed or at start
        if step_cmd != last_dispatched_cmd:
            try:
                controller.command_positions(step_cmd)
                write_frame_count += NUM_MOTORS
                last_dispatched_cmd = list(step_cmd)
                expected_motion_targets.append(list(step_cmd))
            except Exception as exc:
                safety_aborted = True
                abort_reason = _exception_stop_reason(exc, "motion target dispatch")
                logger.error("🛑 SAFETY STOP: %s", abort_reason)
                stop_result = _pause_and_verify(
                    controller,
                    reason=abort_reason,
                    sleep_fn=sleep_fn,
                    verify_interval_s=stop_verify_interval_s,
                    pause_already_attempted=(not isinstance(exc, TargetValidationError)),
                )
                break

        # Telemetry sampling
        try:
            current_snapshot = controller.read_state_snapshot()
        except Exception as exc:
            safety_aborted = True
            abort_reason = _exception_stop_reason(exc, "feedback read")
            logger.error("🛑 SAFETY STOP: %s", abort_reason)
            stop_result = _pause_and_verify(
                controller,
                reason=abort_reason,
                sleep_fn=sleep_fn,
                verify_interval_s=stop_verify_interval_s,
            )
            break
        record = {
            "t": round(current_t, 3),
            "tau": round(tau, 4),
            "s": round(s_prog, 4),
            "cmd_steps": step_cmd,
            "actual_steps": current_snapshot.positions_steps,
            "currents_ma": current_snapshot.currents_mA,
            "temperatures_c": current_snapshot.temperatures_C,
            "forces_g": current_snapshot.forces_g_raw,
            "faults": current_snapshot.fault_codes,
        }
        trajectory_log.append(record)

        # Safety Check: strict Safety STOP conditions only
        if any(f != 0 for f in current_snapshot.fault_codes):
            safety_aborted = True
            abort_reason = f"Motor hardware fault detected: {current_snapshot.fault_codes}"
            logger.error(f"🛑 SAFETY STOP: {abort_reason}")
            stop_result = _pause_and_verify(
                controller,
                reason=abort_reason,
                sleep_fn=sleep_fn,
                verify_interval_s=stop_verify_interval_s,
            )
            break

        if any(c > MAX_CURRENT_MA for c in current_snapshot.currents_mA):
            safety_aborted = True
            abort_reason = f"Overcurrent detected: {current_snapshot.currents_mA} mA (> {MAX_CURRENT_MA} mA)"
            logger.error(f"🛑 SAFETY STOP: {abort_reason}")
            stop_result = _pause_and_verify(
                controller,
                reason=abort_reason,
                sleep_fn=sleep_fn,
                verify_interval_s=stop_verify_interval_s,
            )
            break

        tracking_errors = [
            abs(actual - command)
            for actual, command in zip(current_snapshot.positions_steps, step_cmd)
        ]
        max_tracking_error = max(tracking_errors) if tracking_errors else 0
        if max_tracking_error > tracking_failure_tolerance_steps:
            # S-curve acceleration can produce a short-lived lag spike. Only
            # stop after the violation persists for the configured duration.
            tracking_violation_count += 1
        else:
            # Recovery below the tracking limit clears the transient watchdog.
            tracking_violation_count = 0

        if tracking_violation_count >= tracking_violation_sample_limit:
            safety_aborted = True
            abort_reason = (
                f"Tracking failure: actual/target deviation {tracking_errors} steps "
                f"exceeds tolerance {tracking_failure_tolerance_steps} for "
                f"{tracking_violation_count} consecutive samples"
            )
            logger.error("🛑 SAFETY STOP: %s", abort_reason)
            stop_result = _pause_and_verify(
                controller,
                reason=abort_reason,
                sleep_fn=sleep_fn,
                verify_interval_s=stop_verify_interval_s,
            )
            break

        current_t = time.time() - start_time
        sleep_rem = dt - (time.time() - loop_start)
        if sleep_rem > 0:
            sleep_fn(sleep_rem)

    # Phase 4: normal completion retains the existing settle window. Safety
    # aborts have already executed the pause path above and never enter it.
    if not safety_aborted:
        logger.info(f"Phase 4: Settling observation and stop confirmation ({settle_time:.2f}s)...")
        sleep_fn(settle_time)
        final_snapshot = controller.read_positions_snapshot()
        stop_result = _pause_and_verify(
            controller,
            reason="normal trajectory completion",
            sleep_fn=sleep_fn,
            verify_interval_s=stop_verify_interval_s,
            safety_triggered=False,
        )
    else:
        verification = (stop_result or {}).get("verification") or {}
        stopped_positions = verification.get("first_positions_steps")
        if stopped_positions is not None:
            final_positions = list(stopped_positions)
        else:
            try:
                final_positions = list(controller.read_positions_snapshot().positions_steps)
            except Exception:
                final_positions = list(start_steps)

        class _FinalSnapshot:
            positions_steps = final_positions

        final_snapshot = _FinalSnapshot()

    # Performance Evaluation (Recorded as performance metrics, not abort conditions)
    final_errors = [
        abs(f - t) for f, t in zip(final_snapshot.positions_steps, validated_target)
    ]
    max_error = max(final_errors) if final_errors else 0
    tracking_pass = max_error <= 2  # +/- 2 step normal quantization/settle band

    relative_steps = raw_to_relative_steps(final_snapshot.positions_steps)

    logger.info("=" * 60)
    logger.info("HOMING / PARKING EXECUTION SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Action:                  {action.upper()}")
    logger.info(f"Hardware Mode:           {hw_mode.value}")
    logger.info(f"Start Raw Steps:         {start_steps}")
    logger.info(f"Target Raw Steps:        {validated_target}")
    logger.info(f"Final Raw Steps:         {final_snapshot.positions_steps}")
    logger.info(f"Final Relative Steps:    {relative_steps}")
    logger.info(f"Final Position Errors:   {final_errors} (max: {max_error} step)")
    logger.info(f"Tracking Tolerable (<=2):{'PASS' if tracking_pass else 'RECORDED (deviation > 2)'}")
    logger.info(f"Safety Invariants Kept:  {'PASS' if not safety_aborted else 'FAILED (' + abort_reason + ')'}")
    logger.info("=" * 60)

    # Build report object
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    report = {
        "timestamp": timestamp_str,
        "action": action,
        "hardware_mode": hw_mode.value,
        "planned_duration_s": total_duration,
        "planned_peak_speed_step_s": planned_peak_speed,
        "actual_elapsed_s": time.time() - start_time,
        "speed_step_s": speed,
        "port": port if hw_mode == HardwareMode.ACTIVE else None,
        "motor_reg_speed_0x28": motor_reg_speed,
        "control_rate_hz": control_rate_hz,
        "dispatched_write_frames": write_frame_count,
        "q_start_raw": start_steps,
        "trajectory_start_raw": trajectory_start_steps,
        "park_baseline_deviation_steps": park_baseline_deviation_steps,
        "park_baseline_within_tolerance": park_baseline_within_tolerance,
        "park_baseline_tolerance_steps": PARK_BASELINE_TOLERANCE_STEPS,
        "q_target_raw": validated_target,
        "q_final_raw": final_snapshot.positions_steps,
        "q_final_relative": relative_steps,
        "final_errors_steps": final_errors,
        "max_error_steps": max_error,
        "tracking_tolerable": tracking_pass,
        "safety_aborted": safety_aborted,
        "abort_reason": abort_reason,
        "stop_result": stop_result,
        "trajectory_sample_count": len(trajectory_log),
    }

    report["write_audit"] = classify_write_audit(
        controller,
        expected_motion_targets=expected_motion_targets,
    )
    report["write_audit"]["total_reads"] = getattr(controller, "read_count", 0)

    # Save log
    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"j3_homing_{action}_{timestamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump({"report": report, "trajectory": trajectory_log}, f, indent=2)
    logger.info(f"Detailed log saved to: {log_file}")

    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="J3 6-Axis Startup Homing / Controlled Parking")
    parser.add_argument("--action", choices=["home", "park"], default="home", help="Target action: home ([1000]*6) or park ([0]*6)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Cruise speed in step/s (default: 50)")
    parser.add_argument("--control-rate", type=float, default=10.0, help="Control dispatch rate in Hz (default: 10.0)")
    parser.add_argument("--real-hardware", action="store_true", help="Enable real serial bus hardware connection")
    parser.add_argument("--confirm-motion", action="store_true", help="Explicit human confirmation for physical movement")
    parser.add_argument("--validated-config", action="store_true", help="Explicit human confirmation for validated parameters")
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=f"UART device (default: validated by-id path {DEFAULT_SERIAL_PORT})",
    )

    args = parser.parse_args()
    report = run_homing_session(
        action=args.action,
        speed=args.speed,
        control_rate_hz=args.control_rate,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
        port=args.port,
    )

    if report["safety_aborted"]:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
