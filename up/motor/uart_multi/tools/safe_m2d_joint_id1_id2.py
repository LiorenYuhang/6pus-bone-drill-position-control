#!/usr/bin/env python3
"""Stage M2-D: move ID1 and ID2 once on one audited UART bus."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import logging
from pathlib import Path
import statistics
import sys
import time
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import (  # noqa: E402
    CMD_FRAME_HEADER,
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    CMD_WR_REGISTER,
    Frame,
    StatusFeedback,
    parse_frame,
)
from tools.safe_endpoint_escape_id1 import (  # noqa: E402
    BaselineBandGuard,
    MotionTrace,
    SafetyViolation,
    _check_status,
    _stop_confirmation_metrics,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (1, 2)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
MOVE_STEPS = 10
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 2.0
MAX_CURRENT_MA = 500
BASELINE_DURATION_S = 2.0
BASELINE_RATE_HZ = 50.0
POLL_INTERVAL_S = 0.02
STOP_CONFIRM_DURATION_S = 0.5
INTEGRATION_TOLERANCE_STEPS = 2
STRICT_TOLERANCE_STEPS = 1
SOFTWARE_UPPER_LIMIT = 1800
ALLOWED_WRITE_REGISTERS = {
    CONTROL_MODE_ADDR,
    TARGET_POSITION_ADDR,
    PAUSE_ADDR,
}

logger = logging.getLogger("safe_m2d_joint_id1_id2")


@dataclass(frozen=True)
class ConfigSnapshot:
    mode: int
    speed: int
    target: int


@dataclass(frozen=True)
class M2dPositionBaseline:
    reference: int
    safety_min: int
    safety_max: int
    histogram: dict[int, int]


@dataclass(frozen=True)
class MotorBaseline:
    position: M2dPositionBaseline
    force_f0: float
    current_max_mA: int


class JointAuditedBus:
    """Audit every frame and limit M2-D writes to ID1/ID2 safety registers."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.write_frames: list[Frame] = []
        self.read_counts: Counter[int] = Counter()

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if expected_id is not None and expected_id != parsed.motor_id:
            raise SafetyViolation(
                f"request expected_id={expected_id} does not match "
                f"destination ID={parsed.motor_id}"
            )
        if parsed.motor_id not in MOTOR_IDS:
            raise SafetyViolation(
                f"destination ID={parsed.motor_id} is outside M2-D ID1/ID2 scope"
            )
        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                logger.critical(
                    "WRITE_BLOCKED destination_id=%d register=0x%04X",
                    parsed.motor_id,
                    parsed.register_addr,
                )
                raise SafetyViolation(
                    f"write register 0x{parsed.register_addr:04X} is forbidden in M2-D"
                )
            self.write_frames.append(parsed)
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=%d register=0x%04X "
                "data=%s verified=true",
                parsed.motor_id,
                parsed.register_addr,
                parsed.data.hex(" "),
            )
        elif parsed.command in (CMD_RD_STATUS, CMD_RD_REGISTER):
            self.read_counts[parsed.motor_id] += 1
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X "
                "register=0x%04X",
                parsed.motor_id,
                parsed.command,
                parsed.register_addr,
            )
        else:
            raise SafetyViolation(
                f"outgoing command 0x{parsed.command:02X} is outside M2-D allowlist"
            )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        destinations = [frame.motor_id for frame in self.write_frames]
        registers = [f"0x{frame.register_addr:04X}" for frame in self.write_frames]
        return (
            f"ACCESS_SUMMARY write_count={len(self.write_frames)} "
            f"write_destinations={destinations} write_registers={registers} "
            f"id1_read_count={self.read_counts[1]} "
            f"id2_read_count={self.read_counts[2]} "
            f"other_id_write_count=0"
        )


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(log_path, mode="x", encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    for target in (logger, logging.getLogger("uart_bus")):
        target.setLevel(logging.INFO)
        target.handlers.clear()
        target.addHandler(handler)
        target.propagate = False


def _read_value(motor: Motor, register_addr: int) -> int:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise SafetyViolation(
            f"ID{motor.motor_id} register 0x{register_addr:02X} returned "
            f"{len(response.values)} values"
        )
    return response.values[0]


def _read_config(motor: Motor) -> ConfigSnapshot:
    return ConfigSnapshot(
        mode=_read_value(motor, CONTROL_MODE_ADDR),
        speed=_read_value(motor, TARGET_SPEED_ADDR),
        target=_read_value(motor, TARGET_POSITION_ADDR),
    )


def _validate_limits(motor_id: int, lower: int, upper: int) -> None:
    if not 0 <= lower <= upper <= 2000:
        raise SafetyViolation(
            f"ID{motor_id} invalid hardware limits lower={lower} upper={upper}"
        )


def _log_status(
    motor_id: int,
    source: str,
    status: StatusFeedback,
    *,
    force_f0: Optional[float] = None,
    target: Optional[int] = None,
    elapsed_s: Optional[float] = None,
) -> None:
    error = abs(status.current_position - target) if target is not None else "-"
    delta = status.force_g - force_f0 if force_f0 is not None else None
    logger.info(
        "ID%d_STATUS source=%s elapsed_s=%s target=%s actual=%d error=%s "
        "current_mA=%d force_g=%d delta_force_g=%s temperature_C=%d fault=0x%02X",
        motor_id,
        source,
        f"{elapsed_s:.6f}" if elapsed_s is not None else "-",
        target if target is not None else "-",
        status.current_position,
        error,
        status.current_mA,
        status.force_g,
        f"{delta:.3f}" if delta is not None else "-",
        status.temperature,
        status.error_code,
    )


def _build_m2d_position_baseline(positions: list[int]) -> M2dPositionBaseline:
    if not positions:
        raise SafetyViolation("M2-D baseline has no position samples")
    histogram = dict(sorted(Counter(positions).items()))
    highest_count = max(histogram.values())
    modes = [
        position
        for position, count in histogram.items()
        if count == highest_count
    ]
    if len(modes) != 1:
        raise SafetyViolation(
            "M2-D baseline position has no unique mode: "
            f"histogram={histogram} modes={modes}"
        )
    reference = modes[0]
    safety_min = min(positions)
    safety_max = max(positions)
    if any(abs(position - reference) >= 2 for position in positions):
        raise SafetyViolation(
            "M2-D baseline sample deviates from unique mode by at least 2 steps: "
            f"baseline_reference={reference} baseline_safety_min={safety_min} "
            f"baseline_safety_max={safety_max} histogram={histogram}"
        )
    return M2dPositionBaseline(
        reference=reference,
        safety_min=safety_min,
        safety_max=safety_max,
        histogram=histogram,
    )


def _collect_baselines(
    motors: dict[int, Motor], limits: dict[int, tuple[int, int]]
) -> dict[int, MotorBaseline]:
    sample_count = max(1, int(round(BASELINE_DURATION_S * BASELINE_RATE_HZ)))
    positions = {motor_id: [] for motor_id in MOTOR_IDS}
    forces = {motor_id: [] for motor_id in MOTOR_IDS}
    currents = {motor_id: [] for motor_id in MOTOR_IDS}
    started = time.monotonic()
    for index in range(sample_count):
        scheduled = started + index / BASELINE_RATE_HZ
        delay = scheduled - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            lower, upper = limits[motor_id]
            _check_status(
                status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            _log_status(motor_id, "baseline", status)
            positions[motor_id].append(status.current_position)
            forces[motor_id].append(status.force_g)
            currents[motor_id].append(status.current_mA)
    return {
        motor_id: MotorBaseline(
            position=_build_m2d_position_baseline(positions[motor_id]),
            force_f0=statistics.fmean(forces[motor_id]),
            current_max_mA=max(currents[motor_id]),
        )
        for motor_id in MOTOR_IDS
    }


def _pause_both_once(
    motors: dict[int, Motor],
) -> tuple[dict[int, bool], dict[int, StatusFeedback]]:
    results: dict[int, bool] = {}
    responses: dict[int, StatusFeedback] = {}
    for motor_id in MOTOR_IDS:
        try:
            response = motors[motor_id].write_register(PAUSE_ADDR, [1])
            results[motor_id] = True
            responses[motor_id] = response
            logger.warning(
                "ID%d_PAUSE sent_once=true destination_id=%d actual=%d "
                "current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
                motor_id,
                motor_id,
                response.current_position,
                response.current_mA,
                response.force_g,
                response.temperature,
                response.error_code,
            )
        except Exception as exc:
            results[motor_id] = False
            logger.critical("ID%d pause command failed: %s", motor_id, exc)
    if not all(results.get(motor_id, False) for motor_id in MOTOR_IDS):
        message = (
            "HARD STOP REQUIRED: M2-D one or more pause commands failed; use the "
            "hardware emergency stop or remove motor power immediately"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return results, responses


def _confirm_both_stopped(
    motors: dict[int, Motor],
) -> tuple[bool, dict[int, StatusFeedback]]:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses = {motor_id: [] for motor_id in MOTOR_IDS}
    while True:
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            statuses[motor_id].append(status)
            _log_status(motor_id, "post_pause_read_only", status)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    stable_by_id: dict[int, bool] = {}
    metrics_by_id: dict[int, dict[str, object]] = {}
    for motor_id in MOTOR_IDS:
        stable_by_id[motor_id], metrics_by_id[motor_id] = (
            _stop_confirmation_metrics(statuses[motor_id])
        )
    stable = all(stable_by_id.values())
    logger.info(
        "M2D_STOP_CONFIRMATION stable=%s id1_stable=%s id1_reason=%s "
        "id2_stable=%s id2_reason=%s",
        stable,
        stable_by_id[1],
        metrics_by_id[1]["reason"],
        stable_by_id[2],
        metrics_by_id[2]["reason"],
    )
    if not stable:
        message = (
            "HARD STOP REQUIRED: M2-D stop confirmation failed; use the hardware "
            "emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return stable, {
        motor_id: statuses[motor_id][-1] for motor_id in MOTOR_IDS
    }


def _confirm_operator() -> None:
    answer = input(
        "确认仅连接并固定 ID1+ID2、两台正方向区域安全、急停可用；"
        "输入 M2-D MOVE ID1 ID2："
    ).strip()
    if answer != "M2-D MOVE ID1 ID2":
        raise SafetyViolation("operator did not arm Stage M2-D")


def _hard_stop_for_confirmation_exception(exc: Exception) -> None:
    message = (
        "HARD STOP REQUIRED: M2-D post-pause read-only confirmation failed; "
        "use the hardware emergency stop or remove motor power"
    )
    logger.critical("%s: %s", message, exc)
    print(message, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m2/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m2d_joint_id1_id2.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[JointAuditedBus] = None
    motors: dict[int, Motor] = {}
    traces: dict[int, MotionTrace] = {}
    armed = False
    pause_attempted = False
    pause_results: dict[int, bool] = {}
    confirmation_attempted = False
    stop_confirmation: Optional[bool] = None
    targets: dict[int, int] = {}
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = JointAuditedBus(bus)
        motors = {
            motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS
        }

        limits: dict[int, tuple[int, int]] = {}
        configs_before: dict[int, ConfigSnapshot] = {}
        for motor_id in MOTOR_IDS:
            lower = _read_value(motors[motor_id], LOWER_LIMIT_ADDR)
            upper = _read_value(motors[motor_id], UPPER_LIMIT_ADDR)
            _validate_limits(motor_id, lower, upper)
            limits[motor_id] = (lower, upper)
            configs_before[motor_id] = _read_config(motors[motor_id])
            initial = motors[motor_id].read_status()
            _check_status(
                initial,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            _log_status(motor_id, "preflight", initial)

        baselines = _collect_baselines(motors, limits)
        for motor_id in MOTOR_IDS:
            baseline = baselines[motor_id]
            targets[motor_id] = baseline.position.reference + MOVE_STEPS
            lower, upper = limits[motor_id]
            effective_upper = min(upper, SOFTWARE_UPPER_LIMIT)
            if not lower <= targets[motor_id] <= effective_upper:
                raise SafetyViolation(
                    f"ID{motor_id} target={targets[motor_id]} outside allowed "
                    f"[{lower},{effective_upper}]"
                )
            logger.info(
                "M2D_PRECHECK ID%d config=%s limits=[%d,%d] "
                "baseline_reference=%d baseline_safety_min=%d "
                "baseline_safety_max=%d baseline_histogram=%s "
                "force_f0=%.3f baseline_current_max_mA=%d target=%d "
                "delta_steps=%d speed=%d timeout_s=%.3f max_current_mA=%d",
                motor_id,
                configs_before[motor_id],
                lower,
                upper,
                baseline.position.reference,
                baseline.position.safety_min,
                baseline.position.safety_max,
                baseline.position.histogram,
                baseline.force_f0,
                baseline.current_max_mA,
                targets[motor_id],
                MOVE_STEPS,
                SPEED_STEPS_PER_SECOND,
                MOTION_TIMEOUT_S,
                MAX_CURRENT_MA,
            )
        print(
            "M2-D PRECHECK "
            f"ID1 baseline_reference={baselines[1].position.reference} "
            f"baseline_safety=[{baselines[1].position.safety_min},"
            f"{baselines[1].position.safety_max}] target={targets[1]}; "
            f"ID2 baseline_reference={baselines[2].position.reference} "
            f"baseline_safety=[{baselines[2].position.safety_min},"
            f"{baselines[2].position.safety_max}] target={targets[2]}"
        )
        _confirm_operator()

        logger.warning(
            "M2D_ARMING allowed_write_destinations=[1,2] "
            "allowed_write_registers=[0x0025,0x0029,0x001A]"
        )
        armed = True
        for motor_id in MOTOR_IDS:
            baseline = baselines[motor_id]
            setup = motors[motor_id].write_register(
                CONTROL_MODE_ADDR,
                [
                    SPEED_MODE,
                    0,
                    0,
                    SPEED_STEPS_PER_SECOND,
                    baseline.position.safety_max,
                ],
            )
            lower, upper = limits[motor_id]
            _check_status(
                setup,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
                target=baseline.position.safety_max,
            )
            _log_status(
                motor_id,
                "setup_write_response",
                setup,
                force_f0=baseline.force_f0,
                target=baseline.position.safety_max,
            )

        t0_mono = time.monotonic()
        t0_wall = datetime.now(timezone.utc).isoformat()
        traces = {
            motor_id: MotionTrace(
                baseline_min=baselines[motor_id].position.safety_min,
                baseline_max=baselines[motor_id].position.safety_max,
                target=targets[motor_id],
                force_f0=baselines[motor_id].force_f0,
                t0_mono=t0_mono,
                t0_wall=t0_wall,
                samples=[],
            )
            for motor_id in MOTOR_IDS
        }
        direction_guards = {
            motor_id: BaselineBandGuard(
                baseline_min=baselines[motor_id].position.safety_min,
                baseline_max=baselines[motor_id].position.safety_max,
            )
            for motor_id in MOTOR_IDS
        }
        target_write_times: dict[int, float] = {}
        target_write_responses: dict[int, StatusFeedback] = {}
        for motor_id in MOTOR_IDS:
            target_write_times[motor_id] = time.monotonic()
            response = motors[motor_id].write_register(
                TARGET_POSITION_ADDR, [targets[motor_id]]
            )
            target_write_responses[motor_id] = response
            error = abs(response.current_position - targets[motor_id])
            elapsed = traces[motor_id].observe(response, error, source="command")
            lower, upper = limits[motor_id]
            _check_status(
                response,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
                target=targets[motor_id],
                direction_guard=direction_guards[motor_id],
            )
            _log_status(
                motor_id,
                "target_write_response",
                response,
                force_f0=baselines[motor_id].force_f0,
                target=targets[motor_id],
                elapsed_s=elapsed,
            )
        for motor_id in MOTOR_IDS:
            readback_target = _read_value(motors[motor_id], TARGET_POSITION_ADDR)
            if readback_target != targets[motor_id]:
                raise SafetyViolation(
                    f"ID{motor_id} target write readback mismatch: "
                    f"commanded_target={targets[motor_id]} "
                    f"readback_target_position={readback_target}"
                )
            logger.info(
                "ID%d_TARGET_WRITE_READBACK commanded_target=%d "
                "readback_target_position=%d actual_position=%d current_mA=%d "
                "force_g=%d fault=0x%02X",
                motor_id,
                targets[motor_id],
                readback_target,
                target_write_responses[motor_id].current_position,
                target_write_responses[motor_id].current_mA,
                target_write_responses[motor_id].force_g,
                target_write_responses[motor_id].error_code,
            )
        dispatch_skew = target_write_times[2] - target_write_times[1]
        logger.info(
            "M2D_MOTION_T0 t0=%s ID1_target=%d ID2_target=%d "
            "target_write_order=[1,2] target_dispatch_skew_s=%.6f "
            "synchronization_tolerance_defined=false",
            t0_wall,
            targets[1],
            targets[2],
            dispatch_skew,
        )

        deadline = t0_mono + MOTION_TIMEOUT_S
        while True:
            all_ready = True
            for motor_id in MOTOR_IDS:
                status = motors[motor_id].read_status()
                error = abs(status.current_position - targets[motor_id])
                elapsed = traces[motor_id].observe(status, error)
                lower, upper = limits[motor_id]
                _check_status(
                    status,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                    target=targets[motor_id],
                    direction_guard=direction_guards[motor_id],
                )
                _log_status(
                    motor_id,
                    "joint_motion_poll",
                    status,
                    force_f0=baselines[motor_id].force_f0,
                    target=targets[motor_id],
                    elapsed_s=elapsed,
                )
                positive_motion = traces[motor_id].first_motion_mono is not None
                all_ready = (
                    all_ready
                    and positive_motion
                    and error <= INTEGRATION_TOLERANCE_STEPS
                )
            if all_ready:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SafetyViolation(
                    "ID1/ID2 did not both confirm positive motion and reach "
                    f"+/-{INTEGRATION_TOLERANCE_STEPS} step within "
                    f"{MOTION_TIMEOUT_S:.2f}s"
                )
            time.sleep(min(POLL_INTERVAL_S, remaining))

        pause_attempted = True
        pause_results, pause_responses = _pause_both_once(motors)
        for motor_id, response in pause_responses.items():
            traces[motor_id].set_pause_status(response)
        if not all(pause_results.get(motor_id, False) for motor_id in MOTOR_IDS):
            raise SafetyViolation("M2-D completion pause failed")

        confirmation_attempted = True
        try:
            stop_confirmation, final_statuses = _confirm_both_stopped(motors)
        except Exception as exc:
            _hard_stop_for_confirmation_exception(exc)
            stop_confirmation = False
            raise
        for motor_id, status in final_statuses.items():
            traces[motor_id].set_post_pause_status(status)
        if not stop_confirmation:
            raise SafetyViolation("M2-D stop confirmation failed")

        final_errors: dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            final_status = final_statuses[motor_id]
            lower, upper = limits[motor_id]
            _check_status(
                final_status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            final_errors[motor_id] = abs(
                final_status.current_position - targets[motor_id]
            )
            if final_errors[motor_id] > INTEGRATION_TOLERANCE_STEPS:
                raise SafetyViolation(
                    f"ID{motor_id} post-pause final_error={final_errors[motor_id]} "
                    f"exceeds {INTEGRATION_TOLERANCE_STEPS} step"
                )
            post_pause_target = _read_value(motors[motor_id], TARGET_POSITION_ADDR)
            logger.info(
                "ID%d_POST_PAUSE_STATE commanded_target=%d "
                "readback_target_position=%d actual_position=%d error=%d "
                "current_mA=%d force_g=%d fault=0x%02X "
                "target_sync_observation_only=true",
                motor_id,
                targets[motor_id],
                post_pause_target,
                final_status.current_position,
                final_errors[motor_id],
                final_status.current_mA,
                final_status.force_g,
                final_status.error_code,
            )

        logger.info(
            "M2D_RESULT decision=PASS ID1_target=%d ID1_final_actual=%d "
            "ID1_final_error=%d ID1_strict_tolerance_pass=%s "
            "ID2_target=%d ID2_final_actual=%d ID2_final_error=%d "
            "ID2_strict_tolerance_pass=%s target_dispatch_skew_s=%.6f "
            "pause_id1=%s pause_id2=%s stop_confirmation=%s",
            targets[1],
            final_statuses[1].current_position,
            final_errors[1],
            final_errors[1] <= STRICT_TOLERANCE_STEPS,
            targets[2],
            final_statuses[2].current_position,
            final_errors[2],
            final_errors[2] <= STRICT_TOLERANCE_STEPS,
            dispatch_skew,
            pause_results[1],
            pause_results[2],
            stop_confirmation,
        )
        for motor_id in MOTOR_IDS:
            logger.info(
                "ID%d_%s",
                motor_id,
                traces[motor_id].result_line(
                    decision="PASS",
                    pause_triggered=True,
                    pause_success=pause_results[motor_id],
                    stop_confirmation=stop_confirmation,
                ),
            )
        print(
            f"PASS M2-D ID1 target={targets[1]} "
            f"final={final_statuses[1].current_position} error={final_errors[1]}; "
            f"ID2 target={targets[2]} final={final_statuses[2].current_position} "
            f"error={final_errors[2]} pause_both=true stop_confirmation=true"
        )
        return 0
    except Exception as exc:
        if armed and not pause_attempted and motors:
            pause_attempted = True
            pause_results, pause_responses = _pause_both_once(motors)
            for motor_id, response in pause_responses.items():
                if motor_id in traces:
                    traces[motor_id].set_pause_status(response)
        if (
            armed
            and pause_results
            and all(pause_results.get(motor_id, False) for motor_id in MOTOR_IDS)
            and not confirmation_attempted
            and motors
        ):
            confirmation_attempted = True
            try:
                stop_confirmation, final_statuses = _confirm_both_stopped(motors)
                for motor_id, status in final_statuses.items():
                    if motor_id in traces:
                        traces[motor_id].set_post_pause_status(status)
            except Exception as confirmation_exc:
                _hard_stop_for_confirmation_exception(confirmation_exc)
                stop_confirmation = False
        for motor_id, trace in traces.items():
            logger.info(
                "ID%d_%s",
                motor_id,
                trace.result_line(
                    decision="FAIL",
                    pause_triggered=pause_attempted,
                    pause_success=pause_results.get(motor_id),
                    stop_confirmation=stop_confirmation,
                ),
            )
        logger.error(
            "M2D_RESULT decision=FAIL pause_attempted=%s pause_id1=%s "
            "pause_id2=%s stop_confirmation=%s reason=%s",
            pause_attempted,
            pause_results.get(1),
            pause_results.get(2),
            stop_confirmation,
            exc,
        )
        print(f"STOP M2-D: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Stage M2-D: fixed +10 step joint ID1/ID2 motion on one UART bus"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log", help="独立日志路径；已有文件不会覆盖")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("timeout 必须大于 0")
    if args.retries < 0:
        parser.error("retries 不能小于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
