#!/usr/bin/env python3
"""Single, read-guarded positive escape from the ID1 low endpoint.

This is deliberately narrower than ``safe_micro_move_id1.py``: it permits one
positive five- or ten-step command only when the measured position is below the
low endpoint gate. Force is recorded relative to a short static baseline, but
no force threshold is invented until the baseline and the manufacturer's limit
semantics have been reviewed.
"""

from __future__ import annotations

import argparse
from collections import Counter
import logging
import statistics
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import Register, StatusFeedback  # noqa: E402
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 1
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
ESCAPE_SPEED_STEPS_PER_SECOND = 50
DEFAULT_ESCAPE_STEP = 5
ALLOWED_ESCAPE_STEPS = (5, 10)
LOW_ENDPOINT_GATE = 199
DEFAULT_BASELINE_DURATION = 2.0
DEFAULT_BASELINE_RATE = 50.0
SETTLE_TOLERANCE_STEPS = 1

logger = logging.getLogger("safe_endpoint_escape_id1")


@dataclass
class MotionTrace:
    baseline_min: int
    baseline_max: int
    target: int
    force_f0: float
    t0_mono: float
    t0_wall: str
    samples: list[tuple[float, StatusFeedback, int]]
    poll_statuses: list[StatusFeedback] = field(default_factory=list)
    first_motion_mono: Optional[float] = None
    first_motion_wall: Optional[str] = None
    first_motion_current_mA: Optional[int] = None
    first_in_tolerance_mono: Optional[float] = None
    first_in_tolerance_wall: Optional[str] = None
    post_pause_status: Optional[StatusFeedback] = None
    pause_status: Optional[StatusFeedback] = None

    def set_post_pause_status(self, status: StatusFeedback) -> None:
        self.post_pause_status = status

    def set_pause_status(self, status: StatusFeedback) -> None:
        self.pause_status = status

    def observe(
        self, status: StatusFeedback, error: int, *, source: str = "poll"
    ) -> float:
        now = time.monotonic()
        wall = datetime.now(timezone.utc).isoformat()
        elapsed = now - self.t0_mono
        self.samples.append((elapsed, status, error))
        if source == "poll":
            self.poll_statuses.append(status)
        # Values inside the measured baseline band are static quantization,
        # not motion. Positive motion begins only above the upper boundary.
        if (
            self.first_motion_mono is None
            and status.current_position > self.baseline_max
        ):
            self.first_motion_mono = now
            self.first_motion_wall = wall
            self.first_motion_current_mA = status.current_mA
        if self.first_in_tolerance_mono is None and error <= SETTLE_TOLERANCE_STEPS:
            self.first_in_tolerance_mono = now
            self.first_in_tolerance_wall = wall
        return elapsed

    def result_line(
        self,
        *,
        decision: str,
        pause_triggered: bool,
        pause_success: Optional[bool],
        stop_confirmation: Optional[bool],
    ) -> str:
        sample_statuses = [item[1] for item in self.samples]
        poll_currents = [status.current_mA for status in self.poll_statuses]
        observed_statuses = list(sample_statuses)
        if self.pause_status is not None:
            observed_statuses.append(self.pause_status)
        if self.post_pause_status is not None:
            observed_statuses.append(self.post_pause_status)
        observed_currents = [status.current_mA for status in observed_statuses]
        currents: list[int] = []
        forces: list[int] = []
        deltas: list[float] = []
        temperatures: list[int] = []
        faults: list[int] = []
        final_actual: object = "-"
        trigger_actual: object = "-"
        trigger_error: object = "-"
        post_stop_final_actual: object = "-"
        post_stop_final_error: object = "-"
        if self.samples:
            _, trigger_status, trigger_error_value = self.samples[-1]
            trigger_actual = trigger_status.current_position
            trigger_error = trigger_error_value
        if self.post_pause_status is not None:
            final_status = self.post_pause_status
            final_error = abs(final_status.current_position - self.target)
            final_actual = final_status.current_position
            post_stop_final_actual = final_status.current_position
            post_stop_final_error = final_error
            if self.samples:
                currents = [item[1].current_mA for item in self.samples]
                forces = [item[1].force_g for item in self.samples]
                deltas = [item[1].force_g - self.force_f0 for item in self.samples]
                temperatures = [item[1].temperature for item in self.samples]
                faults = sorted({item[1].error_code for item in self.samples})
        elif self.samples:
            _, final_status, final_error = self.samples[-1]
            currents = [item[1].current_mA for item in self.samples]
            forces = [item[1].force_g for item in self.samples]
            deltas = [item[1].force_g - self.force_f0 for item in self.samples]
            temperatures = [item[1].temperature for item in self.samples]
            faults = sorted({item[1].error_code for item in self.samples})
            final_actual = final_status.current_position
        else:
            final_actual, final_error = "-", "-"
        first_motion_elapsed = (
            f"{self.first_motion_mono - self.t0_mono:.6f}"
            if self.first_motion_mono is not None
            else "-"
        )
        first_tolerance_elapsed = (
            f"{self.first_in_tolerance_mono - self.t0_mono:.6f}"
            if self.first_in_tolerance_mono is not None
            else "-"
        )
        actual_max: object = "-"
        final_platform_actual: object = "-"
        final_platform_elapsed: object = "-"
        if self.samples:
            actual_max = max(item[1].current_position for item in self.samples)
            final_platform_actual = self.samples[-1][1].current_position
            platform_start = len(self.samples) - 1
            while (
                platform_start > 0
                and self.samples[platform_start - 1][1].current_position
                == final_platform_actual
            ):
                platform_start -= 1
            final_platform_elapsed = f"{self.samples[platform_start][0]:.6f}"
        return (
            f"RUN_RESULT decision={decision} baseline_min={self.baseline_min} "
            f"baseline_max={self.baseline_max} target={self.target} "
            f"delta_steps={self.target - self.baseline_max} t0={self.t0_wall} "
            f"first_motion_time={self.first_motion_wall or '-'} "
            f"first_motion_elapsed_s={first_motion_elapsed} "
            f"first_motion_current_mA={self.first_motion_current_mA if self.first_motion_current_mA is not None else '-'} "
            f"first_in_tolerance_time={self.first_in_tolerance_wall or '-'} "
            f"first_in_tolerance_elapsed_s={first_tolerance_elapsed} "
            f"actual_max={actual_max} final_platform_actual={final_platform_actual} "
            f"final_platform_elapsed_s={final_platform_elapsed} "
            f"trigger_actual={trigger_actual} trigger_error={trigger_error} "
            f"post_stop_final_actual={post_stop_final_actual} post_stop_final_error={post_stop_final_error} "
            f"final_actual={final_actual} final_error={final_error} "
            f"max_current_mA={max(observed_currents) if observed_currents else '-'} "
            f"max_poll_current_mA={max(poll_currents) if poll_currents else '-'} "
            f"max_observed_current_mA={max(observed_currents) if observed_currents else '-'} "
            f"force_min_g={min(forces) if forces else '-'} force_max_g={max(forces) if forces else '-'} "
            f"delta_force_min_g={min(deltas) if deltas else '-'} delta_force_max_g={max(deltas) if deltas else '-'} "
            f"temperature_min_C={min(temperatures) if temperatures else '-'} "
            f"temperature_max_C={max(temperatures) if temperatures else '-'} "
            f"faults={','.join(f'0x{fault:02X}' for fault in faults) or '-'} "
            f"pause_triggered={pause_triggered} pause_success={pause_success} "
            f"stop_confirmation={stop_confirmation}"
        )


class SafetyViolation(RuntimeError):
    """A safety precondition or runtime limit was violated."""


@dataclass(frozen=True)
class PositionBaseline:
    minimum: int
    maximum: int
    histogram: dict[int, int]

    @property
    def span(self) -> int:
        return self.maximum - self.minimum


@dataclass(frozen=True)
class BaselineBandGuard:
    """Reject only feedback below the measured pre-motion baseline band."""

    baseline_min: int
    baseline_max: int

    def observe(self, actual_position: int) -> None:
        if actual_position < self.baseline_min:
            raise SafetyViolation(
                "negative-direction movement detected below baseline band: "
                f"baseline=[{self.baseline_min},{self.baseline_max}] "
                f"actual={actual_position}"
            )


class _DeferredClose:
    """Keep a constructed UartBus open until the surrounding handler finishes."""

    def __init__(self, bus: UartBus) -> None:
        self.bus = bus

    def __enter__(self) -> UartBus:
        return self.bus

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        # The outer finally closes the bus after the safety handler has run.
        return False


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
            f"register 0x{register_addr:02X} returned {len(response.values)} values"
        )
    return response.values[0]


def _build_position_baseline(positions: list[int]) -> PositionBaseline:
    """Build the accepted pre-motion position band."""

    if not positions:
        raise SafetyViolation("baseline position samples are empty")
    histogram = dict(sorted(Counter(positions).items()))
    baseline = PositionBaseline(
        minimum=min(positions),
        maximum=max(positions),
        histogram=histogram,
    )
    if baseline.span > 1:
        raise SafetyViolation(
            "baseline position span exceeds one step: "
            f"min={baseline.minimum} max={baseline.maximum} "
            f"span={baseline.span} histogram={histogram}"
        )
    return baseline


def _check_status(
    status: StatusFeedback,
    *,
    lower: int,
    upper: int,
    max_current_ma: int,
    target: Optional[int] = None,
    direction_guard: Optional[BaselineBandGuard] = None,
) -> int:
    if status.error_code != 0:
        raise SafetyViolation(f"fault code=0x{status.error_code:02X}")
    if status.current_mA > max_current_ma:
        raise SafetyViolation(
            f"current={status.current_mA}mA exceeds {max_current_ma}mA"
        )
    if not lower <= status.current_position <= upper:
        raise SafetyViolation(
            f"actual position={status.current_position} outside device limits [{lower}, {upper}]"
        )
    if direction_guard is not None:
        direction_guard.observe(status.current_position)
    error_steps = abs(status.current_position - target) if target is not None else 0
    return error_steps


def _log_status(
    label: str,
    status: StatusFeedback,
    *,
    baseline_force: Optional[float],
    target: Optional[int],
    elapsed_s: Optional[float] = None,
) -> None:
    error = abs(status.current_position - target) if target is not None else "-"
    delta = (
        f"{status.force_g - baseline_force:.3f}"
        if baseline_force is not None
        else "-"
    )
    logger.info(
        "%s elapsed_s=%s target=%s actual=%d error=%s current_mA=%d force_g=%d delta_force_g=%s "
        "temperature_C=%d fault=0x%02X",
        label,
        f"{elapsed_s:.6f}" if elapsed_s is not None else "-",
        target if target is not None else "-",
        status.current_position,
        error,
        status.current_mA,
        status.force_g,
        delta,
        status.temperature,
        status.error_code,
    )


def _pause_once(motor: Optional[Motor]) -> tuple[bool, Optional[StatusFeedback]]:
    if motor is None:
        return False, None
    try:
        response = motor.write_register(PAUSE_ADDR, [1])
        logger.warning(
            "safety pause command sent once response_actual=%d response_current_mA=%d "
            "response_force_g=%d response_temperature_C=%d response_fault=0x%02X",
            response.current_position,
            response.current_mA,
            response.force_g,
            response.temperature,
            response.error_code,
        )
        return True, response
    except Exception as exc:
        message = (
            "HARD STOP REQUIRED: safety pause command failed; "
            "use the hardware emergency stop or remove motor power immediately"
        )
        logger.critical("%s: %s", message, exc)
        print(message, file=sys.stderr)
        return False, None


def _stop_confirmation_metrics(
    statuses: list[StatusFeedback], *, stable_tail_samples: int = 3
) -> tuple[bool, dict[str, object]]:
    """Evaluate bounded post-pause jitter without masking sustained drift."""

    positions = [status.current_position for status in statuses]
    currents = [status.current_mA for status in statuses]
    faults = sorted({status.error_code for status in statuses if status.error_code != 0})
    differences = [b - a for a, b in zip(positions, positions[1:])]
    increases = sum(delta > 0 for delta in differences)
    decreases = sum(delta < 0 for delta in differences)
    direction_changes = sum(
        previous * current < 0
        for previous, current in zip(differences, differences[1:])
    )
    tail_length = 0
    if positions:
        tail_value = positions[-1]
        for position in reversed(positions):
            if position != tail_value:
                break
            tail_length += 1
    span = max(positions) - min(positions) if positions else None
    metrics: dict[str, object] = {
        "samples": len(statuses),
        "position_first": positions[0] if positions else None,
        "position_last": positions[-1] if positions else None,
        "position_min": min(positions) if positions else None,
        "position_max": max(positions) if positions else None,
        "position_span_steps": span,
        "increases": increases,
        "decreases": decreases,
        "direction_changes": direction_changes,
        "stable_tail_samples": tail_length,
        "max_current_mA": max(currents) if currents else None,
        "faults": faults,
    }
    if not statuses:
        return False, {**metrics, "reason": "no post-pause samples"}
    if any(current != 0 for current in currents):
        return False, {**metrics, "reason": "post-pause current is nonzero"}
    if faults:
        return False, {**metrics, "reason": "post-pause fault is nonzero"}
    if span is None or span > 1:
        return False, {**metrics, "reason": "position span exceeds one step"}
    # A bounded two-sided sequence is the confirmed static quantization
    # pattern. A one-step final landing is accepted once its tail is stable.
    if increases and decreases:
        return True, {**metrics, "reason": "bounded bidirectional jitter"}
    if tail_length >= stable_tail_samples:
        return True, {**metrics, "reason": "bounded stable final position"}
    return False, {**metrics, "reason": "bounded but directional trend not settled"}


def _confirm_stopped(
    motor: Motor, *, duration: float = 0.5
) -> tuple[bool, Optional[StatusFeedback]]:
    """Read-only post-pause confirmation that position is no longer changing."""

    deadline = time.monotonic() + duration
    statuses: list[StatusFeedback] = []
    last_status: Optional[StatusFeedback] = None
    while True:
        status = motor.read_status()
        last_status = status
        statuses.append(status)
        logger.info(
            "post_pause position=%d current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
            status.current_position,
            status.current_mA,
            status.force_g,
            status.temperature,
            status.error_code,
        )
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    stable, metrics = _stop_confirmation_metrics(statuses)
    logger.info(
        "post_pause_summary stable=%s reason=%s samples=%s position_first=%s "
        "position_last=%s position_min=%s position_max=%s position_span_steps=%s "
        "increases=%s decreases=%s direction_changes=%s stable_tail_samples=%s "
        "max_current_mA=%s faults=%s",
        stable,
        metrics["reason"],
        metrics["samples"],
        metrics["position_first"],
        metrics["position_last"],
        metrics["position_min"],
        metrics["position_max"],
        metrics["position_span_steps"],
        metrics["increases"],
        metrics["decreases"],
        metrics["direction_changes"],
        metrics["stable_tail_samples"],
        metrics["max_current_mA"],
        metrics["faults"],
    )
    if not stable:
        message = (
            "HARD STOP REQUIRED: post-pause stop confirmation criteria failed "
            f"({metrics['reason']}); use the hardware emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return stable, last_status


def _confirm() -> None:
    answer = input(
        "确认仅连接并固定 ID1、正方向区域安全、急停可用；输入 ESCAPE ID1："
    ).strip()
    if answer != "ESCAPE ID1":
        raise SafetyViolation("operator did not arm ID1 endpoint escape")


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/id1_motion/"
        f"id1_endpoint_escape_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    motor: Optional[Motor] = None
    trace: Optional[MotionTrace] = None
    direction_guard: Optional[BaselineBandGuard] = None
    armed = False
    pause_triggered = False
    pause_success: Optional[bool] = None
    stop_confirmation: Optional[bool] = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        with _DeferredClose(bus):
            motor = Motor(bus, MOTOR_ID)
            hardware_lower = _read_value(motor, LOWER_LIMIT_ADDR)
            hardware_upper = _read_value(motor, UPPER_LIMIT_ADDR)
            mode = _read_value(motor, CONTROL_MODE_ADDR)
            speed = _read_value(motor, TARGET_SPEED_ADDR)
            target_before = _read_value(motor, TARGET_POSITION_ADDR)
            if not 0 <= hardware_lower <= hardware_upper <= 2000:
                raise SafetyViolation(
                    f"invalid hardware limits lower={hardware_lower} upper={hardware_upper}"
                )
            initial = motor.read_status()
            _check_status(
                initial,
                lower=hardware_lower,
                upper=hardware_upper,
                max_current_ma=args.max_current_ma,
            )
            logger.info(
                "preflight mode=%d speed=%d target=%d device_limits=[%d,%d] "
                "precheck_single_position=%d",
                mode,
                speed,
                target_before,
                hardware_lower,
                hardware_upper,
                initial.current_position,
            )
            print(
                f"PRECHECK ID1 precheck_single_position={initial.current_position} "
                f"mode={mode} speed={speed} "
                f"device_limits=[{hardware_lower},{hardware_upper}] "
                f"force={initial.force_g}g current={initial.current_mA}mA "
                f"fault=0x{initial.error_code:02X}"
            )
            baseline_count = max(1, int(round(args.baseline_duration * args.baseline_rate)))
            baseline_forces: list[int] = []
            baseline_positions: list[int] = []
            baseline_start = time.monotonic()
            for index in range(baseline_count):
                scheduled = baseline_start + index / args.baseline_rate
                delay = scheduled - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                status = motor.read_status()
                _check_status(
                    status,
                    lower=hardware_lower,
                    upper=hardware_upper,
                    max_current_ma=args.max_current_ma,
                )
                baseline_forces.append(status.force_g)
                baseline_positions.append(status.current_position)
                _log_status("baseline", status, baseline_force=None, target=None)
            try:
                baseline = _build_position_baseline(baseline_positions)
            except SafetyViolation:
                logger.warning(
                    "precheck_single_position=%d baseline_position_histogram=%s "
                    "baseline_band=REJECTED",
                    initial.current_position,
                    dict(sorted(Counter(baseline_positions).items())),
                )
                raise
            logger.info(
                "precheck_single_position=%d baseline_position_histogram=%s "
                "baseline_min=%d baseline_max=%d baseline_span=%d",
                initial.current_position,
                baseline.histogram,
                baseline.minimum,
                baseline.maximum,
                baseline.span,
            )
            escape_step = getattr(args, "escape_step", DEFAULT_ESCAPE_STEP)
            if escape_step not in ALLOWED_ESCAPE_STEPS:
                raise SafetyViolation(
                    f"escape step={escape_step} is not one of {ALLOWED_ESCAPE_STEPS}"
                )
            target = baseline.maximum + escape_step
            print(
                f"BASELINE_POSITION precheck_single_position={initial.current_position} "
                f"histogram={baseline.histogram} min={baseline.minimum} "
                f"max={baseline.maximum} span={baseline.span}"
            )
            if baseline.maximum > LOW_ENDPOINT_GATE:
                raise SafetyViolation(
                    f"baseline_max={baseline.maximum} is not in low-endpoint gate "
                    f"<= {LOW_ENDPOINT_GATE}"
                )
            if target > hardware_upper:
                raise SafetyViolation(f"positive target {target} exceeds device upper limit")
            if target < hardware_lower:
                raise SafetyViolation(f"positive target {target} is below device lower limit")
            force_f0 = statistics.fmean(baseline_forces)
            logger.info(
                "force baseline F0_g=%.3f mean std=%.3f min=%d max=%d",
                force_f0,
                statistics.pstdev(baseline_forces),
                min(baseline_forces),
                max(baseline_forces),
            )
            print(
                f"BASELINE ID1 samples={len(baseline_forces)} F0={force_f0:.3f}g "
                f"min={min(baseline_forces)} max={max(baseline_forces)}"
            )
            _confirm()

            # One setup write and one target write; UartBus/Motor never retries writes.
            setup = motor.write_register(
                CONTROL_MODE_ADDR,
                [
                    SPEED_MODE,
                    0,
                    0,
                    ESCAPE_SPEED_STEPS_PER_SECOND,
                    baseline.maximum,
                ],
            )
            # The combined mode/speed/target=baseline_max write has completed.
            # From this point every abnormal exit must pause before UART close.
            armed = True
            logger.info(
                "ARMED mode=%d speed=%d target=%d setup_actual=%d",
                SPEED_MODE,
                ESCAPE_SPEED_STEPS_PER_SECOND,
                baseline.maximum,
                setup.current_position,
            )
            _check_status(
                setup,
                lower=hardware_lower,
                upper=hardware_upper,
                max_current_ma=args.max_current_ma,
                target=baseline.maximum,
            )
            _log_status(
                "setup", setup, baseline_force=force_f0, target=baseline.maximum
            )

            # t0 is immediately before the single target write. This is the
            # reference for first-motion and first-in-tolerance timestamps.
            trace = MotionTrace(
                baseline_min=baseline.minimum,
                baseline_max=baseline.maximum,
                target=target,
                force_f0=force_f0,
                t0_mono=time.monotonic(),
                t0_wall=datetime.now(timezone.utc).isoformat(),
                samples=[],
            )
            direction_guard = BaselineBandGuard(
                baseline_min=baseline.minimum,
                baseline_max=baseline.maximum,
            )
            logger.info(
                "MOTION_T0 t0=%s baseline_min=%d baseline_max=%d target=%d "
                "delta_steps=%d mode=%d speed=%d",
                trace.t0_wall,
                baseline.minimum,
                baseline.maximum,
                target,
                target - baseline.maximum,
                SPEED_MODE,
                ESCAPE_SPEED_STEPS_PER_SECOND,
            )
            response = motor.write_register(TARGET_POSITION_ADDR, [target])
            response_error = abs(response.current_position - target)
            response_elapsed = trace.observe(response, response_error, source="command")
            _log_status(
                "command_once",
                response,
                baseline_force=force_f0,
                target=target,
                elapsed_s=response_elapsed,
            )
            _check_status(
                response,
                lower=hardware_lower,
                upper=hardware_upper,
                max_current_ma=args.max_current_ma,
                target=target,
                direction_guard=direction_guard,
            )

            deadline = time.monotonic() + args.settle_timeout
            while True:
                status = motor.read_status()
                error = abs(status.current_position - target)
                elapsed = trace.observe(status, error)
                _log_status(
                    "poll",
                    status,
                    baseline_force=force_f0,
                    target=target,
                    elapsed_s=elapsed,
                )
                _check_status(
                    status,
                    lower=hardware_lower,
                    upper=hardware_upper,
                    max_current_ma=args.max_current_ma,
                    target=target,
                    direction_guard=direction_guard,
                )
                if error <= SETTLE_TOLERANCE_STEPS:
                    final = status
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise SafetyViolation(
                        f"target={target} did not settle within {args.settle_timeout:.2f}s"
                    )
                time.sleep(min(args.poll_interval, remaining))

            final_mode = _read_value(motor, CONTROL_MODE_ADDR)
            final_speed = _read_value(motor, TARGET_SPEED_ADDR)
            final_target = _read_value(motor, TARGET_POSITION_ADDR)
            logger.info(
                "final mode=%d speed=%d target=%d actual=%d force_g=%d delta_force_g=%.3f "
                "current_mA=%d temperature_C=%d fault=0x%02X restored=false",
                final_mode,
                final_speed,
                final_target,
                final.current_position,
                final.force_g,
                final.force_g - force_f0,
                final.current_mA,
                final.temperature,
                final.error_code,
            )
            print(
                f"PASS ID1 baseline=[{baseline.minimum},{baseline.maximum}] "
                f"final={final.current_position} target={target} "
                f"force={final.force_g}g delta_force={final.force_g - force_f0:.3f}g "
                f"mode={final_mode} speed={final_speed} restored=false"
            )
            logger.info(
                trace.result_line(
                    decision="PASS",
                    pause_triggered=False,
                    pause_success=False,
                    stop_confirmation=True,
                )
            )
        return 0
    except Exception as exc:
        if armed:
            pause_triggered = True
            pause_ok, pause_response = _pause_once(motor)
            pause_success = pause_ok
            if trace is not None and pause_response is not None:
                trace.set_pause_status(pause_response)
            if pause_ok and motor is not None:
                try:
                    stop_confirmation, post_pause_status = _confirm_stopped(motor)
                    if trace is not None and post_pause_status is not None:
                        trace.set_post_pause_status(post_pause_status)
                except Exception as confirm_exc:
                    message = (
                        "HARD STOP REQUIRED: post-pause read-only confirmation failed; "
                        "use the hardware emergency stop or remove motor power"
                    )
                    logger.critical("%s: %s", message, confirm_exc)
                    print(message, file=sys.stderr)
            if trace is not None:
                logger.info(
                    trace.result_line(
                        decision="FAIL",
                        pause_triggered=pause_triggered,
                        pause_success=pause_success,
                        stop_confirmation=stop_confirmation,
                    )
                )
        logger.error("ID1 endpoint escape STOP: %s", exc)
        print(f"STOP ID1 endpoint escape: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ID1 低端单次 +5/+10 step 安全脱离")
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--max-current-ma", type=int, required=True)
    parser.add_argument(
        "--escape-step",
        type=int,
        choices=ALLOWED_ESCAPE_STEPS,
        default=DEFAULT_ESCAPE_STEP,
        help="endpoint 正方向固定步长，只允许 5 或 10 step",
    )
    parser.add_argument("--baseline-duration", type=float, default=DEFAULT_BASELINE_DURATION)
    parser.add_argument("--baseline-rate", type=float, default=DEFAULT_BASELINE_RATE)
    parser.add_argument("--settle-timeout", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.02)
    parser.add_argument("--log", help="独立日志路径；已有文件不会覆盖")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.baseline_duration <= 0 or args.baseline_rate <= 0:
        parser.error("timeout、baseline-duration、baseline-rate 必须大于 0")
    if args.settle_timeout <= 0 or args.poll_interval <= 0:
        parser.error("settle-timeout、poll-interval 必须大于 0")
    if args.retries < 0 or args.max_current_ma <= 0:
        parser.error("retries 不能小于 0，max-current-ma 必须大于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
