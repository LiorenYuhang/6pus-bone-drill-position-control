#!/usr/bin/env python3
"""Stage M2-B: move ID1 once while ID2 remains strictly read-only."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, field
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
    PositionBaseline,
    SafetyViolation,
    _build_position_baseline,
    _check_status,
    _stop_confirmation_metrics,
)
from uart_bus import UartBus  # noqa: E402


TARGET_ID = 1
OBSERVER_ID = 2
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

logger = logging.getLogger("safe_m2b_id1_only")


@dataclass(frozen=True)
class ConfigSnapshot:
    mode: int
    speed: int
    target: int


@dataclass(frozen=True)
class MotorBaseline:
    position: PositionBaseline
    force_f0: float
    current_max_mA: int


@dataclass
class ObserverTrace:
    baseline: MotorBaseline
    statuses: list[StatusFeedback] = field(default_factory=list)

    def observe(self, status: StatusFeedback, *, source: str) -> None:
        self.statuses.append(status)
        logger.info(
            "ID2_READ_ONLY source=%s actual=%d current_mA=%d force_g=%d "
            "delta_force_g=%.3f temperature_C=%d fault=0x%02X",
            source,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.force_g - self.baseline.force_f0,
            status.temperature,
            status.error_code,
        )
        if status.error_code != 0:
            raise SafetyViolation(f"ID2 observer fault=0x{status.error_code:02X}")
        if not (
            self.baseline.position.minimum
            <= status.current_position
            <= self.baseline.position.maximum
        ):
            raise SafetyViolation(
                "ID2 observer position left baseline band: "
                f"baseline=[{self.baseline.position.minimum},"
                f"{self.baseline.position.maximum}] actual={status.current_position}"
            )
        if status.current_mA > self.baseline.current_max_mA:
            raise SafetyViolation(
                "ID2 observer current exceeded baseline maximum: "
                f"baseline_max={self.baseline.current_max_mA}mA "
                f"actual={status.current_mA}mA"
            )

    def summary(self) -> str:
        if not self.statuses:
            return "id2_samples=0"
        positions = [status.current_position for status in self.statuses]
        currents = [status.current_mA for status in self.statuses]
        forces = [status.force_g for status in self.statuses]
        faults = sorted({status.error_code for status in self.statuses})
        return (
            f"id2_samples={len(self.statuses)} "
            f"id2_actual_min={min(positions)} id2_actual_max={max(positions)} "
            f"id2_current_max_mA={max(currents)} "
            f"id2_force_min_g={min(forces)} id2_force_max_g={max(forces)} "
            f"id2_faults={','.join(f'0x{value:02X}' for value in faults)}"
        )


class Id1OnlyAuditedBus:
    """Audit every outgoing frame and block writes not addressed to ID1."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.write_frames: list[Frame] = []
        self.read_counts: Counter[int] = Counter()

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if expected_id is not None and expected_id != parsed.motor_id:
            raise SafetyViolation(
                f"request expected_id={expected_id} does not match destination ID={parsed.motor_id}"
            )
        if parsed.command == CMD_WR_REGISTER:
            if parsed.motor_id != TARGET_ID:
                logger.critical(
                    "WRITE_BLOCKED destination_id=%d register=0x%04X",
                    parsed.motor_id,
                    parsed.register_addr,
                )
                raise SafetyViolation(
                    f"write destination ID={parsed.motor_id} is forbidden; only ID1 is writable"
                )
            self.write_frames.append(parsed)
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=1 register=0x%04X "
                "data=%s verified=true",
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
                f"outgoing command 0x{parsed.command:02X} is outside M2-B allowlist"
            )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        destinations = [frame.motor_id for frame in self.write_frames]
        registers = [f"0x{frame.register_addr:04X}" for frame in self.write_frames]
        return (
            f"ACCESS_SUMMARY write_count={len(self.write_frames)} "
            f"write_destinations={destinations} write_registers={registers} "
            f"id1_read_count={self.read_counts[TARGET_ID]} "
            f"id2_read_count={self.read_counts[OBSERVER_ID]} id2_write_count=0"
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


def _check_config_unchanged(
    expected: ConfigSnapshot, actual: ConfigSnapshot, *, source: str
) -> None:
    unchanged = actual == expected
    logger.info(
        "ID2_CONFIG_READ_ONLY source=%s unchanged=%s mode=%d speed=%d target=%d",
        source,
        unchanged,
        actual.mode,
        actual.speed,
        actual.target,
    )
    if not unchanged:
        raise SafetyViolation(
            f"ID2 configuration changed during {source}: before={expected} after={actual}"
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


def _collect_baselines(
    motor1: Motor,
    motor2: Motor,
    *,
    limits1: tuple[int, int],
    limits2: tuple[int, int],
) -> tuple[MotorBaseline, MotorBaseline]:
    sample_count = max(1, int(round(BASELINE_DURATION_S * BASELINE_RATE_HZ)))
    positions1: list[int] = []
    positions2: list[int] = []
    forces1: list[int] = []
    forces2: list[int] = []
    currents1: list[int] = []
    currents2: list[int] = []
    started = time.monotonic()
    for index in range(sample_count):
        scheduled = started + index / BASELINE_RATE_HZ
        delay = scheduled - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        status1 = motor1.read_status()
        status2 = motor2.read_status()
        _check_status(
            status1,
            lower=limits1[0],
            upper=limits1[1],
            max_current_ma=MAX_CURRENT_MA,
        )
        _check_status(
            status2,
            lower=limits2[0],
            upper=limits2[1],
            max_current_ma=MAX_CURRENT_MA,
        )
        _log_status(TARGET_ID, "baseline", status1)
        _log_status(OBSERVER_ID, "baseline_read_only", status2)
        positions1.append(status1.current_position)
        positions2.append(status2.current_position)
        forces1.append(status1.force_g)
        forces2.append(status2.force_g)
        currents1.append(status1.current_mA)
        currents2.append(status2.current_mA)
    return (
        MotorBaseline(
            position=_build_position_baseline(positions1),
            force_f0=statistics.fmean(forces1),
            current_max_mA=max(currents1),
        ),
        MotorBaseline(
            position=_build_position_baseline(positions2),
            force_f0=statistics.fmean(forces2),
            current_max_mA=max(currents2),
        ),
    )


def _pause_id1_once(motor1: Optional[Motor]) -> tuple[bool, Optional[StatusFeedback]]:
    if motor1 is None:
        return False, None
    try:
        response = motor1.write_register(PAUSE_ADDR, [1])
        logger.warning(
            "ID1_PAUSE sent_once=true destination_id=1 actual=%d current_mA=%d "
            "force_g=%d temperature_C=%d fault=0x%02X",
            response.current_position,
            response.current_mA,
            response.force_g,
            response.temperature,
            response.error_code,
        )
        return True, response
    except Exception as exc:
        message = (
            "HARD STOP REQUIRED: ID1 pause command failed; use the hardware "
            "emergency stop or remove motor power immediately"
        )
        logger.critical("%s: %s", message, exc)
        print(message, file=sys.stderr)
        return False, None


def _confirm_both_stopped(
    motor1: Motor,
    motor2: Motor,
    observer: ObserverTrace,
) -> tuple[bool, Optional[StatusFeedback], Optional[StatusFeedback]]:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses1: list[StatusFeedback] = []
    statuses2: list[StatusFeedback] = []
    while True:
        status1 = motor1.read_status()
        status2 = motor2.read_status()
        statuses1.append(status1)
        statuses2.append(status2)
        _log_status(TARGET_ID, "post_pause_read_only", status1)
        try:
            observer.observe(status2, source="post_pause_read_only")
        except SafetyViolation as exc:
            logger.error("ID2 post-pause read-only check failed: %s", exc)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    stable1, metrics1 = _stop_confirmation_metrics(statuses1)
    stable2, metrics2 = _stop_confirmation_metrics(statuses2)
    id2_in_band = all(
        observer.baseline.position.minimum
        <= status.current_position
        <= observer.baseline.position.maximum
        and status.current_mA <= observer.baseline.current_max_mA
        and status.error_code == 0
        for status in statuses2
    )
    stable = stable1 and stable2 and id2_in_band
    logger.info(
        "M2B_STOP_CONFIRMATION stable=%s id1_stable=%s id1_reason=%s "
        "id2_stable=%s id2_reason=%s id2_in_baseline_band=%s",
        stable,
        stable1,
        metrics1["reason"],
        stable2,
        metrics2["reason"],
        id2_in_band,
    )
    if not stable:
        message = (
            "HARD STOP REQUIRED: M2-B stop confirmation failed; use the hardware "
            "emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return (
        stable,
        statuses1[-1] if statuses1 else None,
        statuses2[-1] if statuses2 else None,
    )


def _confirm_operator() -> None:
    answer = input(
        "确认仅连接并固定 ID1+ID2、只允许 ID1 运动、ID2 严格只读、急停可用；"
        "输入 M2-B MOVE ID1 ONLY："
    ).strip()
    if answer != "M2-B MOVE ID1 ONLY":
        raise SafetyViolation("operator did not arm Stage M2-B")


def _hard_stop_for_confirmation_exception(exc: Exception) -> None:
    message = (
        "HARD STOP REQUIRED: M2-B post-pause read-only confirmation failed; "
        "use the hardware emergency stop or remove motor power"
    )
    logger.critical("%s: %s", message, exc)
    print(message, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m2/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m2b_id1_only.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[Id1OnlyAuditedBus] = None
    motor1: Optional[Motor] = None
    motor2: Optional[Motor] = None
    observer: Optional[ObserverTrace] = None
    trace: Optional[MotionTrace] = None
    armed = False
    pause_attempted = False
    pause_success: Optional[bool] = None
    confirmation_attempted = False
    stop_confirmation: Optional[bool] = None
    positive_motion = False
    final_status1: Optional[StatusFeedback] = None
    id2_config_unchanged: Optional[bool] = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = Id1OnlyAuditedBus(bus)
        motor1 = Motor(audited_bus, TARGET_ID)
        motor2 = Motor(audited_bus, OBSERVER_ID)

        lower1 = _read_value(motor1, LOWER_LIMIT_ADDR)
        upper1 = _read_value(motor1, UPPER_LIMIT_ADDR)
        lower2 = _read_value(motor2, LOWER_LIMIT_ADDR)
        upper2 = _read_value(motor2, UPPER_LIMIT_ADDR)
        _validate_limits(TARGET_ID, lower1, upper1)
        _validate_limits(OBSERVER_ID, lower2, upper2)
        config1_before = _read_config(motor1)
        config2_before = _read_config(motor2)
        initial1 = motor1.read_status()
        initial2 = motor2.read_status()
        _check_status(
            initial1,
            lower=lower1,
            upper=upper1,
            max_current_ma=MAX_CURRENT_MA,
        )
        _check_status(
            initial2,
            lower=lower2,
            upper=upper2,
            max_current_ma=MAX_CURRENT_MA,
        )
        _log_status(TARGET_ID, "preflight", initial1)
        _log_status(OBSERVER_ID, "preflight_read_only", initial2)

        baseline1, baseline2 = _collect_baselines(
            motor1,
            motor2,
            limits1=(lower1, upper1),
            limits2=(lower2, upper2),
        )
        observer = ObserverTrace(baseline=baseline2)
        target = baseline1.position.maximum + MOVE_STEPS
        effective_upper1 = min(upper1, SOFTWARE_UPPER_LIMIT)
        if target < lower1 or target > effective_upper1:
            raise SafetyViolation(
                f"ID1 target={target} outside allowed [{lower1},{effective_upper1}]"
            )
        logger.info(
            "M2B_PRECHECK ID1_config=%s ID2_config=%s ID1_limits=[%d,%d] "
            "ID2_limits=[%d,%d] ID1_baseline=%s ID2_baseline=%s "
            "ID1_force_f0=%.3f ID2_force_f0=%.3f ID2_baseline_current_max_mA=%d "
            "target=%d delta_steps=%d speed=%d timeout_s=%.3f max_current_mA=%d",
            config1_before,
            config2_before,
            lower1,
            upper1,
            lower2,
            upper2,
            baseline1.position,
            baseline2.position,
            baseline1.force_f0,
            baseline2.force_f0,
            baseline2.current_max_mA,
            target,
            MOVE_STEPS,
            SPEED_STEPS_PER_SECOND,
            MOTION_TIMEOUT_S,
            MAX_CURRENT_MA,
        )
        print(
            f"M2-B PRECHECK ID1 baseline=[{baseline1.position.minimum},"
            f"{baseline1.position.maximum}] target={target}; "
            f"ID2 baseline=[{baseline2.position.minimum},{baseline2.position.maximum}] "
            f"current_baseline_max={baseline2.current_max_mA}mA read_only=true"
        )
        _confirm_operator()

        logger.warning(
            "M2B_ARMING allowed_write_destination=1 ID2_write_forbidden=true"
        )
        armed = True
        setup = motor1.write_register(
            CONTROL_MODE_ADDR,
            [SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, baseline1.position.maximum],
        )
        _check_status(
            setup,
            lower=lower1,
            upper=upper1,
            max_current_ma=MAX_CURRENT_MA,
            target=baseline1.position.maximum,
        )
        _log_status(
            TARGET_ID,
            "setup_write_response",
            setup,
            force_f0=baseline1.force_f0,
            target=baseline1.position.maximum,
        )
        _check_config_unchanged(
            config2_before, _read_config(motor2), source="after_id1_setup_write"
        )
        observer.observe(motor2.read_status(), source="after_id1_setup_write")

        trace = MotionTrace(
            baseline_min=baseline1.position.minimum,
            baseline_max=baseline1.position.maximum,
            target=target,
            force_f0=baseline1.force_f0,
            t0_mono=time.monotonic(),
            t0_wall=datetime.now(timezone.utc).isoformat(),
            samples=[],
        )
        direction_guard = BaselineBandGuard(
            baseline_min=baseline1.position.minimum,
            baseline_max=baseline1.position.maximum,
        )
        logger.info(
            "M2B_MOTION_T0 t0=%s ID1_target=%d ID2_read_only=true",
            trace.t0_wall,
            target,
        )
        command_response = motor1.write_register(TARGET_POSITION_ADDR, [target])
        command_error = abs(command_response.current_position - target)
        elapsed = trace.observe(command_response, command_error, source="command")
        _check_status(
            command_response,
            lower=lower1,
            upper=upper1,
            max_current_ma=MAX_CURRENT_MA,
            target=target,
            direction_guard=direction_guard,
        )
        _log_status(
            TARGET_ID,
            "target_write_response",
            command_response,
            force_f0=baseline1.force_f0,
            target=target,
            elapsed_s=elapsed,
        )
        _check_config_unchanged(
            config2_before, _read_config(motor2), source="after_id1_target_write"
        )
        observer.observe(motor2.read_status(), source="after_id1_target_write")

        deadline = trace.t0_mono + MOTION_TIMEOUT_S
        final_status1 = command_response
        while True:
            status1 = motor1.read_status()
            status2 = motor2.read_status()
            error = abs(status1.current_position - target)
            elapsed = trace.observe(status1, error)
            _check_status(
                status1,
                lower=lower1,
                upper=upper1,
                max_current_ma=MAX_CURRENT_MA,
                target=target,
                direction_guard=direction_guard,
            )
            _log_status(
                TARGET_ID,
                "motion_poll",
                status1,
                force_f0=baseline1.force_f0,
                target=target,
                elapsed_s=elapsed,
            )
            observer.observe(status2, source="motion_poll")
            final_status1 = status1
            positive_motion = trace.first_motion_mono is not None
            if positive_motion and error <= INTEGRATION_TOLERANCE_STEPS:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SafetyViolation(
                    f"ID1 target={target} did not reach integration tolerance "
                    f"+/-{INTEGRATION_TOLERANCE_STEPS} within {MOTION_TIMEOUT_S:.2f}s"
                )
            time.sleep(min(POLL_INTERVAL_S, remaining))

        pause_attempted = True
        pause_success, pause_response = _pause_id1_once(motor1)
        if trace is not None and pause_response is not None:
            trace.set_pause_status(pause_response)
        if not pause_success:
            raise SafetyViolation("ID1 completion pause failed")
        confirmation_attempted = True
        stop_confirmation, post1, _post2 = _confirm_both_stopped(
            motor1, motor2, observer
        )
        if trace is not None and post1 is not None:
            trace.set_post_pause_status(post1)
        if not stop_confirmation or post1 is None:
            raise SafetyViolation("M2-B stop confirmation failed")
        final_status1 = post1
        final_error = abs(final_status1.current_position - target)
        if final_error > INTEGRATION_TOLERANCE_STEPS:
            raise SafetyViolation(
                f"ID1 post-pause final_error={final_error} exceeds "
                f"{INTEGRATION_TOLERANCE_STEPS} step"
            )
        config2_after = _read_config(motor2)
        _check_config_unchanged(config2_before, config2_after, source="final")
        id2_config_unchanged = True
        logger.info(
            "M2B_RESULT decision=PASS positive_motion=%s ID1_target=%d "
            "ID1_final_actual=%d ID1_final_error=%d strict_tolerance_pass=%s "
            "integration_tolerance_pass=true ID2_config_unchanged=true %s "
            "pause_success=%s stop_confirmation=%s",
            positive_motion,
            target,
            final_status1.current_position,
            final_error,
            final_error <= STRICT_TOLERANCE_STEPS,
            observer.summary(),
            pause_success,
            stop_confirmation,
        )
        logger.info(
            trace.result_line(
                decision="PASS",
                pause_triggered=True,
                pause_success=pause_success,
                stop_confirmation=stop_confirmation,
            )
        )
        print(
            f"PASS M2-B ID1 target={target} final={final_status1.current_position} "
            f"error={final_error} ID2_read_only=true ID2_config_unchanged=true "
            f"pause_success={pause_success} stop_confirmation={stop_confirmation}"
        )
        return 0
    except Exception as exc:
        if trace is not None:
            positive_motion = trace.first_motion_mono is not None
        if armed and not pause_attempted:
            pause_attempted = True
            pause_success, pause_response = _pause_id1_once(motor1)
            if trace is not None and pause_response is not None:
                trace.set_pause_status(pause_response)
        if (
            armed
            and pause_success
            and not confirmation_attempted
            and motor1 is not None
            and motor2 is not None
            and observer is not None
        ):
            confirmation_attempted = True
            try:
                stop_confirmation, post1, _post2 = _confirm_both_stopped(
                    motor1, motor2, observer
                )
                if trace is not None and post1 is not None:
                    trace.set_post_pause_status(post1)
            except Exception as confirmation_exc:
                _hard_stop_for_confirmation_exception(confirmation_exc)
                stop_confirmation = False
        if trace is not None:
            logger.info(
                trace.result_line(
                    decision="FAIL",
                    pause_triggered=pause_attempted,
                    pause_success=pause_success,
                    stop_confirmation=stop_confirmation,
                )
            )
        logger.error(
            "M2B_RESULT decision=FAIL positive_motion=%s ID2_config_unchanged=%s %s "
            "pause_attempted=%s pause_success=%s stop_confirmation=%s reason=%s",
            positive_motion,
            id2_config_unchanged,
            observer.summary() if observer is not None else "id2_samples=0",
            pause_attempted,
            pause_success,
            stop_confirmation,
            exc,
        )
        print(f"STOP M2-B: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stage M2-B: ID1 fixed +10 step motion with ID2 strictly read-only"
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
