#!/usr/bin/env python3
"""Stage M4-B: move one addressed motor while three peers stay read-only."""

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
    SafetyViolation,
    _check_status,
    _stop_confirmation_metrics,
)
from tools.safe_m2d_joint_id1_id2 import (  # noqa: E402
    M2dPositionBaseline,
    _build_m2d_position_baseline,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (1, 2, 3, 4)
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
PAUSE_TRIGGER_TOLERANCE_STEPS = 2
FINAL_ACCEPTANCE_TOLERANCE_STEPS = 3
STRICT_TOLERANCE_STEPS = 1
SOFTWARE_UPPER_LIMIT = 1800
ALLOWED_WRITE_REGISTERS = {
    CONTROL_MODE_ADDR,
    TARGET_POSITION_ADDR,
    PAUSE_ADDR,
}

logger = logging.getLogger("safe_m4b_one_motor")


@dataclass(frozen=True)
class ConfigSnapshot:
    mode: int
    speed: int
    target: int


@dataclass(frozen=True)
class TargetBaseline:
    reference: int
    safety_min: int
    safety_max: int
    histogram: dict[int, int]


@dataclass(frozen=True)
class MotorBaseline:
    position: M2dPositionBaseline
    force_f0: float
    current_max_mA: int
    target_baseline: TargetBaseline


@dataclass
class ObserverTrace:
    motor_id: int
    baseline: MotorBaseline
    statuses: list[StatusFeedback] = field(default_factory=list)

    def observe(self, status: StatusFeedback, *, source: str) -> None:
        self.statuses.append(status)
        reference = self.baseline.position.reference
        logger.info(
            "ID%d_READ_ONLY source=%s baseline_reference=%d actual=%d "
            "current_mA=%d force_g=%d delta_force_g=%.3f "
            "temperature_C=%d fault=0x%02X",
            self.motor_id,
            source,
            reference,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.force_g - self.baseline.force_f0,
            status.temperature,
            status.error_code,
        )
        if status.error_code != 0:
            raise SafetyViolation(
                f"ID{self.motor_id} observer fault=0x{status.error_code:02X}"
            )
        if abs(status.current_position - reference) > 1:
            raise SafetyViolation(
                f"ID{self.motor_id} observer position left reference +/-1 band: "
                f"baseline_reference={reference} actual={status.current_position}"
            )
        if status.current_mA > self.baseline.current_max_mA:
            raise SafetyViolation(
                f"ID{self.motor_id} observer current exceeded baseline maximum: "
                f"baseline_max={self.baseline.current_max_mA}mA "
                f"actual={status.current_mA}mA"
            )
        target_min = self.baseline.target_baseline.safety_min
        target_max = self.baseline.target_baseline.safety_max
        if not (target_min <= status.target_position <= target_max):
            raise SafetyViolation(
                f"ID{self.motor_id} observer target={status.target_position} "
                f"outside baseline target safety band [{target_min},{target_max}] "
                f"during {source}"
            )

    def summary(self) -> str:
        prefix = f"id{self.motor_id}"
        if not self.statuses:
            return f"{prefix}_samples=0"
        positions = [status.current_position for status in self.statuses]
        currents = [status.current_mA for status in self.statuses]
        forces = [status.force_g for status in self.statuses]
        faults = sorted({status.error_code for status in self.statuses})
        return (
            f"{prefix}_samples={len(self.statuses)} "
            f"{prefix}_actual_min={min(positions)} "
            f"{prefix}_actual_max={max(positions)} "
            f"{prefix}_current_max_mA={max(currents)} "
            f"{prefix}_force_min_g={min(forces)} "
            f"{prefix}_force_max_g={max(forces)} "
            f"{prefix}_faults="
            f"{','.join(f'0x{value:02X}' for value in faults)}"
        )


class M4bAuditedBus:
    """Allow reads for ID1~ID4 and writes only to the selected moving ID."""

    def __init__(self, bus: UartBus, moving_id: int) -> None:
        if moving_id not in MOTOR_IDS:
            raise SafetyViolation(f"moving ID{moving_id} is outside M4-B scope")
        self._bus = bus
        self.moving_id = moving_id
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
                f"destination ID={parsed.motor_id} is outside M4-B ID1~ID4 scope"
            )
        if parsed.command == CMD_WR_REGISTER:
            if parsed.motor_id != self.moving_id:
                logger.critical(
                    "WRITE_BLOCKED destination_id=%d moving_id=%d register=0x%04X",
                    parsed.motor_id,
                    self.moving_id,
                    parsed.register_addr,
                )
                raise SafetyViolation(
                    f"write destination ID={parsed.motor_id} is forbidden; "
                    f"only moving ID{self.moving_id} is writable"
                )
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                raise SafetyViolation(
                    f"write register 0x{parsed.register_addr:04X} is forbidden in M4-B"
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
                f"outgoing command 0x{parsed.command:02X} is outside M4-B allowlist"
            )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        destinations = [frame.motor_id for frame in self.write_frames]
        registers = [f"0x{frame.register_addr:04X}" for frame in self.write_frames]
        read_counts = " ".join(
            f"id{motor_id}_read_count={self.read_counts[motor_id]}"
            for motor_id in MOTOR_IDS
        )
        observer_write_count = sum(
            frame.motor_id != self.moving_id for frame in self.write_frames
        )
        return (
            f"ACCESS_SUMMARY moving_id={self.moving_id} "
            f"write_count={len(self.write_frames)} "
            f"write_destinations={destinations} write_registers={registers} "
            f"{read_counts} observer_write_count={observer_write_count}"
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


def _build_target_baseline(targets: list[int]) -> TargetBaseline:
    if not targets:
        raise SafetyViolation("no target samples collected for baseline")
    counts = Counter(targets)
    reference = counts.most_common(1)[0][0]
    safety_min = min(targets)
    safety_max = max(targets)
    histogram = dict(sorted(counts.items()))
    return TargetBaseline(
        reference=reference,
        safety_min=safety_min,
        safety_max=safety_max,
        histogram=histogram,
    )


def _check_observer_config(
    motor_id: int,
    expected: ConfigSnapshot,
    actual: ConfigSnapshot,
    baseline: MotorBaseline,
    *,
    source: str,
) -> None:
    mode_unchanged = actual.mode == expected.mode
    speed_unchanged = actual.speed == expected.speed
    target_min = baseline.target_baseline.safety_min
    target_max = baseline.target_baseline.safety_max
    target_in_band = target_min <= actual.target <= target_max
    unchanged = mode_unchanged and speed_unchanged and target_in_band
    logger.info(
        "ID%d_CONFIG_READ_ONLY source=%s unchanged=%s mode=%d speed=%d target=%d "
        "expected_mode=%d expected_speed=%d target_band=[%d,%d]",
        motor_id,
        source,
        unchanged,
        actual.mode,
        actual.speed,
        actual.target,
        expected.mode,
        expected.speed,
        target_min,
        target_max,
    )
    if not mode_unchanged:
        raise SafetyViolation(
            f"ID{motor_id} observer mode changed during {source}: "
            f"before={expected.mode} after={actual.mode}"
        )
    if not speed_unchanged:
        raise SafetyViolation(
            f"ID{motor_id} observer speed changed during {source}: "
            f"before={expected.speed} after={actual.speed}"
        )
    if not target_in_band:
        raise SafetyViolation(
            f"ID{motor_id} observer target={actual.target} outside baseline "
            f"target safety band [{target_min},{target_max}] during {source}"
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
    motors: dict[int, Motor],
    limits: dict[int, tuple[int, int]],
    configs_before: Optional[dict[int, ConfigSnapshot]] = None,
) -> dict[int, MotorBaseline]:
    sample_count = max(1, int(round(BASELINE_DURATION_S * BASELINE_RATE_HZ)))
    positions = {motor_id: [] for motor_id in MOTOR_IDS}
    targets = {motor_id: [] for motor_id in MOTOR_IDS}
    forces = {motor_id: [] for motor_id in MOTOR_IDS}
    currents = {motor_id: [] for motor_id in MOTOR_IDS}
    if configs_before:
        for motor_id in MOTOR_IDS:
            if motor_id in configs_before:
                targets[motor_id].append(configs_before[motor_id].target)
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
            targets[motor_id].append(status.target_position)
            forces[motor_id].append(status.force_g)
            currents[motor_id].append(status.current_mA)
    return {
        motor_id: MotorBaseline(
            position=_build_m2d_position_baseline(positions[motor_id]),
            force_f0=statistics.fmean(forces[motor_id]),
            current_max_mA=max(currents[motor_id]),
            target_baseline=_build_target_baseline(targets[motor_id]),
        )
        for motor_id in MOTOR_IDS
    }


def _pause_moving_once(
    motor: Optional[Motor], moving_id: int
) -> tuple[bool, Optional[StatusFeedback]]:
    if motor is None:
        return False, None
    try:
        response = motor.write_register(PAUSE_ADDR, [1])
        logger.warning(
            "ID%d_PAUSE sent_once=true destination_id=%d actual=%d "
            "current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
            moving_id,
            moving_id,
            response.current_position,
            response.current_mA,
            response.force_g,
            response.temperature,
            response.error_code,
        )
        return True, response
    except Exception as exc:
        message = (
            f"HARD STOP REQUIRED: ID{moving_id} pause command failed; use the "
            "hardware emergency stop or remove motor power immediately"
        )
        logger.critical("%s: %s", message, exc)
        print(message, file=sys.stderr)
        return False, None


def _confirm_all_stopped(
    motors: dict[int, Motor],
    moving_id: int,
    observers: dict[int, ObserverTrace],
    limits: dict[int, tuple[int, int]],
) -> tuple[bool, dict[int, StatusFeedback]]:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses = {motor_id: [] for motor_id in MOTOR_IDS}
    observer_errors: list[str] = []
    while True:
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            statuses[motor_id].append(status)
            lower, upper = limits[motor_id]
            try:
                _check_status(
                    status,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                )
                if motor_id == moving_id:
                    _log_status(motor_id, "post_pause_read_only", status)
                else:
                    observers[motor_id].observe(
                        status, source="post_pause_read_only"
                    )
            except SafetyViolation as exc:
                observer_errors.append(str(exc))
                logger.error("ID%d post-pause check failed: %s", motor_id, exc)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))

    stable_by_id: dict[int, bool] = {}
    reasons: dict[int, str] = {}
    for motor_id in MOTOR_IDS:
        stable_by_id[motor_id], metrics = _stop_confirmation_metrics(
            statuses[motor_id]
        )
        reasons[motor_id] = str(metrics["reason"])
    stable = all(stable_by_id.values()) and not observer_errors
    logger.info(
        "M4B_STOP_CONFIRMATION stable=%s moving_id=%d "
        "id1_stable=%s id1_reason=%s id2_stable=%s id2_reason=%s "
        "id3_stable=%s id3_reason=%s id4_stable=%s id4_reason=%s "
        "observer_errors=%s",
        stable,
        moving_id,
        stable_by_id[1],
        reasons[1],
        stable_by_id[2],
        reasons[2],
        stable_by_id[3],
        reasons[3],
        stable_by_id[4],
        reasons[4],
        observer_errors,
    )
    if not stable:
        message = (
            "HARD STOP REQUIRED: M4-B stop confirmation failed; use the "
            "hardware emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return stable, {
        motor_id: statuses[motor_id][-1] for motor_id in MOTOR_IDS
    }


def _confirm_operator(moving_id: int) -> None:
    answer = input(
        f"确认仅连接并固定 ID1~ID4、仅 ID{moving_id} 正方向区域安全、"
        f"急停可用；输入 M4-B MOVE ID{moving_id}："
    ).strip()
    if answer != f"M4-B MOVE ID{moving_id}":
        raise SafetyViolation("operator did not arm Stage M4-B")


def _hard_stop_for_confirmation_exception(exc: Exception) -> None:
    message = (
        "HARD STOP REQUIRED: M4-B post-pause read-only confirmation failed; "
        "use the hardware emergency stop or remove motor power"
    )
    logger.critical("%s: %s", message, exc)
    print(message, file=sys.stderr)


def _observer_summary(observers: dict[int, ObserverTrace]) -> str:
    return " ".join(observers[motor_id].summary() for motor_id in sorted(observers))


def run(args: argparse.Namespace) -> int:
    moving_id = args.moving_id
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m4/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m4b_id{moving_id}_only.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[M4bAuditedBus] = None
    motors: dict[int, Motor] = {}
    observers: dict[int, ObserverTrace] = {}
    limits: dict[int, tuple[int, int]] = {}
    configs_before: dict[int, ConfigSnapshot] = {}
    trace: Optional[MotionTrace] = None
    armed = False
    pause_attempted = False
    pause_success: Optional[bool] = None
    confirmation_attempted = False
    stop_confirmation: Optional[bool] = None
    positive_motion = False
    target: Optional[int] = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = M4bAuditedBus(bus, moving_id)
        motors = {
            motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS
        }

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

        baselines = _collect_baselines(motors, limits, configs_before)
        moving_baseline = baselines[moving_id]
        target = moving_baseline.position.reference + MOVE_STEPS
        lower, upper = limits[moving_id]
        effective_upper = min(upper, SOFTWARE_UPPER_LIMIT)
        if not lower <= target <= effective_upper:
            raise SafetyViolation(
                f"ID{moving_id} target={target} outside allowed "
                f"[{lower},{effective_upper}]"
            )
        observers = {
            motor_id: ObserverTrace(motor_id, baselines[motor_id])
            for motor_id in MOTOR_IDS
            if motor_id != moving_id
        }
        for motor_id in MOTOR_IDS:
            baseline = baselines[motor_id]
            logger.info(
                "M4B_PRECHECK ID%d moving=%s config=%s limits=[%d,%d] "
                "baseline_reference=%d baseline_safety_min=%d "
                "baseline_safety_max=%d baseline_histogram=%s "
                "target_reference=%d target_safety_min=%d "
                "target_safety_max=%d target_histogram=%s "
                "force_f0=%.3f baseline_current_max_mA=%d target=%s "
                "speed=%d timeout_s=%.3f max_current_mA=%d",
                motor_id,
                motor_id == moving_id,
                configs_before[motor_id],
                limits[motor_id][0],
                limits[motor_id][1],
                baseline.position.reference,
                baseline.position.safety_min,
                baseline.position.safety_max,
                baseline.position.histogram,
                baseline.target_baseline.reference,
                baseline.target_baseline.safety_min,
                baseline.target_baseline.safety_max,
                baseline.target_baseline.histogram,
                baseline.force_f0,
                baseline.current_max_mA,
                target if motor_id == moving_id else "read_only",
                SPEED_STEPS_PER_SECOND,
                MOTION_TIMEOUT_S,
                MAX_CURRENT_MA,
            )
        print(
            f"M4-B PRECHECK moving_id={moving_id} "
            f"baseline_reference={moving_baseline.position.reference} "
            f"baseline_safety=[{moving_baseline.position.safety_min},"
            f"{moving_baseline.position.safety_max}] target={target} "
            f"read_only_ids={sorted(observers)}"
        )
        _confirm_operator(moving_id)

        logger.warning(
            "M4B_ARMING moving_id=%d read_only_ids=%s "
            "allowed_write_registers=[0x0025,0x0029,0x001A]",
            moving_id,
            sorted(observers),
        )
        armed = True
        setup = motors[moving_id].write_register(
            CONTROL_MODE_ADDR,
            [
                SPEED_MODE,
                0,
                0,
                SPEED_STEPS_PER_SECOND,
                moving_baseline.position.safety_max,
            ],
        )
        _check_status(
            setup,
            lower=lower,
            upper=upper,
            max_current_ma=MAX_CURRENT_MA,
            target=moving_baseline.position.safety_max,
        )
        _log_status(
            moving_id,
            "setup_write_response",
            setup,
            force_f0=moving_baseline.force_f0,
            target=moving_baseline.position.safety_max,
        )
        for observer_id, observer in observers.items():
            observer.observe(
                motors[observer_id].read_status(), source="after_setup"
            )
            _check_observer_config(
                observer_id,
                configs_before[observer_id],
                _read_config(motors[observer_id]),
                baselines[observer_id],
                source="after_setup",
            )

        t0_mono = time.monotonic()
        t0_wall = datetime.now(timezone.utc).isoformat()
        trace = MotionTrace(
            baseline_min=moving_baseline.position.safety_min,
            baseline_max=moving_baseline.position.safety_max,
            target=target,
            force_f0=moving_baseline.force_f0,
            t0_mono=t0_mono,
            t0_wall=t0_wall,
            samples=[],
        )
        direction_guard = BaselineBandGuard(
            baseline_min=moving_baseline.position.safety_min,
            baseline_max=moving_baseline.position.safety_max,
        )
        response = motors[moving_id].write_register(TARGET_POSITION_ADDR, [target])
        error = abs(response.current_position - target)
        elapsed = trace.observe(response, error, source="command")
        _check_status(
            response,
            lower=lower,
            upper=upper,
            max_current_ma=MAX_CURRENT_MA,
            target=target,
            direction_guard=direction_guard,
        )
        _log_status(
            moving_id,
            "target_write_response",
            response,
            force_f0=moving_baseline.force_f0,
            target=target,
            elapsed_s=elapsed,
        )
        readback_target = _read_value(motors[moving_id], TARGET_POSITION_ADDR)
        if readback_target != target:
            raise SafetyViolation(
                f"ID{moving_id} target write readback mismatch: "
                f"commanded_target={target} "
                f"readback_target_position={readback_target}"
            )
        logger.info(
            "ID%d_TARGET_WRITE_READBACK commanded_target=%d "
            "readback_target_position=%d actual_position=%d current_mA=%d "
            "force_g=%d fault=0x%02X",
            moving_id,
            target,
            readback_target,
            response.current_position,
            response.current_mA,
            response.force_g,
            response.error_code,
        )
        for observer_id, observer in observers.items():
            observer.observe(
                motors[observer_id].read_status(), source="after_target"
            )
            _check_observer_config(
                observer_id,
                configs_before[observer_id],
                _read_config(motors[observer_id]),
                baselines[observer_id],
                source="after_target",
            )

        deadline = t0_mono + MOTION_TIMEOUT_S
        while True:
            status = motors[moving_id].read_status()
            error = abs(status.current_position - target)
            elapsed = trace.observe(status, error)
            _check_status(
                status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
                target=target,
                direction_guard=direction_guard,
            )
            _log_status(
                moving_id,
                "motion_poll",
                status,
                force_f0=moving_baseline.force_f0,
                target=target,
                elapsed_s=elapsed,
            )
            for observer_id, observer in observers.items():
                observer.observe(
                    motors[observer_id].read_status(), source="motion_poll"
                )
            positive_motion = trace.first_motion_mono is not None
            if positive_motion and error <= PAUSE_TRIGGER_TOLERANCE_STEPS:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SafetyViolation(
                    f"ID{moving_id} did not confirm positive motion and reach "
                    f"+/-{PAUSE_TRIGGER_TOLERANCE_STEPS} step within "
                    f"{MOTION_TIMEOUT_S:.2f}s"
                )
            time.sleep(min(POLL_INTERVAL_S, remaining))

        pause_attempted = True
        pause_success, pause_response = _pause_moving_once(
            motors[moving_id], moving_id
        )
        if pause_response is not None:
            trace.set_pause_status(pause_response)
        if not pause_success:
            raise SafetyViolation(f"ID{moving_id} completion pause failed")

        confirmation_attempted = True
        try:
            stop_confirmation, final_statuses = _confirm_all_stopped(
                motors, moving_id, observers, limits
            )
        except Exception as exc:
            _hard_stop_for_confirmation_exception(exc)
            stop_confirmation = False
            raise
        final_status = final_statuses[moving_id]
        trace.set_post_pause_status(final_status)
        if not stop_confirmation:
            raise SafetyViolation("M4-B stop confirmation failed")
        final_error = abs(final_status.current_position - target)
        if final_error > FINAL_ACCEPTANCE_TOLERANCE_STEPS:
            raise SafetyViolation(
                f"ID{moving_id} post-pause final_error={final_error} exceeds "
                f"{FINAL_ACCEPTANCE_TOLERANCE_STEPS} step"
            )
        post_pause_target = _read_value(
            motors[moving_id], TARGET_POSITION_ADDR
        )
        logger.info(
            "ID%d_POST_PAUSE_STATE commanded_target=%d "
            "readback_target_position=%d actual_position=%d error=%d "
            "current_mA=%d force_g=%d fault=0x%02X "
            "target_sync_observation_only=true",
            moving_id,
            target,
            post_pause_target,
            final_status.current_position,
            final_error,
            final_status.current_mA,
            final_status.force_g,
            final_status.error_code,
        )
        for observer_id in observers:
            _check_observer_config(
                observer_id,
                configs_before[observer_id],
                _read_config(motors[observer_id]),
                baselines[observer_id],
                source="final",
            )

        logger.info(
            "M4B_RESULT decision=PASS moving_id=%d target=%d final_actual=%d "
            "final_error=%d strict_tolerance_pass=%s "
            "final_tolerance_pass=true observer_config_unchanged=true "
            "%s pause_success=%s stop_confirmation=%s",
            moving_id,
            target,
            final_status.current_position,
            final_error,
            final_error <= STRICT_TOLERANCE_STEPS,
            _observer_summary(observers),
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
            f"PASS M4-B moving_id={moving_id} target={target} "
            f"final={final_status.current_position} error={final_error} "
            f"read_only_ids={sorted(observers)} "
            f"pause_success={pause_success} stop_confirmation={stop_confirmation}"
        )
        return 0
    except Exception as exc:
        if trace is not None:
            positive_motion = trace.first_motion_mono is not None
        if armed and not pause_attempted:
            pause_attempted = True
            pause_success, pause_response = _pause_moving_once(
                motors.get(moving_id), moving_id
            )
            if trace is not None and pause_response is not None:
                trace.set_pause_status(pause_response)
        if (
            armed
            and pause_success
            and not confirmation_attempted
            and motors
            and observers
            and limits
        ):
            confirmation_attempted = True
            try:
                stop_confirmation, final_statuses = _confirm_all_stopped(
                    motors, moving_id, observers, limits
                )
                if trace is not None:
                    trace.set_post_pause_status(final_statuses[moving_id])
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
            "M4B_RESULT decision=FAIL moving_id=%d positive_motion=%s %s "
            "pause_attempted=%s pause_success=%s stop_confirmation=%s reason=%s",
            moving_id,
            positive_motion,
            _observer_summary(observers),
            pause_attempted,
            pause_success,
            stop_confirmation,
            exc,
        )
        print(f"STOP M4-B: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Stage M4-B: one fixed +10 step motor with three read-only peers"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--moving-id", type=int, required=True, choices=MOTOR_IDS)
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
