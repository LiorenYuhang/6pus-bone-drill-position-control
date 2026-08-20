#!/usr/bin/env python3
"""Stage M6-B: move one addressed motor while five peers stay strictly read-only."""

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


MOTOR_IDS = (1, 2, 3, 4, 5, 6)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
MOVE_STEPS = 10
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 3.0
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

logger = logging.getLogger("safe_m6b_one_motor")


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
            "ID%d_OBSERVER_STATUS source=%s baseline_reference=%d actual=%d "
            "target=%d current_mA=%d force_g=%d fault=0x%02X",
            self.motor_id,
            source,
            reference,
            status.current_position,
            status.target_position,
            status.current_mA,
            status.force_g,
            status.error_code,
        )
        if status.error_code != 0:
            raise SafetyViolation(
                f"ID{self.motor_id} observer fault=0x{status.error_code:02X}"
            )
        if not (
            self.baseline.position.safety_min
            <= status.current_position
            <= self.baseline.position.safety_max
        ):
            raise SafetyViolation(
                f"ID{self.motor_id} observer position left baseline safety band "
                f"[{self.baseline.position.safety_min},{self.baseline.position.safety_max}]: "
                f"actual={status.current_position}"
            )
        if status.current_mA > self.baseline.current_max_mA:
            raise SafetyViolation(
                f"ID{self.motor_id} observer current exceeded baseline maximum: "
                f"baseline_max={self.baseline.current_max_mA}mA "
                f"actual={status.current_mA}mA"
            )
        if not (
            self.baseline.target_baseline.safety_min
            <= status.target_position
            <= self.baseline.target_baseline.safety_max
        ):
            raise SafetyViolation(
                f"ID{self.motor_id} observer target left baseline safety band "
                f"[{self.baseline.target_baseline.safety_min},{self.baseline.target_baseline.safety_max}]: "
                f"target={status.target_position}"
            )


class M6bAuditedBus:
    """Bus wrapper ensuring only the chosen moving_id receives write frames."""

    def __init__(self, bus: UartBus, moving_id: int):
        self._bus = bus
        self.moving_id = moving_id
        self.write_count = 0
        self.write_destinations: list[int] = []
        self.write_registers: list[str] = []
        self.read_counts: dict[int, int] = {motor_id: 0 for motor_id in MOTOR_IDS}

    def request(self, frame: bytes, **kwargs) -> Frame:
        parsed = parse_frame(frame)
        if parsed.motor_id not in MOTOR_IDS:
            raise SafetyViolation(
                f"destination ID{parsed.motor_id} is outside M6-B scope {MOTOR_IDS}"
            )
        if parsed.command == CMD_WR_REGISTER:
            self.write_count += 1
            self.write_destinations.append(parsed.motor_id)
            reg_hex = (
                f"0x{parsed.register_addr:04X}"
                if parsed.register_addr is not None
                else "None"
            )
            self.write_registers.append(reg_hex)
            if parsed.motor_id != self.moving_id:
                raise SafetyViolation(
                    f"M6-B write frame sent to observer ID{parsed.motor_id} "
                    f"(moving ID is {self.moving_id})"
                )
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                raise SafetyViolation(
                    f"M6-B write register {reg_hex} is forbidden"
                )
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=%d register=%s "
                "data=%s verified=true",
                parsed.motor_id,
                reg_hex,
                parsed.data.hex(" ").upper(),
            )
            return self._bus.request(frame, **kwargs)

        if parsed.command in (CMD_RD_REGISTER, CMD_RD_STATUS):
            self.read_counts[parsed.motor_id] = (
                self.read_counts.get(parsed.motor_id, 0) + 1
            )
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X "
                "register=0x%04X",
                parsed.motor_id,
                parsed.command,
                parsed.register_addr or 0,
            )
            return self._bus.request(frame, **kwargs)

        raise SafetyViolation(f"unsupported command 0x{parsed.command:02X}")

    def close(self) -> None:
        self._bus.close()


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        raise RuntimeError(f"log file already exists: {log_path}")

    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    bus_logger = logging.getLogger("uart_bus")
    bus_logger.setLevel(logging.INFO)
    bus_logger.handlers.clear()
    bus_logger.addHandler(file_handler)


def _read_value(motor: Motor, address: int) -> int:
    return motor.read_register(address).values[0]


def _read_config_snapshot(motor: Motor) -> ConfigSnapshot:
    mode = _read_value(motor, CONTROL_MODE_ADDR)
    speed = _read_value(motor, TARGET_SPEED_ADDR)
    target = _read_value(motor, TARGET_POSITION_ADDR)
    return ConfigSnapshot(mode=mode, speed=speed, target=target)


def _build_target_baseline(targets: list[int], motor_id: int) -> TargetBaseline:
    if not targets:
        raise SafetyViolation(f"ID{motor_id} collected 0 target baseline samples")
    histogram = dict(sorted(Counter(targets).items()))
    unique_targets = sorted(histogram.keys())
    t_min = min(unique_targets)
    t_max = max(unique_targets)
    if t_max - t_min > 1:
        raise SafetyViolation(
            f"ID{motor_id} target baseline histogram={histogram} span "
            f"({t_min}..{t_max}) exceeds 1 step"
        )
    return TargetBaseline(
        reference=unique_targets[0],
        safety_min=t_min,
        safety_max=t_max,
        histogram=histogram,
    )


def _check_no_unidirectional_drift(
    samples: list[int], motor_id: int, name: str = "position"
) -> None:
    """Reject observer samples that exhibit a sustained unidirectional drift."""
    if len(samples) < 4:
        return
    # 1. Monotonic non-decreasing with net change
    is_monotonic_inc = (
        all(b >= a for a, b in zip(samples, samples[1:]))
        and samples[-1] > samples[0]
    )
    # 2. Monotonic non-increasing with net change
    is_monotonic_dec = (
        all(b <= a for a, b in zip(samples, samples[1:]))
        and samples[-1] < samples[0]
    )
    if is_monotonic_inc or is_monotonic_dec:
        raise SafetyViolation(
            f"ID{motor_id} observer {name} exhibited sustained monotonic drift: "
            f"start={samples[0]} end={samples[-1]}"
        )
    # 3. Disjoint early/late halves
    mid = len(samples) // 2
    first_half = samples[:mid]
    second_half = samples[mid:]
    if min(second_half) > max(first_half) or max(second_half) < min(first_half):
        raise SafetyViolation(
            f"ID{motor_id} observer {name} exhibited sustained half-split drift: "
            f"first_half=[{min(first_half)},{max(first_half)}] "
            f"second_half=[{min(second_half)},{max(second_half)}]"
        )


def _build_observer_position_baseline(
    positions: list[int], motor_id: int
) -> M2dPositionBaseline:
    if not positions:
        raise SafetyViolation(f"ID{motor_id} collected 0 position baseline samples")

    for p in positions:
        if p < 0 or p > 2000:
            raise SafetyViolation(
                f"ID{motor_id} observer position {p} out of bounds [0, 2000]"
            )

    histogram = dict(sorted(Counter(positions).items()))
    counts = Counter(positions).most_common()
    if len(counts) > 1 and counts[0][1] == counts[1][1]:
        raise SafetyViolation(
            f"ID{motor_id} observer position histogram={histogram} has no unique mode: "
            f"top_modes={counts[0]}, {counts[1]}"
        )
    unique_mode = counts[0][0]

    p_min = min(positions)
    p_max = max(positions)
    span = p_max - p_min
    if span > 2:
        raise SafetyViolation(
            f"ID{motor_id} observer position span {span} ({p_min}..{p_max}) "
            f"exceeds allowed span <= 2 steps: histogram={histogram}"
        )

    _check_no_unidirectional_drift(positions, motor_id, name="position")

    return M2dPositionBaseline(
        reference=unique_mode,
        safety_min=p_min,
        safety_max=p_max,
        histogram=histogram,
    )


def _build_observer_target_baseline(
    targets: list[int], motor_id: int
) -> TargetBaseline:
    if not targets:
        raise SafetyViolation(f"ID{motor_id} collected 0 target baseline samples")

    histogram = dict(sorted(Counter(targets).items()))
    counts = Counter(targets).most_common()
    if len(counts) > 1 and counts[0][1] == counts[1][1]:
        raise SafetyViolation(
            f"ID{motor_id} observer target histogram={histogram} has no unique mode: "
            f"top_modes={counts[0]}, {counts[1]}"
        )
    unique_mode = counts[0][0]

    t_min = min(targets)
    t_max = max(targets)
    span = t_max - t_min
    if span > 2:
        raise SafetyViolation(
            f"ID{motor_id} observer target span {span} ({t_min}..{t_max}) "
            f"exceeds allowed span <= 2 steps: histogram={histogram}"
        )

    _check_no_unidirectional_drift(targets, motor_id, name="target")

    return TargetBaseline(
        reference=unique_mode,
        safety_min=t_min,
        safety_max=t_max,
        histogram=histogram,
    )


def _collect_baseline(
    motors: dict[int, Motor],
    moving_id: int,
    read_only_ids: list[int],
    duration_s: float = BASELINE_DURATION_S,
    rate_hz: float = BASELINE_RATE_HZ,
) -> dict[int, MotorBaseline]:
    total_samples = max(1, int(duration_s * rate_hz))
    interval_s = 1.0 / rate_hz if rate_hz > 0 else 0.02
    positions: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}
    currents: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}
    forces: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}
    targets: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}
    faults: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}

    for idx in range(1, total_samples + 1):
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            _check_status(
                status,
                lower=0,
                upper=2000,
                max_current_ma=MAX_CURRENT_MA,
            )
            positions[motor_id].append(status.current_position)
            currents[motor_id].append(status.current_mA)
            forces[motor_id].append(status.force_g)
            targets[motor_id].append(status.target_position)
            faults[motor_id].append(status.error_code)
            logger.info(
                "M6B_BASELINE_SAMPLE sample=%d/%d motor_id=%d actual=%d "
                "target=%d current_mA=%d force_g=%d fault=0x%02X",
                idx,
                total_samples,
                motor_id,
                status.current_position,
                status.target_position,
                status.current_mA,
                status.force_g,
                status.error_code,
            )
        time.sleep(interval_s)

    baselines: dict[int, MotorBaseline] = {}
    for motor_id in MOTOR_IDS:
        if motor_id == moving_id:
            pos_base = _build_m2d_position_baseline(positions[motor_id])
            target_base = _build_target_baseline(targets[motor_id], motor_id)
        else:
            if max(currents[motor_id]) != 0:
                raise SafetyViolation(
                    f"ID{motor_id} observer current={max(currents[motor_id])}mA "
                    f"must be 0mA during baseline"
                )
            if any(f != 0 for f in faults[motor_id]):
                raise SafetyViolation(
                    f"ID{motor_id} observer fault detected during baseline"
                )
            pos_base = _build_observer_position_baseline(
                positions[motor_id], motor_id
            )
            target_base = _build_observer_target_baseline(
                targets[motor_id], motor_id
            )

        baselines[motor_id] = MotorBaseline(
            position=pos_base,
            force_f0=statistics.mean(forces[motor_id]),
            current_max_mA=max(currents[motor_id]),
            target_baseline=target_base,
        )
        logger.info(
            "BASELINE_ESTABLISHED motor_id=%d role=%s reference=%d "
            "safety_min=%d safety_max=%d target_ref=%d target_band=[%d,%d] "
            "target_hist=%s f0=%.1f current_max_mA=%d",
            motor_id,
            "moving" if motor_id == moving_id else "observer",
            pos_base.reference,
            pos_base.safety_min,
            pos_base.safety_max,
            target_base.reference,
            target_base.safety_min,
            target_base.safety_max,
            target_base.histogram,
            baselines[motor_id].force_f0,
            baselines[motor_id].current_max_mA,
        )
    return baselines


def _audit_observer_config(
    motor: Motor,
    baseline_config: ConfigSnapshot,
    baseline_target_band: TargetBaseline,
    stage_name: str,
) -> None:
    current = _read_config_snapshot(motor)
    if current.mode != baseline_config.mode:
        raise SafetyViolation(
            f"ID{motor.motor_id} mode changed during {stage_name}: "
            f"expected {baseline_config.mode}, got {current.mode}"
        )
    if current.speed != baseline_config.speed:
        raise SafetyViolation(
            f"ID{motor.motor_id} speed changed during {stage_name}: "
            f"expected {baseline_config.speed}, got {current.speed}"
        )
    if not (
        baseline_target_band.safety_min
        <= current.target
        <= baseline_target_band.safety_max
    ):
        raise SafetyViolation(
            f"ID{motor.motor_id} target changed during {stage_name}: "
            f"got {current.target}, outside baseline band "
            f"[{baseline_target_band.safety_min},{baseline_target_band.safety_max}]"
        )


def _pause_single(motor: Motor) -> tuple[bool, Optional[StatusFeedback]]:
    try:
        response = motor.write_register(PAUSE_ADDR, [1])
        logger.warning(
            "ID%d_PAUSE sent_once=true destination_id=%d actual=%d "
            "current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
            motor.motor_id,
            motor.motor_id,
            response.current_position,
            response.current_mA,
            response.force_g,
            response.temperature,
            response.error_code,
        )
        return True, response
    except Exception as exc:
        message = (
            f"HARD STOP REQUIRED: ID{motor.motor_id} pause command failed; use the "
            "hardware emergency stop or remove motor power immediately"
        )
        logger.critical("%s: %s", message, exc)
        print(message, file=sys.stderr)
        return False, None


def run(args: argparse.Namespace) -> int:
    moving_id = args.moving_id
    if moving_id not in MOTOR_IDS:
        print(f"ERROR: moving_id={moving_id} must be in {MOTOR_IDS}", file=sys.stderr)
        return 1

    read_only_ids = [m for m in MOTOR_IDS if m != moving_id]

    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m6/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m6b{moving_id}_id{moving_id}_only.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[M6bAuditedBus] = None
    armed = False

    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = M6bAuditedBus(bus, moving_id=moving_id)
        motors = {
            motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS
        }

        # Step 1: Preflight limits and config
        baseline_configs: dict[int, ConfigSnapshot] = {}
        for motor_id in MOTOR_IDS:
            lower = _read_value(motors[motor_id], LOWER_LIMIT_ADDR)
            upper = _read_value(motors[motor_id], UPPER_LIMIT_ADDR)
            if lower != 0 or upper != 2000:
                raise SafetyViolation(
                    f"ID{motor_id} limits [{lower},{upper}] must equal [0,2000]"
                )
            baseline_configs[motor_id] = _read_config_snapshot(motors[motor_id])

        # Step 2: Baseline establishment
        baselines = _collect_baseline(motors, moving_id, read_only_ids)
        moving_base = baselines[moving_id]
        target_position = moving_base.position.reference + MOVE_STEPS
        if target_position > SOFTWARE_UPPER_LIMIT:
            raise SafetyViolation(
                f"calculated target {target_position} exceeds limit {SOFTWARE_UPPER_LIMIT}"
            )

        observers = {
            oid: ObserverTrace(
                motor_id=oid,
                baseline=baselines[oid],
            )
            for oid in read_only_ids
        }

        # Step 3: Human confirmation
        expected_word = f"M6-B MOVE ID{moving_id}"
        prompt = (
            f"Ready to move ID{moving_id} +{MOVE_STEPS} steps "
            f"({moving_base.position.reference} -> {target_position}).\n"
            f"Observers {read_only_ids} stay strictly read-only.\n"
            f"Type '{expected_word}' to proceed: "
        )
        if input(prompt).strip() != expected_word:
            logger.warning("OPERATOR_ABORT confirmation mismatch, zero writes")
            print("ABORTED: confirmation mismatch, 0 writes executed")
            return 1

        # Armed for motion
        armed = True

        # Step 4: Setup moving motor (5 registers: 0x25..0x29)
        # 0x25: CONTROL_MODE = SPEED_MODE (2)
        # 0x26: VOLTAGE_TARGET = 0
        # 0x27: FORCE_TARGET = 0
        # 0x28: TARGET_SPEED = SPEED_STEPS_PER_SECOND (50)
        # 0x29: TARGET_POSITION = moving_base.position.safety_max
        motors[moving_id].write_register(
            CONTROL_MODE_ADDR,
            [
                SPEED_MODE,
                0,
                0,
                SPEED_STEPS_PER_SECOND,
                moving_base.position.safety_max,
            ],
        )
        readback_mode = _read_value(motors[moving_id], CONTROL_MODE_ADDR)
        readback_speed = _read_value(motors[moving_id], TARGET_SPEED_ADDR)
        if readback_mode != SPEED_MODE:
            raise SafetyViolation(
                f"ID{moving_id} mode readback {readback_mode} != {SPEED_MODE}"
            )
        if readback_speed != SPEED_STEPS_PER_SECOND:
            raise SafetyViolation(
                f"ID{moving_id} speed readback {readback_speed} != {SPEED_STEPS_PER_SECOND}"
            )
        for oid in read_only_ids:
            _audit_observer_config(
                motors[oid],
                baseline_configs[oid],
                baselines[oid].target_baseline,
                stage_name="after_setup",
            )

        # Step 5: Write target position
        motors[moving_id].write_register(TARGET_POSITION_ADDR, [target_position])
        readback_target = _read_value(motors[moving_id], TARGET_POSITION_ADDR)
        if readback_target != target_position:
            raise SafetyViolation(
                f"ID{moving_id} target readback {readback_target} != {target_position}"
            )
        for oid in read_only_ids:
            _audit_observer_config(
                motors[oid],
                baseline_configs[oid],
                baselines[oid].target_baseline,
                stage_name="after_target_write",
            )

        # Step 6: Motion polling
        motion_trace = MotionTrace(
            baseline_min=moving_base.position.safety_min,
            baseline_max=moving_base.position.safety_max,
            target=target_position,
            force_f0=moving_base.force_f0,
            t0_mono=time.monotonic(),
            t0_wall=datetime.now(timezone.utc).isoformat(),
            samples=[],
        )

        poll_start = time.time()
        pause_triggered = False
        final_status: Optional[StatusFeedback] = None

        while time.time() - poll_start < MOTION_TIMEOUT_S:
            # 1. Read moving motor
            m_status = motors[moving_id].read_status()
            _check_status(
                m_status,
                lower=0,
                upper=2000,
                max_current_ma=MAX_CURRENT_MA,
            )
            err = abs(target_position - m_status.current_position)
            motion_trace.observe(m_status, err, source="poll")
            logger.info(
                "ID%d_STATUS source=motion_poll actual=%d target=%d current_mA=%d "
                "force_g=%d fault=0x%02X",
                moving_id,
                m_status.current_position,
                m_status.target_position,
                m_status.current_mA,
                m_status.force_g,
                m_status.error_code,
            )

            # 2. Read observer motors
            for oid in read_only_ids:
                o_status = motors[oid].read_status()
                _check_status(
                    o_status,
                    lower=0,
                    upper=2000,
                    max_current_ma=MAX_CURRENT_MA,
                )
                observers[oid].observe(o_status, source="motion_poll")

            # 3. Check pause condition
            err = abs(target_position - m_status.current_position)
            if err <= PAUSE_TRIGGER_TOLERANCE_STEPS:
                pause_triggered = True
                pause_ok, final_status = _pause_single(motors[moving_id])
                if not pause_ok:
                    raise SafetyViolation(
                        f"ID{moving_id} pause failed; HARD STOP REQUIRED"
                    )
                break

            time.sleep(POLL_INTERVAL_S)

        if not pause_triggered:
            raise SafetyViolation(
                f"ID{moving_id} failed to reach error <= {PAUSE_TRIGGER_TOLERANCE_STEPS} "
                f"within {MOTION_TIMEOUT_S}s"
            )

        # Step 7: Stop confirmation & post-pause verification
        stop_samples = max(1, int(STOP_CONFIRM_DURATION_S / 0.02))
        stop_statuses: list[StatusFeedback] = []
        for _ in range(stop_samples):
            st = motors[moving_id].read_status()
            stop_statuses.append(st)
            for oid in read_only_ids:
                ost = motors[oid].read_status()
                observers[oid].observe(ost, source="stop_confirm")
            time.sleep(0.02)

        stable, metrics = _stop_confirmation_metrics(stop_statuses)
        if not stable:
            raise SafetyViolation(
                f"ID{moving_id} stop confirmation failed: {metrics.get('reason')}"
            )

        post_pause_actual = stop_statuses[-1].current_position
        final_err = abs(target_position - post_pause_actual)
        if final_err > FINAL_ACCEPTANCE_TOLERANCE_STEPS:
            raise SafetyViolation(
                f"ID{moving_id} post-pause final_error={final_err} exceeds "
                f"{FINAL_ACCEPTANCE_TOLERANCE_STEPS} step gate"
            )

        # Final write audit
        if audited_bus.write_count != 3:
            raise SafetyViolation(
                f"M6-B write audit failed: write_count={audited_bus.write_count} != 3"
            )

        logger.info(
            "M6B_RESULT decision=PASS moving_id=%d target=%d final=%d "
            "error=%d read_only_ids=%s pause_success=True stop_confirmation=True",
            moving_id,
            target_position,
            post_pause_actual,
            final_err,
            read_only_ids,
        )
        print(
            f"PASS M6-B moving_id={moving_id} target={target_position} "
            f"final={post_pause_actual} error={final_err} "
            f"read_only_ids={read_only_ids} pause_success=True "
            f"stop_confirmation=True"
        )
        return 0
    except Exception as exc:
        logger.error("M6B_ABORT error=%s", exc)
        print(f"STOP M6-B: {exc}", file=sys.stderr)
        if armed and audited_bus is not None and motors.get(moving_id):
            try:
                pause_ok, _ = _pause_single(motors[moving_id])
                if not pause_ok:
                    print("HARD STOP REQUIRED: emergency pause failed", file=sys.stderr)
            except Exception as pause_exc:
                logger.critical("EMERGENCY_PAUSE_FAILED error=%s", pause_exc)
                print("HARD STOP REQUIRED: emergency pause exception", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(
                "ACCESS_SUMMARY write_count=%d write_destinations=%s "
                "write_registers=%s read_counts=%s",
                audited_bus.write_count,
                audited_bus.write_destinations,
                audited_bus.write_registers,
                audited_bus.read_counts,
            )
            try:
                audited_bus.close()
            except Exception as close_exc:
                logger.error("CLOSE_ERROR %s", close_exc)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage M6-B: move one addressed motor while five peers stay read-only"
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="serial device path",
    )
    parser.add_argument(
        "--moving-id",
        type=int,
        required=True,
        help="ID of motor to move (5 for M6-B5, 6 for M6-B6)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="serial read timeout in seconds",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="serial read retries",
    )
    parser.add_argument(
        "--log",
        default="",
        help="path to log file",
    )
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if args.retries < 0:
        parser.error("retries must be non-negative")
    if args.moving_id not in MOTOR_IDS:
        parser.error(f"moving-id must be one of {MOTOR_IDS}")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
