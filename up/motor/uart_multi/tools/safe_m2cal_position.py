#!/usr/bin/env python3
"""M2-CAL: characterize target versus actual position for ID1 and ID2."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import logging
from pathlib import Path
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
    SafetyViolation,
    _check_status,
    _stop_confirmation_metrics,
)
from uart_bus import UartBus  # noqa: E402

MOTOR_IDS = (1, 2)
CAL_TARGETS = (10, 20, 30, 40)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 2.0
MAX_CURRENT_MA = 500
POLL_INTERVAL_S = 0.02
STOP_CONFIRM_DURATION_S = 0.5
SOFTWARE_UPPER_LIMIT = 1800
MAX_BASELINE_EXCURSION_STEPS = 40
ALLOWED_WRITE_REGISTERS = {
    CONTROL_MODE_ADDR,
    TARGET_POSITION_ADDR,
    PAUSE_ADDR,
}

logger = logging.getLogger("safe_m2cal_position")


class CalibrationAuditedBus:
    """Allow only addressed mode/target/pause writes for ID1 and ID2."""

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
                f"destination ID={parsed.motor_id} is outside M2-CAL scope"
            )
        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                logger.critical(
                    "WRITE_BLOCKED destination_id=%d register=0x%04X",
                    parsed.motor_id,
                    parsed.register_addr,
                )
                raise SafetyViolation(
                    f"write register 0x{parsed.register_addr:04X} is forbidden in M2-CAL"
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
                f"outgoing command 0x{parsed.command:02X} is outside M2-CAL allowlist"
            )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        destinations = [frame.motor_id for frame in self.write_frames]
        registers = [f"0x{frame.register_addr:04X}" for frame in self.write_frames]
        return (
            f"ACCESS_SUMMARY write_count={len(self.write_frames)} "
            f"write_destinations={destinations} write_registers={registers} "
            f"id1_read_count={self.read_counts[1]} id2_read_count={self.read_counts[2]} "
            "other_id_write_count=0 compensation_write_count=0"
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


def _log_sample(
    motor_id: int,
    phase: str,
    target: int,
    status: StatusFeedback,
) -> None:
    error = status.current_position - target
    logger.info(
        "CAL_SAMPLE motor_id=%d phase=%s target_position=%d actual_position=%d "
        "error=%d current_mA=%d force_g=%d fault=0x%02X temperature_C=%d",
        motor_id,
        phase,
        target,
        status.current_position,
        error,
        status.current_mA,
        status.force_g,
        status.error_code,
        status.temperature,
    )


def _confirm_stopped(
    motor: Motor,
    motor_id: int,
    lower: int,
    upper: int,
    target: int,
) -> tuple[bool, StatusFeedback]:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses: list[StatusFeedback] = []
    while True:
        status = motor.read_status()
        _check_status(
            status,
            lower=lower,
            upper=upper,
            max_current_ma=MAX_CURRENT_MA,
        )
        statuses.append(status)
        _log_sample(motor_id, "pause_after", target, status)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(POLL_INTERVAL_S, remaining))
    stable, metrics = _stop_confirmation_metrics(statuses)
    logger.info(
        "CAL_STOP_CONFIRMATION motor_id=%d stable=%s reason=%s samples=%d",
        motor_id,
        stable,
        metrics["reason"],
        len(statuses),
    )
    return stable, statuses[-1]


def _confirm_operator() -> None:
    answer = input(
        "确认仅连接并固定 ID1+ID2、校准目标仅为 10/20/30/40、急停可用；"
        "输入 M2-CAL MOVE ID1 ID2 TARGETS 10 20 30 40："
    ).strip()
    if answer != "M2-CAL MOVE ID1 ID2 TARGETS 10 20 30 40":
        raise SafetyViolation("operator did not arm M2-CAL")


def _hard_stop(message: str) -> None:
    full = f"HARD STOP REQUIRED: {message}; use hardware emergency stop or remove power"
    logger.critical(full)
    print(full, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m2cal/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m2cal_position.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited: Optional[CalibrationAuditedBus] = None
    motors: dict[int, Motor] = {}
    limits: dict[int, tuple[int, int]] = {}
    active_motor: Optional[int] = None
    pause_done = False
    confirmation_done = False
    point_count = 0
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited = CalibrationAuditedBus(bus)
        motors = {motor_id: Motor(audited, motor_id) for motor_id in MOTOR_IDS}

        initial_positions: dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            lower = _read_value(motors[motor_id], LOWER_LIMIT_ADDR)
            upper = _read_value(motors[motor_id], UPPER_LIMIT_ADDR)
            if not 0 <= lower <= upper <= 2000:
                raise SafetyViolation(
                    f"ID{motor_id} invalid limits lower={lower} upper={upper}"
                )
            limits[motor_id] = (lower, upper)
            initial = motors[motor_id].read_status()
            _check_status(
                initial,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            initial_positions[motor_id] = initial.current_position
            _log_sample(motor_id, "preflight", -1, initial)
            allowed_upper = min(upper, SOFTWARE_UPPER_LIMIT)
            if any(not lower <= target <= allowed_upper for target in CAL_TARGETS):
                raise SafetyViolation(
                    f"ID{motor_id} calibration target outside [{lower},{allowed_upper}]"
                )
            if max(abs(target - initial.current_position) for target in CAL_TARGETS) > MAX_BASELINE_EXCURSION_STEPS:
                raise SafetyViolation(
                    f"ID{motor_id} calibration span exceeds {MAX_BASELINE_EXCURSION_STEPS} steps "
                    f"from initial position {initial.current_position}"
                )

        logger.info(
            "M2CAL_PRECHECK ids=%s targets=%s speed=%d timeout_s=%.1f "
            "max_current_mA=%d baseline_positions=%s",
            MOTOR_IDS,
            CAL_TARGETS,
            SPEED_STEPS_PER_SECOND,
            MOTION_TIMEOUT_S,
            MAX_CURRENT_MA,
            initial_positions,
        )
        print(
            "M2-CAL PRECHECK "
            f"ids={MOTOR_IDS} targets={CAL_TARGETS} "
            f"baseline_positions={initial_positions}"
        )
        _confirm_operator()

        for motor_id in MOTOR_IDS:
            motor = motors[motor_id]
            lower, upper = limits[motor_id]
            for target in CAL_TARGETS:
                active_motor = motor_id
                pause_done = False
                confirmation_done = False
                point_count += 1
                before = motor.read_status()
                _check_status(before, lower=lower, upper=upper, max_current_ma=MAX_CURRENT_MA)
                _log_sample(motor_id, "point_before", target, before)

                setup = motor.write_register(
                    CONTROL_MODE_ADDR,
                    [SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, before.current_position],
                )
                _check_status(setup, lower=lower, upper=upper, max_current_ma=MAX_CURRENT_MA)
                target_response = motor.write_register(TARGET_POSITION_ADDR, [target])
                _check_status(
                    target_response,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                    target=target,
                )
                _log_sample(motor_id, "target_write", target, target_response)

                deadline = time.monotonic() + MOTION_TIMEOUT_S
                last = target_response
                while True:
                    last = motor.read_status()
                    _check_status(
                        last,
                        lower=lower,
                        upper=upper,
                        max_current_ma=MAX_CURRENT_MA,
                    )
                    _log_sample(motor_id, "motion", target, last)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(POLL_INTERVAL_S, remaining))

                pause_before = motor.read_status()
                _check_status(
                    pause_before,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                )
                _log_sample(motor_id, "pause_before", target, pause_before)
                pause_response = motor.write_register(PAUSE_ADDR, [1])
                pause_done = True
                _check_status(
                    pause_response,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                )
                _log_sample(motor_id, "pause_response", target, pause_response)
                stable, final = _confirm_stopped(motor, motor_id, lower, upper, target)
                confirmation_done = True
                if not stable:
                    raise SafetyViolation(f"ID{motor_id} target={target} stop confirmation failed")
                logger.info(
                    "CAL_POINT motor_id=%d target_position=%d actual_position=%d "
                    "error=%d current_mA=%d force_g=%d fault=0x%02X "
                    "pause_success=%s stop_confirmation=%s",
                    motor_id,
                    target,
                    final.current_position,
                    final.current_position - target,
                    final.current_mA,
                    final.force_g,
                    final.error_code,
                    pause_done,
                    stable,
                )
                active_motor = None

        logger.info(
            "M2CAL_RESULT decision=PASS points=%d targets=%s "
            "safety_only=true compensation_write_count=0",
            point_count,
            CAL_TARGETS,
        )
        print(
            f"PASS M2-CAL points={point_count} targets={CAL_TARGETS} "
            "safety_only=true compensation_write_count=0"
        )
        return 0
    except Exception as exc:
        if active_motor is not None and not pause_done and motors:
            try:
                motors[active_motor].write_register(PAUSE_ADDR, [1])
                pause_done = True
            except Exception as pause_exc:
                _hard_stop(f"ID{active_motor} pause failed: {pause_exc}")
        if active_motor is not None and pause_done and not confirmation_done and motors:
            try:
                lower, upper = limits[active_motor]
                _confirm_stopped(motors[active_motor], active_motor, lower, upper, -1)
            except Exception as confirm_exc:
                _hard_stop(f"ID{active_motor} stop confirmation failed: {confirm_exc}")
        logger.error("M2CAL_RESULT decision=FAIL points=%d reason=%s", point_count, exc)
        print(f"STOP M2-CAL: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited is not None:
            logger.info(audited.summary())
        if bus is not None:
            bus.close()


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="M2-CAL: fixed target points 10/20/30/40 for ID1 and ID2"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
