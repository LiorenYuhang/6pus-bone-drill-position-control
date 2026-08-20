#!/usr/bin/env python3
"""ID4-OFFSET: characterize the persistent raw -2 position state safely."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
    _stop_confirmation_metrics,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 4
TARGETS = (10, 20, 30, 40)
CONTROL_MODE_ADDR = 0x25
TARGET_POSITION_ADDR = 0x29
ACTUAL_POSITION_ADDR = 0x2A
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 2.0
MAX_CURRENT_MA = 500
POLL_INTERVAL_S = 0.02
STOP_CONFIRM_DURATION_S = 0.5
ENTRY_SAMPLE_COUNT = 100
ENTRY_SAMPLE_INTERVAL_S = 0.02
ENTRY_POSITION = -2
ENTRY_RAW_U16 = 0xFFFE
DEVICE_LOWER_LIMIT = 0
DEVICE_UPPER_LIMIT = 2000
KNOWN_RESIDUAL_MIN = -3
KNOWN_RESIDUAL_MAX = 0
READ_REGISTERS = {
    LOWER_LIMIT_ADDR,
    UPPER_LIMIT_ADDR,
    TARGET_POSITION_ADDR,
    ACTUAL_POSITION_ADDR,
}
EXPECTED_WRITES = (
    (CONTROL_MODE_ADDR, (SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, TARGETS[0])),
    (PAUSE_ADDR, (1,)),
    (TARGET_POSITION_ADDR, (TARGETS[1],)),
    (PAUSE_ADDR, (1,)),
    (TARGET_POSITION_ADDR, (TARGETS[2],)),
    (PAUSE_ADDR, (1,)),
    (TARGET_POSITION_ADDR, (TARGETS[3],)),
    (PAUSE_ADDR, (1,)),
)

logger = logging.getLogger("safe_id4_offset_characterization")


@dataclass(frozen=True)
class RawPosition:
    raw_u16: int
    signed: int


@dataclass(frozen=True)
class PointResult:
    commanded_target: int
    readback_target: int
    actual_position: int
    error: int
    current_mA: int
    force_g: int
    fault: int
    pause_response: StatusFeedback
    pause_after_target: int
    pause_after_actual: int


class Id4OffsetAuditedBus:
    """Permit only the exact ID4-OFFSET read/write sequence."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.write_frames: list[Frame] = []
        self.read_register_count = 0
        self.read_status_count = 0

    @staticmethod
    def _write_values(frame: Frame) -> tuple[int, ...]:
        if len(frame.data) % 2:
            raise SafetyViolation("ID4-OFFSET write payload has odd byte length")
        return tuple(
            int.from_bytes(frame.data[index : index + 2], "little")
            for index in range(0, len(frame.data), 2)
        )

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if parsed.motor_id != MOTOR_ID:
            raise SafetyViolation(
                f"destination ID={parsed.motor_id} is forbidden; ID4-OFFSET is ID4 only"
            )
        if expected_id is not None and expected_id != MOTOR_ID:
            raise SafetyViolation(
                f"expected_id={expected_id} is forbidden; ID4-OFFSET is ID4 only"
            )
        if parsed.command == CMD_RD_REGISTER:
            if parsed.register_addr not in READ_REGISTERS or parsed.data != b"\x01":
                raise SafetyViolation(
                    f"register read 0x{parsed.register_addr:04X} is outside "
                    "the single-register ID4-OFFSET allowlist"
                )
            self.read_register_count += 1
        elif parsed.command == CMD_RD_STATUS:
            if parsed.register_addr != 0 or parsed.data:
                raise SafetyViolation("malformed status read in ID4-OFFSET")
            self.read_status_count += 1
        elif parsed.command == CMD_WR_REGISTER:
            write_index = len(self.write_frames)
            if write_index >= len(EXPECTED_WRITES):
                raise SafetyViolation("unexpected extra write in ID4-OFFSET")
            expected_register, expected_values = EXPECTED_WRITES[write_index]
            actual_values = self._write_values(parsed)
            if (
                parsed.register_addr != expected_register
                or actual_values != expected_values
            ):
                raise SafetyViolation(
                    "write sequence mismatch: "
                    f"index={write_index} register=0x{parsed.register_addr:04X} "
                    f"values={actual_values} expected_register="
                    f"0x{expected_register:04X} expected_values={expected_values}"
                )
            self.write_frames.append(parsed)
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=4 register=0x%04X "
                "values=%s verified=true",
                parsed.register_addr,
                actual_values,
            )
        else:
            raise SafetyViolation(
                f"outgoing command 0x{parsed.command:02X} is outside ID4-OFFSET"
            )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        destinations = [frame.motor_id for frame in self.write_frames]
        registers = [f"0x{frame.register_addr:04X}" for frame in self.write_frames]
        return (
            f"ACCESS_SUMMARY write_count={len(self.write_frames)} "
            f"write_destinations={destinations} write_registers={registers} "
            f"read_register_count={self.read_register_count} "
            f"read_status_count={self.read_status_count} broadcast_write_count=0 "
            "other_id_write_count=0 calibration_write_count=0 "
            "flash_write_count=0 compensation_write_count=0"
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


def _signed_u16(value: int) -> int:
    if not 0 <= value <= 0xFFFF:
        raise SafetyViolation(f"raw register value={value} is outside uint16")
    return value - 0x10000 if value & 0x8000 else value


def _read_raw_position(motor: Motor, register_addr: int) -> RawPosition:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise SafetyViolation(
            f"ID4 register 0x{register_addr:02X} returned "
            f"{len(response.values)} values"
        )
    raw_u16 = response.values[0]
    return RawPosition(raw_u16=raw_u16, signed=_signed_u16(raw_u16))


def _read_unsigned(motor: Motor, register_addr: int) -> int:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise SafetyViolation(
            f"ID4 register 0x{register_addr:02X} returned "
            f"{len(response.values)} values"
        )
    return response.values[0]


def _check_electrical(status: StatusFeedback) -> None:
    if status.error_code != 0:
        raise SafetyViolation(f"ID4 fault=0x{status.error_code:02X}")
    if status.current_mA > MAX_CURRENT_MA:
        raise SafetyViolation(
            f"ID4 current={status.current_mA}mA exceeds {MAX_CURRENT_MA}mA"
        )


def _check_normal_status(status: StatusFeedback) -> None:
    _check_electrical(status)
    if not DEVICE_LOWER_LIMIT <= status.current_position <= DEVICE_UPPER_LIMIT:
        raise SafetyViolation(
            f"ID4 actual={status.current_position} outside unchanged "
            f"[{DEVICE_LOWER_LIMIT},{DEVICE_UPPER_LIMIT}] gate"
        )


def _check_entry_transition(
    status: StatusFeedback, *, entered_normal: bool, commanded_target: int
) -> bool:
    _check_electrical(status)
    actual = status.current_position
    if entered_normal:
        _check_normal_status(status)
        return True
    if actual < ENTRY_POSITION or actual > commanded_target:
        raise SafetyViolation(
            f"ID4 entry transition actual={actual} outside "
            f"[{ENTRY_POSITION},{commanded_target}]"
        )
    return actual >= DEVICE_LOWER_LIMIT


def _verify_entry_state(motor: Motor) -> None:
    for index in range(1, ENTRY_SAMPLE_COUNT + 1):
        target = _read_raw_position(motor, TARGET_POSITION_ADDR)
        actual = _read_raw_position(motor, ACTUAL_POSITION_ADDR)
        status = motor.read_status()
        exact_minus2 = (
            target.raw_u16 == ENTRY_RAW_U16
            and actual.raw_u16 == ENTRY_RAW_U16
            and target.signed == ENTRY_POSITION
            and actual.signed == ENTRY_POSITION
            and status.target_position == ENTRY_POSITION
            and status.current_position == ENTRY_POSITION
            and status.current_mA == 0
            and status.error_code == 0
        )
        logger.info(
            "ENTRY_SAMPLE index=%d target_raw_u16=%d target_raw_hex=0x%04X "
            "target_signed=%d actual_raw_u16=%d actual_raw_hex=0x%04X "
            "actual_signed=%d status_target=%d status_actual=%d current_mA=%d "
            "force_g=%d fault=0x%02X exact_persistent_minus2=%s",
            index,
            target.raw_u16,
            target.raw_u16,
            target.signed,
            actual.raw_u16,
            actual.raw_u16,
            actual.signed,
            status.target_position,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.error_code,
            exact_minus2,
        )
        if not exact_minus2:
            raise SafetyViolation(
                "ID4 restricted entry rejected: target/actual/status must all be "
                "exactly raw 0xFFFE (-2), current=0, fault=0"
            )
        if index < ENTRY_SAMPLE_COUNT:
            time.sleep(ENTRY_SAMPLE_INTERVAL_S)


def _log_status(label: str, target: int, status: StatusFeedback) -> None:
    logger.info(
        "%s commanded_target=%d status_target=%d actual_position=%d error=%d "
        "current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
        label,
        target,
        status.target_position,
        status.current_position,
        status.current_position - target,
        status.current_mA,
        status.force_g,
        status.temperature,
        status.error_code,
    )


def _pause_once(motor: Motor, target: int) -> StatusFeedback:
    try:
        response = motor.write_register(PAUSE_ADDR, [1])
    except Exception as exc:
        _hard_stop(f"ID4 pause command failed: {exc}")
        raise
    _check_electrical(response)
    logger.warning(
        "PAUSE_RESPONSE destination_id=4 commanded_target=%d "
        "status_target=%d actual_position=%d current_mA=%d force_g=%d "
        "fault=0x%02X sent_once=true",
        target,
        response.target_position,
        response.current_position,
        response.current_mA,
        response.force_g,
        response.error_code,
    )
    return response


def _confirm_stopped(
    motor: Motor, target: int, *, allow_entry_position: bool = False
) -> StatusFeedback:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses: list[StatusFeedback] = []
    while True:
        status = motor.read_status()
        _check_electrical(status)
        if allow_entry_position:
            if not ENTRY_POSITION <= status.current_position <= DEVICE_UPPER_LIMIT:
                raise SafetyViolation(
                    f"post-pause entry actual={status.current_position} is unsafe"
                )
        else:
            _check_normal_status(status)
        statuses.append(status)
        _log_status("PAUSE_AFTER_SAMPLE", target, status)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    stable, metrics = _stop_confirmation_metrics(statuses)
    logger.info(
        "STOP_CONFIRMATION stable=%s reason=%s samples=%s position_min=%s "
        "position_max=%s max_current_mA=%s faults=%s",
        stable,
        metrics["reason"],
        metrics["samples"],
        metrics["position_min"],
        metrics["position_max"],
        metrics["max_current_mA"],
        metrics["faults"],
    )
    if not stable:
        _hard_stop(f"ID4 stop confirmation failed: {metrics['reason']}")
        raise SafetyViolation("ID4 stop confirmation failed")
    return statuses[-1]


def _classify_points(points: list[PointResult]) -> tuple[str, str]:
    if len(points) != len(TARGETS):
        return "INCONCLUSIVE", "INCOMPLETE_POINTS"
    if tuple(point.commanded_target for point in points) != TARGETS:
        return "FAIL", "TARGET_SEQUENCE_MISMATCH"
    if any(point.readback_target != point.commanded_target for point in points):
        return "FAIL", "PRE_PAUSE_TARGET_READBACK_MISMATCH"
    errors = [point.error for point in points]
    if any(error < KNOWN_RESIDUAL_MIN or error > KNOWN_RESIDUAL_MAX for error in errors):
        return "FAIL", "POSITION_ERROR_OUTSIDE_0_TO_3_STEP_BAND"
    if all(error == -2 for error in errors):
        return "PASS", "FIXED_MINUS2_ZERO_REFERENCE_OFFSET_SUPPORTED"
    if all(error == 0 for error in errors):
        return "PASS", "ZERO_ENDPOINT_ONLY_SUPPORTED"
    if len(set(errors)) > 1:
        return "PASS", "ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED"
    return "INCONCLUSIVE", "CONSTANT_NON_MINUS2_ERROR"


def _confirm_operator() -> None:
    answer = input(
        "确认仅连接并固定 ID4、正方向 0~40 step 区域安全、急停可用；"
        "输入 ID4-OFFSET MOVE ID4 TARGETS 10 20 30 40："
    ).strip()
    if answer != "ID4-OFFSET MOVE ID4 TARGETS 10 20 30 40":
        raise SafetyViolation("operator did not arm ID4-OFFSET")


def _hard_stop(reason: str) -> None:
    message = (
        f"HARD STOP REQUIRED: {reason}; use the hardware emergency stop "
        "or remove ID4 motor power immediately"
    )
    logger.critical(message)
    print(message, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/id4_offset/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id4_offset.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited: Optional[Id4OffsetAuditedBus] = None
    motor: Optional[Motor] = None
    points: list[PointResult] = []
    active_target: Optional[int] = None
    movement_write_attempted = False
    pause_attempted = False
    stop_confirmed = False
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited = Id4OffsetAuditedBus(bus)
        motor = Motor(audited, MOTOR_ID)

        lower = _read_unsigned(motor, LOWER_LIMIT_ADDR)
        upper = _read_unsigned(motor, UPPER_LIMIT_ADDR)
        if (lower, upper) != (DEVICE_LOWER_LIMIT, DEVICE_UPPER_LIMIT):
            raise SafetyViolation(
                f"ID4 hardware limits=[{lower},{upper}] do not match unchanged "
                f"[{DEVICE_LOWER_LIMIT},{DEVICE_UPPER_LIMIT}] gate"
            )
        _verify_entry_state(motor)
        logger.info(
            "ID4_OFFSET_PRECHECK entry_samples=%d raw_entry=0x%04X signed_entry=%d "
            "limits=[%d,%d] targets=%s speed=%d timeout_s=%.1f "
            "max_current_mA=%d global_gate_unchanged=true",
            ENTRY_SAMPLE_COUNT,
            ENTRY_RAW_U16,
            ENTRY_POSITION,
            lower,
            upper,
            TARGETS,
            SPEED_STEPS_PER_SECOND,
            MOTION_TIMEOUT_S,
            MAX_CURRENT_MA,
        )
        print(
            "ID4-OFFSET PRECHECK entry=-2 exact=true samples="
            f"{ENTRY_SAMPLE_COUNT} limits=[0,2000] targets={TARGETS}"
        )
        _confirm_operator()

        previous_actual = ENTRY_POSITION
        for index, target in enumerate(TARGETS):
            active_target = target
            pause_attempted = False
            stop_confirmed = False
            if index == 0:
                logger.warning(
                    "RESTRICTED_ENTRY_ARMED destination_id=4 from_raw=-2 "
                    "atomic_mode=2 speed=50 first_target=10 global_gate_unchanged=true"
                )
                movement_write_attempted = True
                response = motor.write_register(
                    CONTROL_MODE_ADDR,
                    [SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, target],
                )
            else:
                before = motor.read_status()
                _check_normal_status(before)
                previous_actual = before.current_position
                movement_write_attempted = True
                response = motor.write_register(TARGET_POSITION_ADDR, [target])

            entered_normal = index != 0
            if index == 0:
                entered_normal = _check_entry_transition(
                    response,
                    entered_normal=entered_normal,
                    commanded_target=target,
                )
            else:
                _check_normal_status(response)
                if response.current_position > target:
                    raise SafetyViolation(
                        f"ID4 actual={response.current_position} exceeded "
                        f"commanded target={target}"
                    )
            _log_status("TARGET_WRITE_RESPONSE", target, response)
            positive_motion = response.current_position > previous_actual
            readback_after_write = _read_raw_position(motor, TARGET_POSITION_ADDR)
            if readback_after_write.signed != target:
                raise SafetyViolation(
                    f"ID4 target write readback mismatch commanded={target} "
                    f"readback={readback_after_write.signed}"
                )
            logger.info(
                "TARGET_WRITE_READBACK commanded_target=%d raw_u16=%d "
                "raw_hex=0x%04X readback_target_position=%d",
                target,
                readback_after_write.raw_u16,
                readback_after_write.raw_u16,
                readback_after_write.signed,
            )

            deadline = time.monotonic() + MOTION_TIMEOUT_S
            while True:
                status = motor.read_status()
                if index == 0:
                    entered_normal = _check_entry_transition(
                        status,
                        entered_normal=entered_normal,
                        commanded_target=target,
                    )
                else:
                    _check_normal_status(status)
                    if status.current_position > target:
                        raise SafetyViolation(
                            f"ID4 actual={status.current_position} exceeded "
                            f"commanded target={target}"
                        )
                    if status.current_position < previous_actual - 1:
                        raise SafetyViolation(
                            f"ID4 negative movement at target={target}: "
                            f"start={previous_actual} actual={status.current_position}"
                        )
                positive_motion = positive_motion or (
                    status.current_position > previous_actual
                )
                _log_status("MOTION_SAMPLE", target, status)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(POLL_INTERVAL_S, remaining))
            if not entered_normal:
                raise SafetyViolation(
                    "ID4 did not leave restricted raw -2 entry and enter [0,2000]"
                )
            if not positive_motion:
                raise SafetyViolation(
                    f"ID4 positive motion not observed for target={target}"
                )

            readback_target = _read_raw_position(motor, TARGET_POSITION_ADDR)
            raw_actual = _read_raw_position(motor, ACTUAL_POSITION_ADDR)
            pause_before = motor.read_status()
            _check_normal_status(pause_before)
            if readback_target.signed != target:
                raise SafetyViolation(
                    f"ID4 pre-pause target mismatch commanded={target} "
                    f"readback={readback_target.signed}"
                )
            logger.info(
                "POINT_PAUSE_BEFORE commanded_target=%d "
                "readback_target_position=%d target_raw_u16=%d "
                "target_raw_hex=0x%04X actual_position=%d actual_raw_u16=%d "
                "actual_raw_hex=0x%04X status_target=%d status_actual=%d error=%d "
                "current_mA=%d force_g=%d fault=0x%02X",
                target,
                readback_target.signed,
                readback_target.raw_u16,
                readback_target.raw_u16,
                pause_before.current_position,
                raw_actual.raw_u16,
                raw_actual.raw_u16,
                pause_before.target_position,
                pause_before.current_position,
                pause_before.current_position - target,
                pause_before.current_mA,
                pause_before.force_g,
                pause_before.error_code,
            )

            pause_attempted = True
            pause_response = _pause_once(motor, target)
            pause_after = _confirm_stopped(motor, target)
            stop_confirmed = True
            pause_after_target = _read_raw_position(motor, TARGET_POSITION_ADDR)
            pause_after_actual = _read_raw_position(motor, ACTUAL_POSITION_ADDR)
            logger.info(
                "POINT_PAUSE_AFTER commanded_target=%d "
                "readback_target_position=%d target_raw_u16=%d "
                "target_raw_hex=0x%04X actual_position=%d actual_raw_u16=%d "
                "actual_raw_hex=0x%04X status_target=%d status_actual=%d "
                "current_mA=%d force_g=%d fault=0x%02X "
                "target_sync_observation_only=true",
                target,
                pause_after_target.signed,
                pause_after_target.raw_u16,
                pause_after_target.raw_u16,
                pause_after_actual.signed,
                pause_after_actual.raw_u16,
                pause_after_actual.raw_u16,
                pause_after.target_position,
                pause_after.current_position,
                pause_after.current_mA,
                pause_after.force_g,
                pause_after.error_code,
            )
            points.append(
                PointResult(
                    commanded_target=target,
                    readback_target=readback_target.signed,
                    actual_position=pause_before.current_position,
                    error=pause_before.current_position - target,
                    current_mA=pause_before.current_mA,
                    force_g=pause_before.force_g,
                    fault=pause_before.error_code,
                    pause_response=pause_response,
                    pause_after_target=pause_after_target.signed,
                    pause_after_actual=pause_after_actual.signed,
                )
            )
            if not KNOWN_RESIDUAL_MIN <= points[-1].error <= KNOWN_RESIDUAL_MAX:
                raise SafetyViolation(
                    f"ID4 target={target} error={points[-1].error} outside "
                    "known -3..0 step residual band; later points are blocked"
                )
            previous_actual = pause_after.current_position
            movement_write_attempted = False
            active_target = None

        decision, classification = _classify_points(points)
        errors = [point.error for point in points]
        logger.info(
            "ID4_OFFSET_RESULT decision=%s classification=%s points=%d "
            "targets=%s errors=%s pause_all=true stop_confirmation_all=true "
            "raw_values_preserved=true compensation_write_count=0",
            decision,
            classification,
            len(points),
            TARGETS,
            errors,
        )
        print(
            f"{decision} ID4-OFFSET classification={classification} "
            f"targets={TARGETS} errors={errors}"
        )
        return 0 if decision == "PASS" else (2 if decision == "INCONCLUSIVE" else 1)
    except Exception as exc:
        if movement_write_attempted and motor is not None and not pause_attempted:
            pause_attempted = True
            try:
                _pause_once(motor, active_target if active_target is not None else -1)
            except Exception:
                pass
        if movement_write_attempted and motor is not None and pause_attempted and not stop_confirmed:
            try:
                _confirm_stopped(
                    motor,
                    active_target if active_target is not None else -1,
                    allow_entry_position=True,
                )
                stop_confirmed = True
            except Exception as confirm_exc:
                _hard_stop(f"ID4 abnormal stop confirmation failed: {confirm_exc}")
        decision = "INCONCLUSIVE" if not movement_write_attempted and points else "FAIL"
        logger.error(
            "ID4_OFFSET_RESULT decision=%s classification=INCOMPLETE points=%d "
            "active_target=%s pause_attempted=%s stop_confirmed=%s reason=%s",
            decision,
            len(points),
            active_target,
            pause_attempted,
            stop_confirmed,
            exc,
        )
        print(f"STOP ID4-OFFSET: {exc}", file=sys.stderr)
        return 2 if decision == "INCONCLUSIVE" else 1
    finally:
        if audited is not None:
            logger.info(audited.summary())
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "ID4-OFFSET: fixed ID4 targets 10/20/30/40 from verified raw -2"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log", help="independent log path; existing files are not overwritten")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("timeout must be greater than zero")
    if args.retries < 0:
        parser.error("retries cannot be negative")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
