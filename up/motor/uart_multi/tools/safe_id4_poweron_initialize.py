#!/usr/bin/env python3
"""One-shot ID4 power-on recovery from the persistent raw -2 endpoint."""

from __future__ import annotations

import argparse
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
CONTROL_MODE_ADDR = 0x25
TARGET_POSITION_ADDR = 0x29
ACTUAL_POSITION_ADDR = 0x2A
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
RECOVERY_TARGET = 10
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
KNOWN_RESIDUAL_STEPS = 3
READ_REGISTERS = {
    LOWER_LIMIT_ADDR,
    UPPER_LIMIT_ADDR,
    TARGET_POSITION_ADDR,
    ACTUAL_POSITION_ADDR,
}
EXPECTED_WRITES = (
    (
        CONTROL_MODE_ADDR,
        (SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, RECOVERY_TARGET),
    ),
    (PAUSE_ADDR, (1,)),
)

logger = logging.getLogger("safe_id4_poweron_initialize")


class Id4InitializeAuditedBus:
    """Permit only ID4 reads and the exact setup-then-pause write sequence."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.write_frames: list[Frame] = []
        self.read_register_count = 0
        self.read_status_count = 0

    @staticmethod
    def _write_values(frame: Frame) -> tuple[int, ...]:
        if len(frame.data) % 2:
            raise SafetyViolation("ID4 initialization write has odd payload length")
        return tuple(
            int.from_bytes(frame.data[index : index + 2], "little")
            for index in range(0, len(frame.data), 2)
        )

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if parsed.motor_id != MOTOR_ID:
            raise SafetyViolation(
                f"destination ID={parsed.motor_id} is forbidden; initialization is ID4 only"
            )
        if expected_id is not None and expected_id != MOTOR_ID:
            raise SafetyViolation(
                f"expected_id={expected_id} is forbidden; initialization is ID4 only"
            )
        if parsed.command == CMD_RD_REGISTER:
            if parsed.register_addr not in READ_REGISTERS or parsed.data != b"\x01":
                raise SafetyViolation(
                    f"register read 0x{parsed.register_addr:04X} is outside "
                    "the initialization allowlist"
                )
            self.read_register_count += 1
        elif parsed.command == CMD_RD_STATUS:
            if parsed.register_addr != 0 or parsed.data:
                raise SafetyViolation("malformed ID4 initialization status read")
            self.read_status_count += 1
        elif parsed.command == CMD_WR_REGISTER:
            write_index = len(self.write_frames)
            if write_index >= len(EXPECTED_WRITES):
                raise SafetyViolation("unexpected extra ID4 initialization write")
            expected_register, expected_values = EXPECTED_WRITES[write_index]
            actual_values = self._write_values(parsed)
            if (
                parsed.register_addr != expected_register
                or actual_values != expected_values
            ):
                raise SafetyViolation(
                    "initialization write sequence mismatch: "
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
                f"outgoing command 0x{parsed.command:02X} is outside initialization"
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


def _read_raw(motor: Motor, register_addr: int) -> tuple[int, int]:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise SafetyViolation(
            f"ID4 register 0x{register_addr:02X} returned "
            f"{len(response.values)} values"
        )
    raw_u16 = response.values[0]
    return raw_u16, _signed_u16(raw_u16)


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


def _check_transition(status: StatusFeedback, *, entered_normal: bool) -> bool:
    _check_electrical(status)
    actual = status.current_position
    if entered_normal:
        if not DEVICE_LOWER_LIMIT <= actual <= RECOVERY_TARGET:
            raise SafetyViolation(
                f"ID4 actual={actual} outside restored recovery envelope "
                f"[{DEVICE_LOWER_LIMIT},{RECOVERY_TARGET}]"
            )
        return True
    if not ENTRY_POSITION <= actual <= RECOVERY_TARGET:
        raise SafetyViolation(
            f"ID4 entry actual={actual} outside [{ENTRY_POSITION},{RECOVERY_TARGET}]"
        )
    return actual >= DEVICE_LOWER_LIMIT


def _check_normal(status: StatusFeedback) -> None:
    _check_electrical(status)
    if not DEVICE_LOWER_LIMIT <= status.current_position <= DEVICE_UPPER_LIMIT:
        raise SafetyViolation(
            f"ID4 actual={status.current_position} outside unchanged "
            f"[{DEVICE_LOWER_LIMIT},{DEVICE_UPPER_LIMIT}] gate"
        )


def _verify_entry(motor: Motor) -> str:
    exact_minus2_count = 0
    legal_no_action_count = 0
    for index in range(1, ENTRY_SAMPLE_COUNT + 1):
        target_raw, target = _read_raw(motor, TARGET_POSITION_ADDR)
        actual_raw, actual = _read_raw(motor, ACTUAL_POSITION_ADDR)
        status = motor.read_status()
        exact = (
            target_raw == ENTRY_RAW_U16
            and actual_raw == ENTRY_RAW_U16
            and target == ENTRY_POSITION
            and actual == ENTRY_POSITION
            and status.target_position == ENTRY_POSITION
            and status.current_position == ENTRY_POSITION
            and status.current_mA == 0
            and status.error_code == 0
        )
        legal_no_action = (
            DEVICE_LOWER_LIMIT <= target <= DEVICE_UPPER_LIMIT
            and DEVICE_LOWER_LIMIT <= actual <= DEVICE_UPPER_LIMIT
            and DEVICE_LOWER_LIMIT <= status.target_position <= DEVICE_UPPER_LIMIT
            and DEVICE_LOWER_LIMIT <= status.current_position <= DEVICE_UPPER_LIMIT
            and status.current_mA == 0
            and status.error_code == 0
        )
        exact_minus2_count += int(exact)
        legal_no_action_count += int(legal_no_action)
        logger.info(
            "ENTRY_SAMPLE index=%d target_raw_u16=%d target_raw_hex=0x%04X "
            "target_signed=%d actual_raw_u16=%d actual_raw_hex=0x%04X "
            "actual_signed=%d status_target=%d status_actual=%d current_mA=%d "
            "force_g=%d fault=0x%02X exact_persistent_minus2=%s "
            "legal_no_action=%s",
            index,
            target_raw,
            target_raw,
            target,
            actual_raw,
            actual_raw,
            actual,
            status.target_position,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.error_code,
            exact,
            legal_no_action,
        )
        if index < ENTRY_SAMPLE_COUNT:
            time.sleep(ENTRY_SAMPLE_INTERVAL_S)
    if exact_minus2_count == ENTRY_SAMPLE_COUNT:
        return "RECOVERY_REQUIRED"
    if legal_no_action_count == ENTRY_SAMPLE_COUNT:
        return "ALREADY_IN_RANGE_NO_ACTION"
    raise SafetyViolation(
        "restricted initialization rejected: samples are neither consistently "
        "raw -2/current 0/fault 0 nor consistently legal [0,2000]/current 0/fault 0"
    )


def _pause_once(motor: Motor) -> StatusFeedback:
    try:
        response = motor.write_register(PAUSE_ADDR, [1])
    except Exception as exc:
        _hard_stop(f"ID4 initialization pause failed: {exc}")
        raise
    _check_electrical(response)
    logger.warning(
        "PAUSE_RESPONSE destination_id=4 status_target=%d actual_position=%d "
        "current_mA=%d force_g=%d fault=0x%02X sent_once=true",
        response.target_position,
        response.current_position,
        response.current_mA,
        response.force_g,
        response.error_code,
    )
    return response


def _confirm_stopped(motor: Motor, *, allow_entry_position: bool = False) -> StatusFeedback:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses: list[StatusFeedback] = []
    while True:
        status = motor.read_status()
        _check_electrical(status)
        if allow_entry_position:
            if not ENTRY_POSITION <= status.current_position <= RECOVERY_TARGET:
                raise SafetyViolation(
                    f"ID4 abnormal-stop actual={status.current_position} is unsafe"
                )
        else:
            _check_normal(status)
        statuses.append(status)
        logger.info(
            "STOP_SAMPLE target=%d actual=%d current_mA=%d force_g=%d fault=0x%02X",
            status.target_position,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.error_code,
        )
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
        _hard_stop(f"ID4 initialization stop confirmation failed: {metrics['reason']}")
        raise SafetyViolation("ID4 initialization stop confirmation failed")
    return statuses[-1]


def _confirm_operator() -> None:
    answer = input(
        "确认 ID4 正方向 -2~10 step 区域安全、ID4 保持供电直至后续复核、"
        "急停可用；输入 ID4-INIT RECOVER ID4 FROM -2 TO 10："
    ).strip()
    if answer != "ID4-INIT RECOVER ID4 FROM -2 TO 10":
        raise SafetyViolation("operator did not arm ID4 initialization")


def _hard_stop(reason: str) -> None:
    message = (
        f"HARD STOP REQUIRED: {reason}; use the hardware emergency stop "
        "or remove ID4 motor power immediately"
    )
    logger.critical(message)
    print(message, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/id4_init/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id4_poweron_init.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited: Optional[Id4InitializeAuditedBus] = None
    motor: Optional[Motor] = None
    movement_write_attempted = False
    pause_attempted = False
    stop_confirmed = False
    entered_normal = False
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited = Id4InitializeAuditedBus(bus)
        motor = Motor(audited, MOTOR_ID)

        lower = _read_unsigned(motor, LOWER_LIMIT_ADDR)
        upper = _read_unsigned(motor, UPPER_LIMIT_ADDR)
        if (lower, upper) != (DEVICE_LOWER_LIMIT, DEVICE_UPPER_LIMIT):
            raise SafetyViolation(
                f"ID4 limits=[{lower},{upper}] do not match unchanged [0,2000]"
            )
        entry_state = _verify_entry(motor)
        logger.info(
            "ID4_INIT_PRECHECK entry_samples=%d raw_entry=0x%04X "
            "limits=[0,2000] recovery_target=%d speed=%d timeout_s=%.1f "
            "max_current_mA=%d entry_state=%s global_gate_unchanged=true",
            ENTRY_SAMPLE_COUNT,
            ENTRY_RAW_U16,
            RECOVERY_TARGET,
            SPEED_STEPS_PER_SECOND,
            MOTION_TIMEOUT_S,
            MAX_CURRENT_MA,
            entry_state,
        )
        print(
            f"ID4-INIT PRECHECK entry_state={entry_state} "
            f"samples={ENTRY_SAMPLE_COUNT} limits=[0,2000] target={RECOVERY_TARGET}"
        )
        if entry_state == "ALREADY_IN_RANGE_NO_ACTION":
            logger.info(
                "ID4_INIT_RESULT decision=PASS "
                "classification=ALREADY_IN_RANGE_NO_ACTION write_count=0 "
                "global_gate_restored=true compensation_write_count=0"
            )
            print(
                "PASS ID4-INIT classification=ALREADY_IN_RANGE_NO_ACTION "
                "write_count=0 global_gate_restored=true"
            )
            return 0
        _confirm_operator()

        movement_write_attempted = True
        response = motor.write_register(
            CONTROL_MODE_ADDR,
            [SPEED_MODE, 0, 0, SPEED_STEPS_PER_SECOND, RECOVERY_TARGET],
        )
        entered_normal = _check_transition(
            response, entered_normal=entered_normal
        )
        positive_motion = response.current_position > ENTRY_POSITION
        logger.info(
            "RECOVERY_WRITE_RESPONSE commanded_target=%d status_target=%d "
            "actual=%d current_mA=%d force_g=%d fault=0x%02X",
            RECOVERY_TARGET,
            response.target_position,
            response.current_position,
            response.current_mA,
            response.force_g,
            response.error_code,
        )
        target_raw, target_readback = _read_raw(motor, TARGET_POSITION_ADDR)
        if target_readback != RECOVERY_TARGET:
            raise SafetyViolation(
                f"ID4 recovery target readback={target_readback}, "
                f"expected={RECOVERY_TARGET}"
            )
        logger.info(
            "TARGET_WRITE_READBACK commanded_target=%d raw_u16=%d "
            "raw_hex=0x%04X readback_target_position=%d",
            RECOVERY_TARGET,
            target_raw,
            target_raw,
            target_readback,
        )

        deadline = time.monotonic() + MOTION_TIMEOUT_S
        reached_residual_band = False
        while True:
            status = motor.read_status()
            entered_normal = _check_transition(
                status, entered_normal=entered_normal
            )
            positive_motion = positive_motion or (
                status.current_position > ENTRY_POSITION
            )
            error = RECOVERY_TARGET - status.current_position
            reached_residual_band = entered_normal and 0 <= error <= KNOWN_RESIDUAL_STEPS
            logger.info(
                "RECOVERY_SAMPLE commanded_target=%d status_target=%d actual=%d "
                "error=%d current_mA=%d force_g=%d fault=0x%02X "
                "entered_normal=%s residual_band=%s",
                RECOVERY_TARGET,
                status.target_position,
                status.current_position,
                status.current_position - RECOVERY_TARGET,
                status.current_mA,
                status.force_g,
                status.error_code,
                entered_normal,
                reached_residual_band,
            )
            if positive_motion and reached_residual_band:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SafetyViolation(
                    "ID4 did not enter [0,2000] and reach target 10 within "
                    "the known 0..3 step residual band in 2.0s"
                )
            time.sleep(min(POLL_INTERVAL_S, remaining))

        pre_target_raw, pre_target = _read_raw(motor, TARGET_POSITION_ADDR)
        pre_actual_raw, pre_actual = _read_raw(motor, ACTUAL_POSITION_ADDR)
        pre_pause = motor.read_status()
        _check_normal(pre_pause)
        if pre_target != RECOVERY_TARGET:
            raise SafetyViolation(
                f"ID4 pre-pause target={pre_target}, expected={RECOVERY_TARGET}"
            )
        if not 0 <= RECOVERY_TARGET - pre_pause.current_position <= KNOWN_RESIDUAL_STEPS:
            raise SafetyViolation(
                f"ID4 pre-pause actual={pre_pause.current_position} is outside "
                "the target 10 residual band"
            )
        logger.info(
            "INIT_PAUSE_BEFORE commanded_target=%d readback_target=%d "
            "target_raw_u16=%d target_raw_hex=0x%04X actual=%d "
            "actual_raw_u16=%d actual_raw_hex=0x%04X error=%d current_mA=%d "
            "force_g=%d fault=0x%02X",
            RECOVERY_TARGET,
            pre_target,
            pre_target_raw,
            pre_target_raw,
            pre_pause.current_position,
            pre_actual_raw,
            pre_actual_raw,
            pre_pause.current_position - RECOVERY_TARGET,
            pre_pause.current_mA,
            pre_pause.force_g,
            pre_pause.error_code,
        )

        pause_attempted = True
        _pause_once(motor)
        final = _confirm_stopped(motor)
        stop_confirmed = True
        post_target_raw, post_target = _read_raw(motor, TARGET_POSITION_ADDR)
        post_actual_raw, post_actual = _read_raw(motor, ACTUAL_POSITION_ADDR)
        if not DEVICE_LOWER_LIMIT <= post_target <= DEVICE_UPPER_LIMIT:
            raise SafetyViolation(
                f"ID4 post-pause raw target={post_target} is outside [0,2000]"
            )
        if not DEVICE_LOWER_LIMIT <= post_actual <= DEVICE_UPPER_LIMIT:
            raise SafetyViolation(
                f"ID4 post-pause raw actual={post_actual} is outside [0,2000]"
            )
        logger.info(
            "ID4_INIT_RESULT decision=PASS "
            "classification=ID4_POWERON_RECOVERY_COMPLETE commanded_target=%d "
            "final_target=%d final_actual=%d final_error=%d "
            "target_raw_u16=%d target_raw_hex=0x%04X actual_raw_u16=%d "
            "actual_raw_hex=0x%04X current_mA=%d force_g=%d fault=0x%02X "
            "pause_success=true stop_confirmation=true "
            "global_gate_restored=true compensation_write_count=0",
            RECOVERY_TARGET,
            post_target,
            post_actual,
            post_actual - RECOVERY_TARGET,
            post_target_raw,
            post_target_raw,
            post_actual_raw,
            post_actual_raw,
            final.current_mA,
            final.force_g,
            final.error_code,
        )
        print(
            "PASS ID4-INIT classification=ID4_POWERON_RECOVERY_COMPLETE "
            f"target={RECOVERY_TARGET} final={post_actual} "
            "global_gate_restored=true"
        )
        return 0
    except Exception as exc:
        if movement_write_attempted and motor is not None and not pause_attempted:
            pause_attempted = True
            try:
                _pause_once(motor)
            except Exception:
                pass
        if movement_write_attempted and motor is not None and pause_attempted and not stop_confirmed:
            try:
                _confirm_stopped(motor, allow_entry_position=not entered_normal)
                stop_confirmed = True
            except Exception as confirm_exc:
                _hard_stop(f"ID4 initialization abnormal stop failed: {confirm_exc}")
        logger.error(
            "ID4_INIT_RESULT decision=FAIL classification=NOT_INITIALIZED "
            "entered_normal=%s pause_attempted=%s stop_confirmed=%s reason=%s",
            entered_normal,
            pause_attempted,
            stop_confirmed,
            exc,
        )
        print(f"STOP ID4-INIT: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited is not None:
            logger.info(audited.summary())
        if bus is not None:
            bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="One-shot ID4 raw -2 to target 10 power-on recovery"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log", help="independent log path; never overwritten")
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
