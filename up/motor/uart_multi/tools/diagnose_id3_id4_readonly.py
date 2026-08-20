#!/usr/bin/env python3
"""Paired, bounded, read-only state comparison for motor IDs 3 and 4."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import logging
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import (  # noqa: E402
    CMD_FRAME_HEADER,
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    FrameLengthError,
    StatusFeedback,
    parse_frame,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (3, 4)
MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
READ_REGISTER_ADDRS = frozenset(
    (MODE_ADDR, TARGET_SPEED_ADDR, TARGET_POSITION_ADDR)
)
SAMPLE_COUNT = 100
SAMPLE_INTERVAL_S = 0.05

logger = logging.getLogger("diagnose_id3_id4_readonly")


class ReadOnlyViolation(RuntimeError):
    """An outgoing request is outside the fixed ID3/ID4 read-only scope."""


@dataclass(frozen=True)
class MotorState:
    motor_id: int
    mode: int
    target_speed: int
    target_raw_u16: int
    target_position: int
    status: StatusFeedback

    @property
    def comparison_fields(self) -> dict[str, int]:
        return {
            "mode": self.mode,
            "target_speed": self.target_speed,
            "register_target": self.target_position,
            "status_target": self.status.target_position,
            "actual": self.status.current_position,
            "current": self.status.current_mA,
            "force": self.status.force_g,
            "fault": self.status.error_code,
        }


@dataclass(frozen=True)
class PairedSample:
    index: int
    elapsed_s: float
    id3: MotorState
    id4: MotorState


class Id3Id4ReadOnlyAuditedBus:
    """Permit only the fixed ID3/ID4 register and status reads."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.read_register_counts = {motor_id: 0 for motor_id in MOTOR_IDS}
        self.read_status_counts = {motor_id: 0 for motor_id in MOTOR_IDS}

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if parsed.motor_id not in MOTOR_IDS:
            raise ReadOnlyViolation(
                f"destination ID={parsed.motor_id} is outside fixed ID3/ID4 scope"
            )
        if expected_id is not None and expected_id != parsed.motor_id:
            raise ReadOnlyViolation(
                f"expected_id={expected_id} does not match destination "
                f"ID={parsed.motor_id}"
            )
        if parsed.command == CMD_RD_REGISTER:
            if parsed.register_addr not in READ_REGISTER_ADDRS:
                raise ReadOnlyViolation(
                    f"register 0x{parsed.register_addr:04X} is outside read whitelist"
                )
            if parsed.data != b"\x01":
                raise ReadOnlyViolation("only single-register reads are allowed")
            self.read_register_counts[parsed.motor_id] += 1
        elif parsed.command == CMD_RD_STATUS:
            if parsed.register_addr != 0 or parsed.data:
                raise ReadOnlyViolation("malformed status read")
            self.read_status_counts[parsed.motor_id] += 1
        else:
            raise ReadOnlyViolation(
                f"outgoing command 0x{parsed.command:02X} is not read-only"
            )
        logger.info(
            "READ_ONLY_AUDIT destination_id=%d command=0x%02X "
            "register=0x%04X verified=true",
            parsed.motor_id,
            parsed.command,
            parsed.register_addr,
        )
        return self._bus.request(frame, **kwargs)

    def summary(self) -> str:
        return (
            "READ_ONLY_SUMMARY allowed_ids=[3,4] write_count=0 "
            f"id3_read_register_count={self.read_register_counts[3]} "
            f"id3_read_status_count={self.read_status_counts[3]} "
            f"id4_read_register_count={self.read_register_counts[4]} "
            f"id4_read_status_count={self.read_status_counts[4]}"
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
        raise ValueError(f"value={value} is outside uint16")
    return value - 0x10000 if value & 0x8000 else value


def _read_one_register(motor: Motor, address: int) -> int:
    response = motor.read_register(address)
    if len(response.values) != 1:
        raise FrameLengthError(
            f"ID{motor.motor_id} register 0x{address:02X} returned "
            f"{len(response.values)} values"
        )
    return response.values[0]


def _read_state(motor: Motor) -> MotorState:
    mode = _read_one_register(motor, MODE_ADDR)
    target_speed = _read_one_register(motor, TARGET_SPEED_ADDR)
    target_raw_u16 = _read_one_register(motor, TARGET_POSITION_ADDR)
    return MotorState(
        motor_id=motor.motor_id,
        mode=mode,
        target_speed=target_speed,
        target_raw_u16=target_raw_u16,
        target_position=_signed_u16(target_raw_u16),
        status=motor.read_status(),
    )


def _histogram(values: list[int]) -> dict[int, int]:
    return dict(sorted(Counter(values).items()))


def _classification(samples: list[PairedSample], expected_count: int) -> str:
    if len(samples) != expected_count:
        return "INCONCLUSIVE"
    if all(
        sample.id3.target_position == 0
        and sample.id3.status.target_position == 0
        and sample.id3.status.current_position == 0
        and sample.id4.target_position == -2
        and sample.id4.status.target_position == -2
        and sample.id4.status.current_position == -2
        for sample in samples
    ):
        return "PERSISTENT_ID3_ZERO_ID4_MINUS2"
    if all(
        sample.id4.target_position == -2
        and sample.id4.status.target_position == -2
        and sample.id4.status.current_position == -2
        for sample in samples
    ):
        return "PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE"
    if any(
        -2
        in (
            sample.id4.target_position,
            sample.id4.status.target_position,
            sample.id4.status.current_position,
        )
        for sample in samples
    ):
        return "INTERMITTENT_OR_MIXED_ID4_MINUS2"
    return "ID4_MINUS2_NOT_OBSERVED"


def _id_summary(motor_id: int, states: list[MotorState]) -> str:
    fields = (
        ("mode", [state.mode for state in states]),
        ("target_speed", [state.target_speed for state in states]),
        ("register_target", [state.target_position for state in states]),
        ("status_target", [state.status.target_position for state in states]),
        ("actual", [state.status.current_position for state in states]),
        ("current_mA", [state.status.current_mA for state in states]),
        ("force_g", [state.status.force_g for state in states]),
        ("fault", [state.status.error_code for state in states]),
    )
    histograms = " ".join(
        f"{name}_histogram={_histogram(values)}" for name, values in fields
    )
    return f"ID{motor_id}_SUMMARY samples={len(states)} {histograms}"


def _comparison_summary(samples: list[PairedSample], expected_count: int) -> str:
    classification = _classification(samples, expected_count)
    if not samples:
        return (
            f"ID3_ID4_COMPARISON classification={classification} samples=0 "
            f"expected_samples={expected_count} write_count=0"
        )
    field_names = tuple(samples[0].id3.comparison_fields)
    comparisons = []
    for field_name in field_names:
        equal_count = sum(
            sample.id3.comparison_fields[field_name]
            == sample.id4.comparison_fields[field_name]
            for sample in samples
        )
        comparisons.append(f"{field_name}_equal={equal_count}")
        comparisons.append(f"{field_name}_different={len(samples) - equal_count}")
    return (
        f"ID3_ID4_COMPARISON classification={classification} "
        f"samples={len(samples)} expected_samples={expected_count} "
        f"{' '.join(comparisons)} write_count=0"
    )


def _log_sample(sample: PairedSample) -> None:
    for state in (sample.id3, sample.id4):
        logger.info(
            "ID%d_SAMPLE index=%d elapsed_s=%.6f mode=%d target_speed=%d "
            "register_target_position=%d target_raw_u16=%d target_raw_hex=0x%04X "
            "status_target_position=%d actual_position=%d current_mA=%d "
            "force_g=%d fault=0x%02X",
            state.motor_id,
            sample.index,
            sample.elapsed_s,
            state.mode,
            state.target_speed,
            state.target_position,
            state.target_raw_u16,
            state.target_raw_u16,
            state.status.target_position,
            state.status.current_position,
            state.status.current_mA,
            state.status.force_g,
            state.status.error_code,
        )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m4/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id3_id4_readonly_comparison.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus = None
    audited_bus = None
    samples: list[PairedSample] = []
    error = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = Id3Id4ReadOnlyAuditedBus(bus)
        motors = {motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS}
        started = time.monotonic()
        for index in range(1, SAMPLE_COUNT + 1):
            scheduled = started + (index - 1) * SAMPLE_INTERVAL_S
            delay = scheduled - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            id3 = _read_state(motors[3])
            id4 = _read_state(motors[4])
            sample = PairedSample(
                index=index,
                elapsed_s=time.monotonic() - started,
                id3=id3,
                id4=id4,
            )
            samples.append(sample)
            _log_sample(sample)
    except Exception as exc:
        error = exc
        logger.error(
            "ID3_ID4_COMPARISON_ERROR samples=%d reason=%s", len(samples), exc
        )
    finally:
        id3_states = [sample.id3 for sample in samples]
        id4_states = [sample.id4 for sample in samples]
        if id3_states:
            logger.info(_id_summary(3, id3_states))
            logger.info(_id_summary(4, id4_states))
        summary = _comparison_summary(samples, SAMPLE_COUNT)
        logger.info(summary)
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()

    print(summary)
    if error is not None:
        print(f"ERROR ID3/ID4 read-only comparison incomplete: {error}", file=sys.stderr)
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Fixed 100-round ID3/ID4 read-only mode/speed/position/current/force/"
            "fault comparison"
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
