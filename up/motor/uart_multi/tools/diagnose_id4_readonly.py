#!/usr/bin/env python3
"""Bounded read-only diagnosis of the anomalous ID4 position state."""

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


MOTOR_ID = 4
TARGET_POSITION_ADDR = 0x29
SAMPLE_COUNT = 100
SAMPLE_INTERVAL_S = 0.05

logger = logging.getLogger("diagnose_id4_readonly")


class ReadOnlyViolation(RuntimeError):
    """An outgoing request is outside the fixed ID4 read-only scope."""


@dataclass(frozen=True)
class Id4Sample:
    index: int
    elapsed_s: float
    target_raw_u16: int
    target_position: int
    status: StatusFeedback

    @property
    def all_position_fields_minus_two(self) -> bool:
        return (
            self.target_position == -2
            and self.status.target_position == -2
            and self.status.current_position == -2
        )


class Id4ReadOnlyAuditedBus:
    """Permit only ID4 register/status reads and reject every other frame."""

    def __init__(self, bus: UartBus) -> None:
        self._bus = bus
        self.read_register_count = 0
        self.read_status_count = 0

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame, expected_header=CMD_FRAME_HEADER)
        expected_id = kwargs.get("expected_id")
        if parsed.motor_id != MOTOR_ID:
            raise ReadOnlyViolation(
                f"destination ID={parsed.motor_id} is outside fixed ID4 scope"
            )
        if expected_id is not None and expected_id != MOTOR_ID:
            raise ReadOnlyViolation(
                f"expected_id={expected_id} is outside fixed ID4 scope"
            )
        if parsed.command == CMD_RD_REGISTER:
            self.read_register_count += 1
        elif parsed.command == CMD_RD_STATUS:
            self.read_status_count += 1
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
            "READ_ONLY_SUMMARY destination_id=4 write_count=0 "
            f"read_register_count={self.read_register_count} "
            f"read_status_count={self.read_status_count}"
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


def _read_target_position(motor: Motor) -> tuple[int, int]:
    response = motor.read_register(TARGET_POSITION_ADDR)
    if len(response.values) != 1:
        raise FrameLengthError(
            f"ID4 register 0x29 returned {len(response.values)} values"
        )
    raw_u16 = response.values[0]
    return raw_u16, _signed_u16(raw_u16)


def _classification(samples: list[Id4Sample], expected_count: int) -> str:
    if len(samples) != expected_count:
        return "INCONCLUSIVE"
    if all(sample.all_position_fields_minus_two for sample in samples):
        return "PERSISTENT_MINUS2"
    if any(
        -2
        in (
            sample.target_position,
            sample.status.target_position,
            sample.status.current_position,
        )
        for sample in samples
    ):
        return "INTERMITTENT_OR_MIXED_MINUS2"
    return "MINUS2_NOT_OBSERVED"


def _histogram(values: list[int]) -> dict[int, int]:
    return dict(sorted(Counter(values).items()))


def _summary(samples: list[Id4Sample], expected_count: int) -> str:
    classification = _classification(samples, expected_count)
    if not samples:
        return (
            f"ID4_DIAGNOSIS classification={classification} "
            f"samples=0 expected_samples={expected_count} write_count=0"
        )
    register_targets = [sample.target_position for sample in samples]
    status_targets = [sample.status.target_position for sample in samples]
    actuals = [sample.status.current_position for sample in samples]
    currents = [sample.status.current_mA for sample in samples]
    forces = [sample.status.force_g for sample in samples]
    faults = [sample.status.error_code for sample in samples]
    consistent_minus2_count = sum(
        sample.all_position_fields_minus_two for sample in samples
    )
    return (
        f"ID4_DIAGNOSIS classification={classification} "
        f"samples={len(samples)} expected_samples={expected_count} "
        f"register_target_histogram={_histogram(register_targets)} "
        f"status_target_histogram={_histogram(status_targets)} "
        f"actual_histogram={_histogram(actuals)} "
        f"consistent_minus2_count={consistent_minus2_count} "
        f"current_min_mA={min(currents)} current_max_mA={max(currents)} "
        f"force_min_g={min(forces)} force_max_g={max(forces)} "
        f"fault_histogram={_histogram(faults)} write_count=0"
    )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m4/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id4_readonly_diagnosis.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus = None
    audited_bus = None
    samples: list[Id4Sample] = []
    error = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = Id4ReadOnlyAuditedBus(bus)
        motor = Motor(audited_bus, MOTOR_ID)
        started = time.monotonic()
        for index in range(1, SAMPLE_COUNT + 1):
            scheduled = started + (index - 1) * SAMPLE_INTERVAL_S
            delay = scheduled - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            target_raw_u16, target_position = _read_target_position(motor)
            status = motor.read_status()
            elapsed_s = time.monotonic() - started
            sample = Id4Sample(
                index=index,
                elapsed_s=elapsed_s,
                target_raw_u16=target_raw_u16,
                target_position=target_position,
                status=status,
            )
            samples.append(sample)
            logger.info(
                "ID4_SAMPLE index=%d elapsed_s=%.6f "
                "target_position=%d target_raw_u16=%d target_raw_hex=0x%04X "
                "status_target_position=%d actual_position=%d current_mA=%d "
                "force_g=%d fault=0x%02X all_position_fields_minus2=%s",
                index,
                elapsed_s,
                target_position,
                target_raw_u16,
                target_raw_u16,
                status.target_position,
                status.current_position,
                status.current_mA,
                status.force_g,
                status.error_code,
                sample.all_position_fields_minus_two,
            )
    except Exception as exc:
        error = exc
        logger.error("ID4_DIAGNOSIS_ERROR samples=%d reason=%s", len(samples), exc)
    finally:
        summary = _summary(samples, SAMPLE_COUNT)
        logger.info(summary)
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()

    print(summary)
    if error is not None:
        print(f"ERROR ID4 read-only diagnosis incomplete: {error}", file=sys.stderr)
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "ID4 fixed 100-sample read-only target/actual/current/force/fault diagnosis"
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
