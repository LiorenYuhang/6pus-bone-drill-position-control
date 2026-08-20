#!/usr/bin/env python3
"""Strictly read-only comparison of documented ID3/ID4 configuration."""

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
SAMPLE_COUNT = 3
SAMPLE_INTERVAL_S = 0.05


@dataclass(frozen=True)
class RegisterSpec:
    address: int
    name: str


REGISTER_SPECS = (
    RegisterSpec(0x16, "id_address"),
    RegisterSpec(0x17, "baud_rate"),
    RegisterSpec(0x1E, "over_temperature_limit"),
    RegisterSpec(0x1F, "temperature_restart_limit"),
    RegisterSpec(0x20, "over_current_limit"),
    RegisterSpec(0x21, "positive_output_max"),
    RegisterSpec(0x22, "negative_output_max"),
    RegisterSpec(0x23, "travel_upper_limit"),
    RegisterSpec(0x24, "travel_lower_limit"),
    RegisterSpec(0x25, "control_mode"),
    RegisterSpec(0x26, "motor_output_voltage"),
    RegisterSpec(0x27, "force_target"),
    RegisterSpec(0x28, "target_speed"),
    RegisterSpec(0x29, "target_position"),
)
REGISTER_NAMES = {spec.address: spec.name for spec in REGISTER_SPECS}
READ_REGISTER_ADDRS = frozenset(REGISTER_NAMES)
IDENTITY_ADDR = 0x16
TARGET_POSITION_ADDR = 0x29
STATIC_COMPARE_ADDRS = READ_REGISTER_ADDRS - {
    IDENTITY_ADDR,
    TARGET_POSITION_ADDR,
}

logger = logging.getLogger("audit_id3_id4_config_readonly")


class ReadOnlyViolation(RuntimeError):
    """An outgoing request is outside the fixed documented read-only scope."""


@dataclass(frozen=True)
class ConfigSnapshot:
    motor_id: int
    values: dict[int, int]
    status: StatusFeedback

    def value(self, address: int) -> int:
        return self.values[address]


@dataclass(frozen=True)
class PairedSnapshot:
    index: int
    elapsed_s: float
    id3: ConfigSnapshot
    id4: ConfigSnapshot


class ConfigReadOnlyAuditedBus:
    """Allow only documented single-register reads and status for ID3/ID4."""

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
                    f"register 0x{parsed.register_addr:04X} is outside config whitelist"
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


def _read_one_register(motor: Motor, spec: RegisterSpec) -> int:
    response = motor.read_register(spec.address)
    if len(response.values) != 1:
        raise FrameLengthError(
            f"ID{motor.motor_id} register 0x{spec.address:02X} returned "
            f"{len(response.values)} values"
        )
    return response.values[0]


def _read_snapshot(motor: Motor) -> ConfigSnapshot:
    values = {spec.address: _read_one_register(motor, spec) for spec in REGISTER_SPECS}
    return ConfigSnapshot(
        motor_id=motor.motor_id,
        values=values,
        status=motor.read_status(),
    )


def _histogram(values: list[int]) -> dict[int, int]:
    return dict(sorted(Counter(values).items()))


def _id4_minus_two_persistent(samples: list[PairedSnapshot]) -> bool:
    return bool(samples) and all(
        _signed_u16(sample.id4.value(TARGET_POSITION_ADDR)) == -2
        and sample.id4.status.target_position == -2
        and sample.id4.status.current_position == -2
        for sample in samples
    )


def _classification(samples: list[PairedSnapshot], expected_count: int) -> str:
    if len(samples) != expected_count:
        return "INCONCLUSIVE"
    static_difference = any(
        sample.id3.value(address) != sample.id4.value(address)
        for sample in samples
        for address in STATIC_COMPARE_ADDRS
    )
    if _id4_minus_two_persistent(samples):
        if static_difference:
            return "CONFIG_DIFFERENCES_OBSERVED_WITH_ID4_MINUS2"
        return "CONFIG_MATCH_EXCEPT_ID_AND_TARGET_WITH_ID4_MINUS2"
    if static_difference:
        return "CONFIG_DIFFERENCES_OBSERVED"
    return "CONFIG_MATCH_EXCEPT_ID_AND_TARGET"


def _log_snapshot(snapshot: PairedSnapshot) -> None:
    for config in (snapshot.id3, snapshot.id4):
        for spec in REGISTER_SPECS:
            raw = config.value(spec.address)
            logger.info(
                "ID%d_CONFIG_SAMPLE index=%d elapsed_s=%.6f address=0x%02X "
                "name=%s raw_u16=%d raw_hex=0x%04X signed_i16=%d",
                config.motor_id,
                snapshot.index,
                snapshot.elapsed_s,
                spec.address,
                spec.name,
                raw,
                raw,
                _signed_u16(raw),
            )
        status = config.status
        logger.info(
            "ID%d_STATUS_SAMPLE index=%d elapsed_s=%.6f "
            "status_target_position=%d actual_position=%d current_mA=%d "
            "force_g=%d force_adc=%d temperature_C=%d fault=0x%02X",
            config.motor_id,
            snapshot.index,
            snapshot.elapsed_s,
            status.target_position,
            status.current_position,
            status.current_mA,
            status.force_g,
            status.force_adc,
            status.temperature,
            status.error_code,
        )


def _summary_lines(samples: list[PairedSnapshot], expected_count: int) -> list[str]:
    protocol_family = "V2_STYLE_0x30_0x31" if samples else "NOT_CONFIRMED"
    lines = [
        "DEVICE_INFO_AVAILABILITY "
        "firmware_version=NOT_DOCUMENTED protocol_version=NOT_DOCUMENTED "
        "device_model=NOT_DOCUMENTED position_zero_register=NOT_DOCUMENTED "
        "position_offset_register=NOT_DOCUMENTED "
        f"protocol_family_observed={protocol_family}"
    ]
    if not samples:
        lines.append(
            f"CONFIG_AUDIT classification=INCONCLUSIVE samples=0 "
            f"expected_samples={expected_count} write_count=0"
        )
        return lines

    for spec in REGISTER_SPECS:
        id3_values = [sample.id3.value(spec.address) for sample in samples]
        id4_values = [sample.id4.value(spec.address) for sample in samples]
        equal_count = sum(left == right for left, right in zip(id3_values, id4_values))
        lines.append(
            f"CONFIG_COMPARE address=0x{spec.address:02X} name={spec.name} "
            f"id3_histogram={_histogram(id3_values)} "
            f"id4_histogram={_histogram(id4_values)} "
            f"equal={equal_count} different={len(samples) - equal_count} "
            f"identity_field={spec.address == IDENTITY_ADDR} "
            f"dynamic_target_field={spec.address == TARGET_POSITION_ADDR}"
        )

    status_fields = (
        ("status_target", lambda item: item.status.target_position),
        ("actual", lambda item: item.status.current_position),
        ("current_mA", lambda item: item.status.current_mA),
        ("force_g", lambda item: item.status.force_g),
        ("force_adc", lambda item: item.status.force_adc),
        ("temperature_C", lambda item: item.status.temperature),
        ("fault", lambda item: item.status.error_code),
    )
    for name, getter in status_fields:
        id3_values = [getter(sample.id3) for sample in samples]
        id4_values = [getter(sample.id4) for sample in samples]
        equal_count = sum(left == right for left, right in zip(id3_values, id4_values))
        lines.append(
            f"STATUS_COMPARE name={name} id3_histogram={_histogram(id3_values)} "
            f"id4_histogram={_histogram(id4_values)} "
            f"equal={equal_count} different={len(samples) - equal_count}"
        )

    lines.append(
        f"CONFIG_AUDIT classification={_classification(samples, expected_count)} "
        f"samples={len(samples)} expected_samples={expected_count} write_count=0"
    )
    return lines


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m4/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id3_id4_config_readonly_audit.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus = None
    audited_bus = None
    samples: list[PairedSnapshot] = []
    error = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = ConfigReadOnlyAuditedBus(bus)
        motors = {motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS}
        started = time.monotonic()
        for index in range(1, SAMPLE_COUNT + 1):
            scheduled = started + (index - 1) * SAMPLE_INTERVAL_S
            delay = scheduled - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            snapshot = PairedSnapshot(
                index=index,
                elapsed_s=time.monotonic() - started,
                id3=_read_snapshot(motors[3]),
                id4=_read_snapshot(motors[4]),
            )
            samples.append(snapshot)
            _log_snapshot(snapshot)
    except Exception as exc:
        error = exc
        logger.error("CONFIG_AUDIT_ERROR samples=%d reason=%s", len(samples), exc)
    finally:
        summary_lines = _summary_lines(samples, SAMPLE_COUNT)
        for line in summary_lines:
            logger.info(line)
        if audited_bus is not None:
            logger.info(audited_bus.summary())
        if bus is not None:
            bus.close()

    print("\n".join(summary_lines))
    if error is not None:
        print(f"ERROR ID3/ID4 config audit incomplete: {error}", file=sys.stderr)
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fixed ID3/ID4 documented configuration read-only audit"
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
