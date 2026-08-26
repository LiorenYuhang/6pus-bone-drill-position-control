"""Mock-only tests for the J7-C10 read-only integration entry."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_c10_read_only_preflight import (
    parse_motor_ids,
    run_read_only_preflight,
)
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import MOTOR_IDS


class MockSerial:
    def __init__(self) -> None:
        self.device = FakeBus()
        self.rx = bytearray()
        self.writes: list[bytes] = []
        self.hardware_write_count = 0

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, frame: bytes) -> int:
        self.writes.append(frame)
        self.rx.extend(self.device.request(frame))
        return len(frame)

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data


class Clock:
    def __init__(self) -> None:
        self.now = 0

    def now_ns(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += int(seconds * 1e9)


def test_j7_c10_read_only_scan_does_not_configure_or_move(tmp_path: Path) -> None:
    clock = Clock()
    serial = MockSerial()
    adapter = J7UartAdapter(
        serial,
        timeout_s=0.005,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
    )
    audit_path = tmp_path / "c10.json"

    result = run_read_only_preflight(
        adapter,
        port="mock://c10",
        baudrate=921600,
        motor_ids=MOTOR_IDS,
        clock_ns=clock.now_ns,
        audit_file=audit_path,
    )

    assert result.motor_ids == MOTOR_IDS
    assert len(result.feedback.samples) == 6
    assert all(sample.fault == 0 for sample in result.feedback.samples)
    frames = [parse_frame(frame) for frame in serial.writes]
    assert all(frame.command in (0x30, 0x31) for frame in frames)
    assert all(frame.command != 0x32 for frame in frames)
    assert all(frame.register_addr != 0x29 for frame in frames)
    assert serial.hardware_write_count == 0
    assert audit_path.exists()


def test_j7_c10_motor_ids_must_match_fixed_six_axis_order() -> None:
    assert parse_motor_ids("1,2,3,4,5,6") == MOTOR_IDS
    with pytest.raises(ValueError, match="fixed order"):
        parse_motor_ids("1,2,3")
    with pytest.raises(ValueError, match="fixed order"):
        parse_motor_ids("6,5,4,3,2,1")

