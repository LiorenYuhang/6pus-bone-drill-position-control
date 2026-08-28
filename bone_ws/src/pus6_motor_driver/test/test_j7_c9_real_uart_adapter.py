"""Mock-only tests for the isolated J7 UART and stop adapters."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_real_uart_adapter import (
    J7AdapterRxError,
    J7RxOwner,
    J7AdapterTimeout,
    J7StopAdapter,
    J7UartAdapter,
)
from pus6_motor_driver.j7_servo_transport import ServoRxPolicy
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.protocol import (
    CMD_RD_STATUS,
    pack_write_register_cmd,
    pack_read_status_cmd,
    parse_frame,
)
from pus6_motor_driver.safety import MOTOR_IDS


class Clock:
    def __init__(self) -> None:
        self.now = 0

    def now_ns(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += int(seconds * 1e9)


class MockSerial:
    """Serial-like device backed by FakeBus; it never opens hardware."""

    def __init__(self) -> None:
        self.device = FakeBus()
        self.rx = bytearray()
        self.writes: list[bytes] = []
        self.hardware_write_count = 0
        self.fail_write_id: int | None = None
        self.respond = True
        self.read_count = 0

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, frame: bytes) -> int:
        parsed = parse_frame(frame)
        self.writes.append(frame)
        if self.fail_write_id == parsed.motor_id:
            raise OSError(f"mock serial write failure on ID{parsed.motor_id}")
        if self.respond:
            self.rx.extend(self.device.request(frame))
        return len(frame)

    def read(self, size: int = 1) -> bytes:
        self.read_count += 1
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data


def make_adapter(
    *,
    clock: Clock | None = None,
) -> tuple[Clock, MockSerial, J7UartAdapter]:
    clock = clock or Clock()
    serial = MockSerial()
    adapter = J7UartAdapter(
        serial,
        timeout_s=0.005,
        min_frame_interval_s=0.001,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
    )
    return clock, serial, adapter


def test_j7_c9_adapter_supports_c5_configuration_and_write_only_motion() -> None:
    clock, serial, adapter = make_adapter()
    writer = J7SixAxisServoTransport(
        adapter,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
    )

    configs = writer.configure_servo(speed=50)
    cycle = writer.send_cycle([100, 101, 102, 103, 104, 105], cycle_id=1, target_timestamp_ns=0)

    assert [config.motor_id for config in configs] == list(MOTOR_IDS)
    assert [parse_frame(frame).motor_id for frame in serial.writes[-6:]] == list(MOTOR_IDS)
    assert all(record.response_wait is False for record in cycle.tx_records)
    assert all(record.retry_count == 0 for record in cycle.tx_records)
    assert adapter.uart_lock is not None
    assert serial.hardware_write_count == 0


def test_j7_c9_single_rx_owner_classifies_stale_before_feedback() -> None:
    clock, serial, adapter = make_adapter()
    writer = J7SixAxisServoTransport(
        adapter,
        rx_policy=ServoRxPolicy.DEFER_TO_FEEDBACK,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
    )
    writer.configure_servo(speed=50)
    writer.send_cycle([123] * 6, cycle_id=2, target_timestamp_ns=0)

    response = adapter.request(
        pack_read_status_cmd(1),
        expected_id=1,
        expected_command=CMD_RD_STATUS,
        retries=0,
    )
    assert parse_frame(response).motor_id == 1
    rx_audit = adapter.rx_owner.audit_snapshot()
    assert rx_audit.motion_write_response_count >= 6
    assert rx_audit.stale_response_count == 0
    assert rx_audit.feedback_response_count >= 1
    assert rx_audit.malformed_count == 0
    assert serial.hardware_write_count == 0


def test_j7_c9_stop_success_is_six_axis_and_idempotent() -> None:
    clock, serial, adapter = make_adapter()
    stop = J7StopAdapter(adapter)

    first = stop("operator_request")
    second = stop("second_request")

    assert first.success
    assert [record.motor_id for record in first.records] == list(MOTOR_IDS)
    assert all(record.response_received for record in first.records)
    assert first.callback_count == 1
    assert second == first
    assert len(serial.writes) == 6
    assert all(parse_frame(frame).register_addr == 0x1A for frame in serial.writes)
    assert serial.hardware_write_count == 0


def test_j7_c9_stop_failure_continues_other_axes_without_retry() -> None:
    _clock, serial, adapter = make_adapter()
    serial.fail_write_id = 3
    stop = J7StopAdapter(adapter)

    audit = stop("write_failure")

    assert not audit.success
    assert len(audit.records) == 6
    failed = [record for record in audit.records if record.motor_id == 3][0]
    assert not failed.response_received
    assert not failed.timeout
    assert [parse_frame(frame).motor_id for frame in serial.writes] == [1, 2, 3, 4, 5, 6]
    assert audit.callback_count == 1
    assert serial.hardware_write_count == 0


def test_j7_c9_stop_timeout_is_recorded_without_retry() -> None:
    clock, serial, adapter = make_adapter(clock=Clock())
    serial.respond = False
    stop = J7StopAdapter(adapter)

    audit = stop("timeout")

    assert not audit.success
    assert len(audit.records) == 6
    assert all(record.timeout for record in audit.records)
    assert all(not record.response_received for record in audit.records)
    assert audit.callback_count == 1
    assert len(serial.writes) == 6
    assert serial.hardware_write_count == 0


def test_j7_c9_delayed_interleaved_motion_and_feedback_frames_are_classified_once() -> None:
    _clock, serial, adapter = make_adapter()
    motion = pack_write_register_cmd(2, 0x29, [222])
    delayed_status = serial.device.request(pack_read_status_cmd(2))
    serial.rx.extend(serial.device.request(motion))
    serial.rx.extend(delayed_status)

    response = adapter.request(
        pack_read_status_cmd(1),
        expected_id=1,
        expected_command=CMD_RD_STATUS,
        expected_register=0x0000,
        retries=0,
    )

    assert parse_frame(response).motor_id == 1
    audit = adapter.rx_owner.audit_snapshot()
    assert audit.motion_write_response_count == 1
    assert audit.stale_response_count == 1
    assert audit.feedback_response_count == 1
    assert audit.residual_frames == 0
    assert serial.hardware_write_count == 0


def test_j7_c9_malformed_motion_status_is_rejected_by_single_rx_owner() -> None:
    owner = J7RxOwner()
    motion = pack_write_register_cmd(1, 0x29, [222])
    raw = bytearray(FakeBus().request(motion))
    raw[2] = 0x0E
    raw[-1] = sum(raw[2:-1]) & 0xFF

    with pytest.raises(J7AdapterRxError):
        owner.ingest(bytes(raw))


def test_j7_c9_feedback_match_requires_current_axis_and_register() -> None:
    _clock, serial, adapter = make_adapter()
    wrong_register = bytearray(serial.device.request(pack_read_status_cmd(1)))
    wrong_register[5] = 0x01
    wrong_register[6] = 0x00
    wrong_register[-1] = sum(wrong_register[2:-1]) & 0xFF
    serial.rx.extend(wrong_register)

    response = adapter.request(
        pack_read_status_cmd(1),
        expected_id=1,
        expected_command=CMD_RD_STATUS,
        expected_register=0x0000,
        retries=0,
    )

    assert parse_frame(response).register_addr == 0x0000
    audit = adapter.rx_owner.audit_snapshot()
    assert audit.stale_response_count == 1
    assert audit.feedback_response_count == 1
