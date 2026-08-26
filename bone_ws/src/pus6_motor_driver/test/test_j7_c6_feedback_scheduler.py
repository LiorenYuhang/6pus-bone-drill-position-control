"""Mock-only tests for the isolated J7 Servo feedback scheduler."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import threading
import time
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_servo_feedback_scheduler import (
    FeedbackRxError,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoRxPolicy
from pus6_motor_driver.protocol import (
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
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


class FeedbackMockBus(FakeBus):
    """Fake six-axis device with explicit RX and transaction audit queues."""

    def __init__(self) -> None:
        super().__init__()
        self.rx = deque[bytes]()
        self.write_calls: list[bytes] = []
        self.request_log: list[tuple[int, int, int]] = []
        self.hardware_write_count = 0
        self.timeout_next = False
        self.mismatch_next = False
        self.malformed_next = False

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        self.request_log.append((parsed.motor_id, parsed.command, parsed.register_addr))
        if self.timeout_next:
            self.timeout_next = False
            raise TimeoutError("mock feedback timeout")
        if self.malformed_next:
            self.malformed_next = False
            return b"\xAA\x55\x03\x01\x30"
        if self.mismatch_next:
            self.mismatch_next = False
            return super().request(pack_read_status_cmd(2))
        return super().request(frame, **kwargs)

    def write(self, frame: bytes) -> int:
        self.write_calls.append(frame)
        # A write-only motion frame may still leave a response in RX. The
        # scheduler must classify it as stale before issuing feedback reads.
        self.rx.append(super().request(frame))
        return len(frame)

    def read_available(self) -> bytes:
        data = b"".join(self.rx)
        self.rx.clear()
        return data


def make_stack(
    *,
    rx_policy: ServoRxPolicy = ServoRxPolicy.DEFER_TO_FEEDBACK,
    pause_events: list[str] | None = None,
) -> tuple[Clock, FeedbackMockBus, J7SixAxisServoTransport, J7ServoFeedbackScheduler]:
    clock = Clock()
    bus = FeedbackMockBus()
    writer = J7SixAxisServoTransport(
        bus,
        rx_policy=rx_policy,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
        pause_callback=(pause_events.append if pause_events is not None else None),
    )
    writer.configure_servo(speed=50)
    scheduler = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=0.100,
        deadline_guard_s=0.001,
        feedback_budget_s=0.010,
        clock_ns=clock.now_ns,
    )
    return clock, bus, writer, scheduler


def test_j7_c6_round_robin_feedback_uses_shared_order() -> None:
    clock, bus, writer, scheduler = make_stack()
    writer.send_cycle([100] * 6, cycle_id=1, target_timestamp_ns=0)

    audits = [
        scheduler.poll_once(
            servo_cycle_id=index,
            servo_cycle_timestamp_ns=clock.now_ns(),
        )
        for index in range(6)
    ]

    assert [audit.motor_ids for audit in audits] == [(1,), (2,), (3,), (4,), (5,), (6,)]
    assert all(not audit.deferred for audit in audits)
    assert all(audit.samples[0].position == 100 for audit in audits)
    assert all(audit.samples[0].current_mA == 0 for audit in audits)
    assert all(audit.samples[0].fault == 0 for audit in audits)
    assert all(audit.samples[0].mode is None for audit in audits)

    # Runtime feedback is status-only. Motion responses are classified before
    # each request and no control-mode register read is inserted in the stream.
    assert [parse_frame(frame).motor_id for frame in bus.write_calls] == list(MOTOR_IDS)
    runtime_requests = bus.request_log[-len(MOTOR_IDS):]
    assert [item[0] for item in runtime_requests] == list(MOTOR_IDS)
    assert [item[1] for item in runtime_requests] == [CMD_RD_STATUS] * len(MOTOR_IDS)
    assert all(item[1] != CMD_RD_REGISTER for item in runtime_requests)
    assert bus.hardware_write_count == 0


def test_j7_c6_full_scan_reads_all_axes() -> None:
    clock, bus, writer, scheduler = make_stack()
    audit = scheduler.full_scan(servo_cycle_id=10, servo_cycle_timestamp_ns=clock.now_ns())

    assert audit.full_scan
    assert audit.motor_ids == MOTOR_IDS
    assert len(audit.samples) == 6
    assert all(sample.mode is None for sample in audit.samples)
    assert all(sample.timeout is False for sample in audit.samples)
    assert bus.hardware_write_count == 0


def test_j7_c6_runtime_poll_reads_status_only_when_mode_is_zero_or_unavailable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    bus.modes[1] = 0
    runtime_request_start = len(bus.request_log)
    original_request = bus.request

    def reject_runtime_mode_read(frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        if parsed.command == CMD_RD_REGISTER:
            raise AssertionError("runtime feedback must not issue mode read 0x31")
        return original_request(frame, **kwargs)

    monkeypatch.setattr(bus, "request", reject_runtime_mode_read)

    audit = scheduler.poll_once(
        servo_cycle_id=1,
        servo_cycle_timestamp_ns=clock.now_ns(),
    )

    runtime_requests = bus.request_log[runtime_request_start:]
    assert not audit.deferred
    assert len(audit.samples) == 1
    assert audit.samples[0].motor_id == 1
    assert audit.samples[0].position is not None
    assert audit.samples[0].current_mA == 0
    assert audit.samples[0].fault == 0
    assert audit.samples[0].mode is None
    assert not audit.samples[0].timeout
    assert [command for _, command, _ in runtime_requests] == [CMD_RD_STATUS]
    assert not scheduler.stopped
    assert not writer.stopped
    assert scheduler.audit_snapshot().stop_reason is None
    assert pauses == []
    assert bus.hardware_write_count == 0


def test_j7_c6_runtime_full_scan_uses_status_only_when_mode_is_unavailable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    bus.modes.update({motor_id: 0 for motor_id in MOTOR_IDS})
    runtime_request_start = len(bus.request_log)
    original_request = bus.request

    def reject_runtime_mode_read(frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        if parsed.command == CMD_RD_REGISTER:
            raise AssertionError("runtime full scan must not issue mode read 0x31")
        return original_request(frame, **kwargs)

    monkeypatch.setattr(bus, "request", reject_runtime_mode_read)

    audit = scheduler.full_scan(
        servo_cycle_id=2,
        servo_cycle_timestamp_ns=clock.now_ns(),
    )

    runtime_requests = bus.request_log[runtime_request_start:]
    assert audit.full_scan
    assert audit.motor_ids == MOTOR_IDS
    assert [sample.motor_id for sample in audit.samples] == list(MOTOR_IDS)
    assert all(sample.position is not None for sample in audit.samples)
    assert all(sample.fault == 0 for sample in audit.samples)
    assert all(not sample.timeout for sample in audit.samples)
    assert all(sample.mode is None for sample in audit.samples)
    assert [command for _, command, _ in runtime_requests] == [CMD_RD_STATUS] * len(MOTOR_IDS)
    assert not scheduler.stopped
    assert not writer.stopped
    assert scheduler.audit_snapshot().stop_reason is None
    assert pauses == []
    assert bus.hardware_write_count == 0


def test_j7_c6_feedback_defers_when_writer_holds_uart() -> None:
    clock, _bus, writer, scheduler = make_stack()
    ready = threading.Event()
    release = threading.Event()

    def hold_lock() -> None:
        with writer.uart_lock:
            ready.set()
            release.wait(timeout=1.0)

    holder = threading.Thread(target=hold_lock)
    holder.start()
    assert ready.wait(timeout=1.0)
    try:
        audit = scheduler.poll_once(
            servo_cycle_id=11,
            servo_cycle_timestamp_ns=clock.now_ns(),
        )
    finally:
        release.set()
        holder.join(timeout=1.0)

    assert audit.deferred
    assert audit.motor_ids == tuple()
    assert scheduler.audit_snapshot().defer_count == 1


def test_j7_c6_stale_motion_responses_are_not_feedback_matches() -> None:
    clock, bus, writer, scheduler = make_stack()
    writer.send_cycle([123] * 6, cycle_id=12, target_timestamp_ns=0)

    audit = scheduler.poll_once(servo_cycle_id=12, servo_cycle_timestamp_ns=clock.now_ns())

    assert audit.samples[0].position == 123
    assert audit.rx_residual_count == 6
    assert audit.stale_response_count >= 6
    assert scheduler.audit_snapshot().mismatch_count == 0
    assert bus.hardware_write_count == 0


def test_j7_c6_feedback_mismatch_stops_safely() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    bus.mismatch_next = True

    with pytest.raises(FeedbackRxError, match="response ID"):
        scheduler.poll_once(servo_cycle_id=13, servo_cycle_timestamp_ns=clock.now_ns())

    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().mismatch_count == 1
    assert pauses == ["feedback_mismatch"]
    assert bus.hardware_write_count == 0


def test_j7_c6_timeout_enters_safe_stop_without_retry() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    bus.timeout_next = True

    audit = scheduler.poll_once(servo_cycle_id=14, servo_cycle_timestamp_ns=clock.now_ns())

    assert audit.samples[0].timeout
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().timeout_count == 1
    assert pauses == ["feedback_timeout"]
    # One status request was attempted; no scheduler retry is performed.
    assert len(bus.request_log) == 7
    assert bus.hardware_write_count == 0


def test_j7_c6_malformed_response_stops_safely() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    bus.malformed_next = True

    with pytest.raises(FeedbackRxError):
        scheduler.poll_once(servo_cycle_id=15, servo_cycle_timestamp_ns=clock.now_ns())

    assert scheduler.stopped
    assert scheduler.audit_snapshot().malformed_count == 1
    assert pauses == ["malformed_feedback_response"]
    assert bus.hardware_write_count == 0

