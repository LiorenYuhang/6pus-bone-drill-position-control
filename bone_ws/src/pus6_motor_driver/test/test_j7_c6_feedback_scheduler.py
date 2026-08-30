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
    FeedbackSchedulerError,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoRxPolicy
from pus6_motor_driver.protocol import (
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    StatusFeedback,
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

    assert scheduler.feedback_age_watchdog_armed

    assert audit.full_scan
    assert audit.motor_ids == MOTOR_IDS
    assert len(audit.samples) == 6
    assert all(sample.mode is None for sample in audit.samples)
    assert all(sample.timeout is False for sample in audit.samples)
    assert bus.hardware_write_count == 0


def test_j7_c6_startup_before_all_axes_initialized_does_not_age_stop() -> None:
    clock, bus, writer, scheduler = make_stack()
    scheduler.full_scan_period_ns = 10**18
    clock.now = 1_000_000_000

    first = scheduler.poll_once(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert first.motor_ids == (1,)
    assert not scheduler.feedback_age_watchdog_armed

    # A long startup gap must not evaluate the partially initialized session.
    clock.sleep(0.300)
    second = scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())
    assert second.motor_ids == (2,)
    assert not scheduler.stopped
    assert not scheduler.feedback_age_watchdog_armed
    assert scheduler._consecutive_timeouts == {mid: 0 for mid in MOTOR_IDS}
    assert bus.hardware_write_count == 0


def test_j7_c6_watchdog_arms_only_after_fresh_six_axis_feedback() -> None:
    clock, bus, writer, scheduler = make_stack()
    scheduler.full_scan_period_ns = 10**18
    clock.now = 1_000_000_000

    scheduler.poll_once(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert not scheduler.feedback_age_watchdog_armed
    assert scheduler._initialized_axes == {1}

    audit = scheduler.full_scan(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())
    assert audit.motor_ids == MOTOR_IDS
    assert all(not sample.timeout for sample in audit.samples)
    assert scheduler._initialized_axes == set(MOTOR_IDS)
    assert scheduler.feedback_age_watchdog_armed
    assert scheduler._consecutive_timeouts == {mid: 0 for mid in MOTOR_IDS}
    assert bus.hardware_write_count == 0


def test_j7_c6_post_arm_stale_feedback_stops() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed

    clock.sleep((scheduler.max_feedback_age_ns / 1e9) + 0.001)
    with pytest.raises(FeedbackSchedulerError, match="feedback age exceeded"):
        scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())

    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason == "feedback_age_exceeded_ID1"
    assert pauses == ["feedback_age_exceeded_ID1"]
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
    assert scheduler.audit_snapshot().motion_write_response_count >= 6
    assert audit.stale_response_count == 0
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


def test_j7_c6_isolated_timeout_recovers_without_stop() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed
    bus.timeout_next = True

    timeout_audit = scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())
    assert timeout_audit.samples[0].timeout
    assert not scheduler.stopped
    assert not writer.stopped
    assert pauses == []

    # The round-robin slot returns to ID1 after five other axis polls.
    for cycle_id in range(3, 8):
        scheduler.poll_once(servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=clock.now_ns())
    recovered = scheduler.poll_once(servo_cycle_id=8, servo_cycle_timestamp_ns=clock.now_ns())
    assert recovered.motor_ids == (1,)
    assert recovered.samples[0].timeout is False
    assert not scheduler.stopped
    assert scheduler.audit_snapshot().timeout_count == 1
    assert bus.hardware_write_count == 0


def test_j7_c6_persistent_timeout_escalates_to_stop() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())

    bus.timeout_next = True
    first = scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())
    assert first.samples[0].timeout
    for cycle_id in range(3, 8):
        scheduler.poll_once(servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=clock.now_ns())

    bus.timeout_next = True
    second = scheduler.poll_once(servo_cycle_id=8, servo_cycle_timestamp_ns=clock.now_ns())
    assert second.samples[0].timeout
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "persistent_feedback_loss_ID1"
    assert pauses == ["persistent_feedback_loss_ID1"]
    assert bus.hardware_write_count == 0


def test_j7_c6_age_watchdog_does_not_preempt_second_same_axis_poll() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    scheduler.full_scan_period_ns = 10**18
    clock.now = 1_000_000_000
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.max_feedback_age_ns == 263_324_000

    # ID1's first retry opportunity is approximately 240 ms after its last
    # valid sample: one initial poll plus five intervening axis polls.
    bus.timeout_next = True
    clock.sleep(0.120)
    first = scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())
    assert first.samples[0].motor_id == 1
    assert first.samples[0].timeout
    assert not scheduler.stopped

    for cycle_id in range(3, 8):
        clock.sleep(0.020)
        scheduler.poll_once(servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=clock.now_ns())

    bus.timeout_next = True
    clock.sleep(0.020)
    second = scheduler.poll_once(servo_cycle_id=8, servo_cycle_timestamp_ns=clock.now_ns())

    assert second.samples[0].motor_id == 1
    assert second.samples[0].timeout
    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason == "persistent_feedback_loss_ID1"
    assert pauses == ["persistent_feedback_loss_ID1"]
    assert [motor_id for motor_id, _command, _register in bus.request_log[-7:]] == [1, 2, 3, 4, 5, 6, 1]
    assert bus.hardware_write_count == 0


def test_j7_c6_age_watchdog_stops_after_scheduler_stall() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    requests_before_stall = len(bus.request_log)

    clock.sleep(0.263325)
    with pytest.raises(FeedbackSchedulerError, match="feedback age exceeded"):
        scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())

    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason == "feedback_age_exceeded_ID1"
    assert pauses == ["feedback_age_exceeded_ID1"]
    assert len(bus.request_log) == requests_before_stall
    assert bus.hardware_write_count == 0

def test_j7_c6_motor_fault_stops_immediately() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    scheduler.full_scan(servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    bus.faults[1] = 7

    audit = scheduler.poll_once(servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())

    assert audit.samples[0].fault == 7
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "motor_fault_ID1"
    assert pauses == ["motor_fault_ID1"]
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


def test_j8_design_b_healthy_cycles_do_not_trigger_feedback_age_stop() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)

    for cycle_id in range(1, 20):
        clock.sleep(0.020)
        feedbacks = {
            mid: StatusFeedback(
                target_position=1000 + cycle_id,
                current_position=1000 + cycle_id,
                current_mA=0,
                force_g=0,
                force_adc=0,
                temperature=25,
                error_code=0,
            )
            for mid in (1, 2, 3, 4, 5, 6)
        }
        audit = scheduler.ingest_cycle_feedback(
            feedbacks,
            servo_cycle_id=cycle_id,
            servo_cycle_timestamp_ns=clock.now_ns(),
        )
        assert not audit.deferred
        assert len(audit.samples) == 6
        assert all(not s.timeout and s.fault == 0 for s in audit.samples)
        assert not scheduler.stopped
        assert scheduler.audit_snapshot().stop_reason is None


def test_j8_design_b_tier1_missing_axis_escalation() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000
    scheduler.timeout_escalation_misses = 4

    # 1. Arm watchdog with healthy cycle
    feedbacks_ok = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 2, 3, 4, 5, 6)
    }
    scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed
    assert not scheduler.stopped

    # 2. Cycles 2, 3, 4 miss axis ID3 (3 consecutive misses: warning, not yet stopped)
    feedbacks_missing_id3 = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 2, 4, 5, 6)
    }
    for c in range(2, 5):
        clock.sleep(0.020)
        scheduler.ingest_cycle_feedback(feedbacks_missing_id3, servo_cycle_id=c, servo_cycle_timestamp_ns=clock.now_ns())
        assert not scheduler.stopped

    # 3. 4th consecutive miss (N=4) -> Tier-1 escalation trips persistent feedback loss
    clock.sleep(0.020)
    with pytest.raises(FeedbackSchedulerError, match="persistent.*feedback loss"):
        scheduler.ingest_cycle_feedback(feedbacks_missing_id3, servo_cycle_id=5, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason.startswith("persistent_")


def test_j8_design_b_tier1_incomplete_cycle_escalation() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000
    scheduler.timeout_escalation_misses = 4

    # 1. Arm watchdog
    feedbacks_ok = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 2, 3, 4, 5, 6)
    }
    scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed

    # 2. Multiple missing axes (ID2 and ID5)
    feedbacks_incomplete = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 3, 4, 6)
    }
    for c in range(2, 5):
        clock.sleep(0.020)
        scheduler.ingest_cycle_feedback(feedbacks_incomplete, servo_cycle_id=c, servo_cycle_timestamp_ns=clock.now_ns())
        assert not scheduler.stopped

    # 3. 4th consecutive incomplete cycle -> Tier-1 escalation
    clock.sleep(0.020)
    with pytest.raises(FeedbackSchedulerError):
        scheduler.ingest_cycle_feedback(feedbacks_incomplete, servo_cycle_id=5, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason.startswith("persistent_")


def test_j8_design_b_incomplete_cycle_counter_resets_after_complete_feedback() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000
    scheduler.timeout_escalation_misses = 4

    feedbacks_ok = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 2, 3, 4, 5, 6)
    }
    feedbacks_incomplete = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 3, 4, 6)
    }

    scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed

    for cycle_id in range(2, 5):
        clock.sleep(0.020)
        scheduler.ingest_cycle_feedback(feedbacks_incomplete, servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=clock.now_ns())
        assert not scheduler.stopped

    clock.sleep(0.020)
    scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=5, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler._consecutive_incomplete_cycles == 0
    assert not scheduler.stopped

    for cycle_id in range(6, 9):
        clock.sleep(0.020)
        scheduler.ingest_cycle_feedback(feedbacks_incomplete, servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=clock.now_ns())
        assert not scheduler.stopped

    clock.sleep(0.020)
    with pytest.raises(FeedbackSchedulerError, match="persistent.*feedback loss"):
        scheduler.ingest_cycle_feedback(feedbacks_incomplete, servo_cycle_id=9, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.stopped
    assert pauses == ["persistent_feedback_loss_ID2"] or pauses == ["persistent_feedback_loss_ID3"] or pauses == ["persistent_feedback_loss_ID4"] or pauses == ["persistent_feedback_loss_ID6"]


def test_j8_design_b_tier2_scheduler_deadline_fault() -> None:
    pauses: list[str] = []
    clock, bus, writer, scheduler = make_stack(pause_events=pauses)
    clock.now = 1_000_000_000

    # 1. Arm watchdog
    feedbacks_ok = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=0,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in (1, 2, 3, 4, 5, 6)
    }
    scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=1, servo_cycle_timestamp_ns=clock.now_ns())
    assert scheduler.feedback_age_watchdog_armed

    # 2. Simulate long scheduler gap > 263.324 ms
    clock.sleep(0.270)
    with pytest.raises(FeedbackSchedulerError, match="Servo scheduler deadline exceeded"):
        scheduler.ingest_cycle_feedback(feedbacks_ok, servo_cycle_id=2, servo_cycle_timestamp_ns=clock.now_ns())

    assert scheduler.stopped
    assert scheduler.audit_snapshot().stop_reason == "scheduler_deadline_exceeded"
    assert pauses == ["scheduler_deadline_exceeded"]


def test_j8_demo_health_check_reports_original_stop_reason_without_attribute_error() -> None:
    import ast
    from pathlib import Path

    demo_path = Path(__file__).resolve().parents[3] / "scripts" / "run_j8_final_video_demonstration.py"
    tree = ast.parse(demo_path.read_text())
    selected = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "demo_health_failure_reason"
    ]
    namespace: dict[str, object] = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(demo_path), "exec"), namespace)
    demo_health_failure_reason = namespace["demo_health_failure_reason"]

    class MockNode:
        driver_fault_latched = False
        driver_fault_reason = None
        j7_writer = None
        j7_feedback = None
        _latest_faults = {mid: 0 for mid in (1, 2, 3, 4, 5, 6)}

    node = MockNode()
    assert demo_health_failure_reason(node) is None

    # Test driver latched fault
    node.driver_fault_latched = True
    node.driver_fault_reason = "feedback_age_exceeded_ID1"
    assert demo_health_failure_reason(node) == "driver fault latched: feedback_age_exceeded_ID1"

    # Test transport stop reason via audit_snapshot
    class MockAudit:
        stop_reason = "transport_write_timeout"

    class MockTransport:
        stopped = True
        def audit_snapshot(self):
            return MockAudit()

    node.driver_fault_latched = False
    node.j7_writer = MockTransport()
    assert demo_health_failure_reason(node) == "driver writer stopped: transport_write_timeout"


