"""Mock-only tests for the isolated J7 single-axis Servo transport."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.j7_servo_transport import (  # type: ignore
    J7SingleAxisServoTransport,
    RxSafetyError,
    ServoRxPolicy,
    ServoTransportError,
)


class AdapterMockBus:
    """Wrap production FakeBus while exposing a write-only RX queue."""

    def __init__(self) -> None:
        self.inner = FakeBus()
        self.rx = deque[bytes]()
        self.request_calls: list[bytes] = []
        self.write_calls: list[bytes] = []
        self.read_calls = 0
        self.fail_write = False

    def request(self, frame: bytes, **kwargs) -> bytes:
        self.request_calls.append(frame)
        return self.inner.request(frame, **kwargs)

    def write(self, frame: bytes) -> int:
        self.write_calls.append(frame)
        if self.fail_write:
            raise OSError("mock write failure")
        # The device response is intentionally left in RX.  The adapter must
        # not consume it as part of write_only_position().
        self.rx.append(self.inner.request(frame))
        return len(frame)

    def read_available(self) -> bytes:
        self.read_calls += 1
        data = b"".join(self.rx)
        self.rx.clear()
        return data


class Clock:
    def __init__(self) -> None:
        self.now = 0
        self.sleeps: list[float] = []

    def now_ns(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += int(seconds * 1e9)


def make_transport(
    bus: AdapterMockBus,
    *,
    policy: ServoRxPolicy = ServoRxPolicy.DEFER_TO_FEEDBACK,
    max_residual_frames: int = 32,
    clock: Clock | None = None,
    pause_events: list[str] | None = None,
) -> J7SingleAxisServoTransport:
    clock = clock or Clock()
    return J7SingleAxisServoTransport(
        bus,
        motor_id=1,
        rx_policy=policy,
        max_residual_frames=max_residual_frames,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
        pause_callback=(pause_events.append if pause_events is not None else None),
    )


def test_j7_c2_4_configures_mode_one_without_runtime_readback() -> None:
    bus = AdapterMockBus()
    transport = make_transport(bus)

    config = transport.configure_servo(speed=50)

    assert config.mode == 1
    assert config.speed == 50
    assert config.mode_readback is None
    assert config.speed_readback is None
    assert len(bus.request_calls) == 1  # config write only
    assert parse_frame(bus.request_calls[0]).register_addr == 0x25
    assert transport.audit_snapshot().response_wait_count == 0


def test_j7_c2_4_motion_is_write_only_and_leaves_response_residual() -> None:
    bus = AdapterMockBus()
    transport = make_transport(bus)
    transport.configure_servo(speed=50)
    request_count_after_config = len(bus.request_calls)

    tx = transport.write_only_position(1234, cycle_id=7)

    assert tx.response_wait is False
    assert tx.retry_count == 0
    assert len(bus.write_calls) == 1
    assert len(bus.request_calls) == request_count_after_config
    assert transport.residual_frame_count == 0  # response is not readable until next poll

    drained = transport.drain_residual(policy=ServoRxPolicy.DEFER_TO_FEEDBACK)
    assert drained.residual_frames_before == 1
    assert drained.discarded_frames == 0
    assert transport.residual_frame_count == 1

    placeholder = transport.feedback_request(kind="position")
    assert placeholder.implemented is False
    assert len(bus.request_calls) == request_count_after_config


def test_j7_c2_4_discard_policy_records_stale_response_before_next_tx() -> None:
    bus = AdapterMockBus()
    transport = make_transport(bus, policy=ServoRxPolicy.DISCARD_BEFORE_TX)
    transport.configure_servo(speed=50)

    transport.write_only_position(1000, cycle_id=1)
    tx2 = transport.write_only_position(1001, cycle_id=2)

    assert tx2.residual_frames_before == 0
    audit = transport.audit_snapshot()
    assert any(item.discarded_frames == 1 for item in audit.rx_records)
    assert transport.residual_frame_count == 0
    assert audit.response_wait_count == 0
    assert audit.motion_retry_count == 0


def test_j7_c2_4_reject_policy_stops_on_residual_without_retry() -> None:
    bus = AdapterMockBus()
    pauses: list[str] = []
    transport = make_transport(
        bus,
        policy=ServoRxPolicy.REJECT_RESIDUAL,
        pause_events=pauses,
    )
    transport.configure_servo(speed=50)
    transport.write_only_position(1000, cycle_id=1)

    with pytest.raises(RxSafetyError, match="residual"):
        transport.write_only_position(1001, cycle_id=2)

    audit = transport.audit_snapshot()
    assert transport.stopped
    assert audit.delivery_uncertain
    assert audit.stop_reason == "rx_safety_failure"
    assert pauses == ["rx_safety_failure"]
    assert len(bus.write_calls) == 1


def test_j7_c2_4_write_failure_stops_once_and_never_retries() -> None:
    bus = AdapterMockBus()
    pauses: list[str] = []
    transport = make_transport(bus, pause_events=pauses)
    transport.configure_servo(speed=50)
    bus.fail_write = True

    with pytest.raises(OSError, match="mock write failure"):
        transport.write_only_position(900, cycle_id=1)
    with pytest.raises(ServoTransportError, match="stopped"):
        transport.write_only_position(901, cycle_id=2)

    audit = transport.audit_snapshot()
    assert audit.delivery_uncertain
    assert audit.motion_retry_count == 0
    assert len(bus.write_calls) == 1
    assert pauses == ["motion_write_failure"]


def test_j7_c2_4_frame_interval_is_serialized_by_one_lock() -> None:
    bus = AdapterMockBus()
    clock = Clock()
    transport = make_transport(bus, clock=clock)
    transport.configure_servo(speed=50)
    transport.write_only_position(1000, cycle_id=1)
    transport.write_only_position(1002, cycle_id=2)

    assert clock.sleeps == [pytest.approx(0.001)]
    assert bus.write_calls[0] != bus.write_calls[1]


def test_j7_c2_4_residual_overflow_stops_before_next_motion_write() -> None:
    bus = AdapterMockBus()
    pauses: list[str] = []
    transport = make_transport(bus, max_residual_frames=1, pause_events=pauses)
    transport.configure_servo(speed=50)
    transport.write_only_position(1000, cycle_id=1)

    # One queued response is within the limit; the second response overflows
    # when the next TX attempts to ingest RX.
    bus.rx.append(bus.rx[0])
    with pytest.raises(RxSafetyError, match="residual limit"):
        transport.write_only_position(1001, cycle_id=2)

    assert transport.stopped
    assert pauses == ["rx_safety_failure"]
    assert len(bus.write_calls) == 1
