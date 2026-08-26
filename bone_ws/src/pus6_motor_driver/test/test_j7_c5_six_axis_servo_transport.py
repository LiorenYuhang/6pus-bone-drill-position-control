"""Mock-only tests for the isolated J7 six-axis Servo transport."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
)
from pus6_motor_driver.j7_servo_transport import (
    RxSafetyError,
    ServoRxPolicy,
    ServoTransportError,
)
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import (
    CONTROL_MODE_ADDR,
    MOTOR_IDS,
    TARGET_POSITION_ADDR,
    TARGET_SPEED_ADDR,
)


class Clock:
    def __init__(self) -> None:
        self.now = 0
        self.sleeps: list[float] = []

    def now_ns(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += int(seconds * 1e9)


class SixAxisMockBus(FakeBus):
    """Protocol-aware mock; no operation increments hardware_write_count."""

    def __init__(self) -> None:
        super().__init__()
        self.rx = deque[bytes]()
        self.write_calls: list[bytes] = []
        self.hardware_write_count = 0
        self.motion_response_wait_count = 0
        self.motion_retry_count = 0
        self.fail_on_id: int | None = None

    def write(self, frame: bytes) -> int:
        parsed = parse_frame(frame)
        self.write_calls.append(frame)
        if self.fail_on_id == parsed.motor_id:
            raise OSError(f"mock write failure on ID{parsed.motor_id}")
        # Model a device that may still answer a write-only motion frame. The
        # transport must leave this response for its RX policy, not wait here.
        response = super().request(frame)
        self.rx.append(response)
        return len(frame)

    def read_available(self) -> bytes:
        data = b"".join(self.rx)
        self.rx.clear()
        return data


def make_transport(
    bus: SixAxisMockBus,
    *,
    clock: Clock | None = None,
    policy: ServoRxPolicy = ServoRxPolicy.DISCARD_BEFORE_TX,
    pause_events: list[str] | None = None,
) -> J7SixAxisServoTransport:
    clock = clock or Clock()
    return J7SixAxisServoTransport(
        bus,
        rx_policy=policy,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
        pause_callback=(pause_events.append if pause_events is not None else None),
    )


def test_j7_c5_configures_all_axes_mode_one_and_reads_back() -> None:
    bus = SixAxisMockBus()
    transport = make_transport(bus)

    configs = transport.configure_servo(speed=50)

    assert [item.motor_id for item in configs] == list(MOTOR_IDS)
    assert all(item.mode == 1 for item in configs)
    assert all(item.speed == 50 for item in configs)
    assert bus.modes == {mid: 1 for mid in MOTOR_IDS}
    assert bus.speeds == {mid: 50 for mid in MOTOR_IDS}
    assert bus.write_calls == []
    assert bus.hardware_write_count == 0


def test_j7_c5_sends_ordered_write_only_six_axis_batch() -> None:
    bus = SixAxisMockBus()
    clock = Clock()
    transport = make_transport(bus, clock=clock)
    transport.configure_servo(speed=50)

    audit = transport.send_cycle(
        [100, 101, 102, 103, 104, 105],
        cycle_id=7,
        target_timestamp_ns=0,
    )

    frames = [parse_frame(frame) for frame in bus.write_calls]
    assert [frame.motor_id for frame in frames] == list(MOTOR_IDS)
    assert all(frame.register_addr == TARGET_POSITION_ADDR for frame in frames)
    assert [record.motor_id for record in audit.tx_records] == list(MOTOR_IDS)
    assert [record.target_position for record in audit.tx_records] == [100, 101, 102, 103, 104, 105]
    assert all(record.response_wait is False for record in audit.tx_records)
    assert all(record.retry_count == 0 for record in audit.tx_records)
    assert audit.tx_skew_ns >= 0
    assert audit.batch_complete_ns == audit.cycle_end_ns
    assert not audit.deadline_miss
    assert transport.audit_snapshot().response_wait_count == 0
    assert transport.audit_snapshot().motion_retry_count == 0
    assert bus.hardware_write_count == 0


def test_j7_c5_rx_policy_discards_motion_residual_before_next_batch() -> None:
    bus = SixAxisMockBus()
    clock = Clock()
    transport = make_transport(bus, clock=clock, policy=ServoRxPolicy.DISCARD_BEFORE_TX)
    transport.configure_servo(speed=50)

    transport.send_cycle([200] * 6, cycle_id=1, target_timestamp_ns=0)
    assert transport.residual_frame_count == 0
    transport.send_cycle([201] * 6, cycle_id=2, target_timestamp_ns=clock.now_ns())

    audit = transport.audit_snapshot()
    assert len(audit.cycles) == 2
    assert any(item.discarded_frames == 6 for item in audit.rx_records)
    assert transport.residual_frame_count == 0
    assert bus.motion_response_wait_count == 0
    assert bus.motion_retry_count == 0
    assert bus.hardware_write_count == 0


def test_j7_c5_write_failure_stops_without_retry() -> None:
    bus = SixAxisMockBus()
    pauses: list[str] = []
    transport = make_transport(bus, pause_events=pauses)
    transport.configure_servo(speed=50)
    bus.fail_on_id = 3

    with pytest.raises(OSError, match="ID3"):
        transport.send_cycle([300] * 6, cycle_id=3, target_timestamp_ns=0)

    assert transport.stopped
    assert transport.audit_snapshot().stop_reason == "motion_write_failure"
    assert transport.audit_snapshot().delivery_uncertain
    assert [parse_frame(frame).motor_id for frame in bus.write_calls] == [1, 2, 3]
    assert transport.audit_snapshot().motion_retry_count == 0
    assert pauses == ["motion_write_failure"]
    assert bus.hardware_write_count == 0


def test_j7_c5_reject_residual_stops_before_next_batch() -> None:
    bus = SixAxisMockBus()
    pauses: list[str] = []
    transport = make_transport(
        bus,
        policy=ServoRxPolicy.REJECT_RESIDUAL,
        pause_events=pauses,
    )
    transport.configure_servo(speed=50)
    transport.send_cycle([400] * 6, cycle_id=4, target_timestamp_ns=0)

    with pytest.raises(RxSafetyError, match="residual"):
        transport.send_cycle([401] * 6, cycle_id=5, target_timestamp_ns=clock_ns(bus))

    assert transport.stopped
    assert pauses == ["rx_safety_failure"]
    assert len(bus.write_calls) == 6
    assert bus.hardware_write_count == 0


def clock_ns(_bus: SixAxisMockBus) -> int:
    """Return a valid non-negative timestamp for the reject-policy test."""
    return 1

