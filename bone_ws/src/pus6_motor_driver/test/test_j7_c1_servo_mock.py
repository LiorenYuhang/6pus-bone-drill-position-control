"""J7-C1 isolated Servo Mode scheduler validation.

This module is deliberately independent from the ROS 2 node and the real UART
bus.  It uses the production protocol frame builders only to verify frame
shape; no serial device can be opened by these tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Any

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.protocol import CMD_WR_REGISTER, pack_write_register_cmd, parse_frame
from pus6_motor_driver.safety import (
    CONTROL_MODE_ADDR,
    MOTOR_IDS,
    PAUSE_ADDR,
    TARGET_POSITION_ADDR,
    TARGET_SPEED_ADDR,
)


class VirtualClock:
    """Monotonic clock controlled by the test; no wall-clock sleeps are used."""

    def __init__(self, start_ns: int = 0) -> None:
        self._now_ns = start_ns

    def now_ns(self) -> int:
        return self._now_ns

    def set_ns(self, timestamp_ns: int) -> None:
        if timestamp_ns < self._now_ns:
            raise ValueError("VirtualClock cannot move backwards")
        self._now_ns = timestamp_ns

    def advance_ns(self, delta_ns: int) -> None:
        if delta_ns < 0:
            raise ValueError("VirtualClock delta must be non-negative")
        self._now_ns += delta_ns


class J7ServoMockBus:
    """Protocol-aware FakeBus for configuration, write-only motion, and pause."""

    def __init__(self, clock: VirtualClock, tx_duration_ns: int = 100_000) -> None:
        self.clock = clock
        self.tx_duration_ns = tx_duration_ns
        self.modes = {mid: 0 for mid in MOTOR_IDS}
        self.speeds = {mid: 0 for mid in MOTOR_IDS}
        self.targets = {mid: 0 for mid in MOTOR_IDS}
        self.config_audit: list[dict[str, Any]] = []
        self.read_audit: list[dict[str, Any]] = []
        self.motion_audit: list[dict[str, Any]] = []
        self.pause_audit: list[dict[str, Any]] = []
        self.fail_on_axis: int | None = None
        self.motion_attempts = {mid: 0 for mid in MOTOR_IDS}
        self.motion_retry_count = 0
        self.motion_response_wait_count = 0
        self.hardware_write_count = 0
        self.closed = False

    @staticmethod
    def _payload(frame: bytes) -> list[int]:
        parsed = parse_frame(frame)
        return [
            int.from_bytes(parsed.data[offset : offset + 2], "little")
            for offset in range(0, len(parsed.data), 2)
        ]

    def configure_axis(self, motor_id: int, mode: int, speed: int) -> None:
        """Send 0x25..0x28 configuration and return a simulated response."""
        frame = pack_write_register_cmd(motor_id, CONTROL_MODE_ADDR, [mode, 0, 0, speed])
        parsed = parse_frame(frame)
        assert parsed.command == CMD_WR_REGISTER
        assert parsed.register_addr == CONTROL_MODE_ADDR
        payload = self._payload(frame)
        start_ns = self.clock.now_ns()
        self.clock.advance_ns(self.tx_duration_ns)
        self.modes[motor_id] = payload[0]
        self.speeds[motor_id] = payload[3]
        self.config_audit.append(
            {
                "motor_id": motor_id,
                "register": parsed.register_addr,
                "payload": payload,
                "tx_start_ns": start_ns,
                "tx_end_ns": self.clock.now_ns(),
                "response": True,
            }
        )

    def read_register(self, motor_id: int, register: int) -> int:
        self.read_audit.append(
            {"motor_id": motor_id, "register": register, "timestamp_ns": self.clock.now_ns()}
        )
        if register == CONTROL_MODE_ADDR:
            return self.modes[motor_id]
        if register == TARGET_SPEED_ADDR:
            return self.speeds[motor_id]
        if register == TARGET_POSITION_ADDR:
            return self.targets[motor_id]
        raise ValueError(f"unsupported mock read register 0x{register:02X}")

    def write_motion(self, motor_id: int, target: int, cycle_timestamp_ns: int) -> dict[str, Any]:
        """Write one 0x29 frame without receiving or waiting for a response."""
        frame = pack_write_register_cmd(motor_id, TARGET_POSITION_ADDR, [target])
        parsed = parse_frame(frame)
        assert parsed.command == CMD_WR_REGISTER
        assert parsed.register_addr == TARGET_POSITION_ADDR
        self.motion_attempts[motor_id] += 1
        start_ns = self.clock.now_ns()
        self.clock.advance_ns(self.tx_duration_ns)
        end_ns = self.clock.now_ns()
        record = {
            "servo_cycle_timestamp_ns": cycle_timestamp_ns,
            "axis_id": motor_id,
            "target": target,
            "tx_start_ns": start_ns,
            "tx_end_ns": end_ns,
            "response_wait": False,
            "failed": self.fail_on_axis == motor_id,
        }
        self.motion_audit.append(record)
        if self.fail_on_axis == motor_id:
            raise OSError(f"simulated motion write failure on ID{motor_id}")
        self.targets[motor_id] = target
        return record

    def pause_all(self) -> None:
        """Record one best-effort pause write per axis for a graceful stop."""
        for motor_id in MOTOR_IDS:
            frame = pack_write_register_cmd(motor_id, PAUSE_ADDR, [1])
            parsed = parse_frame(frame)
            assert parsed.register_addr == PAUSE_ADDR
            start_ns = self.clock.now_ns()
            self.clock.advance_ns(self.tx_duration_ns)
            self.pause_audit.append(
                {
                    "axis_id": motor_id,
                    "tx_start_ns": start_ns,
                    "tx_end_ns": self.clock.now_ns(),
                    "response": True,
                }
            )

    def close(self) -> None:
        self.closed = True


class LatestTargetBuffer:
    """Single-slot target buffer with Latest-Value-Wins overwrite accounting."""

    def __init__(self) -> None:
        self.pending: tuple[list[int], int] | None = None
        self.last_update_ns: int | None = None
        self.overwrite_count = 0
        self.update_count = 0

    def update(self, targets: list[int], timestamp_ns: int) -> None:
        if self.pending is not None:
            self.overwrite_count += 1
        self.pending = (list(targets), timestamp_ns)
        self.last_update_ns = timestamp_ns
        self.update_count += 1

    def consume(self) -> tuple[list[int], int] | None:
        value = self.pending
        self.pending = None
        return value


@dataclass
class ServoCycleAudit:
    servo_cycle_timestamp_ns: int
    expected_cycle_timestamp_ns: int
    cycle_jitter_ns: int
    axis_tx: list[dict[str, Any]] = field(default_factory=list)
    batch_complete_time_ns: int | None = None
    deadline_miss: bool = False
    target_age_ns: int | None = None
    overwrite_count: int = 0
    watchdog_timeout: bool = False
    error: str | None = None


class J7ServoScheduler:
    """Deterministic 50 Hz write-only Servo scheduler for Mock validation."""

    def __init__(
        self,
        clock: VirtualClock,
        bus: J7ServoMockBus,
        *,
        rate_hz: float = 50.0,
        watchdog_timeout_sec: float = 0.10,
    ) -> None:
        if rate_hz != 50.0:
            raise ValueError("J7-C1 validation requires a fixed 50 Hz stream")
        self.clock = clock
        self.bus = bus
        self.period_ns = 20_000_000
        self.watchdog_timeout_ns = int(watchdog_timeout_sec * 1e9)
        self.targets = LatestTargetBuffer()
        self.cycle_audit: list[ServoCycleAudit] = []
        self.watchdog_events: list[dict[str, Any]] = []
        self.configured = False
        self.stopped = False
        self.stop_reason: str | None = None
        self.cycle_index = 0
        self._stream_started = False
        self._start_ns = clock.now_ns()

    @property
    def next_cycle_ns(self) -> int:
        return self._start_ns + self.cycle_index * self.period_ns

    def configure_servo(self, mode: int = 1, speed: int = 50) -> None:
        if mode != 1:
            raise ValueError("Servo Mode configuration must use mode=1")
        for motor_id in MOTOR_IDS:
            self.bus.configure_axis(motor_id, mode, speed)
        for motor_id in MOTOR_IDS:
            assert self.bus.read_register(motor_id, CONTROL_MODE_ADDR) == mode
            assert self.bus.read_register(motor_id, TARGET_SPEED_ADDR) == speed
        self.configured = True
        self._start_ns = self.clock.now_ns()
        self.cycle_index = 0

    def submit_targets(self, targets: list[int]) -> None:
        if len(targets) != len(MOTOR_IDS) or any(not isinstance(value, int) for value in targets):
            raise ValueError("Servo target must contain six integer positions")
        if self.stopped:
            raise RuntimeError("scheduler is stopped")
        self.targets.update(targets, self.clock.now_ns())
        self._stream_started = True

    def _graceful_stop(self, reason: str) -> None:
        if self.stopped:
            return
        self.stopped = True
        self.stop_reason = reason
        self.bus.pause_all()

    def graceful_stop(self) -> None:
        self._graceful_stop("requested")

    def run_cycle(self) -> ServoCycleAudit | None:
        if not self.configured or self.stopped:
            return None
        cycle_timestamp_ns = self.clock.now_ns()
        expected_ns = self.next_cycle_ns
        audit = ServoCycleAudit(
            servo_cycle_timestamp_ns=cycle_timestamp_ns,
            expected_cycle_timestamp_ns=expected_ns,
            cycle_jitter_ns=cycle_timestamp_ns - expected_ns,
            overwrite_count=self.targets.overwrite_count,
        )
        if self._stream_started and self.targets.last_update_ns is not None:
            age_ns = cycle_timestamp_ns - self.targets.last_update_ns
            audit.target_age_ns = age_ns
            if age_ns > self.watchdog_timeout_ns:
                audit.watchdog_timeout = True
                self.watchdog_events.append(
                    {
                        "timestamp_ns": cycle_timestamp_ns,
                        "target_age_ns": age_ns,
                        "timeout_ns": self.watchdog_timeout_ns,
                    }
                )
                self.cycle_audit.append(audit)
                self._graceful_stop("watchdog_timeout")
                self.cycle_index += 1
                return audit

        pending = self.targets.consume()
        if pending is None:
            self.cycle_audit.append(audit)
            self.cycle_index += 1
            return audit

        targets, submitted_ns = pending
        audit.target_age_ns = cycle_timestamp_ns - submitted_ns
        try:
            for motor_id, target in zip(MOTOR_IDS, targets):
                audit.axis_tx.append(self.bus.write_motion(motor_id, target, cycle_timestamp_ns))
            audit.batch_complete_time_ns = self.clock.now_ns()
            audit.deadline_miss = (
                audit.batch_complete_time_ns - cycle_timestamp_ns > self.period_ns
            )
        except Exception as exc:
            audit.error = str(exc)
            audit.batch_complete_time_ns = self.clock.now_ns()
            self.cycle_audit.append(audit)
            self._graceful_stop("motion_write_failure")
            self.cycle_index += 1
            raise

        self.cycle_audit.append(audit)
        self.cycle_index += 1
        return audit


def _advance_to_next(scheduler: J7ServoScheduler) -> None:
    scheduler.clock.set_ns(scheduler.next_cycle_ns)


def test_j7_c1_servo_mode_configuration_and_readback() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock)
    scheduler = J7ServoScheduler(clock, bus)
    scheduler.configure_servo(mode=1, speed=50)

    assert scheduler.configured
    assert len(bus.config_audit) == 6
    assert all(item["register"] == CONTROL_MODE_ADDR for item in bus.config_audit)
    assert all(item["payload"] == [1, 0, 0, 50] for item in bus.config_audit)
    assert all(item["response"] for item in bus.config_audit)
    assert len(bus.read_audit) == 12
    assert set(bus.modes.values()) == {1}
    assert set(bus.speeds.values()) == {50}
    assert bus.motion_audit == []
    assert bus.hardware_write_count == 0


def test_j7_c1_normal_50hz_write_only_stream_and_audit() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock)
    scheduler = J7ServoScheduler(clock, bus)
    scheduler.configure_servo()

    for cycle in range(5):
        _advance_to_next(scheduler)
        scheduler.submit_targets([100 + cycle] * 6)
        audit = scheduler.run_cycle()
        assert audit is not None
        assert len(audit.axis_tx) == 6
        assert not audit.deadline_miss
        assert not audit.watchdog_timeout

    cycle_timestamps = [item.servo_cycle_timestamp_ns for item in scheduler.cycle_audit]
    assert cycle_timestamps == [i * scheduler.period_ns + scheduler._start_ns for i in range(5)]
    assert all(
        cycle_timestamps[index + 1] - cycle_timestamps[index] == scheduler.period_ns
        for index in range(4)
    )
    assert len(bus.motion_audit) == 30
    assert bus.read_audit == bus.read_audit[:12]  # no reads after configuration
    assert bus.motion_response_wait_count == 0
    assert bus.motion_retry_count == 0
    assert all(item["response_wait"] is False for item in bus.motion_audit)
    assert scheduler.targets.overwrite_count == 0
    assert all(item.cycle_jitter_ns == 0 for item in scheduler.cycle_audit)
    assert bus.hardware_write_count == 0


def test_j7_c1_latest_value_overwrite_and_deadline_audit() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock, tx_duration_ns=4_000_000)
    scheduler = J7ServoScheduler(clock, bus, watchdog_timeout_sec=0.20)
    scheduler.configure_servo()
    _advance_to_next(scheduler)
    scheduler.submit_targets([101] * 6)
    scheduler.submit_targets([102] * 6)
    audit = scheduler.run_cycle()

    assert audit is not None
    assert audit.overwrite_count == 1
    assert [item["target"] for item in bus.motion_audit] == [102] * 6
    assert audit.deadline_miss
    assert audit.batch_complete_time_ns - audit.servo_cycle_timestamp_ns == 24_000_000
    assert bus.hardware_write_count == 0


def test_j7_c1_watchdog_stops_stale_target_stream() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock)
    scheduler = J7ServoScheduler(clock, bus, watchdog_timeout_sec=0.05)
    scheduler.configure_servo()

    _advance_to_next(scheduler)
    scheduler.submit_targets([200] * 6)
    scheduler.run_cycle()
    for _ in range(3):
        _advance_to_next(scheduler)
        scheduler.run_cycle()

    assert scheduler.stopped
    assert scheduler.stop_reason == "watchdog_timeout"
    assert len(scheduler.watchdog_events) == 1
    assert scheduler.watchdog_events[0]["target_age_ns"] > 50_000_000
    assert len(bus.motion_audit) == 6
    assert len(bus.pause_audit) == 6
    assert all(item.watchdog_timeout is False for item in scheduler.cycle_audit[:-1])
    assert scheduler.cycle_audit[-1].watchdog_timeout
    assert bus.hardware_write_count == 0


def test_j7_c1_motion_write_failure_has_no_retry_and_pauses() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock)
    scheduler = J7ServoScheduler(clock, bus)
    scheduler.configure_servo()
    bus.fail_on_axis = 3
    _advance_to_next(scheduler)
    scheduler.submit_targets([300] * 6)

    with pytest.raises(OSError, match="ID3"):
        scheduler.run_cycle()

    assert scheduler.stopped
    assert scheduler.stop_reason == "motion_write_failure"
    assert bus.motion_attempts[1] == 1
    assert bus.motion_attempts[2] == 1
    assert bus.motion_attempts[3] == 1
    assert bus.motion_attempts[4] == 0
    assert bus.motion_attempts[5] == 0
    assert bus.motion_attempts[6] == 0
    assert len(bus.motion_audit) == 3
    assert bus.motion_retry_count == 0
    assert len(bus.pause_audit) == 6
    assert bus.hardware_write_count == 0


def test_j7_c1_explicit_graceful_stop_is_idempotent() -> None:
    clock = VirtualClock()
    bus = J7ServoMockBus(clock)
    scheduler = J7ServoScheduler(clock, bus)
    scheduler.configure_servo()

    scheduler.graceful_stop()
    scheduler.graceful_stop()
    assert scheduler.stopped
    assert scheduler.stop_reason == "requested"
    assert len(bus.pause_audit) == 6
    assert bus.hardware_write_count == 0
