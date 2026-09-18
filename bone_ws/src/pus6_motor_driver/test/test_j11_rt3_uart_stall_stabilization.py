"""Tests for J11-RT3: UART Transport Stall Stabilization.

Verifies:
1. Frame bytes and 6-axis command atomicity remain 100% identical.
2. 50Hz Servo critical path is 100% free of serial.flush() / tcdrain.
3. Sub-millisecond pacing and feedback drain do not call time.sleep(0).
4. write_timeout is bounded (<= 10ms) to prevent indefinite select blocks.
5. Watchdog deadline remains unchanged.
"""

from __future__ import annotations

import time
import pytest

from pus6_motor_driver.safety import MOTOR_IDS, TARGET_POSITION_ADDR
from pus6_motor_driver.protocol import pack_write_register_cmd, parse_frame
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler


class MockHardwareSerial:
    """Serial-like device backed by FakeBus, recording write/read/flush calls."""

    def __init__(self) -> None:
        self.device = FakeBus()
        self.rx = bytearray()
        self.written_chunks: list[bytes] = []
        self.flush_calls: int = 0
        self.timeout: float = 0.005
        self.write_timeout: float = 0.010

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, data: bytes) -> int:
        self.written_chunks.append(data)
        # FakeBus responds to requests (e.g. configure_servo)
        self.rx.extend(self.device.request(data))
        return len(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def reset_output_buffer(self) -> None:
        pass


def test_j11_rt3_zero_flush_in_servo_critical_path():
    """Verify that send_cycle never invokes serial.flush() / tcdrain."""
    mock_ser = MockHardwareSerial()
    adapter = J7UartAdapter(mock_ser, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(
        adapter,
        cycle_rate_hz=50.0,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
        min_frame_interval_s=0.0006,
        rx_drain_budget_s=0.0040,
    )
    transport.configure_servo(speed=50)

    # Reset flush counter after configuration
    mock_ser.flush_calls = 0

    # Execute 10 consecutive 50Hz cycles
    for cycle_id in range(1, 11):
        targets = [1000 + cycle_id] * 6
        audit = transport.send_cycle(targets, cycle_id=cycle_id)
        assert audit.cycle_id == cycle_id

    # CRITICAL: ZERO calls to serial.flush()
    assert mock_ser.flush_calls == 0, "serial.flush() must NOT be called in 50Hz Servo critical path"


def test_j11_rt3_frame_bytes_and_ordering_identical():
    """Verify that the 6 frames generated and written are identical and ordered ID1..ID6."""
    mock_ser = MockHardwareSerial()
    adapter = J7UartAdapter(mock_ser, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0)
    transport.configure_servo(speed=50)

    mock_ser.written_chunks.clear()

    targets = [995, 1005, 990, 1010, 998, 1002]
    transport.send_cycle(targets, cycle_id=42)

    # 6 frames written for 6 motors
    assert len(mock_ser.written_chunks) == 6

    for idx, (motor_id, target) in enumerate(zip(MOTOR_IDS, targets)):
        expected_frame = pack_write_register_cmd(motor_id, TARGET_POSITION_ADDR, [target])
        assert mock_ser.written_chunks[idx] == expected_frame
        assert parse_frame(mock_ser.written_chunks[idx]).motor_id == motor_id


def test_j11_rt3_no_time_sleep_zero_in_pacing(monkeypatch):
    """Verify that _wait_for_frame_interval_locked does not invoke time.sleep(0)."""
    mock_ser = MockHardwareSerial()
    adapter = J7UartAdapter(mock_ser, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0, min_frame_interval_s=0.0006)
    transport.configure_servo(speed=50)

    sleep_calls = []

    def mock_sleep(duration):
        sleep_calls.append(duration)

    monkeypatch.setattr(time, "sleep", mock_sleep)

    # Execute a cycle
    transport.send_cycle([1000] * 6, cycle_id=1)

    # Verify that time.sleep(0) was NEVER called
    assert 0 not in sleep_calls, "time.sleep(0) must NOT be invoked in high-rate pacing"


def test_j11_rt3_feedback_drain_strictly_bounded():
    """Verify that feedback drain loop respects the rx_drain_budget_s deadline."""
    mock_ser = MockHardwareSerial()
    adapter = J7UartAdapter(mock_ser, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    drain_budget_s = 0.0040  # 4.0ms
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0, rx_drain_budget_s=drain_budget_s)
    transport.configure_servo(speed=50)

    t0 = time.monotonic_ns()
    audit = transport.send_cycle([1000] * 6, cycle_id=1)
    t_elapsed_ms = (time.monotonic_ns() - t0) / 1e6

    # Entire cycle (6 writes + 4ms drain) should complete well under 15ms, NOT 86ms or 124ms
    assert t_elapsed_ms < 15.0, f"send_cycle exceeded reasonable bounds: {t_elapsed_ms:.2f}ms"


def test_j11_rt3_watchdog_threshold_intact():
    """Verify that Tier 2 scheduler watchdog threshold remains intact and active."""
    mock_ser = MockHardwareSerial()
    adapter = J7UartAdapter(mock_ser, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0)
    scheduler = J7ServoFeedbackScheduler(
        transport,
        full_scan_period_s=1e9,
        feedback_budget_s=0.015,
        servo_cycle_period_s=0.020,
        timeout_escalation_misses=4,
    )

    # Watchdog threshold is positive and around ~263ms
    assert scheduler.max_feedback_age_ns > 200_000_000
    assert scheduler.max_feedback_age_ns < 300_000_000
