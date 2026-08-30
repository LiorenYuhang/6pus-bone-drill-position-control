"""Unit tests for production J7 Six-Axis 50Hz Servo HOME and PARK runner."""

from __future__ import annotations

import math
import time
from pathlib import Path
import sys
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = Path(__file__).resolve().parents[3] / "scripts"
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import run_j7_servo_home_park as home_park_runner
from run_j7_servo_home_park import (
    HOME_TARGET,
    PARK_TARGET,
    MockServoBus,
    STANDALONE_WRITE_TIMEOUT_S,
    quintic_progress,
    run_servo_home_park,
)
from pus6_motor_driver.safety import MOTOR_IDS


def test_quintic_progress_boundary_and_monotonicity():
    """Verify quintic polynomial s(0)=0, s(1)=1, derivative=0 at ends."""
    assert quintic_progress(0.0) == pytest.approx(0.0)
    assert quintic_progress(1.0) == pytest.approx(1.0)
    assert quintic_progress(-0.5) == 0.0
    assert quintic_progress(1.5) == 1.0

    prev = 0.0
    for i in range(1, 101):
        tau = i / 100.0
        val = quintic_progress(tau)
        assert val >= prev
        prev = val

    dt = 1e-4
    mid_deriv = (quintic_progress(0.5 + dt) - quintic_progress(0.5 - dt)) / (2 * dt)
    assert mid_deriv == pytest.approx(1.875, abs=1e-3)


def test_home_target_constant():
    """Requirement 1: HOME target = [1000]*6."""
    assert HOME_TARGET == (1000, 1000, 1000, 1000, 1000, 1000)


def test_park_target_constant():
    """Requirement 2: PARK target = [0]*6."""
    assert PARK_TARGET == (0, 0, 0, 0, 0, 0)


def test_standalone_write_timeout_is_bounded_at_25ms():
    """Requirement 1: standalone HOME/PARK bounded write timeout is 25 ms."""
    assert STANDALONE_WRITE_TIMEOUT_S == pytest.approx(0.025)


def test_mock_home_execution_from_park():
    """Test full HOME action from PARK position in mock environment."""
    bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))
    res = run_servo_home_park(
        action="home",
        speed=200.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is True
    assert res.q_initial == (0, 0, 0, 0, 0, 0)
    assert res.q_final == HOME_TARGET
    assert res.fault_count == 0
    assert res.timeout_count == 0
    assert res.stop_success is True


def test_mock_park_execution_from_home():
    """Test full PARK action from HOME position in mock environment."""
    bus = MockServoBus(initial_positions=(1000, 1000, 1000, 1000, 1000, 1000))
    res = run_servo_home_park(
        action="park",
        speed=200.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is True
    assert res.q_initial == (1000, 1000, 1000, 1000, 1000, 1000)
    assert res.q_final == PARK_TARGET
    assert res.fault_count == 0
    assert res.timeout_count == 0
    assert res.stop_success is True


def test_arbitrary_initial_positions_smooth_planning():
    """Requirement 3 & 5: Arbitrary non-uniform initial positions planned smoothly."""
    arbitrary_init = (250, 400, 120, 890, 600, 750)
    bus = MockServoBus(initial_positions=arbitrary_init)
    res = run_servo_home_park(
        action="home",
        speed=150.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is True
    assert res.q_initial == arbitrary_init
    assert res.q_final == HOME_TARGET
    assert res.max_tracking_err <= 1


def test_action_validation_and_safety_rejection():
    """Requirement 6: Invalid action names and unconfirmed real hardware rejected."""
    with pytest.raises(ValueError, match="Invalid action"):
        run_servo_home_park(action="invalid_action", sleep_fn=lambda _: None)

    with pytest.raises(ValueError, match="--real-hardware requires --confirm-motion"):
        run_servo_home_park(
            action="home",
            real_hardware=True,
            confirm_motion=False,
            sleep_fn=lambda _: None,
        )


def test_fault_triggers_safety_stop():
    """Requirement 7: Motor fault during motion triggers immediate STOP."""
    bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))
    bus.fault_on_axis = 4  # Overcurrent / driver fault on ID1
    res = run_servo_home_park(
        action="home",
        speed=100.0,
        rate_hz=50.0,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is False
    assert res.stop_success is True


def test_timeout_triggers_safety_stop():
    """Requirement 8: UART feedback timeout triggers immediate safety halt."""
    bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))
    bus.timeout_on_axis = 1  # Timeout on ID1
    res = run_servo_home_park(
        action="home",
        speed=100.0,
        rate_hz=50.0,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is False


def test_park_special_storage_position_allowed():
    """Requirement 10: PARK allows position 0 without being rejected by [200, 1800] envelope."""
    bus = MockServoBus(initial_positions=(1000, 1000, 1000, 1000, 1000, 1000))
    res = run_servo_home_park(
        action="park",
        speed=200.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is True
    assert res.q_final == (0, 0, 0, 0, 0, 0)
    for pos in res.q_final:
        assert pos == 0


def test_small_negative_measured_startup_accepted_and_homes_safely():
    """Verify that power-on resting positions [-3, -5, -2, -2, -4, 0] within [-5, 2005] home smoothly."""
    q_init = (-3, -5, -2, -2, -4, 0)
    bus = MockServoBus(initial_positions=q_init)
    res = run_servo_home_park(
        action="home",
        speed=200.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is True
    assert res.q_initial == q_init
    assert res.q_final == HOME_TARGET
    assert res.fault_count == 0
    assert res.timeout_count == 0


def test_clearly_invalid_negative_measurement_still_rejected():
    """Verify that a position outside [-5, 2005] (e.g. -15) is rejected during preflight."""
    bus = MockServoBus(initial_positions=(-15, 0, 0, 0, 0, 0))
    res = run_servo_home_park(
        action="home",
        speed=200.0,
        rate_hz=50.0,
        mock_bus=bus,
        sleep_fn=lambda _: None,
    )
    assert res.passed is False
    assert res.total_cycles == 0
    assert res.stop_success is True


def test_negative_command_still_strictly_rejected():
    """Verify that validate_target_vector strictly forbids negative commanded targets."""
    from pus6_motor_driver.safety import TargetValidationError, validate_target_vector

    with pytest.raises(TargetValidationError, match="outside safe boundary"):
        validate_target_vector([-1, 1000, 1000, 1000, 1000, 1000])


def test_trajectory_from_negative_measured_startup_begins_at_zero():
    """Verify that trajectory plan starting reference is clamped to >= 0 so cycle 0 is non-negative."""
    from pus6_motor_driver.safety import ABSOLUTE_RAW_LOWER_LIMIT, ABSOLUTE_RAW_UPPER_LIMIT

    q_initial = (-3, -5, -2, -2, -4, 0)
    target = (1000, 1000, 1000, 1000, 1000, 1000)
    q_plan_start = tuple(
        max(ABSOLUTE_RAW_LOWER_LIMIT, min(ABSOLUTE_RAW_UPPER_LIMIT, q_initial[i]))
        for i in range(6)
    )
    assert q_plan_start == (0, 0, 0, 0, 0, 0)
    for c in range(1, 100):
        s = quintic_progress(c / 100.0)
        tgt = tuple(
            max(
                ABSOLUTE_RAW_LOWER_LIMIT,
                min(
                    ABSOLUTE_RAW_UPPER_LIMIT,
                    int(round(q_plan_start[i] + (target[i] - q_plan_start[i]) * s)),
                ),
            )
            for i in range(6)
        )
        assert all(0 <= v <= 1000 for v in tgt)


def test_tx_write_failure_provenance_recording():
    """Verify that when a write failure occurs, full passive provenance is recorded."""
    from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7AdapterError
    from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy

    class FailingSerial:
        def __init__(self):
            self.out_waiting = 16
            self.in_waiting = 0
            self.write_timeout = 0.015

        def write(self, data):
            raise TimeoutError("Write timeout")

        def read(self, size):
            return b""

    failing_ser = FailingSerial()
    adapter = J7UartAdapter(failing_ser, timeout_s=0.015)
    transport = J7SixAxisServoTransport(
        adapter,
        cycle_rate_hz=50.0,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
    )
    # Simulate configuration
    transport._configured = tuple()

    with pytest.raises(Exception):
        transport.send_cycle([1000] * 6, cycle_id=12)

    prov = transport.last_tx_failure_provenance
    assert prov is not None
    assert prov["cycle_id"] == 12
    assert prov["motor_id"] == 1
    assert prov["command"] == "0x32"
    assert prov["register_address"] == 0x29
    assert prov["target_step"] == 1000
    assert prov["frame_length"] == 10
    assert prov["serial_out_waiting"] == 16
    assert prov["write_timeout"] == 0.015
    assert prov["delivery_uncertain"] is True
    assert prov["stop_reason"] == "motion_write_failure"
    assert "exception_type" in prov
    assert "exception_message" in prov


def test_runner_propagates_tx_failure_provenance(monkeypatch):
    """Verify that run_servo_home_park propagates last_tx_failure_provenance upon TX exception."""
    from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport

    mock_bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))

    original_send_cycle = J7SixAxisServoTransport.send_cycle

    def failing_send_cycle(self, targets, *, cycle_id, target_timestamp_ns=None):
        if cycle_id >= 2:
            self.last_tx_failure_provenance = {
                "monotonic_timestamp_ns": 123456789,
                "cycle_id": cycle_id,
                "motor_id": 3,
                "command": "0x29",
                "register_address": 0x29,
                "target_step": 15,
                "frame_length": 16,
                "raw_frame_hex": "aa55...",
                "elapsed_write_time_ms": 15.2,
                "batch_elapsed_time_ms": 18.4,
                "serial_out_waiting": 16,
                "serial_in_waiting": 0,
                "write_timeout": 0.015,
                "last_successful_motor_id": 2,
                "delivery_uncertain": True,
                "stop_reason": "motion_write_failure",
            }
            raise RuntimeError("simulated write timeout")
        return original_send_cycle(self, targets, cycle_id=cycle_id, target_timestamp_ns=target_timestamp_ns)

    monkeypatch.setattr(J7SixAxisServoTransport, "send_cycle", failing_send_cycle)

    res = run_servo_home_park(
        action="home",
        mock_bus=mock_bus,
        speed=100.0,
        rate_hz=50.0,
        hold_duration_s=0.1,
    )

    assert res.passed is False
    assert res.last_tx_failure_provenance is not None
    assert res.last_tx_failure_provenance["cycle_id"] == 2
    assert res.last_tx_failure_provenance["motor_id"] == 3
    assert res.last_tx_failure_provenance["write_timeout"] == 0.015
    assert res.last_tx_failure_provenance["delivery_uncertain"] is True
    assert "cycle_elapsed_at_failure_ms" in res.last_tx_failure_provenance
    assert "emergency_stop_duration_ms" in res.last_tx_failure_provenance
    assert res.transport_exception_timestamp_ns is not None
    assert res.first_internal_stop_start_ns is not None
    assert res.first_internal_stop_end_ns is not None
    assert res.first_internal_stop_duration_ms >= 0.0
    assert res.outer_redundant_stop_called is True
    assert res.transport_failure_to_first_stop_complete_latency_ms >= 0.0


def test_design_b_motion_consumes_all_six_feedbacks_and_eliminates_0x30_polling():
    """Verify that during motion, all six 0x32 feedbacks are consumed and no 0x30 polls occur."""
    mock_bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))

    res = run_servo_home_park(
        action="home",
        mock_bus=mock_bus,
        speed=200.0,
        rate_hz=50.0,
        hold_duration_s=0.05,
        sleep_fn=lambda _: None,
    )

    assert res.passed is True
    assert res.total_cycles > 0
    assert len(mock_bus.rx_queue) == 0, "No residual unconsumed response frames should remain in RX queue"

    # Analyze written frames
    from pus6_motor_driver.protocol import CMD_RD_STATUS, CMD_WR_REGISTER, parse_frame
    parsed_writes = [parse_frame(f) for f in mock_bus.written_frames]

    # Startup preflight has 3 scans * 6 = 18 0x30 frames, Phase 4 has 6 0x30 frames.
    # Motion itself must have ZERO 0x30 frames!
    status_0x30_frames = [p for p in parsed_writes if p.command == CMD_RD_STATUS]
    assert len(status_0x30_frames) == 18 + 6, f"Expected 24 pre/post 0x30 frames, got {len(status_0x30_frames)}"

    # Motion writes must all be 0x32 target position writes
    motion_writes = [p for p in parsed_writes if p.command == CMD_WR_REGISTER and p.register_addr == 0x29]
    assert len(motion_writes) == res.total_cycles * 6


def test_deadline_miss_is_recorded_without_auto_stop(monkeypatch):
    """Verify that a >20ms cycle is audited but does not stop the standalone run by itself."""
    from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport

    mock_bus = MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0))
    original_send_cycle = J7SixAxisServoTransport.send_cycle
    call_count = 0

    def slow_send_cycle(self, targets, *, cycle_id, target_timestamp_ns=None):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            time.sleep(0.025)
        return original_send_cycle(self, targets, cycle_id=cycle_id, target_timestamp_ns=target_timestamp_ns)

    monkeypatch.setattr(J7SixAxisServoTransport, "send_cycle", slow_send_cycle)

    res = run_servo_home_park(
        action="home",
        mock_bus=mock_bus,
        speed=10_000.0,
        rate_hz=50.0,
        hold_duration_s=0.0,
        sleep_fn=lambda _: None,
    )

    assert res.passed is True
    assert res.deadline_miss_count >= 1
    assert res.stop_success is True
    assert res.outer_redundant_stop_called is False




