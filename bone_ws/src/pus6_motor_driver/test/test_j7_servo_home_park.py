"""Unit tests for production J7 Six-Axis 50Hz Servo HOME and PARK runner."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = Path(__file__).resolve().parents[3] / "scripts"
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from run_j7_servo_home_park import (
    HOME_TARGET,
    PARK_TARGET,
    MockServoBus,
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
