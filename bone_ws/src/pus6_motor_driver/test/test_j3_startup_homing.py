"""Test suite for J3 Startup Homing, Controlled Parking, and Dual-Pose Management."""

from __future__ import annotations

import sys
import inspect
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)
SCRIPTS_PATH = str(WORKSPACE_ROOT / "scripts")
if SCRIPTS_PATH not in sys.path:
    sys.path.insert(0, SCRIPTS_PATH)

import pytest
from pus6_control.joint_trajectory_controller import (
    compute_synchronized_step_point,
    compute_synchronized_trajectory_point,
    quintic_progress,
)
from pus6_motor_driver.safety import (
    OPERATING_HOME_RAW_STEPS,
    STORAGE_PARK_RAW_STEPS,
    DEFAULT_SERIAL_PORT,
    raw_to_relative_steps,
    relative_to_raw_steps,
)
from run_j3_startup_homing import run_homing_session


def test_homing_uses_validated_serial_port_default():
    """Standalone HOME/PARK must retain the shared validated by-id default."""
    port_parameter = inspect.signature(run_homing_session).parameters["port"]
    assert port_parameter.default == DEFAULT_SERIAL_PORT
    assert DEFAULT_SERIAL_PORT == "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


def test_quintic_progress_curve():
    """Verify S-curve boundary values, symmetry, and monotonicity."""
    assert quintic_progress(0.0) == 0.0
    assert quintic_progress(1.0) == 1.0
    assert abs(quintic_progress(0.5) - 0.5) < 1e-6

    # Clamping
    assert quintic_progress(-0.5) == 0.0
    assert quintic_progress(1.5) == 1.0

    # Monotonicity test
    samples = [quintic_progress(i / 100.0) for i in range(101)]
    assert all(samples[i] <= samples[i + 1] for i in range(100))


def test_quintic_peak_derivative_and_duration_scaling():
    """Verify max(ds/dtau) == 1.875 at tau = 0.5 and T >= 1.875 * D_max / v_peak."""
    # Numerical derivative estimation: ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4
    # At tau = 0.5: ds/dtau = 30*(0.25) - 60*(0.125) + 30*(0.0625) = 7.5 - 7.5 + 1.875 = 1.875
    dt = 1e-6
    tau_mid = 0.5
    ds_dtau_mid = (quintic_progress(tau_mid + dt) - quintic_progress(tau_mid - dt)) / (2.0 * dt)
    assert abs(ds_dtau_mid - 1.875) < 1e-4

    # For D_max = 975 steps and v_peak = 50 step/s: T = 1.875 * 975 / 50 = 36.5625s
    d_max = 975
    v_peak = 50.0
    expected_T = 1.875 * d_max / v_peak
    assert abs(expected_T - 36.5625) < 1e-6



def test_synchronized_trajectory_proportional_progress():
    """Verify 6 axes with different initial positions move strictly synchronously."""
    start = [74, 86, 44, 88, 36, 25]
    target = [1000, 1000, 1000, 1000, 1000, 1000]
    total_T = 20.0

    # At t = 0 -> exact start
    p0 = compute_synchronized_step_point(start, target, 0.0, total_T)
    assert p0 == start

    # At t = 10 (halfway tau=0.5, s=0.5) -> exactly midpoint for every axis
    p_half = compute_synchronized_step_point(start, target, 10.0, total_T)
    expected_half = [int(round(s + 0.5 * (1000 - s))) for s in start]
    assert p_half == expected_half

    # At t = 20 -> exact target
    p_end = compute_synchronized_step_point(start, target, 20.0, total_T)
    assert p_end == target


def test_dual_pose_relative_transformations():
    """Verify raw step <-> software relative step conversions."""
    # Operating Home raw=1000 -> relative=0
    assert raw_to_relative_steps(1000) == 0
    assert raw_to_relative_steps(list(OPERATING_HOME_RAW_STEPS)) == [0, 0, 0, 0, 0, 0]

    # Storage Park raw=0 -> relative=-1000
    assert raw_to_relative_steps(0) == -1000
    assert raw_to_relative_steps(list(STORAGE_PARK_RAW_STEPS)) == [-1000, -1000, -1000, -1000, -1000, -1000]

    # Delta heave +10 steps from home
    assert raw_to_relative_steps(1010) == 10
    assert relative_to_raw_steps(10) == 1010
    assert relative_to_raw_steps([0, 0, 0, 0, 0, 0]) == [1000, 1000, 1000, 1000, 1000, 1000]


def test_startup_homing_mock_execution():
    """Test full startup homing session in mock mode."""
    custom_baseline = [74, 86, 44, 88, 36, 25]
    report = run_homing_session(
        action="home",
        speed=100,
        control_rate_hz=20.0,
        real_hardware=False,
        custom_start=custom_baseline,
        duration=0.1,
        settle_time=0.01,
    )

    assert not report["safety_aborted"]
    assert report["q_start_raw"] == custom_baseline
    assert report["q_target_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
    assert report["q_final_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
    assert report["q_final_relative"] == [0, 0, 0, 0, 0, 0]
    assert report["tracking_tolerable"] is True


def test_controlled_parking_mock_execution():
    """Test controlled parking from OPERATING_HOME to STORAGE_PARK."""
    start_at_home = [1000, 1000, 1000, 1000, 1000, 1000]
    report = run_homing_session(
        action="park",
        speed=100,
        control_rate_hz=20.0,
        real_hardware=False,
        custom_start=start_at_home,
        duration=0.1,
        settle_time=0.01,
    )

    assert not report["safety_aborted"]
    assert report["q_start_raw"] == start_at_home
    assert report["q_target_raw"] == [0, 0, 0, 0, 0, 0]
    assert report["q_final_raw"] == [0, 0, 0, 0, 0, 0]
    assert report["q_final_relative"] == [-1000, -1000, -1000, -1000, -1000, -1000]
    assert report["tracking_tolerable"] is True
