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
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.protocol import CMD_RD_STATUS, CMD_WR_REGISTER, parse_frame
from pus6_motor_driver.six_motor_controller import SixMotorController
from pus6_motor_driver.safety import (
    CONTROL_MODE_ADDR,
    TARGET_POSITION_ADDR,
    OPERATING_HOME_RAW_STEPS,
    STORAGE_PARK_RAW_STEPS,
    DEFAULT_SERIAL_PORT,
    MOTOR_IDS,
    raw_to_relative_steps,
    relative_to_raw_steps,
)
from run_j3_startup_homing import run_homing_session


class TrackingWatchdogBus(FakeBus):
    """Mock feedback source for transient/persistent watchdog tests."""

    def __init__(self, scenario: str):
        super().__init__()
        self.scenario = scenario
        self.status_cycle = -1
        self.transient_used = False

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        if parsed.command == CMD_RD_STATUS:
            if parsed.motor_id == MOTOR_IDS[0]:
                self.status_cycle += 1
                if self.status_cycle == 0:
                    self.positions = {
                        mid: value
                        for mid, value in zip(MOTOR_IDS, [0, 0, -2, -1, -2, -2])
                    }
                elif self.scenario == "transient":
                    if not self.transient_used and max(self.targets.values()) >= 20:
                        self.positions = {
                            mid: max(0, self.targets[mid] - 11)
                            for mid in MOTOR_IDS
                        }
                        self.transient_used = True
                    else:
                        self.positions = dict(self.targets)
                elif self.scenario == "persistent":
                    if max(self.targets.values()) >= 20:
                        self.positions = {
                            mid: max(0, self.targets[mid] - 11)
                            for mid in MOTOR_IDS
                        }
                    else:
                        self.positions = dict(self.targets)
                elif self.scenario == "fault":
                    self.faults[1] = 1
                elif self.scenario == "communication":
                    raise OSError("simulated feedback communication failure")
        return super().request(frame, **kwargs)


def _run_watchdog_session(scenario: str):
    bus = TrackingWatchdogBus(scenario)
    controller = SixMotorController(bus=bus)
    return run_homing_session(
        action="home",
        speed=50,
        control_rate_hz=10.0,
        real_hardware=False,
        custom_start=[0, 0, -2, -1, -2, -2],
        duration=0.5 if scenario == "transient" else 4.0,
        settle_time=0.0,
        sleep_fn=lambda _seconds: None,
        stop_verify_interval_s=0.0,
        controller=controller,
    )


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



def test_single_transient_tracking_violation_does_not_stop():
    report = _run_watchdog_session("transient")

    assert report["safety_aborted"] is False
    assert report["abort_reason"] == ""
    assert report["q_final_raw"] == [1000] * 6


def test_persistent_tracking_violation_still_stops_after_debounce():
    report = _run_watchdog_session("persistent")

    assert report["safety_aborted"] is True
    assert "Tracking failure" in report["abort_reason"]
    assert "consecutive samples" in report["abort_reason"]
    assert report["stop_result"]["pause_results"]


def test_motor_fault_still_stops_immediately():
    report = _run_watchdog_session("fault")

    assert report["safety_aborted"] is True
    assert "Motor hardware fault" in report["abort_reason"]


def test_feedback_communication_failure_still_stops_immediately():
    report = _run_watchdog_session("communication")

    assert report["safety_aborted"] is True
    assert "Communication failure during feedback read" in report["abort_reason"]


@pytest.mark.parametrize(
    ("action", "custom_start", "expected_target"),
    [
        ("home", [0, 0, 0, 0, 0, 0], [1000, 1000, 1000, 1000, 1000, 1000]),
        ("park", [1000, 1000, 1000, 1000, 1000, 1000], [0, 0, 0, 0, 0, 0]),
    ],
)
def test_home_park_phase2_positioning_mode_preserves_target_dispatch(
    action, custom_start, expected_target
):
    """Commissioning uses mode 0 while retaining the 0x29 trajectory path."""
    bus = FakeBus()
    controller = SixMotorController(bus=bus)
    report = run_homing_session(
        action=action,
        speed=50,
        control_rate_hz=10.0,
        real_hardware=False,
        custom_start=custom_start,
        duration=0.1,
        settle_time=0.0,
        stop_verify_interval_s=0.0,
        controller=controller,
    )

    assert report["q_target_raw"] == expected_target
    assert report["q_final_raw"] == expected_target

    writes = [event for event in bus.events if event[1] == CMD_WR_REGISTER]
    config_writes = [event for event in writes if event[2] == CONTROL_MODE_ADDR]
    target_writes = [event for event in writes if event[2] == TARGET_POSITION_ADDR]

    assert len(config_writes) == 6
    assert all(event[3][0] == 0 for event in config_writes)
    assert all(len(event[3]) == 4 for event in config_writes)
    assert all(event[3][3] == report["motor_reg_speed_0x28"] for event in config_writes)
    assert all(event[2] != TARGET_POSITION_ADDR for event in writes[:6])

    assert target_writes
    assert all(event[2] == TARGET_POSITION_ADDR for event in target_writes)
    assert [event[0] for event in target_writes[-6:]] == list(MOTOR_IDS)
    assert [event[3][0] for event in target_writes[-6:]] == expected_target
