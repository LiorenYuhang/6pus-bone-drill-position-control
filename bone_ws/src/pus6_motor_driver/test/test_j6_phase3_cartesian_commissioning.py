from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for path in (
    ROOT / "scripts",
    ROOT / "src" / "pus6_motor_driver",
    ROOT / "src" / "pus6_kinematics",
    ROOT / "src" / "pus6_control",
):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase3_cartesian_commissioning import (  # noqa: E402
    ALLOWED_TESTS,
    HOME_RAW_STEPS,
    PARK_RAW_STEPS,
    CartesianCommissioningSession,
    CommissioningState,
    make_mock_controller,
    run_j6_phase3_cartesian_commissioning,
)
from pus6_motor_driver.safety import HardwareMode, SafetyViolation, TargetValidationError  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


SAFE_OBSERVATIONS = {
    "platform_motion": "expected",
    "ball_joint_condition": "clear",
    "interference": "none",
    "abnormal_sound": "none",
    "observer_notes": "mock evidence",
}
MOCK_EVIDENCE = {
    "imu_base": {"tid": 1, "host_monotonic_arrival_time_ns": 100},
    "imu_platform": {"tid": 1, "host_monotonic_arrival_time_ns": 101},
    "relative_attitude": {"convention": "base_to_platform", "valid": True},
    "camera": {"enabled": True, "snapshot_timestamp_monotonic_ns": 102, "frame_id": "mock-1"},
}


def _session(test_name: str = "z_pos"):
    controller = make_mock_controller(start_steps=PARK_RAW_STEPS)
    session = CartesianCommissioningSession(
        controller,
        test_name=test_name,
        sleep_fn=lambda _seconds: None,
        control_rate_hz=20.0,
    )
    return session, controller


def _through_hold(test_name: str = "z_pos"):
    session, controller = _session(test_name)
    session.record_baseline(evidence=MOCK_EVIDENCE)
    session.home_transition(home_authorized=True, duration_s=0.01)
    session.confirm_home(observations=SAFE_OBSERVATIONS, evidence=MOCK_EVIDENCE)
    preview = session.preview_cartesian()
    session.forward_motion(forward_authorized=True, duration_s=0.01)
    session.observation_hold(
        observations=SAFE_OBSERVATIONS,
        evidence=MOCK_EVIDENCE,
        hold_samples=2,
        hold_interval_s=0.0,
    )
    return session, controller, preview


def test_allowed_sequence_excludes_yaw():
    assert ALLOWED_TESTS == (
        "z_pos", "z_neg", "x_pos", "x_neg", "y_pos", "y_neg",
        "roll_pos", "roll_neg", "pitch_pos", "pitch_neg",
    )
    with pytest.raises(TargetValidationError):
        CartesianCommissioningSession(make_mock_controller(), test_name="yaw_pos")


def test_full_mock_flow_requires_explicit_return_and_records_evidence():
    session, controller, preview = _through_hold("z_pos")

    assert session.state == CommissioningState.OBSERVATION_HOLD
    assert preview["pass"] is True
    assert len(preview["ik_delta_steps"]) == 6
    assert preview["fk"]["available"] is True
    assert session.report["automatic_return"] is False

    report_before_return = session.finalise()
    assert report_before_return["pass"] is False
    assert report_before_return["outcome"] == "HOLD"
    assert report_before_return["stages"]["return_home_authorized"] is None

    session.authorize_return_home(
        return_authorized=True,
        duration_s=0.01,
        evidence=MOCK_EVIDENCE,
    )
    report = session.finalise()
    assert report["pass"] is True
    assert report["outcome"] == "PASS"
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] > 0
    assert report["unexpected_writes"] == []
    assert report["pass_conditions"]["dual_h30_data_recorded"] is True
    assert report["pass_conditions"]["no_automatic_return"] is True
    assert controller.bus.get_telemetry()["retry_count"] == 0


def test_optional_park_is_not_executed_without_explicit_authorization():
    session, controller, _ = _through_hold("x_pos")
    session.authorize_return_home(return_authorized=True, duration_s=0.01, evidence=MOCK_EVIDENCE)
    report = session.finalise()
    assert report["park_executed"] is False
    assert report["stages"]["park_authorized"] is None

    session.authorize_park(park_authorized=True, duration_s=0.01)
    report = session.finalise()
    assert report["pass"] is True
    assert report["park_executed"] is True
    assert controller.bus.positions == {mid: 0 for mid in range(1, 7)}


def test_preview_rejects_commissioning_limit_before_motion_write():
    controller = make_mock_controller(start_steps=PARK_RAW_STEPS)
    limits = {mid: (1000, 1000) for mid in range(1, 7)}
    session = CartesianCommissioningSession(
        controller,
        test_name="z_pos",
        commissioning_limits=limits,
        sleep_fn=lambda _seconds: None,
    )
    session.record_baseline()
    session.home_transition(home_authorized=True, duration_s=0.01)
    session.confirm_home(observations=SAFE_OBSERVATIONS)
    with pytest.raises(SafetyViolation):
        session.preview_cartesian()
    assert session.report["stages"]["forward_perturbation"] is None
    assert session.report["stages"]["cartesian_preview"]["pass"] is False


def test_forward_stops_on_abnormal_current_and_records_pause():
    session, controller = _session("y_pos")
    session.record_baseline()
    session.home_transition(home_authorized=True, duration_s=0.01)
    session.confirm_home(observations=SAFE_OBSERVATIONS)
    session.preview_cartesian()
    controller.bus.currents[1] = 501
    with pytest.raises(SafetyViolation):
        session.forward_motion(forward_authorized=True, duration_s=0.01)
    report = session.finalise()
    assert report["outcome"] == "STOP"
    assert report["pass"] is False
    assert any("501mA" in reason for reason in report["failure_reasons"])
    assert report["unexpected_writes"] == []
    assert any(record["register_address"] == 0x1A for record in report["write_audit"])


def test_active_and_read_only_modes_require_safe_authorization():
    read_only = SixMotorController(hardware_mode=HardwareMode.READ_ONLY, retries=0)
    with pytest.raises(TargetValidationError):
        CartesianCommissioningSession(read_only, test_name="z_pos")

    active = SixMotorController(hardware_mode=HardwareMode.ACTIVE, retries=0)
    with pytest.raises(TargetValidationError):
        CartesianCommissioningSession(active, test_name="z_pos")


def test_convenience_runner_stops_at_hold_without_return_authorization():
    report = run_j6_phase3_cartesian_commissioning(
        test_name="pitch_pos",
        home_authorized=True,
        home_confirmed=True,
        forward_authorized=True,
        observations=SAFE_OBSERVATIONS,
        evidence=MOCK_EVIDENCE,
        duration_s=0.01,
        hold_samples=1,
        sleep_fn=lambda _seconds: None,
    )
    assert report["outcome"] == "HOLD"
    assert report["pass"] is False
    assert report["stages"]["return_home_authorized"] is None
    assert report["hardware_write_count"] == 0

