from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_motor_driver"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase4_home_park import _write_report, run_phase4_home_park  # noqa: E402
from run_j3_startup_homing import run_homing_session  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import (  # noqa: E402
    CONTROL_MODE_ADDR,
    MOTOR_IDS,
    TARGET_POSITION_ADDR,
    TargetValidationError,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


def _j3_report(*, safety_aborted=False, tracking_tolerable=True, writes=6):
    configuration = [{"motor_id": mid, "register_address": 0x25, "payload": []} for mid in range(1, 7)]
    audit = {
        "total_writes": writes,
        "classified_write_count": writes,
        "expected_configuration_writes": configuration,
        "expected_motion_writes": [],
        "expected_stop_writes": [],
        "unexpected_writes": [],
        "unexpected_writes_checked": True,
        "motion_sequence_matches": True,
        "configuration_sequence_matches": True,
    }
    return {
        "q_start_raw": [50] * 6,
        "q_target_raw": [1000] * 6,
        "q_final_raw": [1000] * 6,
        "final_errors_steps": [0] * 6,
        "max_error_steps": 0,
        "tracking_tolerable": tracking_tolerable,
        "safety_aborted": safety_aborted,
        "stop_result": {"verified_stopped": True},
        "write_audit": audit,
    }

def _delegate(**kwargs):
    assert kwargs["speed"] == 50
    assert kwargs["real_hardware"] is False
    return _j3_report()


def test_home_signed_feedback_is_decoded_and_negative_target_is_not_written():
    report = run_homing_session(
        action="home",
        real_hardware=False,
        custom_start=[0, 0, -2, -1, -2, -2],
        duration=0.1,
        settle_time=0.0,
        sleep_fn=lambda _seconds: None,
    )
    assert report["q_start_raw"] == [0, 0, -2, -1, -2, -2]
    assert report["safety_aborted"] is False
    assert report["trajectory_start_raw"] == [0, 0, 0, 0, 0, 0]
    assert report["park_baseline_tolerance_steps"] == 5
    assert report["park_baseline_within_tolerance"] is True
    assert report["q_final_raw"] == [1000] * 6
    assert report["write_audit"]["motion_write_count"] > 0
    assert all(int(value) >= 0 for record in report["write_audit"]["expected_motion_writes"] for value in record["payload"])


def test_park_accepts_signed_near_zero_feedback_without_negative_targets():
    report = run_homing_session(
        action="park",
        real_hardware=False,
        custom_start=[0, 0, -2, -1, -2, -2],
        duration=0.1,
        settle_time=0.0,
        sleep_fn=lambda _seconds: None,
    )
    assert report["safety_aborted"] is False
    assert report["trajectory_start_raw"] == [0, 0, 0, 0, 0, 0]
    assert report["park_baseline_deviation_steps"] == [0, 0, -2, -1, -2, -2]
    assert report["q_final_raw"] == [0] * 6
    assert all(int(value) >= 0 for record in report["write_audit"]["expected_motion_writes"] for value in record["payload"])


@pytest.mark.parametrize(
    ("action", "target"),
    (("park", [0] * 6), ("home", [1000] * 6)),
)
def test_home_park_config_then_command_has_no_hidden_target_write(action, target):
    """Configuration must not write 0x29; only command_positions may do so."""
    bus = FakeBus()
    bus.positions = {mid: value for mid, value in zip(MOTOR_IDS, [0, 0, -2, -1, -2, -2])}
    bus.targets = dict(bus.positions)
    controller = SixMotorController(bus=bus)
    controller.connect()

    controller.configure_all(mode=2, speed=50)
    configuration_writes = list(bus.write_audit)
    assert len(configuration_writes) == 6
    assert all(record["register_address"] == CONTROL_MODE_ADDR for record in configuration_writes)
    assert all(record["payload"] == [2, 0, 0, 50] for record in configuration_writes)
    assert not any(record["register_address"] == TARGET_POSITION_ADDR for record in configuration_writes)

    controller.command_positions(target)
    motion_writes = bus.write_audit[len(configuration_writes):]
    assert len(motion_writes) == 6
    assert [record["register_address"] for record in motion_writes] == [TARGET_POSITION_ADDR] * 6
    assert [record["payload"] for record in motion_writes] == [[value] for value in target]


def test_true_negative_command_target_remains_rejected():
    with pytest.raises(TargetValidationError):
        validate_target_vector([-1, 0, 0, 0, 0, 0])


def test_phase4_home_mock_delegate_passes_and_reports_mock_writes():
    report = run_phase4_home_park(
        action="home",
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        custom_start=[50] * 6,
        delegate=_delegate,
    )
    assert report["pass"] is True
    assert report["outcome"] == "PASS"
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] == 6


def test_phase4_reuses_existing_j3_homing_with_fakebus(tmp_path, monkeypatch):
    import run_j3_startup_homing

    monkeypatch.setattr(run_j3_startup_homing, "WORKSPACE_ROOT", tmp_path)
    report = run_phase4_home_park(
        action="home",
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        custom_start=[50] * 6,
        duration=1.0,
        settle_time=0.0,
        delegate=run_homing_session,
    )
    assert report["pass"] is True, report
    assert report["q_target_raw"] == [1000] * 6
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] > 0


def test_phase4_park_requires_maintenance_authorization_before_delegate():
    called = False

    def delegate(**_kwargs):
        nonlocal called
        called = True
        return _j3_report()

    report = run_phase4_home_park(
        action="park",
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=delegate,
    )
    assert report["pass"] is False
    assert report["outcome"] == "STOP"
    assert called is False
    assert any("maintenance authorization" in item for item in report["failure_reasons"])


def test_phase4_rejects_missing_prior_gates_without_delegate():
    called = False

    def delegate(**_kwargs):
        nonlocal called
        called = True
        return _j3_report()

    report = run_phase4_home_park(
        phase2_passed=False,
        phase3_passed=False,
        e_stop_ready=False,
        two_person_observation=False,
        delegate=delegate,
    )
    assert report["outcome"] == "STOP"
    assert report["hardware_write_count"] == 0
    assert called is False


def test_phase4_speed_is_hard_limited():
    report = run_phase4_home_park(
        speed=51,
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=_delegate,
    )
    assert report["pass"] is False
    assert report["outcome"] == "STOP"
    assert "exactly 50 step/s" in " ".join(report["failure_reasons"])


def test_phase4_rejects_delegate_target_mismatch():
    def wrong_target(**_kwargs):
        result = _j3_report()
        result["q_target_raw"] = [0] * 6
        return result

    report = run_phase4_home_park(
        action="home",
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=wrong_target,
    )
    assert report["pass"] is False
    assert report["pass_conditions"]["target_matches_action"] is False


def test_phase4_delegate_safety_abort_is_not_pass():
    report = run_phase4_home_park(
        phase2_passed=True,
        phase3_passed=True,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=lambda **_kwargs: _j3_report(safety_aborted=True),
    )
    assert report["pass"] is False
    assert report["outcome"] == "STOP"


def test_phase4_report_refuses_overwrite(tmp_path):
    path = tmp_path / "phase4.json"
    _write_report({"pass": True}, path)
    with pytest.raises(FileExistsError):
        _write_report({"pass": True}, path)
    assert json.loads(path.read_text(encoding="utf-8"))["pass"] is True
