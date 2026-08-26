from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_motor_driver", ROOT / "src" / "pus6_kinematics"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase5_small_cartesian import (  # noqa: E402
    HOME_RAW_STEPS,
    SMALL_TEST_OFFSETS,
    _write_report,
    run_phase5_small_cartesian,
)
from run_j3_cartesian_motion import run_cartesian_test_session  # noqa: E402


def _source(*, final_error=0, safety_aborted=False, cycles=1, writes=24):
    return {
        "cycles": cycles,
        "target_pose_cartesian": [0.0, 0.0, 0.132, 0.0, 0.0, 0.0],
        "ik_differential_steps": [1, 2, 3, 4, 5, 6],
        "ik_target_steps": [1001, 1002, 1003, 1004, 1005, 1006],
        "q_start_raw": [1000] * 6,
        "q_final_raw": [1000 + final_error] * 6,
        "final_return_errors_steps": [final_error] * 6,
        "max_return_error_steps": final_error,
        "safety_aborted": safety_aborted,
        "write_audit": {"total_writes": writes},
    }


def _delegate(**kwargs):
    assert kwargs["cycles"] == 1
    assert kwargs["peak_speed"] == 50.0
    assert kwargs["real_hardware"] is False
    assert kwargs["custom_pose"][2] > 0.13
    return _source()


def _gates(**kwargs):
    values = dict(
        test_name="z_pos",
        phase4_passed=True,
        home_baseline_confirmed=True,
        home_baseline_steps=HOME_RAW_STEPS,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=_delegate,
    )
    values.update(kwargs)
    return values


def test_phase5_allowed_z_test_passes_with_mock_delegate():
    report = run_phase5_small_cartesian(**_gates())
    assert report["pass"] is True
    assert report["phase"] == "J6_PHASE5"
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] == 24
    assert report["pass_conditions"]["returned_to_home"] is True


@pytest.mark.parametrize("test_name", sorted(SMALL_TEST_OFFSETS))
def test_all_allowed_tests_are_accepted(test_name):
    report = run_phase5_small_cartesian(**_gates(test_name=test_name))
    assert report["pass"] is True


def test_phase5_rejects_large_or_yaw_test_before_delegate():
    called = False

    def delegate(**_kwargs):
        nonlocal called
        called = True
        return _source()

    report = run_phase5_small_cartesian(
        test_name="yaw_pos",
        phase4_passed=True,
        home_baseline_confirmed=True,
        home_baseline_steps=HOME_RAW_STEPS,
        e_stop_ready=True,
        two_person_observation=True,
        delegate=delegate,
    )
    assert report["pass"] is False
    assert report["outcome"] == "STOP"
    assert called is False


def test_phase5_requires_all_safety_gates_without_delegate():
    called = False

    def delegate(**_kwargs):
        nonlocal called
        called = True
        return _source()

    report = run_phase5_small_cartesian(
        test_name="x_pos",
        phase4_passed=False,
        home_baseline_confirmed=False,
        home_baseline_steps=[999] * 6,
        e_stop_ready=False,
        two_person_observation=False,
        delegate=delegate,
    )
    assert report["pass"] is False
    assert report["outcome"] == "STOP"
    assert called is False


def test_phase5_rejects_non_home_custom_start():
    report = run_phase5_small_cartesian(**_gates(custom_start=[999] * 6))
    assert report["pass"] is False
    assert any("custom_start must equal HOME baseline" in item for item in report["failure_reasons"])


def test_phase5_speed_is_hard_limited():
    report = run_phase5_small_cartesian(**_gates(peak_speed=51.0))
    assert report["pass"] is False
    assert "exactly 50 step/s" in " ".join(report["failure_reasons"])


def test_phase5_rejects_non_home_baseline():
    report = run_phase5_small_cartesian(**_gates(home_baseline_steps=[999] * 6))
    assert report["pass"] is False
    assert any("HOME baseline must equal" in item for item in report["failure_reasons"])


def test_phase5_fails_if_delegate_does_not_return_home():
    report = run_phase5_small_cartesian(**_gates(delegate=lambda **_: _source(final_error=3)))
    assert report["pass"] is False
    assert report["pass_conditions"]["returned_to_home"] is False


def test_phase5_safety_abort_is_not_pass():
    report = run_phase5_small_cartesian(**_gates(delegate=lambda **_: _source(safety_aborted=True)))
    assert report["pass"] is False
    assert report["outcome"] == "STOP"


def test_phase5_reuses_existing_j3_fakebus_runner(tmp_path, monkeypatch):
    import run_j3_cartesian_motion

    monkeypatch.setattr(run_j3_cartesian_motion, "WORKSPACE_ROOT", tmp_path)
    report = run_phase5_small_cartesian(
        test_name="z_pos",
        phase4_passed=True,
        home_baseline_confirmed=True,
        home_baseline_steps=HOME_RAW_STEPS,
        e_stop_ready=True,
        two_person_observation=True,
        custom_start=HOME_RAW_STEPS,
        hold_time_s=0.0,
        settle_time_s=0.0,
        override_duration=1.0,
        delegate=run_cartesian_test_session,
    )
    assert report["pass"] is True, report
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] > 0
    assert report["return_to_home_verified"] is True


def test_phase5_report_refuses_overwrite(tmp_path):
    path = tmp_path / "phase5.json"
    _write_report({"pass": True}, path)
    with pytest.raises(FileExistsError):
        _write_report({"pass": True}, path)
    assert json.loads(path.read_text(encoding="utf-8"))["pass"] is True
