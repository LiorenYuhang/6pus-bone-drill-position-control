from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_motor_driver"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase3_first_motion_probe import (  # noqa: E402
    ALLOWED_DISPLACEMENTS,
    DEFAULT_DISPLACEMENT_STEPS,
    run_first_motion_probe,
)
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import HardwareMode, TargetValidationError  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


SAFE_OBSERVATIONS = {
    "platform_motion": "expected",
    "ball_joint_condition": "clear",
    "interference": "none",
    "abnormal_sound": "none",
}


def _controller(*, axis: int = 1, baseline: int = 100):
    bus = FakeBus()
    bus.positions[axis] = baseline
    bus.targets[axis] = baseline
    return bus, SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK, retries=0)


def test_probe_default_is_two_steps_and_does_not_return_automatically():
    bus, controller = _controller(axis=2, baseline=100)

    report = run_first_motion_probe(
        controller,
        axis=2,
        hold_samples=2,
        hold_interval_s=0,
        observations=SAFE_OBSERVATIONS,
    )

    assert DEFAULT_DISPLACEMENT_STEPS == 2
    assert report["displacement_steps"] == DEFAULT_DISPLACEMENT_STEPS
    assert report["outcome"] == "HOLD"
    assert report["pass"] is False
    assert report["stages"]["return_authorization"]["authorized"] is False
    assert report["stages"]["return_authorization"]["executed"] is False
    assert report["metrics"]["target_write_count"] == 1
    assert report["mock_write_count"] == 1
    assert report["hardware_write_count"] == 0
    assert report["unexpected_writes"] == []
    assert len(bus.write_audit) == 1
    assert bus.write_audit[0]["motor_id"] == 2
    assert bus.write_audit[0]["register_address"] == 0x29
    assert bus.positions[2] == 102


def test_probe_return_requires_explicit_authorization_and_safe_observations():
    bus, controller = _controller(axis=1, baseline=100)

    report = run_first_motion_probe(
        controller,
        axis=1,
        displacement_steps=5,
        hold_samples=1,
        hold_interval_s=0,
        observations=SAFE_OBSERVATIONS,
        return_authorized=True,
    )

    assert report["outcome"] == "PASS"
    assert report["pass"] is True
    assert report["stages"]["return_authorization"]["authorized"] is True
    assert report["stages"]["return_authorization"]["executed"] is True
    assert report["stages"]["return_authorization"]["final_error_steps"] == 0
    assert report["metrics"]["target_write_count"] == 2
    assert report["mock_write_count"] == 2
    assert [record["motor_id"] for record in bus.write_audit] == [1, 1]
    assert [record["register_address"] for record in bus.write_audit] == [0x29, 0x29]
    assert bus.positions[1] == 100


def test_probe_displacement_is_limited_to_two_five_or_ten_steps():
    assert ALLOWED_DISPLACEMENTS == (2, 5, 10)
    for displacement in (1, 3, 4, 11, 20, -1, -3):
        controller = SixMotorController(bus=FakeBus(), hardware_mode=HardwareMode.MOCK, retries=0)
        with pytest.raises(TargetValidationError):
            run_first_motion_probe(controller, axis=1, displacement_steps=displacement)
        assert controller.bus.events == []


def test_probe_abnormal_interference_blocks_authorized_return_without_retry():
    bus, controller = _controller(axis=3, baseline=100)
    observations = dict(SAFE_OBSERVATIONS, interference="present")

    report = run_first_motion_probe(
        controller,
        axis=3,
        displacement_steps=2,
        hold_samples=1,
        hold_interval_s=0,
        observations=observations,
        return_authorized=True,
    )

    assert report["outcome"] == "STOP"
    assert report["pass"] is False
    assert report["stages"]["return_authorization"]["authorized"] is False
    assert report["stages"]["return_authorization"]["executed"] is False
    assert report["metrics"]["target_write_count"] == 1
    assert report["metrics"]["unexpected_write_count"] == 0
    assert len(bus.write_audit) == 1
    assert bus.write_audit[0]["motor_id"] == 3


def test_probe_logs_full_six_axis_feedback_and_operator_fields():
    _, controller = _controller(axis=4, baseline=100)

    report = run_first_motion_probe(
        controller,
        axis=4,
        displacement_steps=10,
        hold_samples=1,
        hold_interval_s=0,
        observations=SAFE_OBSERVATIONS,
    )

    assert report["stages"]["baseline_recording"]["feedback"]["target_positions_steps"]
    for stage_name in ("baseline_recording", "forward_perturbation"):
        feedback = report["stages"][stage_name]["feedback"]
        assert len(feedback["target_positions_steps"]) == 6
        assert len(feedback["actual_positions_steps"]) == 6
        assert len(feedback["currents_mA"]) == 6
        assert len(feedback["fault_codes"]) == 6
        assert len(feedback["temperatures_C"]) == 6
        assert len(feedback["per_axis_timestamps_ns"]) == 6
    hold = report["stages"]["observation_hold"]
    assert len(hold["samples"]) == 1
    assert len(hold["samples"][0]["actual_positions_steps"]) == 6
    assert report["operator_observations"] == SAFE_OBSERVATIONS


def test_probe_rejects_read_only_and_nonzero_retry_controller_before_bus_access():
    read_only_bus = FakeBus()
    read_only_controller = SixMotorController(
        bus=read_only_bus,
        hardware_mode=HardwareMode.READ_ONLY,
        retries=0,
    )
    with pytest.raises(TargetValidationError, match="requires MOCK"):
        run_first_motion_probe(read_only_controller, axis=1)
    assert read_only_bus.events == []

    retry_bus = FakeBus()
    retry_controller = SixMotorController(
        bus=retry_bus,
        hardware_mode=HardwareMode.MOCK,
        retries=1,
    )
    with pytest.raises(TargetValidationError, match="retries=0"):
        run_first_motion_probe(retry_controller, axis=1)
    assert retry_bus.events == []


def test_probe_rejects_target_out_of_bounds_before_motion_write():
    bus, controller = _controller(axis=1, baseline=0)

    report = run_first_motion_probe(
        controller,
        axis=1,
        displacement_steps=-2,
        hold_samples=1,
        hold_interval_s=0,
    )

    assert report["outcome"] == "FAIL"
    assert report["pass"] is False
    assert report["metrics"]["target_write_count"] == 0
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] == 0
    assert bus.write_audit == []
