"""End-to-End J3 Integration tests: 6PUS IK pipeline to SixMotorController."""

from __future__ import annotations

import sys
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parent.parent.parent
if str(SRC_ROOT / "pus6_kinematics") not in sys.path:
    sys.path.insert(0, str(SRC_ROOT / "pus6_kinematics"))
if str(SRC_ROOT / "pus6_motor_driver") not in sys.path:
    sys.path.insert(0, str(SRC_ROOT / "pus6_motor_driver"))

import pytest
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    HardwareMode,
    NUM_MOTORS,
    TargetValidationError,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController


def test_j3_ik_pure_heave_pipeline_to_mock_controller():
    """Verify desired Cartesian +0.25mm heave -> +10 step differential -> mock dispatch."""
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    des_pose = [0.0, 0.0, ik.nominal_height + 0.00025, 0.0, 0.0, 0.0]  # +0.25mm = +10 step

    delta_steps = ik.differential_inverse_kinematics_steps(des_pose, ref_pose)
    assert delta_steps == [10, 10, 10, 10, 10, 10]

    # Mock controller with realistic baseline reference
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    controller.connect()

    snapshot = controller.read_positions_snapshot()
    live_ref = snapshot.positions_steps  # [50, 50, 50, 50, 50, 50] in FakeBus default

    target_raw_steps = [r + d for r, d in zip(live_ref, delta_steps)]
    assert target_raw_steps == [60, 60, 60, 60, 60, 60]

    # Validate target vector against bounds
    validated_targets = validate_target_vector(target_raw_steps)
    assert len(validated_targets) == NUM_MOTORS

    # Configure and command
    controller.configure_all()
    controller.command_positions(validated_targets)
    final_snapshot = controller.read_positions_snapshot()

    assert final_snapshot.positions_steps == [60, 60, 60, 60, 60, 60]
    controller.pause_all()


def test_j3_ik_pitch_differential_pipeline_to_mock_controller():
    """Verify small pitch rotation maps to asymmetric differential steps and dispatches safely."""
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    des_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.003, 0.0]  # +0.003 rad pitch (~0.172 deg)

    delta_steps = ik.differential_inverse_kinematics_steps(des_pose, ref_pose)
    assert delta_steps == [2, 3, 2, -2, -3, -2]

    # Baseline: [74, 86, 44, 88, 36, 25] (similar to hardware J2-C/D baseline)
    custom_baseline = {1: 74, 2: 86, 3: 44, 4: 88, 5: 36, 6: 25}
    bus = FakeBus()
    bus.positions = dict(custom_baseline)
    bus.targets = dict(custom_baseline)
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    controller.connect()

    live_ref = controller.read_positions_snapshot().positions_steps
    target_raw_steps = [r + d for r, d in zip(live_ref, delta_steps)]
    assert target_raw_steps == [76, 89, 46, 86, 33, 23]

    validated_targets = validate_target_vector(target_raw_steps)
    controller.configure_all()
    controller.command_positions(validated_targets)

    final_snapshot = controller.read_positions_snapshot()
    assert final_snapshot.positions_steps == [76, 89, 46, 86, 33, 23]
    controller.pause_all()



def test_j3_ik_target_out_of_stroke_bounds_is_rejected():
    """Verify software stroke limit rejects out-of-bound targets."""
    # Near upper limit 1795
    live_ref = [1795, 1795, 1795, 1795, 1795, 1795]
    delta_steps = [10, 10, 10, 10, 10, 10]
    target_raw_steps = [r + d for r, d in zip(live_ref, delta_steps)]  # 1805 > 1800 limit

    with pytest.raises(TargetValidationError, match="outside safe boundary"):
        validate_target_vector(target_raw_steps)

