"""Test suite for J3 Pre-Assembly Cartesian IK Motion Matrix."""

from __future__ import annotations

import sys
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
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from run_j3_cartesian_motion import get_predefined_pose, run_cartesian_test_session


@pytest.mark.parametrize(
    "test_name, expected_delta",
    [
        ("heave_pos", [400, 400, 400, 400, 400, 400]),
        ("heave_neg", [-400, -400, -400, -400, -400, -400]),
        ("surge_pos", [-2, 33, 41, -29, -21, 14]),
        ("surge_neg", [14, -21, -29, 41, 33, -2]),
        ("sway_pos", [-30, 31, 17, 17, 31, -30]),
        ("sway_neg", [42, -19, -5, -5, -19, 42]),
        ("pitch_pos", [27, 55, 28, -27, -55, -28]),
        ("pitch_neg", [-28, -55, -27, 28, 55, 27]),
        ("roll_pos", [48, 0, -48, -48, 0, 48]),
        ("roll_neg", [-47, 0, 48, 48, 0, -47]),
        ("yaw_pos", [8, -6, 8, -6, 8, -6]),
        ("yaw_neg", [-6, 8, -6, 8, -6, 8]),
        ("combined_6dof", [236, 171, 178, 162, 208, 272]),
    ],
)
def test_cartesian_ik_matrix_generation(test_name: str, expected_delta: list[int]):
    """Verify that all predefined Cartesian test cases produce exact expected IK steps."""
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    pose, _ = get_predefined_pose(ik, test_name)
    delta_steps = ik.differential_inverse_kinematics_steps(pose, ref_pose)
    assert delta_steps == expected_delta


def test_cartesian_mock_session_execution():
    """Verify full end-to-end execution of a Cartesian session in mock mode."""
    report = run_cartesian_test_session(
        test_name="pitch_pos",
        peak_speed=200.0,
        control_rate_hz=20.0,
        hold_time_s=0.01,
        settle_time_s=0.01,
        cycles=1,
        real_hardware=False,
        override_duration=0.1,
    )

    assert not report["safety_aborted"]
    assert report["test_name"] == "pitch_pos"
    assert report["ik_differential_steps"] == [27, 55, 28, -27, -55, -28]
    assert report["ik_target_steps"] == [1027, 1055, 1028, 973, 945, 972]
    assert report["q_final_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
    assert report["q_final_relative"] == [0, 0, 0, 0, 0, 0]


def test_cartesian_cyclic_mock_session():
    """Verify multi-cycle execution in mock mode."""
    report = run_cartesian_test_session(
        test_name="combined_6dof",
        peak_speed=200.0,
        control_rate_hz=20.0,
        hold_time_s=0.01,
        settle_time_s=0.01,
        cycles=3,
        real_hardware=False,
        override_duration=0.1,
    )

    assert not report["safety_aborted"]
    assert report["cycles"] == 3
    assert len(report["cycle_reports"]) == 3
    assert report["q_final_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
