import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import pytest
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import KinematicsError, WorkspaceLimitError


def test_analytical_ik_nominal_pose():
    ik = Analytical6PusKinematics()
    pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    sliders = ik.inverse_kinematics(pose)
    assert len(sliders) == 6
    assert all(isinstance(s, float) for s in sliders)
    # Under authoritative nominal height 132.612mm, mid-stroke slider height is exactly 50.0mm (0.050m)
    assert all(abs(s - 0.050) < 1e-5 for s in sliders)


def test_analytical_ik_invalid_length():
    ik = Analytical6PusKinematics()
    with pytest.raises(KinematicsError, match="must have length 6"):
        ik.inverse_kinematics([0.0, 0.0, 0.1326])


def test_analytical_ik_workspace_limit():
    ik = Analytical6PusKinematics()
    # z too low (< 0.05m)
    with pytest.raises(WorkspaceLimitError, match="outside reachable height range"):
        ik.inverse_kinematics([0.0, 0.0, 0.01, 0.0, 0.0, 0.0])

    # z too high (> 0.30m)
    with pytest.raises(WorkspaceLimitError, match="outside reachable height range"):
        ik.inverse_kinematics([0.0, 0.0, 0.8, 0.0, 0.0, 0.0])

    # excessive horizontal displacement exceeding strut length (84mm)
    with pytest.raises(WorkspaceLimitError, match="exceeds strut length"):
        ik.inverse_kinematics([0.15, 0.0, ik.nominal_height, 0.0, 0.0, 0.0])


def test_differential_ik_zero_delta_when_poses_equal():
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    deltas = ik.differential_inverse_kinematics(ref_pose, ref_pose)
    delta_steps = ik.differential_inverse_kinematics_steps(ref_pose, ref_pose)

    assert len(deltas) == 6
    assert all(abs(d) < 1e-9 for d in deltas)
    assert delta_steps == [0, 0, 0, 0, 0, 0]


def test_differential_ik_pure_heave():
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    # Pure +0.25mm (+0.00025m) vertical translation
    des_pose = [0.0, 0.0, ik.nominal_height + 0.00025, 0.0, 0.0, 0.0]

    deltas = ik.differential_inverse_kinematics(des_pose, ref_pose)
    delta_steps = ik.differential_inverse_kinematics_steps(des_pose, ref_pose)

    # Pure heave should displace all 6 vertical sliders equally by exactly +0.00025m (+10 steps)
    assert all(abs(d - 0.00025) < 1e-6 for d in deltas)
    assert delta_steps == [10, 10, 10, 10, 10, 10]


def test_differential_ik_pure_pitch_symmetry():
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    # Small pitch tilt +0.003 rad (~0.172 deg)
    des_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.003, 0.0]

    delta_steps = ik.differential_inverse_kinematics_steps(des_pose, ref_pose)
    assert len(delta_steps) == 6
    assert delta_steps == [2, 3, 2, -2, -3, -2]


def test_differential_ik_pure_roll_symmetry():
    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]
    # Small roll tilt +0.003 rad (~0.172 deg)
    des_pose = [0.0, 0.0, ik.nominal_height, 0.003, 0.0, 0.0]

    delta_steps = ik.differential_inverse_kinematics_steps(des_pose, ref_pose)
    assert len(delta_steps) == 6
    # Limb 2 and 5 lie on X axis (y=0), so their delta is exactly 0
    assert delta_steps == [3, 0, -3, -3, 0, 3]


def test_analytical_fk():
    ik = Analytical6PusKinematics()
    joints = [0.050, 0.050, 0.050, 0.050, 0.050, 0.050]
    pose = ik.forward_kinematics(joints)
    assert len(pose) == 6
    assert pose[0] == 0.0
    assert pose[1] == 0.0
    assert abs(pose[2] - ik.nominal_height) < 1e-4


