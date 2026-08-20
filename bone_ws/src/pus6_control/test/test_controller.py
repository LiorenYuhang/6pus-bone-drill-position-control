import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import pytest
from pus6_control.controller_interface import ControlError
from pus6_control.joint_trajectory_controller import (
    JointTrajectoryController,
    StatefulJerkLimitedShaper,
)


def test_joint_trajectory_controller_shaping():
    ctrl = JointTrajectoryController(
        max_speed_m_per_sec=0.005,
        max_accel_m_per_sec2=0.050,
        max_jerk_m_per_sec3=0.500,
        omega_n=20.0,
    )
    current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target = [0.005, 0.005, 0.005, 0.005, 0.005, 0.005]  # +5 mm target

    # At dt = 0.02s (50 Hz), state smoothly updates towards target
    out1 = ctrl.update(current, target, dt=0.02)
    assert len(out1) == 6
    assert all(0.0 < p <= 0.005 for p in out1)

    # Next step dt = 0.02s -> advances further
    out2 = ctrl.update(out1, target, dt=0.02)
    assert all(out2[i] > out1[i] for i in range(6))


def test_joint_trajectory_controller_invalid_dimensions():
    ctrl = JointTrajectoryController()
    with pytest.raises(ControlError, match="must have length 6"):
        ctrl.update([1.0, 2.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dt=0.1)


def test_joint_trajectory_controller_reset():
    ctrl = JointTrajectoryController()
    ctrl.update([0.0] * 6, [0.005] * 6, dt=0.1)
    assert ctrl._initialized is True
    ctrl.reset([0.0] * 6)
    assert ctrl.shaper.q == [0.0] * 6
    assert ctrl.shaper.dq == [0.0] * 6
    assert ctrl.shaper.ddq == [0.0] * 6
