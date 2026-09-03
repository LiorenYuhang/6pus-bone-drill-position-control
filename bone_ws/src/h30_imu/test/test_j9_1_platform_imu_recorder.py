import importlib.util
import math
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_platform_imu_recorder.py"
spec = importlib.util.spec_from_file_location("j9_1_platform_imu_recorder", SCRIPT)
recorder = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = recorder
spec.loader.exec_module(recorder)


def test_relative_quaternion_identity():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q = (0.0, 0.0, 0.0, 1.0)

    q_rel = recorder.relative_quat_xyzw(q0, q)
    assert q_rel == (0.0, 0.0, 0.0, 1.0)
    assert recorder.quat_to_rpy_deg_xyzw(q_rel) == (0.0, 0.0, 0.0)


def test_relative_quaternion_conjugate_left_multiply_yaw():
    half = math.radians(5.0) / 2.0
    q0 = (0.0, 0.0, 0.0, 1.0)
    q_yaw_5 = (0.0, 0.0, math.sin(half), math.cos(half))

    q_rel = recorder.relative_quat_xyzw(q0, q_yaw_5)
    roll, pitch, yaw = recorder.quat_to_rpy_deg_xyzw(q_rel)

    assert math.isclose(roll, 0.0, abs_tol=1e-9)
    assert math.isclose(pitch, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 5.0, abs_tol=1e-9)


def test_average_quaternion_handles_sign_equivalence():
    q_avg = recorder.average_quat_xyzw(
        [
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, -1.0),
        ]
    )
    assert q_avg == (0.0, 0.0, 0.0, 1.0)
