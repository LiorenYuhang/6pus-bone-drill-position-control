import importlib.util
import math
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_so3_error_offline_analysis.py"
spec = importlib.util.spec_from_file_location("j9_1_so3_error_offline_analysis", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_zero_error():
    """1. R_meas = R_ref -> e_W = 0, e_ref = 0."""
    ref_rpy = (0.5, 0.4, -0.8)
    meas_rpy = (0.5, 0.4, -0.8)
    eval_so3, e_w = mod.evaluate_so3_candidate_controller(ref_rpy, meas_rpy, active_axes={"roll", "pitch", "yaw"})
    assert np.allclose(e_w, [0.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(eval_so3.e_vec_deg, [0.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(eval_so3.residual_deg, [0.0, 0.0, 0.0], atol=1e-12)


def test_pure_single_axes_small_angle():
    """2, 3, 4, 5. Pure Roll, Pitch, Yaw small angles in positive and negative directions."""
    for axis_idx, axis_name, angle in [
        (0, "roll", 0.5), (0, "roll", -0.5),
        (1, "pitch", 0.5), (1, "pitch", -0.5),
        (2, "yaw", 2.0), (2, "yaw", -2.0),
    ]:
        ref = [0.0, 0.0, 0.0]
        meas = [0.0, 0.0, 0.0]
        ref[axis_idx] = angle
        meas[axis_idx] = 0.8 * angle

        eval_so3, e_w = mod.evaluate_so3_candidate_controller(ref, meas, active_axes={axis_name})
        eval_euler = mod.evaluate_euler_channel_controller(ref, meas, active_axes={axis_name})

        # Check that error is concentrated on the target axis
        assert math.isclose(eval_so3.e_vec_deg[axis_idx], 0.2 * angle, abs_tol=1e-5)
        # Check sign consistency
        assert np.sign(eval_so3.e_vec_deg[axis_idx]) == np.sign(angle)
        assert np.sign(eval_so3.residual_deg[axis_idx]) == np.sign(angle)


def test_exp_log_roundtrip():
    """6. Exp(Log(R)) round-trip test on random rotations."""
    np.random.seed(42)
    for _ in range(20):
        rot = R.random()
        rotvec = rot.as_rotvec()
        rot_recovered = R.from_rotvec(rotvec)
        diff = rot * rot_recovered.inv()
        dist = np.linalg.norm(diff.as_rotvec())
        assert dist < 1e-14


def test_quaternion_normalized_finite():
    """7. Quaternion normalized and finite for various configurations."""
    for r in [-2.0, 0.0, 1.5]:
        for p in [-1.0, 0.0, 1.0]:
            for y in [-2.0, 0.0, 2.0]:
                eval_so3, _ = mod.evaluate_so3_candidate_controller(
                    (r, p, y), (0.0, 0.0, 0.0), active_axes={"roll", "pitch", "yaw"}
                )
                assert all(math.isfinite(x) for x in eval_so3.q_cmd_xyzw)
                norm = np.linalg.norm(eval_so3.q_cmd_xyzw)
                assert math.isclose(norm, 1.0, abs_tol=1e-12)


def test_world_left_and_body_right_composition_equivalence():
    """8. World-left composition Exp(Kp*e_W) * R_ref vs Body-right R_ref * Exp(Kp*e_ref) strict numerical equivalence."""
    np.random.seed(123)
    kp = 0.5
    for _ in range(50):
        rot_ref = R.random()
        rot_meas = R.random()

        E_W = rot_ref * rot_meas.inv()
        e_W = E_W.as_rotvec()
        e_ref = rot_ref.as_matrix().T @ e_W

        R_cmd_left = R.from_rotvec(kp * e_W) * rot_ref
        R_cmd_right = rot_ref * R.from_rotvec(kp * e_ref)

        dist_rad = mod.so3_angular_distance_rad(R_cmd_left.as_matrix(), R_cmd_right.as_matrix())
        assert dist_rad < 1e-10, f"World-left vs Body-right composition failed equivalence: dist={dist_rad}"


def test_single_axis_so3_vs_euler_channel_equivalence():
    """9. Single-axis: SO(3) candidate vs Euler-channel output equivalence under small angles."""
    for axis, val in [("roll", 0.5), ("roll", -0.5), ("pitch", 0.5), ("pitch", -0.5), ("yaw", 2.0), ("yaw", -2.0)]:
        ref = [0.0, 0.0, 0.0]
        meas = [0.0, 0.0, 0.0]
        idx = {"roll": 0, "pitch": 1, "yaw": 2}[axis]
        ref[idx] = val
        meas[idx] = 0.85 * val

        eval_euler = mod.evaluate_euler_channel_controller(ref, meas, active_axes={axis})
        eval_so3, _ = mod.evaluate_so3_candidate_controller(ref, meas, active_axes={axis})

        # Angular distance between commands must be negligible for single axis
        dist_rad = mod.so3_angular_distance_rad(eval_euler.r_cmd_matrix, eval_so3.r_cmd_matrix)
        assert dist_rad < 1e-6, f"Single-axis {axis} SO(3) vs Euler divergence: dist={dist_rad} rad"
        # Residual must be nearly identical
        assert math.isclose(eval_euler.residual_deg[idx], eval_so3.residual_deg[idx], abs_tol=1e-5)
