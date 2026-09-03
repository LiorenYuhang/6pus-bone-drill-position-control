import importlib.util
import math
import sys
from pathlib import Path
import numpy as np

# Load script module directly
SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_3dof_orientation_servo.py"
spec = importlib.util.spec_from_file_location("j9_1_3dof_orientation_servo", SCRIPT)
exp_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = exp_module
spec.loader.exec_module(exp_module)


def test_zero_error():
    """1. Test zero error produces zero residuals and identity residual quaternion."""
    harness = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    import threading
    harness.lock = threading.Lock()
    harness.active_axes = {"roll", "pitch", "yaw"}
    harness.kp_roll = 0.5
    harness.kp_pitch = 0.5
    harness.kp_yaw = 0.5
    harness.residual_limit_roll = 0.25
    harness.residual_limit_pitch = 0.25
    harness.residual_limit_yaw = 0.50
    harness.ref_rpy = (0.5, 0.0, 0.0)
    harness.latest_plat_rpy = (0.5, 0.0, 0.0)

    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()
    assert math.isclose(errs[0], 0.0, abs_tol=1e-6)
    assert math.isclose(residuals[0], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[0], 0.5, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[1], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[2], 0.0, abs_tol=1e-6)


def test_positive_and_negative_roll_error_and_clamp():
    """2, 3, 4. Test positive Roll error, negative Roll error, and clamp."""
    harness = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    import threading
    harness.lock = threading.Lock()
    harness.active_axes = {"roll"}  # Roll only active
    harness.kp_roll = 0.5
    harness.kp_pitch = 0.5
    harness.kp_yaw = 0.5
    harness.residual_limit_roll = 0.25
    harness.residual_limit_pitch = 0.25
    harness.residual_limit_yaw = 0.50
    harness.ref_rpy = (0.5, 0.0, 0.0)

    # Positive roll error (meas = 0.4 deg -> err = 0.1 deg)
    harness.latest_plat_rpy = (0.4, 0.0, 0.0)
    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()
    assert math.isclose(errs[0], 0.1, abs_tol=1e-6)
    assert math.isclose(residuals[0], 0.05, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[0], 0.55, abs_tol=1e-6)

    # Negative roll error (meas = 0.6 deg -> err = -0.1 deg)
    harness.latest_plat_rpy = (0.6, 0.0, 0.0)
    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()
    assert math.isclose(errs[0], -0.1, abs_tol=1e-6)
    assert math.isclose(residuals[0], -0.05, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[0], 0.45, abs_tol=1e-6)

    # Large positive error -> clamp at +0.25 deg
    harness.latest_plat_rpy = (0.0, 0.0, 0.0)
    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()
    assert math.isclose(errs[0], 0.5, abs_tol=1e-6)
    assert math.isclose(residuals[0], 0.25, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[0], 0.75, abs_tol=1e-6)

    # Large negative error -> clamp at -0.25 deg
    harness.latest_plat_rpy = (1.2, 0.0, 0.0)
    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()
    assert math.isclose(errs[0], -0.7, abs_tol=1e-6)
    assert math.isclose(residuals[0], -0.25, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[0], 0.25, abs_tol=1e-6)


def test_pitch_yaw_disabled_and_zero_reference():
    """5. Test disabled Pitch/Yaw channels do not produce abnormal command even with nonzero measurement."""
    harness = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    import threading
    harness.lock = threading.Lock()
    harness.active_axes = {"roll"}  # Only Roll active; Pitch/Yaw disabled
    harness.kp_roll = 0.5
    harness.kp_pitch = 0.5
    harness.kp_yaw = 0.5
    harness.residual_limit_roll = 0.25
    harness.residual_limit_pitch = 0.25
    harness.residual_limit_yaw = 0.50
    harness.ref_rpy = (0.5, 0.0, 0.0)

    # Cross-axis coupling present on Pitch and Yaw measurements
    harness.latest_plat_rpy = (0.4, 0.08, -0.15)
    errs, residuals, cmd_rpy, cmd_q = harness.compute_control_step()

    # Errors should record the deviation
    assert math.isclose(errs[0], 0.1, abs_tol=1e-6)
    assert math.isclose(errs[1], -0.08, abs_tol=1e-6)
    assert math.isclose(errs[2], 0.15, abs_tol=1e-6)

    # But Pitch/Yaw residuals MUST be strictly zero because they are disabled
    assert math.isclose(residuals[0], 0.05, abs_tol=1e-6)
    assert math.isclose(residuals[1], 0.0, abs_tol=1e-6)
    assert math.isclose(residuals[2], 0.0, abs_tol=1e-6)

    # Command RPY must strictly keep Pitch=0 and Yaw=0
    assert math.isclose(cmd_rpy[0], 0.55, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[1], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy[2], 0.0, abs_tol=1e-6)


def test_so3_composition_finite_and_normalized():
    """6. Test SO(3) quaternion composition is always finite, normalized, and smooth."""
    for r_ref in [-2.0, 0.0, 1.5]:
        for p_ref in [-1.0, 0.0, 1.0]:
            for y_ref in [-2.0, 0.0, 2.0]:
                for d_r in [-0.25, 0.0, 0.25]:
                    for d_p in [-0.25, 0.0, 0.25]:
                        for d_y in [-0.50, 0.0, 0.50]:
                            q_cmd, cmd_rpy = exp_module.compose_so3_orientation(
                                (r_ref, p_ref, y_ref), (d_r, d_p, d_y)
                            )
                            # Check finite
                            assert all(math.isfinite(x) for x in q_cmd)
                            assert all(math.isfinite(x) for x in cmd_rpy)
                            # Check unit norm
                            norm = math.sqrt(sum(x * x for x in q_cmd))
                            assert math.isclose(norm, 1.0, abs_tol=1e-6)


def test_rz90_transform_preserved():
    """7. Test canonical R_PI = Rz(+90 deg) quaternion is unchanged."""
    harness = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    harness.q_PI = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    # Check that q_PI rotates X into Y
    # q_PI * (1, 0, 0, 0) * q_PI* = (0, 1, 0, 0)
    vx, vy, vz = 1.0, 0.0, 0.0
    qv = (vx, vy, vz, 0.0)
    q_rot = exp_module.quat_multiply_xyzw(
        exp_module.quat_multiply_xyzw(harness.q_PI, qv),
        exp_module.quat_conjugate_xyzw(harness.q_PI),
    )
    assert math.isclose(q_rot[0], 0.0, abs_tol=1e-6)
    assert math.isclose(q_rot[1], 1.0, abs_tol=1e-6)
    assert math.isclose(q_rot[2], 0.0, abs_tol=1e-6)


def test_single_axis_roll_equivalence_with_j9_1_2():
    """Verify that when active_axes={'roll'}, output matches j9_1_roll_attitude_closed_loop identically."""
    SCRIPT_ROLL = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_roll_attitude_closed_loop.py"
    spec_r = importlib.util.spec_from_file_location("j9_1_roll_closed_loop", SCRIPT_ROLL)
    mod_r = importlib.util.module_from_spec(spec_r)
    spec_r.loader.exec_module(mod_r)

    # 1. Single-axis runner harness
    h_single = mod_r.J91RollClosedLoopHarness.__new__(mod_r.J91RollClosedLoopHarness)
    import threading
    h_single.lock = threading.Lock()
    h_single.kp = 0.5
    h_single.residual_limit_deg = 0.25
    h_single.roll_ref_deg = 0.5
    h_single.latest_plat_rpy = (0.42, 0.05, -0.02)

    err_s, res_s, cmd_s = h_single.compute_control_step()

    # 2. Unified 3DOF runner harness
    h_3dof = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    h_3dof.lock = threading.Lock()
    h_3dof.active_axes = {"roll"}
    h_3dof.kp_roll = 0.5
    h_3dof.kp_pitch = 0.5
    h_3dof.kp_yaw = 0.5
    h_3dof.residual_limit_roll = 0.25
    h_3dof.residual_limit_pitch = 0.25
    h_3dof.residual_limit_yaw = 0.50
    h_3dof.ref_rpy = (0.5, 0.0, 0.0)
    h_3dof.latest_plat_rpy = (0.42, 0.05, -0.02)

    err_3, res_3, cmd_rpy_3, cmd_q_3 = h_3dof.compute_control_step()

    assert math.isclose(err_s, err_3[0], abs_tol=1e-6)
    assert math.isclose(res_s, res_3[0], abs_tol=1e-6)
    assert math.isclose(cmd_s, cmd_rpy_3[0], abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[1], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[2], 0.0, abs_tol=1e-6)


def test_single_axis_pitch_equivalence_with_j9_1_2():
    """Verify that when active_axes={'pitch'}, output matches j9_1_pitch_attitude_closed_loop identically."""
    SCRIPT_PITCH = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_pitch_attitude_closed_loop.py"
    spec_p = importlib.util.spec_from_file_location("j9_1_pitch_closed_loop", SCRIPT_PITCH)
    mod_p = importlib.util.module_from_spec(spec_p)
    spec_p.loader.exec_module(mod_p)

    # 1. Single-axis runner harness
    h_single = mod_p.J91PitchClosedLoopHarness.__new__(mod_p.J91PitchClosedLoopHarness)
    import threading
    h_single.lock = threading.Lock()
    h_single.kp = 0.5
    h_single.residual_limit_deg = 0.25
    h_single.pitch_ref_deg = 0.5
    h_single.latest_plat_rpy = (0.04, 0.42, -0.03)

    err_s, res_s, cmd_s = h_single.compute_control_step()

    # 2. Unified 3DOF runner harness
    h_3dof = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    h_3dof.lock = threading.Lock()
    h_3dof.active_axes = {"pitch"}
    h_3dof.kp_roll = 0.5
    h_3dof.kp_pitch = 0.5
    h_3dof.kp_yaw = 0.5
    h_3dof.residual_limit_roll = 0.25
    h_3dof.residual_limit_pitch = 0.25
    h_3dof.residual_limit_yaw = 0.50
    h_3dof.ref_rpy = (0.0, 0.5, 0.0)
    h_3dof.latest_plat_rpy = (0.04, 0.42, -0.03)

    err_3, res_3, cmd_rpy_3, cmd_q_3 = h_3dof.compute_control_step()

    assert math.isclose(err_s, err_3[1], abs_tol=1e-6)
    assert math.isclose(res_s, res_3[1], abs_tol=1e-6)
    assert math.isclose(cmd_s, cmd_rpy_3[1], abs_tol=1e-6)
    # Disabled roll and yaw channels must be 0
    assert math.isclose(res_3[0], 0.0, abs_tol=1e-6)
    assert math.isclose(res_3[2], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[0], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[2], 0.0, abs_tol=1e-6)


def test_single_axis_yaw_equivalence_with_j9_1_2():
    """Verify that when active_axes={'yaw'}, output matches j9_1_yaw_attitude_closed_loop identically."""
    SCRIPT_YAW = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_yaw_attitude_closed_loop.py"
    spec_y = importlib.util.spec_from_file_location("j9_1_yaw_closed_loop", SCRIPT_YAW)
    mod_y = importlib.util.module_from_spec(spec_y)
    spec_y.loader.exec_module(mod_y)

    # 1. Single-axis runner harness
    h_single = mod_y.J91YawClosedLoopHarness.__new__(mod_y.J91YawClosedLoopHarness)
    import threading
    h_single.lock = threading.Lock()
    h_single.kp = 0.5
    h_single.residual_limit_deg = 0.50
    h_single.yaw_ref_deg = 2.0
    h_single.latest_plat_rpy = (0.04, -0.02, 1.48)

    err_s, res_s, cmd_s = h_single.compute_control_step()

    # 2. Unified 3DOF runner harness
    h_3dof = exp_module.J913DofOrientationServo.__new__(exp_module.J913DofOrientationServo)
    h_3dof.lock = threading.Lock()
    h_3dof.active_axes = {"yaw"}
    h_3dof.kp_roll = 0.5
    h_3dof.kp_pitch = 0.5
    h_3dof.kp_yaw = 0.5
    h_3dof.residual_limit_roll = 0.25
    h_3dof.residual_limit_pitch = 0.25
    h_3dof.residual_limit_yaw = 0.50
    h_3dof.ref_rpy = (0.0, 0.0, 2.0)
    h_3dof.latest_plat_rpy = (0.04, -0.02, 1.48)

    err_3, res_3, cmd_rpy_3, cmd_q_3 = h_3dof.compute_control_step()

    assert math.isclose(err_s, err_3[2], abs_tol=1e-6)
    assert math.isclose(res_s, res_3[2], abs_tol=1e-6)
    assert math.isclose(cmd_s, cmd_rpy_3[2], abs_tol=1e-6)
    # Disabled roll and pitch channels must be 0
    assert math.isclose(res_3[0], 0.0, abs_tol=1e-6)
    assert math.isclose(res_3[1], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[0], 0.0, abs_tol=1e-6)
    assert math.isclose(cmd_rpy_3[1], 0.0, abs_tol=1e-6)




def test_csv_field_consistency():
    assert len(exp_module.CSV_FIELDS) == 54
    assert "active_axes" in exp_module.CSV_FIELDS
    assert "roll_residual_deg" in exp_module.CSV_FIELDS
    assert "cmd_qw" in exp_module.CSV_FIELDS
