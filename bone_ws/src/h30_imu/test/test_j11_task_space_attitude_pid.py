#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J11-1 Unit Tests & Numerical Regressions for Task-Space Attitude PID Controller.

Covers:
  1. SO(3) Attitude Error & Math:
     - Zero error -> zero output
     - Positive Roll, Pitch, Yaw error signs
     - Combined 3-axis error consistency
     - Small-angle equivalence with Euler channel
     - Quaternion sign invariance: q and -q produce identical physical results
     - SO(3) composition and round-trip consistency
  2. Gyro Coordinate Frame Transformation:
     - omega_P = R_P_IP @ omega_IP with R_P_IP = Rz(+90 deg)
     - IP +X -> P +Y
     - IP +Y -> P -X
     - IP +Z -> P +Z
  3. TaskSpaceAttitudePid Features:
     - P-only calculation and independent axis gains
     - Integral accumulation and clamp
     - Integrator anti-windup under output saturation
     - Invalid dt rejection (dt <= 0, nan, inf)
     - Excessive dt protection (dt > max_dt clamped)
     - Reset and Disable behavior (zeroes state, deterministic inactive)
     - D-term gyro damping (-Kd * omega_P) implementation check
  4. Numerical Regression with J9 P-Only Baseline:
     - Numerical verification that J11 with Ki=0, Kd=0 produces identical
       control directions and residuals as J9 verified P-only controller.
"""

import importlib.util
import math
import sys
from pathlib import Path
from typing import Set

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

# Resolve and import J11 controller module
SCRIPT_PATH = Path(__file__).resolve().parent.parent / "scripts" / "j11_task_space_attitude_pid.py"
spec = importlib.util.spec_from_file_location("j11_task_space_attitude_pid", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
j11_mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = j11_mod
spec.loader.exec_module(j11_mod)

# Resolve and import J9 SO(3) analysis module for numerical regression
J9_SO3_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_so3_error_offline_analysis.py"
spec_j9 = importlib.util.spec_from_file_location("j9_1_so3_error_offline_analysis", J9_SO3_SCRIPT)
assert spec_j9 is not None and spec_j9.loader is not None
j9_mod = importlib.util.module_from_spec(spec_j9)
sys.modules[spec_j9.name] = j9_mod
spec_j9.loader.exec_module(j9_mod)


# ==============================================================================
# 1. SO(3) Attitude Error & Math Tests
# ==============================================================================

def test_so3_zero_error():
    """Verify that identical reference and measured rotations yield strictly zero error."""
    R_ref = R.from_euler("xyz", [1.2, -0.8, 2.5], degrees=True).as_matrix()
    R_meas = R_ref.copy()

    e_W, e_ref = j11_mod.compute_so3_attitude_error(R_ref, R_meas)
    assert np.allclose(e_W, [0.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(e_ref, [0.0, 0.0, 0.0], atol=1e-12)

    # Output from PID on zero error must be zero
    pid = j11_mod.TaskSpaceAttitudePid(
        gains=j11_mod.PidGains.create(kp_roll=0.5, kp_pitch=0.5, kp_yaw=0.5)
    )
    cmd = pid.update(e_ref, dt_s=0.020)
    assert np.allclose(cmd, [0.0, 0.0, 0.0], atol=1e-12)


def test_so3_single_axis_positive_signs():
    """
    Verify that when reference > measured on each single axis,
    the error and P-only residual are strictly positive.
    """
    for axis_idx, axis_name in [(0, "roll"), (1, "pitch"), (2, "yaw")]:
        ref_deg = [0.0, 0.0, 0.0]
        meas_deg = [0.0, 0.0, 0.0]
        ref_deg[axis_idx] = 1.0
        meas_deg[axis_idx] = 0.5  # under-rotated measured attitude

        R_ref = j11_mod.euler_deg_to_rot_matrix(*ref_deg)
        R_meas = j11_mod.euler_deg_to_rot_matrix(*meas_deg)

        e_W, e_ref = j11_mod.compute_so3_attitude_error(R_ref, R_meas)

        # Target axis must be positive
        assert e_ref[axis_idx] > 0.0, f"Expected positive error on {axis_name}, got {e_ref[axis_idx]}"
        # Off-axis terms must be negligible
        for other_idx in range(3):
            if other_idx != axis_idx:
                assert abs(e_ref[other_idx]) < 1e-6

        # PID output must also be strictly positive
        pid = j11_mod.TaskSpaceAttitudePid(
            gains=j11_mod.PidGains.create(kp_roll=0.5, kp_pitch=0.5, kp_yaw=0.5)
        )
        cmd = pid.update(e_ref, dt_s=0.020)
        assert cmd[axis_idx] > 0.0, f"Expected positive command on {axis_name}, got {cmd[axis_idx]}"


def test_so3_combined_3axis_error():
    """Verify combined 3-axis attitude tracking error computation."""
    ref_deg = (1.5, -2.0, 3.0)
    meas_deg = (1.0, -1.0, 1.5)

    R_ref = j11_mod.euler_deg_to_rot_matrix(*ref_deg)
    R_meas = j11_mod.euler_deg_to_rot_matrix(*meas_deg)

    e_W, e_ref = j11_mod.compute_so3_attitude_error(R_ref, R_meas)
    assert len(e_W) == 3 and len(e_ref) == 3
    assert np.all(np.isfinite(e_W)) and np.all(np.isfinite(e_ref))

    # All three axes should reflect the respective deviations
    # Roll: ref(1.5) > meas(1.0) -> positive
    # Pitch: ref(-2.0) < meas(-1.0) -> negative
    # Yaw: ref(3.0) > meas(1.5) -> positive
    assert e_ref[0] > 0.0
    assert e_ref[1] < 0.0
    assert e_ref[2] > 0.0


def test_so3_small_angle_consistency():
    """Verify that for small angles (< 2 deg), e_ref matches Euler differences within 1%."""
    for r, p, y in [(0.5, 0.4, -0.6), (-1.0, 0.8, 1.2), (0.2, -0.3, 0.5)]:
        R_ref = j11_mod.euler_deg_to_rot_matrix(r, p, y)
        # Measured has small delta:
        delta_r, delta_p, delta_y = 0.1, -0.15, 0.2
        R_meas = j11_mod.euler_deg_to_rot_matrix(r - delta_r, p - delta_p, y - delta_y)

        _, e_ref_rad = j11_mod.compute_so3_attitude_error(R_ref, R_meas)
        e_ref_deg = np.degrees(e_ref_rad)

        expected_deg = np.array([delta_r, delta_p, delta_y])
        assert np.allclose(e_ref_deg, expected_deg, atol=0.01), (
            f"Small angle consistency failed: got {e_ref_deg}, expected {expected_deg}"
        )


def test_quaternion_sign_invariance():
    """
    Verify that quaternion antipodal equivalence q and -q represent the exact
    same physical attitude and produce strictly identical attitude error vectors.
    """
    np.random.seed(42)
    for _ in range(20):
        rot_ref = R.random()
        rot_meas = R.random()

        q_ref = rot_ref.as_quat()  # [x, y, z, w]
        q_meas = rot_meas.as_quat()

        # Compute error with standard signs
        e_w_1, e_ref_1 = j11_mod.compute_so3_attitude_error_quat(q_ref, q_meas)

        # Invert measured quaternion sign: -q_meas
        e_w_2, e_ref_2 = j11_mod.compute_so3_attitude_error_quat(q_ref, -q_meas)

        # Invert reference quaternion sign: -q_ref
        e_w_3, e_ref_3 = j11_mod.compute_so3_attitude_error_quat(-q_ref, q_meas)

        # Invert both signs: -q_ref, -q_meas
        e_w_4, e_ref_4 = j11_mod.compute_so3_attitude_error_quat(-q_ref, -q_meas)

        assert np.allclose(e_w_1, e_w_2, atol=1e-12)
        assert np.allclose(e_ref_1, e_ref_2, atol=1e-12)
        assert np.allclose(e_w_1, e_w_3, atol=1e-12)
        assert np.allclose(e_ref_1, e_ref_3, atol=1e-12)
        assert np.allclose(e_w_1, e_w_4, atol=1e-12)
        assert np.allclose(e_ref_1, e_ref_4, atol=1e-12)


def test_so3_residual_composition_and_normalization():
    """Verify SO(3) residual composition output matrix and quaternion normalization."""
    R_ref = j11_mod.euler_deg_to_rot_matrix(1.0, -2.0, 3.0)
    delta_phi = np.array([math.radians(0.2), math.radians(-0.1), math.radians(0.3)])

    R_cmd, q_cmd = j11_mod.compose_so3_residual(R_ref, delta_phi)

    # Verify R_cmd is valid SO(3)
    assert np.allclose(R_cmd @ R_cmd.T, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(R_cmd), 1.0, abs_tol=1e-12)

    # Verify unit quaternion
    norm_q = np.linalg.norm(q_cmd)
    assert math.isclose(norm_q, 1.0, abs_tol=1e-12)


# ==============================================================================
# 2. Gyro Frame Transformation Tests
# ==============================================================================

def test_gyro_frame_transformation_cardinal_axes():
    """
    Verify transformation: omega_P = R_P_IP @ omega_IP with R_P_IP = Rz(+90 deg).
    Required mappings:
      IP +X [1, 0, 0] -> P +Y [0, 1, 0]
      IP +Y [0, 1, 0] -> P -X [-1, 0, 0]
      IP +Z [0, 0, 1] -> P +Z [0, 0, 1]
    """
    # IP +X
    w_ip_x = [1.0, 0.0, 0.0]
    w_p_x = j11_mod.transform_gyro_ip_to_p(w_ip_x)
    assert np.allclose(w_p_x, [0.0, 1.0, 0.0], atol=1e-12), f"IP +X failed: {w_p_x}"

    # IP +Y
    w_ip_y = [0.0, 1.0, 0.0]
    w_p_y = j11_mod.transform_gyro_ip_to_p(w_ip_y)
    assert np.allclose(w_p_y, [-1.0, 0.0, 0.0], atol=1e-12), f"IP +Y failed: {w_p_y}"

    # IP +Z
    w_ip_z = [0.0, 0.0, 1.0]
    w_p_z = j11_mod.transform_gyro_ip_to_p(w_ip_z)
    assert np.allclose(w_p_z, [0.0, 0.0, 1.0], atol=1e-12), f"IP +Z failed: {w_p_z}"


def test_gyro_frame_transformation_arbitrary_vector():
    """Verify arbitrary 3D vector transformation under Rz(+90 deg)."""
    w_ip = [2.5, -4.0, 1.8]
    # Under Rz(+90 deg): [x, y, z] -> [-y, x, z]
    expected_w_p = [-w_ip[1], w_ip[0], w_ip[2]]
    w_p = j11_mod.transform_gyro_ip_to_p(w_ip)
    assert np.allclose(w_p, expected_w_p, atol=1e-12)


# ==============================================================================
# 3. Task-Space PID Controller Tests
# ==============================================================================

def test_pid_p_only_and_independent_gains():
    """Verify P-only calculation with distinct gains on Roll, Pitch, and Yaw."""
    gains = j11_mod.PidGains.create(kp_roll=0.4, kp_pitch=0.6, kp_yaw=0.8, ki_roll=0, ki_pitch=0, ki_yaw=0)
    limits = j11_mod.PidLimits(residual_limit_rad=np.array([1.0, 1.0, 1.0]))
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    error = np.array([0.01, 0.02, 0.03])  # rad
    cmd = pid.update(error, dt_s=0.020)

    expected = np.array([0.4 * 0.01, 0.6 * 0.02, 0.8 * 0.03])
    assert np.allclose(cmd, expected, atol=1e-12)


def test_pid_integral_accumulation_and_clamp():
    """Verify integrator accumulation and per-axis integrator state clamping."""
    i_lim = math.radians(0.5)  # clamp integral state at 0.5 deg*s
    limits = j11_mod.PidLimits(integral_limit_rad_s=np.array([i_lim, i_lim, i_lim]))
    gains = j11_mod.PidGains.create(
        kp_roll=0.0, kp_pitch=0.0, kp_yaw=0.0,
        ki_roll=1.0, ki_pitch=1.0, ki_yaw=1.0,
    )
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    error = np.array([0.05, -0.05, 0.05])
    # Run 20 cycles with dt = 0.02s: unconstrained integral would be 0.05 * 0.4 = 0.02 rad*s (~1.14 deg*s)
    for _ in range(20):
        pid.update(error, dt_s=0.020)

    # Integrator state must be clamped to i_lim
    assert math.isclose(pid.integral[0], i_lim, abs_tol=1e-9)
    assert math.isclose(pid.integral[1], -i_lim, abs_tol=1e-9)
    assert math.isclose(pid.integral[2], i_lim, abs_tol=1e-9)


def test_pid_anti_windup_under_output_saturation():
    """Verify conditional integration freezes integral when output is saturated in error direction."""
    res_lim = math.radians(0.25)
    limits = j11_mod.PidLimits(
        residual_limit_rad=np.array([res_lim, res_lim, res_lim]),
        integral_limit_rad_s=np.array([10.0, 10.0, 10.0]),  # large integral clamp
    )
    gains = j11_mod.PidGains.create(kp_roll=1.0, ki_roll=1.0)
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    # Large error that immediately saturates output: 1.0 rad >> 0.25 deg
    large_err = np.array([1.0, 0.0, 0.0])
    pid.update(large_err, dt_s=0.020)
    assert pid.saturated_axes[0] is True
    int_after_step1 = pid.integral[0]

    # Next step with same-sign error: anti-windup should freeze integration
    pid.update(large_err, dt_s=0.020)
    assert math.isclose(pid.integral[0], int_after_step1, abs_tol=1e-9), (
        f"Anti-windup failed: integral grew from {int_after_step1} to {pid.integral[0]}"
    )


def test_pid_output_residual_saturation():
    """Verify output residual is strictly bounded within residual_limit_rad."""
    lim = math.radians(0.5)
    limits = j11_mod.PidLimits(residual_limit_rad=np.array([lim, lim, lim]))
    gains = j11_mod.PidGains.create(kp_roll=10.0, kp_pitch=10.0, kp_yaw=10.0)
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    huge_error = np.array([1.5, -2.0, 0.8])
    cmd = pid.update(huge_error, dt_s=0.020)

    assert math.isclose(cmd[0], lim, abs_tol=1e-12)
    assert math.isclose(cmd[1], -lim, abs_tol=1e-12)
    assert math.isclose(cmd[2], lim, abs_tol=1e-12)


def test_pid_dt_protections():
    """Verify invalid dt rejection and excessive dt clamping."""
    gains = j11_mod.PidGains.create(kp_roll=1.0, ki_roll=1.0)
    limits = j11_mod.PidLimits(min_dt_s=0.001, max_dt_s=0.050)
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    # 1. Invalid dt (<= 0 or NaN/Inf)
    cmd0 = pid.update([0.1, 0.0, 0.0], dt_s=0.020)
    # Passing dt = 0: should hold last cmd and not integrate
    int_before = pid.integral.copy()
    cmd_zero = pid.update([0.1, 0.0, 0.0], dt_s=0.0)
    assert np.allclose(cmd_zero, cmd0)
    assert np.allclose(pid.integral, int_before)

    # Passing dt = NaN
    cmd_nan = pid.update([0.1, 0.0, 0.0], dt_s=float("nan"))
    assert np.allclose(cmd_nan, cmd0)
    assert np.allclose(pid.integral, int_before)

    # 2. Excessive dt: 1.0s >> max_dt_s(0.050s)
    pid.reset()
    pid.update([0.1, 0.0, 0.0], dt_s=1.0)
    # Integral should only advance by 0.1 * 0.050 = 0.005, not 0.1 * 1.0 = 0.10
    assert math.isclose(pid.integral[0], 0.1 * 0.050, abs_tol=1e-9)


def test_pid_disable_and_reset_behavior():
    """
    Verify PID disable zeroes integral and command residual,
    entering deterministic inactive state without auto-return to HOME.
    """
    gains = j11_mod.PidGains.create(kp_roll=1.0, ki_roll=1.0)
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains)

    # Run steps to build up state
    pid.update([0.2, 0.1, -0.1], dt_s=0.020)
    assert not np.allclose(pid.last_cmd, 0.0)
    assert not np.allclose(pid.integral, 0.0)

    # Disable
    pid.disable()
    assert pid.enabled is False
    assert np.allclose(pid.integral, [0.0, 0.0, 0.0])
    assert np.allclose(pid.last_cmd, [0.0, 0.0, 0.0])

    # Updates while disabled return zero residual
    cmd_disabled = pid.update([0.5, 0.5, 0.5], dt_s=0.020)
    assert np.allclose(cmd_disabled, [0.0, 0.0, 0.0])


def test_pid_d_term_gyro_damping():
    """Verify D-term damping via Platform-frame angular velocity."""
    gains = j11_mod.PidGains.create(
        kp_roll=0.0, kp_pitch=0.0, kp_yaw=0.0,
        kd_roll=0.1, kd_pitch=0.1, kd_yaw=0.1,
    )
    limits = j11_mod.PidLimits(residual_limit_rad=np.array([1.0, 1.0, 1.0]))
    pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)

    # Error is 0, but platform is rotating at omega_P = [1.0, -2.0, 0.5] rad/s
    omega_p = [1.0, -2.0, 0.5]
    cmd = pid.update([0.0, 0.0, 0.0], dt_s=0.020, omega_p_rad_s=omega_p)

    # D-term opposes angular velocity: -Kd * omega_P
    expected = -0.1 * np.array(omega_p)
    assert np.allclose(cmd, expected, atol=1e-12)


# ==============================================================================
# 4. Numerical Regression Against J9 Verified P-Only Baseline
# ==============================================================================

def test_j9_p_only_numerical_regression():
    """
    Strict numerical regression: verify that J11 TaskSpaceAttitudePid
    (with Ki=0, Kd=0) produces identical control direction, error formulation,
    and clamped residual outputs as J9.1-3 verified candidate controller.
    """
    np.random.seed(999)
    kp_val = 0.5
    lim_deg = (0.25, 0.25, 0.50)
    lim_rad = np.radians(lim_deg)

    gains = j11_mod.PidGains.create(kp_roll=kp_val, kp_pitch=kp_val, kp_yaw=kp_val)
    limits = j11_mod.PidLimits(residual_limit_rad=lim_rad)

    for _ in range(50):
        # Small attitude configurations within operational window
        ref_rpy_deg = np.random.uniform(-1.0, 1.0, size=3)
        meas_rpy_deg = np.random.uniform(-1.0, 1.0, size=3)

        # 1. J9 evaluation
        j9_eval, _ = j9_mod.evaluate_so3_candidate_controller(
            ref_rpy_deg,
            meas_rpy_deg,
            active_axes={"roll", "pitch", "yaw"},
            kp=(kp_val, kp_val, kp_val),
            limits_deg=lim_deg,
        )

        # 2. J11 evaluation
        R_ref = j11_mod.euler_deg_to_rot_matrix(*ref_rpy_deg)
        R_meas = j11_mod.euler_deg_to_rot_matrix(*meas_rpy_deg)
        _, e_ref_rad = j11_mod.compute_so3_attitude_error(R_ref, R_meas)

        pid = j11_mod.TaskSpaceAttitudePid(gains=gains, limits=limits)
        cmd_rad = pid.update(e_ref_rad, dt_s=0.020)
        cmd_deg = np.degrees(cmd_rad)

        # Assert identical error vector in reference body frame
        assert np.allclose(np.degrees(e_ref_rad), j9_eval.e_vec_deg, atol=1e-10)

        # Assert identical clamped residual output
        assert np.allclose(cmd_deg, j9_eval.residual_deg, atol=1e-10)

        # Assert identical composed target quaternion
        _, q_cmd_j11 = j11_mod.compose_so3_residual(R_ref, cmd_rad)
        # Quaternions must represent same rotation (dot product magnitude close to 1)
        dot_q = abs(sum(q_cmd_j11[i] * j9_eval.q_cmd_xyzw[i] for i in range(4)))
        assert math.isclose(dot_q, 1.0, abs_tol=1e-10)
