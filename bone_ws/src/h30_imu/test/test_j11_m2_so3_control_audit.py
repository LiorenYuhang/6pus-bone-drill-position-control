#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J11-M2-0: Comprehensive SO(3) Control-Law and Interface Audit Offline Test Suite.

Covers all 11 required audit checks from Section 11:
  1. zero error -> zero PID residual
  2. +Roll error -> corrective Roll direction
  3. -Roll symmetry
  4. Pitch +/- error & correction
  5. Yaw +/- error & correction
  6. Combined 3DOF SO(3) decoupling and correction
  7. Ideal feedforward -> Platform residual ≈ 0
  8. Artificial Platform error under tilted Base -> corrective command
  9. Frame-transform identity checks
  10. M1 frozen behavior unchanged
  11. hardware_write_count = 0 enforcement
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time
from typing import List, Tuple

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

# Add paths for h30_imu scripts and pus6_kinematics
_TEST_DIR = Path(__file__).resolve().parent
_SCRIPTS_DIR = _TEST_DIR.parent / "scripts"
_KINEMATICS_DIR = _TEST_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPTS_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

try:
    import rclpy
except ImportError:
    rclpy = None

import j11_base_feedforward_runner as m1
import j11_m2_pid_residual_runner as m2


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initializes rclpy session for nodes used in tests."""
    if rclpy is not None and not rclpy.ok():
        rclpy.init()
    yield
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


# ==============================================================================
# 1. Zero Error -> Zero PID Residual
# ==============================================================================
def test_zero_error_yields_zero_residual():
    """Verify that when platform world attitude matches reference (R_WP = I), error and PID residuals are strictly zero."""
    R_WP_des = np.eye(3)
    R_WP_meas = np.eye(3)

    E_W, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)
    assert np.allclose(E_W, np.eye(3), atol=1e-15)
    assert np.allclose(e_R_W, [0.0, 0.0, 0.0], atol=1e-15)

    pid = m2.M2TaskSpaceAttitudePid(
        gains=m2.M2PidGains(
            kp=np.array([1.0, 1.0, 1.0]),
            ki=np.array([0.5, 0.5, 0.5]),
            kd=np.array([0.1, 0.1, 0.1]),
        )
    )
    res = pid.update(e_R_W, dt_s=0.020)
    assert np.allclose(res, [0.0, 0.0, 0.0], atol=1e-15)

    # Command composition with zero residual must yield pure feedforward
    R_WB = R.from_euler("xyz", [5.0, -3.0, 8.0], degrees=True).as_matrix()
    R_BP_FF = R_WB.T
    _, _, R_BP_cmd, _ = m2.compose_so3_pid_residual(R_WB, R_BP_FF, res)
    assert np.allclose(R_BP_cmd, R_BP_FF, atol=1e-15)


# ==============================================================================
# 2. +Roll Error -> Corrective Roll Direction (-Roll)
# ==============================================================================
def test_positive_roll_error_corrective_direction():
    """Verify +Roll disturbance results in negative e_R_W and negative Roll correction (true negative feedback)."""
    deg = 1.0
    theta = math.radians(deg)
    R_WP_meas = R.from_euler("xyz", [deg, 0.0, 0.0], degrees=True).as_matrix()
    R_WP_des = np.eye(3)

    E_W, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)
    # E_W = I @ R_x(+theta).T = R_x(-theta)
    assert np.allclose(E_W, R.from_rotvec([-theta, 0, 0]).as_matrix(), atol=1e-12)
    assert e_R_W[0] < 0.0, "Expected negative error vector on Roll"
    assert abs(e_R_W[1]) < 1e-12 and abs(e_R_W[2]) < 1e-12

    # Proportional controller Kp > 0
    kp_val = 0.6
    pid = m2.M2TaskSpaceAttitudePid(
        gains=m2.M2PidGains(kp=np.array([kp_val, 0.0, 0.0]))
    )
    delta_phi_W = pid.update(e_R_W, dt_s=0.020)

    # Corrective residual must be strictly negative on Roll (counteracting +Roll)
    assert delta_phi_W[0] < 0.0
    assert np.isclose(delta_phi_W[0], -kp_val * theta, atol=1e-6)

    # Apply to stationary base: resulting platform attitude must have reduced roll
    R_WB = np.eye(3)
    R_BP_FF = np.eye(3)
    _, _, R_BP_cmd, _ = m2.compose_so3_pid_residual(R_WB, R_BP_FF, delta_phi_W)
    R_WP_after = R_WP_meas @ R_BP_cmd  # platform applies corrective command
    roll_after_deg = m2.quat_to_rpy_deg(m2.rot_matrix_to_quat_xyzw(R_WP_after))[0]
    assert 0.0 < roll_after_deg < deg, f"Expected attenuated roll, got {roll_after_deg}"
    assert np.isclose(roll_after_deg, deg * (1.0 - kp_val), atol=1e-3)


# ==============================================================================
# 3. -Roll Symmetry
# ==============================================================================
def test_negative_roll_symmetry():
    """Verify -Roll error produces strictly symmetric positive corrective residual."""
    theta = math.radians(2.0)
    R_WP_meas_pos = R.from_euler("xyz", [2.0, 0.0, 0.0], degrees=True).as_matrix()
    R_WP_meas_neg = R.from_euler("xyz", [-2.0, 0.0, 0.0], degrees=True).as_matrix()
    R_WP_des = np.eye(3)

    _, e_pos = m2.compute_so3_platform_error(R_WP_des, R_WP_meas_pos)
    _, e_neg = m2.compute_so3_platform_error(R_WP_des, R_WP_meas_neg)

    assert np.isclose(e_pos[0], -e_neg[0], atol=1e-15)
    assert np.isclose(e_pos[1], -e_neg[1], atol=1e-15)
    assert np.isclose(e_pos[2], -e_neg[2], atol=1e-15)

    pid_pos = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.5, 0.5, 0.5])))
    pid_neg = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.5, 0.5, 0.5])))

    res_pos = pid_pos.update(e_pos, dt_s=0.020)
    res_neg = pid_neg.update(e_neg, dt_s=0.020)

    assert np.allclose(res_pos, -res_neg, atol=1e-15)


# ==============================================================================
# 4. Pitch +/- Error & Correction
# ==============================================================================
def test_pitch_positive_negative_correction():
    """Verify Pitch +/- disturbances produce decoupled negative feedback without Roll/Yaw cross-talk."""
    R_WP_des = np.eye(3)
    for sign in (+1.0, -1.0):
        deg = sign * 1.5
        R_WP_meas = R.from_euler("xyz", [0.0, deg, 0.0], degrees=True).as_matrix()
        _, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)

        # Expected error sign on pitch is opposite to disturbance
        assert (e_R_W[1] * sign) < 0.0
        assert abs(e_R_W[0]) < 1e-12 and abs(e_R_W[2]) < 1e-12

        pid = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.0, 0.8, 0.0])))
        res = pid.update(e_R_W, dt_s=0.020)

        assert (res[1] * sign) < 0.0
        assert abs(res[0]) < 1e-12 and abs(res[2]) < 1e-12


# ==============================================================================
# 5. Yaw +/- Error & Correction
# ==============================================================================
def test_yaw_positive_negative_correction():
    """Verify Yaw +/- disturbances produce decoupled negative feedback without Roll/Pitch cross-talk."""
    R_WP_des = np.eye(3)
    for sign in (+1.0, -1.0):
        deg = sign * 2.5
        R_WP_meas = R.from_euler("xyz", [0.0, 0.0, deg], degrees=True).as_matrix()
        _, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)

        assert (e_R_W[2] * sign) < 0.0
        assert abs(e_R_W[0]) < 1e-12 and abs(e_R_W[1]) < 1e-12

        pid = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.0, 0.0, 0.4])))
        res = pid.update(e_R_W, dt_s=0.020)

        assert (res[2] * sign) < 0.0
        assert abs(res[0]) < 1e-12 and abs(res[1]) < 1e-12


# ==============================================================================
# 6. Combined 3DOF SO(3) Decoupling and Correction
# ==============================================================================
def test_combined_3dof_so3_correction():
    """Verify combined Roll, Pitch, Yaw disturbance produces simultaneous 3-axis decoupled correction."""
    r_deg, p_deg, y_deg = 1.8, -1.2, 2.4
    R_WP_meas = R.from_euler("xyz", [r_deg, p_deg, y_deg], degrees=True).as_matrix()
    R_WP_des = np.eye(3)

    _, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)
    e_deg = np.degrees(e_R_W)

    # First-order check: errors should be roughly opposite the perturbation
    assert np.isclose(e_deg[0], -r_deg, atol=0.05)
    assert np.isclose(e_deg[1], -p_deg, atol=0.05)
    assert np.isclose(e_deg[2], -y_deg, atol=0.05)

    kp = np.array([0.5, 0.6, 0.7])
    pid = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=kp))
    res_W = pid.update(e_R_W, dt_s=0.020)

    # Check signs
    assert res_W[0] < 0.0  # counteracts +r
    assert res_W[1] > 0.0  # counteracts -p
    assert res_W[2] < 0.0  # counteracts +y


# ==============================================================================
# 7. Ideal Feedforward -> Platform Residual ≈ 0
# ==============================================================================
def test_ideal_feedforward_zero_platform_residual():
    """Under ideal feedforward kinematics, platform perfectly cancels base motion; platform error is zero."""
    for base_angles in [
        [3.0, 0.0, 0.0],
        [0.0, -4.0, 0.0],
        [0.0, 0.0, 5.0],
        [2.0, -2.5, 3.5],
    ]:
        R_WB = R.from_euler("xyz", base_angles, degrees=True).as_matrix()
        R_BP_FF = R_WB.T

        # Under ideal kinematics, platform follows R_BP_FF:
        R_WP_ideal = R_WB @ R_BP_FF
        assert np.allclose(R_WP_ideal, np.eye(3), atol=1e-15)

        # Platform measurement is R_WP_ideal = I
        _, e_R_W = m2.compute_so3_platform_error(np.eye(3), R_WP_ideal)
        assert np.allclose(e_R_W, [0.0, 0.0, 0.0], atol=1e-15)

        # PID residual is zero
        pid = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.5, 0.5, 0.5])))
        res = pid.update(e_R_W, dt_s=0.020)
        assert np.allclose(res, [0.0, 0.0, 0.0], atol=1e-15)

        # Resulting command is pure feedforward
        _, _, R_BP_cmd, _ = m2.compose_so3_pid_residual(R_WB, R_BP_FF, res)
        assert np.allclose(R_BP_cmd, R_BP_FF, atol=1e-15)


# ==============================================================================
# 8. Artificial Platform Error Under Tilted Base -> Corrective Command
# ==============================================================================
def test_artificial_platform_error_under_tilted_base():
    """Verify that when base is tilted (Roll +8 deg) and platform has an error (Pitch +1.5 deg in World),

    the PID residual transforms correctly to Base frame and corrects the Pitch error without creating Roll/Yaw distortion.
    """
    R_WB = R.from_euler("xyz", [8.0, 0.0, 0.0], degrees=True).as_matrix()
    R_BP_FF = R_WB.T

    # Platform has a Pitch lag of +1.5 deg in World
    R_WP_meas = R.from_euler("xyz", [0.0, 1.5, 0.0], degrees=True).as_matrix()
    _, e_R_W = m2.compute_so3_platform_error(np.eye(3), R_WP_meas)

    kp = 0.5
    pid = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kp=np.array([0.0, kp, 0.0])))
    delta_phi_W = pid.update(e_R_W, dt_s=0.020)

    Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, _ = m2.compose_so3_pid_residual(
        R_WB, R_BP_FF, delta_phi_W
    )

    # Check resulting platform world orientation if platform executes R_BP_cmd:
    R_WP_executed = R_WB @ R_BP_cmd
    assert np.allclose(R_WP_executed, Delta_R_PID_W, atol=1e-14)

    # Convert to euler angles: Roll and Yaw should be strictly 0.0, Pitch should be -kp * 1.5 deg
    r, p, y = m2.rot_matrix_to_euler_deg(R_WP_executed)
    assert abs(r) < 1e-12, f"Spurious roll cross-talk: {r}"
    assert abs(y) < 1e-12, f"Spurious yaw cross-talk: {y}"
    assert np.isclose(p, -kp * 1.5, atol=1e-4), f"Expected pitch {-kp * 1.5}, got {p}"


# ==============================================================================
# 9. Frame-Transform Identity Checks
# ==============================================================================
def test_frame_transform_algebraic_identities():
    """Verify all mathematical identities of the authoritative SO(3) composition law:

    1. Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB
    2. Delta_R_PID_B @ R_BP_FF == R_BP_FF @ Delta_R_PID_W
    3. R_WB @ (Delta_R_PID_B @ R_BP_FF) == Delta_R_PID_W
    4. Delta_R_PID_B == Exp([R_WB.T @ delta_phi_W]_x)
    """
    np.random.seed(42)
    for _ in range(20):
        R_WB = R.random().as_matrix()
        R_BP_FF = R_WB.T

        phi_W = (np.random.rand(3) - 0.5) * 0.05  # small rotation vector
        Delta_R_PID_W = R.from_rotvec(phi_W).as_matrix()

        # Transform to base
        Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB

        # Left multiplication on R_BP_FF
        R_cmd_left = Delta_R_PID_B @ R_BP_FF

        # Right multiplication on R_BP_FF
        R_cmd_right = R_BP_FF @ Delta_R_PID_W

        # Identity 1: Left composition in Base frame == Right composition in World frame
        assert np.allclose(R_cmd_left, R_cmd_right, atol=1e-15)

        # Identity 2: Resulting world attitude is exactly Delta_R_PID_W
        R_WP_actual = R_WB @ R_cmd_left
        assert np.allclose(R_WP_actual, Delta_R_PID_W, atol=1e-15)

        # Identity 3: Lie algebra adjoint representation Exp([R.T v]_x) == R.T Exp([v]_x) R
        phi_B = R_WB.T @ phi_W
        Delta_R_PID_B_adjoint = R.from_rotvec(phi_B).as_matrix()
        assert np.allclose(Delta_R_PID_B, Delta_R_PID_B_adjoint, atol=1e-15)


# ==============================================================================
# 10. M1 Frozen Behavior Unchanged Verification
# ==============================================================================
def test_m1_frozen_behavior_unchanged():
    """Verify that when PID is disabled or gains are 0, M2 math produces identical outputs to frozen M1."""
    # Compare math functions between M1 runner and M2 runner
    q0_base = (0.01, 0.02, 0.03, math.sqrt(1 - 0.01**2 - 0.02**2 - 0.03**2))
    q_base = (0.05, -0.04, 0.02, math.sqrt(1 - 0.05**2 - 0.04**2 - 0.02**2))

    # M1 calculation
    q_rel_m1 = m1.relative_quat(q0_base, q_base)
    q_mech_m1 = m1.apply_z90_transform(q_rel_m1)
    Delta_R_B_m1 = m1.quat_to_rot_matrix(q_mech_m1)
    R_BP_cmd_m1 = Delta_R_B_m1.T

    # M2 calculation with zero PID residual
    q_rel_m2 = m2.relative_quat(q0_base, q_base)
    q_mech_m2 = m2.apply_z90_transform(q_rel_m2)
    R_WB_m2 = m2.quat_to_rot_matrix(q_mech_m2)
    R_BP_FF_m2 = R_WB_m2.T
    _, _, R_BP_cmd_m2, _ = m2.compose_so3_pid_residual(R_WB_m2, R_BP_FF_m2, [0.0, 0.0, 0.0])

    assert np.allclose(R_BP_cmd_m1, R_BP_cmd_m2, atol=1e-15)


# ==============================================================================
# 11. Hardware Write Count = 0 Enforcement
# ==============================================================================
def test_hardware_write_count_strictly_zero_in_dry_run():
    """Verify that M2 runner in dry-run mode executes zero hardware writes."""
    runner = m2.J11M2PidResidualRunner(
        execute_motion=False,
        dry_run=True,
        auto_start=True,
    )
    assert runner.hardware_write_count == 0
    assert runner.dry_run is True
    assert runner.execute_motion is False

    # Simulate ticks with dummy data
    q_dummy = (0.0, 0.0, 0.0, 1.0)
    runner.latest_base_q_raw = q_dummy
    runner.latest_plat_q_raw = q_dummy
    runner.latest_base_rx_time_ns = time.perf_counter_ns()
    runner.latest_plat_rx_time_ns = time.perf_counter_ns()

    for _ in range(60):
        runner._on_control_tick()

    assert runner.hardware_write_count == 0
    assert len(runner.telemetry_history) == 60
    assert all(not r.hardware_write_issued for r in runner.telemetry_history)


# ==============================================================================
# 12. IK Sanity Check for Roll Residuals (±0.1°, ±0.5°)
# ==============================================================================
def test_ik_sanity_roll_residuals():
    """Verify analytical IK step displacements for Roll residuals ±0.1 deg and ±0.5 deg."""
    from analytical_6pus import Analytical6PusKinematics
    from geometry_config import load_geometry_config

    kin = Analytical6PusKinematics()
    geom = load_geometry_config()
    z0 = kin.nominal_height
    home_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    home_steps = list(geom.operating_home_raw_steps)

    expected_deltas = {
        +0.1: [2, 0, -2, -2, 0, 2],
        -0.1: [-2, 0, 2, 2, 0, -2],
        +0.5: [8, 0, -8, -8, 0, 8],
        -0.5: [-8, 0, 8, 8, 0, -8],
    }

    for test_roll_deg, exp_ds in expected_deltas.items():
        roll_rad = math.radians(test_roll_deg)
        cart_pose = [0.0, 0.0, z0, roll_rad, 0.0, 0.0]
        delta_steps = kin.differential_inverse_kinematics_steps(cart_pose, home_pose)
        assert delta_steps == exp_ds, f"Mismatch on Roll {test_roll_deg}: got {delta_steps}, expected {exp_ds}"

        pred_steps = [h + ds for h, ds in zip(home_steps, delta_steps)]
        assert all(0 <= s <= 2000 for s in pred_steps)
        assert all(abs(s - 1000) <= 10 for s in pred_steps)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

