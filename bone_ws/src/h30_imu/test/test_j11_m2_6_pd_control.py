#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J11-M2-6-0: Comprehensive 3DOF Task-Space PD Architecture Offline Test Suite.

Covers all required architectural and mathematical verification items:
  1. Zero motion: e_dot = 0, D = 0
  2. Constant positive angular motion: D opposes motion (Roll / Pitch / Yaw)
  3. Constant negative angular motion: D opposes motion (Roll / Pitch / Yaw)
  4. Step error: derivative transient bounded and smooth exponential decay
  5. Sinusoidal error: filtered derivative phase lead and proper damping opposing platform velocity
  6. Kd = 0 backward compatibility: numerically identical to P-only baseline (atol=1e-15)
  7. Residual saturation limiting: strictly respected even under large derivative spikes
  8. Configurable LPF filter: cutoff frequency response and bypass behavior
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import List, Tuple

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

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
# 1. Zero Motion: e_dot = 0, D = 0
# ==============================================================================
def test_zero_motion_yields_zero_derivative():
    """Verify that stationary errors (including zero error and static non-zero error) produce D = 0."""
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.02, 0.02, 0.01]),  # Active Kd
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(0.5)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    # Sequence of identical zero errors
    for _ in range(10):
        cmd = pid.update(e_R_W_rad=[0.0, 0.0, 0.0], dt_s=0.020)
        assert np.allclose(pid.last_d_term, [0.0, 0.0, 0.0], atol=1e-15)
        assert np.allclose(pid.last_raw_derivative, [0.0, 0.0, 0.0], atol=1e-15)
        assert np.allclose(pid.last_filtered_derivative, [0.0, 0.0, 0.0], atol=1e-15)
        assert np.allclose(cmd, [0.0, 0.0, 0.0], atol=1e-15)

    # Static non-zero error: e stays constant at [0.01, -0.02, 0.015] rad
    static_e = [0.01, -0.02, 0.015]
    pid.reset()
    cmd1 = pid.update(e_R_W_rad=static_e, dt_s=0.020)
    # First cycle: no prev error -> derivative is 0
    assert np.allclose(pid.last_d_term, [0.0, 0.0, 0.0], atol=1e-15)

    # Subsequent cycles with unchanged error -> derivative strictly 0
    for _ in range(10):
        cmd = pid.update(e_R_W_rad=static_e, dt_s=0.020)
        assert np.allclose(pid.last_raw_derivative, [0.0, 0.0, 0.0], atol=1e-15)
        assert np.allclose(pid.last_filtered_derivative, [0.0, 0.0, 0.0], atol=1e-15)
        assert np.allclose(pid.last_d_term, [0.0, 0.0, 0.0], atol=1e-15)
        # Total output must be purely proportional
        assert np.allclose(cmd, gains.kp * np.asarray(static_e), atol=1e-15)


# ==============================================================================
# 2. Constant Positive Angular Motion: D Opposes Motion
# ==============================================================================
@pytest.mark.parametrize("axis_idx,axis_name", [(0, "Roll"), (1, "Pitch"), (2, "Yaw")])
def test_constant_positive_angular_motion_d_opposes(axis_idx, axis_name):
    """Verify that when Platform rotates in positive direction, D contribution is negative (opposing motion)."""
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.02, 0.02, 0.01]),
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(1.0)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    dt = 0.020  # 50 Hz
    rate_deg_s = 5.0  # +5 deg/s
    rate_rad_s = math.radians(rate_deg_s)
    R_WP_des = np.eye(3)

    # Simulate 20 steps of positive angular rate
    for k in range(20):
        angle_rad = rate_rad_s * (k * dt)
        rotvec = np.zeros(3)
        rotvec[axis_idx] = angle_rad
        R_WP_meas = R.from_rotvec(rotvec).as_matrix()

        _, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)
        pid.update(e_R_W, dt_s=dt)

        if k >= 5:  # Allow filter to settle
            # Measured attitude increasing in positive direction -> error is negative and decreasing
            assert e_R_W[axis_idx] < 0.0, f"{axis_name} error should be negative"
            assert pid.last_raw_derivative[axis_idx] < 0.0, f"{axis_name} raw derivative must be negative"
            assert pid.last_filtered_derivative[axis_idx] < 0.0, f"{axis_name} filtered derivative must be negative"
            assert pid.last_d_term[axis_idx] < 0.0, f"{axis_name} D term must be NEGATIVE to oppose +motion"

            # Check numerical rate agreement with tolerance for discrete approximation
            expected_deriv = -rate_rad_s
            assert abs(pid.last_filtered_derivative[axis_idx] - expected_deriv) < 0.05 * rate_rad_s


# ==============================================================================
# 3. Constant Negative Angular Motion: D Opposes Motion
# ==============================================================================
@pytest.mark.parametrize("axis_idx,axis_name", [(0, "Roll"), (1, "Pitch"), (2, "Yaw")])
def test_constant_negative_angular_motion_d_opposes(axis_idx, axis_name):
    """Verify that when Platform rotates in negative direction, D contribution is positive (opposing motion)."""
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.02, 0.02, 0.01]),
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(1.0)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    dt = 0.020
    rate_deg_s = -5.0  # -5 deg/s
    rate_rad_s = math.radians(rate_deg_s)
    R_WP_des = np.eye(3)

    for k in range(20):
        angle_rad = rate_rad_s * (k * dt)
        rotvec = np.zeros(3)
        rotvec[axis_idx] = angle_rad
        R_WP_meas = R.from_rotvec(rotvec).as_matrix()

        _, e_R_W = m2.compute_so3_platform_error(R_WP_des, R_WP_meas)
        pid.update(e_R_W, dt_s=dt)

        if k >= 5:
            # Measured attitude decreasing (negative rate) -> error is positive and increasing
            assert e_R_W[axis_idx] > 0.0, f"{axis_name} error should be positive"
            assert pid.last_raw_derivative[axis_idx] > 0.0, f"{axis_name} raw derivative must be positive"
            assert pid.last_filtered_derivative[axis_idx] > 0.0, f"{axis_name} filtered derivative must be positive"
            assert pid.last_d_term[axis_idx] > 0.0, f"{axis_name} D term must be POSITIVE to oppose -motion"


# ==============================================================================
# 4. Step Error: Derivative Transient Bounded
# ==============================================================================
def test_step_error_derivative_transient_bounded():
    """Verify that a sudden step disturbance produces a bounded, smooth decaying derivative transient."""
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.02, 0.02, 0.01]),
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(0.5)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    dt = 0.020
    # Step from 0 to 0.02 rad at step 5
    d_history = []
    cmd_history = []
    for k in range(25):
        e = [0.0, 0.02 if k >= 5 else 0.0, 0.0]
        cmd = pid.update(e, dt_s=dt)
        d_history.append(pid.last_d_term[1])
        cmd_history.append(cmd[1])

    # 1. Before step: D == 0
    for k in range(5):
        assert d_history[k] == 0.0

    # 2. At step (k=5): raw derivative spikes to 0.02/0.02 = 1.0 rad/s
    # D term is positive (e increased) and bounded
    assert d_history[5] > 0.0
    assert math.isfinite(d_history[5])

    # 3. After step (k > 5): raw derivative is 0, filtered derivative decays exponentially
    for k in range(6, 25):
        assert d_history[k] < d_history[k - 1], "Derivative transient must monotonically decay"
        assert d_history[k] >= 0.0

    # 4. By step 20, transient is virtually decayed
    assert d_history[20] < 0.01 * d_history[5]

    # 5. Output command is strictly within limits
    for c in cmd_history:
        assert abs(c) <= math.radians(0.5) + 1e-12


# ==============================================================================
# 5. Sinusoidal Error: Phase Lead and Motion Opposing Damping
# ==============================================================================
def test_sinusoidal_error_phase_and_damping():
    """Verify that for harmonic disturbance, D term leads error and strictly opposes platform velocity."""
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.02, 0.02, 0.01]),
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(1.0)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    dt = 0.020  # 50 Hz
    f = 1.0     # 1 Hz oscillation
    omega = 2.0 * math.pi * f
    A = math.radians(2.0)  # 2 deg amplitude

    errors = []
    p_terms = []
    d_terms = []
    plat_velocities = []

    # Simulate 2 full cycles (100 steps)
    for k in range(100):
        t = k * dt
        plat_angle = A * math.sin(omega * t)
        plat_vel = A * omega * math.cos(omega * t)  # Analytical velocity

        # Error with R_WP_des = I: e(t) = -plat_angle
        e = [-plat_angle, 0.0, 0.0]
        pid.update(e, dt_s=dt)

        if k >= 20:  # Skip initial transient
            errors.append(e[0])
            p_terms.append(pid.last_p_term[0])
            d_terms.append(pid.last_d_term[0])
            plat_velocities.append(plat_vel)

    # Correlation between D term and platform velocity must be strongly negative (opposing motion)
    corr_d_vel = np.corrcoef(d_terms, plat_velocities)[0, 1]
    assert corr_d_vel < -0.90, f"D term must strongly oppose platform velocity (corr={corr_d_vel:.4f} < -0.90)"

    # Phase relation: P and D must be approximately orthogonal (quarter-period shift)
    corr_p_d = np.corrcoef(p_terms, d_terms)[0, 1]
    assert abs(corr_p_d) < 0.35, f"P and D terms should have near-orthogonal phase (corr={corr_p_d:.4f})"


# ==============================================================================
# 6. Kd = 0 Numerical Equivalence / Backward Compatibility
# ==============================================================================
def test_kd_zero_exact_backward_compatibility():
    """Verify that when Kd = [0, 0, 0], PD controller output is bitwise / numerically identical to P-only."""
    kp_baseline = np.array([0.05, 0.05, 0.025])
    gains_p_only = m2.M2PidGains(
        kp=kp_baseline,
        ki=np.zeros(3),
        kd=np.zeros(3),
    )
    gains_pd_zero_kd = m2.M2PidGains(
        kp=kp_baseline,
        ki=np.zeros(3),
        kd=np.zeros(3),
    )

    limits_p = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(0.5)] * 3),
        derivative_enabled=False,
    )
    limits_pd = m2.M2PidLimits(
        residual_limit_rad=np.array([math.radians(0.5)] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=10.0,
    )

    pid_p = m2.M2TaskSpaceAttitudePid(gains=gains_p_only, limits=limits_p)
    pid_pd = m2.M2TaskSpaceAttitudePid(gains=gains_pd_zero_kd, limits=limits_pd)

    # Generate synthetic varying trajectory
    np.random.seed(42)
    for _ in range(50):
        e_cand = np.random.uniform(-0.02, 0.02, size=3)
        dt = np.random.uniform(0.018, 0.022)

        out_p = pid_p.update(e_cand, dt_s=dt)
        out_pd = pid_pd.update(e_cand, dt_s=dt)

        assert np.allclose(out_p, out_pd, atol=1e-15), "Kd=0 must be numerically identical to P-only"
        assert np.allclose(pid_pd.last_d_term, [0.0, 0.0, 0.0], atol=1e-15)


# ==============================================================================
# 7. Residual Saturation Limiting Under Extreme Derivative
# ==============================================================================
def test_residual_limiting_with_extreme_derivative():
    """Verify that even under extreme angular rates, output is strictly clamped to max_residual."""
    limit_deg = 0.5
    limit_rad = math.radians(limit_deg)
    gains = m2.M2PidGains(
        kp=np.array([0.05, 0.05, 0.025]),
        ki=np.array([0.0, 0.0, 0.0]),
        kd=np.array([0.50, 0.50, 0.50]),  # Very high Kd
    )
    limits = m2.M2PidLimits(
        residual_limit_rad=np.array([limit_rad] * 3),
        derivative_enabled=True,
        derivative_lpf_cutoff_hz=20.0,
    )
    pid = m2.M2TaskSpaceAttitudePid(gains=gains, limits=limits)

    dt = 0.020
    # Extreme angular jump: 0 to 1.0 rad (~57.3 deg) in 1 cycle
    pid.update([0.0, 0.0, 0.0], dt_s=dt)
    cmd = pid.update([1.0, -1.0, 1.0], dt_s=dt)

    assert np.all(pid.saturated_axes)
    assert np.allclose(cmd[0], limit_rad, atol=1e-15)
    assert np.allclose(cmd[1], -limit_rad, atol=1e-15)
    assert np.allclose(cmd[2], limit_rad, atol=1e-15)


# ==============================================================================
# 8. Configurable LPF Cutoff Variation
# ==============================================================================
def test_derivative_lpf_cutoff_behavior():
    """Verify that cutoff parameter modulates filter smoothing appropriately."""
    dt = 0.020  # 50 Hz

    # Compare 5 Hz, 10 Hz, 20 Hz, and bypass (0 Hz)
    cutoffs = [5.0, 10.0, 20.0]
    pids = []
    for fc in cutoffs:
        g = m2.M2PidGains(kd=np.array([1.0, 1.0, 1.0]))
        lim = m2.M2PidLimits(derivative_lpf_cutoff_hz=fc)
        p = m2.M2TaskSpaceAttitudePid(gains=g, limits=lim)
        p.update([0.0, 0.0, 0.0], dt_s=dt)
        pids.append(p)

    # Apply same step error: [0.01, 0.0, 0.0]
    # At step 1: raw derivative is 0.01 / 0.02 = 0.5 rad/s.
    # At step 2: raw derivative is 0.
    # Filter state at step 2: y[2] = alpha * y[1]
    # Lower fc -> higher tau -> higher alpha -> higher remnant derivative at step 2
    for p in pids:
        p.update([0.01, 0.0, 0.0], dt_s=dt)

    step2_derivs = []
    for p in pids:
        p.update([0.01, 0.0, 0.0], dt_s=dt)
        step2_derivs.append(p.last_filtered_derivative[0])

    # 5Hz filter should have the highest remnant value (slowest decay), 20Hz should have the lowest (fastest decay)
    assert step2_derivs[0] > step2_derivs[1] > step2_derivs[2]

    # Bypass test: fc <= 0
    lim_bypass = m2.M2PidLimits(derivative_lpf_cutoff_hz=0.0)
    p_bypass = m2.M2TaskSpaceAttitudePid(gains=m2.M2PidGains(kd=np.array([1.0, 1.0, 1.0])), limits=lim_bypass)
    p_bypass.update([0.0, 0.0, 0.0], dt_s=dt)
    p_bypass.update([0.01, 0.0, 0.0], dt_s=dt)
    p_bypass.update([0.01, 0.0, 0.0], dt_s=dt)
    # Raw derivative at step 2 is 0 -> bypass must immediately be 0
    assert np.allclose(p_bypass.last_filtered_derivative, [0.0, 0.0, 0.0], atol=1e-15)
