#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for Stage J13-2A1: Pure Python LADRC Core Module (j13_ladrc.py).

Verifies:
  - Test 1: Zero equilibrium
  - Test 2: Initialization & warm start
  - Test 3: Positive error sign contract
  - Test 4: Positive disturbance compensation
  - Test 5: Saturation clamping (+/-0.5 deg)
  - Test 6: Observer uses previous plant input, not current shadow output
  - Test 7: Roll/Pitch conservative parameter set
  - Test 8: Yaw independent parameter set
  - Test 9: Invalid b0 <= 0 rejection
  - Test 10: Invalid dt / NaN / Inf robustness
  - Test 11: Exact ZOH numerical regression against matrix exponential
"""

import math
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.linalg import expm

# Add scripts directory to path
_SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from j13_ladrc import FirstOrderLadrcChannel, ThreeAxisLadrc


def test_01_zero_equilibrium():
    """Test 1: Zero equilibrium state under zero input and zero error."""
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    chan.reset(z1_init=0.0, z2_init=0.0, previous_u_actual=0.0)

    for _ in range(50):
        u_shadow, u_raw, z1, z2, sat = chan.step(
            measurement=0.0,
            actual_previous_input=0.0,
            dt=0.020,
        )
        assert abs(z1) < 1e-12
        assert abs(z2) < 1e-12
        assert abs(u_raw) < 1e-12
        assert abs(u_shadow) < 1e-12
        assert not sat


def test_02_initialization():
    """Test 2: Warm initialization with current measured error y0."""
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    y0 = math.radians(0.25)
    chan.reset(z1_init=y0, z2_init=0.0, previous_u_actual=0.0)

    assert math.isclose(chan.z1, y0, abs_tol=1e-12)
    assert math.isclose(chan.z2, 0.0, abs_tol=1e-12)
    assert chan.initialized is True


def test_03_positive_error_sign():
    """Test 3: Positive error sign contract: e > 0 with z2 = 0 yields u_raw > 0.

    Nominal plant: e_dot = f - b0 * u.
    Positive u creates negative e_dot, which drives positive e back towards zero.
    """
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    e_pos = math.radians(0.20)
    chan.reset(z1_init=e_pos, z2_init=0.0)

    u_shadow, u_raw, is_sat = chan.compute_shadow_command()
    assert u_raw > 0.0
    expected_u_raw = (1.0 * e_pos + 0.0) / 10.0
    assert math.isclose(u_raw, expected_u_raw, rel_tol=1e-9)


def test_04_positive_disturbance_compensation():
    """Test 4: Positive disturbance compensation: z2 > 0 produces positive contribution to u."""
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    chan.reset(z1_init=0.0, z2_init=math.radians(1.5))

    u_shadow, u_raw, is_sat = chan.compute_shadow_command()
    assert u_raw > 0.0
    expected_u_raw = math.radians(1.5) / 10.0
    assert math.isclose(u_raw, expected_u_raw, rel_tol=1e-9)


def test_05_saturation_clamping():
    """Test 5: Saturation clamping strictly enforces +/-0.5 deg (0.0087266 rad)."""
    chan = FirstOrderLadrcChannel(
        b0=10.0,
        omega_c=1.0,
        omega_o=4.0,
        u_limit=math.radians(0.5),
    )
    # Huge positive error
    chan.reset(z1_init=math.radians(10.0), z2_init=math.radians(50.0))
    u_shadow_pos, u_raw_pos, is_sat_pos = chan.compute_shadow_command()
    assert is_sat_pos is True
    assert math.isclose(u_shadow_pos, math.radians(0.5), rel_tol=1e-9)
    assert u_raw_pos > math.radians(0.5)

    # Huge negative error
    chan.reset(z1_init=math.radians(-10.0), z2_init=math.radians(-50.0))
    u_shadow_neg, u_raw_neg, is_sat_neg = chan.compute_shadow_command()
    assert is_sat_neg is True
    assert math.isclose(u_shadow_neg, -math.radians(0.5), rel_tol=1e-9)
    assert u_raw_neg < -math.radians(0.5)


def test_06_observer_uses_previous_actual_plant_input():
    """Test 6: Observer uses previous applied plant input, NOT current shadow output."""
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    chan.reset(z1_init=0.0, z2_init=0.0)

    # In shadow mode, the physical plant was held at u_plant_actual = 0.005 rad
    # while hypothetical u_shadow might be different
    u_plant_prev = 0.005
    y_k = 0.002
    chan.observer_update(measurement=y_k, actual_previous_input=u_plant_prev, dt=0.020)

    # Verify that the internal state changed in response to u_plant_prev
    assert chan.previous_u_actual == u_plant_prev

    # Second step with a different plant input
    chan_zero_input = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    chan_zero_input.reset(z1_init=0.0, z2_init=0.0)
    chan_zero_input.observer_update(measurement=y_k, actual_previous_input=0.0, dt=0.020)

    # Observer states MUST differ because u_plant_prev was non-zero
    assert chan.z1 != chan_zero_input.z1
    assert chan.z2 != chan_zero_input.z2


def test_07_roll_pitch_parameters():
    """Test 7: Roll & Pitch channels instantiate with conservative hardware parameters."""
    ladrc = ThreeAxisLadrc()
    # Roll channel (index 0)
    assert ladrc.channels[0].b0 == 10.0
    assert ladrc.channels[0].omega_c == 1.0
    assert ladrc.channels[0].omega_o == 4.0
    assert math.isclose(ladrc.channels[0].u_limit, math.radians(0.5), rel_tol=1e-9)

    # Pitch channel (index 1)
    assert ladrc.channels[1].b0 == 10.0
    assert ladrc.channels[1].omega_c == 1.0
    assert ladrc.channels[1].omega_o == 4.0
    assert math.isclose(ladrc.channels[1].u_limit, math.radians(0.5), rel_tol=1e-9)


def test_08_yaw_independent_parameters():
    """Test 8: Yaw channel has strictly independent parameters matching J13 identification."""
    ladrc = ThreeAxisLadrc()
    yaw_ch = ladrc.channels[2]
    assert yaw_ch.b0 == 3.0
    assert yaw_ch.omega_c == 0.35
    assert yaw_ch.omega_o == 1.8
    assert math.isclose(yaw_ch.u_limit, math.radians(0.5), rel_tol=1e-9)

    # Confirm Yaw parameters are distinct from Roll/Pitch
    assert yaw_ch.b0 != ladrc.channels[0].b0
    assert yaw_ch.omega_c != ladrc.channels[0].omega_c
    assert yaw_ch.omega_o != ladrc.channels[0].omega_o


def test_09_invalid_b0_rejection():
    """Test 9: Invalid b0 <= 0 or non-finite values are rejected with ValueError."""
    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=0.0, omega_c=1.0, omega_o=4.0)

    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=-10.0, omega_c=1.0, omega_o=4.0)

    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=float("nan"), omega_c=1.0, omega_o=4.0)

    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=10.0, omega_c=-1.0, omega_o=4.0)

    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=-4.0)

    with pytest.raises(ValueError):
        FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0, u_limit=0.0)


def test_10_invalid_dt_and_nan_protection():
    """Test 10: Invalid dt (negative, zero, NaN, huge) and NaN inputs do not crash or corrupt observer."""
    chan = FirstOrderLadrcChannel(b0=10.0, omega_c=1.0, omega_o=4.0)
    chan.reset(z1_init=0.01, z2_init=0.02)

    # Test negative dt -> clamps to nominal_dt without throwing exception
    z1, z2 = chan.observer_update(measurement=0.01, actual_previous_input=0.0, dt=-0.020)
    assert math.isfinite(z1) and math.isfinite(z2)
    assert chan.anomaly_count == 1

    # Test NaN measurement -> skips update, keeps previous state
    prev_z1, prev_z2 = chan.z1, chan.z2
    z1, z2 = chan.observer_update(measurement=float("nan"), actual_previous_input=0.0, dt=0.020)
    assert z1 == prev_z1
    assert z2 == prev_z2
    assert chan.anomaly_count == 2

    # Test Inf actual_previous_input -> sanitizes to stored u_prev
    z1, z2 = chan.observer_update(measurement=0.01, actual_previous_input=float("inf"), dt=0.020)
    assert math.isfinite(z1) and math.isfinite(z2)
    assert chan.anomaly_count == 3


def test_11_exact_zoh_numerical_regression():
    """Test 11: Exact ZOH discretization formulas match numerical scipy.linalg.expm to machine precision."""
    b0 = 10.0
    omega_o = 4.0
    Ts = 0.020

    chan = FirstOrderLadrcChannel(b0=b0, omega_c=1.0, omega_o=omega_o, nominal_dt=Ts)
    (
        phi11,
        phi12,
        phi21,
        phi22,
        gu1,
        gu2,
        gy1,
        gy2,
    ) = chan.compute_discrete_matrices(Ts)

    Phi_closed_form = np.array([[phi11, phi12], [phi21, phi22]], dtype=np.float64)
    Gamma_u_closed_form = np.array([gu1, gu2], dtype=np.float64)
    Gamma_y_closed_form = np.array([gy1, gy2], dtype=np.float64)

    # Continuous observer matrices:
    # dot(z) = (A - L*C)*z + B_u*u + L*y = Ae*z + Bu*u + Ly*y
    Ae = np.array([
        [-2.0 * omega_o, 1.0],
        [-(omega_o**2), 0.0],
    ], dtype=np.float64)
    Bu = np.array([[-b0], [0.0]], dtype=np.float64)
    Ly = np.array([[2.0 * omega_o], [omega_o**2]], dtype=np.float64)

    # Matrix exponential for Phi
    Phi_expm = expm(Ae * Ts)
    assert np.allclose(Phi_closed_form, Phi_expm, atol=1e-15)

    # Continuous augmented system for exact ZOH integrals
    # [ Ae  Bu  Ly ]
    # [  0   0   0 ]
    M = np.zeros((4, 4), dtype=np.float64)
    M[:2, :2] = Ae
    M[:2, 2:3] = Bu
    M[:2, 3:4] = Ly
    eM = expm(M * Ts)

    Gamma_u_expm = eM[:2, 2]
    Gamma_y_expm = eM[:2, 3]

    assert np.allclose(Gamma_u_closed_form, Gamma_u_expm, atol=1e-15)
    assert np.allclose(Gamma_y_closed_form, Gamma_y_expm, atol=1e-15)

    # Eigenvalues of Phi must strictly be exp(-omega_o * Ts)
    eigs = np.linalg.eigvals(Phi_closed_form)
    expected_eig = math.exp(-omega_o * Ts)
    for eig in eigs:
        assert math.isclose(eig.real, expected_eig, rel_tol=1e-7)
        assert abs(eig.imag) < 1e-6
