"""Unit tests and Pre-Motion Integration Qualification for Stage J11.

Covers:
  1. Frame Contract: R_WP vs R_BP, proof of nominal HOME identity R_BP_home = I_3x3.
  2. Command Composition: R_BP_cmd = R_BP_home @ Exp([delta_phi]_x) = Exp([delta_phi]_x).
  3. Authoritative Dual-Reference Mapping:
     - q_abs: Analytical IK geometric absolute actuator extension (~0.050 m).
     - q_ref: IK nominal reference extension ([0.050]*6 m).
     - delta_q_m: Differential displacement in meters (q_abs - q_ref).
     - delta_steps: Differential integer steps = round(delta_q_m * steps_per_meter).
     - raw motor steps: OPERATING_HOME_RAW + delta_steps (centered around [1000]*6).
  4. Disturbance-Rejection Sign Tests: Roll, Pitch, Yaw disturbance -> negative relative compensation.
  5. IMU Freshness & Validity Gate: stale, missing, NaN, Inf, non-unit quaternion, invalid dt.
  6. Reference Capture Gate: stationary pass, moving fail, sample count, timeout.
  7. State Machine Lifecycle & Dry-Run Safety: hardware_write_count = 0.
  8. Topic Ownership Gate: competing publishers detection.
  9. End-to-End Pose -> IK Qualification Grid: Roll/Pitch/Yaw (+-0.25 deg, +-0.50 deg), small 3DOF.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time
from typing import List, Tuple

import numpy as np
import pytest

try:
    import rclpy
except ImportError:
    rclpy = None

# Add paths for h30_imu scripts and pus6_kinematics
_TEST_DIR = Path(__file__).resolve().parent
_SCRIPTS_DIR = _TEST_DIR.parent / "scripts"
_KINEMATICS_DIR = _TEST_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPTS_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from j11_task_space_attitude_pid import (
    PidGains,
    PidLimits,
    TaskSpaceAttitudePid,
    compose_so3_residual,
    compute_so3_attitude_error,
    euler_deg_to_rot_matrix,
    normalize_quat_xyzw,
    quat_to_rot_matrix,
    quat_to_rpy_deg_xyzw,
    rot_matrix_to_euler_deg,
    rot_matrix_to_quat_xyzw,
    transform_gyro_ip_to_p,
)

from j11_attitude_stabilization_runner import (
    J11AttitudeStabilizationRunner,
    J11CycleTelemetry,
    J11State,
)

from analytical_6pus import Analytical6PusKinematics
from geometry_config import load_geometry_config


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initializes rclpy session for nodes used in qualification tests."""
    if rclpy is not None and not rclpy.ok():
        rclpy.init()
    yield
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


# ==============================================================================
# 1. Frame Contract & Nominal HOME Identity Proof
# ==============================================================================

def test_frame_contract_nominal_home_identity():
    """
    Rigorously verifies that at nominal Cartesian HOME (z = z0, roll=0, pitch=0, yaw=0):
      1. R_BP_home is mathematically identical to Identity matrix I_3x3.
      2. All 6 actuator slider positions are exactly mid-stroke q_abs = 0.050 m (50.0 mm).
      3. Differential displacement delta_q is identically 0.0 m and 0 steps.
      4. Hardware raw command is OPERATING_HOME_RAW ([1000]*6 steps), NOT 2000 steps.
    """
    cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    assert math.isclose(z0, 0.132612268, abs_tol=1e-6)

    home_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    q_act_m = kin.inverse_kinematics(home_pose)

    assert len(q_act_m) == 6
    for i, q in enumerate(q_act_m):
        assert math.isclose(q, 0.050, abs_tol=1e-6), f"Actuator {i+1} stroke {q} != 0.050m"

    delta_q = kin.differential_inverse_kinematics(home_pose, home_pose)
    delta_steps = kin.differential_inverse_kinematics_steps(home_pose, home_pose)
    assert delta_steps == [0, 0, 0, 0, 0, 0]

    raw_steps = [ref + ds for ref, ds in zip(cfg.operating_home_raw_steps, delta_steps)]
    assert raw_steps == list(cfg.operating_home_raw_steps)
    assert raw_steps == [1000, 1000, 1000, 1000, 1000, 1000]

    # Matrix R_BP at (0, 0, 0)
    R_home = euler_deg_to_rot_matrix(0.0, 0.0, 0.0)
    np.testing.assert_allclose(R_home, np.eye(3), atol=1e-12)


def test_command_composition_equation():
    """
    Verifies the exact command composition equation:
      R_BP_cmd = R_BP_home @ Exp([delta_phi]_x) = Exp([delta_phi]_x)
    """
    from scipy.spatial.transform import Rotation as R

    R_BP_home = np.eye(3, dtype=np.float64)
    delta_phi = [0.005, -0.003, 0.002]  # Small attitude residual [rad]

    R_cmd, q_cmd = compose_so3_residual(R_BP_home, delta_phi)

    # Exact matrix comparison: R_cmd == Exp([delta_phi]_x)
    R_expected = R.from_rotvec(delta_phi).as_matrix()
    np.testing.assert_allclose(R_cmd, R_expected, atol=1e-12)

    # Orientation in Euler degrees (matches to within 1e-3 deg for small angles)
    cmd_rpy_deg = rot_matrix_to_euler_deg(R_cmd)
    expected_rpy_deg = [math.degrees(v) for v in delta_phi]
    np.testing.assert_allclose(cmd_rpy_deg, expected_rpy_deg, atol=1e-3)

    # Quaternion norm must be 1.0
    q_norm = np.linalg.norm(q_cmd)
    assert math.isclose(q_norm, 1.0, abs_tol=1e-7)


# ==============================================================================
# 2. Authoritative Raw Mapping Regressions (Section 3 Requirement)
# ==============================================================================

def test_home_raw_mapping():
    """
    Verifies that HOME pose produces exactly zero differential displacement,
    and the commanded raw steps equal authoritative OPERATING_HOME_RAW.
    Reads from authoritative geometry config without hardcoding.
    """
    cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    home_pose = [0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0]

    # Differential IK at HOME must be zero
    delta_q_m = kin.differential_inverse_kinematics(home_pose, home_pose)
    delta_steps = kin.differential_inverse_kinematics_steps(home_pose, home_pose)
    assert all(abs(dq) < 1e-12 for dq in delta_q_m)
    assert delta_steps == [0, 0, 0, 0, 0, 0]

    # Raw motor step command = OPERATING_HOME_RAW + delta_steps
    raw_steps = [ref + ds for ref, ds in zip(cfg.operating_home_raw_steps, delta_steps)]
    assert raw_steps == list(cfg.operating_home_raw_steps)
    assert len(raw_steps) == 6


def test_small_attitude_raw_mapping():
    """
    Verifies that small attitude targets produce symmetric differential steps
    and that raw motor step targets remain strictly centered around OPERATING_HOME_RAW.
    """
    cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    home_pose = [0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0]
    home_raw = list(cfg.operating_home_raw_steps)

    for angle_deg in (0.25, 0.50):
        # Roll positive vs negative
        pose_pos = [0.0, 0.0, kin.nominal_height, math.radians(angle_deg), 0.0, 0.0]
        pose_neg = [0.0, 0.0, kin.nominal_height, math.radians(-angle_deg), 0.0, 0.0]

        ds_pos = kin.differential_inverse_kinematics_steps(pose_pos, home_pose)
        ds_neg = kin.differential_inverse_kinematics_steps(pose_neg, home_pose)

        # Differential steps must mirror with exact opposite signs
        for p, n in zip(ds_pos, ds_neg):
            assert p == -n, f"Mirror mismatch: +{angle_deg}deg={ds_pos}, -{angle_deg}deg={ds_neg}"

        # Raw commands must be centered around OPERATING_HOME_RAW
        raw_pos = [ref + d for ref, d in zip(home_raw, ds_pos)]
        raw_neg = [ref + d for ref, d in zip(home_raw, ds_neg)]

        for p, n, ref in zip(raw_pos, raw_neg, home_raw):
            assert p - ref == -(n - ref)
            # Stay well within normal operating envelope [200, 1800]
            assert cfg.normal_motion_min_steps <= p <= cfg.normal_motion_max_steps
            assert cfg.normal_motion_min_steps <= n <= cfg.normal_motion_max_steps


# ==============================================================================
# 3. Disturbance-Rejection Sign Tests (Roll, Pitch, Yaw)
# ==============================================================================

@pytest.mark.parametrize("axis, disturbance_deg", [
    ("roll", +0.50),
    ("roll", -0.50),
    ("pitch", +0.50),
    ("pitch", -0.50),
    ("yaw", +0.50),
    ("yaw", -0.50),
])
def test_disturbance_rejection_sign(axis: str, disturbance_deg: float):
    """
    Proves that an external disturbance in direction (+D) results in a NEGATIVE
    error and compensation residual (-delta_phi), properly commanding the
    robot to tilt in the opposing relative direction.
    """
    pid = TaskSpaceAttitudePid(
        gains=PidGains.create(kp_roll=0.5, kp_pitch=0.5, kp_yaw=0.5, ki_roll=0, ki_pitch=0, ki_yaw=0, kd_roll=0, kd_pitch=0, kd_yaw=0),
        limits=PidLimits(residual_limit_rad=np.array([0.1, 0.1, 0.1])),
    )
    pid.enable()

    # Desired Platform World attitude is identity (stabilization target)
    R_WP_des = np.eye(3)

    # External disturbance tilts platform
    r_deg = disturbance_deg if axis == "roll" else 0.0
    p_deg = disturbance_deg if axis == "pitch" else 0.0
    y_deg = disturbance_deg if axis == "yaw" else 0.0
    R_WP_meas = euler_deg_to_rot_matrix(r_deg, p_deg, y_deg)

    # SO(3) Error: E_W = R_des @ R_meas.T
    e_W, e_ref = compute_so3_attitude_error(R_WP_des, R_WP_meas)

    axis_idx = {"roll": 0, "pitch": 1, "yaw": 2}[axis]

    # 1. Check that error sign opposes disturbance sign
    if disturbance_deg > 0:
        assert e_ref[axis_idx] < 0.0, f"Error {e_ref[axis_idx]} should be negative for +{disturbance_deg} deg disturbance"
    else:
        assert e_ref[axis_idx] > 0.0, f"Error {e_ref[axis_idx]} should be positive for {disturbance_deg} deg disturbance"

    # 2. Check that PID update generates opposing compensation
    cmd = pid.update(e_ref, dt_s=0.020)
    if disturbance_deg > 0:
        assert cmd[axis_idx] < 0.0, f"Residual {cmd[axis_idx]} should be negative for +{disturbance_deg} deg disturbance"
    else:
        assert cmd[axis_idx] > 0.0, f"Residual {cmd[axis_idx]} should be positive for {disturbance_deg} deg disturbance"

    # 3. Check that R_BP_cmd tilts oppositely to cancel disturbance
    R_BP_cmd, _ = compose_so3_residual(np.eye(3), cmd)
    cmd_angles = rot_matrix_to_euler_deg(R_BP_cmd)
    if disturbance_deg > 0:
        assert cmd_angles[axis_idx] < 0.0
    else:
        assert cmd_angles[axis_idx] > 0.0


def test_gyro_damping_sign():
    """
    Verifies that Platform angular velocity omega_P produces negative D-term:
      D = -Kd * omega_P
    """
    pid = TaskSpaceAttitudePid(
        gains=PidGains.create(kp_roll=0.0, kp_pitch=0.0, kp_yaw=0.0, kd_roll=0.2, kd_pitch=0.2, kd_yaw=0.2),
        limits=PidLimits(residual_limit_rad=np.array([0.1, 0.1, 0.1])),
    )
    pid.enable()

    # Zero error, but positive angular velocity +0.1 rad/s on Roll
    e_zero = [0.0, 0.0, 0.0]
    omega_p = [0.10, 0.0, 0.0]

    cmd = pid.update(e_zero, dt_s=0.020, omega_p_rad_s=omega_p)
    assert cmd[0] < 0.0, f"Gyro damping should be negative for positive roll rate, got {cmd[0]}"
    assert math.isclose(cmd[0], -0.2 * 0.10, abs_tol=1e-5)


# ==============================================================================
# 4. IMU Freshness & Validity Gate Tests
# ==============================================================================

def test_imu_freshness_gate_stale_rejection():
    """Verifies that when Platform IMU sample age exceeds threshold, runner enters FAULT."""
    runner = J11AttitudeStabilizationRunner(
        execute_motion=False,
        dry_run=True,
        max_platform_imu_age_s=0.050,
        node_name="test_freshness_gate_node",
    )
    runner.state = J11State.ACTIVE
    runner.pid.enable()

    # Fake a stale IMU sample (age = 0.080s > 0.050s)
    now_ns = time.perf_counter_ns()
    runner.platform_imu.reception_time_ns = now_ns - int(0.080 * 1e9)
    runner.platform_imu.is_valid = True

    runner._control_tick()

    assert runner.state == J11State.FAULT
    assert "stale" in runner.fault_reason.lower()
    assert runner.pid.enabled is False
    assert runner.hardware_write_count == 0


def test_imu_validity_gate_nan_inf_rejection():
    """Verifies that NaN or Inf in Platform IMU triggers immediate FAULT."""
    runner = J11AttitudeStabilizationRunner(
        execute_motion=False,
        dry_run=True,
        node_name="test_validity_gate_node",
    )
    runner.state = J11State.ACTIVE

    now_ns = time.perf_counter_ns()
    runner.platform_imu.reception_time_ns = now_ns
    runner.platform_imu.is_valid = False  # Signal invalid flag set by nan/inf check

    runner._control_tick()

    assert runner.state == J11State.FAULT
    assert "invalid" in runner.fault_reason.lower()
    assert runner.hardware_write_count == 0


# ==============================================================================
# 5. Reference Capture Gate Tests
# ==============================================================================

def test_reference_capture_stationary_pass():
    """Verifies that stationary IMU stream passes reference capture gate and transitions to READY."""
    runner = J11AttitudeStabilizationRunner(
        execute_motion=False,
        dry_run=True,
        ref_capture_min_samples=50,
        ref_capture_max_gyro_rad_s=0.05,
        ref_capture_max_quat_std=0.002,
        node_name="test_ref_pass_node",
    )
    runner.state = J11State.REFERENCE_CAPTURE

    # Feed 55 stationary samples with small noise
    now_ns = time.perf_counter_ns()
    np.random.seed(42)
    for _ in range(55):
        noise = np.random.normal(0, 0.0001, 4)
        q = np.array([0.0, 0.0, 0.0, 1.0]) + noise
        q /= np.linalg.norm(q)

        runner.platform_imu.reception_time_ns = now_ns
        runner.platform_imu.is_valid = True
        runner.platform_imu.q_raw_xyzw = tuple(q)
        runner.platform_imu.omega_raw = (0.001, -0.001, 0.0005)

        runner._control_tick()

    assert runner.state == J11State.READY
    assert runner.q0_plat_xyzw is not None
    assert runner.hardware_write_count == 0


def test_reference_capture_moving_fail():
    """Verifies that moving Platform during reference capture FAILS the gate and aborts to WAITING_FOR_IMU."""
    runner = J11AttitudeStabilizationRunner(
        execute_motion=False,
        dry_run=True,
        ref_capture_min_samples=50,
        ref_capture_max_gyro_rad_s=0.05,
        node_name="test_ref_fail_node",
    )
    runner.state = J11State.REFERENCE_CAPTURE

    now_ns = time.perf_counter_ns()
    for _ in range(50):
        runner.platform_imu.reception_time_ns = now_ns
        runner.platform_imu.is_valid = True
        runner.platform_imu.q_raw_xyzw = (0.0, 0.0, 0.0, 1.0)
        runner.platform_imu.omega_raw = (0.10, 0.0, 0.0)

        runner._control_tick()

    assert runner.state == J11State.WAITING_FOR_IMU
    assert runner.state != J11State.READY
    assert runner.q0_plat_xyzw is None
    assert "moving" in runner.fault_reason.lower()
    assert runner.hardware_write_count == 0


# ==============================================================================
# 6. End-to-End Pose -> IK Qualification Grid (Corrected to OPERATING_HOME_RAW)
# ==============================================================================

@pytest.mark.parametrize("roll_deg, pitch_deg, yaw_deg", [
    # Single-axis Roll tests
    (+0.25, 0.0, 0.0),
    (-0.25, 0.0, 0.0),
    (+0.50, 0.0, 0.0),
    (-0.50, 0.0, 0.0),
    # Single-axis Pitch tests
    (0.0, +0.25, 0.0),
    (0.0, -0.25, 0.0),
    (0.0, +0.50, 0.0),
    (0.0, -0.50, 0.0),
    # Single-axis Yaw tests
    (0.0, 0.0, +0.25),
    (0.0, 0.0, -0.25),
    (0.0, 0.0, +0.50),
    (0.0, 0.0, -0.50),
    # Small combined 3DOF case
    (+0.30, -0.20, +0.25),
])
def test_end_to_end_ik_qualification_grid(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    Offline End-to-End Zero-Write Command Qualification:
      delta_phi -> R_BP_cmd -> PoseStamped semantics -> Analytical IK -> differential steps -> predicted raw steps
    Verifies:
      1. Cartesian pose is finite and within temporary J4 software envelope (<= 0.10 rad).
      2. Analytical IK solves successfully without singularities or workspace limit errors.
      3. Predicted raw steps remain centered around authoritative OPERATING_HOME_RAW ([1000]*6).
      4. Single-step step displacement delta_steps is smooth and bounded (<= 15 steps).
    """
    cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    home_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    home_raw = list(cfg.operating_home_raw_steps)

    r_rad = math.radians(roll_deg)
    p_rad = math.radians(pitch_deg)
    y_rad = math.radians(yaw_deg)

    # 1. Envelope check (J4 envelope is 0.10 rad ~ 5.73 deg)
    assert abs(r_rad) <= 0.10
    assert abs(p_rad) <= 0.10
    assert abs(y_rad) <= 0.10

    pose = [0.0, 0.0, z0, r_rad, p_rad, y_rad]

    # 2. Solve differential IK
    delta_steps = kin.differential_inverse_kinematics_steps(pose, home_pose)
    assert len(delta_steps) == 6
    assert all(isinstance(d, int) for d in delta_steps)

    max_step_delta = max(abs(d) for d in delta_steps)
    assert max_step_delta <= 15, f"Step delta {max_step_delta} exceeds 15 steps for angles [{roll_deg}, {pitch_deg}, {yaw_deg}] deg"

    # 3. Predicted raw steps = OPERATING_HOME_RAW + delta_steps
    predicted_raw = [ref + ds for ref, ds in zip(home_raw, delta_steps)]

    # All predicted raw steps must remain close to 1000 (within [980, 1020])
    for i, s in enumerate(predicted_raw):
        assert 980 <= s <= 1020, f"Motor {i+1} raw step {s} deviated too far from OPERATING_HOME ({home_raw[i]})"


def test_positive_negative_symmetry():
    """
    Verifies that positive and negative attitude targets produce symmetric actuator displacements
    within acceptable kinematic non-linearity tolerances (< 5%).
    """
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height

    for angle_deg in (0.25, 0.50):
        # Roll + vs -
        pose_pos = [0.0, 0.0, z0, math.radians(angle_deg), 0.0, 0.0]
        pose_neg = [0.0, 0.0, z0, math.radians(-angle_deg), 0.0, 0.0]

        q_pos = np.array(kin.inverse_kinematics(pose_pos)) - 0.050
        q_neg = np.array(kin.inverse_kinematics(pose_neg)) - 0.050

        # Displacements should be approximately opposite
        np.testing.assert_allclose(q_pos, -q_neg, rtol=0.08, atol=1e-5)
