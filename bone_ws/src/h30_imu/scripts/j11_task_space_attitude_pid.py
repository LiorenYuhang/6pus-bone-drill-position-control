#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J11: Task-Space Attitude PID Controller Core Module.

Authoritative Baseline:
  - 6PUS World-Attitude Stabilization / Anti-Vibration (J11)
  - J8 Frozen Production Pipeline (Untampered Black Box)
  - J9 Verified Sensing, Coordinate Transforms, and SO(3) Math

Architecture:
  Platform H30 IMU (q_raw, omega_IP)
        ↓
  Establish HOME Relative-Zero q0: q_rel = q0^* ⊗ q_raw
        ↓
  Transform to Platform Mechanical Frame:
    Orientation: q_p_rel = q_PI ⊗ q_rel ⊗ q_PI^*   (q_PI = Rz(+90 deg))
    Angular Vel: omega_P = R_P_IP @ omega_IP       (R_P_IP = Rz(+90 deg))
        ↓
  SO(3) Attitude Tracking Error:
    E_W = R_ref @ R_meas.T
    e_W = Log(E_W)^vee
    e_ref = R_ref.T @ e_W  (In reference platform frame)
        ↓
  Task-Space 3-Axis PID Controller:
    e_R = e_ref
    P_term = Kp * e_R
    I_term = Ki * integral(e_R)
    D_term = -Kd * omega_P  (or Kd * de/dt)
    delta_phi_raw = P_term + I_term + D_term
    delta_phi_cmd = clamp(delta_phi_raw, -residual_limit, +residual_limit)
        ↓
  SO(3) Residual Orientation Composition:
    R_cmd = R_ref @ Exp([delta_phi_cmd]_x)
    q_cmd = rot_matrix_to_quat_xyzw(R_cmd)
        ↓
  /pus6/target_cartesian_pose (50Hz PoseStamped: x=0, y=0, z=z0, q=q_cmd)
        ↓
  [J8 Frozen Production Pipeline: Kinematics -> Shaper -> 50Hz Servo Driver]

HARDWARE SAFETY:
  - Ki = 0, Kd = 0 by default in J11-1 (P-only regression baseline).
  - MATLAB gains (Kp=3000, Ki=400, Kd=18) are for joint force PID and STRICTLY PROHIBITED.
  - hardware_write_count = 0 in J11-1.
  - PID disable enters deterministic inactive state; DOES NOT auto-return to HOME.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

# Canonical R_P_IP transform matrix: Rz(+90 deg)
# v_P = R_P_IP @ v_IP
R_P_IP = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# Canonical Rz(+90 deg) quaternion in [x, y, z, w]
Q_P_IP = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))


# ==============================================================================
# Quaternion & SO(3) Math Utilities (Independent of ROS)
# ==============================================================================

def normalize_quat_xyzw(q: Sequence[float]) -> Optional[Tuple[float, float, float, float]]:
    """Normalizes quaternion [x, y, z, w]. Returns None if norm <= 1e-9 or non-finite."""
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        return None
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-9:
        return None
    return (float(q[0]) / norm, float(q[1]) / norm, float(q[2]) / norm, float(q[3]) / norm)


def quat_conjugate_xyzw(q: Sequence[float]) -> Tuple[float, float, float, float]:
    """Returns quaternion conjugate [ -x, -y, -z, w ]."""
    return (-float(q[0]), -float(q[1]), -float(q[2]), float(q[3]))


def quat_multiply_xyzw(
    a: Sequence[float], b: Sequence[float]
) -> Tuple[float, float, float, float]:
    """Hamilton quaternion multiplication a ⊗ b in [x, y, z, w] convention."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def average_quat_xyzw(
    samples: Sequence[Sequence[float]],
) -> Optional[Tuple[float, float, float, float]]:
    """Computes sample average quaternion with sign disambiguation."""
    normalized = [q for q in (normalize_quat_xyzw(s) for s in samples) if q is not None]
    if not normalized:
        return None
    ref = normalized[0]
    acc = [0.0, 0.0, 0.0, 0.0]
    for q in normalized:
        dot = sum(ref[i] * q[i] for i in range(4))
        sign = -1.0 if dot < 0.0 else 1.0
        for i in range(4):
            acc[i] += sign * q[i]
    return normalize_quat_xyzw(acc)


def relative_quat_xyzw(
    q0_xyzw: Sequence[float], q_xyzw: Sequence[float]
) -> Optional[Tuple[float, float, float, float]]:
    """Computes relative quaternion q_rel = q0^* ⊗ q."""
    q0 = normalize_quat_xyzw(q0_xyzw)
    q = normalize_quat_xyzw(q_xyzw)
    if q0 is None or q is None:
        return None
    return normalize_quat_xyzw(quat_multiply_xyzw(quat_conjugate_xyzw(q0), q))


def quat_to_rpy_deg_xyzw(q: Sequence[float]) -> Tuple[float, float, float]:
    """Extracts extrinsic XYZ (intrinsic ZYX) roll, pitch, yaw in degrees from quaternion [x, y, z, w]."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def euler_deg_to_rot_matrix(r_deg: float, p_deg: float, y_deg: float) -> np.ndarray:
    """Creates rotation matrix R = Rz(y) * Ry(p) * Rx(r) matching Analytical6PusKinematics."""
    return R.from_euler("xyz", [r_deg, p_deg, y_deg], degrees=True).as_matrix()


def rot_matrix_to_euler_deg(rot_mat: np.ndarray) -> np.ndarray:
    """Extracts extrinsic XYZ Euler angles [roll, pitch, yaw] in degrees from 3x3 matrix."""
    return R.from_matrix(rot_mat).as_euler("xyz", degrees=True)


def quat_to_rot_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    """Converts unit quaternion [x, y, z, w] to 3x3 rotation matrix."""
    q_norm = normalize_quat_xyzw(q_xyzw)
    if q_norm is None:
        raise ValueError(f"Invalid quaternion: {q_xyzw}")
    return R.from_quat(q_norm).as_matrix()


def rot_matrix_to_quat_xyzw(rot_mat: np.ndarray) -> Tuple[float, float, float, float]:
    """Converts 3x3 rotation matrix to unit quaternion [x, y, z, w]."""
    q = R.from_matrix(rot_mat).as_quat()
    q_norm = normalize_quat_xyzw(q)
    assert q_norm is not None
    return q_norm


# ==============================================================================
# Gyro Coordinate Frame Transformation
# ==============================================================================

def transform_gyro_ip_to_p(omega_ip: Sequence[float]) -> np.ndarray:
    """
    Transforms angular velocity vector from Platform H30 sensor frame (IP)
    to Platform mechanical frame (P) via R_P_IP = Rz(+90 deg).

    Mapping:
      omega_P = R_P_IP @ omega_IP
      omega_P_x = -omega_IP_y
      omega_P_y =  omega_IP_x
      omega_P_z =  omega_IP_z

    Verification:
      IP +X [1, 0, 0] -> P +Y [0, 1, 0]
      IP +Y [0, 1, 0] -> P -X [-1, 0, 0]
      IP +Z [0, 0, 1] -> P +Z [0, 0, 1]
    """
    if len(omega_ip) != 3:
        raise ValueError(f"Expected 3-element vector, got {len(omega_ip)}")
    v = np.asarray(omega_ip, dtype=np.float64)
    return R_P_IP @ v


# ==============================================================================
# SO(3) Attitude Tracking Error Formulation
# ==============================================================================

def compute_so3_attitude_error(
    R_ref: np.ndarray,
    R_meas: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes frame-consistent SO(3) attitude tracking error.
      E_W = R_ref @ R_meas.T
      e_W = Log(E_W)^vee   (Lie algebra error vector in World frame [rad])
      e_ref = R_ref.T @ e_W (Transported to reference platform body frame [rad])

    Returns:
      (e_W, e_ref) as 3-element numpy arrays in radians.
    """
    E_W_mat = R_ref @ R_meas.T
    rot_err = R.from_matrix(E_W_mat)
    e_w_rad = rot_err.as_rotvec()
    e_ref_rad = R_ref.T @ e_w_rad
    return e_w_rad, e_ref_rad


def compute_so3_attitude_error_quat(
    q_ref_xyzw: Sequence[float],
    q_meas_xyzw: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes frame-consistent SO(3) attitude error directly from quaternions,
    with strict quaternion sign invariance: q and -q represent the exact same
    physical attitude and produce identical error vectors.
    """
    q_ref_norm = normalize_quat_xyzw(q_ref_xyzw)
    q_meas_norm = normalize_quat_xyzw(q_meas_xyzw)
    if q_ref_norm is None or q_meas_norm is None:
        raise ValueError("Invalid quaternion in attitude error computation")

    # Rotation matrix conversion naturally guarantees sign invariance: R(q) == R(-q)
    R_ref = R.from_quat(q_ref_norm).as_matrix()
    R_meas = R.from_quat(q_meas_norm).as_matrix()
    return compute_so3_attitude_error(R_ref, R_meas)


def compose_so3_residual(
    R_ref: np.ndarray,
    delta_phi_ref_rad: Sequence[float],
) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """
    Composes nominal reference attitude with body-frame residual vector:
      R_cmd = R_ref @ Exp([delta_phi_ref]_x)
    Returns:
      (R_cmd_matrix, q_cmd_xyzw)
    """
    res_vec = np.asarray(delta_phi_ref_rad, dtype=np.float64)
    rot_res = R.from_rotvec(res_vec)
    rot_ref = R.from_matrix(R_ref)
    rot_cmd = rot_ref * rot_res
    R_cmd = rot_cmd.as_matrix()
    q_cmd = rot_matrix_to_quat_xyzw(R_cmd)
    return R_cmd, q_cmd


# ==============================================================================
# Task-Space 3-Axis Attitude PID Controller
# ==============================================================================

@dataclass
class PidGains:
    """Independent 3-axis PID gains for Roll, Pitch, Yaw."""
    kp: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    ki: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    @classmethod
    def create(
        cls,
        kp_roll: float = 0.5,
        kp_pitch: float = 0.5,
        kp_yaw: float = 0.5,
        ki_roll: float = 0.0,
        ki_pitch: float = 0.0,
        ki_yaw: float = 0.0,
        kd_roll: float = 0.0,
        kd_pitch: float = 0.0,
        kd_yaw: float = 0.0,
    ) -> PidGains:
        return cls(
            kp=np.array([kp_roll, kp_pitch, kp_yaw], dtype=np.float64),
            ki=np.array([ki_roll, ki_pitch, ki_yaw], dtype=np.float64),
            kd=np.array([kd_roll, kd_pitch, kd_yaw], dtype=np.float64),
        )


@dataclass
class PidLimits:
    """Configurable saturation and anti-windup bounds."""
    # Output residual saturation limit per axis in radians
    residual_limit_rad: np.ndarray = field(
        default_factory=lambda: np.array([math.radians(0.50), math.radians(0.50), math.radians(1.00)], dtype=np.float64)
    )
    # Integrator state clamp limit per axis in radians * s
    integral_limit_rad_s: np.ndarray = field(
        default_factory=lambda: np.array([math.radians(1.0), math.radians(1.0), math.radians(2.0)], dtype=np.float64)
    )
    # Valid dt range protection
    min_dt_s: float = 0.001       # 1 ms (1000 Hz ceiling)
    max_dt_s: float = 0.100       # 100 ms (10 Hz floor; prevents integrator jump on lag)


class TaskSpaceAttitudePid:
    """
    3-Axis Task-Space Attitude PID Controller for 6PUS platform.

    Features:
      - Independent per-axis Kp, Ki, Kd parameters.
      - J11-1 default: Ki=0, Kd=0 (clean P-only baseline).
      - Integrator anti-windup: conditional integration upon output saturation.
      - Robust dt handling: zero/negative dt rejection, excessive dt clamping.
      - Derivative action via platform-frame angular velocity (omega_P) or filtered de/dt.
      - PID disable behavior: zeroes integrator and residual, deterministic inactive state.
        (DOES NOT auto-return to HOME; avoids abrupt motion under tilted base).
    """

    def __init__(
        self,
        gains: Optional[PidGains] = None,
        limits: Optional[PidLimits] = None,
    ) -> None:
        self.gains = gains if gains is not None else PidGains.create()
        self.limits = limits if limits is not None else PidLimits()

        # Controller states
        self.enabled: bool = True
        self.integral: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_error: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_cmd: np.ndarray = np.zeros(3, dtype=np.float64)
        self.has_prev_error: bool = False

        # Diagnostic counters & telemetry cache
        self.step_count: int = 0
        self.saturated_axes: List[bool] = [False, False, False]
        self.last_p_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_i_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_d_term: np.ndarray = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        """Resets all internal integrator, derivative, and command states to zero."""
        self.integral = np.zeros(3, dtype=np.float64)
        self.last_error = np.zeros(3, dtype=np.float64)
        self.last_cmd = np.zeros(3, dtype=np.float64)
        self.has_prev_error = False
        self.saturated_axes = [False, False, False]
        self.last_p_term = np.zeros(3, dtype=np.float64)
        self.last_i_term = np.zeros(3, dtype=np.float64)
        self.last_d_term = np.zeros(3, dtype=np.float64)

    def enable(self) -> None:
        """Enables the PID controller."""
        self.enabled = True

    def disable(self) -> None:
        """
        Disables active compensation.
        Resets integrator and residual to zero.
        Enters deterministic inactive state without issuing movement.
        """
        self.enabled = False
        self.reset()

    def update(
        self,
        e_ref_rad: Sequence[float],
        dt_s: float,
        omega_p_rad_s: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """
        Executes one PID control step.

        Parameters:
          e_ref_rad: Attitude tracking error vector in reference body frame [rad].
          dt_s: Elapsed loop period in seconds.
          omega_p_rad_s: Optional Platform-frame angular velocity [rad/s].
                         If provided, D-term uses -Kd * omega_P (direct gyro damping).
                         If None, D-term uses Kd * (e - prev_e) / dt.

        Returns:
          delta_phi_cmd: Clamped 3-element attitude correction residual [rad].
        """
        if not self.enabled:
            return np.zeros(3, dtype=np.float64)

        e = np.asarray(e_ref_rad, dtype=np.float64)
        if len(e) != 3 or not np.all(np.isfinite(e)):
            # Invalid input error protection
            return self.last_cmd

        # 1. dt Validation and Protection
        if not math.isfinite(dt_s) or dt_s < self.limits.min_dt_s:
            # Invalid or negligible dt: hold current command, skip integrator/derivative updates
            return self.last_cmd

        # Clamp excessive dt to protect against scheduler stall jumps
        effective_dt = min(dt_s, self.limits.max_dt_s)

        # 2. Proportional Term: P = Kp * e
        p_term = self.gains.kp * e

        # 3. Integral Term with Anti-Windup
        # Candidate integration
        integral_candidate = self.integral + e * effective_dt

        # Anti-windup conditional integration:
        # If an axis was saturated in the previous cycle, and the current error has the
        # same sign as the command (pushing further into saturation), freeze integration on that axis.
        for i in range(3):
            if self.saturated_axes[i] and (e[i] * self.last_cmd[i] > 0.0):
                integral_candidate[i] = self.integral[i]

        # Clamp integrator state to bounds
        self.integral = np.clip(
            integral_candidate,
            -self.limits.integral_limit_rad_s,
            self.limits.integral_limit_rad_s,
        )
        i_term = self.gains.ki * self.integral

        # 4. Derivative Term
        if omega_p_rad_s is not None:
            w_p = np.asarray(omega_p_rad_s, dtype=np.float64)
            if len(w_p) == 3 and np.all(np.isfinite(w_p)):
                # D-term from gyro angular velocity damping: D = -Kd * omega_P
                d_term = -self.gains.kd * w_p
            else:
                d_term = np.zeros(3, dtype=np.float64)
        elif self.has_prev_error:
            # Fallback finite-difference de/dt
            de_dt = (e - self.last_error) / effective_dt
            d_term = self.gains.kd * de_dt
        else:
            d_term = np.zeros(3, dtype=np.float64)

        # 5. Raw Output Summation
        raw_output = p_term + i_term + d_term

        # 6. Output Residual Saturation Clamping
        cmd = np.clip(
            raw_output,
            -self.limits.residual_limit_rad,
            self.limits.residual_limit_rad,
        )

        # Update saturation flags for anti-windup in next cycle
        for i in range(3):
            self.saturated_axes[i] = bool(abs(raw_output[i]) > self.limits.residual_limit_rad[i])

        # Update states
        self.last_error = e.copy()
        self.last_cmd = cmd.copy()
        self.has_prev_error = True
        self.step_count += 1

        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term

        return cmd

    def get_diagnostics(self) -> Dict[str, Any]:
        """Returns internal controller diagnostic states."""
        return {
            "enabled": self.enabled,
            "step_count": self.step_count,
            "p_term_deg": np.degrees(self.last_p_term).tolist(),
            "i_term_deg": np.degrees(self.last_i_term).tolist(),
            "d_term_deg": np.degrees(self.last_d_term).tolist(),
            "cmd_residual_deg": np.degrees(self.last_cmd).tolist(),
            "integral_state_deg_s": np.degrees(self.integral).tolist(),
            "saturated_axes": list(self.saturated_axes),
        }
