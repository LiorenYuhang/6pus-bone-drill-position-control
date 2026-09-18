#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J11-M2: Dual-IMU Task-Space Attitude PID Residual Stabilization Runner.

Architecture:
  Dual-IMU Feedforward + Feedback Task-Space PID:
  1. Base IMU -> confirmed R_B_IB = Rz(+90 deg) -> Delta_R_B = R_WB = R_B0^T R_B(t)
              -> R_BP_FF = Delta_R_B^(-1) = Delta_R_B^T
  2. Platform IMU -> confirmed R_P_IP = Rz(+90 deg) -> Delta_R_P = R_WP = R_P0^T R_P(t)
                  -> Tracking Error e_R_W = Log(R_WP_des @ R_WP^T)^vee  (World frame, rad)
                  -> Task-Space 3-Axis PID -> delta_phi_W (World frame residual, rad)
                  -> Delta_R_PID_W = Exp([delta_phi_W]_x)
  3. Frame Transform to Base Mechanical Frame:
     Delta_R_PID_B = R_WB^T @ Delta_R_PID_W @ R_WB
  4. Authoritative Composition:
     R_BP_cmd = Delta_R_PID_B @ R_BP_FF  (algebraically equivalent to R_BP_FF @ Delta_R_PID_W)
  5. Kinematics & Driver:
     -> /pus6/target_cartesian_pose (50Hz PoseStamped in pus6_base frame)
     -> Frozen J8 IK & Control -> Motors

Hardware Safety:
  - Ki = 0, Kd = 0 by default in initial stages.
  - MATLAB joint force gains (3000, 400, 18) strictly prohibited.
  - Full J10 workspace preserved; no artificial +/-4 deg clamp.
  - hardware_write_count = 0 in dry-run mode.
"""

from __future__ import annotations

import argparse
import csv
import enum
import json
import math
from pathlib import Path
import re
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int32MultiArray
    from rcl_interfaces.msg import Log
except ImportError:
    rclpy = None
    Node = object
    Log = object

import numpy as np
from scipy.spatial.transform import Rotation as R

_SCRIPT_DIR = Path(__file__).resolve().parent
_KINEMATICS_DIR = _SCRIPT_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"

for p in (_SCRIPT_DIR, _KINEMATICS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

try:
    from analytical_6pus import Analytical6PusKinematics
    from geometry_config import load_geometry_config
except ImportError:
    Analytical6PusKinematics = None
    load_geometry_config = None


# ------------------------------------------------------------------------------
# Quaternion & SO(3) Math Utilities (Authoritative Frozen M1 Implementation)
# ------------------------------------------------------------------------------
def normalize_quat(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conjugate(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply(q1: Sequence[float], q2: Sequence[float]) -> Tuple[float, float, float, float]:
    """Hamilton quaternion multiplication q1 ⊗ q2 in [x, y, z, w] convention."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def relative_quat(q0: Sequence[float], q: Sequence[float]) -> Tuple[float, float, float, float]:
    """q_rel = q0^* ⊗ q"""
    return normalize_quat(quat_multiply(quat_conjugate(normalize_quat(q0)), normalize_quat(q)))


def apply_z90_transform(q_rel: Sequence[float]) -> Tuple[float, float, float, float]:
    """Applies R_mech = Rz(+90) @ R(q_rel) @ Rz(-90)"""
    qz_pos = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    qz_neg = (0.0, 0.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    return normalize_quat(quat_multiply(quat_multiply(qz_pos, q_rel), qz_neg))


def quat_to_rpy_deg(q: Sequence[float]) -> Tuple[float, float, float]:
    """Extracts extrinsic XYZ roll, pitch, yaw in degrees from [x, y, z, w]."""
    x, y, z, w = normalize_quat(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_to_rot_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quat(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def rot_matrix_to_quat_xyzw(R_mat: np.ndarray) -> Tuple[float, float, float, float]:
    tr = R_mat[0, 0] + R_mat[1, 1] + R_mat[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R_mat[2, 1] - R_mat[1, 2]) / S
        y = (R_mat[0, 2] - R_mat[2, 0]) / S
        z = (R_mat[1, 0] - R_mat[0, 1]) / S
    elif (R_mat[0, 0] > R_mat[1, 1]) and (R_mat[0, 0] > R_mat[2, 2]):
        S = math.sqrt(1.0 + R_mat[0, 0] - R_mat[1, 1] - R_mat[2, 2]) * 2.0
        w = (R_mat[2, 1] - R_mat[1, 2]) / S
        x = 0.25 * S
        y = (R_mat[0, 1] + R_mat[1, 0]) / S
        z = (R_mat[0, 2] + R_mat[2, 0]) / S
    elif R_mat[1, 1] > R_mat[2, 2]:
        S = math.sqrt(1.0 + R_mat[1, 1] - R_mat[0, 0] - R_mat[2, 2]) * 2.0
        w = (R_mat[0, 2] - R_mat[2, 0]) / S
        x = (R_mat[0, 1] + R_mat[1, 0]) / S
        y = 0.25 * S
        z = (R_mat[1, 2] + R_mat[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R_mat[2, 2] - R_mat[0, 0] - R_mat[1, 1]) * 2.0
        w = (R_mat[1, 0] - R_mat[0, 1]) / S
        x = (R_mat[0, 2] + R_mat[2, 0]) / S
        y = (R_mat[1, 2] + R_mat[2, 1]) / S
        z = 0.25 * S
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def rot_matrix_to_euler_deg(R_mat: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(R_mat[0, 0] * R_mat[0, 0] + R_mat[1, 0] * R_mat[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R_mat[2, 1], R_mat[2, 2])
        y = math.atan2(-R_mat[2, 0], sy)
        z = math.atan2(R_mat[1, 0], R_mat[0, 0])
    else:
        x = math.atan2(-R_mat[1, 2], R_mat[1, 1])
        y = math.atan2(-R_mat[2, 0], sy)
        z = 0.0
    return (math.degrees(x), math.degrees(y), math.degrees(z))


def rot_matrix_angle_deg(R_mat: np.ndarray) -> float:
    """Calculates geodesic SO(3) rotation angle in degrees: theta = acos((tr(R)-1)/2)."""
    tr = float(np.trace(R_mat))
    val = (tr - 1.0) / 2.0
    val = max(-1.0, min(1.0, val))
    return math.degrees(math.acos(val))


def rot_matrix_tilt_deg(R_mat: np.ndarray) -> float:
    """Calculates tilt angle of Platform normal axis relative to initial normal axis."""
    val = max(-1.0, min(1.0, float(R_mat[2, 2])))
    return math.degrees(math.acos(val))


def compute_so3_platform_error(
    R_WP_des: np.ndarray,
    R_WP_meas: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Computes frame-consistent SO(3) attitude tracking error.

    E_W = R_WP_des @ R_WP_meas.T
    e_R_W = Log(E_W)^vee   (Lie algebra error vector in World frame [rad])

    Returns:
      (E_W_mat, e_R_W_rad)
    """
    E_W_mat = R_WP_des @ R_WP_meas.T
    rot_err = R.from_matrix(E_W_mat)
    e_R_W_rad = rot_err.as_rotvec()
    return E_W_mat, e_R_W_rad


def compose_so3_pid_residual(
    R_WB: np.ndarray,
    R_BP_FF: np.ndarray,
    delta_phi_W_rad: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Tuple[float, float, float, float]]:
    """Authoritative SO(3) residual composition law for Dual-IMU platform.

    Derivation:
      1. World residual rotation: Delta_R_PID_W = Exp([delta_phi_W]_x)
      2. Transform to Base mechanical frame:
         Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB
      3. Compose command:
         R_BP_cmd = Delta_R_PID_B @ R_BP_FF
         (Note: exactly equivalent to R_BP_FF @ Delta_R_PID_W)

    Returns:
      (Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw)
    """
    phi_W = np.asarray(delta_phi_W_rad, dtype=np.float64)
    Delta_R_PID_W = R.from_rotvec(phi_W).as_matrix()
    Delta_R_PID_B = R_WB.T @ Delta_R_PID_W @ R_WB
    R_BP_cmd = Delta_R_PID_B @ R_BP_FF
    q_cmd_xyzw = rot_matrix_to_quat_xyzw(R_BP_cmd)
    return Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw


# ------------------------------------------------------------------------------
# Task-Space Attitude PID Controller Core
# ------------------------------------------------------------------------------
@dataclass
class M2PidGains:
    kp: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    ki: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


@dataclass
class M2PidLimits:
    residual_limit_rad: np.ndarray = field(
        default_factory=lambda: np.array(
            [math.radians(1.0), math.radians(1.0), math.radians(2.0)], dtype=np.float64
        )
    )
    integral_limit_rad_s: np.ndarray = field(
        default_factory=lambda: np.array(
            [math.radians(2.0), math.radians(2.0), math.radians(4.0)], dtype=np.float64
        )
    )
    min_dt_s: float = 0.001
    max_dt_s: float = 0.100


class M2TaskSpaceAttitudePid:
    def __init__(
        self,
        gains: Optional[M2PidGains] = None,
        limits: Optional[M2PidLimits] = None,
    ) -> None:
        self.gains = gains if gains is not None else M2PidGains()
        self.limits = limits if limits is not None else M2PidLimits()
        self.enabled: bool = True
        self.integral: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_error: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_cmd: np.ndarray = np.zeros(3, dtype=np.float64)
        self.has_prev_error: bool = False
        self.saturated_axes: List[bool] = [False, False, False]
        self.last_p_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_i_term: np.ndarray = np.zeros(3, dtype=np.float64)
        self.last_d_term: np.ndarray = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        self.integral = np.zeros(3, dtype=np.float64)
        self.last_error = np.zeros(3, dtype=np.float64)
        self.last_cmd = np.zeros(3, dtype=np.float64)
        self.has_prev_error = False
        self.saturated_axes = [False, False, False]
        self.last_p_term = np.zeros(3, dtype=np.float64)
        self.last_i_term = np.zeros(3, dtype=np.float64)
        self.last_d_term = np.zeros(3, dtype=np.float64)

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.reset()

    def update(
        self,
        e_R_W_rad: Sequence[float],
        dt_s: float,
        omega_W_rad_s: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """Computes PID corrective residual delta_phi_W [rad] in World frame."""
        if not self.enabled:
            return np.zeros(3, dtype=np.float64)

        e = np.asarray(e_R_W_rad, dtype=np.float64)
        if len(e) != 3 or not np.all(np.isfinite(e)):
            return self.last_cmd

        if not math.isfinite(dt_s) or dt_s < self.limits.min_dt_s:
            return self.last_cmd

        effective_dt = min(dt_s, self.limits.max_dt_s)

        # Proportional term
        p_term = self.gains.kp * e

        # Integral term with anti-windup clamping
        integral_cand = self.integral + e * effective_dt
        for i in range(3):
            if self.saturated_axes[i] and (e[i] * self.last_cmd[i] > 0.0):
                integral_cand[i] = self.integral[i]
        self.integral = np.clip(
            integral_cand,
            -self.limits.integral_limit_rad_s,
            self.limits.integral_limit_rad_s,
        )
        i_term = self.gains.ki * self.integral

        # Derivative term
        if omega_W_rad_s is not None:
            w_w = np.asarray(omega_W_rad_s, dtype=np.float64)
            d_term = -self.gains.kd * w_w if np.all(np.isfinite(w_w)) else np.zeros(3, dtype=np.float64)
        elif self.has_prev_error:
            de_dt = (e - self.last_error) / effective_dt
            d_term = self.gains.kd * de_dt
        else:
            d_term = np.zeros(3, dtype=np.float64)

        raw_output = p_term + i_term + d_term
        cmd = np.clip(raw_output, -self.limits.residual_limit_rad, self.limits.residual_limit_rad)

        for i in range(3):
            self.saturated_axes[i] = bool(abs(raw_output[i]) > self.limits.residual_limit_rad[i])

        self.last_error = e.copy()
        self.last_cmd = cmd.copy()
        self.has_prev_error = True
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_d_term = d_term

        return cmd


# ------------------------------------------------------------------------------
# State Machine & Unified Telemetry Schema (Section 10 Compliance)
# ------------------------------------------------------------------------------
class J11M2State(enum.Enum):
    DISARMED = "DISARMED"
    WAITING_FOR_PREFLIGHT = "WAITING_FOR_PREFLIGHT"
    WAITING_FOR_TRIGGER = "WAITING_FOR_TRIGGER"
    CAPTURING_REFERENCE = "CAPTURING_REFERENCE"
    READY = "READY"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"


@dataclass
class J11M2Telemetry:
    """Authoritative Unified Telemetry Schema for Stage J11-M2."""
    timestamp_ns: int = 0
    cycle_index: int = 0
    dt_ms: float = 20.0
    state: str = J11M2State.DISARMED.value
    controller_mode: str = "pid"

    # Base IMU: R_WB / quaternion / relative SO3
    base_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    base_rel_roll_deg: float = 0.0
    base_rel_pitch_deg: float = 0.0
    base_rel_yaw_deg: float = 0.0
    base_so3_magnitude_deg: float = 0.0
    R_WB: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())

    # Platform IMU: R_WP / quaternion / relative SO3
    platform_rel_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    platform_rel_roll_deg: float = 0.0
    platform_rel_pitch_deg: float = 0.0
    platform_rel_yaw_deg: float = 0.0
    platform_so3_error_deg: float = 0.0
    platform_tilt_error_deg: float = 0.0
    R_WP: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())

    # Reference: R_WP0
    R_WP0: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    q0_base_raw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    q0_plat_raw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    # Feedforward: R_BP_FF
    R_BP_FF: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    ff_roll_deg: float = 0.0
    ff_pitch_deg: float = 0.0
    ff_yaw_deg: float = 0.0

    # Feedback: e_R [rad] & [deg] (World frame)
    e_R_W_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    e_R_W_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # PID: P[3], I[3], D[3], total residual[3]
    pid_p_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_i_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_d_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_total_residual_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_total_residual_deg: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pid_saturated_axes: List[bool] = field(default_factory=lambda: [False, False, False])

    # Command: Delta_R_PID & R_BP_cmd
    Delta_R_PID_W: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    Delta_R_PID_B: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    R_BP_cmd: List[List[float]] = field(default_factory=lambda: np.eye(3).tolist())
    cmd_roll_deg: float = 0.0
    cmd_pitch_deg: float = 0.0
    cmd_yaw_deg: float = 0.0
    cmd_quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    # Actuator
    predicted_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    commanded_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    feedback_steps: List[int] = field(default_factory=lambda: [1000] * 6)
    command_feedback_errors: List[int] = field(default_factory=lambda: [0] * 6)
    joint_currents_raw_mA: List[int] = field(default_factory=lambda: [0] * 6)
    faults: List[int] = field(default_factory=lambda: [0] * 6)

    # Runtime Safety & Timing
    base_imu_age_ms: float = 0.0
    platform_imu_age_ms: float = 0.0
    driver_feedback_age_ms: float = 0.0
    warning_hold: bool = False
    hard_stop: bool = False
    hardware_write_issued: bool = False
    fault_reason: str = ""


# ------------------------------------------------------------------------------
# J11-M2 PID Residual Runner Node
# ------------------------------------------------------------------------------
class J11M2PidResidualRunner(Node if Node is not object else object):
    def __init__(
        self,
        execute_motion: bool = False,
        dry_run: bool = True,
        output_dir: str = "results/j11/m2_pid",
        auto_start: bool = True,
        kp_roll: float = 0.0,
        kp_pitch: float = 0.0,
        kp_yaw: float = 0.0,
        ki_roll: float = 0.0,
        ki_pitch: float = 0.0,
        ki_yaw: float = 0.0,
        kd_roll: float = 0.0,
        kd_pitch: float = 0.0,
        kd_yaw: float = 0.0,
        max_residual_roll_deg: float = 0.5,
        max_residual_pitch_deg: float = 0.5,
        max_residual_yaw_deg: float = 0.5,
    ):
        if Node is not object:
            super().__init__("j11_m2_pid_residual_runner")

        self.execute_motion = execute_motion
        self.dry_run = dry_run
        self.output_dir = Path(output_dir)
        self.auto_start = auto_start
        self.hardware_write_count = 0

        # Kinematics & Config
        if load_geometry_config is not None and Analytical6PusKinematics is not None:
            self.geom_cfg = load_geometry_config()
            self.kin = Analytical6PusKinematics()
            self.z0 = self.kin.nominal_height
            self.operating_home_steps = list(self.geom_cfg.operating_home_raw_steps)
        else:
            self.geom_cfg = None
            self.kin = None
            self.z0 = 0.132612268
            self.operating_home_steps = [1000] * 6

        # State machine
        self.state = J11M2State.DISARMED
        self.fault_reason = ""
        self.cycle_index = 0
        self.active_cycle_count = 0
        self._manual_prompt_emitted = False

        # Controller instantiation
        gains = M2PidGains(
            kp=np.array([kp_roll, kp_pitch, kp_yaw], dtype=np.float64),
            ki=np.array([ki_roll, ki_pitch, ki_yaw], dtype=np.float64),
            kd=np.array([kd_roll, kd_pitch, kd_yaw], dtype=np.float64),
        )
        limits = M2PidLimits(
            residual_limit_rad=np.array([
                math.radians(max_residual_roll_deg),
                math.radians(max_residual_pitch_deg),
                math.radians(max_residual_yaw_deg),
            ], dtype=np.float64)
        )
        self.pid = M2TaskSpaceAttitudePid(gains=gains, limits=limits)

        # Reference quaternions
        self.q0_base_raw: Optional[Tuple[float, float, float, float]] = None
        self.q0_plat_raw: Optional[Tuple[float, float, float, float]] = None
        self.ref_capture_samples_base: List[Tuple[float, float, float, float]] = []
        self.ref_capture_samples_plat: List[Tuple[float, float, float, float]] = []

        # Latest IMU messages
        self.latest_base_q_raw: Optional[Tuple[float, float, float, float]] = None
        self.latest_base_rx_time_ns: int = 0
        self.latest_plat_q_raw: Optional[Tuple[float, float, float, float]] = None
        self.latest_plat_rx_time_ns: int = 0

        # Motor telemetry buffers & Driver monitoring
        self.latest_cmd_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_steps: List[int] = list(self.operating_home_steps)
        self.latest_fb_rx_time_ns: int = 0
        self.latest_fault_codes: List[int] = [0] * 6
        self.latest_fb_currents: List[int] = [0] * 6
        self.has_received_real_feedback: bool = False
        self.driver_stop_latched: bool = False
        self.driver_fault_msg: str = ""

        # Storage
        self.telemetry_history: List[J11M2Telemetry] = []
        self.node_init_time_sec = time.time()
        self.last_tick_time_ns: Optional[int] = None

        # Marker file cleanup
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_driver_stopped", "/tmp/j11_start_signal"]:
            p = Path(marker)
            if p.exists():
                try:
                    p.unlink()
                except OSError:
                    pass

        # ROS 2 Interfaces
        self.target_pose_pub = None
        if Node is not object:
            qos_reliable = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            qos_sensor = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            qos_rosout = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.target_pose_pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", qos_reliable)
            self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_sensor)
            self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_steps_raw", self._on_fb_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_faults_raw", self._on_fb_faults, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_currents_raw", self._on_fb_currents, qos_sensor)
            self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)
            self.timer = self.create_timer(0.020, self._on_control_tick)

    def _on_base_imu(self, msg: Imu) -> None:
        self.latest_base_q_raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.latest_base_rx_time_ns = time.perf_counter_ns()

    def _on_plat_imu(self, msg: Imu) -> None:
        self.latest_plat_q_raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.latest_plat_rx_time_ns = time.perf_counter_ns()

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_cmd_steps = list(msg.data[:6])

    def _on_fb_steps(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fb_steps = list(msg.data[:6])
            self.latest_fb_rx_time_ns = time.perf_counter_ns()
            self.has_received_real_feedback = True

    def _on_fb_faults(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fault_codes = list(msg.data[:6])

    def _on_fb_currents(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_fb_currents = list(msg.data[:6])

    def _on_rosout(self, msg: Log) -> None:
        if "pus6_motor_driver" in msg.name and msg.level >= 40:
            msg_time_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if msg_time_sec > 0 and msg_time_sec < self.node_init_time_sec:
                return
            text = msg.msg
            if any(k in text for k in ["STOP latched", "scheduler_deadline_exceeded", "overcurrent", "Timeout recovery target rejected", "reconnect_max_jump_steps"]):
                self.driver_stop_latched = True
                self.driver_fault_msg = text
                self._enter_fault(f"Driver fatal latch detected via /rosout: {text}")

    def _enter_fault(self, reason: str) -> None:
        if self.state != J11M2State.FAULT:
            self.state = J11M2State.FAULT
            self.fault_reason = str(reason)
            self.pid.disable()
            print(f"\n[J11-M2 FAULT / STOP DETECTED] {self.fault_reason}\n", flush=True)
            for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start"]:
                try:
                    Path(marker).unlink(missing_ok=True)
                except OSError:
                    pass
            try:
                Path("/tmp/j11_driver_stopped").touch()
            except OSError:
                pass

    def _on_control_tick(self) -> None:
        now_ns = time.perf_counter_ns()
        dt_s = (now_ns - self.last_tick_time_ns) * 1e-9 if self.last_tick_time_ns is not None else 0.020
        self.last_tick_time_ns = now_ns
        self.cycle_index += 1

        telem = J11M2Telemetry(
            timestamp_ns=now_ns,
            cycle_index=self.cycle_index,
            dt_ms=dt_s * 1000.0,
            controller_mode="pid",
        )

        base_age_ms = (now_ns - self.latest_base_rx_time_ns) * 1e-6 if self.latest_base_rx_time_ns > 0 else 1e9
        plat_age_ms = (now_ns - self.latest_plat_rx_time_ns) * 1e-6 if self.latest_plat_rx_time_ns > 0 else 1e9
        fb_age_ms = (now_ns - self.latest_fb_rx_time_ns) * 1e-6 if self.latest_fb_rx_time_ns > 0 else 1e9

        telem.base_imu_age_ms = base_age_ms
        telem.platform_imu_age_ms = plat_age_ms
        telem.driver_feedback_age_ms = fb_age_ms

        telem.commanded_steps = list(self.latest_cmd_steps)
        telem.feedback_steps = list(self.latest_fb_steps)
        telem.command_feedback_errors = [c - f for c, f in zip(telem.commanded_steps, telem.feedback_steps)]
        telem.faults = list(self.latest_fault_codes)
        telem.joint_currents_raw_mA = list(self.latest_fb_currents)

        base_q_raw = self.latest_base_q_raw
        plat_q_raw = self.latest_plat_q_raw

        # State Machine Transitions
        if self.state == J11M2State.DISARMED:
            self.state = J11M2State.WAITING_FOR_PREFLIGHT

        elif self.state == J11M2State.WAITING_FOR_PREFLIGHT:
            imu_ok = (base_q_raw is not None and plat_q_raw is not None and base_age_ms < 100.0 and plat_age_ms < 100.0)
            if self.dry_run or not self.execute_motion:
                motors_ok = True
            else:
                motors_ok = (self.has_received_real_feedback and fb_age_ms < 150.0)
                if motors_ok:
                    errs = [abs(s - 1000) for s in self.latest_fb_steps]
                    if any(e > 15 for e in errs):
                        self._enter_fault(f"Preflight FAIL: Motors not at OPERATING_HOME [1000]*6 (actual={self.latest_fb_steps}, max err={max(errs)} steps)")
                        return

            if imu_ok and motors_ok:
                print("\n" + "=" * 70)
                print("[J11-M2] Preflight check PASSED: Dual IMU healthy (<100ms), 6 axes verified at OPERATING_HOME.")
                if self.auto_start:
                    print("[J11-M2] auto_start enabled. Initiating 1.0s stationary reference capture at OPERATING_HOME...")
                    print("=" * 70 + "\n", flush=True)
                    self.ref_capture_samples_base = []
                    self.ref_capture_samples_plat = []
                    self.state = J11M2State.CAPTURING_REFERENCE
                else:
                    print("[J11-M2] Waiting for start trigger (/tmp/j11_start_signal) to capture reference and engage ACTIVE...")
                    print("=" * 70 + "\n", flush=True)
                    try:
                        Path("/tmp/j11_ready_for_start").touch()
                    except OSError:
                        pass
                    self.state = J11M2State.WAITING_FOR_TRIGGER

        elif self.state == J11M2State.WAITING_FOR_TRIGGER:
            trigger_file = Path("/tmp/j11_start_signal")
            if trigger_file.exists():
                try:
                    trigger_file.unlink()
                except OSError:
                    pass
                print("\n[J11-M2] Trigger received! Initiating 1.0s stationary reference capture at OPERATING_HOME...\n", flush=True)
                self.ref_capture_samples_base = []
                self.ref_capture_samples_plat = []
                self.state = J11M2State.CAPTURING_REFERENCE

        elif self.state == J11M2State.CAPTURING_REFERENCE:
            if base_q_raw is None or plat_q_raw is None:
                return
            self.ref_capture_samples_base.append(base_q_raw)
            self.ref_capture_samples_plat.append(plat_q_raw)
            if len(self.ref_capture_samples_base) >= 50:
                self.q0_base_raw = self.ref_capture_samples_base[-1]
                self.q0_plat_raw = self.ref_capture_samples_plat[-1]
                self.state = J11M2State.READY

        elif self.state == J11M2State.READY:
            if not self.dry_run and self.execute_motion:
                if hasattr(self, "get_publishers_info_by_topic"):
                    pubs = self.get_publishers_info_by_topic("/pus6/target_cartesian_pose")
                    pub_nodes = [f"{p.node_namespace}/{p.node_name}".replace("//", "/") for p in pubs]
                    my_node_name = self.get_fully_qualified_name()
                    competing = [n for n in pub_nodes if n != my_node_name]
                    if competing:
                        self._enter_fault(f"Competing publisher on /pus6/target_cartesian_pose: {competing}")
                        return
            self.pid.reset()
            self.pid.enable()
            self.state = J11M2State.ACTIVE
            try:
                Path("/tmp/j11_active_signaled").touch()
            except OSError:
                pass
            print("\n[J11-M2] State transition: READY -> ACTIVE. Dual-IMU Feedforward + Task-Space PID engaged.\n", flush=True)

        elif self.state == J11M2State.ACTIVE:
            self.active_cycle_count += 1
            if base_age_ms > 100.0:
                self._enter_fault("Base IMU age timeout (>100ms)")
                return
            if plat_age_ms > 100.0:
                self._enter_fault("Platform IMU age timeout (>100ms)")
                return
            if self.driver_stop_latched:
                self._enter_fault(f"Driver STOP latched: {self.driver_fault_msg}")
                return
            if not self.dry_run and self.execute_motion:
                if fb_age_ms > 1500.0:
                    self._enter_fault(f"Motor driver feedback stalled: fb_age={fb_age_ms:.1f}ms > 1500ms")
                    return
            if any(fc != 0 for fc in self.latest_fault_codes):
                self._enter_fault(f"Motor driver reported hardware fault: {self.latest_fault_codes}")
                return

        telem.state = self.state.value
        telem.fault_reason = self.fault_reason

        # Computation of Feedforward + Feedback PID
        if (
            self.q0_base_raw is not None
            and base_q_raw is not None
            and self.q0_plat_raw is not None
            and plat_q_raw is not None
            and self.state in (J11M2State.READY, J11M2State.ACTIVE)
        ):
            telem.q0_base_raw = self.q0_base_raw
            telem.q0_plat_raw = self.q0_plat_raw
            telem.R_WP0 = np.eye(3).tolist()

            # 1. Base IMU -> R_WB
            q_rel_base = relative_quat(self.q0_base_raw, base_q_raw)
            q_base_mech = apply_z90_transform(q_rel_base)
            r_b, p_b, y_b = quat_to_rpy_deg(q_base_mech)
            R_WB = quat_to_rot_matrix(q_base_mech)

            telem.base_rel_roll_deg = r_b
            telem.base_rel_pitch_deg = p_b
            telem.base_rel_yaw_deg = y_b
            telem.base_rel_quat_xyzw = q_base_mech
            telem.base_so3_magnitude_deg = rot_matrix_angle_deg(R_WB)
            telem.R_WB = R_WB.tolist()

            # 2. Feedforward command: R_BP_FF = R_WB^T
            R_BP_FF = R_WB.T
            ff_r, ff_p, ff_y = rot_matrix_to_euler_deg(R_BP_FF)
            telem.R_BP_FF = R_BP_FF.tolist()
            telem.ff_roll_deg = ff_r
            telem.ff_pitch_deg = ff_p
            telem.ff_yaw_deg = ff_y

            # 3. Platform IMU -> R_WP
            q_rel_plat = relative_quat(self.q0_plat_raw, plat_q_raw)
            q_plat_mech = apply_z90_transform(q_rel_plat)
            r_p, p_p, y_p = quat_to_rpy_deg(q_plat_mech)
            R_WP = quat_to_rot_matrix(q_plat_mech)

            telem.platform_rel_roll_deg = r_p
            telem.platform_rel_pitch_deg = p_p
            telem.platform_rel_yaw_deg = y_p
            telem.platform_rel_quat_xyzw = q_plat_mech
            telem.platform_so3_error_deg = rot_matrix_angle_deg(R_WP)
            telem.platform_tilt_error_deg = rot_matrix_tilt_deg(R_WP)
            telem.R_WP = R_WP.tolist()

            # 4. SO(3) Platform World Error
            R_WP_des = np.eye(3, dtype=np.float64)
            _, e_R_W_rad = compute_so3_platform_error(R_WP_des, R_WP)
            telem.e_R_W_rad = e_R_W_rad.tolist()
            telem.e_R_W_deg = np.degrees(e_R_W_rad).tolist()

            # 5. Task-Space Attitude PID Update
            delta_phi_W = self.pid.update(e_R_W_rad, dt_s)
            telem.pid_p_rad = self.pid.last_p_term.tolist()
            telem.pid_i_rad = self.pid.last_i_term.tolist()
            telem.pid_d_rad = self.pid.last_d_term.tolist()
            telem.pid_total_residual_rad = delta_phi_W.tolist()
            telem.pid_total_residual_deg = np.degrees(delta_phi_W).tolist()
            telem.pid_saturated_axes = list(self.pid.saturated_axes)

            # 6. Authoritative Residual Composition
            Delta_R_PID_W, Delta_R_PID_B, R_BP_cmd, q_cmd_xyzw = compose_so3_pid_residual(
                R_WB, R_BP_FF, delta_phi_W
            )
            cmd_r, cmd_p, cmd_y = rot_matrix_to_euler_deg(R_BP_cmd)

            telem.Delta_R_PID_W = Delta_R_PID_W.tolist()
            telem.Delta_R_PID_B = Delta_R_PID_B.tolist()
            telem.R_BP_cmd = R_BP_cmd.tolist()
            telem.cmd_roll_deg = cmd_r
            telem.cmd_pitch_deg = cmd_p
            telem.cmd_yaw_deg = cmd_y
            telem.cmd_quat_xyzw = q_cmd_xyzw

            # 7. Predict IK steps & Safety Evaluation
            if self.kin is not None:
                cart_pose = [
                    0.0, 0.0, self.z0,
                    math.radians(cmd_r),
                    math.radians(cmd_p),
                    math.radians(cmd_y),
                ]
                home_pose = [0.0, 0.0, self.z0, 0.0, 0.0, 0.0]
                try:
                    delta_steps = self.kin.differential_inverse_kinematics_steps(cart_pose, home_pose)
                    pred_steps = [ref + ds for ref, ds in zip(self.operating_home_steps, delta_steps)]
                    telem.predicted_steps = pred_steps

                    if any(s < 0 or s > 2000 for s in pred_steps):
                        self._enter_fault(f"Actuator predicted step out of [0, 2000] limit: {pred_steps}")
                        return
                except Exception as exc:
                    self._enter_fault(f"IK computation failed: {exc}")
                    return

            # 8. Hardware Dispatch Sink
            if self.execute_motion and not self.dry_run and self.state == J11M2State.ACTIVE:
                self._dispatch_cartesian_target(telem)
                telem.hardware_write_issued = True
            else:
                telem.hardware_write_issued = False

            # Terminal guidance
            if self.state == J11M2State.ACTIVE:
                if not self._manual_prompt_emitted and self.active_cycle_count >= 50:
                    self._manual_prompt_emitted = True
                    print("\n" + "=" * 70, flush=True)
                    print("[J11-M2] Dual-IMU Feedforward + PID ACTIVE", flush=True)
                    print("=" * 70 + "\n", flush=True)
                elif self._manual_prompt_emitted and self.active_cycle_count % 25 == 0:
                    print(
                        f"[M2 ACTIVE t={self.active_cycle_count*0.02:.1f}s] "
                        f"Base SO3: {telem.base_so3_magnitude_deg:.2f}° | "
                        f"Plat SO3 Err: {telem.platform_so3_error_deg:.2f}° "
                        f"(R:{telem.platform_rel_roll_deg:+.2f}°, P:{telem.platform_rel_pitch_deg:+.2f}°) | "
                        f"PID Res: (R:{telem.pid_total_residual_deg[0]:+.2f}°, P:{telem.pid_total_residual_deg[1]:+.2f}°) | "
                        f"FB: {telem.feedback_steps}",
                        flush=True,
                    )

        self.telemetry_history.append(telem)

    def _dispatch_cartesian_target(self, telem: J11M2Telemetry) -> None:
        if self.target_pose_pub is None:
            return
        msg = PoseStamped()
        if hasattr(self, "get_clock"):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = self.z0
        msg.pose.orientation.x = float(telem.cmd_quat_xyzw[0])
        msg.pose.orientation.y = float(telem.cmd_quat_xyzw[1])
        msg.pose.orientation.z = float(telem.cmd_quat_xyzw[2])
        msg.pose.orientation.w = float(telem.cmd_quat_xyzw[3])

        self.target_pose_pub.publish(msg)
        self.hardware_write_count += 1


def run_m2_session(
    execute_motion: bool = False,
    dry_run: bool = True,
    duration_s: float = 10.0,
    output_dir: str = "results/j11/m2_pid",
    stage: str = "j11_m2_0_audit",
    auto_start: bool = True,
    kp_roll: float = 0.0,
    kp_pitch: float = 0.0,
    kp_yaw: float = 0.0,
    max_residual_roll_deg: float = 1.0,
) -> dict:
    mode_str = "POWERED REAL EXECUTION" if (execute_motion and not dry_run) else "DRY-RUN (hardware_write_count = 0)"
    print(f"\n[{stage.upper()}] Starting J11-M2 Dual-IMU PID Runner in {mode_str}...")

    if rclpy is not None:
        rclpy.init()
    runner = J11M2PidResidualRunner(
        execute_motion=execute_motion,
        dry_run=dry_run,
        output_dir=output_dir,
        auto_start=auto_start,
        kp_roll=kp_roll,
        kp_pitch=kp_pitch,
        kp_yaw=kp_yaw,
        max_residual_roll_deg=max_residual_roll_deg,
    )

    t_start = time.monotonic()
    spin_thread = None
    if rclpy is not None:
        spin_thread = threading.Thread(target=rclpy.spin, args=(runner,), daemon=True)
        spin_thread.start()

    stop_file = Path("/tmp/j11_stop_requested")
    if stop_file.exists():
        try:
            stop_file.unlink()
        except OSError:
            pass

    try:
        while time.monotonic() - t_start < duration_s:
            if stop_file.exists():
                print(f"\n[{stage.upper()}] Stop requested via /tmp/j11_stop_requested.")
                break
            if runner.state == J11M2State.FAULT:
                print(f"\n[{stage.upper()}] Terminating early due to FAULT: {runner.fault_reason}")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f"\n[{stage.upper()}] Interrupted by user.")
    finally:
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start"]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass

    out_p = Path(output_dir)
    out_p.mkdir(parents=True, exist_ok=True)
    csv_file = out_p / (f"{stage}_telemetry_powered.csv" if (execute_motion and not dry_run) else f"{stage}_dry_run_telemetry.csv")
    summary_file = out_p / (f"{stage}_summary_powered.json" if (execute_motion and not dry_run) else f"{stage}_dry_run_summary.json")

    # Save Unified Telemetry CSV
    if runner.telemetry_history:
        keys = list(asdict(runner.telemetry_history[0]).keys())
        with open(csv_file, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for rec in runner.telemetry_history:
                d = asdict(rec)
                for k, v in d.items():
                    if isinstance(v, (list, tuple)):
                        d[k] = json.dumps(v)
                w.writerow(d)
        print(f"[{stage.upper()}] Unified Telemetry CSV saved to: {csv_file} ({len(runner.telemetry_history)} rows)")

    summary = {
        "stage": stage,
        "mode": mode_str,
        "hardware_write_count": runner.hardware_write_count,
        "total_cycles": len(runner.telemetry_history),
        "final_state": runner.state.value,
        "fault_reason": runner.fault_reason,
    }
    with open(summary_file, "w") as f:
        json.dump(summary, f, indent=2)

    return summary


def main():
    parser = argparse.ArgumentParser(description="J11-M2: Dual-IMU Task-Space Attitude PID Residual Runner")
    parser.add_argument("--execute-motion", action="store_true", default=False)
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output-dir", default="results/j11/m2_pid")
    parser.add_argument("--stage", default="j11_m2_0_audit")
    parser.add_argument("--auto-start", action="store_true", default=True)
    parser.add_argument("--no-auto-start", action="store_true", default=False)
    parser.add_argument("--kp-roll", type=float, default=0.0)
    parser.add_argument("--kp-pitch", type=float, default=0.0)
    parser.add_argument("--kp-yaw", type=float, default=0.0)
    parser.add_argument("--max-residual-roll-deg", type=float, default=0.5)
    args = parser.parse_args()

    if not args.execute_motion:
        dry_run_effective = True
    else:
        dry_run_effective = False if not args.dry_run else True

    auto_start = False if args.no_auto_start else True

    run_m2_session(
        execute_motion=args.execute_motion,
        dry_run=dry_run_effective,
        duration_s=args.duration,
        output_dir=args.output_dir,
        stage=args.stage,
        auto_start=auto_start,
        kp_roll=args.kp_roll,
        kp_pitch=args.kp_pitch,
        kp_yaw=args.kp_yaw,
        max_residual_roll_deg=args.max_residual_roll_deg,
    )


if __name__ == "__main__":
    main()
