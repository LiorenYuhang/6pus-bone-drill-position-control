#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J10-A1 single-candidate workspace measurement runner.

This is an experiment-layer runner only. It reuses the frozen J8/J9 motion
path when motion execution is enabled, but its default mode is dry preflight.
No production package is modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"]:
    p = WS_SRC / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int32MultiArray
    from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
except Exception:  # pragma: no cover - import errors are surfaced in tests
    rclpy = None  # type: ignore
    PoseStamped = object  # type: ignore
    MultiThreadedExecutor = object  # type: ignore
    Node = object  # type: ignore
    qos_profile_sensor_data = object()  # type: ignore
    Imu = object  # type: ignore
    Int32MultiArray = object  # type: ignore
    FkStateStamped = object  # type: ignore
    TimedJointFeedback = object  # type: ignore

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.safety import HOME_MEASUREMENT_BAND_LOWER, HOME_MEASUREMENT_BAND_UPPER

G_STANDARD = 9.80665
DEFAULT_OUTPUT_DIR = Path("/home/lyapunov/bone_drill/results/J10/workspace_measurement")
TASK_GATE_MAX_XY_M = 0.015
TASK_GATE_MAX_Z_DELTA_M = 0.020
TASK_GATE_MAX_ANGLE_RAD = 0.10
# The J8 path's observed closed-loop feedback travel rate is approximately
# 60 steps/s on the large X+ moves.  Experiment-layer timeouts are derived
# from the actual multi-axis step distance using this rate plus a bounded
# settling margin; they are not fixed amplitude-independent waits.
OBSERVED_FEEDBACK_TRAVEL_RATE_STEPS_S = 60.0
TRAVEL_TIMEOUT_MARGIN_S = 1.5
MIN_TRAVEL_TIMEOUT_S = 1.0
COMMAND_TRACK_TOLERANCE_STEPS = 1
FEEDBACK_TRACK_TOLERANCE_STEPS = 3
J10_TASK_SPACE_X_LIMIT_M = 0.04575
J10_TASK_SPACE_Y_POS_LIMIT_M = 0.049
J10_TASK_SPACE_Y_NEG_LIMIT_M = 0.0455
J10_TASK_SPACE_ROLL_POS_LIMIT_RAD = 62.5 * math.pi / 180.0
J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD = 69.0 * math.pi / 180.0
J10_TASK_SPACE_PITCH_POS_LIMIT_RAD = 55.0 * math.pi / 180.0
J10_TASK_SPACE_PITCH_NEG_LIMIT_RAD = 55.0 * math.pi / 180.0
J10_TASK_SPACE_YAW_POS_LIMIT_RAD = 97.5 * math.pi / 180.0
J10_TASK_SPACE_YAW_NEG_LIMIT_RAD = 100.0 * math.pi / 180.0
J10_TASK_SPACE_Z_LIMIT_M = 0.025
ABSOLUTE_LIMIT_LO = 0
ABSOLUTE_LIMIT_HI = 2000
HISTORICAL_LIMIT_LO = 200
HISTORICAL_LIMIT_HI = 1800
# Retained for historical artifact compatibility only. These values are not
# used as a J10 qualification gate; absolute [0, 2000] is authoritative.
EXPLORATORY_LIMIT_LO = 100
EXPLORATORY_LIMIT_HI = 1900

CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "stage",
    "axis",
    "direction",
    "target_amplitude",
    "target_units",
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "target_roll_deg",
    "target_pitch_deg",
    "target_yaw_deg",
    "requested_target_steps_1",
    "requested_target_steps_2",
    "requested_target_steps_3",
    "requested_target_steps_4",
    "requested_target_steps_5",
    "requested_target_steps_6",
    "predicted_target_steps_1",
    "predicted_target_steps_2",
    "predicted_target_steps_3",
    "predicted_target_steps_4",
    "predicted_target_steps_5",
    "predicted_target_steps_6",
    "actuator_margin_1",
    "actuator_margin_2",
    "actuator_margin_3",
    "actuator_margin_4",
    "actuator_margin_5",
    "actuator_margin_6",
    "absolute_margin_1",
    "absolute_margin_2",
    "absolute_margin_3",
    "absolute_margin_4",
    "absolute_margin_5",
    "absolute_margin_6",
    "absolute_margin_min",
    "actuator_margin_semantics",
    "limiting_actuator",
    "qualification",
    "limit_reason",
    "historical_envelope_exceeded",
    "absolute_min",
    "absolute_max",
    "historical_min",
    "historical_max",
    "exploratory_min",
    "exploratory_max",
    "exploratory_envelope_deprecated",
    "exploratory_limits_hard_gate",
    "maintenance_mode_required",
    "default_task_space_limit_x",
    "active_task_space_limit_x_positive",
    "active_task_space_limit_x_negative",
    "active_task_space_limit_y_positive",
    "active_task_space_limit_y_negative",
    "active_task_space_limit_roll_positive_deg",
    "active_task_space_limit_roll_negative_deg",
    "j10_extended_task_space_gate",
    "home_return_result",
    "home_return_confirmed",
    "home_return_duration_s",
    "home_return_final_feedback_1",
    "home_return_final_feedback_2",
    "home_return_final_feedback_3",
    "home_return_final_feedback_4",
    "home_return_final_feedback_5",
    "home_return_final_feedback_6",
    "home_return_max_error_steps",
    "home_return_failure",
    "joint_cmd_1",
    "joint_cmd_2",
    "joint_cmd_3",
    "joint_cmd_4",
    "joint_cmd_5",
    "joint_cmd_6",
    "joint_fb_1",
    "joint_fb_2",
    "joint_fb_3",
    "joint_fb_4",
    "joint_fb_5",
    "joint_fb_6",
    "fk_x_mm",
    "fk_y_mm",
    "fk_z_mm",
    "fk_roll_deg",
    "fk_pitch_deg",
    "fk_yaw_deg",
    "p_stamp_ns",
    "p_qx",
    "p_qy",
    "p_qz",
    "p_qw",
    "p_gx_rad_s",
    "p_gy_rad_s",
    "p_gz_rad_s",
    "p_ax_raw",
    "p_ay_raw",
    "p_az_raw",
    "p_aw_x",
    "p_aw_y",
    "p_aw_z",
    "b_stamp_ns",
    "b_qx",
    "b_qy",
    "b_qz",
    "b_qw",
    "b_gx_rad_s",
    "b_gy_rad_s",
    "b_gz_rad_s",
    "b_ax_raw",
    "b_ay_raw",
    "b_az_raw",
    "b_aw_x",
    "b_aw_y",
    "b_aw_z",
    "uart_cycle_count",
    "six_axis_0x29_frame_count",
    "fault_codes",
    "motion_executed",
]


def _units_for_axis(axis: str) -> str:
    return "deg" if axis in ("roll", "pitch", "yaw") else "mm"


def _axis_pose(axis: str, signed_amplitude: float, units: str, *, z0: float) -> list[float]:
    pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    if axis == "x":
        pose[0] = signed_amplitude * 1e-3
    elif axis == "y":
        pose[1] = signed_amplitude * 1e-3
    elif axis == "z":
        pose[2] = z0 + signed_amplitude * 1e-3
    elif axis == "roll":
        pose[3] = math.radians(signed_amplitude) if units == "deg" else signed_amplitude
    elif axis == "pitch":
        pose[4] = math.radians(signed_amplitude) if units == "deg" else signed_amplitude
    elif axis == "yaw":
        pose[5] = math.radians(signed_amplitude) if units == "deg" else signed_amplitude
    else:
        raise ValueError(f"Unsupported axis: {axis}")
    return pose


def _per_axis_margin(predicted_steps: Sequence[int], lower: int, upper: int) -> tuple[list[int], str]:
    margins = [min(s - lower, upper - s) for s in predicted_steps]
    limiting_idx = int(min(range(len(margins)), key=lambda i: margins[i]))
    return margins, f"ID{limiting_idx + 1}"


def _is_within_range(predicted_steps: Sequence[int], lower: int, upper: int) -> bool:
    return all(lower <= s <= upper for s in predicted_steps)


def _estimate_step_travel_timeout_s(
    start_steps: Sequence[int],
    target_steps: Sequence[int],
    *,
    rate_steps_s: float = OBSERVED_FEEDBACK_TRAVEL_RATE_STEPS_S,
    margin_s: float = TRAVEL_TIMEOUT_MARGIN_S,
) -> float:
    """Estimate a bounded experiment-layer timeout from multi-axis travel."""
    if rate_steps_s <= 0.0 or margin_s < 0.0:
        raise ValueError("travel rate must be positive and timeout margin non-negative")
    distance_steps = max(
        (abs(int(target) - int(start)) for start, target in zip(start_steps, target_steps)),
        default=0,
    )
    return max(MIN_TRAVEL_TIMEOUT_S, distance_steps / rate_steps_s + margin_s)


def _task_space_gate(
    pose: Sequence[float], z0: float, *,
    x_positive_limit_m: float = TASK_GATE_MAX_XY_M,
    x_negative_limit_m: float = TASK_GATE_MAX_XY_M,
    y_positive_limit_m: float = TASK_GATE_MAX_XY_M,
    y_negative_limit_m: float = TASK_GATE_MAX_XY_M,
    z_positive_limit_m: float = TASK_GATE_MAX_Z_DELTA_M,
    z_negative_limit_m: float = TASK_GATE_MAX_Z_DELTA_M,
    roll_positive_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
    roll_negative_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
    pitch_positive_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
    pitch_negative_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
    yaw_positive_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
    yaw_negative_limit_rad: float = TASK_GATE_MAX_ANGLE_RAD,
) -> Tuple[bool, str]:
    x, y, z, roll, pitch, yaw = pose
    if (
        x > x_positive_limit_m + 1e-12
        or x < -x_negative_limit_m - 1e-12
        or y > y_positive_limit_m + 1e-12
        or y < -y_negative_limit_m - 1e-12
    ):
        return False, "task_space_gate: planar translation outside existing J4 envelope"
    if (z - z0) > z_positive_limit_m + 1e-12 or (z - z0) < -z_negative_limit_m - 1e-12:
        return False, "task_space_gate: vertical delta outside existing J4 envelope"
    if roll > roll_positive_limit_rad + 1e-12 or roll < -roll_negative_limit_rad - 1e-12:
        return False, "task_space_gate: roll outside existing J4 envelope"
    if pitch > pitch_positive_limit_rad + 1e-12 or pitch < -pitch_negative_limit_rad - 1e-12:
        return False, "task_space_gate: pitch outside existing J4 envelope"
    if yaw > yaw_positive_limit_rad + 1e-12 or yaw < -yaw_negative_limit_rad - 1e-12:
        return False, "task_space_gate: yaw outside existing J4 envelope"
    return True, ""


@dataclass
class WorkspaceCandidatePlan:
    axis: str
    direction: str
    target_amplitude: float
    target_units: str
    target_pose: list[float]
    requested_target_steps: list[int]
    predicted_target_steps: list[int]
    actuator_margin: list[int]
    absolute_margin: list[int]
    absolute_margin_min: int
    limiting_actuator: str
    qualification: str
    limit_reason: str
    historical_envelope_exceeded: bool
    absolute_min: int
    absolute_max: int
    historical_min: int
    historical_max: int
    exploratory_min: int
    exploratory_max: int
    maintenance_mode_required: bool
    home_return_result: str
    task_space_gate_passed: bool
    ik_valid: bool


def compute_workspace_candidate_preflight(
    axis: str,
    direction: str,
    amplitude: float,
    units: str | None = None,
    *,
    kin: Analytical6PusKinematics | None = None,
    j10_extended_task_space_gate: bool = False,
) -> WorkspaceCandidatePlan:
    axis = axis.lower().strip()
    direction = direction.lower().strip()
    units = _units_for_axis(axis) if units is None else units.lower().strip()
    if axis not in ("x", "y", "z", "roll", "pitch", "yaw"):
        raise ValueError(f"Unsupported axis: {axis}")
    if direction not in ("positive", "negative"):
        raise ValueError(f"Unsupported direction: {direction}")
    if units not in ("mm", "deg"):
        raise ValueError(f"Unsupported units: {units}")

    kin = kin or Analytical6PusKinematics()
    z0 = kin.nominal_height
    signed = float(abs(amplitude)) * (1.0 if direction == "positive" else -1.0)
    pose = _axis_pose(axis, signed, units, z0=z0)

    extended_x_candidate = j10_extended_task_space_gate and axis == "x"
    extended_y_candidate = j10_extended_task_space_gate and axis == "y"
    extended_z_candidate = j10_extended_task_space_gate and axis == "z"
    extended_roll_candidate = j10_extended_task_space_gate and axis == "roll"
    extended_pitch_candidate = j10_extended_task_space_gate and axis == "pitch"
    extended_yaw_candidate = j10_extended_task_space_gate and axis == "yaw"
    x_positive_limit_m = J10_TASK_SPACE_X_LIMIT_M if extended_x_candidate else TASK_GATE_MAX_XY_M
    x_negative_limit_m = J10_TASK_SPACE_X_LIMIT_M if extended_x_candidate else TASK_GATE_MAX_XY_M
    y_positive_limit_m = J10_TASK_SPACE_Y_POS_LIMIT_M if extended_y_candidate else TASK_GATE_MAX_XY_M
    y_negative_limit_m = J10_TASK_SPACE_Y_NEG_LIMIT_M if extended_y_candidate else TASK_GATE_MAX_XY_M
    z_positive_limit_m = J10_TASK_SPACE_Z_LIMIT_M if extended_z_candidate else TASK_GATE_MAX_Z_DELTA_M
    z_negative_limit_m = J10_TASK_SPACE_Z_LIMIT_M if extended_z_candidate else TASK_GATE_MAX_Z_DELTA_M
    roll_positive_limit_rad = J10_TASK_SPACE_ROLL_POS_LIMIT_RAD if extended_roll_candidate else TASK_GATE_MAX_ANGLE_RAD
    roll_negative_limit_rad = J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD if extended_roll_candidate else TASK_GATE_MAX_ANGLE_RAD
    pitch_positive_limit_rad = J10_TASK_SPACE_PITCH_POS_LIMIT_RAD if extended_pitch_candidate else TASK_GATE_MAX_ANGLE_RAD
    pitch_negative_limit_rad = J10_TASK_SPACE_PITCH_NEG_LIMIT_RAD if extended_pitch_candidate else TASK_GATE_MAX_ANGLE_RAD
    yaw_positive_limit_rad = J10_TASK_SPACE_YAW_POS_LIMIT_RAD if extended_yaw_candidate else TASK_GATE_MAX_ANGLE_RAD
    yaw_negative_limit_rad = J10_TASK_SPACE_YAW_NEG_LIMIT_RAD if extended_yaw_candidate else TASK_GATE_MAX_ANGLE_RAD
    task_ok, task_reason = _task_space_gate(
        pose, z0,
        x_positive_limit_m=x_positive_limit_m,
        x_negative_limit_m=x_negative_limit_m,
        y_positive_limit_m=y_positive_limit_m,
        y_negative_limit_m=y_negative_limit_m,
        z_positive_limit_m=z_positive_limit_m,
        z_negative_limit_m=z_negative_limit_m,
        roll_positive_limit_rad=roll_positive_limit_rad,
        roll_negative_limit_rad=roll_negative_limit_rad,
        pitch_positive_limit_rad=pitch_positive_limit_rad,
        pitch_negative_limit_rad=pitch_negative_limit_rad,
        yaw_positive_limit_rad=yaw_positive_limit_rad,
        yaw_negative_limit_rad=yaw_negative_limit_rad,
    )
    home_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]

    # The explicit J10 extension applies only to experiment-layer candidates;
    # all default and other paths retain the original task-space gate.
    extended_candidate = (
        extended_x_candidate
        or extended_y_candidate
        or extended_z_candidate
        or extended_roll_candidate
        or extended_pitch_candidate
        or extended_yaw_candidate
    )
    if not task_ok and not extended_candidate:
        return WorkspaceCandidatePlan(
            axis=axis,
            direction=direction,
            target_amplitude=float(abs(amplitude)),
            target_units=units,
            target_pose=pose,
            requested_target_steps=[1000] * 6,
            predicted_target_steps=[1000] * 6,
            actuator_margin=[ABSOLUTE_LIMIT_HI - 1000] * 6,
            absolute_margin=[ABSOLUTE_LIMIT_HI - 1000] * 6,
            absolute_margin_min=ABSOLUTE_LIMIT_HI - 1000,
            limiting_actuator="ID1",
            qualification="LIMIT",
            limit_reason=task_reason,
            historical_envelope_exceeded=False,
            absolute_min=ABSOLUTE_LIMIT_LO,
            absolute_max=ABSOLUTE_LIMIT_HI,
            historical_min=HISTORICAL_LIMIT_LO,
            historical_max=HISTORICAL_LIMIT_HI,
            exploratory_min=EXPLORATORY_LIMIT_LO,
            exploratory_max=EXPLORATORY_LIMIT_HI,
            maintenance_mode_required=False,
            home_return_result="not_executed",
            task_space_gate_passed=False,
            ik_valid=False,
        )

    try:
        requested_delta_steps = kin.differential_inverse_kinematics_steps(pose, home_pose)
        requested_target_steps = [1000 + int(step) for step in requested_delta_steps]
        _ = kin.inverse_kinematics(pose)
        ik_valid = True
    except Exception as exc:
        return WorkspaceCandidatePlan(
            axis=axis,
            direction=direction,
            target_amplitude=float(abs(amplitude)),
            target_units=units,
            target_pose=pose,
            requested_target_steps=[1000] * 6,
            predicted_target_steps=[1000] * 6,
            actuator_margin=[ABSOLUTE_LIMIT_HI - 1000] * 6,
            absolute_margin=[ABSOLUTE_LIMIT_HI - 1000] * 6,
            absolute_margin_min=ABSOLUTE_LIMIT_HI - 1000,
            limiting_actuator="ID1",
            qualification="LIMIT",
            limit_reason=f"ik_invalid: {exc}",
            historical_envelope_exceeded=False,
            absolute_min=ABSOLUTE_LIMIT_LO,
            absolute_max=ABSOLUTE_LIMIT_HI,
            historical_min=HISTORICAL_LIMIT_LO,
            historical_max=HISTORICAL_LIMIT_HI,
            exploratory_min=EXPLORATORY_LIMIT_LO,
            exploratory_max=EXPLORATORY_LIMIT_HI,
            maintenance_mode_required=False,
            home_return_result="not_executed",
            task_space_gate_passed=task_ok,
            ik_valid=False,
        )

    historical_envelope_exceeded = not _is_within_range(
        requested_target_steps,
        HISTORICAL_LIMIT_LO,
        HISTORICAL_LIMIT_HI,
    )
    maintenance_mode_required = historical_envelope_exceeded
    margins, limiting_actuator = _per_axis_margin(
        requested_target_steps,
        ABSOLUTE_LIMIT_LO,
        ABSOLUTE_LIMIT_HI,
    )
    absolute_margin = list(margins)
    absolute_margin_min = min(absolute_margin)
    if not _is_within_range(requested_target_steps, ABSOLUTE_LIMIT_LO, ABSOLUTE_LIMIT_HI):
        return WorkspaceCandidatePlan(
            axis=axis,
            direction=direction,
            target_amplitude=float(abs(amplitude)),
            target_units=units,
            target_pose=pose,
            requested_target_steps=list(requested_target_steps),
            predicted_target_steps=list(requested_target_steps),
            actuator_margin=margins,
            absolute_margin=absolute_margin,
            absolute_margin_min=absolute_margin_min,
            limiting_actuator=limiting_actuator,
            qualification="LIMIT",
            limit_reason="predicted_step_outside_absolute_range",
            historical_envelope_exceeded=historical_envelope_exceeded,
            absolute_min=ABSOLUTE_LIMIT_LO,
            absolute_max=ABSOLUTE_LIMIT_HI,
            historical_min=HISTORICAL_LIMIT_LO,
            historical_max=HISTORICAL_LIMIT_HI,
            exploratory_min=EXPLORATORY_LIMIT_LO,
            exploratory_max=EXPLORATORY_LIMIT_HI,
            maintenance_mode_required=maintenance_mode_required,
            home_return_result="not_executed",
            task_space_gate_passed=task_ok,
            ik_valid=ik_valid,
        )

    return WorkspaceCandidatePlan(
        axis=axis,
        direction=direction,
        target_amplitude=float(abs(amplitude)),
        target_units=units,
        target_pose=pose,
        requested_target_steps=list(requested_target_steps),
        predicted_target_steps=list(requested_target_steps),
        actuator_margin=margins,
        absolute_margin=absolute_margin,
        absolute_margin_min=absolute_margin_min,
        limiting_actuator=limiting_actuator,
        qualification="PASS",
        limit_reason="",
        historical_envelope_exceeded=historical_envelope_exceeded,
        absolute_min=ABSOLUTE_LIMIT_LO,
        absolute_max=ABSOLUTE_LIMIT_HI,
        historical_min=HISTORICAL_LIMIT_LO,
        historical_max=HISTORICAL_LIMIT_HI,
        exploratory_min=EXPLORATORY_LIMIT_LO,
        exploratory_max=EXPLORATORY_LIMIT_HI,
        maintenance_mode_required=maintenance_mode_required,
        home_return_result="pending",
        task_space_gate_passed=task_ok,
        ik_valid=ik_valid,
    )


class WorkspaceMeasurementHarness(Node):
    def __init__(self, *, csv_path: Path, z0: float, plan: WorkspaceCandidatePlan, j10_extended_task_space_gate: bool = False) -> None:
        super().__init__("j10_a1_workspace_measurement_harness")
        self.csv_path = csv_path
        self.z0 = z0
        self.plan = plan
        self.j10_extended_task_space_gate = bool(j10_extended_task_space_gate)
        self.default_task_space_limit_x = TASK_GATE_MAX_XY_M * 1000.0
        self.active_task_space_limit_x_positive = J10_TASK_SPACE_X_LIMIT_M * 1000.0 if self.j10_extended_task_space_gate and plan.axis == "x" else 15.0
        self.active_task_space_limit_x_negative = J10_TASK_SPACE_X_LIMIT_M * 1000.0 if self.j10_extended_task_space_gate and plan.axis == "x" else 15.0
        self.active_task_space_limit_y_positive = J10_TASK_SPACE_Y_POS_LIMIT_M * 1000.0 if self.j10_extended_task_space_gate and plan.axis == "y" else 15.0
        self.active_task_space_limit_y_negative = J10_TASK_SPACE_Y_NEG_LIMIT_M * 1000.0 if self.j10_extended_task_space_gate and plan.axis == "y" else 15.0
        self.active_task_space_limit_roll_positive_deg = math.degrees(J10_TASK_SPACE_ROLL_POS_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.active_task_space_limit_roll_negative_deg = math.degrees(J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.active_task_space_limit_pitch_positive_deg = math.degrees(J10_TASK_SPACE_PITCH_POS_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "pitch" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.active_task_space_limit_pitch_negative_deg = math.degrees(J10_TASK_SPACE_PITCH_NEG_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "pitch" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.active_task_space_limit_yaw_positive_deg = math.degrees(J10_TASK_SPACE_YAW_POS_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "yaw" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.active_task_space_limit_yaw_negative_deg = math.degrees(J10_TASK_SPACE_YAW_NEG_LIMIT_RAD) if self.j10_extended_task_space_gate and plan.axis == "yaw" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self.lock = threading.Lock()

        self.current_stage = "preflight_only"
        self.target_x_mm = float(plan.target_pose[0] * 1000.0)
        self.target_y_mm = float(plan.target_pose[1] * 1000.0)
        self.target_z_mm = float(plan.target_pose[2] * 1000.0)
        self.target_rpy_deg = [
            math.degrees(plan.target_pose[3]),
            math.degrees(plan.target_pose[4]),
            math.degrees(plan.target_pose[5]),
        ]
        self.latest_cmd_steps = [1000] * 6
        self.latest_fb_steps = [1000] * 6
        self.latest_fault_codes = [0] * 6
        self.uart_cycle_count = 0
        self.six_axis_0x29_frame_count = 0
        self.motion_executed = False
        self.candidate_reached = False
        self.candidate_reach_duration_s = 0.0
        self.candidate_reach_failure = ""
        self.measurement_tracking_valid = False
        self.measurement_tracking_invalid_samples = 0

        self.requested_target_steps = list(plan.requested_target_steps)
        self.predicted_target_steps = list(plan.predicted_target_steps)
        self.actuator_margin = list(plan.actuator_margin)
        self.absolute_margin = list(plan.absolute_margin)
        self.absolute_margin_min = plan.absolute_margin_min
        self.limiting_actuator = plan.limiting_actuator
        self.qualification = plan.qualification
        self.limit_reason = plan.limit_reason
        self.home_return_result = plan.home_return_result
        self.home_return_confirmed = False
        self.home_return_duration_s = 0.0
        self.home_return_final_feedback = [1000] * 6
        self.home_return_max_error_steps = 0
        self.home_return_failure = ""
        self.feedback_sample_count = 0
        self.historical_envelope_exceeded = plan.historical_envelope_exceeded
        self.absolute_min = plan.absolute_min
        self.absolute_max = plan.absolute_max
        self.historical_min = plan.historical_min
        self.historical_max = plan.historical_max
        self.exploratory_min = plan.exploratory_min
        self.exploratory_max = plan.exploratory_max
        self.maintenance_mode_required = plan.maintenance_mode_required

        self.fk_xyz_m = [0.0, 0.0, z0]
        self.fk_rpy_rad = [0.0, 0.0, 0.0]
        self.latest_base_imu: Dict[str, Any] = {
            "stamp_ns": 0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "gyro": [0.0, 0.0, 0.0],
            "accel_raw": [0.0, 0.0, G_STANDARD],
            "aw": [0.0, 0.0, 0.0],
        }
        self.latest_plat_imu: Dict[str, Any] = {
            "stamp_ns": 0,
            "quat": [0.0, 0.0, 0.0, 1.0],
            "gyro": [0.0, 0.0, 0.0],
            "accel_raw": [0.0, 0.0, G_STANDARD],
            "aw": [0.0, 0.0, 0.0],
        }

        self.kin = Analytical6PusKinematics()

        self.target_pose_pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", 10)
        telemetry_qos = qos_profile_sensor_data
        self.cmd_sub = self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, telemetry_qos)
        self.fb_sub = self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_feedback, telemetry_qos)
        self.fk_sub = self.create_subscription(FkStateStamped, "/pus6/fk/state", self._on_fk_state, telemetry_qos)
        self.base_imu_sub = self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_profile_sensor_data)
        self.plat_imu_sub = self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_platform_imu, qos_profile_sensor_data)

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def set_stage(self, stage: str, x_m: float, y_m: float = 0.0, z_m: float | None = None, roll_rad: float = 0.0, pitch_rad: float = 0.0, yaw_rad: float = 0.0) -> None:
        if z_m is None:
            z_m = self.z0
        with self.lock:
            self.current_stage = stage
            self.target_x_mm = x_m * 1000.0
            self.target_y_mm = y_m * 1000.0
            self.target_z_mm = z_m * 1000.0
            self.target_rpy_deg = [math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)]

    def publish_target_pose(self, x_m: float, y_m: float, z_m: float, roll_rad: float, pitch_rad: float, yaw_rad: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x_m)
        msg.pose.position.y = float(y_m)
        msg.pose.position.z = float(z_m)
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)
        self.target_pose_pub.publish(msg)

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        with self.lock:
            if len(msg.data) >= 6:
                self.latest_cmd_steps = list(msg.data[:6])

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            if len(msg.raw_steps) >= 6:
                self.latest_fb_steps = [int(s) for s in msg.raw_steps[:6]]
                self.latest_fault_codes = [int(f) for f in msg.fault_codes[:6]]
                self.feedback_sample_count += 1
                # The driver publishes a zero-based cycle sequence; expose a
                # count of observed complete cycles in experiment artifacts.
                sequence = int(getattr(msg, "uart_cycle_sequence", -1))
                self.uart_cycle_count = max(self.uart_cycle_count, sequence + 1)
                self.six_axis_0x29_frame_count = self.uart_cycle_count * 6
                slider_m = [0.05 + (s - 1000) / 40000.0 for s in self.latest_fb_steps]
                try:
                    fk_res = self.kin.forward_kinematics(slider_m)
                    self.fk_xyz_m = fk_res[0:3]
                    self.fk_rpy_rad = fk_res[3:6]
                except Exception:
                    pass

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        with self.lock:
            if hasattr(msg, "pose") and hasattr(msg, "euler_angles_rad"):
                p = msg.pose.position
                self.fk_xyz_m = [float(p.x), float(p.y), float(p.z)]
                self.fk_rpy_rad = [float(v) for v in msg.euler_angles_rad[:3]]
            else:
                self.fk_xyz_m = [float(msg.x_m), float(msg.y_m), float(msg.z_m)]
                self.fk_rpy_rad = [float(msg.roll_rad), float(msg.pitch_rad), float(msg.yaw_rad)]

    def _on_base_imu(self, msg: Imu) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        f_world = np.array([ax, ay, az], dtype=np.float64)
        with self.lock:
            self.latest_base_imu = {
                "stamp_ns": stamp_ns,
                "quat": [qx, qy, qz, qw],
                "gyro": [gx, gy, gz],
                "accel_raw": [ax, ay, az],
                "aw": [float(f_world[0]), float(f_world[1]), float(f_world[2] - G_STANDARD)],
            }

    def _on_platform_imu(self, msg: Imu) -> None:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        aw_x = ax
        aw_y = ay
        aw_z = az - G_STANDARD
        with self.lock:
            if self._csv_file.closed:
                return
            self.latest_plat_imu = {
                "stamp_ns": stamp_ns,
                "quat": [qx, qy, qz, qw],
                "gyro": [gx, gy, gz],
                "accel_raw": [ax, ay, az],
                "aw": [aw_x, aw_y, aw_z],
            }
            b = self.latest_base_imu
            row = {
                "host_log_time_ns": host_ns,
                "host_monotonic_time_ns": mono_ns,
                "stage": self.current_stage,
                "axis": self.plan.axis,
                "direction": self.plan.direction,
                "target_amplitude": self.plan.target_amplitude,
                "target_units": self.plan.target_units,
                "target_x_mm": self.target_x_mm,
                "target_y_mm": self.target_y_mm,
                "target_z_mm": self.target_z_mm,
                "target_roll_deg": self.target_rpy_deg[0],
                "target_pitch_deg": self.target_rpy_deg[1],
                "target_yaw_deg": self.target_rpy_deg[2],
                "requested_target_steps_1": self.requested_target_steps[0],
                "requested_target_steps_2": self.requested_target_steps[1],
                "requested_target_steps_3": self.requested_target_steps[2],
                "requested_target_steps_4": self.requested_target_steps[3],
                "requested_target_steps_5": self.requested_target_steps[4],
                "requested_target_steps_6": self.requested_target_steps[5],
                "predicted_target_steps_1": self.predicted_target_steps[0],
                "predicted_target_steps_2": self.predicted_target_steps[1],
                "predicted_target_steps_3": self.predicted_target_steps[2],
                "predicted_target_steps_4": self.predicted_target_steps[3],
                "predicted_target_steps_5": self.predicted_target_steps[4],
                "predicted_target_steps_6": self.predicted_target_steps[5],
                "actuator_margin_1": self.actuator_margin[0],
                "actuator_margin_2": self.actuator_margin[1],
                "actuator_margin_3": self.actuator_margin[2],
                "actuator_margin_4": self.actuator_margin[3],
                "actuator_margin_5": self.actuator_margin[4],
                "actuator_margin_6": self.actuator_margin[5],
                "absolute_margin_1": self.absolute_margin[0],
                "absolute_margin_2": self.absolute_margin[1],
                "absolute_margin_3": self.absolute_margin[2],
                "absolute_margin_4": self.absolute_margin[3],
                "absolute_margin_5": self.absolute_margin[4],
                "absolute_margin_6": self.absolute_margin[5],
                "absolute_margin_min": self.absolute_margin_min,
                "actuator_margin_semantics": "absolute_[0,2000]",
                "limiting_actuator": self.limiting_actuator,
                "qualification": self.qualification,
                "limit_reason": self.limit_reason,
                "historical_envelope_exceeded": self.historical_envelope_exceeded,
                "absolute_min": self.absolute_min,
                "absolute_max": self.absolute_max,
                "historical_min": self.historical_min,
                "historical_max": self.historical_max,
                "exploratory_min": self.exploratory_min,
                "exploratory_max": self.exploratory_max,
                "exploratory_envelope_deprecated": True,
                "exploratory_limits_hard_gate": False,
                "maintenance_mode_required": self.maintenance_mode_required,
                "default_task_space_limit_x": self.default_task_space_limit_x,
                "active_task_space_limit_x_positive": self.active_task_space_limit_x_positive,
                "active_task_space_limit_x_negative": self.active_task_space_limit_x_negative,
                "active_task_space_limit_y_positive": self.active_task_space_limit_y_positive,
                "active_task_space_limit_y_negative": self.active_task_space_limit_y_negative,
                "active_task_space_limit_roll_positive_deg": self.active_task_space_limit_roll_positive_deg,
                "active_task_space_limit_roll_negative_deg": self.active_task_space_limit_roll_negative_deg,
                "j10_extended_task_space_gate": self.j10_extended_task_space_gate,
                "home_return_result": self.home_return_result,
                "home_return_confirmed": self.home_return_confirmed,
                "home_return_duration_s": self.home_return_duration_s,
                "home_return_final_feedback_1": self.home_return_final_feedback[0],
                "home_return_final_feedback_2": self.home_return_final_feedback[1],
                "home_return_final_feedback_3": self.home_return_final_feedback[2],
                "home_return_final_feedback_4": self.home_return_final_feedback[3],
                "home_return_final_feedback_5": self.home_return_final_feedback[4],
                "home_return_final_feedback_6": self.home_return_final_feedback[5],
                "home_return_max_error_steps": self.home_return_max_error_steps,
                "home_return_failure": self.home_return_failure,
                "joint_cmd_1": self.latest_cmd_steps[0],
                "joint_cmd_2": self.latest_cmd_steps[1],
                "joint_cmd_3": self.latest_cmd_steps[2],
                "joint_cmd_4": self.latest_cmd_steps[3],
                "joint_cmd_5": self.latest_cmd_steps[4],
                "joint_cmd_6": self.latest_cmd_steps[5],
                "joint_fb_1": self.latest_fb_steps[0],
                "joint_fb_2": self.latest_fb_steps[1],
                "joint_fb_3": self.latest_fb_steps[2],
                "joint_fb_4": self.latest_fb_steps[3],
                "joint_fb_5": self.latest_fb_steps[4],
                "joint_fb_6": self.latest_fb_steps[5],
                "fk_x_mm": self.fk_xyz_m[0] * 1000.0,
                "fk_y_mm": self.fk_xyz_m[1] * 1000.0,
                "fk_z_mm": self.fk_xyz_m[2] * 1000.0,
                "fk_roll_deg": math.degrees(self.fk_rpy_rad[0]),
                "fk_pitch_deg": math.degrees(self.fk_rpy_rad[1]),
                "fk_yaw_deg": math.degrees(self.fk_rpy_rad[2]),
                "p_stamp_ns": stamp_ns,
                "p_qx": qx,
                "p_qy": qy,
                "p_qz": qz,
                "p_qw": qw,
                "p_gx_rad_s": gx,
                "p_gy_rad_s": gy,
                "p_gz_rad_s": gz,
                "p_ax_raw": ax,
                "p_ay_raw": ay,
                "p_az_raw": az,
                "p_aw_x": aw_x,
                "p_aw_y": aw_y,
                "p_aw_z": aw_z,
                "b_stamp_ns": b["stamp_ns"],
                "b_qx": b["quat"][0],
                "b_qy": b["quat"][1],
                "b_qz": b["quat"][2],
                "b_qw": b["quat"][3],
                "b_gx_rad_s": b["gyro"][0],
                "b_gy_rad_s": b["gyro"][1],
                "b_gz_rad_s": b["gyro"][2],
                "b_ax_raw": b["accel_raw"][0],
                "b_ay_raw": b["accel_raw"][1],
                "b_az_raw": b["accel_raw"][2],
                "b_aw_x": b["aw"][0],
                "b_aw_y": b["aw"][1],
                "b_aw_z": b["aw"][2],
                "uart_cycle_count": self.uart_cycle_count,
                "six_axis_0x29_frame_count": self.six_axis_0x29_frame_count,
                "fault_codes": str(self.latest_fault_codes),
                "motion_executed": self.motion_executed,
            }
            self._writer.writerow(row)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J10-A1 single-candidate workspace measurement runner")
    parser.add_argument("--axis", required=True, choices=["x", "y", "z", "roll", "pitch", "yaw"])
    parser.add_argument("--direction", required=True, choices=["positive", "negative"])
    parser.add_argument("--amplitude", required=True, type=float)
    parser.add_argument("--units", choices=["mm", "deg"], help="Defaults to mm for translation axes and deg for attitude axes")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--execute-motion", action="store_true", help="Opt-in live motion path using the frozen J8 stack")
    parser.add_argument("--j10-extended-task-space-gate", action="store_true", help="J10-only X/Y gate extension")
    parser.add_argument("--maintenance-mode", action="store_true", help="Opt-in extended envelope execution via existing maintenance_mode=true")
    parser.add_argument("--hold-s", type=float, default=3.0)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--measurement-window-s", type=float, default=1.0)
    parser.add_argument("--candidate-timeout-s", type=float, help="Override derived candidate feedback travel timeout")
    parser.add_argument("--home-timeout-s", type=float, help="Override derived HOME feedback travel timeout")
    parser.add_argument("--home-consecutive-samples", type=int, default=5)
    parser.add_argument("--init-quiet-s", type=float, default=1.5)
    parser.add_argument("--motor-port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--base-imu-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00")
    parser.add_argument("--plat-imu-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00")
    parser.add_argument("--baud-motor", type=int, default=921600)
    parser.add_argument("--baud-imu", type=int, default=460800)
    return parser


def _analysis_from_rows(rows: Sequence[Dict[str, str]]) -> Dict[str, Any]:
    if not rows:
        return {"status": "EMPTY"}
    last = rows[-1]
    measurement_rows = [r for r in rows if r.get("stage") == "candidate_measurement"]
    fk_keys = ["fk_x_mm", "fk_y_mm", "fk_z_mm", "fk_roll_deg", "fk_pitch_deg", "fk_yaw_deg"]
    measurement_fk_median = (
        [statistics.median(float(r[k]) for r in measurement_rows) for k in fk_keys]
        if measurement_rows and all(k in measurement_rows[0] for k in fk_keys)
        else None
    )
    return {
        "status": "PASS" if last.get("qualification") == "PASS" else "LIMIT",
        "axis": last.get("axis", ""),
        "direction": last.get("direction", ""),
        "qualification": last.get("qualification", ""),
        "limit_reason": last.get("limit_reason", ""),
        "historical_envelope_exceeded": last.get("historical_envelope_exceeded", ""),
        "maintenance_mode_required": last.get("maintenance_mode_required", ""),
        "default_task_space_limit_x": last.get("default_task_space_limit_x", "0.015"),
        "active_task_space_limit_x_positive": last.get("active_task_space_limit_x_positive", "0.015"),
        "active_task_space_limit_x_negative": last.get("active_task_space_limit_x_negative", "0.015"),
        "active_task_space_limit_y_positive": last.get("active_task_space_limit_y_positive", "0.015"),
        "active_task_space_limit_y_negative": last.get("active_task_space_limit_y_negative", "0.015"),
        "active_task_space_limit_roll_positive_deg": last.get("active_task_space_limit_roll_positive_deg", "5.7295779513"),
        "active_task_space_limit_roll_negative_deg": last.get("active_task_space_limit_roll_negative_deg", "5.7295779513"),
        "j10_extended_task_space_gate": last.get("j10_extended_task_space_gate", "False"),
        "home_return_result": last.get("home_return_result", ""),
        "home_return_confirmed": last.get("home_return_confirmed", "False"),
        "home_return_duration_s": float(last.get("home_return_duration_s") or 0.0),
        "home_return_max_error_steps": int(last.get("home_return_max_error_steps") or 0),
        "home_return_failure": last.get("home_return_failure", ""),
        "row_count": len(rows),
        "uart_cycle_count": max((int(r.get("uart_cycle_count") or 0) for r in rows), default=0),
        "six_axis_0x29_frame_count": max((int(r.get("six_axis_0x29_frame_count") or 0) for r in rows), default=0),
        "measurement_window_stage": "candidate_measurement",
        "measurement_window_row_count": len(measurement_rows),
        "measurement_window_fk_median": measurement_fk_median,
    }


def _home_feedback_in_band(feedback: Sequence[int]) -> bool:
    return len(feedback) >= 6 and all(
        HOME_MEASUREMENT_BAND_LOWER <= int(step) <= HOME_MEASUREMENT_BAND_UPPER
        for step in feedback[:6]
    )


def _home_timeout_from_feedback(feedback: Sequence[int], home_steps: Sequence[int]) -> float:
    """Derive a bounded HOME wait from the feedback actually observed."""
    if len(feedback) < 6 or len(home_steps) < 6:
        return _estimate_step_travel_timeout_s(home_steps, home_steps)
    return _estimate_step_travel_timeout_s(feedback[:6], home_steps[:6])


def _install_j10_extended_gate(node: Any, *, enabled: bool, z0: float) -> None:
    """Apply the opt-in gate to this J10 node instance only."""
    if not enabled:
        return

    def validate_j10_pose(pose: Sequence[float]) -> bool:
        return _task_space_gate(
            pose,
            z0,
            x_positive_limit_m=J10_TASK_SPACE_X_LIMIT_M,
            x_negative_limit_m=J10_TASK_SPACE_X_LIMIT_M,
            y_positive_limit_m=J10_TASK_SPACE_Y_POS_LIMIT_M,
            y_negative_limit_m=J10_TASK_SPACE_Y_NEG_LIMIT_M,
            z_positive_limit_m=J10_TASK_SPACE_Z_LIMIT_M,
            z_negative_limit_m=J10_TASK_SPACE_Z_LIMIT_M,
            roll_positive_limit_rad=J10_TASK_SPACE_ROLL_POS_LIMIT_RAD,
            roll_negative_limit_rad=J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD,
            pitch_positive_limit_rad=J10_TASK_SPACE_PITCH_POS_LIMIT_RAD,
            pitch_negative_limit_rad=J10_TASK_SPACE_PITCH_NEG_LIMIT_RAD,
            yaw_positive_limit_rad=J10_TASK_SPACE_YAW_POS_LIMIT_RAD,
            yaw_negative_limit_rad=J10_TASK_SPACE_YAW_NEG_LIMIT_RAD,
        )[0]

    node.validate_cartesian_workspace = validate_j10_pose


def run_workspace_measurement_experiment(
    *,
    axis: str,
    direction: str,
    amplitude: float,
    units: str | None = None,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
    execute_motion: bool = False,
    dry_run: bool = True,
    maintenance_mode: bool = False,
    hold_s: float = 3.0,
    settle_s: float = 0.5,
    measurement_window_s: float = 1.0,
    candidate_timeout_s: float | None = None,
    home_timeout_s: float | None = None,
    home_consecutive_samples: int = 5,
    init_quiet_s: float = 1.5,
    motor_port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    base_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_motor: int = 921600,
    baud_imu: int = 460800,
    emit_artifacts: bool = True,
    j10_extended_task_space_gate: bool = False,
) -> Dict[str, Any]:
    if candidate_timeout_s is not None and candidate_timeout_s <= 0.0:
        raise ValueError("candidate timeout must be positive when provided")
    if home_timeout_s is not None and home_timeout_s <= 0.0:
        raise ValueError("home timeout must be positive when provided")
    if home_consecutive_samples < 1:
        raise ValueError("consecutive sample count must be at least 1")
    kin = Analytical6PusKinematics()
    plan = compute_workspace_candidate_preflight(
        axis, direction, amplitude, units, kin=kin,
        j10_extended_task_space_gate=j10_extended_task_space_gate,
    )
    home_steps = [1000] * 6
    derived_candidate_timeout_s = _estimate_step_travel_timeout_s(
        home_steps, plan.predicted_target_steps
    )
    active_candidate_timeout_s = (
        float(candidate_timeout_s)
        if candidate_timeout_s is not None
        else max(float(hold_s), derived_candidate_timeout_s)
    )
    derived_home_timeout_s = _estimate_step_travel_timeout_s(
        plan.predicted_target_steps, home_steps
    )
    active_home_timeout_s = (
        float(home_timeout_s)
        if home_timeout_s is not None
        else derived_home_timeout_s
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = time.strftime("%Y%m%d_%H%M%S")
    dir_tag = f"{direction[:3]}{int(round(abs(amplitude) * 10)):02d}{plan.target_units}"
    csv_path = output_dir / f"J10_workspace_{axis}_{dir_tag}_{stamp_str}.csv"
    meta_path = output_dir / f"J10_workspace_{axis}_{dir_tag}_{stamp_str}_metadata.json"

    report: Dict[str, Any] = {
        "experiment_name": "J10-A1_Single_Candidate_Workspace_Measurement",
        "timestamp": stamp_str,
        "axis": axis,
        "direction": direction,
        "target_amplitude": abs(amplitude),
        "target_units": plan.target_units,
        "plan": plan.__dict__,
        "motion_executed": False,
        "uart_cycle_count": 0,
        "six_axis_0x29_frame_count": 0,
        "home_return_confirmed": False,
        "home_return_duration_s": 0.0,
        "home_return_final_feedback": [1000] * 6,
        "home_return_max_error_steps": 0,
        "home_return_failure": "",
        "maintenance_mode": maintenance_mode,
        "maintenance_mode_required": plan.maintenance_mode_required,
        "default_task_space_limit_x": TASK_GATE_MAX_XY_M * 1000.0,
        "active_task_space_limit_x_positive": (J10_TASK_SPACE_X_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "x" else 15.0),
        "active_task_space_limit_x_negative": (J10_TASK_SPACE_X_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "x" else 15.0),
        "active_task_space_limit_y_positive": (J10_TASK_SPACE_Y_POS_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "y" else 15.0),
        "active_task_space_limit_y_negative": (J10_TASK_SPACE_Y_NEG_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "y" else 15.0),
        "active_task_space_limit_roll_positive_deg": (math.degrees(J10_TASK_SPACE_ROLL_POS_LIMIT_RAD) if j10_extended_task_space_gate and axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_roll_negative_deg": (math.degrees(J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD) if j10_extended_task_space_gate and axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_pitch_positive_deg": (math.degrees(J10_TASK_SPACE_PITCH_POS_LIMIT_RAD) if j10_extended_task_space_gate and axis == "pitch" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_pitch_negative_deg": (math.degrees(J10_TASK_SPACE_PITCH_NEG_LIMIT_RAD) if j10_extended_task_space_gate and axis == "pitch" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_yaw_positive_deg": (math.degrees(J10_TASK_SPACE_YAW_POS_LIMIT_RAD) if j10_extended_task_space_gate and axis == "yaw" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_yaw_negative_deg": (math.degrees(J10_TASK_SPACE_YAW_NEG_LIMIT_RAD) if j10_extended_task_space_gate and axis == "yaw" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
        "active_task_space_limit_z_positive": (J10_TASK_SPACE_Z_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "z" else 20.0),
        "active_task_space_limit_z_negative": (J10_TASK_SPACE_Z_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "z" else 20.0),
        "j10_extended_task_space_gate": bool(j10_extended_task_space_gate),
        "measurement_window": {
            "settle_s": settle_s,
            "measurement_window_s": measurement_window_s,
            "final_window_stage": "candidate_measurement",
        },
        "home_confirmation": {
            "band_lower_steps": HOME_MEASUREMENT_BAND_LOWER,
            "band_upper_steps": HOME_MEASUREMENT_BAND_UPPER,
            "timeout_s": active_home_timeout_s,
            "consecutive_samples": home_consecutive_samples,
        },
        "pre_candidate_home_confirmed": False,
        "pre_candidate_home_duration_s": 0.0,
        "pre_candidate_home_timeout_s": 0.0,
        "candidate_reached": False,
        "candidate_reach_duration_s": 0.0,
        "candidate_reach_timeout_s": active_candidate_timeout_s,
        "candidate_reach_failure": "",
        "measurement_tracking_valid": False,
        "measurement_tracking_invalid_samples": 0,
        "status": "LIMIT" if (plan.qualification != "PASS" or not plan.task_space_gate_passed) else "PASS",
    }

    if not execute_motion or dry_run or plan.qualification != "PASS" or not plan.task_space_gate_passed:
        if not plan.task_space_gate_passed:
            report["candidate_reach_failure"] = plan.limit_reason or "task_space_gate_exceeded"
        if emit_artifacts:
            rows = [{
                "host_log_time_ns": time.time_ns(),
                "host_monotonic_time_ns": time.monotonic_ns(),
                "stage": "preflight_only",
                "axis": plan.axis,
                "direction": plan.direction,
                "target_amplitude": plan.target_amplitude,
                "target_units": plan.target_units,
                "target_x_mm": plan.target_pose[0] * 1000.0,
                "target_y_mm": plan.target_pose[1] * 1000.0,
                "target_z_mm": plan.target_pose[2] * 1000.0,
                "target_roll_deg": math.degrees(plan.target_pose[3]),
                "target_pitch_deg": math.degrees(plan.target_pose[4]),
                "target_yaw_deg": math.degrees(plan.target_pose[5]),
                "requested_target_steps_1": plan.requested_target_steps[0],
                "requested_target_steps_2": plan.requested_target_steps[1],
                "requested_target_steps_3": plan.requested_target_steps[2],
                "requested_target_steps_4": plan.requested_target_steps[3],
                "requested_target_steps_5": plan.requested_target_steps[4],
                "requested_target_steps_6": plan.requested_target_steps[5],
                "predicted_target_steps_1": plan.predicted_target_steps[0],
                "predicted_target_steps_2": plan.predicted_target_steps[1],
                "predicted_target_steps_3": plan.predicted_target_steps[2],
                "predicted_target_steps_4": plan.predicted_target_steps[3],
                "predicted_target_steps_5": plan.predicted_target_steps[4],
                "predicted_target_steps_6": plan.predicted_target_steps[5],
                "actuator_margin_1": plan.actuator_margin[0],
                "actuator_margin_2": plan.actuator_margin[1],
                "actuator_margin_3": plan.actuator_margin[2],
                "actuator_margin_4": plan.actuator_margin[3],
                "actuator_margin_5": plan.actuator_margin[4],
                "actuator_margin_6": plan.actuator_margin[5],
                "absolute_margin_1": plan.absolute_margin[0],
                "absolute_margin_2": plan.absolute_margin[1],
                "absolute_margin_3": plan.absolute_margin[2],
                "absolute_margin_4": plan.absolute_margin[3],
                "absolute_margin_5": plan.absolute_margin[4],
                "absolute_margin_6": plan.absolute_margin[5],
                "absolute_margin_min": plan.absolute_margin_min,
                "actuator_margin_semantics": "absolute_[0,2000]",
                "limiting_actuator": plan.limiting_actuator,
                "qualification": plan.qualification,
                "limit_reason": plan.limit_reason,
                "historical_envelope_exceeded": plan.historical_envelope_exceeded,
                "absolute_min": plan.absolute_min,
                "absolute_max": plan.absolute_max,
                "historical_min": plan.historical_min,
                "historical_max": plan.historical_max,
                "exploratory_min": plan.exploratory_min,
                "exploratory_max": plan.exploratory_max,
                "exploratory_envelope_deprecated": True,
                "exploratory_limits_hard_gate": False,
                "maintenance_mode_required": plan.maintenance_mode_required,
                "default_task_space_limit_x": TASK_GATE_MAX_XY_M * 1000.0,
                "active_task_space_limit_x_positive": (J10_TASK_SPACE_X_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "x" else 15.0),
                "active_task_space_limit_x_negative": (J10_TASK_SPACE_X_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "x" else 15.0),
                "active_task_space_limit_y_positive": (J10_TASK_SPACE_Y_POS_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "y" else 15.0),
                "active_task_space_limit_y_negative": (J10_TASK_SPACE_Y_NEG_LIMIT_M * 1000.0 if j10_extended_task_space_gate and axis == "y" else 15.0),
                "active_task_space_limit_roll_positive_deg": (math.degrees(J10_TASK_SPACE_ROLL_POS_LIMIT_RAD) if j10_extended_task_space_gate and axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
                "active_task_space_limit_roll_negative_deg": (math.degrees(J10_TASK_SPACE_ROLL_NEG_LIMIT_RAD) if j10_extended_task_space_gate and axis == "roll" else math.degrees(TASK_GATE_MAX_ANGLE_RAD)),
                "j10_extended_task_space_gate": j10_extended_task_space_gate,
                "home_return_result": "not_executed",
                "joint_cmd_1": "",
                "joint_cmd_2": "",
                "joint_cmd_3": "",
                "joint_cmd_4": "",
                "joint_cmd_5": "",
                "joint_cmd_6": "",
                "joint_fb_1": "",
                "joint_fb_2": "",
                "joint_fb_3": "",
                "joint_fb_4": "",
                "joint_fb_5": "",
                "joint_fb_6": "",
                "fk_x_mm": 0.0,
                "fk_y_mm": 0.0,
                "fk_z_mm": kin.nominal_height * 1000.0,
                "fk_roll_deg": 0.0,
                "fk_pitch_deg": 0.0,
                "fk_yaw_deg": 0.0,
                "p_stamp_ns": 0,
                "p_qx": 0.0,
                "p_qy": 0.0,
                "p_qz": 0.0,
                "p_qw": 1.0,
                "p_gx_rad_s": 0.0,
                "p_gy_rad_s": 0.0,
                "p_gz_rad_s": 0.0,
                "p_ax_raw": 0.0,
                "p_ay_raw": 0.0,
                "p_az_raw": 0.0,
                "p_aw_x": 0.0,
                "p_aw_y": 0.0,
                "p_aw_z": 0.0,
                "b_stamp_ns": 0,
                "b_qx": 0.0,
                "b_qy": 0.0,
                "b_qz": 0.0,
                "b_qw": 1.0,
                "b_gx_rad_s": 0.0,
                "b_gy_rad_s": 0.0,
                "b_gz_rad_s": 0.0,
                "b_ax_raw": 0.0,
                "b_ay_raw": 0.0,
                "b_az_raw": 0.0,
                "b_aw_x": 0.0,
                "b_aw_y": 0.0,
                "b_aw_z": 0.0,
                "uart_cycle_count": 0,
                "six_axis_0x29_frame_count": 0,
                "fault_codes": "[0, 0, 0, 0, 0, 0]",
                "motion_executed": False,
            }]
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
            report["csv_path"] = str(csv_path)
            report["metadata_path"] = str(meta_path)
            report["analysis"] = _analysis_from_rows(rows)
            meta_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report

    if rclpy is None:
        raise RuntimeError("ROS 2 dependencies are unavailable for execute-motion mode")

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    meta_path.parent.mkdir(parents=True, exist_ok=True)

    print("[MOTION] Starting live workspace measurement with the frozen J8 path.")
    base_proc = plat_proc = None
    harness = None
    try:
        from pus6_control.control_node import Pus6ControlNode
        from pus6_kinematics.fk_node import Pus6FkNode
        from pus6_kinematics.kinematics_node import Pus6KinematicsNode
        from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode

        print("[PREFLIGHT] Running read-only motion path preflight was already satisfied by local planning.")
        print("[SPAWNING] Starting Base H30 node (/pus6/imu_base/data)...")
        base_proc = subprocess.Popen(
            [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_base",
                "-p", f"port:={base_imu_port}",
                "-p", f"baud_rate:={baud_imu}",
                "-p", "frame_id:=pus6_base_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_base/data",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        print("[SPAWNING] Starting Platform H30 node (/pus6/imu_platform/data)...")
        plat_proc = subprocess.Popen(
            [
                "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
                "-r", "__ns:=/pus6/imu_platform",
                "-p", f"port:={plat_imu_port}",
                "-p", f"baud_rate:={baud_imu}",
                "-p", "frame_id:=pus6_platform_imu_link",
                "-p", "world_frame:=world",
                "-p", "imu_topic:=/pus6/imu_platform/data",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        time.sleep(1.5)

        rclpy.init()
        kinematics_node = Pus6KinematicsNode()
        _install_j10_extended_gate(
            kinematics_node,
            enabled=j10_extended_task_space_gate,
            z0=kin.nominal_height,
        )
        control_node = Pus6ControlNode()
        motor_driver_node = Pus6MotorDriverNode(
            hardware_mode="active",
            port=motor_port,
            baudrate=baud_motor,
            timeout=0.050,
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
            use_j7_servo=True,
            servo_speed=50,
            publish_raw_diagnostics=True,
            maintenance_mode=maintenance_mode,
        )
        fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0)
        harness = WorkspaceMeasurementHarness(
            csv_path=csv_path, z0=kin.nominal_height, plan=plan,
            j10_extended_task_space_gate=j10_extended_task_space_gate,
        )
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (kinematics_node, control_node, motor_driver_node, fk_node, harness):
            executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # HOME -> candidate -> HOME.
        harness.set_stage("pre_home", 0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
        tend = time.monotonic() + init_quiet_s
        while time.monotonic() < tend:
            harness.publish_target_pose(0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
            time.sleep(0.020)

        # A failed prior candidate may leave the mechanism away from HOME.
        # Confirm fresh HOME feedback before starting the candidate transition;
        # otherwise the candidate timeout would be computed from a false origin.
        with harness.lock:
            pre_home_start_feedback = list(harness.latest_fb_steps)
            last_pre_home_feedback_count = harness.feedback_sample_count
        pre_home_timeout_s = _home_timeout_from_feedback(pre_home_start_feedback, home_steps)
        pre_home_start = time.monotonic()
        pre_home_deadline = pre_home_start + pre_home_timeout_s
        pre_home_consecutive = 0
        pre_home_confirmed = False
        while time.monotonic() < pre_home_deadline:
            harness.publish_target_pose(0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
            time.sleep(0.020)
            with harness.lock:
                feedback_count = harness.feedback_sample_count
                feedback = list(harness.latest_fb_steps)
            if feedback_count != last_pre_home_feedback_count:
                last_pre_home_feedback_count = feedback_count
                if _home_feedback_in_band(feedback):
                    pre_home_consecutive += 1
                else:
                    pre_home_consecutive = 0
                if pre_home_consecutive >= home_consecutive_samples:
                    pre_home_confirmed = True
                    break

        report["pre_candidate_home_confirmed"] = pre_home_confirmed
        report["pre_candidate_home_duration_s"] = time.monotonic() - pre_home_start
        report["pre_candidate_home_timeout_s"] = pre_home_timeout_s
        if not pre_home_confirmed:
            harness.motion_executed = True
            report["motion_executed"] = True
            report["status"] = "LIMIT"
            report["candidate_reach_failure"] = "pre_candidate_home_feedback_timeout"
            return report

        x, y, z, roll, pitch, yaw = plan.target_pose
        harness.set_stage("candidate_transition", x, y, z, roll, pitch, yaw)
        candidate_start = time.monotonic()
        tend = candidate_start + active_candidate_timeout_s
        reached_count = 0
        with harness.lock:
            last_candidate_feedback_count = harness.feedback_sample_count
        while time.monotonic() < tend:
            harness.publish_target_pose(x, y, z, roll, pitch, yaw)
            time.sleep(0.020)

            with harness.lock:
                command_reached = all(abs(a - b) <= COMMAND_TRACK_TOLERANCE_STEPS for a, b in zip(harness.latest_cmd_steps, plan.predicted_target_steps))
                feedback_reached = all(abs(a - b) <= FEEDBACK_TRACK_TOLERANCE_STEPS for a, b in zip(harness.latest_fb_steps, plan.predicted_target_steps))
                feedback_count = harness.feedback_sample_count
            if feedback_count != last_candidate_feedback_count:
                last_candidate_feedback_count = feedback_count
                reached_count = reached_count + 1 if command_reached and feedback_reached else 0
            if reached_count >= 5:
                harness.candidate_reached = True
                harness.candidate_reach_duration_s = time.monotonic() - candidate_start
                break

        if not harness.candidate_reached:
            harness.candidate_reach_failure = "candidate_feedback_target_timeout"
            harness.motion_executed = True
            report["motion_executed"] = True
            report["status"] = "LIMIT"
            report["candidate_reached"] = False
            report["candidate_reach_duration_s"] = time.monotonic() - candidate_start
            report["candidate_reach_timeout_s"] = active_candidate_timeout_s
            report["candidate_reach_failure"] = harness.candidate_reach_failure
            return report

        harness.set_stage("candidate_settle", x, y, z, roll, pitch, yaw)
        tend = time.monotonic() + settle_s
        while time.monotonic() < tend:
            harness.publish_target_pose(x, y, z, roll, pitch, yaw)
            time.sleep(0.020)

        harness.set_stage("candidate_measurement", x, y, z, roll, pitch, yaw)
        harness.measurement_tracking_valid = True
        tend = time.monotonic() + measurement_window_s
        with harness.lock:
            last_measurement_feedback_count = harness.feedback_sample_count
        while time.monotonic() < tend:
            harness.publish_target_pose(x, y, z, roll, pitch, yaw)
            time.sleep(0.020)
            with harness.lock:
                command_reached = all(abs(a - b) <= COMMAND_TRACK_TOLERANCE_STEPS for a, b in zip(harness.latest_cmd_steps, plan.predicted_target_steps))
                feedback_reached = all(abs(a - b) <= FEEDBACK_TRACK_TOLERANCE_STEPS for a, b in zip(harness.latest_fb_steps, plan.predicted_target_steps))
                feedback_count = harness.feedback_sample_count
            if feedback_count != last_measurement_feedback_count and not (command_reached and feedback_reached):
                harness.measurement_tracking_valid = False
                harness.measurement_tracking_invalid_samples += 1
            last_measurement_feedback_count = feedback_count

        harness.set_stage("return_home", 0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
        home_start = time.monotonic()
        with harness.lock:
            home_start_feedback = list(harness.latest_fb_steps)
        active_runtime_home_timeout_s = (
            float(home_timeout_s)
            if home_timeout_s is not None
            else _estimate_step_travel_timeout_s(home_start_feedback, [1000] * 6)
        )
        home_deadline = home_start + active_runtime_home_timeout_s
        with harness.lock:
            last_feedback_count = harness.feedback_sample_count
        consecutive_home_samples = 0
        confirmed_home = False
        while time.monotonic() < home_deadline:
            harness.publish_target_pose(0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
            time.sleep(0.020)

            with harness.lock:
                feedback_count = harness.feedback_sample_count
                feedback = list(harness.latest_fb_steps)
            if feedback_count != last_feedback_count:
                last_feedback_count = feedback_count
                if _home_feedback_in_band(feedback):
                    consecutive_home_samples += 1
                else:
                    consecutive_home_samples = 0
                if consecutive_home_samples >= home_consecutive_samples:
                    confirmed_home = True
                    break

        harness.motion_executed = True
        with harness.lock:
            harness.home_return_final_feedback = list(harness.latest_fb_steps)
        harness.home_return_confirmed = confirmed_home
        harness.home_return_duration_s = time.monotonic() - home_start
        harness.home_return_max_error_steps = max(
            abs(int(step) - 1000) for step in harness.home_return_final_feedback
        )
        harness.home_return_failure = "" if confirmed_home else "feedback_band_timeout"
        harness.home_return_result = "confirmed" if confirmed_home else "not_confirmed"
        harness.set_stage("post_home", 0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
        tend = time.monotonic() + init_quiet_s
        while time.monotonic() < tend:
            harness.publish_target_pose(0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0)
            time.sleep(0.020)

        report["motion_executed"] = True
        report["uart_cycle_count"] = harness.uart_cycle_count
        report["six_axis_0x29_frame_count"] = harness.six_axis_0x29_frame_count
        report["csv_path"] = str(csv_path)
        report["metadata_path"] = str(meta_path)
        report["status"] = "PASS" if plan.qualification == "PASS" and harness.candidate_reached and harness.measurement_tracking_valid and confirmed_home else "LIMIT"
        report["candidate_reached"] = harness.candidate_reached
        report["candidate_reach_duration_s"] = harness.candidate_reach_duration_s
        report["candidate_reach_timeout_s"] = active_candidate_timeout_s
        report["candidate_reach_failure"] = harness.candidate_reach_failure
        report["measurement_tracking_valid"] = harness.measurement_tracking_valid
        report["measurement_tracking_invalid_samples"] = harness.measurement_tracking_invalid_samples
        report["home_timeout_s_derived_or_override"] = active_runtime_home_timeout_s
        report["home_return_confirmed"] = confirmed_home
        report["home_return_duration_s"] = harness.home_return_duration_s
        report["home_return_final_feedback"] = harness.home_return_final_feedback
        report["home_return_max_error_steps"] = harness.home_return_max_error_steps
        report["home_return_failure"] = harness.home_return_failure
        report["analysis"] = {
            "status": report["status"],
            "qualification": plan.qualification,
            "home_return_result": harness.home_return_result,
        }
        return report
    finally:
        teardown_warning = False
        try:
            if harness is not None:
                with harness.lock:
                    if not harness._csv_file.closed:
                        harness._csv_file.flush()
                teardown_warning = bool(report.get("motion_executed"))
                harness.close()
                report["post_run_teardown_warning"] = teardown_warning
                report["motion_stop"] = False
                report["fault_stop"] = False
                report["metadata_finalization"] = "after_csv_close_and_teardown_state"
        except Exception as exc:
            report["artifact_finalize_error"] = str(exc)
        try:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if emit_artifacts:
            try:
                rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8", newline="")))
                report["csv_row_count"] = len(rows)
                report["analysis"] = _analysis_from_rows(rows)
                report["qualification"] = plan.qualification
                report["home_return_result"] = report.get("home_return_result", "completed" if report.get("motion_executed") else "not_executed")
                report["uart_cycle_count"] = max((int(r.get("uart_cycle_count") or 0) for r in rows), default=0)
                report["six_axis_0x29_frame_count"] = max((int(r.get("six_axis_0x29_frame_count") or 0) for r in rows), default=0)
                meta_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
            except Exception as exc:
                report["artifact_finalize_error"] = str(exc)
        try:
            if base_proc and base_proc.poll() is None:
                os.killpg(os.getpgid(base_proc.pid), signal.SIGINT)
        except Exception:
            pass
        try:
            if plat_proc and plat_proc.poll() is None:
                os.killpg(os.getpgid(plat_proc.pid), signal.SIGINT)
        except Exception:
            pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    units = args.units or _units_for_axis(args.axis)
    try:
        report = run_workspace_measurement_experiment(
            axis=args.axis,
            direction=args.direction,
            amplitude=args.amplitude,
            units=units,
            output_dir=Path(args.output_dir),
            execute_motion=args.execute_motion,
            dry_run=args.dry_run,
            maintenance_mode=args.maintenance_mode,
            hold_s=args.hold_s,
            settle_s=args.settle_s,
            measurement_window_s=args.measurement_window_s,
            candidate_timeout_s=args.candidate_timeout_s,
            init_quiet_s=args.init_quiet_s,
            motor_port=args.motor_port,
            base_imu_port=args.base_imu_port,
            plat_imu_port=args.plat_imu_port,
            baud_motor=args.baud_motor,
            baud_imu=args.baud_imu,
            j10_extended_task_space_gate=args.j10_extended_task_space_gate,
        )
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(f"[FATAL] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
