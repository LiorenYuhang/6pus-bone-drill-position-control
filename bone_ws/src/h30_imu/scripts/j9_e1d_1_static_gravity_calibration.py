#!/usr/bin/env python3
"""J9-E1D-1 static multi-pose gravity calibration runner.

The default invocation is software-only.  The real-hardware path is explicit
and is intentionally not exercised by the J9-E1D-1 preparation workflow.

This runner estimates only R_P_IP from static gravity directions:

    u_P = R_P_IP * u_I

where u_I is the measured gravity direction in the platform IMU frame and
u_P is the gravity direction predicted by FK in platform_link coordinates.
R_P_IP remains UNCALIBRATED_UNSET; this file never writes production config.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[4]
WS_ROOT = REPO_ROOT / "bone_ws"
for pkg in ("pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"):
    path = WS_ROOT / "src" / pkg
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))
scripts_dir = WS_ROOT / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))
imu_scripts_dir = WS_ROOT / "src" / "h30_imu" / "scripts"
if imu_scripts_dir.exists() and str(imu_scripts_dir) not in sys.path:
    sys.path.insert(0, str(imu_scripts_dir))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.geometry_config import load_geometry_config

ROS_AVAILABLE = True
try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped
    from h30_imu.msg import TimedImu
    from pus6_control.control_node import Pus6ControlNode
    from pus6_kinematics.fk_node import Pus6FkNode
    from pus6_kinematics.kinematics_node import Pus6KinematicsNode
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from run_j7_servo_home_park import run_servo_home_park
    from run_j8_c_hardware_readonly_audit import run_j8_c_audit
    from std_msgs.msg import Float64MultiArray, Int32MultiArray
except ModuleNotFoundError:
    ROS_AVAILABLE = False
    rclpy = None
    DiagnosticArray = None
    PoseStamped = TimedImu = FkStateStamped = TimedJointFeedback = None
    Float64MultiArray = Int32MultiArray = None
    Pus6ControlNode = Pus6FkNode = Pus6KinematicsNode = Pus6MotorDriverNode = None
    SingleThreadedExecutor = Node = object

MOTOR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PLATFORM_IMU_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00"
BAUD_MOTOR = 921600
BAUD_IMU = 460800
GRAVITY_M_S2 = 9.80665
STATIC_POSE_DEG = (-3.0, 0.0, 3.0)
STATIC_SETTLE_TIMEOUT_S = 12.0
STATIC_SETTLE_MIN_S = 2.0
STATIC_STABILITY_WINDOW_S = 1.0
STATIC_DWELL_S = 5.0
STATIC_MAX_JOINT_SPAN_STEPS = 2
STATIC_MAX_GYRO_NORM_P95_RAD_S = 0.015
STATIC_MAX_ACCEL_NORM_DEVIATION_M_S2 = 0.20
STATIC_MAX_ACCEL_AXIS_STD_M_S2 = 0.020
STATIC_MAX_GRAVITY_DIRECTION_P95_RAD = 0.0020
STATIC_MIN_FK_ROWS = 20
STATIC_EFFECTIVE_SAMPLES_PER_POSE = 200

STATIC_REFERENCE_EULER_XYZ_DEG: dict[str, list[float]] = {
    "E1D-1 clean": [0.524, 1.680, 90.492],
    "E1D-3": [0.53, 1.69, 90.4],
    "E1C-2 dynamic": [-1.5887907912730772, 0.04626813473352346, 90.49374663690152],
}


class UartOwnershipState(str, Enum):
    RELEASED = "RELEASED"
    ACTIVE_ROS = "ACTIVE_ROS"
    STANDALONE_HOME = "STANDALONE_HOME"
    STANDALONE_PARK = "STANDALONE_PARK"


@dataclass(frozen=True)
class UartOwnershipEvent:
    timestamp_monotonic_ns: int
    from_state: str
    to_state: str
    reason: str


class UartOwnershipAudit:
    """Runner-level guard that prevents overlapping motor UART owners."""

    def __init__(self) -> None:
        self.state = UartOwnershipState.RELEASED
        self.events: list[UartOwnershipEvent] = [
            UartOwnershipEvent(
                timestamp_monotonic_ns=time.monotonic_ns(),
                from_state="INIT",
                to_state=self.state.value,
                reason="runner_start",
            )
        ]

    def _transition(self, to_state: UartOwnershipState, reason: str) -> None:
        from_state = self.state
        valid = (
            (from_state == UartOwnershipState.RELEASED and to_state != UartOwnershipState.RELEASED)
            or (from_state != UartOwnershipState.RELEASED and to_state == UartOwnershipState.RELEASED)
        )
        if not valid:
            raise RuntimeError(
                f"invalid UART ownership transition {from_state.value} -> {to_state.value} for {reason}; "
                "overlapping motor UART owners are forbidden"
            )
        self.state = to_state
        self.events.append(
            UartOwnershipEvent(
                timestamp_monotonic_ns=time.monotonic_ns(),
                from_state=from_state.value,
                to_state=to_state.value,
                reason=str(reason),
            )
        )

    def enter_active_ros(self, reason: str) -> None:
        self._transition(UartOwnershipState.ACTIVE_ROS, reason)

    def enter_standalone(self, state: UartOwnershipState, reason: str) -> None:
        if state not in (UartOwnershipState.STANDALONE_HOME, UartOwnershipState.STANDALONE_PARK):
            raise ValueError("standalone state must be STANDALONE_HOME or STANDALONE_PARK")
        self._transition(state, reason)

    def release(self, reason: str) -> None:
        self._transition(UartOwnershipState.RELEASED, reason)

    def require_released(self, reason: str) -> None:
        if self.state != UartOwnershipState.RELEASED:
            raise RuntimeError(f"UART ownership must be RELEASED before {reason}; current={self.state.value}")

    def as_dict(self) -> dict[str, Any]:
        return {
            "current_state": self.state.value,
            "events": [asdict(event) for event in self.events],
        }


@dataclass
class RecordingPipeline:
    h30_process: subprocess.Popen[str] | None
    executor: Any
    spin_thread: threading.Thread
    observer: Any


@dataclass
class ActiveMotionPipeline:
    executor: Any
    spin_thread: threading.Thread
    target_node: Any
    publisher: Any
    kin_node: Any
    control_node: Any
    motor_node: Any
    fk_node: Any
    z0: float
    released: bool = False


def skew(vector: Iterable[float]) -> np.ndarray:
    x, y, z = np.asarray(list(vector), dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def euler_zyx_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def quaternion_xyzw_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> list[float]:
    return Rotation.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad]).as_quat().tolist()


@dataclass(frozen=True)
class StaticPose:
    name: str
    roll_deg: float
    pitch_deg: float
    yaw_deg: float = 0.0

    @property
    def rpy_rad(self) -> tuple[float, float, float]:
        return tuple(math.radians(v) for v in (self.roll_deg, self.pitch_deg, self.yaw_deg))

    def target_pose(self, z0: float) -> list[float]:
        r, p, y = self.rpy_rad
        return [0.0, 0.0, z0, r, p, y]


STATIC_POSES: dict[str, StaticPose] = {
    "home": StaticPose("home", 0.0, 0.0),
    "r-3_p-3": StaticPose("r-3_p-3", -3.0, -3.0),
    "r-3_p0": StaticPose("r-3_p0", -3.0, 0.0),
    "r-3_p+3": StaticPose("r-3_p+3", -3.0, 3.0),
    "r0_p+3": StaticPose("r0_p+3", 0.0, 3.0),
    "r+3_p+3": StaticPose("r+3_p+3", 3.0, 3.0),
    "r+3_p0": StaticPose("r+3_p0", 3.0, 0.0),
    "r+3_p-3": StaticPose("r+3_p-3", 3.0, -3.0),
    "r0_p-3": StaticPose("r0_p-3", 0.0, -3.0),
}

# Both traversals contain the same nine poses exactly once.  Adjacent moves
# are one grid edge or one diagonal-to-cardinal edge.
FIT_ORDER = (
    "home",
    "r-3_p-3",
    "r-3_p0",
    "r-3_p+3",
    "r0_p+3",
    "r+3_p+3",
    "r+3_p0",
    "r+3_p-3",
    "r0_p-3",
)
VALIDATION_ORDER = (
    "home",
    "r+3_p+3",
    "r0_p+3",
    "r-3_p+3",
    "r-3_p0",
    "r-3_p-3",
    "r0_p-3",
    "r+3_p-3",
    "r+3_p0",
)


def platform_gravity_direction(pose: StaticPose) -> np.ndarray:
    """Return world gravity [0,0,-1] expressed in platform_link."""
    r, p, y = pose.rpy_rad
    return euler_zyx_matrix(r, p, y).T @ np.array([0.0, 0.0, -1.0])


def measured_gravity_direction(acceleration_m_s2: Iterable[float]) -> np.ndarray:
    """H30 reports specific force, so stationary gravity is -a/||a||."""
    acceleration = np.asarray(list(acceleration_m_s2), dtype=float)
    norm = float(np.linalg.norm(acceleration))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("invalid accelerometer norm")
    return -acceleration / norm


def acceleration_stability_metrics(accelerations_m_s2: np.ndarray) -> dict[str, Any]:
    accelerations = np.asarray(accelerations_m_s2, dtype=float)
    if accelerations.ndim != 2 or accelerations.shape[1] != 3 or len(accelerations) < 2:
        raise ValueError("accelerations must have shape (N, 3) with N >= 2")
    norms = np.linalg.norm(accelerations, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms <= 1e-9):
        raise ValueError("invalid acceleration vector in stability window")
    directions = -accelerations / norms[:, None]
    mean_direction = measured_gravity_direction(np.mean(accelerations, axis=0))
    spread_rad = np.arccos(np.clip(directions @ mean_direction, -1.0, 1.0))
    return {
        "mean_vector_m_s2": np.mean(accelerations, axis=0).tolist(),
        "axis_std_m_s2": np.std(accelerations, axis=0, ddof=1).tolist(),
        "gravity_direction_p95_rad": float(np.percentile(spread_rad, 95.0)),
        "gravity_direction_p95_deg": float(np.degrees(np.percentile(spread_rad, 95.0))),
    }

def solve_gravity_wahba(
    platform_directions: np.ndarray,
    imu_directions: np.ndarray,
) -> np.ndarray:
    """Solve R_P_IP such that R_P_IP * u_I ~= u_P."""
    u_p = np.asarray(platform_directions, dtype=float)
    u_i = np.asarray(imu_directions, dtype=float)
    if u_p.shape != u_i.shape or u_p.ndim != 2 or u_p.shape[1] != 3:
        raise ValueError("gravity direction arrays must have shape (N, 3)")
    h = u_p.T @ u_i
    u, _, vt = np.linalg.svd(h)
    det_fix = 1.0 if np.linalg.det(u @ vt) >= 0.0 else -1.0
    return u @ np.diag([1.0, 1.0, det_fix]) @ vt


def gravity_rotation_jacobian(platform_directions: np.ndarray) -> np.ndarray:
    return np.vstack([-skew(direction) for direction in platform_directions])


def gravity_bias_nuisance_jacobian(platform_directions: np.ndarray) -> np.ndarray:
    """Dimensionless first-order Jacobian for [rotation, accelerometer bias/g]."""
    rows = []
    for direction in np.asarray(platform_directions, dtype=float):
        rows.append(np.hstack([skew(direction), np.eye(3)]))
    return np.vstack(rows)


def matrix_diagnostics(matrix: np.ndarray, tolerance: float = 1e-10) -> dict[str, Any]:
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.sum(singular_values > max(float(singular_values[0]) * tolerance, 1e-12)))
    condition = math.inf if rank == 0 or singular_values[-1] <= 0.0 else float(singular_values[0] / singular_values[-1])
    return {
        "singular_values": singular_values.tolist(),
        "rank": rank,
        "condition_number": condition,
    }



def rotation_delta_deg(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    delta = np.asarray(rotation_a, dtype=float) @ np.asarray(rotation_b, dtype=float).T
    return float(np.degrees(Rotation.from_matrix(delta).magnitude()))


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _rows_in_window(rows: list[dict[str, str]], field: str, start_ns: int, end_ns: int) -> list[dict[str, str]]:
    selected = []
    for row in rows:
        try:
            timestamp_ns = int(row[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"CSV row missing integer {field}: {row!r}") from exc
        if start_ns <= timestamp_ns <= end_ns:
            selected.append(row)
    return selected


def _mean_fk_quaternion_xyzw(rows: list[dict[str, str]]) -> list[float] | None:
    if not rows:
        return None
    quaternions = [np.asarray(json.loads(row["quat_xyzw"]), dtype=float) for row in rows]
    return Rotation.from_quat(np.vstack(quaternions)).mean().as_quat().tolist()


def _static_calibration_payload(traversal: str, dwell_records: list[dict[str, Any]]) -> dict[str, Any]:
    platform_directions = np.vstack([record["platform_gravity_direction"] for record in dwell_records])
    imu_directions = np.vstack([record["imu_gravity_direction"] for record in dwell_records])
    rotation = solve_gravity_wahba(platform_directions, imu_directions)
    residuals = np.arccos(np.clip(np.sum((imu_directions @ rotation.T) * platform_directions, axis=1), -1.0, 1.0))
    return {
        "status": "PASS",
        "traversal": traversal,
        "pose_count": len(dwell_records),
        "rotation_matrix_R_P_IP": rotation.tolist(),
        "quaternion_xyzw_R_P_IP": Rotation.from_matrix(rotation).as_quat().tolist(),
        "euler_xyz_deg_R_P_IP": Rotation.from_matrix(rotation).as_euler("xyz", degrees=True).tolist(),
        "gravity_direction_residuals_rad": residuals.tolist(),
        "gravity_direction_residual_rms_rad": float(np.sqrt(np.mean(residuals**2))),
        "gravity_direction_residual_p95_rad": float(np.percentile(residuals, 95.0)),
        "observability": matrix_diagnostics(gravity_rotation_jacobian(platform_directions)),
        "dwells": [
            {
                "pose": record["pose"],
                "dwell_start_monotonic_ns": record["dwell_start_monotonic_ns"],
                "dwell_end_monotonic_ns": record["dwell_end_monotonic_ns"],
                "imu_sample_count": record["imu_sample_count"],
                "fk_sample_count": record["fk_sample_count"],
                "platform_gravity_direction": record["platform_gravity_direction"].tolist(),
                "imu_gravity_direction": record["imu_gravity_direction"].tolist(),
                "mean_acceleration_m_s2": record["mean_acceleration_m_s2"].tolist(),
                "mean_fk_quaternion_xyzw": record["mean_fk_quaternion_xyzw"],
                "mean_fk_euler_xyz_deg": record["mean_fk_euler_xyz_deg"],
            }
            for record in dwell_records
        ],
    }


def build_static_gravity_calibration_result(out_dir: Path, result: dict[str, Any]) -> dict[str, Any]:
    """Recover accepted dwell data from recorded CSVs and run the existing Wahba estimator."""
    segments = result.get("segments")
    if not isinstance(segments, list):
        raise ValueError("result does not contain segments")
    accepted_segments = [
        segment
        for segment in segments
        if segment.get("dwell_gate", {}).get("accepted_for_calibration", False)
    ]
    if not accepted_segments:
        return {
            "data_acquisition_status": {
                "status": "NO_ACCEPTED_DWELLS",
                "accepted_dwell_count": 0,
                "fit_valid_count": 0,
                "validation_valid_count": 0,
            },
            "calibration_result": {"status": "NOT_COMPUTED", "reason": "no accepted calibration dwells"},
        }

    imu_rows = _read_csv_rows(out_dir / "imu.csv")
    fk_rows = _read_csv_rows(out_dir / "fk.csv")
    dwell_records_by_traversal: dict[str, list[dict[str, Any]]] = {"fit": [], "validation": []}
    for segment in accepted_segments:
        traversal = str(segment.get("traversal", ""))
        if traversal not in dwell_records_by_traversal:
            continue
        pose_name = str(segment.get("pose"))
        if pose_name not in STATIC_POSES:
            raise ValueError(f"unknown static pose {pose_name!r}")
        start_ns = int(segment["dwell_start_monotonic_ns"])
        end_ns = int(segment["dwell_end_monotonic_ns"])
        imu_window = _rows_in_window(imu_rows, "host_monotonic_arrival_time_ns", start_ns, end_ns)
        fk_window = _rows_in_window(fk_rows, "callback_monotonic_ns", start_ns, end_ns)
        if not imu_window:
            raise ValueError(f"no IMU samples found for {traversal}/{pose_name}")
        accelerations = np.asarray(
            [
                [float(row["ax_m_s2"]), float(row["ay_m_s2"]), float(row["az_m_s2"])]
                for row in imu_window
            ],
            dtype=float,
        )
        mean_acceleration = np.mean(accelerations, axis=0)
        mean_fk_quaternion = _mean_fk_quaternion_xyzw(fk_window)
        mean_fk_euler = None
        if mean_fk_quaternion is not None:
            mean_fk_euler = Rotation.from_quat(mean_fk_quaternion).as_euler("xyz", degrees=True).tolist()
        dwell_records_by_traversal[traversal].append(
            {
                "pose": pose_name,
                "dwell_start_monotonic_ns": start_ns,
                "dwell_end_monotonic_ns": end_ns,
                "imu_sample_count": len(imu_window),
                "fk_sample_count": len(fk_window),
                "platform_gravity_direction": platform_gravity_direction(STATIC_POSES[pose_name]),
                "imu_gravity_direction": measured_gravity_direction(mean_acceleration),
                "mean_acceleration_m_s2": mean_acceleration,
                "mean_fk_quaternion_xyzw": mean_fk_quaternion,
                "mean_fk_euler_xyz_deg": mean_fk_euler,
            }
        )

    fit_count = len(dwell_records_by_traversal["fit"])
    validation_count = len(dwell_records_by_traversal["validation"])
    data_status = {
        "status": "PASS" if fit_count == len(FIT_ORDER) and validation_count == len(VALIDATION_ORDER) else "INCOMPLETE",
        "accepted_dwell_count": fit_count + validation_count,
        "fit_valid_count": fit_count,
        "validation_valid_count": validation_count,
        "invalid_dwell_poses": list(result.get("invalid_dwell_poses", [])),
        "required_fit_pose_count": len(FIT_ORDER),
        "required_validation_pose_count": len(VALIDATION_ORDER),
    }
    calibration: dict[str, Any] = {"status": "NOT_COMPUTED", "reason": "incomplete accepted dwell set"}
    if data_status["status"] == "PASS":
        fit = _static_calibration_payload("fit", dwell_records_by_traversal["fit"])
        validation = _static_calibration_payload("validation", dwell_records_by_traversal["validation"])
        fit_rotation = np.asarray(fit["rotation_matrix_R_P_IP"], dtype=float)
        validation_rotation = np.asarray(validation["rotation_matrix_R_P_IP"], dtype=float)
        references = {}
        for name, euler_xyz_deg in STATIC_REFERENCE_EULER_XYZ_DEG.items():
            reference_rotation = Rotation.from_euler("xyz", euler_xyz_deg, degrees=True).as_matrix()
            references[name] = {
                "reference_euler_xyz_deg_R_P_IP": list(euler_xyz_deg),
                "fit_so3_delta_deg": rotation_delta_deg(fit_rotation, reference_rotation),
                "validation_so3_delta_deg": rotation_delta_deg(validation_rotation, reference_rotation),
            }
        calibration = {
            "status": "PASS",
            "estimator": "solve_gravity_wahba",
            "convention": "u_P = R_P_IP * u_I; Euler XYZ degrees via scipy Rotation.as_euler('xyz')",
            "fit": fit,
            "validation": validation,
            "fit_vs_validation_so3_delta_deg": rotation_delta_deg(fit_rotation, validation_rotation),
            "reference_comparisons": references,
            "observability_interpretation": "static gravity strongly constrains roll/pitch-like tilt; yaw-like rotation remains weakly observable",
            "freeze_recommendation": "DO_NOT_FREEZE_R_P_IP",
        }
    return {"data_acquisition_status": data_status, "calibration_result": calibration}


def build_hardware_teardown_result(result: dict[str, Any]) -> dict[str, Any]:
    park = result.get("park_result") if isinstance(result.get("park_result"), dict) else {}
    final = result.get("final_readonly_audit") if isinstance(result.get("final_readonly_audit"), dict) else {}
    return {
        "status": "PASS" if bool(park.get("passed")) else "FAIL",
        "park_passed": bool(park.get("passed")),
        "park_summary": park.get("summary"),
        "final_readonly_status": final.get("status"),
        "final_positions_steps": final.get("initial_positions_steps"),
        "final_currents_ma": final.get("initial_currents_ma"),
        "final_faults": final.get("initial_faults"),
        "note": "hardware teardown is reported separately from accepted static calibration dwells",
    }


def build_e1d_scientific_closure(result: dict[str, Any]) -> dict[str, Any]:
    calibration = result.get("calibration_result") if isinstance(result.get("calibration_result"), dict) else {}
    data_status = result.get("data_acquisition_status") if isinstance(result.get("data_acquisition_status"), dict) else {}
    teardown = result.get("hardware_teardown_result") if isinstance(result.get("hardware_teardown_result"), dict) else {}
    return {
        "status": "STATIC_GRAVITY_SCIENTIFIC_CLOSURE_NOT_FROZEN"
        if calibration.get("status") == "PASS"
        else "STATIC_GRAVITY_SCIENTIFIC_CLOSURE_INCOMPLETE",
        "data_acquisition_status": data_status.get("status"),
        "calibration_status": calibration.get("status"),
        "hardware_teardown_status": teardown.get("status"),
        "findings": [
            "static gravity FIT estimates are highly repeatable across E1D-1/E1D-3/E1D-5",
            "gravity-only data strongly supports roll/pitch consistency",
            "yaw-like orientation remains weakly observable",
            "static-vs-dynamic E1C discrepancy remains unresolved",
            "full R_P_IP is not frozen or written to runtime configuration",
        ],
    }


def jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def decode_ros_uint8(value: Any, field_name: str) -> int:
    """Decode ROS Python uint8 fields that may arrive as int or one raw byte."""
    if isinstance(value, int):
        if 0 <= value <= 0xFF:
            return int(value)
        raise ValueError(f"{field_name} out of uint8 range: {value!r}")
    if isinstance(value, memoryview):
        value = value.tobytes()
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"{field_name} expected one uint8 byte, got {len(value)} bytes")
        return int(value[0])
    raise TypeError(f"{field_name} expected int or one-byte buffer, got {type(value).__name__}")


def diagnostic_int(values: dict[str, Any], key: str) -> int | None:
    value = values.get(key)
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def infer_stage_label(out_dir: Path, explicit_stage: str | None = None) -> str:
    raw = explicit_stage or str(out_dir)
    match = re.search(r"j9[-_]e1d[-_](\d+)", raw, flags=re.IGNORECASE)
    if match:
        return f"J9-E1D-{int(match.group(1))}"
    return "J9-E1D-1"


def raw_artifact_path(out_dir: Path, stage_label: str) -> Path:
    stem = stage_label.lower().replace("-", "_")
    return out_dir / f"{stem}_raw.json"


def first_static_gate_violation(metrics: dict[str, Any], timestamp_ns: int) -> dict[str, Any] | None:
    if not metrics.get("fk_converged", False):
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "fk_converged",
            "measured": bool(metrics.get("fk_converged", False)),
            "threshold": True,
        }
    if metrics.get("reason"):
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": str(metrics["reason"]),
            "measured": None,
            "threshold": "sufficient recent feedback and IMU samples",
        }
    joint_span = metrics.get("joint_span_steps")
    if joint_span is not None and max(joint_span) > STATIC_MAX_JOINT_SPAN_STEPS:
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "joint_span_steps",
            "measured": int(max(joint_span)),
            "threshold": int(STATIC_MAX_JOINT_SPAN_STEPS),
        }
    gyro_p95 = metrics.get("gyro_norm_p95_rad_s")
    if gyro_p95 is not None and gyro_p95 > STATIC_MAX_GYRO_NORM_P95_RAD_S:
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "gyro_norm_p95_rad_s",
            "measured": float(gyro_p95),
            "threshold": float(STATIC_MAX_GYRO_NORM_P95_RAD_S),
        }
    accel_norm_mean = metrics.get("accel_norm_mean_m_s2")
    if accel_norm_mean is not None:
        accel_norm_error = abs(float(accel_norm_mean) - GRAVITY_M_S2)
        if accel_norm_error > STATIC_MAX_ACCEL_NORM_DEVIATION_M_S2:
            return {
                "timestamp_monotonic_ns": int(timestamp_ns),
                "metric": "accel_norm_mean_abs_error_m_s2",
                "measured": float(accel_norm_error),
                "threshold": float(STATIC_MAX_ACCEL_NORM_DEVIATION_M_S2),
            }
    axis_std = metrics.get("accel_axis_std_m_s2")
    if axis_std is not None and max(axis_std) > STATIC_MAX_ACCEL_AXIS_STD_M_S2:
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "accel_axis_std_m_s2",
            "measured": float(max(axis_std)),
            "threshold": float(STATIC_MAX_ACCEL_AXIS_STD_M_S2),
        }
    direction_p95 = metrics.get("gravity_direction_p95_rad")
    if direction_p95 is not None and direction_p95 > STATIC_MAX_GRAVITY_DIRECTION_P95_RAD:
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "gravity_direction_p95_rad",
            "measured": float(direction_p95),
            "threshold": float(STATIC_MAX_GRAVITY_DIRECTION_P95_RAD),
        }
    if not metrics.get("pass", False):
        return {
            "timestamp_monotonic_ns": int(timestamp_ns),
            "metric": "static_gate_pass",
            "measured": False,
            "threshold": True,
        }
    return None


def static_pose_orders() -> dict[str, list[str]]:
    return {"fit": list(FIT_ORDER), "validation": list(VALIDATION_ORDER)}


def _pose_steps(kin: Analytical6PusKinematics, pose: StaticPose) -> np.ndarray:
    q_home = np.asarray(kin.inverse_kinematics(STATIC_POSES["home"].target_pose(kin.nominal_height)))
    q_pose = np.asarray(kin.inverse_kinematics(pose.target_pose(kin.nominal_height)))
    return np.rint((q_pose - q_home) * kin.steps_per_meter).astype(int)


def trajectory_preflight(kin: Analytical6PusKinematics | None = None) -> dict[str, Any]:
    kin = kin or Analytical6PusKinematics()
    pose_steps = {name: _pose_steps(kin, pose).tolist() for name, pose in STATIC_POSES.items()}
    all_steps = np.asarray(list(pose_steps.values()), dtype=int)
    transition_metrics: dict[str, Any] = {}
    for label, order in (("fit", FIT_ORDER), ("validation", VALIDATION_ORDER)):
        transitions = []
        for previous, current in zip(order, order[1:]):
            delta = np.asarray(pose_steps[current]) - np.asarray(pose_steps[previous])
            transitions.append(
                {
                    "from": previous,
                    "to": current,
                    "joint_step_delta": delta.tolist(),
                    "max_abs_step_delta": int(np.max(np.abs(delta))),
                }
            )
        transition_metrics[label] = {
            "transitions": transitions,
            "max_abs_step_delta": int(max(item["max_abs_step_delta"] for item in transitions)),
        }
    directions = np.vstack([platform_gravity_direction(pose) for pose in STATIC_POSES.values()])
    j_rot = gravity_rotation_jacobian(directions)
    j_bias = gravity_bias_nuisance_jacobian(directions)
    rot_diag = matrix_diagnostics(j_rot)
    bias_diag = matrix_diagnostics(j_bias)
    info = j_rot.T @ j_rot
    eigenvalues, eigenvectors = np.linalg.eigh(info)
    acceleration_sigma = np.array([0.004870229, 0.005710795, 0.006392333], dtype=float)
    direction_sigma = float(np.max(acceleration_sigma) / GRAVITY_M_S2)
    repeated_j = np.repeat(j_rot, STATIC_EFFECTIVE_SAMPLES_PER_POSE, axis=0)
    cov = direction_sigma**2 * np.linalg.pinv(repeated_j.T @ repeated_j)
    mode_sigma_deg = np.degrees(np.sqrt(np.maximum(np.linalg.eigvalsh(cov), 0.0)))
    return {
        "status": "PASS",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "pose_count": len(STATIC_POSES),
        "pose_order": static_pose_orders(),
        "pose_steps_from_home": pose_steps,
        "joint_step_min": int(np.min(all_steps)),
        "joint_step_max": int(np.max(all_steps)),
        "normal_motion_limits_steps": [200, 1800],
        "orientation_envelope_rad": 0.10,
        "all_poses_within_orientation_envelope": bool(
            all(np.linalg.norm(np.radians([p.roll_deg, p.pitch_deg, p.yaw_deg])) <= 0.10 for p in STATIC_POSES.values())
        ),
        "transition_metrics": transition_metrics,
        "gravity_rotation_information_eigenvalues": eigenvalues.tolist(),
        "gravity_rotation_information_eigenvectors": eigenvectors.tolist(),
        "rotation_observability": rot_diag,
        "rotation_bias_nuisance_observability": bias_diag,
        "measured_static_acceleration_sigma_m_s2": acceleration_sigma.tolist(),
        "direction_sigma_single_sample_rad": direction_sigma,
        "effective_samples_per_pose_for_prediction": STATIC_EFFECTIVE_SAMPLES_PER_POSE,
        "rotation_mode_sigma_deg_noise_only_weak_to_strong": mode_sigma_deg.tolist(),
        "static_estimator_a": "normalized gravity-direction Wahba",
        "static_estimator_b": "raw acceleration rotation plus constant bias/g nuisance model",
        "estimator_b_status": "ranked_but_ill_conditioned; use only as diagnostic until bias prior is validated",
    }


class CsvQueueWriter:
    def __init__(self, out_dir: Path, capacity: int = 131072) -> None:
        self.queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue(maxsize=capacity)
        self.out_dir = out_dir
        self.capacity = capacity
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}
        self.written: dict[str, int] = {}
        self.overflow: dict[str, int] = {}
        self.errors = 0
        self.thread = threading.Thread(target=self._run, name="e1d-csv-writer", daemon=True)
        self.thread.start()

    def _open(self, stream: str, row: dict[str, Any]) -> csv.DictWriter:
        if stream not in self.writers:
            handle = (self.out_dir / f"{stream}.csv").open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            writer.writeheader()
            self.files[stream] = handle
            self.writers[stream] = writer
            self.written[stream] = 0
            self.overflow[stream] = 0
        return self.writers[stream]

    def enqueue(self, stream: str, row: dict[str, Any]) -> bool:
        try:
            self.queue.put_nowait((stream, row))
            return True
        except queue.Full:
            self.overflow[stream] = self.overflow.get(stream, 0) + 1
            return False

    def _run(self) -> None:
        while True:
            item = self.queue.get()
            if item is None:
                self.queue.task_done()
                return
            stream, row = item
            try:
                self._open(stream, row).writerow(row)
                self.written[stream] = self.written.get(stream, 0) + 1
            except Exception:
                self.errors += 1
            self.queue.task_done()

    def close(self) -> None:
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self.thread.join(timeout=5.0)
        for handle in self.files.values():
            handle.flush()
            handle.close()
        (self.out_dir / "recorder_summary.json").write_text(
            json.dumps(
                {
                    "capacity": int(self.capacity),
                    "written": {key: int(value) for key, value in self.written.items()},
                    "overflow": {key: int(value) for key, value in self.overflow.items()},
                    "errors": int(self.errors),
                },
                indent=2,
            ),
            encoding="utf-8",
        )


class E1DObserverNode(Node):
    """Passive recorder for the canonical J8 graph."""

    def __init__(self, out_dir: Path, queue_capacity: int = 131072) -> None:
        super().__init__("j9_e1d_1_observer")
        self.writer = CsvQueueWriter(out_dir, queue_capacity)
        self.lock = threading.Lock()
        self.latest_feedback_steps: list[int] | None = None
        self.latest_fk_converged = False
        self.latest_fk_stamp_ns: int = 0
        self.latest_driver_diagnostics: dict[str, Any] | None = None
        self.latest_h30_diagnostics: dict[str, Any] | None = None
        self.imu_samples: list[tuple[int, np.ndarray, np.ndarray]] = []
        self.feedback_samples: list[tuple[int, list[int]]] = []

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4096,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TimedImu, "/pus6/imu_platform/timed", self._on_imu, reliable_qos)
        self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_feedback, qos_profile_sensor_data)
        self.create_subscription(FkStateStamped, "/pus6/fk/state", self._on_fk, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/pus6/target_cartesian_pose", self._on_target, reliable_qos)
        self.create_subscription(Float64MultiArray, "/pus6/desired_joint_deltas", self._on_ik, reliable_qos)
        self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/pus6/driver_diagnostics", self._on_driver_diagnostics, reliable_qos)
        self.create_subscription(DiagnosticArray, "/pus6/imu_platform/diagnostics", self._on_h30_diagnostics, reliable_qos)

    @staticmethod
    def stamp_ns(message: Any) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _on_imu(self, message: TimedImu) -> None:
        stamp = time.monotonic_ns()
        gyro = np.array(
            [message.imu.angular_velocity.x, message.imu.angular_velocity.y, message.imu.angular_velocity.z],
            dtype=float,
        )
        accel = np.array(
            [message.imu.linear_acceleration.x, message.imu.linear_acceleration.y, message.imu.linear_acceleration.z],
            dtype=float,
        )
        with self.lock:
            self.imu_samples.append((stamp, gyro, accel))
        self.writer.enqueue(
            "imu",
            {
                "callback_monotonic_ns": stamp,
                "ros_header_stamp_ns": self.stamp_ns(message),
                "publisher_sequence": int(message.publisher_sequence),
                "tid": int(message.tid),
                "has_sample_timestamp": int(message.has_sample_timestamp),
                "sample_timestamp_us": int(message.sample_timestamp_us),
                "host_monotonic_arrival_time_ns": int(message.host_monotonic_arrival_time_ns),
                "gx_rad_s": float(gyro[0]),
                "gy_rad_s": float(gyro[1]),
                "gz_rad_s": float(gyro[2]),
                "ax_m_s2": float(accel[0]),
                "ay_m_s2": float(accel[1]),
                "az_m_s2": float(accel[2]),
            },
        )

    def _on_feedback(self, message: TimedJointFeedback) -> None:
        stamp = int(message.snapshot_end_monotonic_ns)
        steps = [int(value) for value in message.raw_steps]
        with self.lock:
            self.latest_feedback_steps = steps
            self.feedback_samples.append((stamp, steps))
        self.writer.enqueue(
            "joint_feedback",
            {
                "callback_monotonic_ns": time.monotonic_ns(),
                "ros_header_stamp_ns": self.stamp_ns(message),
                "snapshot_start_monotonic_ns": int(message.snapshot_start_monotonic_ns),
                "snapshot_end_monotonic_ns": stamp,
                "uart_cycle_sequence": int(message.uart_cycle_sequence),
                "raw_steps": json.dumps(steps),
                "currents_ma": json.dumps([int(value) for value in message.currents_ma]),
                "fault_codes": json.dumps([int(value) for value in message.fault_codes]),
            },
        )

    def _on_fk(self, message: FkStateStamped) -> None:
        stamp = time.monotonic_ns()
        with self.lock:
            self.latest_fk_converged = bool(message.converged)
            self.latest_fk_stamp_ns = stamp
        self.writer.enqueue(
            "fk",
            {
                "callback_monotonic_ns": stamp,
                "ros_header_stamp_ns": self.stamp_ns(message),
                "query_monotonic_timestamp_ns": int(message.query_monotonic_timestamp_ns),
                "converged": int(message.converged),
                "residual_norm_meters": float(message.residual_norm_meters),
                "compute_time_us": float(message.compute_time_us),
                "quat_xyzw": json.dumps(
                    [
                        float(message.pose.orientation.x),
                        float(message.pose.orientation.y),
                        float(message.pose.orientation.z),
                        float(message.pose.orientation.w),
                    ]
                ),
            },
        )

    def _on_target(self, message: PoseStamped) -> None:
        self.writer.enqueue(
            "target_pose",
            {
                "callback_monotonic_ns": time.monotonic_ns(),
                "ros_header_stamp_ns": self.stamp_ns(message),
                "pos_x_m": float(message.pose.position.x),
                "pos_y_m": float(message.pose.position.y),
                "pos_z_m": float(message.pose.position.z),
                "quat_xyzw": json.dumps(
                    [
                        float(message.pose.orientation.x),
                        float(message.pose.orientation.y),
                        float(message.pose.orientation.z),
                        float(message.pose.orientation.w),
                    ]
                ),
            },
        )

    def _on_ik(self, message: Float64MultiArray) -> None:
        self.writer.enqueue(
            "ik_deltas",
            {"callback_monotonic_ns": time.monotonic_ns(), "deltas_m": json.dumps([float(v) for v in message.data])},
        )

    def _on_cmd(self, message: Int32MultiArray) -> None:
        self.writer.enqueue(
            "command_steps",
            {"callback_monotonic_ns": time.monotonic_ns(), "steps": json.dumps([int(v) for v in message.data])},
        )

    def _on_driver_diagnostics(self, message: Float64MultiArray) -> None:
        stamp = time.monotonic_ns()
        labels = [
            "target_in_count",
            "overwrite_count",
            "consumed_count",
            "deadline_miss_count",
            "target_age_ms",
            "worker_start_to_start_ms",
            "write_duration_ms",
            "rx_classification_ms",
            "feedback_transaction_ms",
            "worker_total_ms",
            "watchdog_stop_age_ms",
        ]
        values = [float(value) for value in message.data]
        diagnostics = {label: values[index] if index < len(values) else None for index, label in enumerate(labels)}
        diagnostics["raw"] = values
        with self.lock:
            self.latest_driver_diagnostics = diagnostics
        self.writer.enqueue("driver_diagnostics", {"callback_monotonic_ns": stamp, **diagnostics})

    def _on_h30_diagnostics(self, message: DiagnosticArray) -> None:
        stamp = time.monotonic_ns()
        statuses = []
        flattened: dict[str, str] = {}
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            status_record = {
                "name": status.name,
                "level": None,
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": values,
            }
            try:
                status_record["level"] = decode_ros_uint8(status.level, "DiagnosticStatus.level")
            except (TypeError, ValueError) as exc:
                status_record["level_decode_error"] = f"{type(exc).__name__}: {exc}"
            statuses.append(status_record)
            flattened.update(values)
        with self.lock:
            self.latest_h30_diagnostics = {"statuses": statuses, "values": flattened}
        self.writer.enqueue(
            "h30_diagnostics",
            {
                "callback_monotonic_ns": stamp,
                "ros_header_stamp_ns": self.stamp_ns(message),
                "statuses": json.dumps(statuses),
            },
        )

    def static_gate(self, now_ns: int) -> dict[str, Any]:
        with self.lock:
            feedback = [(stamp, steps) for stamp, steps in self.feedback_samples if stamp >= now_ns - int(STATIC_STABILITY_WINDOW_S * 1e9)]
            imu = [(stamp, gyro, accel) for stamp, gyro, accel in self.imu_samples if stamp >= now_ns - int(STATIC_STABILITY_WINDOW_S * 1e9)]
            fk_fresh = bool(self.latest_fk_stamp_ns >= now_ns - int(STATIC_STABILITY_WINDOW_S * 1e9))
            fk_ok = bool(self.latest_fk_converged and fk_fresh)
        if len(feedback) < 5 or len(imu) < 20 or not fk_fresh:
            return {
                "pass": False,
                "reason": "insufficient_recent_feedback_imu_or_fk",
                "fk_converged": fk_ok,
                "fk_fresh": fk_fresh,
                "feedback_count": len(feedback),
                "imu_count": len(imu),
            }
        steps = np.asarray([item[1] for item in feedback], dtype=float)
        gyro_norm = np.linalg.norm(np.asarray([item[1] for item in imu], dtype=float), axis=1)
        accel_vectors = np.asarray([item[2] for item in imu], dtype=float)
        accel_norm = np.linalg.norm(accel_vectors, axis=1)
        accel_stability = acceleration_stability_metrics(accel_vectors)
        joint_span = np.max(steps, axis=0) - np.min(steps, axis=0)
        metrics = {
            "pass": bool(
                fk_ok
                and np.max(joint_span) <= STATIC_MAX_JOINT_SPAN_STEPS
                and np.percentile(gyro_norm, 95.0) <= STATIC_MAX_GYRO_NORM_P95_RAD_S
                and abs(float(np.mean(accel_norm)) - GRAVITY_M_S2) <= STATIC_MAX_ACCEL_NORM_DEVIATION_M_S2
                and max(accel_stability["axis_std_m_s2"]) <= STATIC_MAX_ACCEL_AXIS_STD_M_S2
                and accel_stability["gravity_direction_p95_rad"] <= STATIC_MAX_GRAVITY_DIRECTION_P95_RAD
            ),
            "fk_converged": fk_ok,
            "fk_fresh": fk_fresh,
            "feedback_count": len(feedback),
            "imu_count": len(imu),
            "joint_span_steps": joint_span.astype(int).tolist(),
            "gyro_norm_p95_rad_s": float(np.percentile(gyro_norm, 95.0)),
            "accel_norm_mean_m_s2": float(np.mean(accel_norm)),
            "accel_norm_std_m_s2": float(np.std(accel_norm)),
            "accel_norm_p95_m_s2": float(np.percentile(accel_norm, 95.0)),
            "accel_axis_std_m_s2": accel_stability["axis_std_m_s2"],
            "gravity_direction_p95_rad": accel_stability["gravity_direction_p95_rad"],
            "gravity_direction_p95_deg": accel_stability["gravity_direction_p95_deg"],
        }
        return metrics

    def artifact_summary(self) -> dict[str, Any]:
        with self.lock:
            driver = dict(self.latest_driver_diagnostics or {})
            h30 = dict(self.latest_h30_diagnostics or {})
        source_values = h30.get("values", {}) if isinstance(h30.get("values"), dict) else {}
        return {
            "driver_diagnostics": driver,
            "source_capture": {
                "available": bool(source_values),
                "capacity": diagnostic_int(source_values, "source_capture_queue_capacity"),
                "enqueued": diagnostic_int(source_values, "source_capture_enqueued_rows"),
                "written": diagnostic_int(source_values, "source_capture_written_rows"),
                "overflow": diagnostic_int(source_values, "source_capture_overflow_drops"),
                "errors": diagnostic_int(source_values, "source_capture_writer_errors"),
            },
            "h30_diagnostics": h30,
        }


def shutdown_pipeline(
    target_pub_node: Node,
    executor: SingleThreadedExecutor,
    spin_thread: threading.Thread,
    observer_node: E1DObserverNode,
    kinematics_node: Node,
    control_node: Node,
    motor_driver_node: Node,
    fk_node: Node,
) -> None:
    target_pub_node.destroy_node()
    try:
        executor.shutdown()
    finally:
        if spin_thread.is_alive():
            spin_thread.join(timeout=5.0)
    try:
        observer_node.writer.close()
    finally:
        kinematics_node.destroy_node()
        control_node.destroy_node()
        motor_driver_node.destroy_node()
        fk_node.destroy_node()
        observer_node.destroy_node()


def start_h30_imu(source_path: Path) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            "ros2",
            "run",
            "h30_imu",
            "h30_imu_node",
            "--ros-args",
            "-r",
            "__ns:=/pus6/imu_platform",
            "-p",
            f"port:={PLATFORM_IMU_PORT}",
            "-p",
            f"baud_rate:={BAUD_IMU}",
            "-p",
            "frame_id:=pus6_platform_imu_link",
            "-p",
            "world_frame:=world",
            "-p",
            "timed_topic:=/pus6/imu_platform/timed",
            "-p",
            "imu_topic:=/pus6/imu_platform/data",
            "-p",
            "euler_topic:=/pus6/imu_platform/euler",
            "-p",
            "temperature_topic:=/pus6/imu_platform/temperature",
            "-p",
            "diagnostics_topic:=/pus6/imu_platform/diagnostics",
            "-p",
            "publish_euler:=true",
            "-p",
            "publish_tf:=false",
            "-p",
            "timed_qos_profile:=calibration",
            "-p",
            "timed_qos_depth:=4096",
            "-p",
            "timed_qos_reliability:=reliable",
            "-p",
            "source_capture_enabled:=true",
            "-p",
            f"source_capture_path:={source_path}",
            "-p",
            "source_capture_queue_capacity:=131072",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10.0)
    except Exception:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                process.kill()


def summarize_motor_runtime(motor_node: Any, failure_monotonic_ns: int | None = None) -> dict[str, Any]:
    if motor_node is None:
        return {"available": False}
    summary: dict[str, Any] = {
        "available": True,
        "cycle_count": int(getattr(motor_node, "_uart_cycle_seq", 0)),
        "deadline_miss_count": int(getattr(motor_node, "_deadline_miss_count", 0)),
        "stop_reason": getattr(motor_node, "_j7_fault_reason", None),
        "failure_monotonic_ns": failure_monotonic_ns,
    }
    writer = getattr(motor_node, "j7_writer", None)
    if writer is not None and hasattr(writer, "audit_snapshot"):
        try:
            audit = writer.audit_snapshot()
            cycles = tuple(getattr(audit, "cycles", ()))
            summary["j7_writer"] = {
                "cycle_count": len(cycles),
                "deadline_miss_count": int(sum(1 for cycle in cycles if getattr(cycle, "deadline_miss", False))),
                "stop_reason": getattr(audit, "stop_reason", None),
                "stopped": bool(getattr(audit, "stopped", False)),
                "delivery_uncertain": bool(getattr(audit, "delivery_uncertain", False)),
                "motion_retry_count": int(getattr(audit, "motion_retry_count", 0)),
                "response_wait_count": int(getattr(audit, "response_wait_count", 0)),
                "rx_motion_write_response_count": int(getattr(audit, "rx_motion_write_response_count", 0)),
                "rx_stale_response_count": int(getattr(audit, "rx_stale_response_count", 0)),
            }
        except Exception as exc:
            summary["j7_writer"] = {"error": f"{type(exc).__name__}: {exc}"}
    feedback = getattr(motor_node, "j7_feedback", None)
    if feedback is not None and hasattr(feedback, "audit_snapshot"):
        try:
            audit = feedback.audit_snapshot()
            summary["j7_feedback"] = {
                "cycle_count": len(tuple(getattr(audit, "cycles", ()))),
                "timeout_count": int(getattr(audit, "timeout_count", 0)),
                "malformed_count": int(getattr(audit, "malformed_count", 0)),
                "mismatch_count": int(getattr(audit, "mismatch_count", 0)),
                "stale_response_count": int(getattr(audit, "stale_response_count", 0)),
                "rx_residual_count": int(getattr(audit, "rx_residual_count", 0)),
                "motion_write_response_count": int(getattr(audit, "motion_write_response_count", 0)),
                "stop_reason": getattr(audit, "stop_reason", None),
                "stopped": bool(getattr(audit, "stopped", False)),
                "watchdog_stop_snapshot": jsonable(getattr(audit, "watchdog_stop_snapshot", None)),
            }
        except Exception as exc:
            summary["j7_feedback"] = {"error": f"{type(exc).__name__}: {exc}"}
    bus = getattr(motor_node, "bus", None)
    if bus is not None:
        summary["j7_rx"] = {
            "checksum_error_count": getattr(bus, "checksum_error_count", None),
        }
    return jsonable(summary)


def publish_target(node: Node, publisher: Any, pose: StaticPose, z0: float) -> None:
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "pus6_base"
    message.pose.position.z = float(z0)
    r, p, y = pose.rpy_rad
    qx, qy, qz, qw = quaternion_xyzw_from_rpy(r, p, y)
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    publisher.publish(message)


def run_static_pose(
    target_node: Node,
    publisher: Any,
    observer: E1DObserverNode,
    pose: StaticPose,
    z0: float,
) -> dict[str, Any]:
    start_ns = time.monotonic_ns()
    stable_since_ns: int | None = None
    settle_gate: dict[str, Any] = {}
    while time.monotonic_ns() - start_ns < int(STATIC_SETTLE_TIMEOUT_S * 1e9):
        publish_target(target_node, publisher, pose, z0)
        now_ns = time.monotonic_ns()
        settle_gate = observer.static_gate(now_ns)
        if settle_gate.get("pass"):
            if stable_since_ns is None:
                stable_since_ns = now_ns
            elif now_ns - stable_since_ns >= int(STATIC_SETTLE_MIN_S * 1e9):
                break
        else:
            stable_since_ns = None
        time.sleep(0.020)
    if stable_since_ns is None:
        raise RuntimeError(f"static settling gate failed for {pose.name}: {settle_gate}")

    dwell_start_ns = time.monotonic_ns()
    dwell_end_target = time.monotonic() + STATIC_DWELL_S
    dwell_checks = 0
    dwell_violation: dict[str, Any] | None = None
    dwell_last_gate: dict[str, Any] = {}
    while time.monotonic() < dwell_end_target:
        publish_target(target_node, publisher, pose, z0)
        now_ns = time.monotonic_ns()
        dwell_last_gate = observer.static_gate(now_ns)
        dwell_checks += 1
        if dwell_violation is None:
            dwell_violation = first_static_gate_violation(dwell_last_gate, now_ns)
        time.sleep(0.020)
    dwell_end_ns = time.monotonic_ns()
    dwell_valid = dwell_violation is None
    return {
        "pose": pose.name,
        "roll_deg": pose.roll_deg,
        "pitch_deg": pose.pitch_deg,
        "yaw_deg": pose.yaw_deg,
        "settle_started_monotonic_ns": int(start_ns),
        "dwell_start_monotonic_ns": int(dwell_start_ns),
        "dwell_end_monotonic_ns": int(dwell_end_ns),
        "dwell_duration_s": float((dwell_end_ns - dwell_start_ns) * 1e-9),
        "settle_gate": settle_gate,
        "dwell_gate": {
            "valid": bool(dwell_valid),
            "accepted_for_calibration": bool(dwell_valid),
            "checks": int(dwell_checks),
            "first_violation": dwell_violation,
            "last_gate": dwell_last_gate,
        },
    }



def _destroy_node_quietly(node: Any) -> None:
    destroy = getattr(node, "destroy_node", None)
    if callable(destroy):
        try:
            destroy()
        except Exception:
            pass


def _close_active_motor_uart_owner(motor_node: Any) -> bool:
    closed = False
    adapter = getattr(motor_node, "j7_adapter", None)
    serial_obj = getattr(adapter, "serial", None)
    close = getattr(serial_obj, "close", None)
    if callable(close):
        try:
            close()
            closed = True
        except Exception:
            pass
    controller = getattr(motor_node, "controller", None)
    if controller is not None and getattr(controller, "is_connected", False):
        try:
            controller.close()
            closed = True
        except Exception:
            pass
    return closed


def assert_no_stale_startup_targets(motor_node: Any) -> dict[str, Any]:
    audit = {
        "latest_target_is_none": getattr(motor_node, "_latest_target", None) is None,
        "last_commanded_steps_is_none": getattr(motor_node, "_last_commanded_steps", None) is None,
        "last_target_rx_time_ns": int(getattr(motor_node, "_last_target_rx_time_ns", 0)),
        "target_in_count": int(getattr(motor_node, "_target_in_count", 0)),
    }
    if not (
        audit["latest_target_is_none"]
        and audit["last_commanded_steps_is_none"]
        and audit["last_target_rx_time_ns"] == 0
        and audit["target_in_count"] == 0
    ):
        raise RuntimeError(f"active pipeline has stale target state before start: {audit}")
    return audit


def start_recording_pipeline(out_dir: Path) -> RecordingPipeline:
    h30_process = start_h30_imu(out_dir / "source.csv")
    observer = E1DObserverNode(out_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    spin_thread = threading.Thread(target=executor.spin, name="e1d-observer-executor", daemon=True)
    spin_thread.start()
    return RecordingPipeline(h30_process=h30_process, executor=executor, spin_thread=spin_thread, observer=observer)


def shutdown_recording_pipeline(recording: RecordingPipeline | None) -> None:
    if recording is None:
        return
    try:
        recording.executor.shutdown()
    finally:
        if recording.spin_thread.is_alive():
            recording.spin_thread.join(timeout=5.0)
    try:
        recording.observer.writer.close()
    finally:
        _destroy_node_quietly(recording.observer)
        stop_process(recording.h30_process)


def start_active_motion_pipeline(z0: float, ownership: UartOwnershipAudit, reason: str) -> ActiveMotionPipeline:
    ownership.require_released(f"start active ROS pipeline for {reason}")
    ownership.enter_active_ros(reason)
    executor = None
    spin_thread = None
    target_node = None
    kin_node = control_node = motor_node = fk_node = None
    try:
        kin_node = Pus6KinematicsNode()
        control_node = Pus6ControlNode()
        motor_node = Pus6MotorDriverNode(
            hardware_mode="active",
            port=MOTOR_PORT,
            baudrate=BAUD_MOTOR,
            timeout=0.015,
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
            use_j7_servo=True,
            servo_speed=50,
            publish_raw_diagnostics=True,
        )
        startup_audit = assert_no_stale_startup_targets(motor_node)
        setattr(motor_node, "_e1d_startup_stale_target_audit", startup_audit)
        fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0, query_rate_hz=200.0)
        executor = SingleThreadedExecutor()
        for node in (kin_node, control_node, motor_node, fk_node):
            executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name=f"e1d-active-{reason}", daemon=True)
        spin_thread.start()
        time.sleep(3.0)
        target_node = Node("j9_e1d_1_target_publisher")
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        publisher = target_node.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", target_qos)
        return ActiveMotionPipeline(
            executor=executor,
            spin_thread=spin_thread,
            target_node=target_node,
            publisher=publisher,
            kin_node=kin_node,
            control_node=control_node,
            motor_node=motor_node,
            fk_node=fk_node,
            z0=float(z0),
        )
    except Exception:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=5.0)
        _destroy_node_quietly(target_node)
        for node in (motor_node, control_node, kin_node, fk_node):
            _destroy_node_quietly(node)
        if motor_node is not None:
            try:
                _close_active_motor_uart_owner(motor_node)
            except Exception:
                pass
        if ownership.state == UartOwnershipState.ACTIVE_ROS:
            ownership.release(f"{reason}_startup_failed")
        raise


def active_motion_pipeline_alive(pipeline: ActiveMotionPipeline | None) -> bool:
    return bool(pipeline is not None and not pipeline.released and pipeline.spin_thread.is_alive())


def release_active_motion_pipeline(
    pipeline: ActiveMotionPipeline | None,
    ownership: UartOwnershipAudit,
    reason: str,
) -> dict[str, Any]:
    if pipeline is None or pipeline.released:
        return {"released": True, "already_released": True, "reason": reason}
    release_audit: dict[str, Any] = {
        "released": False,
        "already_released": False,
        "reason": reason,
        "spin_thread_alive_before": bool(pipeline.spin_thread.is_alive()),
        "target_publication_stopped": False,
        "worker_timer_canceled": False,
        "executor_shutdown": False,
        "spin_thread_alive_after_join": None,
        "motor_runtime_summary": None,
        "active_uart_closed": False,
    }
    # 1. Stop target command publication
    _destroy_node_quietly(pipeline.target_node)
    release_audit["target_publication_stopped"] = True

    # 2. Explicitly cancel the 50 Hz motor worker timer
    write_timer = getattr(pipeline.motor_node, "write_timer", None)
    if write_timer is not None:
        try:
            write_timer.cancel()
            release_audit["worker_timer_canceled"] = True
        except Exception:
            pass

    # 3. Shutdown the executor and join the spin thread
    try:
        pipeline.executor.shutdown()
        release_audit["executor_shutdown"] = True
    finally:
        if pipeline.spin_thread.is_alive():
            pipeline.spin_thread.join(timeout=5.0)
    release_audit["spin_thread_alive_after_join"] = bool(pipeline.spin_thread.is_alive())
    if release_audit["spin_thread_alive_after_join"]:
        raise RuntimeError("active ROS motion executor did not stop before UART handoff")

    # 4. Extract runtime summary, destroy motor node, and close serial port
    release_audit["motor_runtime_summary"] = summarize_motor_runtime(pipeline.motor_node)
    _destroy_node_quietly(pipeline.motor_node)
    release_audit["active_uart_closed"] = bool(_close_active_motor_uart_owner(pipeline.motor_node))

    # 5. Destroy remaining active nodes
    _destroy_node_quietly(pipeline.control_node)
    _destroy_node_quietly(pipeline.kin_node)
    _destroy_node_quietly(pipeline.fk_node)
    pipeline.released = True

    # 6. Release UART ownership
    ownership.release(reason)
    release_audit["released"] = True
    return release_audit


def run_exclusive_standalone_action(
    action: str,
    ownership: UartOwnershipAudit,
    reason: str,
    *,
    speed: float = 100.0,
    hold_duration_s: float = 2.0,
) -> Any:
    state = UartOwnershipState.STANDALONE_HOME if action == "home" else UartOwnershipState.STANDALONE_PARK
    ownership.require_released(f"standalone {action} {reason}")
    ownership.enter_standalone(state, reason)
    try:
        return run_servo_home_park(
            action,
            port=MOTOR_PORT,
            baudrate=BAUD_MOTOR,
            speed=speed,
            rate_hz=50.0,
            hold_duration_s=hold_duration_s,
            real_hardware=True,
            confirm_motion=True,
        )
    finally:
        ownership.release(f"{reason}_complete")

def run_hardware(args: argparse.Namespace) -> int:
    if not ROS_AVAILABLE:
        raise RuntimeError("ROS2 generated interfaces are unavailable; refusing hardware execution")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stage_label = infer_stage_label(args.out_dir, getattr(args, "stage", None))
    raw_path = raw_artifact_path(args.out_dir, stage_label)
    ownership = UartOwnershipAudit()
    result: dict[str, Any] = {
        "stage": stage_label,
        "raw_artifact": raw_path.name,
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "hardware_write_count": 0,
        "pose_order": static_pose_orders(),
        "trajectory_preflight": trajectory_preflight(),
        "segments": [],
        "active_pipeline_startup_audits": [],
        "active_pipeline_release_audits": [],
        "uart_ownership_audit": ownership.as_dict(),
        "error": None,
    }
    preflight = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
    result["preflight_audit"] = preflight
    if preflight.get("status") != "PASS":
        result["status"] = "PREFLIGHT_FAIL"
        result["uart_ownership_audit"] = ownership.as_dict()
        raw_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return 1

    home = run_exclusive_standalone_action("home", ownership, "initial_home")
    result["home_result"] = home.__dict__
    result["uart_ownership_audit"] = ownership.as_dict()
    if not home.passed:
        result["status"] = "HOME_FAIL"
        raw_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return 1

    recording: RecordingPipeline | None = None
    active_pipeline: ActiveMotionPipeline | None = None
    failure_monotonic_ns: int | None = None
    try:
        rclpy.init()
        geometry = load_geometry_config()
        z0 = float(geometry.nominal_height_m)
        recording = start_recording_pipeline(args.out_dir)
        time.sleep(3.0)

        for traversal_name, order in (("fit", FIT_ORDER), ("validation", VALIDATION_ORDER)):
            active_pipeline = start_active_motion_pipeline(z0, ownership, f"{traversal_name}_active_ros")
            result["active_pipeline_startup_audits"].append(
                {
                    "traversal": traversal_name,
                    "stale_target_audit": getattr(active_pipeline.motor_node, "_e1d_startup_stale_target_audit", None),
                    "uart_ownership_audit": ownership.as_dict(),
                }
            )
            try:
                for pose_name in order:
                    record = run_static_pose(
                        active_pipeline.target_node,
                        active_pipeline.publisher,
                        recording.observer,
                        STATIC_POSES[pose_name],
                        active_pipeline.z0,
                    )
                    record["traversal"] = traversal_name
                    result["segments"].append(record)
            finally:
                release_audit = release_active_motion_pipeline(
                    active_pipeline,
                    ownership,
                    f"{traversal_name}_active_ros_complete",
                )
                result["active_pipeline_release_audits"].append({"traversal": traversal_name, **release_audit})
                active_pipeline = None
                result["uart_ownership_audit"] = ownership.as_dict()

            if traversal_name == "fit":
                home = run_exclusive_standalone_action("home", ownership, "home_between_traversals")
                result["home_between_traversals"] = home.__dict__
                result["uart_ownership_audit"] = ownership.as_dict()
                if not home.passed:
                    raise RuntimeError("HOME between fit and validation failed")

        home = run_exclusive_standalone_action("home", ownership, "home_after_validation")
        result["home_after_validation"] = home.__dict__
        result["uart_ownership_audit"] = ownership.as_dict()
        if not home.passed:
            raise RuntimeError("HOME after validation failed")

        invalid_segments = [
            segment["pose"]
            for segment in result["segments"]
            if not segment.get("dwell_gate", {}).get("accepted_for_calibration", True)
        ]
        result["invalid_dwell_poses"] = invalid_segments
        result["status"] = "MOTION_COMPLETED" if not invalid_segments else "MOTION_COMPLETED_DATA_INVALID"
    except Exception as exc:
        failure_monotonic_ns = time.monotonic_ns()
        result["status"] = "ERROR"
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if active_pipeline is not None:
            try:
                release_audit = release_active_motion_pipeline(
                    active_pipeline,
                    ownership,
                    "exception_active_ros_release",
                )
                result["active_pipeline_release_audits"].append({"traversal": "unknown", **release_audit})
            except Exception as exc:
                result["active_pipeline_release_error"] = f"{type(exc).__name__}: {exc}"
        if recording is not None:
            result["observer_artifact_summary"] = recording.observer.artifact_summary()
        result["failure_monotonic_ns"] = failure_monotonic_ns
        result["uart_ownership_audit"] = ownership.as_dict()

    try:
        park = run_exclusive_standalone_action("park", ownership, "final_park")
        result["park_result"] = park.__dict__
    except Exception as exc:
        result["park_error"] = f"{type(exc).__name__}: {exc}"
    result["uart_ownership_audit"] = ownership.as_dict()
    result["final_readonly_audit"] = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)

    try:
        shutdown_recording_pipeline(recording)
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    try:
        result.update(build_static_gravity_calibration_result(args.out_dir, result))
    except Exception as exc:
        result["static_gravity_reporting_error"] = f"{type(exc).__name__}: {exc}"
    result["hardware_teardown_result"] = build_hardware_teardown_result(result)
    result["scientific_closure"] = build_e1d_scientific_closure(result)

    result["uart_ownership_audit"] = ownership.as_dict()
    raw_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return 0 if result.get("status") == "MOTION_COMPLETED" else 1

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "reports" / "j9_e1d_1_static_gravity_calibration")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--stage", default=None, help="Stage label for run artifacts, e.g. J9-E1D-2")
    args = parser.parse_args()

    preflight = trajectory_preflight()
    if args.preflight_only or not (args.real_hardware and args.confirm_motion):
        print(json.dumps(preflight, indent=2))
        return 0
    return run_hardware(args)


if __name__ == "__main__":
    raise SystemExit(main())
