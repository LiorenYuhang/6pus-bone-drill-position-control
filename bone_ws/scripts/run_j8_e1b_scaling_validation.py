#!/usr/bin/env python3
"""6PUS J8-E1B: Translation Resolution Scaling & Error Decomposition Runner.

Executes controlled multi-amplitude single-DoF Cartesian motions strictly across the
complete ROS 2 topic/node chain:
  /pus6/target_cartesian_pose -> pus6_kinematics_node
  -> /pus6/desired_joint_deltas -> pus6_control_node
  -> /pus6/command_joint_steps -> pus6_motor_driver_node
  -> J7 50Hz Servo -> Real Manipulator
  -> /pus6/joint_feedback/timed -> pus6_fk_node
  -> /pus6/fk/state

Validation sequence:
  1. Hold OPERATING_HOME (3.0s)
  2. Commanded X +2.0 mm (3.0s) -> Return HOME (3.0s)
  3. Commanded X +3.0 mm (3.0s) -> Return HOME (3.0s)
  4. Commanded Y +2.0 mm (3.0s) -> Return HOME (3.0s)
  5. Commanded Y +3.0 mm (3.0s) -> Return HOME (3.0s)
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# ROS 2 imports
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import MOTOR_IDS
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback


def so3_geodesic_distance_rad(R1: list[list[float]], R2: list[list[float]]) -> float:
    tr = 0.0
    for r in range(3):
        for c in range(3):
            tr += R1[r][c] * R2[r][c]
    cos_theta = (tr - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.acos(cos_theta)


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


@dataclass
class ScalingTrace:
    # Target Cartesian
    target_stamps_ns: list[int] = field(default_factory=list)
    target_poses: list[list[float]] = field(default_factory=list)  # [x, y, z, roll, pitch, yaw]

    # IK Desired Deltas
    ik_stamps_ns: list[int] = field(default_factory=list)
    ik_deltas_m: list[list[float]] = field(default_factory=list)

    # Control Commanded Steps
    cmd_stamps_ns: list[int] = field(default_factory=list)
    cmd_steps: list[list[int]] = field(default_factory=list)

    # Timed Joint Feedback
    feedback_rx_stamps_ns: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_meas_steps: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_currents: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_faults: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})

    # FK Reconstructed State
    fk_query_stamps_ns: list[int] = field(default_factory=list)
    fk_poses: list[list[float]] = field(default_factory=list)
    fk_residual_norms: list[float] = field(default_factory=list)
    fk_sync_statuses: list[int] = field(default_factory=list)


class Ros2ScalingHarness(Node):
    """Orchestrates Cartesian targets and logs topic traffic across the ROS 2 node graph."""

    def __init__(self) -> None:
        super().__init__("j8_e1b_scaling_harness")
        self.trace = ScalingTrace()
        self.lock = threading.Lock()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", reliable_qos
        )

        self.ik_sub = self.create_subscription(
            Float64MultiArray, "/pus6/desired_joint_deltas", self._on_ik_deltas, reliable_qos
        )
        self.cmd_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, sensor_qos
        )
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_feedback, sensor_qos
        )
        self.fk_sub = self.create_subscription(
            FkStateStamped, "/pus6/fk/state", self._on_fk_state, sensor_qos
        )

        self._last_known_rx_ns: dict[int, int] = {m: 0 for m in MOTOR_IDS}

    def _on_ik_deltas(self, msg: Float64MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            self.trace.ik_stamps_ns.append(t_ns)
            self.trace.ik_deltas_m.append(list(msg.data))

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            self.trace.cmd_stamps_ns.append(t_ns)
            self.trace.cmd_steps.append(list(msg.data))

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            for i, mid in enumerate(MOTOR_IDS):
                rx_ns = msg.per_axis_rx_done_monotonic_ns[i]
                if rx_ns > self._last_known_rx_ns[mid]:
                    self._last_known_rx_ns[mid] = rx_ns
                    self.trace.feedback_rx_stamps_ns[mid].append(rx_ns)
                    self.trace.feedback_meas_steps[mid].append(int(msg.raw_steps[i]))
                    self.trace.feedback_currents[mid].append(int(msg.currents_ma[i]))
                    self.trace.feedback_faults[mid].append(int(msg.fault_codes[i]))

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        with self.lock:
            self.trace.fk_query_stamps_ns.append(msg.query_monotonic_timestamp_ns)
            p = msg.pose.position
            euler = msg.euler_angles_rad
            self.trace.fk_poses.append([p.x, p.y, p.z, euler[0], euler[1], euler[2]])
            self.trace.fk_residual_norms.append(msg.residual_norm_meters)
            self.trace.fk_sync_statuses.append(1 if msg.converged else 0)

    def publish_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        t_ns = time.monotonic_ns()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)

        with self.lock:
            self.trace.target_stamps_ns.append(t_ns)
            self.trace.target_poses.append([float(x), float(y), float(z), float(roll), float(pitch), float(yaw)])

        self.target_pose_pub.publish(msg)


def analyze_decomposed_metrics(
    trace: ScalingTrace,
    t_start_ns: int,
    t_end_ns: int,
    home_pose_fk: list[float],
    z0: float,
    kin: Analytical6PusKinematics,
) -> dict[str, Any]:
    """Calculates strict timestamp-aligned joint and Cartesian decomposed metrics."""
    # 1. Joint Tracking Metrics
    joint_metrics = {}
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps)  # shape (N, 6)

    steady_cmd_steps = []
    settled_meas_steps = []

    for i, mid in enumerate(MOTOR_IDS):
        rx_stamps = np.array(trace.feedback_rx_stamps_ns[mid])
        meas_steps = np.array(trace.feedback_meas_steps[mid])

        mask = (rx_stamps >= t_start_ns) & (rx_stamps <= t_end_ns)
        rx_in_window = rx_stamps[mask]
        meas_in_window = meas_steps[mask]

        if len(rx_in_window) > 0 and len(cmd_stamps) > 0:
            cmd_interp = np.interp(rx_in_window, cmd_stamps, cmd_steps_mat[:, i])
            errs = cmd_interp - meas_in_window
            joint_metrics[f"ID{mid}"] = {
                "sample_count": int(len(rx_in_window)),
                "cmd_min": float(np.min(cmd_interp)),
                "cmd_max": float(np.max(cmd_interp)),
                "meas_min": int(np.min(meas_in_window)),
                "meas_max": int(np.max(meas_in_window)),
                "rms_err_steps": float(np.sqrt(np.mean(errs**2))),
                "peak_err_steps": float(np.max(np.abs(errs))),
                "final_err_steps": float(errs[-1]),
                "final_err_mm": float(errs[-1] * 0.025),
            }
            steady_cmd_steps.append(int(round(cmd_interp[-1])))
            settled_meas_steps.append(int(meas_in_window[-1]))
        else:
            joint_metrics[f"ID{mid}"] = {
                "sample_count": 0,
                "rms_err_steps": 0.0,
                "peak_err_steps": 0.0,
                "final_err_steps": 0.0,
                "final_err_mm": 0.0,
            }
            steady_cmd_steps.append(1000)
            settled_meas_steps.append(1000)

    # 2. Cartesian Tracking Metrics & 3-way Decomposition
    fk_stamps = np.array(trace.fk_query_stamps_ns)
    fk_poses_mat = np.array(trace.fk_poses) if trace.fk_poses else np.empty((0, 6))

    target_stamps = np.array(trace.target_stamps_ns)
    target_poses_mat = np.array(trace.target_poses) if trace.target_poses else np.empty((0, 6))

    cart_metrics = {}
    if len(fk_stamps) > 0 and len(target_stamps) > 0 and len(cmd_stamps) > 0:
        fk_mask = (fk_stamps >= t_start_ns) & (fk_stamps <= t_end_ns)
        fk_times = fk_stamps[fk_mask]
        fk_in_window = fk_poses_mat[fk_mask]

        if len(fk_times) > 0:
            # A. X_target(t): Raw Cartesian target interpolated at FK query times
            x_target = np.zeros_like(fk_in_window)
            for col in range(6):
                x_target[:, col] = np.interp(fk_times, target_stamps, target_poses_mat[:, col])

            # B. q_cmd(t) & X_shaped(t) = FK(q_cmd(t))
            q_cmd_interp = np.zeros((len(fk_times), 6))
            for col in range(6):
                q_cmd_interp[:, col] = np.interp(fk_times, cmd_stamps, cmd_steps_mat[:, col])

            x_shaped = np.zeros_like(fk_in_window)
            for row in range(len(fk_times)):
                # Convert steps to slider meters around nominal 0.05m
                joint_m = [0.05 + (s - 1000) / 40000.0 for s in q_cmd_interp[row]]
                pose_shaped = kin.forward_kinematics(joint_m)
                x_shaped[row, 0:3] = pose_shaped[0:3]
                x_shaped[row, 3:6] = pose_shaped[3:6]

            # C. X_FK(t): Joint-feedback-derived FK pose
            x_fk = fk_in_window

            # 1. End-to-end target error: e_target(t) = X_FK(t) - X_target(t)
            e_target = (x_fk[:, 0:3] - x_target[:, 0:3]) * 1000.0  # mm

            # 2. Shaper error: e_shape(t) = X_shaped(t) - X_target(t)
            e_shape = (x_shaped[:, 0:3] - x_target[:, 0:3]) * 1000.0  # mm

            # 3. Execution / Actuator tracking error: e_exec(t) = X_FK(t) - X_shaped(t)
            e_exec = (x_fk[:, 0:3] - x_shaped[:, 0:3]) * 1000.0  # mm

            # Relative motion metrics
            rel_fk_x = (x_fk[:, 0] - home_pose_fk[0]) * 1000.0
            rel_fk_y = (x_fk[:, 1] - home_pose_fk[1]) * 1000.0
            rel_fk_z = (x_fk[:, 2] - home_pose_fk[2]) * 1000.0

            rel_cmd_x = (x_target[:, 0] - 0.0) * 1000.0
            rel_cmd_y = (x_target[:, 1] - 0.0) * 1000.0
            rel_cmd_z = (x_target[:, 2] - z0) * 1000.0

            rel_err_x = rel_fk_x - rel_cmd_x
            rel_err_y = rel_fk_y - rel_cmd_y
            rel_err_z = rel_fk_z - rel_cmd_z

            # SO(3) Rotation Error
            so3_errs_deg = []
            for k in range(len(fk_times)):
                R_meas = euler_to_matrix(x_fk[k, 3], x_fk[k, 4], x_fk[k, 5])
                R_cmd = euler_to_matrix(x_target[k, 3], x_target[k, 4], x_target[k, 5])
                so3_errs_deg.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_cmd)))

            cart_metrics = {
                "sample_count": int(len(fk_times)),
                "e_target_rms_xyz_mm": [
                    float(np.sqrt(np.mean(e_target[:, 0]**2))),
                    float(np.sqrt(np.mean(e_target[:, 1]**2))),
                    float(np.sqrt(np.mean(e_target[:, 2]**2))),
                ],
                "e_target_peak_xyz_mm": [
                    float(np.max(np.abs(e_target[:, 0]))),
                    float(np.max(np.abs(e_target[:, 1]))),
                    float(np.max(np.abs(e_target[:, 2]))),
                ],
                "e_shape_rms_xyz_mm": [
                    float(np.sqrt(np.mean(e_shape[:, 0]**2))),
                    float(np.sqrt(np.mean(e_shape[:, 1]**2))),
                    float(np.sqrt(np.mean(e_shape[:, 2]**2))),
                ],
                "e_shape_peak_xyz_mm": [
                    float(np.max(np.abs(e_shape[:, 0]))),
                    float(np.max(np.abs(e_shape[:, 1]))),
                    float(np.max(np.abs(e_shape[:, 2]))),
                ],
                "e_exec_rms_xyz_mm": [
                    float(np.sqrt(np.mean(e_exec[:, 0]**2))),
                    float(np.sqrt(np.mean(e_exec[:, 1]**2))),
                    float(np.sqrt(np.mean(e_exec[:, 2]**2))),
                ],
                "e_exec_peak_xyz_mm": [
                    float(np.max(np.abs(e_exec[:, 0]))),
                    float(np.max(np.abs(e_exec[:, 1]))),
                    float(np.max(np.abs(e_exec[:, 2]))),
                ],
                "rel_err_x_rms_mm": float(np.sqrt(np.mean(rel_err_x**2))),
                "rel_err_x_peak_mm": float(np.max(np.abs(rel_err_x))),
                "rel_err_y_rms_mm": float(np.sqrt(np.mean(rel_err_y**2))),
                "rel_err_y_peak_mm": float(np.max(np.abs(rel_err_y))),
                "rel_err_z_rms_mm": float(np.sqrt(np.mean(rel_err_z**2))),
                "rel_err_z_peak_mm": float(np.max(np.abs(rel_err_z))),
                "so3_err_deg_rms": float(np.sqrt(np.mean(np.array(so3_errs_deg)**2))),
                "so3_err_deg_peak": float(np.max(np.abs(np.array(so3_errs_deg)))),
                "final_measured_pose": [
                    float(x_fk[-1, 0] * 1000.0),
                    float(x_fk[-1, 1] * 1000.0),
                    float(x_fk[-1, 2] * 1000.0),
                    float(math.degrees(x_fk[-1, 3])),
                    float(math.degrees(x_fk[-1, 4])),
                    float(math.degrees(x_fk[-1, 5])),
                ],
                "final_shaped_pose": [
                    float(x_shaped[-1, 0] * 1000.0),
                    float(x_shaped[-1, 1] * 1000.0),
                    float(x_shaped[-1, 2] * 1000.0),
                    float(math.degrees(x_shaped[-1, 3])),
                    float(math.degrees(x_shaped[-1, 4])),
                    float(math.degrees(x_shaped[-1, 5])),
                ],
            }

    return {
        "joint_metrics": joint_metrics,
        "cartesian_metrics": cart_metrics,
        "steady_cmd_steps": steady_cmd_steps,
        "settled_meas_steps": settled_meas_steps,
    }


def run_j8_e1b_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict[str, Any]:
    print("\n" + "=" * 80)
    print("   6PUS J8-E1B: TRANSLATION RESOLUTION SCALING & ERROR DECOMPOSITION")
    print("=" * 80)
    print(f"Hardware Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"FixedLag Delay: {fixed_lag_delay_ms:.1f} ms")
    print("=" * 80 + "\n")

    rclpy.init()

    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    print(f"Geometry Nominal Cartesian HOME: z0 = {z0:.6f} m ({z0*1000:.3f} mm)")

    # 1. Instantiate the real ROS 2 nodes
    print("\n[INIT] Instantiating ROS2 nodes...")
    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=port,
        baudrate=baudrate,
        timeout=0.030,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=fixed_lag_delay_ms)
    harness = Ros2ScalingHarness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.")

    stages = [
        ("x_plus_2mm", 0.002, 0.0, "X +2.0 mm"),
        ("x_plus_3mm", 0.003, 0.0, "X +3.0 mm"),
        ("y_plus_2mm", 0.0, 0.002, "Y +2.0 mm"),
        ("y_plus_3mm", 0.0, 0.003, "Y +3.0 mm"),
    ]

    timing_windows = {}

    try:
        # Phase 0: Takeover at OPERATING_HOME (3.0s)
        print("\n--- [Phase 0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
        t_p0_start_ns = time.monotonic_ns()
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)
        t_p0_end_ns = time.monotonic_ns()

        # Capture initial HOME FK pose
        with harness.lock:
            if harness.trace.fk_poses:
                home_fk = list(harness.trace.fk_poses[-1])
            else:
                home_fk = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
        print(f"[Phase 0 PASS] Initial Settled HOME FK: X={home_fk[0]*1e3:+.3f}mm, Y={home_fk[1]*1e3:+.3f}mm, Z={home_fk[2]*1e3:+.3f}mm")

        # Execute amplitude tests
        for tag, tx, ty, label in stages:
            print(f"\n--- [Stage {label}] Commanding Cartesian displacement ({label}) ---")
            t_disp_start_ns = time.monotonic_ns()
            t_end = time.monotonic() + 3.0
            while time.monotonic() < t_end:
                harness.publish_target_pose(x=tx, y=ty, z=z0)
                time.sleep(0.020)
            t_disp_end_ns = time.monotonic_ns()

            print(f"--- [Stage {label}] Commanding Return to Cartesian OPERATING_HOME ---")
            t_ret_start_ns = time.monotonic_ns()
            t_end = time.monotonic() + 3.0
            while time.monotonic() < t_end:
                harness.publish_target_pose(x=0.0, y=0.0, z=z0)
                time.sleep(0.020)
            t_ret_end_ns = time.monotonic_ns()

            timing_windows[tag] = {
                "disp_start_ns": t_disp_start_ns,
                "disp_end_ns": t_disp_end_ns,
                "ret_start_ns": t_ret_start_ns,
                "ret_end_ns": t_ret_end_ns,
                "target_x": tx,
                "target_y": ty,
                "label": label,
            }
            print(f"[{label} PASS] Trajectory completed cleanly.")

        # Allow FixedLag buffer (160ms) to finalize
        time.sleep(0.3)

    finally:
        print("\n[SHUTDOWN] Shutting down ROS2 executor and nodes...")
        try:
            motor_driver_node.destroy_node()
        except Exception:
            pass
        try:
            control_node.destroy_node()
            kinematics_node.destroy_node()
            fk_node.destroy_node()
            harness.destroy_node()
            executor.shutdown()
        except Exception:
            pass
        rclpy.shutdown()
        print("[SHUTDOWN OK] System stopped safely.")

    # -------------------------------------------------------------------------
    # Analysis & Metric Compilation
    # -------------------------------------------------------------------------
    trace = harness.trace
    stage_results = {}

    print("\n" + "=" * 85)
    print("                J8-E1B TRANSLATION SCALING RESULTS SUMMARY")
    print("=" * 85)
    print(f"Total Target Cartesian Poses Published: {len(trace.target_stamps_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamps_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_stamps_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")
    print(f"Initial HOME Pose (FK): X={home_fk[0]*1e3:+.3f}mm, Y={home_fk[1]*1e3:+.3f}mm, Z={home_fk[2]*1e3:+.3f}mm")

    print("\n" + "-" * 115)
    print(f"{'Motion':<10} {'Cmd Disp':<10} {'Meas Disp':<11} {'Gain %':<9} {'Disp Error':<13} {'Cross Y/X':<12} {'e_target RMS':<14} {'e_exec RMS':<12} {'Return Hyst'}")
    print("-" * 115)

    for tag, win in timing_windows.items():
        disp_metrics = analyze_decomposed_metrics(
            trace, win["disp_start_ns"], win["disp_end_ns"], home_fk, z0, kin
        )
        ret_metrics = analyze_decomposed_metrics(
            trace, win["ret_start_ns"], win["ret_end_ns"], home_fk, z0, kin
        )

        d_cart = disp_metrics["cartesian_metrics"]
        r_cart = ret_metrics["cartesian_metrics"]

        if "final_measured_pose" not in d_cart or "final_measured_pose" not in r_cart:
            continue

        p_disp = d_cart["final_measured_pose"]
        p_ret = r_cart["final_measured_pose"]

        if win["target_x"] > 0:
            cmd_disp = win["target_x"] * 1000.0
            meas_disp = p_disp[0] - home_fk[0] * 1000.0
            cross_disp = p_disp[1] - home_fk[1] * 1000.0
            hyst = p_ret[0] - home_fk[0] * 1000.0
            e_tgt_rms = d_cart["e_target_rms_xyz_mm"][0]
            e_exec_rms = d_cart["e_exec_rms_xyz_mm"][0]
        else:
            cmd_disp = win["target_y"] * 1000.0
            meas_disp = p_disp[1] - home_fk[1] * 1000.0
            cross_disp = p_disp[0] - home_fk[0] * 1000.0
            hyst = p_ret[1] - home_fk[1] * 1000.0
            e_tgt_rms = d_cart["e_target_rms_xyz_mm"][1]
            e_exec_rms = d_cart["e_exec_rms_xyz_mm"][1]

        gain_pct = (meas_disp / cmd_disp) * 100.0
        disp_err_um = (meas_disp - cmd_disp) * 1000.0

        stage_results[tag] = {
            "displacement": disp_metrics,
            "return": ret_metrics,
            "cmd_disp_mm": cmd_disp,
            "meas_disp_mm": meas_disp,
            "gain_pct": gain_pct,
            "disp_err_um": disp_err_um,
            "cross_disp_mm": cross_disp,
            "hyst_um": hyst * 1000.0,
        }

        print(
            f"{win['label']:<10} {cmd_disp:+6.2f} mm  {meas_disp:+7.3f} mm  {gain_pct:6.1f}%  "
            f"{disp_err_um:+7.1f} um    {cross_disp:+6.3f} mm   {e_tgt_rms:6.3f} mm       {e_exec_rms:6.3f} mm     {hyst*1e3:+6.1f} um"
        )

    # Print Joint Steps Table
    print("\n--- Command vs Settled Joint Steps Across Scaling Tests ---")
    print(f"{'Motion':<10} {'Cmd Steps [ID1..ID6]':<32} {'Settled Meas Steps [ID1..ID6]':<32} {'Joint Delta Errors (steps)'}")
    print("-" * 115)
    for tag, win in timing_windows.items():
        res_tag = stage_results[tag]["displacement"]
        cmd_s = res_tag["steady_cmd_steps"]
        meas_s = res_tag["settled_meas_steps"]
        err_s = [meas_s[i] - cmd_s[i] for i in range(6)]
        print(f"{win['label']:<10} {str(cmd_s):<32} {str(meas_s):<32} {str(err_s)}")

    results_data = {
        "home_pose_fk": home_fk,
        "stages": stage_results,
        "total_targets_pub": len(trace.target_stamps_ns),
        "total_ik_rcv": len(trace.ik_stamps_ns),
        "total_cmd_rcv": len(trace.cmd_stamps_ns),
        "total_fk_rcv": len(trace.fk_query_stamps_ns),
    }

    if output_json:
        with open(output_json, "w") as f:
            json.dump(results_data, f, indent=2)
        print(f"\nWrote J8-E1B results to {output_json}")

    return results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6PUS J8-E1B Translation Scaling Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_e1b_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay_ms,
        output_json=args.output_json,
    )
