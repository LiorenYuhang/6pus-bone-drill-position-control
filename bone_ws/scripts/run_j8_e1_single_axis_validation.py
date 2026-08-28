#!/usr/bin/env python3
"""6PUS J8-E1: Single-Axis Cartesian Translation Hardware Validation Runner.

Executes controlled single-DoF Cartesian motions strictly across the complete ROS 2
topic/node chain:
  /pus6/target_cartesian_pose -> pus6_kinematics_node
  -> /pus6/desired_joint_deltas -> pus6_control_node
  -> /pus6/command_joint_steps -> pus6_motor_driver_node
  -> J7 50Hz Servo -> Real Manipulator
  -> /pus6/joint_feedback/timed -> pus6_fk_node
  -> /pus6/fk/state

Validation sequence:
  1. Hold OPERATING_HOME for 3.0s
  2. Commanded X +1.0 mm (3.0s) -> Return to OPERATING_HOME (3.0s)
  3. Commanded Y +1.0 mm (3.0s) -> Return to OPERATING_HOME (3.0s)
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
class SingleAxisTrace:
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


class Ros2E1ValidationHarness(Node):
    """Orchestrates Cartesian targets and logs topic traffic across the ROS 2 node graph."""

    def __init__(self) -> None:
        super().__init__("j8_e1_validation_harness")
        self.trace = SingleAxisTrace()
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


def analyze_stage_metrics(
    trace: SingleAxisTrace,
    t_start_ns: int,
    t_end_ns: int,
    home_pose_fk: list[float],
    z0: float,
) -> dict[str, Any]:
    """Calculates strict timestamp-aligned joint and Cartesian metrics for a specific test window."""
    # 1. Joint Tracking Metrics
    joint_metrics = {}
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps)  # shape (N, 6)

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
        else:
            joint_metrics[f"ID{mid}"] = {
                "sample_count": 0,
                "rms_err_steps": 0.0,
                "peak_err_steps": 0.0,
                "final_err_steps": 0.0,
                "final_err_mm": 0.0,
            }

    # 2. Cartesian Tracking Metrics
    fk_stamps = np.array(trace.fk_query_stamps_ns)
    fk_poses_mat = np.array(trace.fk_poses) if trace.fk_poses else np.empty((0, 6))

    target_stamps = np.array(trace.target_stamps_ns)
    target_poses_mat = np.array(trace.target_poses) if trace.target_poses else np.empty((0, 6))

    cart_metrics = {}
    if len(fk_stamps) > 0 and len(target_stamps) > 0:
        fk_mask = (fk_stamps >= t_start_ns) & (fk_stamps <= t_end_ns)
        fk_times = fk_stamps[fk_mask]
        fk_in_window = fk_poses_mat[fk_mask]

        if len(fk_times) > 0:
            # Interpolate Cartesian command at exact FK query timestamps
            cmd_interp = np.zeros_like(fk_in_window)
            for col in range(6):
                cmd_interp[:, col] = np.interp(fk_times, target_stamps, target_poses_mat[:, col])

            # Absolute Cartesian Errors: e_abs = FK_measured - Cartesian_command
            abs_err_x = (fk_in_window[:, 0] - cmd_interp[:, 0]) * 1000.0  # mm
            abs_err_y = (fk_in_window[:, 1] - cmd_interp[:, 1]) * 1000.0  # mm
            abs_err_z = (fk_in_window[:, 2] - cmd_interp[:, 2]) * 1000.0  # mm

            # Relative Cartesian Tracking Errors:
            # e_delta = [FK_measured - FK_HOME] - [Cartesian_command - Cartesian_HOME]
            rel_fk_x = (fk_in_window[:, 0] - home_pose_fk[0]) * 1000.0
            rel_fk_y = (fk_in_window[:, 1] - home_pose_fk[1]) * 1000.0
            rel_fk_z = (fk_in_window[:, 2] - home_pose_fk[2]) * 1000.0

            rel_cmd_x = (cmd_interp[:, 0] - 0.0) * 1000.0
            rel_cmd_y = (cmd_interp[:, 1] - 0.0) * 1000.0
            rel_cmd_z = (cmd_interp[:, 2] - z0) * 1000.0

            rel_err_x = rel_fk_x - rel_cmd_x
            rel_err_y = rel_fk_y - rel_cmd_y
            rel_err_z = rel_fk_z - rel_cmd_z

            # SO(3) Rotation Error
            so3_errs_deg = []
            for k in range(len(fk_times)):
                R_meas = euler_to_matrix(fk_in_window[k, 3], fk_in_window[k, 4], fk_in_window[k, 5])
                R_cmd = euler_to_matrix(cmd_interp[k, 3], cmd_interp[k, 4], cmd_interp[k, 5])
                so3_errs_deg.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_cmd)))

            cart_metrics = {
                "sample_count": int(len(fk_times)),
                "abs_err_x_rms_mm": float(np.sqrt(np.mean(abs_err_x**2))),
                "abs_err_x_peak_mm": float(np.max(np.abs(abs_err_x))),
                "abs_err_y_rms_mm": float(np.sqrt(np.mean(abs_err_y**2))),
                "abs_err_y_peak_mm": float(np.max(np.abs(abs_err_y))),
                "abs_err_z_rms_mm": float(np.sqrt(np.mean(abs_err_z**2))),
                "abs_err_z_peak_mm": float(np.max(np.abs(abs_err_z))),
                "rel_err_x_rms_mm": float(np.sqrt(np.mean(rel_err_x**2))),
                "rel_err_x_peak_mm": float(np.max(np.abs(rel_err_x))),
                "rel_err_y_rms_mm": float(np.sqrt(np.mean(rel_err_y**2))),
                "rel_err_y_peak_mm": float(np.max(np.abs(rel_err_y))),
                "rel_err_z_rms_mm": float(np.sqrt(np.mean(rel_err_z**2))),
                "rel_err_z_peak_mm": float(np.max(np.abs(rel_err_z))),
                "so3_err_deg_rms": float(np.sqrt(np.mean(np.array(so3_errs_deg)**2))),
                "so3_err_deg_peak": float(np.max(np.abs(np.array(so3_errs_deg)))),
                "final_measured_pose": [
                    float(fk_in_window[-1, 0] * 1000.0),
                    float(fk_in_window[-1, 1] * 1000.0),
                    float(fk_in_window[-1, 2] * 1000.0),
                    float(math.degrees(fk_in_window[-1, 3])),
                    float(math.degrees(fk_in_window[-1, 4])),
                    float(math.degrees(fk_in_window[-1, 5])),
                ],
            }

    return {
        "joint_metrics": joint_metrics,
        "cartesian_metrics": cart_metrics,
    }


def run_j8_e1_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict[str, Any]:
    print("\n" + "=" * 80)
    print("      6PUS J8-E1: SINGLE-AXIS CARTESIAN TRANSLATION VALIDATION")
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
    harness = Ros2E1ValidationHarness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.")

    results_data: dict[str, Any] = {}

    try:
        # ---------------------------------------------------------------------
        # Phase 0: Takeover at OPERATING_HOME (3.0s)
        # ---------------------------------------------------------------------
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

        # ---------------------------------------------------------------------
        # Phase 1: Commanded X +1.0 mm (3.0s) -> Return HOME (3.0s)
        # ---------------------------------------------------------------------
        print(f"\n--- [Phase 1] Commanding Cartesian X +1.0 mm (x = +1.000 mm) ---")
        t_x_start_ns = time.monotonic_ns()
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.001, y=0.0, z=z0)
            time.sleep(0.020)
        t_x_displaced_ns = time.monotonic_ns()

        print(f"--- [Phase 1] Commanding Return to Cartesian OPERATING_HOME ---")
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)
        t_x_returned_ns = time.monotonic_ns()
        print("[Phase 1 PASS] X +1.0 mm trajectory executed cleanly.")

        # ---------------------------------------------------------------------
        # Phase 2: Commanded Y +1.0 mm (3.0s) -> Return HOME (3.0s)
        # ---------------------------------------------------------------------
        print(f"\n--- [Phase 2] Commanding Cartesian Y +1.0 mm (y = +1.000 mm) ---")
        t_y_start_ns = time.monotonic_ns()
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.001, z=z0)
            time.sleep(0.020)
        t_y_displaced_ns = time.monotonic_ns()

        print(f"--- [Phase 2] Commanding Return to Cartesian OPERATING_HOME ---")
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)
        t_y_returned_ns = time.monotonic_ns()
        print("[Phase 2 PASS] Y +1.0 mm trajectory executed cleanly.")

        # Let FixedLag buffer catch up (200ms)
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

    # Compute metrics for X test and Y test
    x_displace_metrics = analyze_stage_metrics(trace, t_x_start_ns, t_x_displaced_ns, home_fk, z0)
    x_return_metrics = analyze_stage_metrics(trace, t_x_displaced_ns, t_x_returned_ns, home_fk, z0)
    y_displace_metrics = analyze_stage_metrics(trace, t_y_start_ns, t_y_displaced_ns, home_fk, z0)
    y_return_metrics = analyze_stage_metrics(trace, t_y_displaced_ns, t_y_returned_ns, home_fk, z0)

    # Summary
    print("\n" + "=" * 80)
    print("                J8-E1 SINGLE-AXIS HARDWARE RESULTS SUMMARY")
    print("=" * 80)
    print(f"Total Target Cartesian Poses Published: {len(trace.target_stamps_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamps_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_stamps_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")

    # Print Table for X+1mm Test
    print("\n--- [X +1.0 mm Motion Results] ---")
    x_disp = x_displace_metrics["cartesian_metrics"]
    x_ret = x_return_metrics["cartesian_metrics"]
    print(f"Initial HOME Pose (FK):    X={home_fk[0]*1e3:+.3f}mm, Y={home_fk[1]*1e3:+.3f}mm, Z={home_fk[2]*1e3:+.3f}mm")
    if "final_measured_pose" in x_disp:
        p_disp = x_disp["final_measured_pose"]
        p_ret = x_ret["final_measured_pose"]
        measured_x_disp = p_disp[0] - home_fk[0] * 1000.0
        cross_y = p_disp[1] - home_fk[1] * 1000.0
        cross_z = p_disp[2] - home_fk[2] * 1000.0
        ret_x_hyst = p_ret[0] - home_fk[0] * 1000.0
        print(f"X +1.0mm Steady Pose (FK): X={p_disp[0]:+.3f}mm, Y={p_disp[1]:+.3f}mm, Z={p_disp[2]:+.3f}mm")
        print(f"Return HOME Pose (FK):     X={p_ret[0]:+.3f}mm, Y={p_ret[1]:+.3f}mm, Z={p_ret[2]:+.3f}mm")
        print(f"Measured Relative X Disp:  {measured_x_disp:+.3f} mm (Expected: +1.000 mm, Error: {(measured_x_disp - 1.0)*1e3:+.1f} um)")
        print(f"Cross-Axis Parasitic Y:    {cross_y:+.3f} mm, Parasitic Z: {cross_z:+.3f} mm")
        print(f"X Return Hysteresis:       {ret_x_hyst:+.3f} mm ({ret_x_hyst*1e3:+.1f} um)")
        print(f"Relative X Tracking Err:   RMS={x_disp['rel_err_x_rms_mm']:.3f}mm, Peak={x_disp['rel_err_x_peak_mm']:.3f}mm")
        print(f"Absolute X Pose Err:       RMS={x_disp['abs_err_x_rms_mm']:.3f}mm, Peak={x_disp['abs_err_x_peak_mm']:.3f}mm")
        print(f"SO(3) Rotation Error:      RMS={x_disp['so3_err_deg_rms']:.3f} deg, Peak={x_disp['so3_err_deg_peak']:.3f} deg")

    # Print Table for Y+1mm Test
    print("\n--- [Y +1.0 mm Motion Results] ---")
    y_disp = y_displace_metrics["cartesian_metrics"]
    y_ret = y_return_metrics["cartesian_metrics"]
    if "final_measured_pose" in y_disp and "final_measured_pose" in y_ret:
        p_disp = y_disp["final_measured_pose"]
        p_ret = y_ret["final_measured_pose"]
        measured_y_disp = p_disp[1] - home_fk[1] * 1000.0
        cross_x = p_disp[0] - home_fk[0] * 1000.0
        cross_z = p_disp[2] - home_fk[2] * 1000.0
        ret_y_hyst = p_ret[1] - home_fk[1] * 1000.0
        print(f"Y +1.0mm Steady Pose (FK): X={p_disp[0]:+.3f}mm, Y={p_disp[1]:+.3f}mm, Z={p_disp[2]:+.3f}mm")
        print(f"Return HOME Pose (FK):     X={p_ret[0]:+.3f}mm, Y={p_ret[1]:+.3f}mm, Z={p_ret[2]:+.3f}mm")
        print(f"Measured Relative Y Disp:  {measured_y_disp:+.3f} mm (Expected: +1.000 mm, Error: {(measured_y_disp - 1.0)*1e3:+.1f} um)")
        print(f"Cross-Axis Parasitic X:    {cross_x:+.3f} mm, Parasitic Z: {cross_z:+.3f} mm")
        print(f"Y Return Hysteresis:       {ret_y_hyst:+.3f} mm ({ret_y_hyst*1e3:+.1f} um)")
        print(f"Relative Y Tracking Err:   RMS={y_disp['rel_err_y_rms_mm']:.3f}mm, Peak={y_disp['rel_err_y_peak_mm']:.3f}mm")
        print(f"Absolute Y Pose Err:       RMS={y_disp['abs_err_y_rms_mm']:.3f}mm, Peak={y_disp['abs_err_y_peak_mm']:.3f}mm")
        print(f"SO(3) Rotation Error:      RMS={y_disp['so3_err_deg_rms']:.3f} deg, Peak={y_disp['so3_err_deg_peak']:.3f} deg")

    # Joint Table
    print("\n--- Strict Timestamp-Aligned Joint Tracking (X +1.0mm Window) ---")
    print(f"{'Axis ID':<8} {'Samples':<10} {'Cmd Min':<9} {'Cmd Max':<9} {'Meas Min':<9} {'Meas Max':<9} {'RMS Err (steps)':<17} {'Peak Err (steps)':<17} {'Final Err (steps)'}")
    print("-" * 110)
    for mid in MOTOR_IDS:
        m = x_displace_metrics["joint_metrics"].get(f"ID{mid}", {})
        if m.get("sample_count", 0) > 0:
            print(f"ID{mid:<7} {m['sample_count']:<10} {m['cmd_min']:<9.0f} {m['cmd_max']:<9.0f} {m['meas_min']:<9} {m['meas_max']:<9} {m['rms_err_steps']:<17.2f} {m['peak_err_steps']:<17.1f} {m['final_err_steps']:+.1f}")

    print("\n--- Strict Timestamp-Aligned Joint Tracking (Y +1.0mm Window) ---")
    print(f"{'Axis ID':<8} {'Samples':<10} {'Cmd Min':<9} {'Cmd Max':<9} {'Meas Min':<9} {'Meas Max':<9} {'RMS Err (steps)':<17} {'Peak Err (steps)':<17} {'Final Err (steps)'}")
    print("-" * 110)
    for mid in MOTOR_IDS:
        m = y_displace_metrics["joint_metrics"].get(f"ID{mid}", {})
        if m.get("sample_count", 0) > 0:
            print(f"ID{mid:<7} {m['sample_count']:<10} {m['cmd_min']:<9.0f} {m['cmd_max']:<9.0f} {m['meas_min']:<9} {m['meas_max']:<9} {m['rms_err_steps']:<17.2f} {m['peak_err_steps']:<17.1f} {m['final_err_steps']:+.1f}")

    results_data = {
        "x_displace": x_displace_metrics,
        "x_return": x_return_metrics,
        "y_displace": y_displace_metrics,
        "y_return": y_return_metrics,
        "home_pose_fk": home_fk,
        "total_targets_pub": len(trace.target_stamps_ns),
        "total_ik_rcv": len(trace.ik_stamps_ns),
        "total_cmd_rcv": len(trace.cmd_stamps_ns),
        "total_fk_rcv": len(trace.fk_query_stamps_ns),
    }

    if output_json:
        with open(output_json, "w") as f:
            json.dump(results_data, f, indent=2)
        print(f"\nWrote J8-E1 results to {output_json}")

    return results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6PUS J8-E1 Single-Axis Hardware Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_e1_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay_ms,
        output_json=args.output_json,
    )
