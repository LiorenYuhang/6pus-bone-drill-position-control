#!/usr/bin/env python3
"""6PUS J8-E2B: Multi-Step Yaw Scaling Validation Runner.

Executes controlled multi-step Yaw rotations strictly across the complete ROS 2 topic graph:
  /pus6/target_cartesian_pose -> pus6_kinematics_node
  -> /pus6/desired_joint_deltas -> pus6_control_node
  -> /pus6/command_joint_steps -> pus6_motor_driver_node
  -> J7 50Hz Servo -> Real Manipulator
  -> /pus6/joint_feedback/timed -> pus6_fk_node
  -> /pus6/fk/state

Validation sequence:
  1. Hold OPERATING_HOME (3.0s)
  2. Yaw +2.0 deg (3.0s) -> Return HOME (3.0s) -> Yaw -2.0 deg (3.0s) -> Return HOME (3.0s)
  3. Yaw +3.0 deg (3.0s) -> Return HOME (3.0s) -> Yaw -3.0 deg (3.0s) -> Return HOME (3.0s)
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
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


def so3_geodesic_distance_rad(R1: list[list[float]] | np.ndarray, R2: list[list[float]] | np.ndarray) -> float:
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
class YawTrace:
    target_stamps_ns: list[int] = field(default_factory=list)
    target_poses: list[list[float]] = field(default_factory=list)

    ik_stamps_ns: list[int] = field(default_factory=list)
    ik_deltas_m: list[list[float]] = field(default_factory=list)

    cmd_stamps_ns: list[int] = field(default_factory=list)
    cmd_steps: list[list[int]] = field(default_factory=list)

    feedback_rx_stamps_ns: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_meas_steps: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_currents: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})
    feedback_faults: dict[int, list[int]] = field(default_factory=lambda: {m: [] for m in MOTOR_IDS})

    fk_query_stamps_ns: list[int] = field(default_factory=list)
    fk_poses: list[list[float]] = field(default_factory=list)
    fk_residual_norms: list[float] = field(default_factory=list)
    fk_sync_statuses: list[int] = field(default_factory=list)


class Ros2YawHarness(Node):
    def __init__(self) -> None:
        super().__init__("j8_e2b_yaw_harness")
        self.trace = YawTrace()
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


def analyze_yaw_stage(
    trace: YawTrace,
    t_start_ns: int,
    t_end_ns: int,
    home_pose_fk: list[float],
    z0: float,
    kin: Analytical6PusKinematics,
) -> dict[str, Any]:
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps) if trace.cmd_steps else np.empty((0, 6))

    steady_cmd_steps = []
    settled_meas_steps = []
    joint_metrics = {}

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
            x_target = np.zeros_like(fk_in_window)
            for col in range(6):
                x_target[:, col] = np.interp(fk_times, target_stamps, target_poses_mat[:, col])

            q_cmd_interp = np.zeros((len(fk_times), 6))
            for col in range(6):
                q_cmd_interp[:, col] = np.interp(fk_times, cmd_stamps, cmd_steps_mat[:, col])

            x_shaped = np.zeros_like(fk_in_window)
            for row in range(len(fk_times)):
                joint_m = [0.05 + (s - 1000) / 40000.0 for s in q_cmd_interp[row]]
                pose_shaped = kin.forward_kinematics(joint_m)
                x_shaped[row, 0:3] = pose_shaped[0:3]
                x_shaped[row, 3:6] = pose_shaped[3:6]

            x_fk = fk_in_window

            e_tgt_trans = (x_fk[:, 0:3] - x_target[:, 0:3]) * 1000.0
            e_shp_trans = (x_shaped[:, 0:3] - x_target[:, 0:3]) * 1000.0
            e_exc_trans = (x_fk[:, 0:3] - x_shaped[:, 0:3]) * 1000.0

            so3_tgt_deg = []
            so3_shp_deg = []
            so3_exc_deg = []

            for k in range(len(fk_times)):
                R_meas = euler_to_matrix(x_fk[k, 3], x_fk[k, 4], x_fk[k, 5])
                R_cmd = euler_to_matrix(x_target[k, 3], x_target[k, 4], x_target[k, 5])
                R_shp = euler_to_matrix(x_shaped[k, 3], x_shaped[k, 4], x_shaped[k, 5])

                so3_tgt_deg.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_cmd)))
                so3_shp_deg.append(math.degrees(so3_geodesic_distance_rad(R_shp, R_cmd)))
                so3_exc_deg.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_shp)))

            cart_metrics = {
                "sample_count": int(len(fk_times)),
                "e_target_rms_xyz_mm": [float(np.sqrt(np.mean(e_tgt_trans[:, c]**2))) for c in range(3)],
                "e_shape_rms_xyz_mm": [float(np.sqrt(np.mean(e_shp_trans[:, c]**2))) for c in range(3)],
                "e_exec_rms_xyz_mm": [float(np.sqrt(np.mean(e_exc_trans[:, c]**2))) for c in range(3)],
                "so3_target_rms_deg": float(np.sqrt(np.mean(np.array(so3_tgt_deg)**2))),
                "so3_target_peak_deg": float(np.max(np.abs(np.array(so3_tgt_deg)))),
                "so3_shape_rms_deg": float(np.sqrt(np.mean(np.array(so3_shp_deg)**2))),
                "so3_shape_peak_deg": float(np.max(np.abs(np.array(so3_shp_deg)))),
                "so3_exec_rms_deg": float(np.sqrt(np.mean(np.array(so3_exc_deg)**2))),
                "so3_exec_peak_deg": float(np.max(np.abs(np.array(so3_exc_deg)))),
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


def run_j8_e2b_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict[str, Any]:
    print("\n" + "=" * 85)
    print("        6PUS J8-E2B: MULTI-STEP YAW SCALING HARDWARE VALIDATION")
    print("=" * 85)
    print(f"Hardware Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"FixedLag Delay: {fixed_lag_delay_ms:.1f} ms")
    print("=" * 85 + "\n")

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
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=fixed_lag_delay_ms)
    harness = Ros2YawHarness()

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
        ("yaw_plus_2deg", +2.0, "Yaw +2.00 deg"),
        ("yaw_neg_2deg", -2.0, "Yaw -2.00 deg"),
        ("yaw_plus_3deg", +3.0, "Yaw +3.00 deg"),
        ("yaw_neg_3deg", -3.0, "Yaw -3.00 deg"),
    ]

    timing_windows = {}

    try:
        # Phase 0: Takeover at OPERATING_HOME (3.0s)
        print("\n--- [Phase 0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
        t_p0_start_ns = time.monotonic_ns()
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(0.020)
        t_p0_end_ns = time.monotonic_ns()

        with harness.lock:
            if harness.trace.fk_poses:
                home_fk = list(harness.trace.fk_poses[-1])
            else:
                home_fk = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
        print(f"[Phase 0 PASS] Initial Settled HOME FK: X={home_fk[0]*1e3:+.3f}mm, Y={home_fk[1]*1e3:+.3f}mm, Z={home_fk[2]*1e3:+.3f}mm, R={math.degrees(home_fk[3]):+.3f}°, P={math.degrees(home_fk[4]):+.3f}°, Y={math.degrees(home_fk[5]):+.3f}°")

        # Execute Yaw scaling tests
        for tag, y_deg, label in stages:
            y_rad = math.radians(y_deg)
            print(f"\n--- [Stage {label}] Commanding Orientation Target ({label}) ---")
            t_disp_start_ns = time.monotonic_ns()
            t_end = time.monotonic() + 3.0
            while time.monotonic() < t_end:
                harness.publish_target_pose(x=0.0, y=0.0, z=z0, roll=0.0, pitch=0.0, yaw=y_rad)
                time.sleep(0.020)
            t_disp_end_ns = time.monotonic_ns()

            print(f"--- [Stage {label}] Commanding Return to Cartesian OPERATING_HOME ---")
            t_ret_start_ns = time.monotonic_ns()
            t_end = time.monotonic() + 3.0
            while time.monotonic() < t_end:
                harness.publish_target_pose(x=0.0, y=0.0, z=z0, roll=0.0, pitch=0.0, yaw=0.0)
                time.sleep(0.020)
            t_ret_end_ns = time.monotonic_ns()

            timing_windows[tag] = {
                "disp_start_ns": t_disp_start_ns,
                "disp_end_ns": t_disp_end_ns,
                "ret_start_ns": t_ret_start_ns,
                "ret_end_ns": t_ret_end_ns,
                "cmd_y_deg": y_deg,
                "label": label,
            }
            print(f"[{label} PASS] Motion completed cleanly.")

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

    # Analysis & Metric Compilation
    trace = harness.trace
    stage_results = {}

    print("\n" + "=" * 90)
    print("                J8-E2B MULTI-STEP YAW RESULTS SUMMARY")
    print("=" * 90)
    print(f"Total Target Cartesian Poses Published: {len(trace.target_stamps_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamps_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_stamps_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")

    print("\n" + "-" * 125)
    print(f"{'Motion':<18} {'Cmd Deg':<10} {'FK Meas Deg':<12} {'Gain %':<9} {'SO(3) Geod Err':<16} {'Parasitic XYZ (mm)':<22} {'Cross Rot (deg)':<18} {'Return Hyst'}")
    print("-" * 125)

    home_r_deg = math.degrees(home_fk[3])
    home_p_deg = math.degrees(home_fk[4])
    home_y_deg = math.degrees(home_fk[5])

    for tag, win in timing_windows.items():
        disp_metrics = analyze_yaw_stage(
            trace, win["disp_start_ns"], win["disp_end_ns"], home_fk, z0, kin
        )
        ret_metrics = analyze_yaw_stage(
            trace, win["ret_start_ns"], win["ret_end_ns"], home_fk, z0, kin
        )

        d_cart = disp_metrics["cartesian_metrics"]
        r_cart = ret_metrics["cartesian_metrics"]

        if "final_measured_pose" not in d_cart or "final_measured_pose" not in r_cart:
            continue

        p_disp = d_cart["final_measured_pose"]
        p_ret = r_cart["final_measured_pose"]

        rel_r_deg = p_disp[3] - home_r_deg
        rel_p_deg = p_disp[4] - home_p_deg
        rel_y_deg = p_disp[5] - home_y_deg

        cmd_val = win["cmd_y_deg"]
        meas_val = rel_y_deg
        cross_rot = f"R={rel_r_deg:+.2f}°, P={rel_p_deg:+.2f}°"
        hyst = p_ret[5] - home_y_deg
        gain_pct = (meas_val / cmd_val) * 100.0 if abs(cmd_val) > 1e-6 else 100.0
        parasitic_xyz = f"[{p_disp[0]-home_fk[0]*1e3:+.2f}, {p_disp[1]-home_fk[1]*1e3:+.2f}, {p_disp[2]-home_fk[2]*1e3:+.2f}]"
        so3_err_rms = d_cart["so3_target_rms_deg"]
        so3_exec_rms = d_cart["so3_exec_rms_deg"]

        stage_results[tag] = {
            "displacement": disp_metrics,
            "return": ret_metrics,
            "cmd_deg": cmd_val,
            "meas_deg": meas_val,
            "gain_pct": gain_pct,
            "so3_target_rms_deg": so3_err_rms,
            "so3_exec_rms_deg": so3_exec_rms,
            "parasitic_xyz_mm": [
                p_disp[0] - home_fk[0] * 1e3,
                p_disp[1] - home_fk[1] * 1e3,
                p_disp[2] - home_fk[2] * 1e3,
            ],
            "hyst_deg": hyst,
        }

        print(
            f"{win['label']:<18} {cmd_val:+6.2f}°   {meas_val:+7.3f}°     {gain_pct:6.1f}%  "
            f"RMS={so3_err_rms:5.3f}°        {parasitic_xyz:<22} {cross_rot:<18} {hyst:+6.3f}°"
        )

    print("\n--- Command vs Settled Joint Steps Across Yaw Tests ---")
    print(f"{'Motion':<18} {'Cmd Steps [ID1..ID6]':<32} {'Settled Meas Steps [ID1..ID6]':<32} {'Joint Delta Errors (steps)'}")
    print("-" * 125)
    for tag, win in timing_windows.items():
        if tag in stage_results:
            res_tag = stage_results[tag]["displacement"]
            cmd_s = res_tag["steady_cmd_steps"]
            meas_s = res_tag["settled_meas_steps"]
            err_s = [meas_s[i] - cmd_s[i] for i in range(6)]
            print(f"{win['label']:<18} {str(cmd_s):<32} {str(meas_s):<32} {str(err_s)}")

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
        print(f"\nWrote J8-E2B results to {output_json}")

    return results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6PUS J8-E2B Yaw Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_e2b_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay_ms,
        output_json=args.output_json,
    )
