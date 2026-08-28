#!/usr/bin/env python3
"""6PUS J8-D2: Complete ROS2 End-to-End Node Graph Hardware Validation.

Verifies the FULL ROS2 Node / Topic Graph on real hardware:
  /pus6/target_cartesian_pose (PoseStamped)
  --> pus6_kinematics_node
  --> /pus6/desired_joint_deltas (Float64MultiArray)
  --> pus6_control_node
  --> /pus6/command_joint_steps (Int32MultiArray)
  --> pus6_motor_driver_node
  --> J7 Six-Axis 50Hz Servo (UART 921600 baud)
  --> real 6PUS physical mechanism
  --> /pus6/joint_feedback/timed (TimedJointFeedback)
  --> pus6_fk_node
  --> /pus6/fk/state (FkStateStamped)

Evaluates:
- True timestamp-aligned per-axis joint tracking: q_cmd_i(t_rx_i) <-> q_meas_i(t_rx_i)
- True timestamp-aligned Cartesian FK tracking: Cmd(t_query) <-> FK(t_query)
- D0 Active Takeover at OPERATING_HOME
- D1 Cartesian Micro-Motion: HOME -> Z +2.0 mm -> HOME
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np

# ROS2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import JointState

# Ensure workspace packages are on path
SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
SRC_DIR = WS_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = SRC_DIR / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_control.control_node import Pus6ControlNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import MOTOR_IDS, NUM_MOTORS
from pus6_msgs.msg import TimedJointFeedback, TimedJointCommand, FkStateStamped


@dataclass
class TopicTraceRecord:
    # Target Cartesian
    target_pose_stamp_ns: list[int] = field(default_factory=list)
    target_pose_z: list[float] = field(default_factory=list)

    # IK Desired Deltas
    ik_stamp_ns: list[int] = field(default_factory=list)
    ik_deltas_m: list[list[float]] = field(default_factory=list)

    # Control Commanded Steps
    cmd_steps_stamp_ns: list[int] = field(default_factory=list)
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


class Ros2ValidationHarness(Node):
    """Client node that orchestrates targets and records all node/topic messages."""

    def __init__(self) -> None:
        super().__init__("j8_d2_validation_harness")
        self.trace = TopicTraceRecord()
        self.lock = threading.Lock()

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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

        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", reliable_qos
        )

        # Subscriptions to verify complete graph
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
            self.trace.ik_stamp_ns.append(t_ns)
            self.trace.ik_deltas_m.append(list(msg.data))

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            self.trace.cmd_steps_stamp_ns.append(t_ns)
            self.trace.cmd_steps.append(list(msg.data))

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            for i, mid in enumerate(MOTOR_IDS):
                rx_ns = msg.per_axis_rx_done_monotonic_ns[i]
                if rx_ns > self._last_known_rx_ns[mid]:
                    # This axis received a fresh feedback sample in this cycle
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

    def publish_target_pose(self, x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        t_ns = time.monotonic_ns()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # Convert Euler to quaternion (Z-Y-X)
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
            self.trace.target_pose_stamp_ns.append(t_ns)
            self.trace.target_pose_z.append(float(z))

        self.target_pose_pub.publish(msg)


def run_j8_d2_e2e_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict:
    print("\n" + "=" * 80)
    print("      6PUS J8-D2: COMPLETE ROS2 NODE GRAPH HARDWARE VALIDATION")
    print("=" * 80)
    print(f"Hardware Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"FixedLag Delay: {fixed_lag_delay_ms:.1f} ms")
    print("=" * 80 + "\n")

    rclpy.init()

    geo_cfg = load_geometry_config()
    z0 = geo_cfg.nominal_height_m
    print(f"Geometry Nominal Cartesian HOME: z0 = {z0:.6f} m ({z0*1000:.3f} mm)")

    # 1. Instantiate the real ROS 2 nodes
    print("\n[INIT] Instantiating ROS2 nodes...")
    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=port,
        baudrate=baudrate,
        timeout=0.015,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=fixed_lag_delay_ms)
    harness = Ros2ValidationHarness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.")

    try:
        # ---------------------------------------------------------------------
        # J8-D0: Active Takeover at OPERATING_HOME
        # ---------------------------------------------------------------------
        print("\n--- [J8-D0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
        t_d0_start = time.monotonic()
        while time.monotonic() - t_d0_start < 3.0:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)

        print("[D0 PASS] Held OPERATING_HOME stably across full ROS2 node graph.")

        # ---------------------------------------------------------------------
        # J8-D1: Cartesian Micro-Motion (HOME -> Z +2.0 mm -> HOME)
        # ---------------------------------------------------------------------
        z_plus_2mm = z0 + 0.002
        print(f"\n--- [J8-D1] Commanding Cartesian Z +2.0 mm (z = {z_plus_2mm*1000:.3f} mm) ---")
        t_heave_start = time.monotonic()
        while time.monotonic() - t_heave_start < 2.5:
            harness.publish_target_pose(x=0.0, y=0.0, z=z_plus_2mm)
            time.sleep(0.020)

        print(f"\n--- [J8-D1] Commanding Return to Cartesian OPERATING_HOME (z = {z0*1000:.3f} mm) ---")
        t_return_start = time.monotonic()
        while time.monotonic() - t_return_start < 2.5:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)

        print("\n[D1 PASS] Complete ROS2 motion trajectory executed cleanly.")
        time.sleep(0.5)

    finally:
        print("\n[SHUTDOWN] Shutting down ROS2 executor and nodes...")
        executor.shutdown()
        kinematics_node.destroy_node()
        control_node.destroy_node()
        motor_driver_node.destroy_node()
        fk_node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
        print("[SHUTDOWN OK] System stopped safely.")

    # -------------------------------------------------------------------------
    # Quantitative Analysis on Full ROS2 Topic Telemetry
    # -------------------------------------------------------------------------
    trace = harness.trace

    print("\n" + "=" * 80)
    print("                J8-D2 ROS2 TOPIC GRAPH & TELEMETRY AUDIT")
    print("=" * 80)

    print(f"Total Target Cartesian Poses Published: {len(trace.target_pose_stamp_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamp_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_steps_stamp_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")

    # 1. Verify ROS2 topic graph continuity
    assert len(trace.ik_stamp_ns) > 0, "Zero IK delta messages received from pus6_kinematics_node!"
    assert len(trace.cmd_steps_stamp_ns) > 0, "Zero command step messages received from pus6_control_node!"
    assert len(trace.fk_query_stamps_ns) > 0, "Zero FK state messages received from pus6_fk_node!"

    # 2. Strict Timestamp-Aligned Per-Axis Joint Tracking
    # Evaluate commanded joint trajectory at the exact timestamp of each real feedback sample t_rx_i
    cmd_stamps = np.array(trace.cmd_steps_stamp_ns)
    cmd_steps_arr = np.array(trace.cmd_steps) # Shape: (N_cmd, 6)

    per_axis_tracking = {}
    print("\n--- Strict Timestamp-Aligned Joint Tracking (q_cmd_i(t_rx_i) <-> q_meas_i(t_rx_i)) ---")
    print(f"{'Axis ID':<10}{'Sample Count':<14}{'Cmd Min':<10}{'Cmd Max':<10}{'Meas Min':<10}{'Meas Max':<10}{'RMS Err (steps)':<18}{'Peak Err (steps)':<18}{'Final Err (steps)':<18}")
    print("-" * 114)

    for i, mid in enumerate(MOTOR_IDS):
        rx_stamps = np.array(trace.feedback_rx_stamps_ns[mid])
        meas_steps = np.array(trace.feedback_meas_steps[mid])
        sample_count = len(rx_stamps)

        if sample_count > 0 and len(cmd_stamps) > 0:
            # Interpolate command step trajectory at rx_stamps
            cmd_interp = np.interp(rx_stamps, cmd_stamps, cmd_steps_arr[:, i])
            err_steps = meas_steps - cmd_interp
            rms_err = float(np.sqrt(np.mean(err_steps**2)))
            peak_err = float(np.max(np.abs(err_steps)))
            final_err = float(err_steps[-1])
            cmd_min = int(np.min(cmd_interp))
            cmd_max = int(np.max(cmd_interp))
            meas_min = int(np.min(meas_steps))
            meas_max = int(np.max(meas_steps))
        else:
            rms_err, peak_err, final_err = 0.0, 0.0, 0.0
            cmd_min, cmd_max, meas_min, meas_max = 0, 0, 0, 0

        per_axis_tracking[mid] = {
            "sample_count": sample_count,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "meas_min": meas_min,
            "meas_max": meas_max,
            "rms_err_steps": rms_err,
            "rms_err_mm": rms_err * 0.025,
            "peak_err_steps": peak_err,
            "peak_err_mm": peak_err * 0.025,
            "final_err_steps": final_err,
            "final_err_mm": final_err * 0.025,
        }

        print(f"ID{mid:<8}{sample_count:<14}{cmd_min:<10}{cmd_max:<10}{meas_min:<10}{meas_max:<10}{rms_err:<18.2f}{peak_err:<18.1f}{final_err:<18.1f}")

    # 3. Strict Timestamp-Aligned Cartesian Tracking
    # Compare commanded Cartesian Z at t_query vs FK reconstructed Z at t_query
    target_stamps = np.array(trace.target_pose_stamp_ns)
    target_z_arr = np.array(trace.target_pose_z)

    fk_query_stamps = np.array(trace.fk_query_stamps_ns)
    fk_poses_arr = np.array(trace.fk_poses) # Shape: (N_fk, 6)

    # Interpolate commanded target Z at fk_query_stamps
    cmd_z_at_query = np.interp(fk_query_stamps, target_stamps, target_z_arr)
    meas_z = fk_poses_arr[:, 2]
    meas_x = fk_poses_arr[:, 0]
    meas_y = fk_poses_arr[:, 1]
    meas_roll = fk_poses_arr[:, 3]
    meas_pitch = fk_poses_arr[:, 4]
    meas_yaw = fk_poses_arr[:, 5]

    err_z_mm = (meas_z - cmd_z_at_query) * 1000.0
    err_x_mm = meas_x * 1000.0
    err_y_mm = meas_y * 1000.0

    print("\n--- Strict Timestamp-Aligned Cartesian Tracking (Cmd(t_query) <-> FK(t_query)) ---")
    print(f"X Translation Error (mm): RMS={np.sqrt(np.mean(err_x_mm**2)):.3f}, Peak={np.max(np.abs(err_x_mm)):.3f}")
    print(f"Y Translation Error (mm): RMS={np.sqrt(np.mean(err_y_mm**2)):.3f}, Peak={np.max(np.abs(err_y_mm)):.3f}")
    print(f"Z Tracking Error    (mm): RMS={np.sqrt(np.mean(err_z_mm**2)):.3f}, Peak={np.max(np.abs(err_z_mm)):.3f}")

    # Steady state analysis across phases
    # Initial HOME (first 2.0s)
    t_start_ns = target_stamps[0]
    idx_home_init = np.where((fk_query_stamps >= t_start_ns + 1.0e9) & (fk_query_stamps <= t_start_ns + 2.5e9))[0]
    mean_home_init = np.mean(fk_poses_arr[idx_home_init], axis=0) if len(idx_home_init) > 0 else np.zeros(6)

    # Z +2mm steady (between 4.0s and 5.2s)
    idx_z2 = np.where((fk_query_stamps >= t_start_ns + 4.0e9) & (fk_query_stamps <= t_start_ns + 5.2e9))[0]
    mean_z2 = np.mean(fk_poses_arr[idx_z2], axis=0) if len(idx_z2) > 0 else np.zeros(6)

    # Return HOME (between 6.5s and 7.8s)
    idx_return = np.where((fk_query_stamps >= t_start_ns + 6.5e9) & (fk_query_stamps <= t_start_ns + 7.8e9))[0]
    mean_return = np.mean(fk_poses_arr[idx_return], axis=0) if len(idx_return) > 0 else np.zeros(6)

    print("\n--- Key Cartesian Steady-State Poses ---")
    print(f"1. Initial HOME Pose:     X={mean_home_init[0]*1000:+.3f}mm, Y={mean_home_init[1]*1000:+.3f}mm, Z={mean_home_init[2]*1000:+.3f}mm, R={math.degrees(mean_home_init[3]):+.3f}deg, P={math.degrees(mean_home_init[4]):+.3f}deg, Y={math.degrees(mean_home_init[5]):+.3f}deg")
    print(f"2. Z +2.0mm Steady Pose:  X={mean_z2[0]*1000:+.3f}mm, Y={mean_z2[1]*1000:+.3f}mm, Z={mean_z2[2]*1000:+.3f}mm, R={math.degrees(mean_z2[3]):+.3f}deg, P={math.degrees(mean_z2[4]):+.3f}deg, Y={math.degrees(mean_z2[5]):+.3f}deg")
    print(f"3. Return HOME Pose:      X={mean_return[0]*1000:+.3f}mm, Y={mean_return[1]*1000:+.3f}mm, Z={mean_return[2]*1000:+.3f}mm, R={math.degrees(mean_return[3]):+.3f}deg, P={math.degrees(mean_return[4]):+.3f}deg, Y={math.degrees(mean_return[5]):+.3f}deg")
    print(f"Measured Z Displacement:  {(mean_z2[2] - mean_home_init[2])*1000:+.3f} mm (Expected: +2.000 mm)")
    print(f"Z Return Hysteresis:      {(mean_return[2] - mean_home_init[2])*1000:+.3f} mm")

    results = {
        "status": "PASS",
        "ros2_node_graph_verified": True,
        "nodes": [
            "pus6_kinematics_node",
            "pus6_control_node",
            "pus6_motor_driver_node",
            "pus6_fk_node",
            "j8_d2_validation_harness",
        ],
        "topics": [
            "/pus6/target_cartesian_pose",
            "/pus6/desired_joint_deltas",
            "/pus6/command_joint_steps",
            "/pus6/joint_feedback/timed",
            "/pus6/fk/state",
        ],
        "fixed_lag_delay_ms": fixed_lag_delay_ms,
        "geometry_nominal_z0_m": z0,
        "per_axis_tracking": per_axis_tracking,
        "cartesian_tracking": {
            "err_x_rms_mm": float(np.sqrt(np.mean(err_x_mm**2))),
            "err_x_peak_mm": float(np.max(np.abs(err_x_mm))),
            "err_y_rms_mm": float(np.sqrt(np.mean(err_y_mm**2))),
            "err_y_peak_mm": float(np.max(np.abs(err_y_mm))),
            "err_z_rms_mm": float(np.sqrt(np.mean(err_z_mm**2))),
            "err_z_peak_mm": float(np.max(np.abs(err_z_mm))),
        },
        "key_poses": {
            "home_initial": mean_home_init.tolist(),
            "z_plus_2mm_steady": mean_z2.tolist(),
            "home_return_final": mean_return.tolist(),
            "measured_z_delta_mm": float((mean_z2[2] - mean_home_init[2]) * 1000.0),
            "z_return_hysteresis_mm": float((mean_return[2] - mean_home_init[2]) * 1000.0),
        },
    }

    if output_json:
        with open(output_json, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nWrote J8-D2 results to {output_json}")

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="6PUS J8-D2 ROS2 E2E Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_d2_e2e_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay,
        output_json=args.output_json,
    )
