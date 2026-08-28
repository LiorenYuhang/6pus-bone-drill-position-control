#!/usr/bin/env python3
"""6PUS J8-E5: Staged Large-Motion Workspace Hardware Validation Runner.

Validates large-range Cartesian displacements on real robot hardware through
the complete, unbypassed ROS 2 topic chain:
  /pus6/target_cartesian_pose -> pus6_kinematics_node -> /pus6/desired_joint_deltas
  -> pus6_control_node -> /pus6/command_joint_steps -> pus6_motor_driver_node (J7 Servo 50Hz)
  -> /pus6/joint_feedback/timed -> pus6_fk_node -> /pus6/fk/state

Staged sequence:
  Translations: +/-5 mm -> +/-10 mm for X, Y, Z
  Rotations:    +/-2 deg -> +/-5 deg -> +/-10 deg for Roll, Pitch, Yaw
  Profile per stage: HOME -> +target -> HOME -> -target -> HOME
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
from typing import Any

import numpy as np

# ROS 2 imports
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_RATE = 921600
MOTOR_IDS = [1, 2, 3, 4, 5, 6]


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def smooth_ramp(s: float) -> float:
    """Quintic S-curve ramp in [0, 1]."""
    s_clamped = max(0.0, min(1.0, s))
    return 10.0 * (s_clamped**3) - 15.0 * (s_clamped**4) + 6.0 * (s_clamped**5)


@dataclass
class E5Trace:
    target_stamps_ns: list[int] = field(default_factory=list)
    target_poses: list[list[float]] = field(default_factory=list)

    ik_deltas_stamps_ns: list[int] = field(default_factory=list)
    ik_deltas_m: list[list[float]] = field(default_factory=list)

    cmd_stamps_ns: list[int] = field(default_factory=list)
    cmd_steps: list[list[int]] = field(default_factory=list)

    feedback_rx_stamps_ns: dict[int, list[int]] = field(
        default_factory=lambda: {mid: [] for mid in MOTOR_IDS}
    )
    feedback_meas_steps: dict[int, list[int]] = field(
        default_factory=lambda: {mid: [] for mid in MOTOR_IDS}
    )

    fk_query_stamps_ns: list[int] = field(default_factory=list)
    fk_poses: list[list[float]] = field(default_factory=list)


class Ros2E5Harness(Node):
    def __init__(self) -> None:
        super().__init__("j8_e5_harness")
        self.trace = E5Trace()
        self.lock = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", 10
        )

        self.ik_deltas_sub = self.create_subscription(
            Float64MultiArray, "/pus6/desired_joint_deltas", self._on_ik_deltas, 10
        )
        self.cmd_steps_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, sensor_qos
        )
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_feedback, sensor_qos
        )
        self.fk_state_sub = self.create_subscription(
            FkStateStamped, "/pus6/fk/state", self._on_fk_state, sensor_qos
        )

        self._last_known_rx_ns: dict[int, int] = {m: 0 for m in MOTOR_IDS}

    def _on_ik_deltas(self, msg: Float64MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            self.trace.ik_deltas_stamps_ns.append(t_ns)
            self.trace.ik_deltas_m.append(list(msg.data))

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            self.trace.cmd_stamps_ns.append(t_ns)
            self.trace.cmd_steps.append(list(msg.data))

    def _on_feedback(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            for i, mid in enumerate(MOTOR_IDS):
                rx_ns = msg.per_axis_rx_done_monotonic_ns[i]
                if rx_ns > self._last_known_rx_ns[mid]:
                    self._last_known_rx_ns[mid] = rx_ns
                    self.trace.feedback_rx_stamps_ns[mid].append(rx_ns)
                    self.trace.feedback_meas_steps[mid].append(int(msg.raw_steps[i]))

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        r, p, y = quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        with self.lock:
            self.trace.fk_query_stamps_ns.append(msg.query_monotonic_timestamp_ns)
            self.trace.fk_poses.append([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                r, p, y,
            ])

    def publish_target_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        t_ns = time.monotonic_ns()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
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


def run_stage_profile(
    harness: Ros2E5Harness,
    dof_idx: int,       # 0:X, 1:Y, 2:Z, 3:Roll, 4:Pitch, 5:Yaw
    target_val: float,  # m or rad
    z0: float,
    ramp_dur_s: float = 3.0,
    hold_dur_s: float = 1.0,
    rate_hz: float = 50.0,
) -> tuple[int, int]:
    """Execute HOME -> +target -> HOME -> -target -> HOME profile."""
    dt = 1.0 / rate_hz
    t_start_ns = time.monotonic_ns()

    def send_pose(val: float) -> None:
        pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
        if dof_idx == 2:
            pose[2] = z0 + val
        else:
            pose[dof_idx] = val
        harness.publish_target_pose(*pose)

    # 1. Hold HOME
    n_hold = int(1.5 * rate_hz)
    for _ in range(n_hold):
        send_pose(0.0)
        time.sleep(dt)

    # 2. Ramp to +target
    n_ramp = int(ramp_dur_s * rate_hz)
    for step in range(n_ramp):
        s = smooth_ramp((step + 1) / n_ramp)
        send_pose(s * target_val)
        time.sleep(dt)

    # 3. Hold at +target
    n_hold = int(hold_dur_s * rate_hz)
    for _ in range(n_hold):
        send_pose(target_val)
        time.sleep(dt)

    # 4. Ramp back to HOME
    for step in range(n_ramp):
        s = smooth_ramp((step + 1) / n_ramp)
        send_pose((1.0 - s) * target_val)
        time.sleep(dt)

    # 5. Hold at HOME
    for _ in range(n_hold):
        send_pose(0.0)
        time.sleep(dt)

    # 6. Ramp to -target
    for step in range(n_ramp):
        s = smooth_ramp((step + 1) / n_ramp)
        send_pose(-s * target_val)
        time.sleep(dt)

    # 7. Hold at -target
    for _ in range(n_hold):
        send_pose(-target_val)
        time.sleep(dt)

    # 8. Ramp back to HOME
    for step in range(n_ramp):
        s = smooth_ramp((step + 1) / n_ramp)
        send_pose(-(1.0 - s) * target_val)
        time.sleep(dt)

    # 9. Settle at HOME
    n_settle = int(1.5 * rate_hz)
    for _ in range(n_settle):
        send_pose(0.0)
        time.sleep(dt)

    t_end_ns = time.monotonic_ns()
    return t_start_ns, t_end_ns


def analyze_stage(
    trace: E5Trace,
    t_start_ns: int,
    t_end_ns: int,
    dof_idx: int,
    target_val: float,
    z0: float,
    home_fk: list[float],
) -> dict[str, Any]:
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps) if trace.cmd_steps else np.empty((0, 6))

    # 1. Joint Tracking
    joint_metrics = {}
    slider_min_meas = [999999] * 6
    slider_max_meas = [-999999] * 6
    slider_min_cmd = [999999] * 6
    slider_max_cmd = [-999999] * 6

    for i, mid in enumerate(MOTOR_IDS):
        rx_stamps = np.array(trace.feedback_rx_stamps_ns[mid])
        meas_steps = np.array(trace.feedback_meas_steps[mid])
        mask = (rx_stamps >= t_start_ns) & (rx_stamps <= t_end_ns)
        rx_in = rx_stamps[mask]
        meas_in = meas_steps[mask]

        if len(rx_in) > 0 and len(cmd_stamps) > 0:
            cmd_interp = np.interp(rx_in, cmd_stamps, cmd_steps_mat[:, i])
            errs = cmd_interp - meas_in
            slider_min_meas[i] = int(np.min(meas_in))
            slider_max_meas[i] = int(np.max(meas_in))
            slider_min_cmd[i] = int(np.min(cmd_interp))
            slider_max_cmd[i] = int(np.max(cmd_interp))

            joint_metrics[f"ID{mid}"] = {
                "rms_err_steps": float(np.sqrt(np.mean(errs**2))),
                "peak_err_steps": float(np.max(np.abs(errs))),
                "rms_err_um": float(np.sqrt(np.mean(errs**2)) * 25.0),
                "peak_err_um": float(np.max(np.abs(errs)) * 25.0),
                "min_meas_step": slider_min_meas[i],
                "max_meas_step": slider_max_meas[i],
            }

    # 2. Cartesian FK Response
    fk_stamps = np.array(trace.fk_query_stamps_ns)
    fk_poses_mat = np.array(trace.fk_poses) if trace.fk_poses else np.empty((0, 6))

    fk_mask = (fk_stamps >= t_start_ns) & (fk_stamps <= t_end_ns)
    fk_in = fk_poses_mat[fk_mask]

    cartesian_metrics = {}
    if len(fk_in) > 10:
        if dof_idx == 2:
            sig = fk_in[:, dof_idx] - home_fk[dof_idx]
        elif dof_idx < 3:
            sig = fk_in[:, dof_idx] - home_fk[dof_idx]
        else:
            sig = (fk_in[:, dof_idx] - home_fk[dof_idx]) * 180.0 / math.pi

        pos_peak = float(np.max(sig))
        neg_peak = float(np.min(sig))
        scale = 1000.0 if dof_idx < 3 else (180.0 / math.pi)
        tgt_scaled = target_val * scale

        pos_meas_val = pos_peak * (1000.0 if dof_idx < 3 else 1.0)
        neg_meas_val = neg_peak * (1000.0 if dof_idx < 3 else 1.0)

        cartesian_metrics = {
            "pos_fk_peak": pos_meas_val,
            "neg_fk_peak": neg_meas_val,
            "target_val_scaled": tgt_scaled,
            "pos_gain_pct": float(pos_meas_val / tgt_scaled * 100.0) if tgt_scaled != 0 else 100.0,
            "neg_gain_pct": float(abs(neg_meas_val) / tgt_scaled * 100.0) if tgt_scaled != 0 else 100.0,
        }

    return {
        "joint_metrics": joint_metrics,
        "slider_step_envelope_cmd": [int(min(slider_min_cmd)), int(max(slider_max_cmd))],
        "slider_step_envelope_meas": [int(min(slider_min_meas)), int(max(slider_max_meas))],
        "cartesian_metrics": cartesian_metrics,
    }


def dof_idx_name(idx: int) -> str:
    names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
    return names[idx] if 0 <= idx < 6 else str(idx)


def main() -> None:
    parser = argparse.ArgumentParser(description="6PUS J8-E5 Large-Motion Staged Hardware Validation")
    parser.add_argument("--output-json", type=str, default="j8_e5_results.json")
    args = parser.parse_args()

    print("=" * 90)
    print("      6PUS J8-E5: STAGED LARGE-MOTION WORKSPACE HARDWARE VALIDATION")
    print("=" * 90)
    print(f"Hardware Port: {SERIAL_PORT} @ {BAUD_RATE}")
    print("=" * 90)

    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    print(f"Geometry Nominal Cartesian HOME: z0 = {z0:.6f} m ({z0*1000:.3f} mm)\n")

    rclpy.init()

    # Create ROS 2 Nodes with expanded envelope parameters (+/-25mm, +/-0.25rad)
    kin_node = Pus6KinematicsNode()
    kin_node.max_xy = 0.025
    kin_node.max_z_delta = 0.025
    kin_node.max_angle = 0.25

    ctrl_node = Pus6ControlNode()
    motor_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0, query_rate_hz=200.0)
    harness = Ros2E5Harness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kin_node)
    executor.add_node(ctrl_node)
    executor.add_node(motor_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.\n")

    time.sleep(1.0)

    # Initial HOME Hold
    print("--- [Phase 0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
    for _ in range(150):
        harness.publish_target_pose(0.0, 0.0, z0, 0.0, 0.0, 0.0)
        time.sleep(0.02)

    time.sleep(1.0)
    with harness.lock:
        home_poses = list(harness.trace.fk_poses)
    if not home_poses:
        print("[ERROR] No FK poses received during initial HOME phase!")
        sys.exit(1)
    home_fk = home_poses[-1]
    print(
        f"[Phase 0 PASS] Initial Settled HOME FK: X={home_fk[0]*1000:+.3f}mm, "
        f"Y={home_fk[1]*1000:+.3f}mm, Z={home_fk[2]*1000:+.3f}mm, "
        f"R={math.degrees(home_fk[3]):+.3f}°, P={math.degrees(home_fk[4]):+.3f}°, Y={math.degrees(home_fk[5]):+.3f}°\n"
    )

    # Define validation stages: (name, dof_idx, target_val, unit_str, scale)
    stages = [
        # Translations: +/-5mm -> +/-10mm
        ("X_5mm", 0, 0.005, "mm", 1000.0),
        ("X_10mm", 0, 0.010, "mm", 1000.0),
        ("Y_5mm", 1, 0.005, "mm", 1000.0),
        ("Y_10mm", 1, 0.010, "mm", 1000.0),
        ("Z_5mm", 2, 0.005, "mm", 1000.0),
        ("Z_10mm", 2, 0.010, "mm", 1000.0),
        # Rotations: +/-2deg -> +/-5deg -> +/-10deg
        ("Roll_2deg", 3, math.radians(2.0), "deg", 180.0/math.pi),
        ("Roll_5deg", 3, math.radians(5.0), "deg", 180.0/math.pi),
        ("Roll_10deg", 3, math.radians(10.0), "deg", 180.0/math.pi),
        ("Pitch_2deg", 4, math.radians(2.0), "deg", 180.0/math.pi),
        ("Pitch_5deg", 4, math.radians(5.0), "deg", 180.0/math.pi),
        ("Pitch_10deg", 4, math.radians(10.0), "deg", 180.0/math.pi),
        ("Yaw_2deg", 5, math.radians(2.0), "deg", 180.0/math.pi),
        ("Yaw_5deg", 5, math.radians(5.0), "deg", 180.0/math.pi),
        ("Yaw_10deg", 5, math.radians(10.0), "deg", 180.0/math.pi),
    ]

    all_stage_results = {}

    print("--- [Phase 1] Executing Staged Large-Motion Workspace Validation ---")
    for name, dof_idx, target_val, unit_str, scale in stages:
        tgt_disp = target_val * scale
        print(f"\n>> Stage [{name}]: Testing {dof_idx_name(dof_idx)} +/-{tgt_disp:.1f} {unit_str} (HOME -> +{tgt_disp:.1f} -> HOME -> -{tgt_disp:.1f} -> HOME)...")

        t_start, t_end = run_stage_profile(
            harness=harness,
            dof_idx=dof_idx,
            target_val=target_val,
            z0=z0,
            ramp_dur_s=2.5,
            hold_dur_s=1.0,
            rate_hz=50.0,
        )

        res = analyze_stage(harness.trace, t_start, t_end, dof_idx, target_val, z0, home_fk)
        all_stage_results[name] = res

        # Check safety gating
        step_env = res["slider_step_envelope_meas"]
        cart = res["cartesian_metrics"]
        print(f"   Slider Step Envelope: [{step_env[0]}, {step_env[1]}] steps")
        print(f"   FK Measured Peak: +{cart.get('pos_fk_peak', 0.0):.2f} {unit_str} ({cart.get('pos_gain_pct', 0.0):.1f}%), {cart.get('neg_fk_peak', 0.0):.2f} {unit_str} ({cart.get('neg_gain_pct', 0.0):.1f}%)")

        # Max joint tracking RMS
        max_rms_steps = max(m["rms_err_steps"] for m in res["joint_metrics"].values()) if res["joint_metrics"] else 0.0
        print(f"   Max Joint Tracking RMS: {max_rms_steps:.2f} steps ({max_rms_steps*25.0:.1f} um)")

        if step_env[0] < 200 or step_env[1] > 1800:
            print(f"[FATAL] Slider step limit exceeded! Stopping validation.")
            break

    # Phase 2: Settle and Return HOME Repeatability
    print("\n--- [Phase 2] Returning to Cartesian OPERATING_HOME & Measuring Final Repeatability (4.0s) ---")
    for _ in range(200):
        harness.publish_target_pose(0.0, 0.0, z0, 0.0, 0.0, 0.0)
        time.sleep(0.02)

    time.sleep(1.0)
    with harness.lock:
        final_home_fk = harness.trace.fk_poses[-1]

    dx_mm = (final_home_fk[0] - home_fk[0]) * 1000.0
    dy_mm = (final_home_fk[1] - home_fk[1]) * 1000.0
    dz_mm = (final_home_fk[2] - home_fk[2]) * 1000.0
    dr_deg = math.degrees(final_home_fk[3] - home_fk[3])
    dp_deg = math.degrees(final_home_fk[4] - home_fk[4])
    dyaw_deg = math.degrees(final_home_fk[5] - home_fk[5])

    print(
        f"\n[Phase 2 PASS] Final Settled HOME FK: X={final_home_fk[0]*1000:+.3f}mm, "
        f"Y={final_home_fk[1]*1000:+.3f}mm, Z={final_home_fk[2]*1000:+.3f}mm, "
        f"R={math.degrees(final_home_fk[3]):+.3f}°, P={math.degrees(final_home_fk[4]):+.3f}°, Y={math.degrees(final_home_fk[5]):+.3f}°"
    )
    print(f"Return-to-HOME Hysteresis: dX={dx_mm:+.3f}mm, dY={dy_mm:+.3f}mm, dZ={dz_mm:+.3f}mm, dRoll={dr_deg:+.3f}°, dPitch={dp_deg:+.3f}°, dYaw={dyaw_deg:+.3f}°")

    # Shutdown
    print("\n[SHUTDOWN] Shutting down ROS2 executor and nodes...")
    executor.shutdown()
    kin_node.destroy_node()
    ctrl_node.destroy_node()
    motor_node.destroy_node()
    fk_node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()

    # Summary Output
    summary = {
        "initial_home_fk": home_fk,
        "final_home_fk": final_home_fk,
        "hysteresis_trans_mm": [dx_mm, dy_mm, dz_mm],
        "hysteresis_rot_deg": [dr_deg, dp_deg, dyaw_deg],
        "stages": all_stage_results,
        "total_targets_pub": len(harness.trace.target_stamps_ns),
        "total_ik_rcv": len(harness.trace.ik_deltas_stamps_ns),
        "total_cmd_rcv": len(harness.trace.cmd_stamps_ns),
        "total_fk_rcv": len(harness.trace.fk_query_stamps_ns),
    }

    out_path = Path(args.output_json)
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nWrote J8-E5 results to {out_path.resolve()}")


if __name__ == "__main__":
    main()
