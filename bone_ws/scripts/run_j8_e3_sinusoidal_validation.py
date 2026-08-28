#!/usr/bin/env python3
"""6PUS J8-E3: Single-DoF Continuous Cartesian Sinusoidal Hardware Validation.

Executes continuous sinusoidal Cartesian motions strictly across the complete ROS 2 topic graph:
  /pus6/target_cartesian_pose -> pus6_kinematics_node
  -> /pus6/desired_joint_deltas -> pus6_control_node
  -> /pus6/command_joint_steps -> pus6_motor_driver_node
  -> J7 50Hz Servo -> Real Manipulator
  -> /pus6/joint_feedback/timed -> pus6_fk_node
  -> /pus6/fk/state

Tests:
  - Translation DoFs (X, Y, Z): Amplitude = +/- 2.0 mm, Frequencies = [0.1, 0.2, 0.5] Hz (2 full cycles each)
  - Orientation DoFs (Roll, Pitch): Amplitude = +/- 0.5 deg, Frequencies = [0.1, 0.2, 0.5] Hz (2 full cycles each)
  - Orientation DoF (Yaw): Amplitude = +/- 2.0 deg, Frequencies = [0.1, 0.2, 0.5] Hz (2 full cycles each)
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
class SineTrace:
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


class Ros2SineHarness(Node):
    def __init__(self) -> None:
        super().__init__("j8_e3_sine_harness")
        self.trace = SineTrace()
        self.lock = threading.Lock()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
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


def compute_cross_correlation_lag_sec(
    t_sec: np.ndarray,
    signal1: np.ndarray,
    signal2: np.ndarray,
    max_lag_sec: float = 1.0,
) -> float:
    """Computes time lag of signal2 relative to signal1 using cross-correlation."""
    if len(t_sec) < 10:
        return 0.0
    dt = float(np.median(np.diff(t_sec)))
    if dt <= 0:
        return 0.0
    s1 = signal1 - np.mean(signal1)
    s2 = signal2 - np.mean(signal2)
    if np.std(s1) < 1e-6 or np.std(s2) < 1e-6:
        return 0.0
    corr = np.correlate(s1, s2, mode="full")
    lags = np.arange(-len(s1) + 1, len(s1)) * dt
    mask = np.abs(lags) <= max_lag_sec
    if not np.any(mask):
        return 0.0
    best_lag = lags[mask][np.argmax(corr[mask])]
    return float(best_lag)


def analyze_sinusoidal_window(
    trace: SineTrace,
    t_start_ns: int,
    t_end_ns: int,
    axis_idx: int,  # 0:X, 1:Y, 2:Z, 3:Roll, 4:Pitch, 5:Yaw
    freq_hz: float,
    cmd_amp: float,
    home_pose_fk: list[float],
    z0: float,
    kin: Analytical6PusKinematics,
) -> dict[str, Any]:
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps) if trace.cmd_steps else np.empty((0, 6))

    # Joint Tracking Metrics
    joint_metrics = {}
    for i, mid in enumerate(MOTOR_IDS):
        rx_stamps = np.array(trace.feedback_rx_stamps_ns[mid])
        meas_steps = np.array(trace.feedback_meas_steps[mid])
        mask = (rx_stamps >= t_start_ns) & (rx_stamps <= t_end_ns)
        rx_in = rx_stamps[mask]
        meas_in = meas_steps[mask]

        if len(rx_in) > 0 and len(cmd_stamps) > 0:
            cmd_interp = np.interp(rx_in, cmd_stamps, cmd_steps_mat[:, i])
            errs = cmd_interp - meas_in
            joint_metrics[f"ID{mid}"] = {
                "sample_count": int(len(rx_in)),
                "rms_err_steps": float(np.sqrt(np.mean(errs**2))),
                "peak_err_steps": float(np.max(np.abs(errs))),
                "final_err_steps": float(errs[-1]),
            }
        else:
            joint_metrics[f"ID{mid}"] = {
                "sample_count": 0,
                "rms_err_steps": 0.0,
                "peak_err_steps": 0.0,
                "final_err_steps": 0.0,
            }

    # Cartesian 3-way Metrics
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
            t_sec = (fk_times - fk_times[0]) / 1e9

            # 1. Target
            x_target = np.zeros_like(fk_in_window)
            for col in range(6):
                x_target[:, col] = np.interp(fk_times, target_stamps, target_poses_mat[:, col])

            # 2. Shaped pose FK(q_cmd)
            q_cmd_interp = np.zeros((len(fk_times), 6))
            for col in range(6):
                q_cmd_interp[:, col] = np.interp(fk_times, cmd_stamps, cmd_steps_mat[:, col])

            x_shaped = np.zeros_like(fk_in_window)
            for row in range(len(fk_times)):
                joint_m = [0.05 + (s - 1000) / 40000.0 for s in q_cmd_interp[row]]
                pose_shaped = kin.forward_kinematics(joint_m)
                x_shaped[row, 0:3] = pose_shaped[0:3]
                x_shaped[row, 3:6] = pose_shaped[3:6]

            # 3. Measured FK pose
            x_fk = fk_in_window

            # Select active axis data
            is_translation = axis_idx < 3
            if is_translation:
                # Convert to mm
                unit = "mm"
                scale = 1000.0
                offset_home = home_pose_fk[axis_idx] if axis_idx != 2 else z0
                tgt_sig = (x_target[:, axis_idx] - (0.0 if axis_idx != 2 else z0)) * scale
                shp_sig = (x_shaped[:, axis_idx] - (0.0 if axis_idx != 2 else z0)) * scale
                fk_sig = (x_fk[:, axis_idx] - home_pose_fk[axis_idx]) * scale
            else:
                # Convert to deg
                unit = "deg"
                scale = 180.0 / math.pi
                home_angle = home_pose_fk[axis_idx]
                tgt_sig = x_target[:, axis_idx] * scale
                shp_sig = x_shaped[:, axis_idx] * scale
                fk_sig = (x_fk[:, axis_idx] - home_angle) * scale

            # Amplitudes
            amp_tgt = float((np.max(tgt_sig) - np.min(tgt_sig)) * 0.5)
            amp_shp = float((np.max(shp_sig) - np.min(shp_sig)) * 0.5)
            amp_fk = float((np.max(fk_sig) - np.min(fk_sig)) * 0.5)

            gain_e2e = amp_fk / amp_tgt if amp_tgt > 1e-4 else 1.0
            gain_shp = amp_shp / amp_tgt if amp_tgt > 1e-4 else 1.0
            gain_exc = amp_fk / amp_shp if amp_shp > 1e-4 else 1.0

            # Errors
            err_e2e = fk_sig - tgt_sig
            err_shp = shp_sig - tgt_sig
            err_exc = fk_sig - shp_sig

            # Phase Lags (seconds and degrees)
            lag_e2e_s = compute_cross_correlation_lag_sec(t_sec, tgt_sig, fk_sig)
            lag_shp_s = compute_cross_correlation_lag_sec(t_sec, tgt_sig, shp_sig)
            lag_exc_s = compute_cross_correlation_lag_sec(t_sec, shp_sig, fk_sig)

            phase_e2e_deg = lag_e2e_s * freq_hz * 360.0
            phase_shp_deg = lag_shp_s * freq_hz * 360.0
            phase_exc_deg = lag_exc_s * freq_hz * 360.0

            # SO(3) Geodesic Rotational Error
            so3_errs = []
            for k in range(len(fk_times)):
                R_meas = euler_to_matrix(x_fk[k, 3], x_fk[k, 4], x_fk[k, 5])
                R_cmd = euler_to_matrix(x_target[k, 3], x_target[k, 4], x_target[k, 5])
                so3_errs.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_cmd)))

            cart_metrics = {
                "sample_count": int(len(fk_times)),
                "unit": unit,
                "amp_target": amp_tgt,
                "amp_shaped": amp_shp,
                "amp_fk": amp_fk,
                "gain_e2e_pct": float(gain_e2e * 100.0),
                "gain_shape_pct": float(gain_shp * 100.0),
                "gain_exec_pct": float(gain_exc * 100.0),
                "e2e_rms": float(np.sqrt(np.mean(err_e2e**2))),
                "e2e_peak": float(np.max(np.abs(err_e2e))),
                "shape_rms": float(np.sqrt(np.mean(err_shp**2))),
                "shape_peak": float(np.max(np.abs(err_shp))),
                "exec_rms": float(np.sqrt(np.mean(err_exc**2))),
                "exec_peak": float(np.max(np.abs(err_exc))),
                "lag_e2e_ms": float(lag_e2e_s * 1000.0),
                "lag_shape_ms": float(lag_shp_s * 1000.0),
                "lag_exec_ms": float(lag_exc_s * 1000.0),
                "phase_e2e_deg": float(phase_e2e_deg),
                "phase_shape_deg": float(phase_shp_deg),
                "phase_exec_deg": float(phase_exc_deg),
                "so3_rms_deg": float(np.sqrt(np.mean(np.array(so3_errs)**2))),
                "so3_peak_deg": float(np.max(np.abs(np.array(so3_errs)))),
            }

    return {
        "joint_metrics": joint_metrics,
        "cartesian_metrics": cart_metrics,
    }


def run_j8_e3_sinusoidal_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict[str, Any]:
    print("\n" + "=" * 90)
    print("      6PUS J8-E3: SINGLE-DOF CONTINUOUS CARTESIAN SINUSOIDAL VALIDATION")
    print("=" * 90)
    print(f"Hardware Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"FixedLag Delay: {fixed_lag_delay_ms:.1f} ms")
    print("=" * 90 + "\n")

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
    harness = Ros2SineHarness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.")

    # Sweep definitions: 6 DoFs across [0.1, 0.2, 0.5] Hz
    dofs = [
        ("X", 0, 0.002, 0.0, "mm"),
        ("Y", 1, 0.002, 0.0, "mm"),
        ("Z", 2, 0.002, 0.0, "mm"),
        ("Roll", 3, 0.0, math.radians(0.50), "deg"),
        ("Pitch", 4, 0.0, math.radians(0.50), "deg"),
        ("Yaw", 5, 0.0, math.radians(2.00), "deg"),
    ]
    freqs = [0.1, 0.2, 0.5]

    timing_windows = {}

    try:
        # Phase 0: Hold OPERATING_HOME (3.0s)
        print("\n--- [Phase 0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)

        with harness.lock:
            if harness.trace.fk_poses:
                home_fk = list(harness.trace.fk_poses[-1])
            else:
                home_fk = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
        print(f"[Phase 0 PASS] Initial Settled HOME FK: X={home_fk[0]*1e3:+.3f}mm, Y={home_fk[1]*1e3:+.3f}mm, Z={home_fk[2]*1e3:+.3f}mm, R={math.degrees(home_fk[3]):+.3f}°, P={math.degrees(home_fk[4]):+.3f}°, Y={math.degrees(home_fk[5]):+.3f}°")

        # Execute sinusoidal tests
        for dof_name, axis_idx, trans_amp_m, rot_amp_rad, unit in dofs:
            amp_val = trans_amp_m * 1000.0 if axis_idx < 3 else math.degrees(rot_amp_rad)
            for f_hz in freqs:
                tag = f"{dof_name.lower()}_{int(f_hz*10)}hz"
                label = f"{dof_name} +/-{amp_val:.1f}{unit} @ {f_hz:.1f}Hz"
                print(f"\n--- [Stage {label}] Executing continuous sinusoidal trajectory ---")

                # Duration: 2 full cycles (minimum 4.0s for 0.5Hz, 10.0s for 0.2Hz, 20.0s for 0.1Hz -> let's do 2 cycles)
                cycles = 2
                dur_s = max(4.0, (cycles / f_hz))
                t_start_ns = time.monotonic_ns()
                t_start_s = time.monotonic()
                t_end_s = t_start_s + dur_s

                while time.monotonic() < t_end_s:
                    t_curr = time.monotonic() - t_start_s
                    # Smooth sin: A * sin(2*pi*f*t)
                    phase = 2.0 * math.pi * f_hz * t_curr
                    val = math.sin(phase)

                    px, py, pz = 0.0, 0.0, z0
                    pr, pp, pyaw = 0.0, 0.0, 0.0

                    if axis_idx == 0:
                        px = trans_amp_m * val
                    elif axis_idx == 1:
                        py = trans_amp_m * val
                    elif axis_idx == 2:
                        pz = z0 + trans_amp_m * val
                    elif axis_idx == 3:
                        pr = rot_amp_rad * val
                    elif axis_idx == 4:
                        pp = rot_amp_rad * val
                    elif axis_idx == 5:
                        pyaw = rot_amp_rad * val

                    harness.publish_target_pose(x=px, y=py, z=pz, roll=pr, pitch=pp, yaw=pyaw)
                    time.sleep(0.020)

                t_end_ns = time.monotonic_ns()

                # Settle at HOME for 1.5s between tests
                t_settle = time.monotonic() + 1.5
                while time.monotonic() < t_settle:
                    harness.publish_target_pose(x=0.0, y=0.0, z=z0)
                    time.sleep(0.020)

                timing_windows[tag] = {
                    "start_ns": t_start_ns,
                    "end_ns": t_end_ns,
                    "dof_name": dof_name,
                    "axis_idx": axis_idx,
                    "freq_hz": f_hz,
                    "amp_val": amp_val,
                    "label": label,
                }
                print(f"[{label} PASS] Completed cleanly.")

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

    print("\n" + "=" * 95)
    print("                J8-E3 SINUSOIDAL VALIDATION RESULTS SUMMARY")
    print("=" * 95)
    print(f"Total Target Cartesian Poses Published: {len(trace.target_stamps_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamps_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_stamps_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")

    print("\n" + "-" * 135)
    print(f"{'Trajectory Test':<24} {'E2E Gain %':<11} {'E2E RMS':<12} {'E2E Lag':<12} {'Shaper Gain':<13} {'Shaper Lag':<12} {'Exec Gain':<11} {'Exec Lag':<11} {'SO(3) RMS'}")
    print("-" * 135)

    for tag, win in timing_windows.items():
        metrics = analyze_sinusoidal_window(
            trace,
            win["start_ns"],
            win["end_ns"],
            win["axis_idx"],
            win["freq_hz"],
            win["amp_val"],
            home_fk,
            z0,
            kin,
        )

        c = metrics["cartesian_metrics"]
        if not c:
            continue

        stage_results[tag] = metrics

        u = c["unit"]
        print(
            f"{win['label']:<24} {c['gain_e2e_pct']:6.1f}%     {c['e2e_rms']:6.3f} {u:<3} {c['lag_e2e_ms']:+6.1f} ms  "
            f"{c['gain_shape_pct']:6.1f}%       {c['lag_shape_ms']:+6.1f} ms   {c['gain_exec_pct']:6.1f}%     {c['lag_exec_ms']:+6.1f} ms   {c['so3_rms_deg']:5.3f}°"
        )

    # Print Joint Steps RMS Table
    print("\n--- Timestamp-Aligned Per-Axis Joint RMS Errors (steps / um) Across Sinusoidal Tests ---")
    print(f"{'Trajectory Test':<24} {'ID1 (steps)':<14} {'ID2 (steps)':<14} {'ID3 (steps)':<14} {'ID4 (steps)':<14} {'ID5 (steps)':<14} {'ID6 (steps)':<14}")
    print("-" * 115)
    for tag, win in timing_windows.items():
        if tag in stage_results:
            jm = stage_results[tag]["joint_metrics"]
            vals = [f"{jm[f'ID{mid}']['rms_err_steps']:4.2f} ({jm[f'ID{mid}']['rms_err_steps']*25:4.1f}um)" for mid in MOTOR_IDS]
            print(f"{win['label']:<24} {vals[0]:<14} {vals[1]:<14} {vals[2]:<14} {vals[3]:<14} {vals[4]:<14} {vals[5]:<14}")

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
        print(f"\nWrote J8-E3 results to {output_json}")

    return results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6PUS J8-E3 Sinusoidal Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_e3_sinusoidal_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay_ms,
        output_json=args.output_json,
    )
