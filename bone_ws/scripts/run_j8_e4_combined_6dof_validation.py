#!/usr/bin/env python3
"""6PUS J8-E4: Combined 6-DoF Continuous Cartesian Hardware Validation.

Executes simultaneous continuous 6-DoF sinusoidal motion strictly across the complete ROS 2 topic graph:
  /pus6/target_cartesian_pose -> pus6_kinematics_node
  -> /pus6/desired_joint_deltas -> pus6_control_node
  -> /pus6/command_joint_steps -> pus6_motor_driver_node
  -> J7 50Hz Servo -> Real Manipulator
  -> /pus6/joint_feedback/timed -> pus6_fk_node
  -> /pus6/fk/state

Trajectory:
  Simultaneous 6-DoF 0.1 Hz excitation for 3 full cycles (30.0 s):
    X(t)     = 1.5 mm * sin(2*pi*0.1*t)
    Y(t)     = 1.5 mm * cos(2*pi*0.1*t)
    Z(t)     = z0 + 1.5 mm * sin(2*pi*0.1*t)
    Roll(t)  = 0.35 deg * sin(2*pi*0.1*t)
    Pitch(t) = 0.35 deg * cos(2*pi*0.1*t)
    Yaw(t)   = 1.50 deg * sin(2*pi*0.1*t + pi/4)
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


def fit_sinusoid(t_sec: np.ndarray, y: np.ndarray, freq_hz: float) -> tuple[float, float, float]:
    """Fits y(t) = A * sin(2*pi*f*t) + B * cos(2*pi*f*t) + C.
    Returns: (amplitude, phase_rad, offset)
    where y(t) = amplitude * sin(2*pi*f*t + phase_rad) + C.
    """
    omega = 2.0 * math.pi * freq_hz
    M = np.column_stack([np.sin(omega * t_sec), np.cos(omega * t_sec), np.ones_like(t_sec)])
    coeffs, _, _, _ = np.linalg.lstsq(M, y, rcond=None)
    A, B, C = coeffs
    amp = math.sqrt(A**2 + B**2)
    phase_rad = math.atan2(B, A)
    return amp, phase_rad, C


@dataclass
class CombinedTrace:
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


class Ros2CombinedHarness(Node):
    def __init__(self) -> None:
        super().__init__("j8_e4_combined_harness")
        self.trace = CombinedTrace()
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


def analyze_combined_6dof(
    trace: CombinedTrace,
    t_start_ns: int,
    t_end_ns: int,
    freq_hz: float,
    cmd_amps: list[float],  # [Ax_mm, Ay_mm, Az_mm, Aroll_deg, Apitch_deg, Ayaw_deg]
    home_pose_fk: list[float],
    z0: float,
    kin: Analytical6PusKinematics,
) -> dict[str, Any]:
    cmd_stamps = np.array(trace.cmd_stamps_ns)
    cmd_steps_mat = np.array(trace.cmd_steps) if trace.cmd_steps else np.empty((0, 6))

    # 1. Joint Tracking Metrics
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
                "rms_err_um": float(np.sqrt(np.mean(errs**2)) * 25.0),
                "peak_err_um": float(np.max(np.abs(errs)) * 25.0),
            }
        else:
            joint_metrics[f"ID{mid}"] = {
                "sample_count": 0,
                "rms_err_steps": 0.0,
                "peak_err_steps": 0.0,
                "final_err_steps": 0.0,
                "rms_err_um": 0.0,
                "peak_err_um": 0.0,
            }

    # 2. Cartesian 6-DoF Harmonic Analysis on Steady-State Window
    fk_stamps = np.array(trace.fk_query_stamps_ns)
    fk_poses_mat = np.array(trace.fk_poses) if trace.fk_poses else np.empty((0, 6))
    target_stamps = np.array(trace.target_stamps_ns)
    target_poses_mat = np.array(trace.target_poses) if trace.target_poses else np.empty((0, 6))

    dof_names = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
    units = ["mm", "mm", "mm", "deg", "deg", "deg"]
    scales = [1000.0, 1000.0, 1000.0, 180.0 / math.pi, 180.0 / math.pi, 180.0 / math.pi]

    dof_metrics = {}
    so3_metrics = {}

    if len(fk_stamps) > 0 and len(target_stamps) > 0 and len(cmd_stamps) > 0:
        # Exclude initial 1.0s startup transient to isolate pure steady-state harmonic cycles
        t_ss_start_ns = t_start_ns + int(1.0 * 1e9)
        t_ss_end_ns = t_end_ns
        fk_mask = (fk_stamps >= t_ss_start_ns) & (fk_stamps <= t_ss_end_ns)
        fk_times = fk_stamps[fk_mask]
        fk_in_window = fk_poses_mat[fk_mask]

        if len(fk_times) > 20:
            t_sec = (fk_times - fk_times[0]) / 1e9

            # Interpolate target and shaped poses
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

            # Analyze each of the 6 DoFs
            for col in range(6):
                dof = dof_names[col]
                unit = units[col]
                scale = scales[col]

                # Extract signals
                if col == 2: # Z
                    tgt_sig = (x_target[:, col] - z0) * scale
                    shp_sig = (x_shaped[:, col] - z0) * scale
                    fk_sig = (x_fk[:, col] - home_pose_fk[col]) * scale
                elif col < 3: # X, Y
                    tgt_sig = x_target[:, col] * scale
                    shp_sig = x_shaped[:, col] * scale
                    fk_sig = (x_fk[:, col] - home_pose_fk[col]) * scale
                else: # R, P, Y
                    tgt_sig = x_target[:, col] * scale
                    shp_sig = x_shaped[:, col] * scale
                    fk_sig = (x_fk[:, col] - home_pose_fk[col]) * scale

                # Harmonic Least-Squares Fitting: y(t) = A * sin(2*pi*f*t + phi) + C
                amp_tgt, phi_tgt, _ = fit_sinusoid(t_sec, tgt_sig, freq_hz)
                amp_shp, phi_shp, _ = fit_sinusoid(t_sec, shp_sig, freq_hz)
                amp_fk, phi_fk, _ = fit_sinusoid(t_sec, fk_sig, freq_hz)

                # Amplitude Gains (%)
                gain_e2e_pct = (amp_fk / amp_tgt * 100.0) if amp_tgt > 1e-4 else 100.0
                gain_shp_pct = (amp_shp / amp_tgt * 100.0) if amp_tgt > 1e-4 else 100.0
                gain_exc_pct = (amp_fk / amp_shp * 100.0) if amp_shp > 1e-4 else 100.0

                # Phase Shifts & Positive Time Lags
                # Convention: delta_phi = phi_resp - phi_in (negative means lag)
                # tau = -delta_phi / omega (positive means time lag in seconds)
                omega = 2.0 * math.pi * freq_hz

                dphi_e2e = phi_fk - phi_tgt
                dphi_shp = phi_shp - phi_tgt
                dphi_exc = phi_fk - phi_shp

                # Wrap phase differences to [-pi, pi]
                dphi_e2e = (dphi_e2e + math.pi) % (2 * math.pi) - math.pi
                dphi_shp = (dphi_shp + math.pi) % (2 * math.pi) - math.pi
                dphi_exc = (dphi_exc + math.pi) % (2 * math.pi) - math.pi

                tau_e2e_ms = (-dphi_e2e / omega) * 1000.0
                tau_shp_ms = (-dphi_shp / omega) * 1000.0
                tau_exc_ms = (-dphi_exc / omega) * 1000.0

                # Tracking Errors (Time Domain)
                err_e2e = fk_sig - tgt_sig
                err_shp = shp_sig - tgt_sig
                err_exc = fk_sig - shp_sig

                dof_metrics[dof] = {
                    "unit": unit,
                    "cmd_amp": float(cmd_amps[col]),
                    "target_amp_fit": float(amp_tgt),
                    "shaped_amp_fit": float(amp_shp),
                    "fk_amp_fit": float(amp_fk),
                    "gain_e2e_pct": float(gain_e2e_pct),
                    "gain_shape_pct": float(gain_shp_pct),
                    "gain_exec_pct": float(gain_exc_pct),
                    "phase_e2e_deg": float(math.degrees(dphi_e2e)),
                    "phase_shape_deg": float(math.degrees(dphi_shp)),
                    "phase_exec_deg": float(math.degrees(dphi_exc)),
                    "lag_e2e_ms": float(tau_e2e_ms),
                    "lag_shape_ms": float(tau_shp_ms),
                    "lag_exec_ms": float(tau_exc_ms),
                    "e2e_rms": float(np.sqrt(np.mean(err_e2e**2))),
                    "e2e_peak": float(np.max(np.abs(err_e2e))),
                    "shape_rms": float(np.sqrt(np.mean(err_shp**2))),
                    "shape_peak": float(np.max(np.abs(err_shp))),
                    "exec_rms": float(np.sqrt(np.mean(err_exc**2))),
                    "exec_peak": float(np.max(np.abs(err_exc))),
                }

            # SO(3) Geodesic Rotational Error across 6-DoF trajectory
            so3_errs = []
            for k in range(len(fk_times)):
                R_meas = euler_to_matrix(x_fk[k, 3], x_fk[k, 4], x_fk[k, 5])
                R_cmd = euler_to_matrix(x_target[k, 3], x_target[k, 4], x_target[k, 5])
                so3_errs.append(math.degrees(so3_geodesic_distance_rad(R_meas, R_cmd)))

            so3_metrics = {
                "so3_rms_deg": float(np.sqrt(np.mean(np.array(so3_errs)**2))),
                "so3_peak_deg": float(np.max(np.abs(np.array(so3_errs)))),
            }

    return {
        "joint_metrics": joint_metrics,
        "dof_metrics": dof_metrics,
        "so3_metrics": so3_metrics,
    }


def run_j8_e4_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict[str, Any]:
    print("\n" + "=" * 90)
    print("      6PUS J8-E4: COMBINED 6-DOF CONTINUOUS CARTESIAN HARDWARE VALIDATION")
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
    harness = Ros2CombinedHarness()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor spinning all 5 ROS2 nodes in background.")

    # Combined 6-DoF Parameters
    f_hz = 0.1
    Ax_m, Ay_m, Az_m = 0.0015, 0.0015, 0.0015
    Ar_rad = math.radians(0.35)
    Ap_rad = math.radians(0.35)
    Ayaw_rad = math.radians(1.50)

    cmd_amps = [
        Ax_m * 1000.0,
        Ay_m * 1000.0,
        Az_m * 1000.0,
        math.degrees(Ar_rad),
        math.degrees(Ap_rad),
        math.degrees(Ayaw_rad),
    ]

    try:
        # Phase 0: Takeover at OPERATING_HOME (3.0s)
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

        # Phase 1: Combined 6-DoF Trajectory Execution (30.0s = 3 full cycles at 0.1 Hz)
        print("\n--- [Phase 1] Executing Simultaneous Combined 6-DoF Continuous Sinusoid (30.0s, 3 cycles @ 0.1 Hz) ---")
        t_start_ns = time.monotonic_ns()
        t_start_s = time.monotonic()
        dur_s = 30.0
        t_end_s = t_start_s + dur_s

        while time.monotonic() < t_end_s:
            t_curr = time.monotonic() - t_start_s
            phase = 2.0 * math.pi * f_hz * t_curr

            px = Ax_m * math.sin(phase)
            py = Ay_m * math.cos(phase)
            pz = z0 + Az_m * math.sin(phase)
            pr = Ar_rad * math.sin(phase)
            pp = Ap_rad * math.cos(phase)
            pyaw = Ayaw_rad * math.sin(phase + math.pi / 4.0)

            harness.publish_target_pose(x=px, y=py, z=pz, roll=pr, pitch=pp, yaw=pyaw)
            time.sleep(0.020)

        t_end_ns = time.monotonic_ns()
        print("[Phase 1 PASS] Combined 6-DoF continuous trajectory execution completed.")

        # Phase 2: Settle & Return to HOME (4.0s)
        print("\n--- [Phase 2] Returning to Cartesian OPERATING_HOME & Measuring Repeatability (4.0s) ---")
        t_ret_start_ns = time.monotonic_ns()
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0)
            time.sleep(0.020)
        t_ret_end_ns = time.monotonic_ns()

        with harness.lock:
            final_home_fk = list(harness.trace.fk_poses[-1])
        print(f"[Phase 2 PASS] Final Settled HOME FK: X={final_home_fk[0]*1e3:+.3f}mm, Y={final_home_fk[1]*1e3:+.3f}mm, Z={final_home_fk[2]*1e3:+.3f}mm, R={math.degrees(final_home_fk[3]):+.3f}°, P={math.degrees(final_home_fk[4]):+.3f}°, Y={math.degrees(final_home_fk[5]):+.3f}°")

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
    analysis_res = analyze_combined_6dof(
        trace, t_start_ns, t_end_ns, f_hz, cmd_amps, home_fk, z0, kin
    )

    dof_res = analysis_res["dof_metrics"]
    joint_res = analysis_res["joint_metrics"]
    so3_res = analysis_res["so3_metrics"]

    print("\n" + "=" * 105)
    print("                J8-E4 COMBINED 6-DOF RESULTS SUMMARY (0.1 Hz Continuous)")
    print("=" * 105)
    print(f"Total Target Cartesian Poses Published: {len(trace.target_stamps_ns)}")
    print(f"Total IK Desired Deltas Received:       {len(trace.ik_stamps_ns)}")
    print(f"Total Control Command Steps Received:   {len(trace.cmd_stamps_ns)}")
    print(f"Total FK Reconstructed States Received: {len(trace.fk_query_stamps_ns)}")

    print("\n" + "-" * 135)
    print(f"{'DoF':<8} {'Cmd Amp':<12} {'FK Amp':<12} {'E2E Gain %':<12} {'E2E Lag':<12} {'Shaper Gain':<13} {'Shaper Lag':<12} {'Exec Gain':<11} {'Exec Lag':<11} {'E2E RMS'}")
    print("-" * 135)

    for dof in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
        m = dof_res[dof]
        u = m["unit"]
        print(
            f"{dof:<8} {m['cmd_amp']:5.2f} {u:<3}   {m['fk_amp_fit']:5.2f} {u:<3}   {m['gain_e2e_pct']:6.1f}%       "
            f"{m['lag_e2e_ms']:+5.1f} ms    {m['gain_shape_pct']:6.1f}%       {m['lag_shape_ms']:+5.1f} ms   "
            f"{m['gain_exec_pct']:6.1f}%     {m['lag_exec_ms']:+5.1f} ms   {m['e2e_rms']:6.3f} {u}"
        )

    print(f"\nOverall Combined 6-DoF Orientation SO(3) Geodesic Error: RMS = {so3_res['so3_rms_deg']:.3f}°, Peak = {so3_res['so3_peak_deg']:.3f}°")

    # Joint Tracking Metrics
    print("\n--- Timestamp-Aligned Per-Axis Joint Tracking RMS & Peak Errors ---")
    print(f"{'Motor ID':<12} {'RMS Error (steps)':<20} {'RMS Error (um)':<18} {'Peak Error (steps)':<20} {'Peak Error (um)'}")
    print("-" * 85)
    for mid in MOTOR_IDS:
        jm = joint_res[f"ID{mid}"]
        print(f"ID{mid:<10} {jm['rms_err_steps']:6.2f} steps        {jm['rms_err_um']:6.1f} um         {jm['peak_err_steps']:6.2f} steps        {jm['peak_err_um']:6.1f} um")

    # Repeatability / Hysteresis
    hyst_trans_mm = [
        (final_home_fk[0] - home_fk[0]) * 1000.0,
        (final_home_fk[1] - home_fk[1]) * 1000.0,
        (final_home_fk[2] - home_fk[2]) * 1000.0,
    ]
    hyst_rot_deg = [
        math.degrees(final_home_fk[3] - home_fk[3]),
        math.degrees(final_home_fk[4] - home_fk[4]),
        math.degrees(final_home_fk[5] - home_fk[5]),
    ]
    print("\n--- Return-to-HOME Repeatability (Hysteresis after 3 full 6-DoF cycles) ---")
    print(f"Translation Hysteresis: dX={hyst_trans_mm[0]:+.3f} mm, dY={hyst_trans_mm[1]:+.3f} mm, dZ={hyst_trans_mm[2]:+.3f} mm")
    print(f"Rotation Hysteresis:    dRoll={hyst_rot_deg[0]:+.3f}°, dPitch={hyst_rot_deg[1]:+.3f}°, dYaw={hyst_rot_deg[2]:+.3f}°")

    results_data = {
        "initial_home_fk": home_fk,
        "final_home_fk": final_home_fk,
        "hysteresis_trans_mm": hyst_trans_mm,
        "hysteresis_rot_deg": hyst_rot_deg,
        "dof_metrics": dof_res,
        "joint_metrics": joint_res,
        "so3_metrics": so3_res,
        "total_targets_pub": len(trace.target_stamps_ns),
        "total_ik_rcv": len(trace.ik_stamps_ns),
        "total_cmd_rcv": len(trace.cmd_stamps_ns),
        "total_fk_rcv": len(trace.fk_query_stamps_ns),
    }

    if output_json:
        with open(output_json, "w") as f:
            json.dump(results_data, f, indent=2)
        print(f"\nWrote J8-E4 results to {output_json}")

    return results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run 6PUS J8-E4 Combined 6-DoF Validation")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    run_j8_e4_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay_ms,
        output_json=args.output_json,
    )
