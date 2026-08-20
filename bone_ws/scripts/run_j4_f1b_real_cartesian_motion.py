#!/usr/bin/env python3
"""Stage J4-F1B — 25 Hz Real-Hardware Continuous Cartesian Motion Runner.

Executes the full ROS 2 Cartesian Pipeline on real hardware:
  Cartesian Reference Stream (50 Hz)
  ──► pus6_kinematics_node (Analytical IK @ 50 Hz)
  ──► pus6_control_node (Stateful 3rd-order reference shaper @ 50 Hz)
  ──► /pus6/command_joint_steps (QoS: KEEP_LAST(1) + BEST_EFFORT)
  ──► Latest-Value-Wins Buffer
  ──► pus6_motor_driver_node (UART Worker @ 25 Hz -> 0x29 x 6)
  ──► Real 6PUS feedback

Phases:
  F1B-0: Static arming & 10-cycle 25 Hz warm-up (captures q_initial, checks cold-start latency)
  F1B-1: Heave-only Continuous Cartesian Motion (Z = z0 +/- 2mm @ 0.2 Hz, 10s duration)

Authoritative References:
  - /home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md
  - /home/lyapunov/bone_drill/AGENTS.md
  - /home/lyapunov/bone_drill/agent_docs/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import (
    MOTOR_IDS,
    NUM_MOTORS,
    DriverError,
    HardwareMode,
    SafetyViolation,
)
from pus6_motor_driver.six_motor_controller import SixAxisStateSnapshot, SixMotorController
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray


@dataclass
class TelemetryRecord:
    timestamp_ns: int
    t_rel_s: float
    # Reference
    ref_pose: list[float]  # [x, y, z, roll, pitch, yaw]
    # Kinematics / Control
    q_ik_deltas_m: list[float]
    q_shaped_deltas_m: list[float]
    q_cmd_raw_steps: list[int]
    # Feedback
    q_actual_raw_steps: list[int]
    currents_mA: list[int]
    forces_g: list[int]
    temperatures_C: list[int]
    fault_codes: list[int]
    # Timing & Metrics
    batch_write_ms: float
    target_age_ms: float


class CartesianTrajectoryGenerator(Node):
    """Generates continuous Cartesian reference trajectory stream at 50 Hz."""

    def __init__(self, rate_hz: float = 50.0) -> None:
        super().__init__("cartesian_trajectory_generator")
        self.rate_hz = rate_hz
        self.pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", 10)
        self.kin = Analytical6PusKinematics()
        self.z0 = self.kin.nominal_height
        self._enabled = False
        self._t_start_ns = 0
        self._seq = 0
        self.last_published_pose: list[float] = [0.0, 0.0, self.z0, 0.0, 0.0, 0.0]

    def start_heave(self) -> None:
        self._t_start_ns = time.perf_counter_ns()
        self._enabled = True

    def stop(self) -> None:
        self._enabled = False

    def publish_step(self, t_rel_s: float, motion_mode: str = "heave") -> list[float]:
        z = self.z0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        x = 0.0
        y = 0.0
        z = self.z0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if motion_mode == "static":
            pass
        elif motion_mode == "heave":
            # Z(t) = z0 + 0.002 * sin(2*pi*0.2*t)
            z = self.z0 + 0.002 * math.sin(2.0 * math.pi * 0.2 * t_rel_s)
        elif motion_mode in ("pitch",):
            # pitch(t) = 0.02 * sin(2*pi*0.2*t) rad
            pitch = 0.02 * math.sin(2.0 * math.pi * 0.2 * t_rel_s)
        elif motion_mode in ("heave_pitch", "compound", "f1c"):
            # Z(t) = z0 + 0.0015 * sin(2*pi*0.2*t) m
            # Pitch(t) = 0.015 * cos(2*pi*0.2*t) rad
            z = self.z0 + 0.0015 * math.sin(2.0 * math.pi * 0.2 * t_rel_s)
            pitch = 0.015 * math.cos(2.0 * math.pi * 0.2 * t_rel_s)
        elif motion_mode in ("f1d", "3dof", "x_z_pitch"):
            # J4-F1D: Non-in-phase 3DoF with 1s Quintic Ramp-in + 15s Full-Amplitude (3 cycles) + 1s Ramp-out + 1s Settling Hold
            T_ramp_in = 1.0
            T_full = 15.0
            T_ramp_out = 1.0
            f0 = 0.2
            Ax = 0.0020
            Az = 0.0015
            Ap = 0.0150

            if t_rel_s < 0.0:
                S = 0.0
            elif t_rel_s < T_ramp_in:
                tau = t_rel_s / T_ramp_in
                S = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            elif t_rel_s <= (T_ramp_in + T_full):
                S = 1.0
            elif t_rel_s < (T_ramp_in + T_full + T_ramp_out):
                tau = (T_ramp_in + T_full + T_ramp_out - t_rel_s) / T_ramp_out
                S = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            else:
                S = 0.0  # Settling hold at HOME

            x = Ax * S * math.sin(2.0 * math.pi * f0 * t_rel_s)
            z = self.z0 + Az * S * math.cos(2.0 * math.pi * f0 * t_rel_s)
            pitch = Ap * S * math.sin(2.0 * math.pi * f0 * t_rel_s + math.pi / 4.0)

        elif motion_mode in ("f1e", "6dof", "full_6dof"):
            # J4-F1E: Full 6DoF Non-In-Phase Continuous Cartesian Motion
            # 1s Quintic Ramp-in + 15s Full-Amplitude (3 cycles @ 0.2Hz) + 1s Ramp-out + 1s HOME Settling Hold
            T_ramp_in = 1.0
            T_full = 15.0
            T_ramp_out = 1.0
            f0 = 0.2

            Ax = 0.0015     # +/- 1.5 mm
            Ay = 0.0015     # +/- 1.5 mm
            Az = 0.0010     # +/- 1.0 mm
            Aroll = 0.010   # +/- 10.0 mrad
            Apitch = 0.010  # +/- 10.0 mrad
            Ayaw = 0.008    # +/- 8.0 mrad

            phi_x = 0.0
            phi_y = math.pi / 3.0
            phi_z = math.pi / 2.0
            phi_roll = 2.0 * math.pi / 3.0
            phi_pitch = math.pi / 4.0
            phi_yaw = 5.0 * math.pi / 6.0

            if t_rel_s < 0.0:
                S = 0.0
            elif t_rel_s < T_ramp_in:
                tau = t_rel_s / T_ramp_in
                S = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            elif t_rel_s <= (T_ramp_in + T_full):
                S = 1.0
            elif t_rel_s < (T_ramp_in + T_full + T_ramp_out):
                tau = (T_ramp_in + T_full + T_ramp_out - t_rel_s) / T_ramp_out
                S = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            else:
                S = 0.0  # Settling hold at HOME

            x = Ax * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_x)
            y = Ay * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_y)
            z = self.z0 + Az * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_z)
            roll = Aroll * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_roll)
            pitch = Apitch * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_pitch)
            yaw = Ayaw * S * math.sin(2.0 * math.pi * f0 * t_rel_s + phi_yaw)

        pose = [x, y, z, roll, pitch, yaw]
        self.last_published_pose = pose

        # Convert roll, pitch, yaw to quaternion (ZYX convention matching kinematics node)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz

        self.pub.publish(msg)
        self._seq += 1
        return pose


class J4F1BRunner:
    """Orchestrates F1B-0 warm-up and F1B-1 Heave-only continuous Cartesian execution."""

    def __init__(
        self,
        *,
        port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        baudrate: int = 921600,
        hardware_mode: HardwareMode = HardwareMode.ACTIVE,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.hardware_mode = hardware_mode
        self.driver_node: Pus6MotorDriverNode | None = None
        self.kinematics_node: Pus6KinematicsNode | None = None
        self.control_node: Pus6ControlNode | None = None
        self.gen_node: CartesianTrajectoryGenerator | None = None
        self.executor: MultiThreadedExecutor | None = None
        self.executor_thread: threading.Thread | None = None

        self.q_initial: list[int] | None = None
        self.telemetry_log: list[TelemetryRecord] = []

    def setup_pipeline(self) -> None:
        """Initialize all official ROS 2 pipeline nodes."""
        self.driver_node = Pus6MotorDriverNode(
            port=self.port,
            baudrate=self.baudrate,
            hardware_mode=self.hardware_mode.value,
            write_rate_hz=25.0,  # 25 Hz UART Worker
            poll_rate_hz=0.0,    # Disable poll timer during active streaming
        )

        if self.hardware_mode == HardwareMode.MOCK:
            from pus6_motor_driver.fake_bus import FakeBus
            if isinstance(self.driver_node.controller.bus, FakeBus):
                self.driver_node.controller.bus.positions = {i: 1000 for i in MOTOR_IDS}
                self.driver_node.controller.bus.target_positions = {i: 1000 for i in MOTOR_IDS}

        self.kinematics_node = Pus6KinematicsNode()
        self.control_node = Pus6ControlNode()
        self.gen_node = CartesianTrajectoryGenerator(rate_hz=50.0)

        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.driver_node)
        self.executor.add_node(self.kinematics_node)
        self.executor.add_node(self.control_node)
        self.executor.add_node(self.gen_node)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(0.50)  # Allow DDS discovery

    def teardown_pipeline(self) -> None:
        if self.executor is not None:
            self.executor.shutdown()
        if self.driver_node is not None:
            self.driver_node.destroy_node()
        if self.kinematics_node is not None:
            self.kinematics_node.destroy_node()
        if self.control_node is not None:
            self.control_node.destroy_node()
        if self.gen_node is not None:
            self.gen_node.destroy_node()

    def run_f1b_0_warmup(self) -> dict[str, Any]:
        """F1B-0: Static Arming and 10-Cycle 25 Hz Warm-up."""
        print("\n================================================================================")
        print(" [F1B-0] Static Arming & 10-Cycle Warm-Up (25 Hz)")
        print("================================================================================")

        assert self.driver_node is not None
        controller = self.driver_node.controller

        # 1. Read-only snapshot (0 write)
        t0_snap = time.perf_counter_ns()
        snapshot = controller.read_state_snapshot()
        t1_snap = time.perf_counter_ns()
        snap_ms = (t1_snap - t0_snap) / 1e6

        self.q_initial = list(snapshot.positions_steps)
        print(f"  Startup Snapshot Duration: {snap_ms:.2f} ms")
        print(f"  Captured q_initial:         {self.q_initial} steps")
        print(f"  Raw Currents:               {snapshot.currents_mA} mA")
        print(f"  Fault Codes:                {snapshot.fault_codes}")

        all_faults_zero = all(f == 0 for f in snapshot.fault_codes)
        in_home_neighborhood = all(900 <= s <= 1100 for s in self.q_initial)
        if not (all_faults_zero and in_home_neighborhood):
            raise RuntimeError(f"F1B-0 pre-check failed: faults={snapshot.fault_codes}, pos={self.q_initial}")

        # Set driver last state snapshot so safety preflight is satisfied
        self.driver_node._last_state_snapshot = snapshot

        # 2. Execute 10 static warm-up cycles directly over UART worker
        warmup_durations_ms: list[float] = []
        for i in range(10):
            t_w0 = time.perf_counter_ns()
            res = controller.command_positions_with_metrics(self.q_initial)
            t_w1 = time.perf_counter_ns()
            w_dur = (t_w1 - t_w0) / 1e6
            warmup_durations_ms.append(w_dur)
            time.sleep(0.040)

        first_write_latency_ms = warmup_durations_ms[0]
        warmup_mean_ms = float(np.mean(warmup_durations_ms))
        warmup_max_ms = float(np.max(warmup_durations_ms))

        print(f"  Warm-Up Writes:             10 cycles executed")
        print(f"  First-Write Latency:        {first_write_latency_ms:.2f} ms")
        print(f"  Warm-Up Duration Mean:      {warmup_mean_ms:.2f} ms (Max: {warmup_max_ms:.2f} ms)")

        pass_warmup = bool(all_faults_zero and in_home_neighborhood and warmup_max_ms < 60.0)
        print(f"  Phase Status:               {'PASS' if pass_warmup else 'FAIL'}")

        return {
            "phase": "F1B-0",
            "snapshot_duration_ms": snap_ms,
            "q_initial": self.q_initial,
            "currents_mA": list(snapshot.currents_mA),
            "fault_codes": list(snapshot.fault_codes),
            "warmup_durations_ms": warmup_durations_ms,
            "first_write_latency_ms": first_write_latency_ms,
            "warmup_mean_ms": warmup_mean_ms,
            "warmup_max_ms": warmup_max_ms,
            "pass": pass_warmup,
        }

    def run_f1b_motion(self, motion_mode: str = "heave", duration_s: float = 5.0) -> dict[str, Any]:
        """Continuous Cartesian Motion through full ROS 2 pipeline with segmented timing."""
        mode_label = "Static Hold" if motion_mode == "static" else "Heave-only (Z = z0 +/- 2mm @ 0.2Hz)"
        print("\n================================================================================")
        print(f" [{motion_mode.upper()}] Continuous Cartesian Motion ({mode_label}, {duration_s}s)")
        print("================================================================================")

        assert self.gen_node is not None
        assert self.driver_node is not None
        assert self.control_node is not None
        assert self.kinematics_node is not None

        dt_50hz = 0.020  # 50 Hz nominal interval
        total_steps = int(duration_s / dt_50hz)
        t_start_ns = time.perf_counter_ns()
        next_wake_ns = t_start_ns

        self.telemetry_log.clear()
        stop_triggered = False
        stop_reason = ""

        # Pre-initialize control shaper at initial position
        self.control_node._current_positions = [0.0] * 6
        self.control_node.controller.reset(initial_positions=[0.0] * 6)

        writes_before = self.driver_node.controller.write_count
        consumed_before = self.driver_node._target_consumed_count
        overwrite_before = self.driver_node._target_overwrite_count
        in_count_before = self.driver_node._target_in_count
        miss_before = self.driver_node._deadline_miss_count

        for step_idx in range(total_steps):
            t_now_ns = time.perf_counter_ns()
            t_rel_s = (t_now_ns - t_start_ns) / 1e9
            if t_rel_s > duration_s:
                break

            # 1. Generate & Publish Cartesian Reference (50 Hz rate regulated)
            ref_pose = self.gen_node.publish_step(t_rel_s, motion_mode=motion_mode)

            # 2. Rate-regulated sleep to maintain precise 50.0 Hz reference generation
            next_wake_ns += 20_000_000
            sleep_ns = next_wake_ns - time.perf_counter_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

            # 3. Capture synchronized telemetry snapshot
            snap = self.driver_node._last_state_snapshot
            if snap is not None:
                # Real-time safety monitoring
                for i, f in enumerate(snap.fault_codes):
                    if f != 0:
                        stop_triggered = True
                        stop_reason = f"Hardware Fault: Motor {i+1} returned fault_code=0x{f:02X}"
                        break
                for i, c in enumerate(snap.currents_mA):
                    if abs(c) > 400:
                        stop_triggered = True
                        stop_reason = f"Current Overload STOP: Motor {i+1} current {abs(c)} mA > 400 mA"
                        break
                for i, s in enumerate(snap.positions_steps):
                    if not (200 <= s <= 1800):
                        stop_triggered = True
                        stop_reason = f"Envelope Violation: Motor {i+1} pos {s} outside [200, 1800]"
                        break

                if stop_triggered:
                    print(f"  [STOP TRIGGERED] {stop_reason}")
                    break

                # Extract latest pipeline intermediate states
                latest_cmd_steps = list(self.driver_node._latest_target.targets_steps) if self.driver_node._latest_target else [1000]*6
                rec = TelemetryRecord(
                    timestamp_ns=time.perf_counter_ns(),
                    t_rel_s=t_rel_s,
                    ref_pose=list(ref_pose),
                    q_ik_deltas_m=list(self.control_node._target_positions or [0.0]*6),
                    q_shaped_deltas_m=list(self.control_node.controller.shaper.q),
                    q_cmd_raw_steps=latest_cmd_steps,
                    q_actual_raw_steps=list(snap.positions_steps),
                    currents_mA=list(snap.currents_mA),
                    forces_g=list(snap.forces_g_raw),
                    temperatures_C=list(snap.temperatures_C),
                    fault_codes=list(snap.fault_codes),
                    batch_write_ms=self.driver_node._six_axis_uart_transaction_ms,
                    target_age_ms=self.driver_node._last_target_age_ms,
                )
                self.telemetry_log.append(rec)

        t_end_ns = time.perf_counter_ns()
        actual_dur_s = (t_end_ns - t_start_ns) / 1e9
        actual_writes = self.driver_node.controller.write_count - writes_before
        actual_consumed = self.driver_node._target_consumed_count - consumed_before
        actual_overwrite = self.driver_node._target_overwrite_count - overwrite_before
        actual_in = self.driver_node._target_in_count - in_count_before
        actual_misses = self.driver_node._deadline_miss_count - miss_before

        # Performance evaluation
        print("\n  ============================================================")
        print("  F1B-1 EXECUTION & TELEMETRY ANALYSIS")
        print("  ============================================================")
        print(f"  Duration:                   {actual_dur_s:.2f} s ({len(self.telemetry_log)} telemetry frames logged)")
        print(f"  UART 0x29 Writes:           {actual_writes} cycles dispatched")
        print(f"  Driver Consumed Targets:    {self.driver_node._target_consumed_count}")
        print(f"  Driver Overwrite Count:     {self.driver_node._target_overwrite_count} (50Hz prod -> 25Hz cons normal ratio ~50%)")
        print(f"  Driver Deadline Misses:     {self.driver_node._deadline_miss_count}")

        # Compute Joint Tracking Errors: e_i = q_cmd - q_actual
        cmd_mat = np.array([r.q_cmd_raw_steps for r in self.telemetry_log])
        act_mat = np.array([r.q_actual_raw_steps for r in self.telemetry_log])
        curr_mat = np.array([r.currents_mA for r in self.telemetry_log])
        err_mat = cmd_mat - act_mat  # error in steps

        mae_steps = np.mean(np.abs(err_mat), axis=0)
        rms_steps = np.sqrt(np.mean(err_mat**2, axis=0))
        p95_steps = np.percentile(np.abs(err_mat), 95, axis=0)
        max_steps = np.max(np.abs(err_mat), axis=0)

        q_cmd_min = np.min(cmd_mat, axis=0)
        q_cmd_max = np.max(cmd_mat, axis=0)
        q_act_min = np.min(act_mat, axis=0)
        q_act_max = np.max(act_mat, axis=0)

        print("\n  Joint Space Execution Performance (e = q_cmd - q_actual):")
        print(f"  Axis   MAE (step)   RMS (step)   P95 (step)   Max (step)   q_cmd range    q_act range")
        print(f"  -------------------------------------------------------------------------------------")
        for i in range(6):
            print(
                f"  M{i+1}:   {mae_steps[i]:6.2f}       {rms_steps[i]:6.2f}       {p95_steps[i]:6.2f}       "
                f"{max_steps[i]:6.0f}       [{q_cmd_min[i]:4d}, {q_cmd_max[i]:4d}]   [{q_act_min[i]:4d}, {q_act_max[i]:4d}]"
            )

        peak_curr_mA = int(np.max(np.abs(curr_mat)))
        print(f"\n  Peak Dynamic Current:       {peak_curr_mA} mA (Temporary monitoring limit <= 400 mA)")

        # Dynamic Fourier & Cross-Correlation Frequency Analysis (f0 = 0.2 Hz)
        f0 = 0.2
        t_vec = np.array([r.t_rel_s for r in self.telemetry_log])
        fourier_kernel = np.exp(-1j * 2.0 * np.pi * f0 * t_vec)
        N_pts = len(t_vec)

        amp_cmd = []
        amp_act = []
        amp_ratios = []
        phase_lags_rad = []
        phase_delays_ms = []
        xcorr_lags_ms = []

        forces_mat = np.array([r.forces_g for r in self.telemetry_log]) if self.telemetry_log else np.zeros((1, 6))
        force_min = [int(np.min(forces_mat[:, i])) for i in range(6)]
        force_max = [int(np.max(forces_mat[:, i])) for i in range(6)]

        for i in range(6):
            c_zero = cmd_mat[:, i] - np.mean(cmd_mat[:, i])
            a_zero = act_mat[:, i] - np.mean(act_mat[:, i])

            # Fourier transform at fundamental frequency f0 = 0.2 Hz
            F_cmd = np.sum(c_zero * fourier_kernel) * (2.0 / max(1, N_pts))
            F_act = np.sum(a_zero * fourier_kernel) * (2.0 / max(1, N_pts))

            A_c = float(np.abs(F_cmd))
            A_a = float(np.abs(F_act))
            phi_c = float(np.angle(F_cmd))
            phi_a = float(np.angle(F_act))

            # Phase lag = phi_cmd - phi_act (act lags cmd -> phase difference > 0)
            dphi = (phi_c - phi_a) % (2.0 * np.pi)
            if dphi > np.pi:
                dphi -= 2.0 * np.pi
            tau_phase_ms = (dphi / (2.0 * np.pi * f0)) * 1e3

            # Cross-correlation lag
            xcorr = np.correlate(c_zero, a_zero, mode="full")
            xcorr_idx = int(np.argmax(xcorr)) - (N_pts - 1)
            dt_est_ms = (t_vec[-1] - t_vec[0]) / max(1, N_pts - 1) * 1e3
            tau_xcorr_ms = float(xcorr_idx * dt_est_ms)

            amp_cmd.append(A_c)
            amp_act.append(A_a)
            amp_ratios.append(A_a / max(1e-6, A_c))
            phase_lags_rad.append(dphi)
            phase_delays_ms.append(tau_phase_ms)
            xcorr_lags_ms.append(tau_xcorr_ms)

        print("\n  Joint Space Dynamic Execution & Phase Frequency Analysis (f0 = 0.2 Hz):")
        print(f"  Axis   Amp_cmd (step)  Amp_act (step)  Amp_Ratio   Phase Lag (rad)  Phase Delay (ms)  XCorr Delay (ms)")
        print(f"  ------------------------------------------------------------------------------------------------------")
        for i in range(6):
            print(
                f"  M{i+1}:     {amp_cmd[i]:6.1f}          {amp_act[i]:6.1f}        {amp_ratios[i]:6.3f}        "
                f"{phase_lags_rad[i]:6.3f} rad        {phase_delays_ms[i]:6.1f} ms        {xcorr_lags_ms[i]:6.1f} ms"
            )

        print(f"\n  Six-Axis Contact Force Min/Max: {list(zip(force_min, force_max))} g")
        peak_curr_mA = int(np.max(np.abs(curr_mat)))
        print(f"  Peak Dynamic Current:           {peak_curr_mA} mA (Temporary monitoring limit <= 400 mA)")

        # Inter-Axis Execution Skew (difference in tracking error across 6 axes at each time frame)
        skews_steps = np.max(err_mat, axis=1) - np.min(err_mat, axis=1)
        skew_mae = float(np.mean(skews_steps))
        skew_rms = float(np.sqrt(np.mean(skews_steps**2)))
        skew_p95 = float(np.percentile(skews_steps, 95))
        skew_max = float(np.max(skews_steps))

        print(f"\n  Six-Axis Inter-Axis Execution Skew: MAE={skew_mae:.2f}, P95={skew_p95:.2f}, Max={skew_max:.2f} steps")

        pass_f1b_1 = bool(
            not stop_triggered
            and self.driver_node._consecutive_comm_failures == 0
            and peak_curr_mA <= 400
            and all(r == 0 for r in self.telemetry_log[-1].fault_codes)
        )
        print(f"  Phase Status:                   {'PASS' if pass_f1b_1 else 'FAIL'}")

        # Dump full telemetry frames
        raw_telemetry_dump = [
            {
                "timestamp_ns": r.timestamp_ns,
                "t_rel_s": r.t_rel_s,
                "ref_pose": r.ref_pose,
                "q_ik_deltas_m": r.q_ik_deltas_m,
                "q_shaped_deltas_m": r.q_shaped_deltas_m,
                "q_cmd_raw_steps": r.q_cmd_raw_steps,
                "q_actual_raw_steps": r.q_actual_raw_steps,
                "currents_mA": r.currents_mA,
                "forces_g": r.forces_g,
                "temperatures_C": r.temperatures_C,
                "fault_codes": r.fault_codes,
                "batch_write_ms": r.batch_write_ms,
                "target_age_ms": r.target_age_ms,
            }
            for r in self.telemetry_log
        ]

        batch_writes_arr = np.array([r.batch_write_ms for r in self.telemetry_log if r.batch_write_ms > 0])
        target_ages_arr = np.array([r.target_age_ms for r in self.telemetry_log if r.target_age_ms > 0])

        initial_residual = 1 if (in_count_before > 0 and self.driver_node._latest_target and not self.driver_node._latest_target.consumed) else 0
        final_residual = 1 if (self.driver_node._latest_target and not self.driver_node._latest_target.consumed) else 0

        return {
            "phase": "F1C" if motion_mode in ("heave_pitch", "f1c") else ("F1B-2" if motion_mode == "pitch" else "F1B-1"),
            "duration_s": actual_dur_s,
            "telemetry_frames_count": len(self.telemetry_log),
            "actual_hardware_writes": actual_writes,
            "cumulative_counters": {
                "driver_target_in_count": self.driver_node._target_in_count,
                "driver_consumed_count": self.driver_node._target_consumed_count,
                "driver_overwrite_count": self.driver_node._target_overwrite_count,
                "deadline_miss_count": self.driver_node._deadline_miss_count,
            },
            "segment_delta_counters": {
                "received_delta": actual_in,
                "consumed_delta": actual_consumed,
                "overwrite_delta": actual_overwrite,
                "initial_residual": initial_residual,
                "final_residual": final_residual,
                "identity_valid": bool((initial_residual + actual_in) == (actual_consumed + actual_overwrite + final_residual)),
            },
            "batch_write_ms": {
                "mean": float(np.mean(batch_writes_arr)) if len(batch_writes_arr) else 0.0,
                "p50": float(np.percentile(batch_writes_arr, 50)) if len(batch_writes_arr) else 0.0,
                "p95": float(np.percentile(batch_writes_arr, 95)) if len(batch_writes_arr) else 0.0,
                "p99": float(np.percentile(batch_writes_arr, 99)) if len(batch_writes_arr) else 0.0,
                "max": float(np.max(batch_writes_arr)) if len(batch_writes_arr) else 0.0,
            },
            "target_age_ms": {
                "mean": float(np.mean(target_ages_arr)) if len(target_ages_arr) else 0.0,
                "p95": float(np.percentile(target_ages_arr, 95)) if len(target_ages_arr) else 0.0,
                "max": float(np.max(target_ages_arr)) if len(target_ages_arr) else 0.0,
            },
            "joint_errors_steps": {
                "mae": [float(v) for v in mae_steps],
                "rms": [float(v) for v in rms_steps],
                "p95": [float(v) for v in p95_steps],
                "max": [float(v) for v in max_steps],
            },
            "inter_axis_execution_skew_steps": {
                "mae": skew_mae,
                "rms": skew_rms,
                "p95": skew_p95,
                "max": skew_max,
            },
            "frequency_analysis_0_2Hz": {
                "amp_cmd_steps": [float(v) for v in amp_cmd],
                "amp_act_steps": [float(v) for v in amp_act],
                "amp_ratio": [float(v) for v in amp_ratios],
                "phase_lag_rad": [float(v) for v in phase_lags_rad],
                "phase_delay_ms": [float(v) for v in phase_delays_ms],
                "xcorr_delay_ms": [float(v) for v in xcorr_lags_ms],
            },
            "segmented_timing_ms": {
                "fetch_ms": self.driver_node._target_fetch_ms,
                "preflight_ms": self.driver_node._final_preflight_ms,
                "pure_uart_transaction_ms": self.driver_node._six_axis_uart_transaction_ms,
                "per_motor_ms": dict(self.driver_node._per_motor_ms),
                "feedback_parse_ms": self.driver_node._feedback_parse_ms,
                "ros_publish_ms": self.driver_node._ros_publish_ms,
                "worker_total_ms": self.driver_node._worker_total_ms,
                "worker_start_to_start_ms": self.driver_node._worker_start_to_start_ms,
            },
            "contention_and_polling": {
                "background_poll_count": self.driver_node._background_poll_count,
                "background_poll_during_active_count": self.driver_node._background_poll_during_active_count,
                "uart_lock_contention_count": self.driver_node._uart_lock_contention_count,
            },
            "timeout_events": list(self.driver_node._timeout_events),
            "effective_worker_rate_hz": float(actual_consumed / max(0.001, actual_dur_s)),
            "forces_g_min": force_min,
            "forces_g_max": force_max,
            "q_cmd_min_steps": [int(v) for v in q_cmd_min],
            "q_cmd_max_steps": [int(v) for v in q_cmd_max],
            "q_act_min_steps": [int(v) for v in q_act_min],
            "q_act_max_steps": [int(v) for v in q_act_max],
            "peak_current_mA": peak_curr_mA,
            "stop_triggered": stop_triggered,
            "stop_reason": stop_reason,
            "pass": pass_f1b_1,
            "telemetry_frames": raw_telemetry_dump,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description="J4 Continuous Cartesian Motion Runner")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--motion", choices=["heave", "pitch", "heave_pitch", "f1c", "f1d", "3dof", "x_z_pitch", "f1e", "6dof", "full_6dof", "static"], default="heave", help="Cartesian trajectory type")
    parser.add_argument("--run-ab-experiment", action="store_true", help="Run A (3s static) and B (5s moving) comparison")
    args = parser.parse_args()

    if args.real_hardware:
        if not (args.confirm_motion and args.validated_config):
            print("[ERROR] Real hardware execution requires --confirm-motion and --validated-config")
            sys.exit(1)
        mode = HardwareMode.ACTIVE
    else:
        mode = HardwareMode.MOCK

    if args.motion in ("f1e", "6dof", "full_6dof"):
        stage_name = "J4-F1E"
    elif args.motion in ("f1d", "3dof", "x_z_pitch"):
        stage_name = "J4-F1D"
    elif args.motion in ("heave_pitch", "f1c"):
        stage_name = "J4-F1C"
    elif args.motion == "pitch":
        stage_name = "J4-F1B-2"
    else:
        stage_name = "J4-F1B-1"

    print("================================================================================")
    print(f" 6PUS {stage_name}: 25 Hz Real-Hardware Continuous Cartesian Motion Runner ({args.motion.upper()})")
    print("================================================================================")
    print(f"Mode:         {mode.value.upper()}")
    print(f"Motion:       {args.motion.upper()}")
    print(f"Port:         {args.port}")
    print(f"Baudrate:     {args.baudrate}")
    print(f"Duration:     {args.duration:.1f} s")
    print(f"A/B Compare:  {args.run_ab_experiment}")

    rclpy.init()
    runner = J4F1BRunner(
        port=args.port,
        baudrate=args.baudrate,
        hardware_mode=mode,
    )

    results: dict[str, Any] = {
        "timestamp": datetime.now().isoformat(),
        "stage": stage_name,
        "motion_mode": args.motion,
        "hardware_mode": mode.value,
    }

    try:
        runner.setup_pipeline()

        # Step 0: Warm-up
        res0 = runner.run_f1b_0_warmup()
        results["F1B-0"] = res0
        if not res0["pass"]:
            print("\n[STOP] F1B-0 warm-up failed. Aborting dynamic motion.")
            sys.exit(1)

        if args.run_ab_experiment:
            # Step A: 3.0s Static Streaming
            res_a = runner.run_f1b_motion(motion_mode="static", duration_s=3.0)
            results["PART-A-STATIC"] = res_a

            # Step B: 5.0s Heave Continuous Motion
            res_b = runner.run_f1b_motion(motion_mode="heave", duration_s=5.0)
            results["PART-B-HEAVE"] = res_b
        else:
            res1 = runner.run_f1b_motion(motion_mode=args.motion, duration_s=args.duration)
            if args.motion in ("f1e", "6dof", "full_6dof"):
                phase_key = "F1E"
            elif args.motion in ("f1d", "3dof", "x_z_pitch"):
                phase_key = "F1D"
            elif args.motion in ("heave_pitch", "f1c"):
                phase_key = "F1C"
            elif args.motion == "pitch":
                phase_key = "F1B-2"
            else:
                phase_key = "F1B-1"
            results[phase_key] = res1

    finally:
        runner.teardown_pipeline()
        rclpy.shutdown()

    # Save summary
    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.motion in ("f1e", "6dof", "full_6dof"):
        motion_tag = "6dof"
        stage_prefix = "j4_f1e"
    elif args.motion in ("f1d", "3dof", "x_z_pitch"):
        motion_tag = "3dof_x_z_pitch"
        stage_prefix = "j4_f1d"
    elif args.motion in ("heave_pitch", "f1c"):
        motion_tag = "heave_pitch"
        stage_prefix = "j4_f1c"
    elif args.motion == "pitch":
        motion_tag = "pitch"
        stage_prefix = "j4_f1b"
    elif args.run_ab_experiment:
        motion_tag = "ab_compare"
        stage_prefix = "j4_f1b"
    else:
        motion_tag = "heave"
        stage_prefix = "j4_f1b"

    log_file = log_dir / f"{stage_prefix}_real_cartesian_{motion_tag}_{stamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"\n[SUMMARY] Saved full test results to: {log_file}")


if __name__ == "__main__":
    main()
