#!/usr/bin/env python3
"""J9.1 Open-Loop Attitude Command & Platform H30 Measurement Runner.

Executes small-angle single-DoF open-loop rotations (+/- 0.5 deg) strictly across
the J8 frozen baseline ROS 2 control chain:
  /pus6/target_cartesian_pose -> Pus6KinematicsNode
  -> /pus6/desired_joint_deltas -> Pus6ControlNode
  -> /pus6/command_joint_steps -> Pus6MotorDriverNode
  -> 50Hz 0x29 Active Servo -> 6PUS Manipulator
  -> /pus6/joint_feedback/timed -> Pus6FkNode

Simultaneously subscribes to Platform H30 IMU (/pus6/imu_platform/data),
establishes HOME relative-zero (q0), and logs dense time-series of commanded
attitude, raw & relative IMU orientation, gyro, and joint command/feedback.
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

# Path resolution for workspace packages
SCRIPT_PATH = Path(__file__).resolve()
WS_SRC = SCRIPT_PATH.parents[2]
WS_SCRIPTS = SCRIPT_PATH.parents[3] / "scripts"
if WS_SCRIPTS.exists() and str(WS_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(WS_SCRIPTS))

for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"]:
    p = WS_SRC / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

scripts_dir = WS_SRC / "h30_imu" / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

import serial
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import MOTOR_IDS
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback

from j9_1_platform_imu_recorder import (
    average_quat_xyzw,
    normalize_quat_xyzw,
    quat_conjugate_xyzw,
    quat_multiply_xyzw,
    quat_to_rpy_deg_xyzw,
    relative_quat_xyzw,
)

CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "ros_stamp_sec",
    "ros_stamp_nanosec",
    "stage",
    "command_roll_deg",
    "command_pitch_deg",
    "command_yaw_deg",
    "qx",
    "qy",
    "qz",
    "qw",
    "relative_roll_deg",
    "relative_pitch_deg",
    "relative_yaw_deg",
    "platform_roll_deg",
    "platform_pitch_deg",
    "platform_yaw_deg",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
    "joint_cmd_1",
    "joint_cmd_2",
    "joint_cmd_3",
    "joint_cmd_4",
    "joint_cmd_5",
    "joint_cmd_6",
    "joint_cmd_stamp_ns",
    "joint_fb_1",
    "joint_fb_2",
    "joint_fb_3",
    "joint_fb_4",
    "joint_fb_5",
    "joint_fb_6",
    "joint_fb_stamp_ns",
    "hardware_write_count",
    "fault_codes",
]


def check_motor_read_only_preflight(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    duration_s: float = 0.2,
) -> Tuple[bool, str]:
    """Performs a non-intrusive 0x30 status scan on all 6 motors before enabling driver."""
    from run_j8_c_hardware_readonly_audit import run_j8_c_audit

    try:
        res = run_j8_c_audit(port=port, baudrate=baudrate, duration_s=duration_s)
        if res.get("status") == "PASS":
            return True, "All 6 motors online, fault=0, within HOME band"
        return False, f"Audit returned status: {res.get('status')}"
    except Exception as exc:
        return False, f"Preflight audit exception: {exc}"


class J91ExperimentHarness(Node):
    """Harness node managing target publication, IMU subscription, and synchronized data logging."""

    def __init__(
        self,
        *,
        imu_topic: str,
        csv_path: Path,
        init_quiet_s: float = 1.5,
        min_init_samples: int = 20,
    ) -> None:
        super().__init__("j9_1_experiment_harness")
        self.imu_topic = imu_topic
        self.csv_path = csv_path
        self.init_quiet_s = max(0.5, float(init_quiet_s))
        self.min_init_samples = max(10, int(min_init_samples))

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self.lock = threading.Lock()

        # State variables
        self.current_stage: str = "pre_init"
        self.cmd_roll_deg: float = 0.0
        self.cmd_pitch_deg: float = 0.0
        self.cmd_yaw_deg: float = 0.0

        self.latest_cmd_steps: List[int] = [1000] * 6
        self.latest_cmd_stamp_ns: int = 0

        self.latest_fb_steps: List[int] = [1000] * 6
        self.latest_fb_currents: List[int] = [0] * 6
        self.latest_fb_faults: List[int] = [0] * 6
        self.latest_fb_stamp_ns: int = 0
        self.hardware_write_count: int = 0

        self._first_imu_host_ns: int | None = None
        self._init_quats: List[Tuple[float, float, float, float]] = []
        self.q0_xyzw: Tuple[float, float, float, float] | None = None

        self.recorded_sample_count: int = 0
        self.total_imu_samples: int = 0
        self.samples_by_stage: Dict[str, List[Dict[str, Any]]] = {}

        # ROS 2 QoS setups
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publisher for Cartesian Target
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", reliable_qos
        )

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self.cmd_steps_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, sensor_qos
        )
        self.feedback_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_feedback, sensor_qos
        )

    def close(self) -> None:
        with self.lock:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()

    def set_stage(self, stage: str, cmd_r_deg: float, cmd_p_deg: float, cmd_y_deg: float) -> None:
        with self.lock:
            self.current_stage = stage
            self.cmd_roll_deg = float(cmd_r_deg)
            self.cmd_pitch_deg = float(cmd_p_deg)
            self.cmd_yaw_deg = float(cmd_y_deg)
            if stage not in self.samples_by_stage:
                self.samples_by_stage[stage] = []

    def publish_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
        yaw_rad: float = 0.0,
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)

        self.target_pose_pub.publish(msg)

    def _on_cmd_steps(self, msg: Int32MultiArray) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            if len(msg.data) >= 6:
                self.latest_cmd_steps = list(msg.data[:6])
                self.latest_cmd_stamp_ns = t_ns

    def _on_timed_feedback(self, msg: TimedJointFeedback) -> None:
        t_ns = time.monotonic_ns()
        with self.lock:
            if len(msg.raw_steps) >= 6:
                self.latest_fb_steps = [int(s) for s in msg.raw_steps[:6]]
                self.latest_fb_currents = [int(c) for c in msg.currents_ma[:6]]
                self.latest_fb_faults = [int(f) for f in msg.fault_codes[:6]]
                self.latest_fb_stamp_ns = t_ns
                self.hardware_write_count = int(getattr(msg, "uart_cycle_sequence", 0))

    def _on_imu(self, msg: Imu) -> None:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()

        with self.lock:
            if self._csv_file.closed:
                return
            self.total_imu_samples += 1
            if self._first_imu_host_ns is None:
                self._first_imu_host_ns = host_ns

            q = normalize_quat_xyzw(
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            )
            if q is None or msg.orientation_covariance[0] == -1.0:
                return

            elapsed_init_s = (host_ns - self._first_imu_host_ns) / 1e9
            if self.q0_xyzw is None:
                self._init_quats.append(q)
                if elapsed_init_s >= self.init_quiet_s and len(self._init_quats) >= self.min_init_samples:
                    self.q0_xyzw = average_quat_xyzw(self._init_quats)
                    self.get_logger().info(
                        f"HOME relative-zero q0 initialized from {len(self._init_quats)} samples: {self.q0_xyzw}"
                    )
                return

            # Compute HOME relative orientation
            q_rel = relative_quat_xyzw(self.q0_xyzw, q)
            if q_rel is None:
                return
            rel_roll, rel_pitch, rel_yaw = quat_to_rpy_deg_xyzw(q_rel)

            # Compute canonical Rz(+90 deg) Platform-frame orientation
            # q_PI = [0, 0, sin(pi/4), cos(pi/4)]
            q_PI = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
            q_p_rel = quat_multiply_xyzw(quat_multiply_xyzw(q_PI, q_rel), quat_conjugate_xyzw(q_PI))
            plat_roll, plat_pitch, plat_yaw = quat_to_rpy_deg_xyzw(q_p_rel)

            gx = float(msg.angular_velocity.x)
            gy = float(msg.angular_velocity.y)
            gz = float(msg.angular_velocity.z)

            row_data = {
                "host_log_time_ns": host_ns,
                "host_monotonic_time_ns": mono_ns,
                "ros_stamp_sec": int(msg.header.stamp.sec),
                "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
                "stage": self.current_stage,
                "command_roll_deg": self.cmd_roll_deg,
                "command_pitch_deg": self.cmd_pitch_deg,
                "command_yaw_deg": self.cmd_yaw_deg,
                "qx": q[0],
                "qy": q[1],
                "qz": q[2],
                "qw": q[3],
                "relative_roll_deg": rel_roll,
                "relative_pitch_deg": rel_pitch,
                "relative_yaw_deg": rel_yaw,
                "platform_roll_deg": plat_roll,
                "platform_pitch_deg": plat_pitch,
                "platform_yaw_deg": plat_yaw,
                "gx_rad_s": gx,
                "gy_rad_s": gy,
                "gz_rad_s": gz,
                "joint_cmd_1": self.latest_cmd_steps[0],
                "joint_cmd_2": self.latest_cmd_steps[1],
                "joint_cmd_3": self.latest_cmd_steps[2],
                "joint_cmd_4": self.latest_cmd_steps[3],
                "joint_cmd_5": self.latest_cmd_steps[4],
                "joint_cmd_6": self.latest_cmd_steps[5],
                "joint_cmd_stamp_ns": self.latest_cmd_stamp_ns,
                "joint_fb_1": self.latest_fb_steps[0],
                "joint_fb_2": self.latest_fb_steps[1],
                "joint_fb_3": self.latest_fb_steps[2],
                "joint_fb_4": self.latest_fb_steps[3],
                "joint_fb_5": self.latest_fb_steps[4],
                "joint_fb_6": self.latest_fb_steps[5],
                "joint_fb_stamp_ns": self.latest_fb_stamp_ns,
                "hardware_write_count": self.hardware_write_count,
                "fault_codes": ":".join(str(c) for c in self.latest_fb_faults),
            }

            self._writer.writerow(row_data)
            self.recorded_sample_count += 1
            if self.current_stage in self.samples_by_stage:
                self.samples_by_stage[self.current_stage].append(row_data)


def compute_stage_settled_metrics(
    samples: List[Dict[str, Any]],
    settle_skip_ratio: float = 0.5,
) -> Dict[str, Any]:
    """Computes settled steady-state metrics over the latter half of a motion stage."""
    if not samples:
        return {
            "count": 0,
            "mean_r": None, "mean_p": None, "mean_y": None,
            "min_r": None, "max_r": None,
            "min_p": None, "max_p": None,
            "min_y": None, "max_y": None,
        }

    start_idx = int(len(samples) * settle_skip_ratio)
    settled = samples[start_idx:] if start_idx < len(samples) else samples

    r_vals = [s["relative_roll_deg"] for s in settled]
    p_vals = [s["relative_pitch_deg"] for s in settled]
    y_vals = [s["relative_yaw_deg"] for s in settled]
    plat_r_vals = [s.get("platform_roll_deg", 0.0) for s in settled]
    plat_p_vals = [s.get("platform_pitch_deg", 0.0) for s in settled]
    plat_y_vals = [s.get("platform_yaw_deg", 0.0) for s in settled]

    return {
        "count": len(settled),
        "total_stage_count": len(samples),
        "mean_r": statistics.fmean(r_vals) if r_vals else 0.0,
        "min_r": min(r_vals) if r_vals else 0.0,
        "max_r": max(r_vals) if r_vals else 0.0,
        "mean_p": statistics.fmean(p_vals) if p_vals else 0.0,
        "min_p": min(p_vals) if p_vals else 0.0,
        "max_p": max(p_vals) if p_vals else 0.0,
        "mean_y": statistics.fmean(y_vals) if y_vals else 0.0,
        "min_y": min(y_vals) if y_vals else 0.0,
        "max_y": max(y_vals) if y_vals else 0.0,
        "mean_plat_r": statistics.fmean(plat_r_vals) if plat_r_vals else 0.0,
        "mean_plat_p": statistics.fmean(plat_p_vals) if plat_p_vals else 0.0,
        "mean_plat_y": statistics.fmean(plat_y_vals) if plat_y_vals else 0.0,
    }


def run_attitude_experiment(
    *,
    dofs: Sequence[str] = ("roll", "pitch", "yaw"),
    angle_deg: float = 0.5,
    hold_s: float = 3.0,
    init_quiet_s: float = 2.0,
    output_dir: Path = Path("/home/lyapunov/bone_drill/results/j9_1/open_loop_attitude"),
    motor_port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    motor_baud: int = 921600,
    imu_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    imu_baud: int = 460800,
    imu_topic: str = "/pus6/imu_platform/data",
    hardware_mode: str = "active",
    start_h30: bool = True,
    require_preflight: bool = True,
) -> Dict[str, Any]:
    print("\n" + "=" * 90)
    print("      J9.1 OPEN-LOOP ATTITUDE COMMAND & PLATFORM H30 MEASUREMENT RUNNER")
    print("=" * 90)
    print(f"Target DOFs:           {dofs}")
    print(f"Target Angle:          +/- {angle_deg:.2f} deg")
    print(f"Hold per stage:        {hold_s:.1f} s")
    print(f"Hardware mode:         {hardware_mode}")
    print(f"Motor UART port:       {motor_port} @ {motor_baud}")
    print(f"H30 IMU port:          {imu_port} @ {imu_baud}")
    print(f"H30 Topic:             {imu_topic}")
    print(f"Output Directory:      {output_dir}")
    print("=" * 90 + "\n")

    # Step 1: Preflight read-only check on motors if in active hardware mode
    if hardware_mode == "active" and require_preflight:
        print("[PREFLIGHT] Running non-intrusive 0x30 status scan on 6 motors...")
        ok, msg = check_motor_read_only_preflight(port=motor_port, baudrate=motor_baud)
        print(f"[PREFLIGHT RESULT] {msg}")
        if not ok:
            raise RuntimeError(f"Motor preflight failed: {msg}")
        print("[PREFLIGHT PASS] Motors verified online and within HOME band.")

    # Step 2: Ensure H30 node is running
    h30_proc: subprocess.Popen | None = None
    if start_h30 and hardware_mode == "active":
        print(f"[H30] Spawning H30 IMU node process for {imu_port}...")
        h30_cmd = [
            "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
            "-r", "__ns:=/pus6/imu_platform",
            "-p", f"port:={imu_port}",
            "-p", f"baud_rate:={imu_baud}",
            "-p", "frame_id:=pus6_platform_imu_link",
            "-p", "world_frame:=world",
            "-p", f"imu_topic:={imu_topic}",
            "-p", "publish_euler:=true",
        ]
        h30_proc = subprocess.Popen(
            h30_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        print(f"[H30] Started H30 node (PID {h30_proc.pid}), waiting 1.5s for initialization...")
        time.sleep(1.5)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"j9_1_open_loop_attitude_{stamp}.csv"
    meta_path = output_dir / f"j9_1_open_loop_attitude_{stamp}_metadata.json"

    if not rclpy.ok():
        rclpy.init()

    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    print(f"[GEOMETRY] Nominal Cartesian HOME z0 = {z0:.6f} m ({z0*1e3:.3f} mm)")

    # Instantiate ROS 2 nodes
    print("[INIT] Instantiating J8 ROS 2 nodes...")
    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
        hardware_mode=hardware_mode,
        port=motor_port,
        baudrate=motor_baud,
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0)
    harness = J91ExperimentHarness(
        imu_topic=imu_topic,
        csv_path=csv_path,
        init_quiet_s=init_quiet_s,
        min_init_samples=20,
    )

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[INIT OK] MultiThreadedExecutor running 5 nodes.")

    experiment_report: Dict[str, Any] = {
        "csv_path": str(csv_path),
        "metadata_path": str(meta_path),
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hardware_mode": hardware_mode,
        "target_angle_deg": angle_deg,
        "dofs_requested": list(dofs),
        "stages_executed": [],
        "dof_results": {},
        "status": "INCOMPLETE",
    }

    try:
        # Phase 0: Hold OPERATING_HOME to establish H30 relative-zero q0
        print(f"\n--- [Phase 0: HOME Relative-Zero Initialization] Holding HOME for {init_quiet_s:.1f}s ---")
        harness.set_stage("init_home", 0.0, 0.0, 0.0)
        t_init_end = time.monotonic() + init_quiet_s + 0.5
        while time.monotonic() < t_init_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0)
            time.sleep(0.020)

        if harness.q0_xyzw is None:
            raise RuntimeError("Failed to establish H30 relative-zero q0! Check if H30 is publishing.")
        print(f"[Phase 0 OK] Relative zero established: {harness.q0_xyzw}")

        # Execute single-DoF sequences
        for dof in dofs:
            dof = dof.lower().strip()
            print(f"\n" + "#" * 60)
            print(f"       STARTING SINGLE-DOF ATTITUDE MOTION: {dof.upper()}")
            print("#" * 60)

            stages = [
                (f"{dof}_pos", +angle_deg, f"{dof.upper()} +{angle_deg:.2f} deg"),
                (f"{dof}_ret1", 0.0, f"Return HOME from +{angle_deg:.2f} deg"),
                (f"{dof}_neg", -angle_deg, f"{dof.upper()} -{angle_deg:.2f} deg"),
                (f"{dof}_ret2", 0.0, f"Return HOME from -{angle_deg:.2f} deg"),
            ]

            for stage_tag, tgt_deg, desc in stages:
                r_deg = tgt_deg if dof == "roll" else 0.0
                p_deg = tgt_deg if dof == "pitch" else 0.0
                y_deg = tgt_deg if dof == "yaw" else 0.0

                r_rad = math.radians(r_deg)
                p_rad = math.radians(p_deg)
                y_rad = math.radians(y_deg)

                print(f"\n>>> Stage [{stage_tag}]: {desc} (holding {hold_s:.1f}s) <<<")
                harness.set_stage(stage_tag, r_deg, p_deg, y_deg)
                experiment_report["stages_executed"].append(stage_tag)

                t_stage_end = time.monotonic() + hold_s
                while time.monotonic() < t_stage_end:
                    harness.publish_target_pose(
                        x=0.0, y=0.0, z=z0, roll_rad=r_rad, pitch_rad=p_rad, yaw_rad=y_rad
                    )
                    time.sleep(0.020)

                    # Continuous safety fault check
                    if any(f != 0 for f in harness.latest_fb_faults):
                        raise RuntimeError(
                            f"Motor fault detected during {stage_tag}: {harness.latest_fb_faults}"
                        )

                print(f">>> Stage [{stage_tag}] complete. Settled steps: {harness.latest_fb_steps} <<<")

            # Intermediate Safety & Sanity Gate after each DOF
            pos_metrics = compute_stage_settled_metrics(harness.samples_by_stage.get(f"{dof}_pos", []))
            neg_metrics = compute_stage_settled_metrics(harness.samples_by_stage.get(f"{dof}_neg", []))

            target_key = f"mean_{dof[0]}"
            meas_pos = pos_metrics.get(target_key, 0.0)
            meas_neg = neg_metrics.get(target_key, 0.0)

            pos_amp = math.sqrt(pos_metrics.get("mean_r", 0)**2 + pos_metrics.get("mean_p", 0)**2 + pos_metrics.get("mean_y", 0)**2)
            neg_amp = math.sqrt(neg_metrics.get("mean_r", 0)**2 + neg_metrics.get("mean_p", 0)**2 + neg_metrics.get("mean_y", 0)**2)

            print(f"\n[{dof.upper()} EVALUATION]")
            print(f"  Commanded: +{angle_deg:.2f} deg -> Measured: {meas_pos:+.4f} deg (err: {angle_deg - meas_pos:+.4f} deg)")
            print(f"    3D Measured: R={pos_metrics.get('mean_r', 0):+.4f}, P={pos_metrics.get('mean_p', 0):+.4f}, Y={pos_metrics.get('mean_y', 0):+.4f} deg (norm={pos_amp:.4f} deg)")
            print(f"    Platform-frame (Rz90): R={pos_metrics.get('mean_plat_r', 0):+.4f}, P={pos_metrics.get('mean_plat_p', 0):+.4f}, Y={pos_metrics.get('mean_plat_y', 0):+.4f} deg")
            print(f"  Commanded: -{angle_deg:.2f} deg -> Measured: {meas_neg:+.4f} deg (err: {-angle_deg - meas_neg:+.4f} deg)")
            print(f"    3D Measured: R={neg_metrics.get('mean_r', 0):+.4f}, P={neg_metrics.get('mean_p', 0):+.4f}, Y={neg_metrics.get('mean_y', 0):+.4f} deg (norm={neg_amp:.4f} deg)")
            print(f"    Platform-frame (Rz90): R={neg_metrics.get('mean_plat_r', 0):+.4f}, P={neg_metrics.get('mean_plat_p', 0):+.4f}, Y={neg_metrics.get('mean_plat_y', 0):+.4f} deg")
            print(f"  Hardware writes: {harness.hardware_write_count}")
            print(f"  Motor faults:    {harness.latest_fb_faults}")

            # Verification assertions
            if hardware_mode == "active":
                if harness.hardware_write_count == 0:
                    raise RuntimeError(f"Hardware write count is 0 after {dof} motion!")
                if any(f != 0 for f in harness.latest_fb_faults):
                    raise RuntimeError(f"Motor fault present: {harness.latest_fb_faults}")
                # Check response amplitude (motion must be detected by H30)
                if pos_amp < 0.10 and neg_amp < 0.10:
                    raise RuntimeError(
                        f"H30 detected negligible 3D response ({pos_amp:.4f} / {neg_amp:.4f}) during {dof}! Check sensor or motion."
                    )

            experiment_report["dof_results"][dof] = {
                "pos_stage": f"{dof}_pos",
                "pos_cmd_deg": +angle_deg,
                "pos_meas": pos_metrics,
                "pos_3d_norm_deg": float(pos_amp),
                "pos_error_deg": float(angle_deg - meas_pos),
                "pos_sign_correct": bool(meas_pos > 0.0),
                "neg_stage": f"{dof}_neg",
                "neg_cmd_deg": -angle_deg,
                "neg_meas": neg_metrics,
                "neg_3d_norm_deg": float(neg_amp),
                "neg_error_deg": float(-angle_deg - meas_neg),
                "neg_sign_correct": bool(meas_neg < 0.0),
            }

        # Final quiet hold at HOME (1.0s)
        harness.set_stage("final_home", 0.0, 0.0, 0.0)
        t_final_end = time.monotonic() + 1.0
        while time.monotonic() < t_final_end:
            harness.publish_target_pose(x=0.0, y=0.0, z=z0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0)
            time.sleep(0.020)

        experiment_report["status"] = "PASS"
        print("\n[EXPERIMENT PASS] All requested DOFs executed cleanly without fault.")

    finally:
        print("\n[TEARDOWN] Shutting down ROS 2 nodes and flushing CSV...")
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
        if rclpy.ok():
            rclpy.shutdown()
        harness.close()

        if h30_proc and h30_proc.poll() is None:
            print(f"[TEARDOWN] Terminating H30 IMU process (PID {h30_proc.pid})...")
            try:
                os.killpg(os.getpgid(h30_proc.pid), signal.SIGINT)
                h30_proc.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(h30_proc.pid), signal.SIGKILL)
                except Exception:
                    pass

    # Compile and write metadata JSON
    experiment_report["end_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    experiment_report["total_recorded_samples"] = harness.recorded_sample_count
    experiment_report["total_imu_samples"] = harness.total_imu_samples
    experiment_report["final_write_count"] = harness.hardware_write_count
    experiment_report["final_motor_steps"] = harness.latest_fb_steps
    experiment_report["final_motor_faults"] = harness.latest_fb_faults
    experiment_report["q0_xyzw"] = list(harness.q0_xyzw) if harness.q0_xyzw else None

    meta_path.write_text(json.dumps(experiment_report, indent=2), encoding="utf-8")
    print(f"[METADATA] Saved metadata JSON to {meta_path}")
    print(f"[CSV] Saved CSV to {csv_path}")

    return experiment_report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J9.1 Open-Loop Attitude Command & Platform H30 Measurement Runner")
    parser.add_argument("--dofs", default="roll,pitch,yaw", help="Comma-separated DOFs: roll,pitch,yaw or single DOF")
    parser.add_argument("--angle-deg", type=float, default=0.5, help="Target rotation angle in degrees (strictly 0.5 deg)")
    parser.add_argument("--hold-time", type=float, default=3.0, help="Hold time per stage in seconds")
    parser.add_argument("--init-quiet-time", type=float, default=2.0, help="Initial quiet hold time at HOME to establish q0")
    parser.add_argument("--output-dir", default="/home/lyapunov/bone_drill/results/j9_1/open_loop_attitude", help="Output directory")
    parser.add_argument("--motor-port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", help="Motor serial port")
    parser.add_argument("--motor-baud", type=int, default=921600, help="Motor baud rate")
    parser.add_argument("--imu-port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00", help="Platform H30 serial port")
    parser.add_argument("--imu-baud", type=int, default=460800, help="H30 baud rate")
    parser.add_argument("--imu-topic", default="/pus6/imu_platform/data", help="H30 ROS 2 topic")
    parser.add_argument("--hardware-mode", choices=["active", "mock"], default="active", help="Hardware mode: active or mock")
    parser.add_argument("--no-start-h30", dest="start_h30", action="store_false", help="Do not spawn H30 node process automatically")
    parser.add_argument("--no-preflight", dest="require_preflight", action="store_false", help="Skip preflight read-only check")

    args = parser.parse_args(list(argv) if argv is not None else None)
    dof_list = [d.strip().lower() for d in args.dofs.split(",") if d.strip()]

    try:
        report = run_attitude_experiment(
            dofs=dof_list,
            angle_deg=args.angle_deg,
            hold_s=args.hold_time,
            init_quiet_s=args.init_quiet_time,
            output_dir=Path(args.output_dir),
            motor_port=args.motor_port,
            motor_baud=args.motor_baud,
            imu_port=args.imu_port,
            imu_baud=args.imu_baud,
            imu_topic=args.imu_topic,
            hardware_mode=args.hardware_mode,
            start_h30=args.start_h30,
            require_preflight=args.require_preflight,
        )
        return 0 if report.get("status") == "PASS" else 1
    except Exception as e:
        print(f"\n[FATAL ERROR] Experiment failed: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
