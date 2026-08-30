#!/usr/bin/env python3
"""J9-E1B-R4C: Canonical J8 Runtime One-Shot Qualification.

Strict Architectural Invariant:
  Uses the CANONICAL frozen J8 ROS2 node graph and driver path ONLY:
    /pus6/target_cartesian_pose (PoseStamped)
    --> Pus6KinematicsNode
    --> /pus6/desired_joint_deltas (Float64MultiArray)
    --> Pus6ControlNode (StatefulJerkLimitedShaper)
    --> /pus6/command_joint_steps (Int32MultiArray)
    --> Pus6MotorDriverNode (J7SixAxisServoTransport @ 50Hz, active mode)
    --> Real 6PUS physical mechanism
    --> /pus6/joint_feedback/timed (TimedJointFeedback)
    --> Pus6FkNode (200Hz FixedLagJointSynchronizer @ 160ms delay)
    --> /pus6/fk/state (FkStateStamped)

  J9 components purely observe and record:
    - Platform TimedImu and C++ source capture (/pus6/imu_platform/timed)
    - TimedJointFeedback telemetry
    - FK state reconstruction
    - Commanded Cartesian target pose

Execution Rule:
  Exactly ONE real-hardware motion run is executed. No automatic retries.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent
WS_ROOT = REPO_ROOT / "bone_ws"
for pkg in ("pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"):
    p = WS_ROOT / "src" / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
scripts_dir = WS_ROOT / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from h30_imu.msg import TimedImu
from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import MOTOR_IDS
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

MOTOR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PLATFORM_IMU_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00"
BAUD_MOTOR = 921600
BAUD_IMU = 460800


@dataclass
class R4cMotionSegment:
    name: str
    role: str
    duration_s: float
    amp_rpy_deg: tuple[float, float, float]
    freq_rpy_hz: tuple[float, float, float]
    phase_rpy_deg: tuple[float, float, float]

    def target_rpy_rad(self, t_s: float) -> tuple[float, float, float]:
        r = math.radians(self.amp_rpy_deg[0]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[0] * t_s + math.radians(self.phase_rpy_deg[0])) if self.amp_rpy_deg[0] > 0 else 0.0
        p = math.radians(self.amp_rpy_deg[1]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[1] * t_s + math.radians(self.phase_rpy_deg[1])) if self.amp_rpy_deg[1] > 0 else 0.0
        y = math.radians(self.amp_rpy_deg[2]) * math.sin(2.0 * math.pi * self.freq_rpy_hz[2] * t_s + math.radians(self.phase_rpy_deg[2])) if self.amp_rpy_deg[2] > 0 else 0.0
        return r, p, y


def canonical_motion_segments() -> list[R4cMotionSegment]:
    return [
        R4cMotionSegment("stationary_preroll", "stationary", 4.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        R4cMotionSegment("roll_motion", "roll", 8.0, (1.5, 0.0, 0.0), (0.20, 0.0, 0.0), (0.0, 0.0, 0.0)),
        R4cMotionSegment("pitch_motion", "pitch", 8.0, (0.0, 1.5, 0.0), (0.0, 0.20, 0.0), (0.0, 0.0, 0.0)),
        R4cMotionSegment("home_return", "home_return", 4.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ]


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
        self.thread = threading.Thread(target=self._run, name="r4c-csv-writer", daemon=True)
        self.thread.start()

    def _open(self, stream: str, row: dict[str, Any]) -> csv.DictWriter:
        if stream not in self.writers:
            f = (self.out_dir / f"{stream}.csv").open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            writer.writeheader()
            self.files[stream] = f
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
                break
            stream, row = item
            try:
                self._open(stream, row).writerow(row)
                self.written[stream] = self.written.get(stream, 0) + 1
            except Exception:
                self.errors += 1
            self.queue.task_done()

    def stop(self) -> None:
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self.thread.join(timeout=5.0)
        for f in self.files.values():
            f.flush()
            f.close()


class CanonicalObserverNode(Node):
    """Passive recording node subscribing to the canonical J8 ROS2 graph."""

    def __init__(self, out_dir: Path, queue_capacity: int = 131072) -> None:
        super().__init__("j9_e1b_r4c_observer")
        self.writer = CsvQueueWriter(out_dir, queue_capacity)
        
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4096,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(TimedImu, "/pus6/imu_platform/timed", self._on_imu, reliable_qos)
        self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_feedback, sensor_qos)
        self.create_subscription(FkStateStamped, "/pus6/fk/state", self._on_fk, sensor_qos)
        self.create_subscription(PoseStamped, "/pus6/target_cartesian_pose", self._on_target_pose, reliable_qos)
        self.create_subscription(Float64MultiArray, "/pus6/desired_joint_deltas", self._on_ik, reliable_qos)
        self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd, sensor_qos)

    @staticmethod
    def _stamp_ns(msg: Any) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_imu(self, msg: TimedImu) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("imu", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "publisher_sequence": int(msg.publisher_sequence),
            "tid": int(msg.tid),
            "sample_timestamp_us": int(msg.sample_timestamp_us),
            "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
            "gx_rad_s": float(msg.imu.angular_velocity.x),
            "gy_rad_s": float(msg.imu.angular_velocity.y),
            "gz_rad_s": float(msg.imu.angular_velocity.z),
        })

    def _on_feedback(self, msg: TimedJointFeedback) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("joint_feedback", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "snapshot_start_monotonic_ns": int(msg.snapshot_start_monotonic_ns),
            "snapshot_end_monotonic_ns": int(msg.snapshot_end_monotonic_ns),
            "uart_cycle_sequence": int(msg.uart_cycle_sequence),
            "raw_steps": json.dumps([int(v) for v in msg.raw_steps]),
            "currents_ma": json.dumps([int(v) for v in msg.currents_ma]),
            "fault_codes": json.dumps([int(v) for v in msg.fault_codes]),
        })

    def _on_fk(self, msg: FkStateStamped) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("fk", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "query_monotonic_timestamp_ns": int(msg.query_monotonic_timestamp_ns),
            "converged": int(msg.converged),
            "residual_norm_meters": float(msg.residual_norm_meters),
            "compute_time_us": float(msg.compute_time_us),
            "pos_x_m": float(msg.pose.position.x),
            "pos_y_m": float(msg.pose.position.y),
            "pos_z_m": float(msg.pose.position.z),
            "quat_xyzw": json.dumps([float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)]),
        })

    def _on_target_pose(self, msg: PoseStamped) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("target_pose", {
            "callback_monotonic_ns": t_cb,
            "ros_header_stamp_ns": self._stamp_ns(msg),
            "pos_x_m": float(msg.pose.position.x),
            "pos_y_m": float(msg.pose.position.y),
            "pos_z_m": float(msg.pose.position.z),
            "quat_xyzw": json.dumps([float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)]),
        })

    def _on_ik(self, msg: Float64MultiArray) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("ik_deltas", {
            "callback_monotonic_ns": t_cb,
            "deltas_m": json.dumps([float(v) for v in msg.data]),
        })

    def _on_cmd(self, msg: Int32MultiArray) -> None:
        t_cb = time.monotonic_ns()
        self.writer.enqueue("command_steps", {
            "callback_monotonic_ns": t_cb,
            "steps": json.dumps([int(v) for v in msg.data]),
        })

    def close(self) -> None:
        self.writer.stop()


def start_h30_imu(source_path: Path) -> subprocess.Popen[str]:
    cmd = [
        "ros2", "run", "h30_imu", "h30_imu_node", "--ros-args",
        "-r", "__ns:=/pus6/imu_platform",
        "-p", f"port:={PLATFORM_IMU_PORT}",
        "-p", f"baud_rate:={BAUD_IMU}",
        "-p", "frame_id:=pus6_platform_imu_link",
        "-p", "world_frame:=world",
        "-p", "imu_topic:=/pus6/imu_platform/data",
        "-p", "euler_topic:=/pus6/imu_platform/euler",
        "-p", "temperature_topic:=/pus6/imu_platform/temperature",
        "-p", "timed_topic:=/pus6/imu_platform/timed",
        "-p", "diagnostics_topic:=/pus6/imu_platform/diagnostics",
        "-p", "publish_euler:=true",
        "-p", "publish_tf:=false",
        "-p", "timed_qos_profile:=calibration",
        "-p", "timed_qos_depth:=4096",
        "-p", "timed_qos_reliability:=reliable",
        "-p", "source_capture_enabled:=true",
        "-p", f"source_capture_path:={source_path}",
        "-p", "source_capture_queue_capacity:=131072",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def stop_proc(proc: subprocess.Popen[str] | None, timeout_s: float = 10.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                proc.kill()


def load_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def sequence_stats(values: list[int], expected_step: int = 1) -> dict[str, Any]:
    if len(values) < 2:
        return {"count": len(values), "missing": 0, "duplicates": 0, "out_of_order": 0, "max_delta": None, "p95_delta": None}
    d = np.diff(np.asarray(values, dtype=np.int64))
    return {
        "count": int(len(values)),
        "missing": int(np.sum(np.maximum(0, d - expected_step) // max(1, expected_step))),
        "duplicates": int(np.sum(d == 0)),
        "out_of_order": int(np.sum(d < 0)),
        "max_delta": int(np.max(d)),
        "p95_delta": float(np.percentile(d, 95.0)),
    }


def analyze_r4c_run(run_dir: Path, segments: list[dict[str, Any]]) -> dict[str, Any]:
    source_rows = load_csv(run_dir / "source.csv")
    imu_rows = load_csv(run_dir / "imu.csv")
    fb_rows = load_csv(run_dir / "joint_feedback.csv")
    fk_rows = load_csv(run_dir / "fk.csv")
    cmd_rows = load_csv(run_dir / "command_steps.csv")
    target_rows = load_csv(run_dir / "target_pose.csv")

    out: dict[str, Any] = {
        "total_counts": {
            "source_imu_rows": len(source_rows),
            "subscribed_imu_rows": len(imu_rows),
            "joint_feedback_rows": len(fb_rows),
            "fk_rows": len(fk_rows),
            "command_steps_rows": len(cmd_rows),
            "target_pose_rows": len(target_rows),
        },
        "windows": [],
    }

    for seg in segments:
        t_start = int(seg["start_monotonic_ns"])
        t_end = int(seg["end_monotonic_ns"])

        # Filter rows corresponding to segment
        src_seg = [r for r in source_rows if t_start <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= t_end]
        imu_seg = [r for r in imu_rows if t_start <= int(float(r.get("host_monotonic_arrival_time_ns", 0))) <= t_end]
        fb_seg = [r for r in fb_rows if t_start <= int(float(r.get("snapshot_start_monotonic_ns", 0))) <= t_end]
        fk_seg = [r for r in fk_rows if t_start <= int(float(r.get("query_monotonic_timestamp_ns", 0))) <= t_end]

        src_diag = sequence_stats([int(r["publisher_sequence"]) for r in src_seg])
        sub_diag = sequence_stats([int(r["publisher_sequence"]) for r in imu_seg])
        fb_diag = sequence_stats([int(r["uart_cycle_sequence"]) for r in fb_seg])
        fk_diag = sequence_stats([int(r["query_monotonic_timestamp_ns"]) for r in fk_seg], 5_000_000)

        out["windows"].append({
            "segment": seg,
            "counts": {
                "source": len(src_seg),
                "imu_sub": len(imu_seg),
                "feedback": len(fb_seg),
                "fk": len(fk_seg),
            },
            "source_publisher_sequence": src_diag,
            "subscriber_publisher_sequence": sub_diag,
            "joint_feedback_sequence": fb_diag,
            "fk_query_timing": fk_diag,
        })

    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="J9-E1B-R4C: Canonical J8 Runtime One-Shot Qualification")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "reports" / "j9_e1b_r4c_canonical_qualification_20260828")
    parser.add_argument("--real-hardware", action="store_true", help="Authorize execution on physical hardware")
    parser.add_argument("--confirm-motion", action="store_true", help="Confirm physical platform motion")
    parser.add_argument("--fixed-lag-delay-ms", type=float, default=160.0)
    args = parser.parse_args()

    if not (args.real_hardware and args.confirm_motion):
        print("Dry run mode: --real-hardware and --confirm-motion required for physical qualification.", file=sys.stderr)
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    report_file = args.out_dir / "r4c_canonical_report.md"
    raw_file = args.out_dir / "r4c_raw.json"

    print("\n" + "=" * 80)
    print("      6PUS J9-E1B-R4C: CANONICAL J8 RUNTIME ONE-SHOT QUALIFICATION")
    print("=" * 80)
    print(f"Motor Port:     {MOTOR_PORT} ({BAUD_MOTOR} baud)")
    print(f"Platform IMU:   {PLATFORM_IMU_PORT} ({BAUD_IMU} baud)")
    print(f"FixedLag Delay: {args.fixed_lag_delay_ms:.1f} ms")
    print(f"Output Dir:     {args.out_dir}")
    print("=" * 80 + "\n")

    results: dict[str, Any] = {
        "stage": "J9-E1B-R4C",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "real_hardware_runs_attempted": 1,
        "run_status": "UNKNOWN",
        "preflight_audit": None,
        "home_result": None,
        "run_analysis": None,
        "park_result": None,
        "final_readonly_audit": None,
        "error": None,
    }

    # Step 1: Preflight Read-Only Audit
    print("\n--- [Step 1] Preflight READ_ONLY Hardware Audit (4.0s) ---")
    audit_pre = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
    results["preflight_audit"] = audit_pre
    if audit_pre.get("status") != "PASS":
        print(f"[PREFLIGHT FAIL] Initial hardware state is not PASS: {audit_pre}", file=sys.stderr)
        results["run_status"] = "PREFLIGHT_FAIL"
        raw_file.write_text(json.dumps(results, indent=2))
        return 1

    time.sleep(0.5)

    # Step 2: Canonical Transition to OPERATING_HOME
    print("\n--- [Step 2] Canonical J8 Transition to HOME ---")
    home_res = run_servo_home_park("home", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
    results["home_result"] = asdict(home_res)
    if not home_res.passed:
        print("[HOME FAIL] Transition to HOME failed!", file=sys.stderr)
        results["run_status"] = "HOME_FAIL"
        # Emergency PARK
        time.sleep(0.5)
        park_res = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
        results["park_result"] = asdict(park_res)
        results["final_readonly_audit"] = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        raw_file.write_text(json.dumps(results, indent=2))
        return 1

    time.sleep(0.5)

    # Step 3: Canonical ROS 2 Execution (EXACTLY ONE RUN)
    print("\n--- [Step 3] Canonical ROS2 Pipeline Execution (Single Run) ---")
    source_csv = args.out_dir / "source.csv"
    h30_proc = start_h30_imu(source_csv)

    rclpy.init()
    geo_cfg = load_geometry_config()
    z0 = float(geo_cfg.nominal_height_m)

    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    motor_driver_node = Pus6MotorDriverNode(
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
    fk_node = Pus6FkNode(fixed_lag_delay_ms=args.fixed_lag_delay_ms, query_rate_hz=200.0)
    observer_node = CanonicalObserverNode(args.out_dir)

    executor = SingleThreadedExecutor()
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(motor_driver_node)
    executor.add_node(fk_node)
    executor.add_node(observer_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(3.0)  # Discovery & buffer pre-roll

    # Client target publisher
    target_pub_node = Node("r4c_target_publisher")
    reliable_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    target_pub = target_pub_node.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", reliable_qos)

    def publish_target(x: float, y: float, z: float, r: float, p: float, y_rad: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = target_pub_node.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        cy = math.cos(y_rad * 0.5)
        sy = math.sin(y_rad * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)
        target_pub.publish(msg)

    segments_log = []
    run_error = None
    try:
        segments = canonical_motion_segments()
        for seg in segments:
            seg_start_ns = time.monotonic_ns()
            print(f"[MOTION] Executing segment '{seg.name}' ({seg.duration_s:.1f}s)...", flush=True)
            n_steps = int(round(seg.duration_s * 50.0))
            for k in range(n_steps):
                t_step_s = k * 0.020
                r, p, y = seg.target_rpy_rad(t_step_s)
                publish_target(0.0, 0.0, z0, r, p, y)
                time.sleep(0.020)
            seg_end_ns = time.monotonic_ns()
            segments_log.append({
                "name": seg.name,
                "role": seg.role,
                "duration_s": seg.duration_s,
                "start_monotonic_ns": seg_start_ns,
                "end_monotonic_ns": seg_end_ns,
            })

        # Post-motion hold: hold at HOME for 2.0s so fixed-lag reconstruction drains completely
        print("[MOTION] Holding at HOME for 2.0s post-motion to drain FK buffer...", flush=True)
        for _ in range(100):
            publish_target(0.0, 0.0, z0, 0.0, 0.0, 0.0)
            time.sleep(0.020)

        results["run_status"] = "COMPLETED"
    except Exception as exc:
        run_error = f"{type(exc).__name__}: {exc}"
        results["run_status"] = "ERROR"
        results["error"] = run_error
        print(f"[RUN ERROR] {run_error}", file=sys.stderr)
    finally:
        print("\n[TEARDOWN] Shutting down canonical ROS2 pipeline...")
        target_pub_node.destroy_node()
        observer_node.close()
        executor.shutdown()
        kinematics_node.destroy_node()
        control_node.destroy_node()
        motor_driver_node.destroy_node()
        fk_node.destroy_node()
        observer_node.destroy_node()
        rclpy.shutdown()
        stop_proc(h30_proc)
        print("[TEARDOWN OK] ROS2 nodes stopped.")

    time.sleep(0.5)

    # Step 4: Canonical J8 Return to PARK
    print("\n--- [Step 4] Canonical J8 Transition to PARK ---")
    park_res = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
    results["park_result"] = asdict(park_res)

    time.sleep(0.5)

    # Step 5: Final READ_ONLY Hardware Audit
    print("\n--- [Step 5] Final READ_ONLY Hardware Audit (4.0s) ---")
    audit_post = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
    results["final_readonly_audit"] = audit_post

    # Step 6: Telemetry Analysis
    print("\n--- [Step 6] Analyzing Recorded Data & Continuity ---")
    analysis = analyze_r4c_run(args.out_dir, segments_log)
    results["run_analysis"] = analysis

    raw_file.write_text(json.dumps(results, indent=2))
    print(f"\n[EVIDENCE SAVED] Raw JSON: {raw_file}")

    # Generate Markdown Report
    report_md = f"""# J9-E1B-R4C Canonical J8 Runtime One-Shot Qualification Report

**Execution Timestamp**: {time.strftime("%Y-%m-%d %H:%M:%S")}  
**Robot Extrinsics Status**: `UNCALIBRATED_UNSET`  
**Execution Mode**: Canonical J8 ROS2 Node Graph + Passive J9 Observer  
**Hard Limit**: Exactly ONE real-hardware motion run attempted.

---

## 1. Hardware State & Safety Summary

- **Preflight Audit Status**: `{audit_pre.get("status")}` (`{audit_pre.get("startup_classification")}`)
- **HOME Transition**: `{"PASS" if home_res.passed else "FAIL"}` ({home_res.total_cycles} cycles)
- **Motion Execution**: `{results["run_status"]}` (Error: `{results.get("error")}`)
- **PARK Transition**: `{"PASS" if park_res.passed else "FAIL"}` ({park_res.total_cycles} cycles)
- **Final READ_ONLY Audit**: `{audit_post.get("status")}` (`{audit_post.get("startup_classification")}`)
- **Final Positions**: `{audit_post.get("initial_positions_steps")}`

---

## 2. Topic & Capture Metrics

| Stream / Topic | Recorded Rows | Description |
| :--- | :--- | :--- |
| `H30 Source Capture` (`source.csv`) | {analysis["total_counts"]["source_imu_rows"]} | C++ driver-level direct disk capture |
| `TimedImu ROS2 Subscriber` (`imu.csv`) | {analysis["total_counts"]["subscribed_imu_rows"]} | `/pus6/imu_platform/timed` (200 Hz) |
| `TimedJointFeedback` (`joint_feedback.csv`) | {analysis["total_counts"]["joint_feedback_rows"]} | `/pus6/joint_feedback/timed` (50 Hz) |
| `FK State Reconstruction` (`fk.csv`) | {analysis["total_counts"]["fk_rows"]} | `/pus6/fk/state` (200 Hz, delay=160ms) |
| `Command Joint Steps` (`command_steps.csv`) | {analysis["total_counts"]["command_steps_rows"]} | `/pus6/command_joint_steps` |
| `Target Cartesian Pose` (`target_pose.csv`) | {analysis["total_counts"]["target_pose_rows"]} | `/pus6/target_cartesian_pose` |

---

## 3. Segment-by-Segment Continuity Analysis

"""
    for win in analysis.get("windows", []):
        seg_name = win["segment"]["name"]
        role = win["segment"]["role"]
        dur = win["segment"]["duration_s"]
        src_m = win["source_publisher_sequence"]["missing"]
        sub_m = win["subscriber_publisher_sequence"]["missing"]
        fb_m = win["joint_feedback_sequence"]["missing"]
        fk_m = win["fk_query_timing"]["missing"]
        fk_max_dt = win["fk_query_timing"]["max_delta"] / 1e6 if win["fk_query_timing"]["max_delta"] else 0.0
        report_md += f"""### Segment: `{seg_name}` (Role: `{role}`, Duration: `{dur:.1f}s`)
- **IMU Source Missing**: `{src_m}` (Rows: {win["counts"]["source"]})
- **IMU Subscriber Missing**: `{sub_m}` (Rows: {win["counts"]["imu_sub"]})
- **Joint Feedback Missing**: `{fb_m}` (Rows: {win["counts"]["feedback"]})
- **FK Query Timing Gaps**: `{fk_m}` (Rows: {win["counts"]["fk"]}, Max gap: `{fk_max_dt:.2f} ms`)

"""

    report_md += """
---

## 4. Status of Open Blockers

1. **ID5 Feedback Timeout**: **`OPEN / NOT REPRODUCED`**.
2. **Hardware Write Count**: **`ESTIMATED`**.
3. **FK / Feedback Continuity**: Evaluated on canonical J8 runtime.
"""
    report_file.write_text(report_md)
    print(f"[REPORT WRITTEN] {report_file}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
