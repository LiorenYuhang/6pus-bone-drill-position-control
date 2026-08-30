#!/usr/bin/env python3
"""J9-E1B-R3 platform IMU orientation hardware recalibration.

This script is scoped to R_P_IP calibration only. It reuses the frozen J8
transport, IK, shaper, HOME/PARK helpers, and read-only audit code without
modifying them. It launches the platform H30 reader with calibration QoS and
records IMU/FK topics in a separate process.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT = Path(__file__).resolve()
PKG_ROOT = SCRIPT.parents[1]
WS_ROOT = SCRIPT.parents[3]
REPO_ROOT = WS_ROOT.parent
SRC_ROOT = WS_ROOT / "src"
for pkg in ["h30_imu", "pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = SRC_ROOT / pkg
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
if str(WS_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(WS_ROOT / "scripts"))

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray
from h30_imu.msg import TimedImu
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.j7_real_uart_adapter import J7StopAdapter, J7UartAdapter
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
from pus6_motor_driver.safety import MOTOR_IDS, classify_startup_state
from run_j7_servo_home_park import run_servo_home_park
from run_j8_c_hardware_readonly_audit import run_j8_c_audit

import j9_e1a_platform_orientation_calibrator as cal

MOTOR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PLATFORM_IMU_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00"
BAUD_MOTOR = 921600
BAUD_IMU = 460800
FIXED_LAG_DELAY_MS = 160.0
CYCLE_HZ = 50.0
CYCLE_DT = 1.0 / CYCLE_HZ


@dataclass
class SegmentSpec:
    name: str
    role: str
    run_index: int
    duration_s: float
    amp_deg: tuple[float, float, float]
    freq_hz: tuple[float, float, float]
    phase_deg: tuple[float, float, float]
    ramp_s: float = 4.0

    def target_rpy_rad(self, t_s: float) -> tuple[float, float, float]:
        if self.duration_s <= 0.0:
            return (0.0, 0.0, 0.0)
        env = envelope(t_s, self.duration_s, self.ramp_s)
        vals = []
        for amp_d, freq, phase_d in zip(self.amp_deg, self.freq_hz, self.phase_deg):
            amp = math.radians(amp_d)
            vals.append(env * amp * math.sin(2.0 * math.pi * freq * t_s + math.radians(phase_d)))
        return (vals[0], vals[1], vals[2])

    def jsonable(self, start_ns: int = 0, end_ns: int = 0) -> dict[str, Any]:
        d = asdict(self)
        d["amp_deg"] = list(self.amp_deg)
        d["freq_hz"] = list(self.freq_hz)
        d["phase_deg"] = list(self.phase_deg)
        d["start_monotonic_ns"] = int(start_ns)
        d["end_monotonic_ns"] = int(end_ns)
        return d


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def envelope(t_s: float, duration_s: float, ramp_s: float) -> float:
    if duration_s <= 0.0:
        return 0.0
    r = max(0.001, min(ramp_s, duration_s / 2.0))
    return min(smoothstep(t_s / r), smoothstep((duration_s - t_s) / r))


def r3_segments(run_index: int) -> list[SegmentSpec]:
    return [
        SegmentSpec(f"run{run_index}_stationary_preroll", "stationary", run_index, 15.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0),
        SegmentSpec(f"run{run_index}_roll_excitation", "roll_axis", run_index, 28.0, (2.0, 0.0, 0.0), (0.23, 0.0, 0.0), (0.0, 0.0, 0.0)),
        SegmentSpec(f"run{run_index}_pitch_excitation", "pitch_axis", run_index, 28.0, (0.0, 2.0, 0.0), (0.0, 0.23, 0.0), (0.0, 0.0, 0.0)),
        SegmentSpec(f"run{run_index}_yaw_excitation", "yaw_axis", run_index, 30.0, (0.0, 0.0, 3.0), (0.0, 0.0, 0.16), (0.0, 0.0, 0.0)),
        SegmentSpec(f"run{run_index}_combined_multiaxis_fit", "combined_fit", run_index, 60.0, (2.0, 2.0, 3.0), (0.20, 0.17, 0.13), (0.0, 90.0, 45.0)),
        SegmentSpec(f"run{run_index}_withheld_validation", "withheld_validation", run_index, 45.0, (2.0, 2.0, 3.0), (0.23, 0.19, 0.15), (60.0, 10.0, 120.0)),
        SegmentSpec(f"run{run_index}_home_return_hold", "home_return", run_index, 10.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0),
    ]


def peak_rates_rad_s(segment: SegmentSpec) -> list[float]:
    return [abs(math.radians(a) * 2.0 * math.pi * f) for a, f in zip(segment.amp_deg, segment.freq_hz)]


class TopicRecorder(Node):
    def __init__(self, qos_depth: int, reliability: str) -> None:
        super().__init__("j9_e1b_r3_isolated_topic_recorder")
        rel = ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
        imu_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=int(qos_depth), reliability=rel, durability=DurabilityPolicy.VOLATILE)
        diag_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.lock = threading.Lock()
        self.platform: list[dict[str, Any]] = []
        self.platform_diagnostics: list[dict[str, Any]] = []
        self.fk: list[dict[str, Any]] = []
        self.joint_feedback: list[dict[str, Any]] = []
        self.driver_diagnostics: list[dict[str, Any]] = []
        self.create_subscription(TimedImu, "/pus6/imu_platform/timed", self._on_imu, imu_qos)
        self.create_subscription(DiagnosticArray, "/pus6/imu_platform/diagnostics", self._on_diag, diag_qos)
        self.create_subscription(FkStateStamped, "/pus6/fk/state", self._on_fk, qos_profile_sensor_data)
        self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_feedback, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/pus6/driver_diagnostics", self._on_driver_diag, qos_profile_sensor_data)

    def _on_imu(self, msg: TimedImu) -> None:
        row = {
            "callback_start_monotonic_ns": time.monotonic_ns(),
            "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "tid": int(msg.tid),
            "publisher_sequence": int(msg.publisher_sequence),
            "has_sample_timestamp": bool(msg.has_sample_timestamp),
            "sample_timestamp_us": int(msg.sample_timestamp_us),
            "has_data_ready_timestamp": bool(msg.has_data_ready_timestamp),
            "data_ready_timestamp_us": int(msg.data_ready_timestamp_us),
            "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
            "angular_velocity_rad_s": [float(msg.imu.angular_velocity.x), float(msg.imu.angular_velocity.y), float(msg.imu.angular_velocity.z)],
            "linear_acceleration_m_s2": [float(msg.imu.linear_acceleration.x), float(msg.imu.linear_acceleration.y), float(msg.imu.linear_acceleration.z)],
            "orientation_xyzw": [float(msg.imu.orientation.x), float(msg.imu.orientation.y), float(msg.imu.orientation.z), float(msg.imu.orientation.w)],
        }
        with self.lock:
            self.platform.append(row)

    def _on_diag(self, msg: DiagnosticArray) -> None:
        rows = []
        for st in msg.status:
            rows.append({
                "name": st.name,
                "hardware_id": st.hardware_id,
                "level": int.from_bytes(st.level, "little", signed=False) if isinstance(st.level, (bytes, bytearray)) else int(st.level),
                "message": st.message,
                "values": {kv.key: kv.value for kv in st.values},
            })
        with self.lock:
            self.platform_diagnostics.append({
                "host_monotonic_ns": time.monotonic_ns(),
                "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
                "status": rows,
            })

    def _on_fk(self, msg: FkStateStamped) -> None:
        row = {
            "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            "query_monotonic_timestamp_ns": int(msg.query_monotonic_timestamp_ns),
            "pose_quaternion_xyzw": [float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)],
            "pose_xyz_m": [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)],
            "euler_angles_rad": [float(v) for v in msg.euler_angles_rad],
            "residual_norm_meters": float(msg.residual_norm_meters),
            "converged": bool(msg.converged),
            "compute_time_us": float(msg.compute_time_us),
        }
        with self.lock:
            self.fk.append(row)

    def _on_feedback(self, msg: TimedJointFeedback) -> None:
        row = {
            "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            "snapshot_start_monotonic_ns": int(msg.snapshot_start_monotonic_ns),
            "snapshot_end_monotonic_ns": int(msg.snapshot_end_monotonic_ns),
            "per_axis_rx_done_monotonic_ns": [int(v) for v in msg.per_axis_rx_done_monotonic_ns],
            "raw_steps": [int(v) for v in msg.raw_steps],
            "positions_meters": [float(v) for v in msg.positions_meters],
            "currents_ma": [int(v) for v in msg.currents_ma],
            "fault_codes": [int(v) for v in msg.fault_codes],
            "uart_cycle_sequence": int(msg.uart_cycle_sequence),
        }
        with self.lock:
            self.joint_feedback.append(row)

    def _on_driver_diag(self, msg: Float64MultiArray) -> None:
        with self.lock:
            self.driver_diagnostics.append({"host_monotonic_ns": time.monotonic_ns(), "data": [float(v) for v in msg.data]})


def run_record_only(out: Path, qos_depth: int, reliability: str) -> int:
    stop_event = threading.Event()
    def _handle(_signum: int, _frame: Any) -> None:
        stop_event.set()
    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)
    rclpy.init()
    node = TopicRecorder(qos_depth, reliability)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    finally:
        time.sleep(0.25)
        with node.lock:
            raw = {
                "platform": list(node.platform),
                "platform_diagnostics": list(node.platform_diagnostics),
                "fk": list(node.fk),
                "joint_feedback": list(node.joint_feedback),
                "driver_diagnostics": list(node.driver_diagnostics),
            }
        out.write_text(json.dumps(raw, indent=2), encoding="utf-8")
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


class RosFeedbackPublisher:
    def __init__(self, qos_depth: int = 4096) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("j9_e1b_r3_feedback_publisher")
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(qos_depth),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.feedback_pub = self.node.create_publisher(TimedJointFeedback, "/pus6/joint_feedback/timed", pub_qos)
        self.cmd_steps_pub = self.node.create_publisher(Int32MultiArray, "/pus6/command_joint_steps", pub_qos)
        self.diag_pub = self.node.create_publisher(Float64MultiArray, "/pus6/driver_diagnostics", pub_qos)

    def publish(self, cycle_seq: int, start_ns: int, end_ns: int, raw_steps: list[int], currents: list[int], faults: list[int], batch_ms: float, deadline_miss: bool, missing_feedback: int) -> None:
        msg = TimedJointFeedback()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.snapshot_start_monotonic_ns = int(start_ns)
        msg.snapshot_end_monotonic_ns = int(end_ns)
        msg.per_axis_rx_done_monotonic_ns = [int(end_ns)] * 6
        msg.raw_steps = [int(v) for v in raw_steps]
        msg.positions_meters = [0.050 + (int(v) - 1000) * 0.000025 for v in raw_steps]
        msg.currents_ma = [int(v) for v in currents]
        msg.forces_g_raw = [0] * 6
        msg.temperatures_c = [0] * 6
        msg.fault_codes = [int(v) for v in faults]
        msg.uart_cycle_sequence = int(cycle_seq)
        self.feedback_pub.publish(msg)
        cmd = Int32MultiArray()
        cmd.data = [int(v) for v in raw_steps]
        self.cmd_steps_pub.publish(cmd)
        diag = Float64MultiArray()
        diag.data = [float(cycle_seq), float(deadline_miss), float(missing_feedback), float(max(currents) if currents else 0), float(max(abs(f) for f in faults) if faults else 0), float(batch_ms)]
        self.diag_pub.publish(diag)

    def close(self) -> None:
        try:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def start_h30() -> subprocess.Popen[str]:
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
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def start_fk() -> subprocess.Popen[str]:
    cmd = [
        "ros2", "run", "pus6_kinematics", "fk_node", "--ros-args",
        "-p", f"fixed_lag_delay_ms:={FIXED_LAG_DELAY_MS}",
        "-p", "query_rate_hz:=200.0",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def start_recorder(out: Path) -> subprocess.Popen[str]:
    cmd = [sys.executable, str(SCRIPT), "--record-only", "--out", str(out), "--timed-qos-depth", "4096", "--timed-qos-reliability", "reliable"]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def drain_log(proc: subprocess.Popen[str], label: str, sink: list[str]) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        text = line.rstrip()
        sink.append(text)
        if "Opened H30 serial port" in text or "Cannot open" in text or "initialized" in text or "error" in text.lower() or "warn" in text.lower():
            print(f"[{label}] {text}", flush=True)


def stop_proc(proc: subprocess.Popen[str] | None, timeout_s: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except Exception:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()


def execute_active_repeat(run_index: int, recorder_json: Path) -> dict[str, Any]:
    recorder = start_recorder(recorder_json)
    h30 = start_h30()
    fk = start_fk()
    logs = {"REC": [], "H30": [], "FK": []}
    try:
        for proc, label in [(recorder, "REC"), (h30, "H30"), (fk, "FK")]:
            threading.Thread(target=drain_log, args=(proc, label, logs[label]), daemon=True).start()
    except Exception:
        stop_proc(h30)
        stop_proc(fk)
        stop_proc(recorder)
        raise
    time.sleep(4.0)

    ser = None
    pub = RosFeedbackPublisher()
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    home_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]
    shaper = StatefulJerkLimitedShaper(max_speed_m_per_sec=0.005, max_accel_m_per_sec2=0.050, max_jerk_m_per_sec3=0.500, omega_n=20.0)
    segments_out: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    motor_summary: dict[str, Any] = {}
    stop_success = False
    status = "PASS"
    error = None
    adapter = None
    stop_adapter = None

    try:
        ser = serial.Serial(MOTOR_PORT, baudrate=BAUD_MOTOR, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.015, write_timeout=0.015)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        adapter = J7UartAdapter(ser, baudrate=BAUD_MOTOR, timeout_s=0.015, min_frame_interval_s=0.0005)
        stop_adapter = J7StopAdapter(adapter)
        transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0, rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX, min_frame_interval_s=0.0005, pause_callback=stop_adapter)
        feedback = J7ServoFeedbackScheduler(transport, full_scan_period_s=1e9, feedback_budget_s=0.010, timeout_escalation_misses=4)

        print(f"[R3-{run_index}] Active preflight full scan at HOME", flush=True)
        pre = feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
        if pre.deferred or len(pre.samples) != 6:
            raise RuntimeError("active preflight full scan incomplete")
        initial_positions = {s.motor_id: int(s.position) for s in pre.samples if s.position is not None}
        initial_currents = {s.motor_id: int(s.current_mA or 0) for s in pre.samples}
        initial_faults = {s.motor_id: int(s.fault or 0) for s in pre.samples}
        q0 = [initial_positions[m] for m in MOTOR_IDS]
        assessment = classify_startup_state(q0, history_samples=[q0])
        if not assessment.is_valid or not assessment.is_home:
            raise RuntimeError(f"R3 active repeat must start from VALID_HOME, got {assessment.classification.value}: {assessment.description}")
        if any(initial_faults[m] != 0 for m in MOTOR_IDS):
            raise RuntimeError(f"nonzero fault at active preflight: {initial_faults}")
        transport.configure_servo(speed=50)
        shaper.reset_state([(initial_positions[m] - 1000) * 0.000025 for m in MOTOR_IDS])
        latest_steps = dict(initial_positions)
        latest_currents = dict(initial_currents)
        latest_faults = dict(initial_faults)
        cycle_seq = 0
        next_deadline_ns = time.monotonic_ns()

        def cycle(target_pose: list[float], phase: str) -> None:
            nonlocal cycle_seq, next_deadline_ns
            t_start = time.monotonic_ns()
            ik_deltas = kin.differential_inverse_kinematics(target_pose, home_pose)
            shaped = shaper.update(ik_deltas, dt=CYCLE_DT)
            targets = [int(round(1000 + dq * 40000.0)) for dq in shaped]
            for val in targets:
                if not (200 <= val <= 1800):
                    raise RuntimeError(f"target step {val} outside J8 active envelope [200,1800]")
            servo_audit = transport.send_cycle(targets, cycle_id=cycle_seq, target_timestamp_ns=t_start)
            missing = 6 - len(servo_audit.feedbacks)
            feedback_audit = feedback.ingest_cycle_feedback(servo_audit.feedbacks, servo_cycle_id=cycle_seq, servo_cycle_timestamp_ns=t_start)
            for sample in feedback_audit.samples:
                if sample.timeout:
                    raise RuntimeError(f"feedback timeout on ID{sample.motor_id} at cycle {cycle_seq}")
                if sample.fault not in (None, 0):
                    raise RuntimeError(f"fault on ID{sample.motor_id}: {sample.fault}")
                if sample.current_mA is not None and sample.current_mA > 1500:
                    raise RuntimeError(f"overcurrent on ID{sample.motor_id}: {sample.current_mA} mA")
                if sample.position is not None:
                    latest_steps[sample.motor_id] = int(sample.position)
                latest_currents[sample.motor_id] = int(sample.current_mA or 0)
                latest_faults[sample.motor_id] = int(sample.fault or 0)
            t_end = int(feedback_audit.transaction_end_ns or time.monotonic_ns())
            batch_ms = (servo_audit.cycle_end_ns - servo_audit.cycle_start_ns) / 1e6
            pub.publish(cycle_seq, int(t_start), t_end, [latest_steps[m] for m in MOTOR_IDS], [latest_currents[m] for m in MOTOR_IDS], [latest_faults[m] for m in MOTOR_IDS], batch_ms, bool(servo_audit.deadline_miss), int(missing))
            records.append({
                "cycle_seq": int(cycle_seq),
                "phase": phase,
                "t_cycle_start_ns": int(t_start),
                "t_cycle_end_ns": int(t_end),
                "target_pose_xyz_rpy": [float(v) for v in target_pose],
                "target_steps": [int(v) for v in targets],
                "latest_steps": [int(latest_steps[m]) for m in MOTOR_IDS],
                "latest_currents_ma": [int(latest_currents[m]) for m in MOTOR_IDS],
                "latest_faults": [int(latest_faults[m]) for m in MOTOR_IDS],
                "batch_ms": float(batch_ms),
                "deadline_miss": bool(servo_audit.deadline_miss),
                "missing_0x29_feedback": int(missing),
            })
            cycle_seq += 1
            next_deadline_ns += int(CYCLE_DT * 1e9)
            sleep_ns = next_deadline_ns - time.monotonic_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

        next_deadline_ns = time.monotonic_ns()
        for seg in r3_segments(run_index):
            seg_start = time.monotonic_ns()
            print(f"[R3-{run_index}] Segment {seg.name}: {seg.duration_s:.1f}s peak_rates={peak_rates_rad_s(seg)}", flush=True)
            n_cycles = int(round(seg.duration_s * CYCLE_HZ))
            for k in range(n_cycles):
                t_s = k * CYCLE_DT
                r, p, y = seg.target_rpy_rad(t_s)
                cycle([0.0, 0.0, z_nom, r, p, y], seg.name)
            seg_end = time.monotonic_ns()
            segments_out.append(seg.jsonable(seg_start, seg_end))

        print(f"[R3-{run_index}] Sending clean stop_all after active repeat", flush=True)
        stop_audit = stop_adapter.stop_all("j9_e1b_r3_repeat_complete")
        stop_success = bool(stop_audit.success)
        final_scan = feedback.full_scan(servo_cycle_id=cycle_seq + 1, servo_cycle_timestamp_ns=time.monotonic_ns())
        final_positions = [int(s.position) if s.position is not None else -999999 for s in final_scan.samples]
        motor_summary = {
            "initial_positions_steps": q0,
            "final_positions_steps": final_positions,
            "final_home_error_steps_max": int(max(abs(v - 1000) for v in final_positions)) if final_positions else 999999,
            "cycles": int(len(records)),
            "hardware_write_count_estimate_0x29": int(len(records) * 6),
            "hardware_write_count_estimate_config_mode": 6,
            "hardware_write_count_estimate_stop_0x1a": 6,
            "fault_count": int(sum(1 for r in records for f in r["latest_faults"] if f != 0)),
            "missing_0x29_feedback_count": int(sum(r["missing_0x29_feedback"] for r in records)),
            "deadline_miss_count": int(sum(1 for r in records if r["deadline_miss"])),
            "batch_ms_mean": float(np.mean([r["batch_ms"] for r in records])) if records else 0.0,
            "batch_ms_p95": float(np.percentile([r["batch_ms"] for r in records], 95.0)) if records else 0.0,
            "batch_ms_max": float(np.max([r["batch_ms"] for r in records])) if records else 0.0,
            "stop_success": stop_success,
        }
    except Exception as exc:
        status = "FAIL"
        error = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        if stop_adapter is not None:
            try:
                stop_audit = stop_adapter.stop_all("j9_e1b_r3_exception")
                stop_success = bool(stop_audit.success)
            except Exception:
                stop_success = False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        pub.close()
        time.sleep(1.0)
        stop_proc(h30)
        stop_proc(fk)
        stop_proc(recorder)

    return {
        "run_index": run_index,
        "status": status,
        "error": error,
        "segments": segments_out,
        "motor_summary": motor_summary,
        "records": records,
        "process_logs": {k.lower(): v[-200:] for k, v in logs.items()},
    }


def tid_stats(values: list[int]) -> dict[str, Any]:
    if len(values) < 2:
        return {"count": len(values), "missing": 0, "duplicates": 0, "out_of_order": 0, "resets": 0, "wraps": 0, "max_delta": None}
    missing = duplicates = out = resets = wraps = 0
    max_delta = -10**9
    last = int(values[0])
    for raw in values[1:]:
        tid = int(raw)
        if tid == last:
            duplicates += 1
            delta = 0
        elif tid > last:
            delta = tid - last
            if delta > 1:
                missing += delta - 1
        else:
            if last >= 59000 and tid <= 1000:
                wraps += 1
                delta = 60000 - last + tid
                if delta > 1:
                    missing += delta - 1
            else:
                resets += 1
                out += 1
                delta = tid - last
        max_delta = max(max_delta, delta)
        last = tid
    return {"count": len(values), "missing": int(missing), "duplicates": int(duplicates), "out_of_order": int(out), "resets": int(resets), "wraps": int(wraps), "max_delta": int(max_delta)}


def sequence_stats(values: list[int], step: int = 1) -> dict[str, Any]:
    vals = np.asarray(values, dtype=np.int64)
    if len(vals) < 2:
        return {"count": int(len(vals)), "missing": 0, "duplicates": 0, "out_of_order": 0, "max_delta": None}
    diff = np.diff(vals)
    return {
        "count": int(len(vals)),
        "missing": int(np.sum(np.maximum(0, diff - step) // max(1, step))),
        "positive_gap_count": int(np.sum(diff > step)),
        "duplicates": int(np.sum(diff == 0)),
        "out_of_order": int(np.sum(diff < 0)),
        "min_delta": int(np.min(diff)),
        "p50_delta": float(np.percentile(diff, 50.0)),
        "p95_delta": float(np.percentile(diff, 95.0)),
        "p99_delta": float(np.percentile(diff, 99.0)),
        "max_delta": int(np.max(diff)),
    }


def h30_window_quality(rows: list[dict[str, Any]]) -> dict[str, Any]:
    failures: list[str] = []
    valid = [r for r in rows if r.get("has_sample_timestamp")]
    pub = sequence_stats([int(r["publisher_sequence"]) for r in valid], 1) if valid and all("publisher_sequence" in r for r in valid) else {"count": 0, "missing": 0, "duplicates": 0, "out_of_order": 0}
    tid = tid_stats([int(r["tid"]) for r in valid]) if valid and all("tid" in r for r in valid) else {"count": 0, "missing": 0, "duplicates": 0, "out_of_order": 0}
    sample_unwrapped, unwrap = cal.unwrap_u32_us([int(r["sample_timestamp_us"]) for r in valid]) if valid else (np.asarray([], dtype=float), {"wraps": 0, "resets": 0, "nonmonotonic": 0})
    if len(sample_unwrapped) >= 2:
        dt = np.diff(sample_unwrapped)
        sample = {
            "count": int(len(sample_unwrapped)),
            "missing": int(np.sum(np.maximum(0.0, dt - 5001.0) // 5000.0)),
            "duplicates": int(np.sum(dt == 0.0)),
            "out_of_order": int(np.sum(dt < 0.0)),
            "min_delta_us": float(np.min(dt)),
            "p95_delta_us": float(np.percentile(dt, 95.0)),
            "max_delta_us": float(np.max(dt)),
            "unwrap": unwrap,
        }
    else:
        sample = {"count": int(len(sample_unwrapped)), "missing": 0, "duplicates": 0, "out_of_order": 0, "unwrap": unwrap}
    for name, stats in [("publisher_sequence", pub), ("TID", tid), ("0x51", sample)]:
        for key in ["missing", "duplicates", "out_of_order"]:
            if int(stats.get(key, 0) or 0) != 0:
                failures.append(f"{name} {key}={stats.get(key)}")
    if int(sample["unwrap"].get("resets", 0)) or int(sample["unwrap"].get("nonmonotonic", 0)):
        failures.append(f"0x51 reset/nonmonotonic={sample['unwrap']}")
    return {"status": "PASS" if not failures else "DO_NOT_FIT", "failures": failures, "publisher_sequence": pub, "tid": tid, "sample_timestamp_0x51": sample, "rows": len(rows), "valid_sample_rows": len(valid)}


def attach_mapped_sample_times(rows: list[dict[str, Any]], affine: dict[str, float]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    valid_rows = [r for r in rows if r.get("has_sample_timestamp")]
    unwrapped, unwrap = cal.unwrap_u32_us([int(r["sample_timestamp_us"]) for r in valid_rows]) if valid_rows else (np.asarray([], dtype=float), {"wraps": 0, "resets": 0, "nonmonotonic": 0})
    mapped = cal.map_sample_timestamp_to_monotonic_ns(unwrapped, affine["a"], affine["b_us"]) if len(unwrapped) else np.asarray([], dtype=np.int64)
    out = []
    j = 0
    for r in rows:
        rr = dict(r)
        if r.get("has_sample_timestamp"):
            rr["mapped_sample_monotonic_ns"] = int(mapped[j])
            j += 1
        out.append(rr)
    return out, unwrap


def select_platform(rows: list[dict[str, Any]], start_ns: int, end_ns: int) -> list[dict[str, Any]]:
    return [r for r in rows if r.get("mapped_sample_monotonic_ns") is not None and start_ns <= int(r["mapped_sample_monotonic_ns"]) <= end_ns]


def select_fk(rows: list[dict[str, Any]], start_ns: int, end_ns: int) -> list[dict[str, Any]]:
    return [r for r in rows if start_ns <= int(r["query_monotonic_timestamp_ns"]) <= end_ns and r.get("converged", True)]


def residual_stats(residuals: np.ndarray) -> dict[str, Any]:
    norm = np.linalg.norm(residuals, axis=1)
    return {"count": int(len(norm)), "rms": float(np.sqrt(np.mean(norm ** 2))) if len(norm) else math.nan, "p95": float(np.percentile(norm, 95.0)) if len(norm) else math.nan, "max": float(np.max(norm)) if len(norm) else math.nan}


def evaluate_fixed_solution(platform_rows: list[dict[str, Any]], fk_rows: list[dict[str, Any]], affine: dict[str, float], rotation_matrix: list[list[float]], bias: list[float]) -> dict[str, Any]:
    imu_t, imu_g, _ = cal.parse_platform_timed_rows(platform_rows, affine["a"], affine["b_us"])
    fk_t, fk_rot = cal.parse_fk_rows(fk_rows)
    omega_fk, omega_valid, smoothing = cal.local_body_angular_velocity_from_rotations(fk_t, fk_rot, window_s=0.16, poly_degree=2)
    gyro_at_fk, interp_valid = cal.interpolate_gyro(imu_t, imu_g, fk_t)
    matched = omega_valid & interp_valid & np.all(np.isfinite(omega_fk), axis=1) & np.all(np.isfinite(gyro_at_fk), axis=1)
    R = Rotation.from_matrix(np.asarray(rotation_matrix, dtype=float))
    b = np.asarray(bias, dtype=float)
    residual = R.apply(gyro_at_fk[matched] - b) - omega_fk[matched]
    return {"matched_count": int(np.sum(matched)), "residual_rad_s": residual_stats(residual), "smoothing": smoothing}


def geodesic_deg(r1: np.ndarray, r2: np.ndarray) -> float:
    return float((Rotation.from_matrix(r1) * Rotation.from_matrix(r2).inv()).magnitude() * 180.0 / math.pi)


def analyze_repeat(run: dict[str, Any], recorder_data: dict[str, Any]) -> dict[str, Any]:
    platform_all = recorder_data.get("platform", [])
    fk_all = recorder_data.get("fk", [])
    valid = [r for r in platform_all if r.get("has_sample_timestamp")]
    unwrapped, unwrap = cal.unwrap_u32_us([int(r["sample_timestamp_us"]) for r in valid])
    host = np.asarray([int(r["host_monotonic_arrival_time_ns"]) for r in valid], dtype=np.int64)
    affine = cal.fit_affine_device_to_host(unwrapped, host)
    mapped_platform, full_unwrap = attach_mapped_sample_times(platform_all, affine)

    out: dict[str, Any] = {
        "run_index": run["run_index"],
        "clock_model": {"model": "host_monotonic_us = a * unwrapped_H30_0x51_sample_timestamp_us + b_us", "platform_affine": affine, "timestamp_unwrap": full_unwrap},
        "capture_counts": {
            "platform_timed": len(platform_all),
            "fk": len(fk_all),
            "joint_feedback": len(recorder_data.get("joint_feedback", [])),
            "platform_diagnostics": len(recorder_data.get("platform_diagnostics", [])),
        },
        "segments": [],
    }
    fit_result = None
    validation_result = None
    fit_seg_out = None
    val_seg_out = None
    for seg in run.get("segments", []):
        start = int(seg["start_monotonic_ns"])
        end = int(seg["end_monotonic_ns"])
        p_rows = select_platform(mapped_platform, start, end)
        f_rows = select_fk(fk_all, start, end)
        dq = h30_window_quality(p_rows)
        seg_out = {"segment": seg, "platform_count": len(p_rows), "fk_count": len(f_rows), "data_quality": dq}
        if seg["role"] == "combined_fit":
            if dq["status"] != "PASS":
                seg_out["fit"] = {"status": "DO_NOT_FIT", "reason": dq["failures"]}
            else:
                fit_result = cal.calibrate_capture(p_rows, f_rows, affine_a=affine["a"], affine_b_us=affine["b_us"], smoothing_window_s=0.16, poly_degree=2)
                seg_out["fit"] = fit_result
            fit_seg_out = seg_out
        elif seg["role"] == "withheld_validation":
            val_seg_out = seg_out
        out["segments"].append(seg_out)

    if fit_result is not None and fit_result.get("status") == "FIT_OK" and val_seg_out is not None:
        val_seg = val_seg_out["segment"]
        vp = select_platform(mapped_platform, int(val_seg["start_monotonic_ns"]), int(val_seg["end_monotonic_ns"]))
        vf = select_fk(fk_all, int(val_seg["start_monotonic_ns"]), int(val_seg["end_monotonic_ns"]))
        vdq = h30_window_quality(vp)
        if vdq["status"] != "PASS":
            validation_result = {"status": "DO_NOT_FIT", "data_quality": vdq}
        else:
            fixed_eval = evaluate_fixed_solution(vp, vf, affine, fit_result["extrinsic"]["rotation_matrix_R_P_IP"], fit_result["extrinsic"]["gyro_bias_ip_rad_s"])
            refit = cal.calibrate_capture(vp, vf, affine_a=affine["a"], affine_b_us=affine["b_us"], smoothing_window_s=0.16, poly_degree=2)
            delta = math.nan
            if refit.get("status") == "FIT_OK":
                delta = geodesic_deg(np.asarray(refit["extrinsic"]["rotation_matrix_R_P_IP"]), np.asarray(fit_result["extrinsic"]["rotation_matrix_R_P_IP"]))
            validation_result = {"status": "VALIDATION_OK" if refit.get("status") == "FIT_OK" else refit.get("status", "FAIL"), "data_quality": vdq, "fixed_solution_residual": fixed_eval, "independent_refit": refit, "rotation_geodesic_error_vs_fit_deg": float(delta)}
        val_seg_out["withheld_validation"] = validation_result

    out["fit_segment_result"] = fit_seg_out
    out["withheld_validation_result"] = validation_result
    return out


def write_report(path: Path, final: dict[str, Any]) -> None:
    lines = []
    lines.append("# J9-E1B-R3 Platform IMU Orientation Extrinsic Hardware Recalibration")
    lines.append("")
    lines.append(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S %z')}")
    lines.append(f"Status: **{final['r3_status']}**")
    lines.append(f"Robot extrinsics status: `{final['robot_extrinsics_status']}`")
    lines.append("")
    lines.append("## Evidence")
    for key, value in final.get("evidence_files", {}).items():
        lines.append(f"- {key}: `{value}`")
    lines.append("")
    lines.append("## Acceptance")
    for key, value in final.get("acceptance", {}).items():
        lines.append(f"- {key}: `{value}`")
    lines.append("")
    for rr in final.get("analysis", {}).get("repeats", []):
        idx = rr["run_index"]
        lines.append(f"## Repeat {idx}")
        aff = rr["clock_model"]["platform_affine"]
        lines.append(f"- affine a/b_us/ppm: `{aff['a']}` / `{aff['b_us']}` / `{aff['ppm_equivalent']}`")
        lines.append(f"- affine residual RMS/P95/max us: `{aff['residual_rms_us']}` / `{aff['residual_p95_abs_us']}` / `{aff['residual_max_abs_us']}`")
        fit = (rr.get("fit_segment_result") or {}).get("fit") or {}
        if fit.get("status") == "FIT_OK":
            ext = fit["extrinsic"]
            obs = fit["observability"]
            res = fit["residual_rad_s"]
            lines.append(f"- R_P_IP matrix: `{ext['rotation_matrix_R_P_IP']}`")
            lines.append(f"- quaternion_xyzw: `{ext['quaternion_xyzw_R_P_IP']}`")
            rpy = Rotation.from_matrix(np.asarray(ext["rotation_matrix_R_P_IP"])).as_euler("xyz", degrees=True).tolist()
            lines.append(f"- RPY xyz deg: `{rpy}`")
            lines.append(f"- gyro_bias_ip_rad_s: `{ext['gyro_bias_ip_rad_s']}`")
            lines.append(f"- rank/condition: `{obs['rank']}` / `{obs['condition_number']}`")
            lines.append(f"- singular_values: `{obs['singular_values']}`")
            lines.append(f"- fit residual RMS/P95/max rad_s: `{res['rms']}` / `{res['p95']}` / `{res['max']}`")
        else:
            lines.append(f"- fit status: `{fit.get('status', 'MISSING')}`")
        val = rr.get("withheld_validation_result") or {}
        if val:
            lines.append(f"- withheld status: `{val.get('status')}`")
            lines.append(f"- withheld fixed residual: `{(val.get('fixed_solution_residual') or {}).get('residual_rad_s')}`")
            lines.append(f"- validation refit delta deg: `{val.get('rotation_geodesic_error_vs_fit_deg')}`")
        lines.append("")
    cand = final.get("recommended_candidate")
    if cand:
        lines.append("## Recommended Candidate")
        lines.append(f"- derived_from: `{cand['derived_from']}`")
        lines.append(f"- R_P_IP matrix: `{cand['rotation_matrix_R_P_IP']}`")
        lines.append(f"- quaternion_xyzw: `{cand['quaternion_xyzw_R_P_IP']}`")
        lines.append(f"- RPY xyz deg: `{cand['rpy_xyz_deg_R_P_IP']}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-only", action="store_true")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--raw-out", type=Path, default=REPO_ROOT / "reports" / "j9_e1b_r3_platform_orientation_raw_20260828.json")
    parser.add_argument("--analysis-out", type=Path, default=REPO_ROOT / "reports" / "j9_e1b_r3_platform_orientation_analysis_20260828.json")
    parser.add_argument("--report-out", type=Path, default=REPO_ROOT / "reports" / "j9_e1b_r3_platform_orientation_recalibration_report_20260828.md")
    parser.add_argument("--timed-qos-depth", type=int, default=4096)
    parser.add_argument("--timed-qos-reliability", choices=["reliable", "best_effort"], default="reliable")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    args = parser.parse_args()
    if args.record_only:
        if args.out is None:
            raise SystemExit("--out is required with --record-only")
        return run_record_only(args.out, args.timed_qos_depth, args.timed_qos_reliability)
    if not (args.real_hardware and args.confirm_motion):
        raise SystemExit("R3 hardware recalibration requires --real-hardware --confirm-motion")

    evidence: dict[str, str] = {}
    raw: dict[str, Any] = {
        "metadata": {
            "stage": "J9-E1B-R3",
            "robot_extrinsics_status_initial": "UNCALIBRATED_UNSET",
            "scope": "R_P_IP only",
            "motor_port": MOTOR_PORT,
            "platform_imu_port": PLATFORM_IMU_PORT,
            "measurement_time": "H30 0x51 sample_timestamp_us mapped to Host CLOCK_MONOTONIC by per-repeat affine fit",
            "ros_callback_arrival_used_as_measurement_time": False,
            "lever_arm_estimated": False,
            "base_imu_calibrated": False,
            "fusion_or_stabilization": False,
        },
        "repeats": [],
        "read_only_audits": [],
        "home_results": [],
        "park_result": None,
        "final_read_only_audit": None,
    }

    try:
        for run_index in (1, 2):
            print(f"[R3] Repeat {run_index}: READ_ONLY audit", flush=True)
            audit = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
            raw["read_only_audits"].append({"run_index": run_index, "audit": audit, "hardware_write_count": 0})
            print(f"[R3] Repeat {run_index}: frozen HOME workflow", flush=True)
            home = run_servo_home_park("home", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
            raw["home_results"].append({"run_index": run_index, "home": asdict(home)})
            if not home.passed:
                raise RuntimeError(f"HOME failed before repeat {run_index}: {home.summary}")
            recorder_json = REPO_ROOT / "reports" / f"j9_e1b_r3_repeat{run_index}_recorder_20260828.json"
            print(f"[R3] Repeat {run_index}: active calibration trajectory and isolated recorder", flush=True)
            run = execute_active_repeat(run_index, recorder_json)
            evidence[f"repeat{run_index}_recorder"] = str(recorder_json)
            raw["repeats"].append({"run": run, "recorder_json": str(recorder_json), "recorder": json.loads(recorder_json.read_text(encoding="utf-8")) if recorder_json.exists() else {}})
            if run.get("status") != "PASS":
                raise RuntimeError(f"active repeat {run_index} failed: {run.get('error')}")

        print("[R3] Frozen PARK workflow", flush=True)
        park = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
        raw["park_result"] = asdict(park)
        print("[R3] Final READ_ONLY audit", flush=True)
        raw["final_read_only_audit"] = {"audit": run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0), "hardware_write_count": 0}
    finally:
        args.raw_out.write_text(json.dumps(raw, indent=2), encoding="utf-8")
        evidence["raw"] = str(args.raw_out)
        print(f"[WRITE] {args.raw_out}", flush=True)

    analysis_repeats = []
    for item in raw["repeats"]:
        analysis_repeats.append(analyze_repeat(item["run"], item["recorder"]))

    acceptance = {
        "full_rank_each_fit": False,
        "condition_improved_vs_84": False,
        "run1_vs_run2_delta_lt_0p5_deg": False,
        "validation_refit_delta_lt_1_deg_each": False,
        "data_quality_gates_pass": False,
        "motor_uart_safety_pass": False,
        "park_pass": bool((raw.get("park_result") or {}).get("passed")),
    }
    fits = []
    validation_deltas = []
    data_quality_pass = True
    motor_pass = True
    conditions = []
    for item, rr in zip(raw["repeats"], analysis_repeats):
        run = item["run"]
        ms = run.get("motor_summary", {})
        motor_pass = motor_pass and run.get("status") == "PASS" and ms.get("fault_count", 1) == 0 and ms.get("missing_0x29_feedback_count", 1) == 0 and ms.get("stop_success", False)
        fit = ((rr.get("fit_segment_result") or {}).get("fit") or {})
        if fit.get("status") == "FIT_OK":
            fits.append(fit)
            conditions.append(float(fit["observability"]["condition_number"]))
        else:
            data_quality_pass = False
        val = rr.get("withheld_validation_result") or {}
        if val.get("status") == "VALIDATION_OK":
            validation_deltas.append(float(val["rotation_geodesic_error_vs_fit_deg"]))
        else:
            data_quality_pass = False
        for seg in rr.get("segments", []):
            if seg.get("segment", {}).get("role") in ("combined_fit", "withheld_validation") and seg.get("data_quality", {}).get("status") != "PASS":
                data_quality_pass = False
    run_delta = math.inf
    if len(fits) == 2:
        run_delta = geodesic_deg(np.asarray(fits[0]["extrinsic"]["rotation_matrix_R_P_IP"]), np.asarray(fits[1]["extrinsic"]["rotation_matrix_R_P_IP"]))
    acceptance["full_rank_each_fit"] = len(fits) == 2 and all(int(f["observability"]["rank"]) == 6 for f in fits)
    acceptance["condition_improved_vs_84"] = len(conditions) == 2 and max(conditions) < 60.0
    acceptance["run1_vs_run2_delta_lt_0p5_deg"] = run_delta < 0.5
    acceptance["validation_refit_delta_lt_1_deg_each"] = len(validation_deltas) == 2 and max(validation_deltas) < 1.0
    acceptance["data_quality_gates_pass"] = data_quality_pass
    acceptance["motor_uart_safety_pass"] = motor_pass
    all_pass = all(bool(v) for v in acceptance.values())

    candidate = None
    if all_pass and len(fits) == 2:
        rotations = Rotation.from_matrix(np.asarray([f["extrinsic"]["rotation_matrix_R_P_IP"] for f in fits], dtype=float))
        mean_rot = rotations.mean()
        candidate = {
            "derived_from": "SO(3) geodesic mean of the two accepted R3 repeat fits",
            "rotation_matrix_R_P_IP": mean_rot.as_matrix().tolist(),
            "quaternion_xyzw_R_P_IP": mean_rot.as_quat().tolist(),
            "rpy_xyz_deg_R_P_IP": mean_rot.as_euler("xyz", degrees=True).tolist(),
        }

    final = {
        "stage": "J9-E1B-R3",
        "r3_status": "PASS" if all_pass else "FAIL",
        "robot_extrinsics_status": "CALIBRATED_CANDIDATE" if all_pass else "UNCALIBRATED_UNSET",
        "acceptance": acceptance,
        "run1_vs_run2_rotation_delta_deg": run_delta,
        "validation_refit_deltas_deg": validation_deltas,
        "recommended_candidate": candidate,
        "analysis": {"repeats": analysis_repeats},
        "evidence_files": evidence,
    }
    args.analysis_out.write_text(json.dumps(final, indent=2), encoding="utf-8")
    evidence["analysis"] = str(args.analysis_out)
    write_report(args.report_out, final)
    print(json.dumps({"r3_status": final["r3_status"], "robot_extrinsics_status": final["robot_extrinsics_status"], "acceptance": acceptance, "run_delta_deg": run_delta, "validation_deltas_deg": validation_deltas}, indent=2), flush=True)
    print(f"[WRITE] {args.analysis_out}", flush=True)
    print(f"[WRITE] {args.report_out}", flush=True)
    return 0 if all_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
