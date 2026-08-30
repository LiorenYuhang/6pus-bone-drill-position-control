#!/usr/bin/env python3
"""J9-E1B-R4 active-load runtime isolation and lossless capture qualification.

This is a qualification harness only. It does not estimate extrinsics and does
not modify the frozen J8 control implementation. During motion, recorder
callbacks enqueue compact rows only; CSV parsing and JSON/report generation are
deferred until after the mechanism is safely parked.
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

import numpy as np

SCRIPT = Path(__file__).resolve()
SRC_ROOT = SCRIPT.parents[2]
WS_ROOT = SCRIPT.parents[3]
REPO_ROOT = WS_ROOT.parent
sys.path.insert(0, str(SCRIPT.parent))
for pkg in ["h30_imu", "pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = WS_ROOT / "src" / pkg
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from h30_imu.msg import TimedImu
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

import j9_e1b_r3_platform_orientation_hardware_recalibration as r3
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.j7_real_uart_adapter import J7StopAdapter, J7UartAdapter
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
from pus6_motor_driver.safety import MOTOR_IDS, classify_startup_state
from run_j7_servo_home_park import run_servo_home_park
from run_j8_c_hardware_readonly_audit import run_j8_c_audit

MOTOR_PORT = r3.MOTOR_PORT
PLATFORM_IMU_PORT = r3.PLATFORM_IMU_PORT
BAUD_MOTOR = r3.BAUD_MOTOR
BAUD_IMU = r3.BAUD_IMU
CYCLE_HZ = 50.0
CYCLE_DT = 1.0 / CYCLE_HZ


@dataclass
class R4Segment:
    name: str
    role: str
    duration_s: float
    amp_deg: tuple[float, float, float]
    freq_hz: tuple[float, float, float]
    phase_deg: tuple[float, float, float]
    ramp_s: float = 3.0

    def target_rpy_rad(self, t_s: float) -> tuple[float, float, float]:
        env = r3.envelope(t_s, self.duration_s, self.ramp_s)
        return tuple(
            env * math.radians(a) * math.sin(2.0 * math.pi * f * t_s + math.radians(ph))
            for a, f, ph in zip(self.amp_deg, self.freq_hz, self.phase_deg)
        )

    def jsonable(self, start_ns: int, end_ns: int) -> dict[str, Any]:
        return {
            "name": self.name,
            "role": self.role,
            "duration_s": self.duration_s,
            "amp_deg": list(self.amp_deg),
            "freq_hz": list(self.freq_hz),
            "phase_deg": list(self.phase_deg),
            "ramp_s": self.ramp_s,
            "start_monotonic_ns": int(start_ns),
            "end_monotonic_ns": int(end_ns),
        }


def qualification_segments(run_index: int) -> list[R4Segment]:
    return [
        R4Segment(f"run{run_index}_stationary_preroll", "stationary", 8.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0),
        R4Segment(f"run{run_index}_roll", "roll", 16.0, (2.0, 0.0, 0.0), (0.20, 0.0, 0.0), (0.0, 0.0, 0.0)),
        R4Segment(f"run{run_index}_pitch", "pitch", 16.0, (0.0, 2.0, 0.0), (0.0, 0.17, 0.0), (0.0, 90.0, 0.0)),
        R4Segment(f"run{run_index}_yaw", "yaw", 18.0, (0.0, 0.0, 3.0), (0.0, 0.0, 0.13), (0.0, 0.0, 45.0)),
        R4Segment(f"run{run_index}_combined", "combined", 24.0, (2.0, 2.0, 3.0), (0.20, 0.17, 0.13), (0.0, 90.0, 45.0)),
        R4Segment(f"run{run_index}_home_return", "home_return", 8.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0),
    ]


def peak_rates(segment: R4Segment) -> list[float]:
    return [abs(math.radians(a) * 2.0 * math.pi * f) for a, f in zip(segment.amp_deg, segment.freq_hz)]


class CsvQueueWriter:
    def __init__(self, out_dir: Path, capacity: int) -> None:
        self.queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue(maxsize=capacity)
        self.out_dir = out_dir
        self.capacity = capacity
        self.stop_requested = False
        self.thread = threading.Thread(target=self._run, name="r4-recorder-writer", daemon=True)
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}
        self.written: dict[str, int] = {}
        self.overflow: dict[str, int] = {}
        self.errors = 0
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
        self.thread.join(timeout=10.0)
        for f in self.files.values():
            f.flush()
            f.close()


class R4Recorder(Node):
    def __init__(self, out_dir: Path, qos_depth: int, reliability: str, queue_capacity: int) -> None:
        super().__init__("j9_e1b_r4_isolated_recorder")
        self.out_dir = out_dir
        self.writer = CsvQueueWriter(out_dir, queue_capacity)
        rel = ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
        imu_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=qos_depth, reliability=rel, durability=DurabilityPolicy.VOLATILE)
        diag_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.callback_count: dict[str, int] = {}
        self.callback_duration_ns: dict[str, list[int]] = {}
        self._subscription_handles = [
            self.create_subscription(TimedImu, "/pus6/imu_platform/timed", lambda m: self._imu(m), imu_qos),
            self.create_subscription(DiagnosticArray, "/pus6/imu_platform/diagnostics", lambda m: self._diag(m), diag_qos),
            self.create_subscription(FkStateStamped, "/pus6/fk/state", lambda m: self._fk(m), qos_profile_sensor_data),
            self.create_subscription(TimedJointFeedback, "/pus6/joint_feedback/timed", lambda m: self._feedback(m), qos_profile_sensor_data),
            self.create_subscription(Float64MultiArray, "/pus6/driver_diagnostics", lambda m: self._driver(m), qos_profile_sensor_data),
        ]

    @staticmethod
    def stamp_ns(msg: Any) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _record(self, stream: str, row: dict[str, Any], start_ns: int) -> None:
        self.callback_count[stream] = self.callback_count.get(stream, 0) + 1
        self.callback_duration_ns.setdefault(stream, []).append(time.monotonic_ns() - start_ns)
        self.writer.enqueue(stream, row)

    def _imu(self, msg: TimedImu) -> None:
        start = time.monotonic_ns()
        self._record("imu", {
            "callback_monotonic_ns": start,
            "ros_header_stamp_ns": self.stamp_ns(msg),
            "publisher_sequence": int(msg.publisher_sequence),
            "tid": int(msg.tid),
            "has_sample_timestamp": int(msg.has_sample_timestamp),
            "sample_timestamp_us": int(msg.sample_timestamp_us),
            "has_data_ready_timestamp": int(msg.has_data_ready_timestamp),
            "data_ready_timestamp_us": int(msg.data_ready_timestamp_us),
            "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
            "gx_rad_s": float(msg.imu.angular_velocity.x),
            "gy_rad_s": float(msg.imu.angular_velocity.y),
            "gz_rad_s": float(msg.imu.angular_velocity.z),
        }, start)

    def _diag(self, msg: DiagnosticArray) -> None:
        start = time.monotonic_ns()
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            level = status.level
            if isinstance(level, (bytes, bytearray)):
                level = int.from_bytes(level, "little", signed=False)
            self._record("imu_diagnostics", {
                "callback_monotonic_ns": start,
                "ros_header_stamp_ns": self.stamp_ns(msg),
                "name": status.name,
                "hardware_id": status.hardware_id,
                "level": int(level),
                "message": status.message,
                "values_json": json.dumps(values, separators=(",", ":")),
            }, start)

    def _fk(self, msg: FkStateStamped) -> None:
        start = time.monotonic_ns()
        self._record("fk", {
            "callback_monotonic_ns": start,
            "ros_header_stamp_ns": self.stamp_ns(msg),
            "query_monotonic_timestamp_ns": int(msg.query_monotonic_timestamp_ns),
            "converged": int(msg.converged),
            "residual_norm_meters": float(msg.residual_norm_meters),
            "compute_time_us": float(msg.compute_time_us),
            "pose_quaternion_xyzw_json": json.dumps([float(msg.pose.orientation.x), float(msg.pose.orientation.y), float(msg.pose.orientation.z), float(msg.pose.orientation.w)]),
        }, start)

    def _feedback(self, msg: TimedJointFeedback) -> None:
        start = time.monotonic_ns()
        self._record("joint_feedback", {
            "callback_monotonic_ns": start,
            "ros_header_stamp_ns": self.stamp_ns(msg),
            "snapshot_start_monotonic_ns": int(msg.snapshot_start_monotonic_ns),
            "snapshot_end_monotonic_ns": int(msg.snapshot_end_monotonic_ns),
            "uart_cycle_sequence": int(msg.uart_cycle_sequence),
            "fault_codes_json": json.dumps([int(v) for v in msg.fault_codes]),
        }, start)

    def _driver(self, msg: Float64MultiArray) -> None:
        start = time.monotonic_ns()
        self._record("driver_diagnostics", {
            "callback_monotonic_ns": start,
            "data_json": json.dumps([float(v) for v in msg.data], separators=(",", ":")),
        }, start)

    def summary(self) -> dict[str, Any]:
        callback = {}
        for stream, values in self.callback_duration_ns.items():
            arr = np.asarray(values, dtype=np.float64) / 1e3
            callback[stream] = {
                "count": int(len(arr)),
                "mean_us": float(np.mean(arr)) if len(arr) else 0.0,
                "p95_us": float(np.percentile(arr, 95.0)) if len(arr) else 0.0,
                "max_us": float(np.max(arr)) if len(arr) else 0.0,
            }
        return {
            "callback_count": self.callback_count,
            "writer_count": self.writer.written,
            "queue_overflow": self.writer.overflow,
            "writer_errors": self.writer.errors,
            "queue_capacity": self.writer.capacity,
            "callback_timing": callback,
        }


def recorder_main(args: argparse.Namespace) -> int:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = R4Recorder(args.out_dir, args.qos_depth, args.reliability, args.recorder_queue_capacity)
    executor = MultiThreadedExecutor(num_threads=args.recorder_threads)
    executor.add_node(node)
    stopped = threading.Event()

    def stop_handler(_signum: int, _frame: Any) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while not stopped.is_set():
            time.sleep(0.1)
    finally:
        try:
            executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        node.writer.stop()
        node.out_dir.joinpath("summary.json").write_text(json.dumps(node.summary(), indent=2), encoding="utf-8")
    return 0


def process_snapshot(pid: int) -> dict[str, Any]:
    out: dict[str, Any] = {"pid": pid, "host_monotonic_ns": time.monotonic_ns()}
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        out["state"] = fields[2]
        out["utime_ticks"] = int(fields[13])
        out["stime_ticks"] = int(fields[14])
        out["rss_pages"] = int(fields[23])
    except Exception as exc:
        out["error"] = str(exc)
    try:
        io = {}
        for line in Path(f"/proc/{pid}/io").read_text().splitlines():
            key, value = line.split(":", 1)
            io[key.strip()] = int(value.strip())
        out["io"] = io
    except Exception:
        out["io"] = {}
    return out


def start_h30(source_capture_path: Path) -> subprocess.Popen[str]:
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
        "-p", f"source_capture_path:={source_capture_path}",
        "-p", "source_capture_queue_capacity:=131072",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def start_fk() -> subprocess.Popen[str]:
    return subprocess.Popen([
        "ros2", "run", "pus6_kinematics", "fk_node", "--ros-args",
        "-p", "fixed_lag_delay_ms:=160.0",
        "-p", "query_rate_hz:=200.0",
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def start_recorder(out_dir: Path, threads: int, queue_capacity: int) -> subprocess.Popen[str]:
    return subprocess.Popen([
        sys.executable, str(SCRIPT), "--recorder-only",
        "--out-dir", str(out_dir), "--recorder-threads", str(threads),
        "--recorder-queue-capacity", str(queue_capacity),
        "--qos-depth", "4096", "--reliability", "reliable",
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def drain_log(proc: subprocess.Popen[str], label: str, log: list[str]) -> None:
    if proc.stdout is None:
        return
    for line in proc.stdout:
        text = line.rstrip()
        log.append(text)
        if any(token in text.lower() for token in ("opened h30", "cannot open", "error", "warn", "deadline")):
            print(f"[{label}] {text}", flush=True)


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


def sequence_stats(values: list[int], expected_step: int = 1) -> dict[str, Any]:
    if len(values) < 2:
        return {"count": len(values), "missing": 0, "duplicates": 0, "out_of_order": 0, "max_delta": None}
    d = np.diff(np.asarray(values, dtype=np.int64))
    return {
        "count": int(len(values)),
        "missing": int(np.sum(np.maximum(0, d - expected_step) // max(1, expected_step))),
        "duplicates": int(np.sum(d == 0)),
        "out_of_order": int(np.sum(d < 0)),
        "max_delta": int(np.max(d)),
        "p95_delta": float(np.percentile(d, 95.0)),
    }


def load_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def active_rows(rows: list[dict[str, str]], start_ns: int, end_ns: int, key: str) -> list[dict[str, str]]:
    return [r for r in rows if start_ns <= int(float(r.get(key, 0))) <= end_ns]


def diag_int(rows: list[dict[str, str]], key: str) -> int | None:
    if not rows:
        return None
    try:
        values = json.loads(rows[-1]["values_json"])
        return int(values[key])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def analyze_capture(run: dict[str, Any], capture_dir: Path) -> dict[str, Any]:
    source = load_csv(capture_dir.parent / "source.csv")
    imu = load_csv(capture_dir / "imu.csv")
    fk = load_csv(capture_dir / "fk.csv")
    feedback = load_csv(capture_dir / "joint_feedback.csv")
    diags = load_csv(capture_dir / "imu_diagnostics.csv")
    summary = json.loads((capture_dir / "summary.json").read_text()) if (capture_dir / "summary.json").exists() else {}
    out: dict[str, Any] = {
        "counts": {
            "source_rows": len(source),
            "timed_imu_subscriber_callbacks": len(imu),
            "timed_imu_recorder_rows": int(summary.get("writer_count", {}).get("imu", len(imu))),
            "fk_rows": len(fk),
            "joint_feedback_rows": len(feedback),
            "diagnostic_rows": len(diags),
        },
        "recorder": summary,
        "windows": [],
    }
    for seg in run["segments"]:
        start = int(seg["start_monotonic_ns"])
        end = int(seg["end_monotonic_ns"])
        source_rows = active_rows(source, start, end, "host_monotonic_arrival_time_ns")
        imu_rows = active_rows(imu, start, end, "callback_monotonic_ns")
        fk_rows = active_rows(fk, start, end, "query_monotonic_timestamp_ns")
        fb_rows = active_rows(feedback, start, end, "snapshot_start_monotonic_ns")
        source_diag = sequence_stats([int(r["publisher_sequence"]) for r in source_rows])
        pub_diag = sequence_stats([int(r["publisher_sequence"]) for r in imu_rows])
        tid_diag = sequence_stats([int(r["tid"]) for r in source_rows])
        sample_diag = sequence_stats([int(r["sample_timestamp_us"]) for r in source_rows], 5000)
        fk_time_diag = sequence_stats([int(r["query_monotonic_timestamp_ns"]) for r in fk_rows], 5_000_000)
        joint_diag = sequence_stats([int(r["uart_cycle_sequence"]) for r in fb_rows])
        window = {
            "segment": seg,
            "source_rows": len(source_rows),
            "subscriber_rows": len(imu_rows),
            "fk_rows": len(fk_rows),
            "joint_feedback_rows": len(fb_rows),
            "source_publisher_sequence": source_diag,
            "subscriber_publisher_sequence": pub_diag,
            "source_tid": tid_diag,
            "source_0x51_sample_timestamp": sample_diag,
            "fk_query_timing": fk_time_diag,
            "joint_feedback_sequence": joint_diag,
            "diagnostics_last": {
                "valid_frames": diag_int(diags, "valid_frames"),
                "true_tid_missing_frames": diag_int(diags, "true_tid_missing_frames"),
                "sample_timestamp_missing_samples": diag_int(diags, "sample_timestamp_missing_samples"),
                "published_timed_imu_count": diag_int(diags, "published_timed_imu_count"),
                "source_capture_enqueued_rows": diag_int(diags, "source_capture_enqueued_rows"),
                "source_capture_written_rows": diag_int(diags, "source_capture_written_rows"),
                "source_capture_overflow_drops": diag_int(diags, "source_capture_overflow_drops"),
                "source_capture_writer_errors": diag_int(diags, "source_capture_writer_errors"),
                "source_valid_frames": diag_int(diags, "valid_frames"),
            },
        }
        out["windows"].append(window)
    return out


def execute_repeat(run_index: int, report_dir: Path) -> dict[str, Any]:
    run_dir = report_dir / f"r4_repeat{run_index}"
    run_dir.mkdir(parents=True, exist_ok=True)
    source_path = run_dir / "source.csv"
    recorder_dir = run_dir / "recorder"
    recorder_dir.mkdir(exist_ok=True)
    recorder = start_recorder(recorder_dir, threads=2, queue_capacity=131072)
    h30 = start_h30(source_path)
    fk = start_fk()
    logs: dict[str, list[str]] = {"REC": [], "H30": [], "FK": []}
    for proc, label in ((recorder, "REC"), (h30, "H30"), (fk, "FK")):
        threading.Thread(target=drain_log, args=(proc, label, logs[label]), daemon=True).start()
    snapshots: list[dict[str, Any]] = []
    processes = [os.getpid(), recorder.pid, h30.pid, fk.pid]
    stop_event = threading.Event()
    monitor_thread = threading.Thread(
        target=lambda: monitor_processes(processes, snapshots, stop_event), daemon=True
    )
    monitor_thread.start()
    time.sleep(4.0)
    if recorder.poll() is not None:
        raise RuntimeError(f"isolated recorder exited before motion with code {recorder.returncode}")
    if h30.poll() is not None:
        raise RuntimeError(f"H30 process exited before motion with code {h30.returncode}")
    if fk.poll() is not None:
        raise RuntimeError(f"FK process exited before motion with code {fk.returncode}")

    pub = r3.RosFeedbackPublisher()
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    home_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]
    shaper = StatefulJerkLimitedShaper(max_speed_m_per_sec=0.005, max_accel_m_per_sec2=0.050, max_jerk_m_per_sec3=0.500, omega_n=20.0)
    records: list[dict[str, Any]] = []
    segments: list[dict[str, Any]] = []
    status = "PASS"
    error: str | None = None
    stop_success = False
    motor_summary: dict[str, Any] = {}
    cycle_seq = 0
    max_cycle_gap_ms = 0.0
    adapter = None
    stop_adapter = None
    ser = None
    try:
        import serial
        ser = serial.Serial(MOTOR_PORT, baudrate=BAUD_MOTOR, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.015, write_timeout=0.015)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        adapter = J7UartAdapter(ser, baudrate=BAUD_MOTOR, timeout_s=0.015, min_frame_interval_s=0.0005)
        stop_adapter = J7StopAdapter(adapter)
        transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0, rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX, min_frame_interval_s=0.0005, pause_callback=stop_adapter)
        feedback = J7ServoFeedbackScheduler(transport, full_scan_period_s=1e9, feedback_budget_s=0.010, timeout_escalation_misses=4)
        pre = feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
        if pre.deferred or len(pre.samples) != 6:
            raise RuntimeError("R4 preflight full scan incomplete")
        initial = {s.motor_id: int(s.position) for s in pre.samples if s.position is not None}
        faults = {s.motor_id: int(s.fault or 0) for s in pre.samples}
        q0 = [initial[m] for m in MOTOR_IDS]
        assessment = classify_startup_state(q0, history_samples=[q0])
        if not assessment.is_valid or not assessment.is_home:
            raise RuntimeError(f"R4 requires VALID_HOME, got {assessment.classification.value}")
        if any(faults.values()):
            raise RuntimeError(f"nonzero preflight fault: {faults}")
        transport.configure_servo(speed=50)
        shaper.reset_state([(initial[m] - 1000) * 0.000025 for m in MOTOR_IDS])
        latest_steps = dict(initial)
        latest_currents = {m: 0 for m in MOTOR_IDS}
        latest_faults = {m: 0 for m in MOTOR_IDS}
        next_deadline_ns = time.monotonic_ns()
        previous_cycle_start_ns: int | None = None
        def cycle(target_pose: list[float], phase: str) -> None:
            nonlocal cycle_seq, next_deadline_ns, previous_cycle_start_ns, max_cycle_gap_ms
            t_start = time.monotonic_ns()
            if previous_cycle_start_ns is not None:
                max_cycle_gap_ms = max(max_cycle_gap_ms, (t_start - previous_cycle_start_ns) / 1e6)
            previous_cycle_start_ns = t_start
            ik_deltas = kin.differential_inverse_kinematics(target_pose, home_pose)
            shaped = shaper.update(ik_deltas, dt=CYCLE_DT)
            targets = [int(round(1000 + dq * 40000.0)) for dq in shaped]
            if any(not (200 <= val <= 1800) for val in targets):
                raise RuntimeError(f"target outside J8 envelope: {targets}")
            servo_audit = transport.send_cycle(targets, cycle_id=cycle_seq, target_timestamp_ns=t_start)
            missing = 6 - len(servo_audit.feedbacks)
            feedback_audit = feedback.ingest_cycle_feedback(servo_audit.feedbacks, servo_cycle_id=cycle_seq, servo_cycle_timestamp_ns=t_start)
            for sample in feedback_audit.samples:
                if sample.timeout:
                    raise RuntimeError(f"feedback timeout ID{sample.motor_id} cycle {cycle_seq}")
                if sample.fault not in (None, 0):
                    raise RuntimeError(f"fault ID{sample.motor_id}: {sample.fault}")
                if sample.current_mA is not None and sample.current_mA > 1500:
                    raise RuntimeError(f"overcurrent ID{sample.motor_id}: {sample.current_mA}")
                if sample.position is not None:
                    latest_steps[sample.motor_id] = int(sample.position)
                latest_currents[sample.motor_id] = int(sample.current_mA or 0)
                latest_faults[sample.motor_id] = int(sample.fault or 0)
            t_end = int(feedback_audit.transaction_end_ns or time.monotonic_ns())
            batch_ms = (servo_audit.cycle_end_ns - servo_audit.cycle_start_ns) / 1e6
            pub.publish(cycle_seq, t_start, t_end, [latest_steps[m] for m in MOTOR_IDS], [latest_currents[m] for m in MOTOR_IDS], [latest_faults[m] for m in MOTOR_IDS], batch_ms, bool(servo_audit.deadline_miss), int(missing))
            records.append({
                "cycle_seq": cycle_seq,
                "phase": phase,
                "t_cycle_start_ns": t_start,
                "t_cycle_end_ns": t_end,
                "batch_ms": batch_ms,
                "deadline_miss": bool(servo_audit.deadline_miss),
                "missing_0x29_feedback": missing,
                "faults": list(latest_faults.values()),
            })
            cycle_seq += 1
            next_deadline_ns += int(CYCLE_DT * 1e9)
            remaining = next_deadline_ns - time.monotonic_ns()
            if remaining > 0:
                time.sleep(remaining / 1e9)

        for seg in qualification_segments(run_index):
            start = time.monotonic_ns()
            print(f"[R4-{run_index}] {seg.name} peak_rates={peak_rates(seg)}", flush=True)
            for k in range(int(round(seg.duration_s * CYCLE_HZ))):
                r, p, y = seg.target_rpy_rad(k * CYCLE_DT)
                cycle([0.0, 0.0, z_nom, r, p, y], seg.name)
            end = time.monotonic_ns()
            segments.append(seg.jsonable(start, end))
        stop_audit = stop_adapter.stop_all(f"j9_e1b_r4_repeat{run_index}_complete")
        stop_success = bool(stop_audit.success)
        final_scan = feedback.full_scan(servo_cycle_id=cycle_seq + 1, servo_cycle_timestamp_ns=time.monotonic_ns())
        final_positions = [int(s.position) if s.position is not None else -999999 for s in final_scan.samples]
        motor_summary = {
            "initial_positions_steps": q0,
            "final_positions_steps": final_positions,
            "cycles": len(records),
            "hardware_write_count_estimate_0x29": len(records) * 6,
            "hardware_write_count_estimate_config_mode": 6,
            "hardware_write_count_estimate_stop_0x1a": 6,
            "missing_0x29_feedback_count": int(sum(r["missing_0x29_feedback"] for r in records)),
            "deadline_miss_count": int(sum(bool(r["deadline_miss"]) for r in records)),
            "fault_count": int(sum(1 for r in records for f in r["faults"] if f)),
            "batch_ms_mean": float(np.mean([r["batch_ms"] for r in records])),
            "batch_ms_p95": float(np.percentile([r["batch_ms"] for r in records], 95.0)),
            "batch_ms_max": float(np.max([r["batch_ms"] for r in records])),
            "max_cycle_gap_ms": max_cycle_gap_ms,
            "stop_success": stop_success,
        }
    except Exception as exc:
        status = "FAIL"
        error = f"{type(exc).__name__}: {exc}"
        print(f"[R4-{run_index}] {error}", flush=True)
        attempted_cycles = int(cycle_seq + 1) if cycle_seq >= 0 else 0
        batch_values = [r["batch_ms"] for r in records]
        motor_summary = {
            "cycles_completed": int(len(records)),
            "cycles_attempted": attempted_cycles,
            "hardware_write_count_estimate_0x29": int(attempted_cycles * 6),
            "hardware_write_count_estimate_config_mode": 6,
            "hardware_write_count_estimate_stop_0x1a": 6,
            "missing_0x29_feedback_count": int(sum(r["missing_0x29_feedback"] for r in records) + (1 if "feedback timeout" in error else 0)),
            "deadline_miss_count": int(sum(bool(r["deadline_miss"]) for r in records)),
            "fault_count": int(sum(1 for r in records for f in r["faults"] if f)),
            "batch_ms_mean": float(np.mean(batch_values)) if batch_values else 0.0,
            "batch_ms_p95": float(np.percentile(batch_values, 95.0)) if batch_values else 0.0,
            "batch_ms_max": float(np.max(batch_values)) if batch_values else 0.0,
            "max_cycle_gap_ms": float(max_cycle_gap_ms),
            "stop_success": False,
            "failure_class": "feedback_timeout" if "feedback timeout" in error else "runtime_exception",
        }
        if stop_adapter is not None:
            try:
                stop_adapter.stop_all(f"j9_e1b_r4_repeat{run_index}_exception")
                stop_success = True
                motor_summary["stop_success"] = True
            except Exception:
                stop_success = False
    finally:
        stop_event.set()
        monitor_thread.join(timeout=2.0)
        if ser is not None:
            ser.close()
        pub.close()
        time.sleep(0.5)
        stop_proc(h30)
        stop_proc(fk)
        stop_proc(recorder)
    run = {
        "run_index": run_index,
        "status": status,
        "error": error,
        "segments": segments,
        "motor_summary": motor_summary,
        "records": records,
        "process_snapshots": snapshots,
        "process_pids": processes,
        "process_logs": {k.lower(): v[-100:] for k, v in logs.items()},
    }
    (run_dir / "run.json").write_text(json.dumps(run, indent=2), encoding="utf-8")
    run["capture_analysis"] = analyze_capture(run, recorder_dir)
    return run


def monitor_processes(pids: list[int | None], snapshots: list[dict[str, Any]], stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        snapshots.append({
            "loadavg": Path("/proc/loadavg").read_text().split()[:3] if Path("/proc/loadavg").exists() else [],
            "processes": [process_snapshot(int(pid)) for pid in pids if pid],
        })
        time.sleep(1.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--recorder-only", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "reports" / "j9_e1b_r4_active_load_qualification_20260828")
    parser.add_argument("--recorder-threads", type=int, default=2)
    parser.add_argument("--recorder-queue-capacity", type=int, default=131072)
    parser.add_argument("--qos-depth", type=int, default=4096)
    parser.add_argument("--reliability", choices=["reliable", "best_effort"], default="reliable")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    args = parser.parse_args()
    if args.recorder_only:
        return recorder_main(args)
    if not (args.real_hardware and args.confirm_motion):
        raise SystemExit("R4 requires --real-hardware --confirm-motion")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    final: dict[str, Any] = {
        "stage": "J9-E1B-R4",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "hardware_write_count": 0,
        "repeats": [],
        "read_only_audits": [],
        "home_results": [],
        "park_result": None,
        "final_read_only_audit": None,
    }
    try:
        for run_index in (1, 2):
            audit = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
            final["read_only_audits"].append({"run_index": run_index, "audit": audit, "hardware_write_count": 0})
            home = run_servo_home_park("home", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
            final["home_results"].append({"run_index": run_index, "home": asdict(home)})
            if not home.passed:
                raise RuntimeError(f"HOME failed before R4 repeat {run_index}")
            run = execute_repeat(run_index, args.out_dir)
            final["repeats"].append(run)
            if run["status"] != "PASS":
                raise RuntimeError(f"R4 repeat {run_index} failed: {run['error']}")
        park = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
        final["park_result"] = asdict(park)
        final["final_read_only_audit"] = {"audit": run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0), "hardware_write_count": 0}
    except Exception as exc:
        final["error"] = f"{type(exc).__name__}: {exc}"
        try:
            park = run_servo_home_park("park", port=MOTOR_PORT, baudrate=BAUD_MOTOR, speed=100.0, rate_hz=50.0, hold_duration_s=2.0, real_hardware=True, confirm_motion=True)
            final["park_result"] = asdict(park)
            final["final_read_only_audit"] = {"audit": run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0), "hardware_write_count": 0}
        except Exception as park_exc:
            final["park_error"] = f"{type(park_exc).__name__}: {park_exc}"

    repeats_pass = len(final["repeats"]) == 2 and all(r["status"] == "PASS" for r in final["repeats"])
    accepted_windows = []
    for run in final["repeats"]:
        for window in run.get("capture_analysis", {}).get("windows", []):
            if window["segment"]["role"] in ("roll", "pitch", "yaw", "combined"):
                accepted_windows.append(window)
    data_pass = bool(accepted_windows) and all(
        w["source_publisher_sequence"]["missing"] == 0
        and w["source_tid"]["missing"] == 0
        and w["source_0x51_sample_timestamp"]["missing"] == 0
        and w["subscriber_publisher_sequence"]["missing"] == 0
        and w["joint_feedback_sequence"]["missing"] == 0
        and w["fk_query_timing"]["missing"] == 0
        and w["fk_rows"] > 0
        for w in accepted_windows
    )
    motor_pass = repeats_pass and all(
        r["motor_summary"].get("deadline_miss_count", 1) == 0
        and r["motor_summary"].get("missing_0x29_feedback_count", 1) == 0
        and r["motor_summary"].get("fault_count", 1) == 0
        and r["motor_summary"].get("stop_success", False)
        for r in final["repeats"]
    )
    queue_pass = repeats_pass and all(
        sum(r["capture_analysis"].get("recorder", {}).get("queue_overflow", {}).values()) == 0
        and r["capture_analysis"].get("recorder", {}).get("writer_errors", 1) == 0
        and all((w.get("diagnostics_last", {}).get("source_capture_overflow_drops") or 0) == 0 for w in r["capture_analysis"].get("windows", []))
        and all((w.get("diagnostics_last", {}).get("source_capture_writer_errors") or 0) == 0 for w in r["capture_analysis"].get("windows", []))
        for r in final["repeats"]
    )
    final["acceptance"] = {
        "source_capture_complete": data_pass and queue_pass,
        "publisher_and_recorder_continuity": data_pass,
        "fk_joint_observation_continuity": data_pass,
        "design_b_deadline_and_motor_safety": motor_pass,
        "two_repeat_success": repeats_pass,
        "park_and_final_audit": bool((final.get("park_result") or {}).get("passed")) and (final.get("final_read_only_audit") or {}).get("audit", {}).get("status") == "PASS",
    }
    final["hardware_write_count"] = int(sum(
        int(r.get("motor_summary", {}).get("hardware_write_count_estimate_0x29", 0))
        + int(r.get("motor_summary", {}).get("hardware_write_count_estimate_config_mode", 0))
        + int(r.get("motor_summary", {}).get("hardware_write_count_estimate_stop_0x1a", 0))
        for r in final["repeats"]
    ))
    final["r4_status"] = "PASS" if all(final["acceptance"].values()) else "FAIL"
    final["recommended_next_stage"] = "R3 recalibration may be reconsidered only after review" if final["r4_status"] == "PASS" else "Diagnose remaining runtime loss or scheduler failures before calibration"
    raw_path = args.out_dir / "r4_raw.json"
    raw_path.write_text(json.dumps(final, indent=2), encoding="utf-8")
    report_path = args.out_dir / "r4_report.md"
    lines = [
        "# J9-E1B-R4 Active-Load Runtime Isolation & Lossless Capture Qualification",
        "",
        f"Status: **{final['r4_status']}**",
        "Robot extrinsics status: `UNCALIBRATED_UNSET`",
        "",
        "No orientation or lever-arm calibration was performed.",
        "",
        "## Acceptance",
    ]
    lines.extend(f"- {key}: `{value}`" for key, value in final["acceptance"].items())
    for run in final["repeats"]:
        lines.extend([
            "",
            f"## Repeat {run['run_index']}",
            f"- status: `{run['status']}`",
            f"- motor summary: `{run.get('motor_summary', {})}`",
        ])
        for window in run.get("capture_analysis", {}).get("windows", []):
            if window["segment"]["role"] in ("roll", "pitch", "yaw", "combined"):
                lines.append(f"- {window['segment']['role']} capture: `{window}`")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"r4_status": final["r4_status"], "acceptance": final["acceptance"], "raw": str(raw_path), "report": str(report_path)}, indent=2), flush=True)
    return 0 if final["r4_status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
