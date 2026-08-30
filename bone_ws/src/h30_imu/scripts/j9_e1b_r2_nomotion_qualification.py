#!/usr/bin/env python3
"""J9-E1B-R2 no-motion calibration data-path qualification.

No motor/control nodes are launched and no command topics are published. The
script launches the platform H30 reader with the opt-in calibration TimedImu QoS,
records in an isolated rclpy executor, and optionally starts a separate process
that publishes fake observation traffic to approximate E1B ROS load without
hardware writes.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose
from h30_imu.msg import TimedImu
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

PLATFORM_IMU_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00"


@dataclass
class Trace:
    samples: list[dict[str, Any]] = field(default_factory=list)
    diagnostics: list[dict[str, Any]] = field(default_factory=list)
    callback_start_ns: list[int] = field(default_factory=list)
    callback_duration_ns: list[int] = field(default_factory=list)


class CalibrationRecorder(Node):
    def __init__(self, qos_depth: int, reliability: str) -> None:
        super().__init__("j9_e1b_r2_calibration_imu_recorder")
        rel = ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(qos_depth),
            reliability=rel,
            durability=DurabilityPolicy.VOLATILE,
        )
        diag_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.trace = Trace()
        self.lock = threading.Lock()
        self.create_subscription(TimedImu, "/pus6/imu_platform/timed", self._on_imu, qos)
        self.create_subscription(DiagnosticArray, "/pus6/imu_platform/diagnostics", self._on_diag, diag_qos)

    def _on_imu(self, msg: TimedImu) -> None:
        t0 = time.monotonic_ns()
        row = {
            "callback_start_monotonic_ns": int(t0),
            "publisher_sequence": int(msg.publisher_sequence),
            "tid": int(msg.tid),
            "has_sample_timestamp": bool(msg.has_sample_timestamp),
            "sample_timestamp_us": int(msg.sample_timestamp_us),
            "has_data_ready_timestamp": bool(msg.has_data_ready_timestamp),
            "data_ready_timestamp_us": int(msg.data_ready_timestamp_us),
            "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
            "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            "frame_id": str(msg.header.frame_id),
            "angular_velocity_rad_s": [
                float(msg.imu.angular_velocity.x),
                float(msg.imu.angular_velocity.y),
                float(msg.imu.angular_velocity.z),
            ],
        }
        t1 = time.monotonic_ns()
        with self.lock:
            self.trace.samples.append(row)
            self.trace.callback_start_ns.append(t0)
            self.trace.callback_duration_ns.append(t1 - t0)

    def _on_diag(self, msg: DiagnosticArray) -> None:
        rows = []
        for status in msg.status:
            level = status.level
            level_int = int.from_bytes(level, "little", signed=False) if isinstance(level, (bytes, bytearray)) else int(level)
            rows.append({
                "name": status.name,
                "hardware_id": status.hardware_id,
                "level": level_int,
                "message": status.message,
                "values": {kv.key: kv.value for kv in status.values},
            })
        with self.lock:
            self.trace.diagnostics.append({
                "callback_start_monotonic_ns": time.monotonic_ns(),
                "ros_header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
                "status": rows,
            })


class RepresentativeLoad(Node):
    def __init__(self) -> None:
        super().__init__("j9_e1b_r2_representative_observation_load")
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.fk_pub = self.create_publisher(FkStateStamped, "/pus6/fk/state", sensor_qos)
        self.feedback_pub = self.create_publisher(TimedJointFeedback, "/pus6/joint_feedback/timed", sensor_qos)
        self.diag_pub = self.create_publisher(Float64MultiArray, "/pus6/driver_diagnostics", sensor_qos)
        self.seq = 0
        self.create_timer(0.005, self._pub_fk)
        self.create_timer(0.020, self._pub_feedback)

    def _pub_fk(self) -> None:
        msg = FkStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "platform_link"
        msg.query_monotonic_timestamp_ns = time.monotonic_ns()
        msg.pose.orientation.w = 1.0
        msg.converged = True
        self.fk_pub.publish(msg)

    def _pub_feedback(self) -> None:
        now = time.monotonic_ns()
        msg = TimedJointFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.snapshot_start_monotonic_ns = now
        msg.snapshot_end_monotonic_ns = now
        msg.per_axis_rx_done_monotonic_ns = [now] * 6
        msg.raw_steps = [1000] * 6
        msg.positions_meters = [0.05] * 6
        msg.currents_ma = [0] * 6
        msg.fault_codes = [0] * 6
        msg.uart_cycle_sequence = self.seq
        self.feedback_pub.publish(msg)
        diag = Float64MultiArray()
        diag.data = [float(self.seq), 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.diag_pub.publish(diag)
        self.seq += 1


def load_only(duration_s: float) -> int:
    rclpy.init()
    node = RepresentativeLoad()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(duration_s)
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    return 0


def start_platform_h30(qos_profile: str, qos_depth: int, reliability: str) -> subprocess.Popen[str]:
    cmd = [
        "ros2", "run", "h30_imu", "h30_imu_node",
        "--ros-args",
        "-r", "__ns:=/pus6/imu_platform",
        "-p", f"port:={PLATFORM_IMU_PORT}",
        "-p", "baud_rate:=460800",
        "-p", "frame_id:=pus6_platform_imu_link",
        "-p", "world_frame:=world",
        "-p", "imu_topic:=/pus6/imu_platform/data",
        "-p", "euler_topic:=/pus6/imu_platform/euler",
        "-p", "temperature_topic:=/pus6/imu_platform/temperature",
        "-p", "timed_topic:=/pus6/imu_platform/timed",
        "-p", "diagnostics_topic:=/pus6/imu_platform/diagnostics",
        "-p", "publish_euler:=true",
        "-p", "publish_tf:=false",
        "-p", f"timed_qos_profile:={qos_profile}",
        "-p", f"timed_qos_depth:={qos_depth}",
        "-p", f"timed_qos_reliability:={reliability}",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def seq_stats(values: list[int], expected_step: int = 1) -> dict[str, Any]:
    vals = np.asarray(values, dtype=np.int64)
    if len(vals) < 2:
        return {"count": int(len(vals)), "missing": 0, "duplicates": 0, "out_of_order": 0}
    diff = np.diff(vals)
    return {
        "count": int(len(vals)),
        "missing": int(np.sum(np.maximum(0, diff - expected_step) // max(1, expected_step))),
        "positive_gap_count": int(np.sum(diff > expected_step)),
        "duplicates": int(np.sum(diff == 0)),
        "out_of_order": int(np.sum(diff < 0)),
        "min_delta": int(np.min(diff)),
        "p50_delta": float(np.percentile(diff, 50.0)),
        "p95_delta": float(np.percentile(diff, 95.0)),
        "p99_delta": float(np.percentile(diff, 99.0)),
        "max_delta": int(np.max(diff)),
    }


def latest_diag(diags: list[dict[str, Any]]) -> dict[str, Any]:
    for diag in reversed(diags):
        if diag.get("status"):
            st = diag["status"][0]
            vals = {}
            for key, value in st.get("values", {}).items():
                try:
                    vals[key] = int(value)
                except ValueError:
                    vals[key] = value
            return {
                "callback_start_monotonic_ns": diag["callback_start_monotonic_ns"],
                "hardware_id": st.get("hardware_id", ""),
                "level": st.get("level", -1),
                "message": st.get("message", ""),
                "values": vals,
            }
    return {"values": {}}


def summarize(trace: Trace, duration_s: float, qos_profile: str, qos_depth: int, reliability: str, load_enabled: bool) -> dict[str, Any]:
    samples = trace.samples
    diags = trace.diagnostics
    last = latest_diag(diags)
    vals = last.get("values", {})
    published = int(vals.get("published_timed_imu_count", 0) or 0)
    source = int(vals.get("valid_frames", 0) or 0)
    sequences = [int(r["publisher_sequence"]) for r in samples]
    tids = [int(r["tid"]) for r in samples]
    sample_ts = [int(r["sample_timestamp_us"]) for r in samples if r.get("has_sample_timestamp")]
    ready_ts = [int(r["data_ready_timestamp_us"]) for r in samples if r.get("has_data_ready_timestamp")]
    cb_start = np.asarray(trace.callback_start_ns, dtype=np.int64)
    cb_dur = np.asarray(trace.callback_duration_ns, dtype=np.int64)
    if samples:
        span = max(1e-9, (samples[-1]["host_monotonic_arrival_time_ns"] - samples[0]["host_monotonic_arrival_time_ns"]) / 1e9)
    else:
        span = 0.0
    callback = {}
    if len(cb_start) > 1:
        dt_ms = np.diff(cb_start) / 1e6
        callback["inter_callback_ms"] = {
            "mean": float(np.mean(dt_ms)),
            "p50": float(np.percentile(dt_ms, 50)),
            "p95": float(np.percentile(dt_ms, 95)),
            "p99": float(np.percentile(dt_ms, 99)),
            "max": float(np.max(dt_ms)),
        }
    if len(cb_dur):
        dur_us = cb_dur / 1e3
        callback["callback_duration_us"] = {
            "mean": float(np.mean(dur_us)),
            "p50": float(np.percentile(dur_us, 50)),
            "p95": float(np.percentile(dur_us, 95)),
            "p99": float(np.percentile(dur_us, 99)),
            "max": float(np.max(dur_us)),
        }
    return {
        "stage": "J9-E1B-R2",
        "motion": "none",
        "hardware_write_count": 0,
        "representative_observation_load_enabled": bool(load_enabled),
        "qos": {
            "publisher_profile": qos_profile,
            "publisher_depth": int(qos_depth),
            "publisher_reliability_arg": reliability,
            "subscriber_depth": int(qos_depth),
            "subscriber_reliability": reliability,
        },
        "duration_requested_s": float(duration_s),
        "observed_span_s": float(span),
        "last_diagnostics": last,
        "counts": {
            "source_parser_valid_frames": source,
            "timed_imu_published_count": published,
            "subscriber_callback_count": len(samples),
            "recorder_count": len(samples),
        },
        "rates_hz": {
            "source_from_last_diag_over_requested": float(source / duration_s) if duration_s > 0 else 0.0,
            "publisher_from_last_diag_over_requested": float(published / duration_s) if duration_s > 0 else 0.0,
            "subscriber_over_observed_span": float(len(samples) / span) if span > 0 else 0.0,
        },
        "loss": {
            "serial_parser_to_publisher": int(source - published),
            "publisher_to_subscriber_at_last_diag_approx": int(published - min(len(samples), published)),
            "subscriber_to_recorder": 0,
        },
        "continuity": {
            "publisher_sequence": seq_stats(sequences, 1),
            "tid": seq_stats(tids, 1),
            "sample_timestamp_0x51_us": seq_stats(sample_ts, 5000),
            "data_ready_timestamp_0x52_us": seq_stats(ready_ts, 5000),
        },
        "callback_timing": callback,
        "frame_ids": sorted(set(r.get("frame_id", "") for r in samples)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--load-only", action="store_true")
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--timed-qos-profile", default="calibration")
    parser.add_argument("--timed-qos-depth", type=int, default=4096)
    parser.add_argument("--timed-qos-reliability", choices=["best_effort", "reliable"], default="reliable")
    parser.add_argument("--representative-load", action="store_true")
    args = parser.parse_args()
    if args.load_only:
        return load_only(args.duration_s)
    if args.out is None:
        raise SystemExit("--out is required unless --load-only is used")

    rclpy.init()
    recorder = CalibrationRecorder(args.timed_qos_depth, args.timed_qos_reliability)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    h30_proc: subprocess.Popen[str] | None = None
    load_proc: subprocess.Popen[str] | None = None
    h30_log: list[str] = []
    try:
        h30_proc = start_platform_h30(args.timed_qos_profile, args.timed_qos_depth, args.timed_qos_reliability)

        def read_h30_log() -> None:
            assert h30_proc is not None and h30_proc.stdout is not None
            for line in h30_proc.stdout:
                h30_log.append(line.rstrip())
                if "Opened H30 serial port" in line or "Cannot open" in line:
                    print("[H30]", line.rstrip(), flush=True)

        threading.Thread(target=read_h30_log, daemon=True).start()
        if args.representative_load:
            load_proc = subprocess.Popen([
                sys.executable,
                str(Path(__file__).resolve()),
                "--load-only",
                "--duration-s",
                str(args.duration_s + 2.0),
            ])
        spin_thread.start()
        print(f"[QUAL] no-motion TimedImu data-path qualification for {args.duration_s:.1f}s", flush=True)
        time.sleep(args.duration_s)
    finally:
        try:
            recorder.destroy_node()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        for proc in [h30_proc, load_proc]:
            if proc is None:
                continue
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=5)
            except Exception:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except Exception:
                    proc.kill()

    with recorder.lock:
        summary = summarize(
            recorder.trace,
            args.duration_s,
            args.timed_qos_profile,
            args.timed_qos_depth,
            args.timed_qos_reliability,
            args.representative_load,
        )
        raw = {
            "summary": summary,
            "samples": recorder.trace.samples,
            "diagnostics": recorder.trace.diagnostics,
            "h30_process_log_tail": h30_log[-120:],
        }
    args.out.write_text(json.dumps(raw, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)
    print(f"[WRITE] {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
