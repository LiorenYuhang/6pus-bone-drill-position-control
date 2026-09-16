#!/usr/bin/env python3
"""Isolated Pure Bypass ROS 2 Recorder for Stewart-3 J8 Figure-6 Experiment.

PASSIVELY subscribes to:
  1. /pus6/target_cartesian_pose   (geometry_msgs/PoseStamped)
  2. /pus6/command_joint_steps     (std_msgs/Int32MultiArray)
  3. /pus6/joint_feedback/timed    (pus6_msgs/TimedJointFeedback)
  4. /pus6/fk/state                (pus6_msgs/FkStateStamped)

STRICT INVARIANTS:
  - ZERO hardware writes (no serial port open, no control topic publication).
  - Non-blocking callback path: all rows queued to background CsvQueueWriter.
  - Common monotonic timebase across all output files:
      timestamp: host CLOCK_MONOTONIC in seconds
      elapsed_time / elapsed_time_s: seconds since common recorder start
      ros_header_stamp_ns: ROS header timestamp in nanoseconds (when available)
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32MultiArray


class CsvQueueWriter:
    """High-throughput multi-stream CSV writer backed by an asynchronous queue."""

    def __init__(self, out_dir: Path, capacity: int = 262144) -> None:
        self.out_dir = out_dir
        self.capacity = capacity
        self.queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue(maxsize=capacity)
        self.files: dict[str, Any] = {}
        self.writers: dict[str, csv.DictWriter] = {}
        self.written: dict[str, int] = {}
        self.overflow: dict[str, int] = {}
        self._last_fsync_ns: dict[str, int] = {}
        self.errors = 0
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self._worker, name="j8-figure6-writer", daemon=True)
        self.thread.start()

    def enqueue(self, stream: str, row: dict[str, Any]) -> bool:
        try:
            self.queue.put_nowait((stream, row))
            return True
        except queue.Full:
            self.overflow[stream] = self.overflow.get(stream, 0) + 1
            return False

    def _worker(self) -> None:
        while True:
            item = self.queue.get()
            if item is None:
                self.queue.task_done()
                break
            stream, row = item
            try:
                if stream not in self.files:
                    path = self.out_dir / f"{stream}.csv"
                    handle = path.open("w", newline="", encoding="utf-8")
                    self.files[stream] = handle
                    writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
                    writer.writeheader()
                    self.writers[stream] = writer
                    self.written[stream] = 0
                self.writers[stream].writerow(row)
                self.written[stream] = self.written.get(stream, 0) + 1
                handle = self.files[stream]
                handle.flush()
                now_ns = time.monotonic_ns()
                if now_ns - self._last_fsync_ns.get(stream, 0) >= 1_000_000_000:
                    os.fsync(handle.fileno())
                    self._last_fsync_ns[stream] = now_ns
            except Exception:
                self.errors += 1
            self.queue.task_done()

    def close(self) -> None:
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self.thread.join(timeout=5.0)
        for handle in self.files.values():
            handle.flush()
            handle.close()
        summary_path = self.out_dir / "recorder_summary.json"
        summary_path.write_text(
            json.dumps(
                {
                    "capacity": self.capacity,
                    "written": {k: int(v) for k, v in self.written.items()},
                    "overflow": {k: int(v) for k, v in self.overflow.items()},
                    "errors": int(self.errors),
                },
                indent=2,
            ),
            encoding="utf-8",
        )


class Stewart3J8Figure6RecorderNode(Node):
    """Pure bypass observer node for J8 figure-6 experiments."""

    def __init__(self, out_dir: Path, start_time_s: float | None = None, queue_capacity: int = 262144) -> None:
        super().__init__("stewart3_j8_figure6_recorder")
        self.start_time_s = start_time_s if start_time_s is not None else time.monotonic()
        self.writer = CsvQueueWriter(out_dir, queue_capacity)
        self.cmd_seq = 0

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        sensor_qos = qos_profile_sensor_data

        # 1. Target Pose
        self.create_subscription(
            PoseStamped,
            "/pus6/target_cartesian_pose",
            self._on_target_pose,
            reliable_qos,
        )

        # 2. Command Steps
        self.create_subscription(
            Int32MultiArray,
            "/pus6/command_joint_steps",
            self._on_command_steps,
            sensor_qos,
        )

        # 3. Joint Feedback
        self.create_subscription(
            TimedJointFeedback,
            "/pus6/joint_feedback/timed",
            self._on_joint_feedback,
            sensor_qos,
        )

        # 4. Forward Kinematics (FK)
        self.create_subscription(
            FkStateStamped,
            "/pus6/fk/state",
            self._on_fk_state,
            sensor_qos,
        )

        self.get_logger().info(
            f"Stewart-3 J8 Figure-6 Pure Recorder initialized. out_dir={out_dir}, t0={self.start_time_s:.6f}"
        )

    def _calc_timebase(self, now_s: float) -> tuple[float, float, float]:
        elapsed = now_s - self.start_time_s
        return round(now_s, 6), round(elapsed, 6), round(elapsed, 6)

    @staticmethod
    def _header_ns(msg: Any) -> int:
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        return 0

    def _on_target_pose(self, msg: PoseStamped) -> None:
        now_s = time.monotonic()
        t_now, elapsed, elapsed_s = self._calc_timebase(now_s)
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        try:
            r = Rotation.from_quat([qx, qy, qz, qw])
            roll, pitch, yaw = r.as_euler("xyz", degrees=False)
        except Exception:
            roll, pitch, yaw = 0.0, 0.0, 0.0

        row = {
            "timestamp": t_now,
            "elapsed_time": elapsed,
            "elapsed_time_s": elapsed_s,
            "ros_header_stamp_ns": self._header_ns(msg),
            "x": round(float(msg.pose.position.x), 8),
            "y": round(float(msg.pose.position.y), 8),
            "z": round(float(msg.pose.position.z), 8),
            "roll": round(float(roll), 8),
            "pitch": round(float(pitch), 8),
            "yaw": round(float(yaw), 8),
            "qx": round(qx, 8),
            "qy": round(qy, 8),
            "qz": round(qz, 8),
            "qw": round(qw, 8),
        }
        self.writer.enqueue("target_pose", row)

    def _on_command_steps(self, msg: Int32MultiArray) -> None:
        now_s = time.monotonic()
        t_now, elapsed, elapsed_s = self._calc_timebase(now_s)
        self.cmd_seq += 1
        data = list(msg.data)
        if len(data) < 6:
            data.extend([0] * (6 - len(data)))

        row = {
            "timestamp": t_now,
            "elapsed_time": elapsed,
            "elapsed_time_s": elapsed_s,
            "cmd_seq": self.cmd_seq,
            "cmd_1": int(data[0]),
            "cmd_2": int(data[1]),
            "cmd_3": int(data[2]),
            "cmd_4": int(data[3]),
            "cmd_5": int(data[4]),
            "cmd_6": int(data[5]),
        }
        self.writer.enqueue("command_steps", row)

    def _on_joint_feedback(self, msg: TimedJointFeedback) -> None:
        now_s = time.monotonic()
        t_now, elapsed, elapsed_s = self._calc_timebase(now_s)
        raw = list(msg.raw_steps) if hasattr(msg, "raw_steps") else [0] * 6
        pos = list(msg.positions_meters) if hasattr(msg, "positions_meters") else [0.0] * 6
        curr = list(msg.currents_ma) if hasattr(msg, "currents_ma") else [0] * 6
        faults = list(msg.fault_codes) if hasattr(msg, "fault_codes") else [0] * 6

        row = {
            "timestamp": t_now,
            "elapsed_time": elapsed,
            "elapsed_time_s": elapsed_s,
            "ros_header_stamp_ns": self._header_ns(msg),
            "snapshot_start_monotonic_ns": int(getattr(msg, "snapshot_start_monotonic_ns", 0)),
            "snapshot_end_monotonic_ns": int(getattr(msg, "snapshot_end_monotonic_ns", 0)),
            "uart_cycle_sequence": int(getattr(msg, "uart_cycle_sequence", 0)),
            "fb_1": int(raw[0]),
            "fb_2": int(raw[1]),
            "fb_3": int(raw[2]),
            "fb_4": int(raw[3]),
            "fb_5": int(raw[4]),
            "fb_6": int(raw[5]),
            "pos_m_1": round(float(pos[0]), 8),
            "pos_m_2": round(float(pos[1]), 8),
            "pos_m_3": round(float(pos[2]), 8),
            "pos_m_4": round(float(pos[3]), 8),
            "pos_m_5": round(float(pos[4]), 8),
            "pos_m_6": round(float(pos[5]), 8),
            "current_ma_1": int(curr[0]),
            "current_ma_2": int(curr[1]),
            "current_ma_3": int(curr[2]),
            "current_ma_4": int(curr[3]),
            "current_ma_5": int(curr[4]),
            "current_ma_6": int(curr[5]),
            "fault_1": int(faults[0]),
            "fault_2": int(faults[1]),
            "fault_3": int(faults[2]),
            "fault_4": int(faults[3]),
            "fault_5": int(faults[4]),
            "fault_6": int(faults[5]),
        }
        self.writer.enqueue("joint_feedback", row)

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        now_s = time.monotonic()
        t_now, elapsed, elapsed_s = self._calc_timebase(now_s)

        roll = float(msg.euler_angles_rad[0]) if len(msg.euler_angles_rad) > 0 else 0.0
        pitch = float(msg.euler_angles_rad[1]) if len(msg.euler_angles_rad) > 1 else 0.0
        yaw = float(msg.euler_angles_rad[2]) if len(msg.euler_angles_rad) > 2 else 0.0

        row = {
            "timestamp": t_now,
            "elapsed_time": elapsed,
            "elapsed_time_s": elapsed_s,
            "ros_header_stamp_ns": self._header_ns(msg),
            "query_monotonic_timestamp_ns": int(getattr(msg, "query_monotonic_timestamp_ns", 0)),
            "x": round(float(msg.pose.position.x), 8),
            "y": round(float(msg.pose.position.y), 8),
            "z": round(float(msg.pose.position.z), 8),
            "roll": round(roll, 8),
            "pitch": round(pitch, 8),
            "yaw": round(yaw, 8),
            "qx": round(float(msg.pose.orientation.x), 8),
            "qy": round(float(msg.pose.orientation.y), 8),
            "qz": round(float(msg.pose.orientation.z), 8),
            "qw": round(float(msg.pose.orientation.w), 8),
            "converged": int(msg.converged),
            "residual_norm_meters": float(msg.residual_norm_meters),
            "compute_time_us": round(float(msg.compute_time_us), 2),
        }
        self.writer.enqueue("fk", row)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Isolated Pure Bypass ROS 2 Recorder for J8 Figure-6")
    parser.add_argument("--out-dir", type=Path, required=True, help="Directory to save CSV logs")
    parser.add_argument("--start-time-s", type=float, default=None, help="Common monotonic start time anchor in seconds")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Stewart3J8Figure6RecorderNode(out_dir=args.out_dir, start_time_s=args.start_time_s)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()

    def _sig_handler(sig: int, frame: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    while not stop_event.is_set():
        time.sleep(0.1)

    node.get_logger().info("Shutting down recorder and flushing CSV buffers...")
    node.writer.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    th.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
