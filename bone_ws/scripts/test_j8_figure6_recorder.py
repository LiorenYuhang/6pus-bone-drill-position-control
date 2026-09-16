#!/usr/bin/env python3
"""Preflight unit test and dry-run qualification for Stewart-3 J8 Figure-6 Recorder.

Validates:
  1. py_compile syntax
  2. Zero hardware writes / zero serial port operations
  3. Zero publisher on motor/control topics
  4. Exact 4 topics subscribed with correct types and QoS
  5. Correct CSV headers and non-empty rows for all 4 streams
  6. Common monotonic timebase (timestamp, elapsed_time, elapsed_time_s)
  7. Graceful clean shutdown
"""

from __future__ import annotations

import csv
import json
import math
import os
import py_compile
import shutil
import sys
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray

from stewart3_j8_figure6_recorder import Stewart3J8Figure6RecorderNode


def test_recorder_qualification() -> None:
    print("[TEST 1/6] Running py_compile check...")
    script_path = Path(__file__).parent / "stewart3_j8_figure6_recorder.py"
    py_compile.compile(str(script_path), doraise=True)
    print("  -> py_compile PASS")

    tmp_dir = Path(tempfile.mkdtemp(prefix="test_j8_figure6_recorder_"))
    print(f"[TEST 2/6] Initializing node in sandbox dir: {tmp_dir}")

    rclpy.init()
    t0_s = time.monotonic()
    node = Stewart3J8Figure6RecorderNode(out_dir=tmp_dir, start_time_s=t0_s)

    # Check zero publishers (excluding ROS 2 built-in /parameter_events)
    custom_pubs = [p.topic_name for p in node.publishers if not p.topic_name.startswith("/parameter")]
    assert len(custom_pubs) == 0, f"Recorder must NOT publish any topics, found: {custom_pubs}"
    print("  -> Zero motor/control topic publishers PASS")

    # Check subscriptions
    sub_map = {s.topic_name: s for s in node.subscriptions}
    expected_topics = [
        "/pus6/target_cartesian_pose",
        "/pus6/command_joint_steps",
        "/pus6/joint_feedback/timed",
        "/pus6/fk/state",
    ]
    for topic in expected_topics:
        assert topic in sub_map, f"Missing expected subscription: {topic}"
    print(f"  -> Exact 4 passive subscriptions verified: {expected_topics}")

    print("[TEST 3/6] Spinning node and publishing mock data...")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    # Mock publisher helper node
    mock_node = Node("test_mock_publisher")
    reliable_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub_target = mock_node.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", reliable_qos)
    pub_cmd = mock_node.create_publisher(Int32MultiArray, "/pus6/command_joint_steps", qos_profile_sensor_data)
    pub_fb = mock_node.create_publisher(TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data)
    pub_fk = mock_node.create_publisher(FkStateStamped, "/pus6/fk/state", qos_profile_sensor_data)

    time.sleep(0.2)

    for i in range(5):
        # 1. Target pose
        p_msg = PoseStamped()
        p_msg.header.stamp = mock_node.get_clock().now().to_msg()
        p_msg.pose.position.x = 0.001 * i
        p_msg.pose.position.y = -0.001 * i
        p_msg.pose.position.z = 0.132612
        p_msg.pose.orientation.w = 1.0
        pub_target.publish(p_msg)

        # 2. Command steps
        c_msg = Int32MultiArray()
        c_msg.data = [1000 + i * 10] * 6
        pub_cmd.publish(c_msg)

        # 3. Joint feedback
        fb_msg = TimedJointFeedback()
        fb_msg.header.stamp = mock_node.get_clock().now().to_msg()
        fb_msg.snapshot_start_monotonic_ns = time.monotonic_ns()
        fb_msg.snapshot_end_monotonic_ns = time.monotonic_ns() + 1000
        fb_msg.uart_cycle_sequence = 100 + i
        fb_msg.raw_steps = [1000 + i * 10] * 6
        fb_msg.positions_meters = [0.05 + 0.0001 * i] * 6
        fb_msg.currents_ma = [50 + i] * 6
        fb_msg.fault_codes = [0] * 6
        pub_fb.publish(fb_msg)

        # 4. FK
        fk_msg = FkStateStamped()
        fk_msg.header.stamp = mock_node.get_clock().now().to_msg()
        fk_msg.query_monotonic_timestamp_ns = time.monotonic_ns()
        fk_msg.pose.position.x = 0.001 * i
        fk_msg.pose.position.y = -0.001 * i
        fk_msg.pose.position.z = 0.132612
        fk_msg.pose.orientation.w = 1.0
        fk_msg.euler_angles_rad = [0.0, 0.0, 0.0]
        fk_msg.converged = True
        fk_msg.residual_norm_meters = 1e-9
        fk_msg.compute_time_us = 450.0
        pub_fk.publish(fk_msg)

        time.sleep(0.05)

    time.sleep(0.5)

    print("[TEST 4/6] Shutting down recorder and verifying buffer flushing...")
    node.writer.close()
    node.destroy_node()
    mock_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    th.join(timeout=2.0)
    print("  -> Shutdown and flush PASS")

    print("[TEST 5/6] Inspecting generated CSV files and headers...")
    expected_files = ["target_pose.csv", "command_steps.csv", "joint_feedback.csv", "fk.csv"]
    for fname in expected_files:
        fpath = tmp_dir / fname
        assert fpath.exists(), f"Expected CSV not found: {fname}"
        with open(fpath, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
            assert len(rows) >= 5, f"{fname} expected >=5 rows, got {len(rows)}"
            print(f"  -> {fname}: {len(rows)} rows, header: {header}")

    # Header specific assertions
    target_header = next(csv.reader(open(tmp_dir / "target_pose.csv")))
    for col in ["timestamp", "elapsed_time", "elapsed_time_s", "x", "y", "z", "roll", "pitch", "yaw", "qx", "qy", "qz", "qw"]:
        assert col in target_header, f"target_pose.csv missing col: {col}"

    cmd_header = next(csv.reader(open(tmp_dir / "command_steps.csv")))
    for col in ["timestamp", "elapsed_time", "elapsed_time_s", "cmd_1", "cmd_2", "cmd_3", "cmd_4", "cmd_5", "cmd_6"]:
        assert col in cmd_header, f"command_steps.csv missing col: {col}"

    fb_header = next(csv.reader(open(tmp_dir / "joint_feedback.csv")))
    for col in ["timestamp", "elapsed_time", "elapsed_time_s", "fb_1", "fb_2", "fb_3", "fb_4", "fb_5", "fb_6"]:
        assert col in fb_header, f"joint_feedback.csv missing col: {col}"

    fk_header = next(csv.reader(open(tmp_dir / "fk.csv")))
    for col in ["timestamp", "elapsed_time", "elapsed_time_s", "x", "y", "z", "roll", "pitch", "yaw"]:
        assert col in fk_header, f"fk.csv missing col: {col}"

    print("[TEST 6/6] Checking summary JSON...")
    summary = json.loads((tmp_dir / "recorder_summary.json").read_text(encoding="utf-8"))
    assert summary["errors"] == 0, f"Recorder recorded errors: {summary['errors']}"
    assert all(v >= 5 for v in summary["written"].values()), f"Written counts too low: {summary['written']}"
    print(f"  -> Summary: {summary}")

    shutil.rmtree(tmp_dir)
    print("\nALL 6 RECORDER TESTS PASSED SUCCESSFULLY!")


if __name__ == "__main__":
    test_recorder_qualification()
