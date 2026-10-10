#!/usr/bin/env python3
"""J14-A1.11 Feedback Pipeline Instrumentation Overhead Benchmark.

Compares pipeline processing time with instrumentation OFF vs ON:
1. _accept_child_feedback (IPC feedback unpacking + snapshot creation + superseded check)
2. _publish_telemetry_from_snapshot (message build + ROS 2 topic publish)
3. Total per-cycle parent telemetry processing latency
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import statistics
import time
from unittest.mock import MagicMock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
from pus6_motor_driver.motor_driver_node import ServoTelemetrySnapshot
from pus6_motor_driver.safety import HardwareMode

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None
    TimedJointFeedback = None


def calc_stats(values_us: list[float]) -> dict[str, float]:
    if not values_us:
        return {}
    v = sorted(values_us)
    n = len(v)

    def q(p: float) -> float:
        idx = (n - 1) * p
        lo = int(idx)
        return v[lo] + (v[min(lo + 1, n - 1)] - v[lo]) * (idx - lo)

    return {
        "n": n,
        "mean_us": round(statistics.mean(v), 3),
        "std_us": round(statistics.stdev(v), 3) if n > 1 else 0.0,
        "p50_us": round(q(0.50), 3),
        "p95_us": round(q(0.95), 3),
        "p99_us": round(q(0.99), 3),
        "max_us": round(v[-1], 3),
    }


def make_dummy_row(seq: int) -> dict:
    now = time.monotonic_ns()
    return {
        "epoch": 100,
        "cycle_seq": seq,
        "command_seq": seq,
        "positions": [1000] * 6,
        "currents": [10] * 6,
        "faults": [0] * 6,
        "rx_ns": [now] * 6,
        "targets": [1000] * 6,
        "tx_start_ns": [now - 1000000] * 6,
        "tx_end_ns": [now - 500000] * 6,
        "tx_wall_ns": [500000] * 6,
        "rx_drain_ns": 20000,
        "cycle_start_ns": now - 2000000,
        "cycle_end_ns": now - 100000,
        "feedback_ready_ns": now - 100000,
        "command_publish_ns": now - 3000000,
        "parent_receive_ns": now,
    }


def benchmark_mode(enabled: bool, n_iterations: int = 1000) -> dict:
    import threading
    ros_node = Node(f"bench_node_{'on' if enabled else 'off'}")
    driver = object.__new__(J14ProcessMotorDriverNode)
    driver._instrumentation_enabled = enabled
    driver._pipeline_records = []
    driver._telemetry_lock = threading.Lock()
    driver._telemetry_ready_event = threading.Event()
    driver._latest_telemetry_snapshot = None
    driver._current_pending_snap = None
    driver._process_client = MagicMock()
    driver._process_client.config = {"epoch": 100}
    driver._latest_positions_steps = {i: 1000 for i in range(1, 7)}
    driver._latest_currents = {i: 0 for i in range(1, 7)}
    driver._latest_faults = {i: 0 for i in range(1, 7)}
    driver._latest_rx_done_ns = {i: 0 for i in range(1, 7)}
    driver._last_commanded_steps = [1000] * 6
    driver._target_in_count = 0
    driver._target_overwrite_count = 0
    driver.get_clock = ros_node.get_clock
    driver.get_logger = ros_node.get_logger
    driver.joint_names = [f"motor_{i}" for i in range(1, 7)]
    driver.hardware_mode = HardwareMode.MOCK
    driver._defer_driver_diagnostics = True
    driver.publish_raw_diagnostics = False
    driver.benchmark_stamp_pub = None
    driver.driver_diagnostics_pub = None
    driver._stall_recorder = None

    driver.joint_state_pub = ros_node.create_publisher(JointState, f"/bench_{'on' if enabled else 'off'}/joint_states", 10)
    if TimedJointFeedback is not None:
        driver.timed_feedback_pub = ros_node.create_publisher(TimedJointFeedback, f"/bench_{'on' if enabled else 'off'}/timed_feedback", 10)
    else:
        driver.timed_feedback_pub = None
    if TimedJointCommand is not None:
        driver.timed_command_pub = ros_node.create_publisher(TimedJointCommand, f"/bench_{'on' if enabled else 'off'}/timed_command", 10)
    else:
        driver.timed_command_pub = None

    accept_durations_us: list[float] = []
    publish_durations_us: list[float] = []
    total_durations_us: list[float] = []

    # Warm-up
    for i in range(50):
        driver._accept_child_feedback(make_dummy_row(i))
        snap = driver._latest_telemetry_snapshot
        if snap is not None:
            driver._publish_telemetry_from_snapshot(snap, record_stall=False)

    driver._pipeline_records.clear()

    # Benchmark loop
    for i in range(n_iterations):
        row = make_dummy_row(100 + i)

        t0 = time.perf_counter_ns()
        driver._accept_child_feedback(row)
        t1 = time.perf_counter_ns()

        snap = driver._latest_telemetry_snapshot
        driver._publish_telemetry_from_snapshot(snap, record_stall=False, t_wake=t1, tc_wake=time.thread_time_ns(), pc_wake=time.process_time_ns())
        t2 = time.perf_counter_ns()

        accept_durations_us.append((t1 - t0) / 1000.0)
        publish_durations_us.append((t2 - t1) / 1000.0)
        total_durations_us.append((t2 - t0) / 1000.0)

    ros_node.destroy_node()

    return {
        "enabled": enabled,
        "iterations": n_iterations,
        "accept_feedback": calc_stats(accept_durations_us),
        "publish_telemetry": calc_stats(publish_durations_us),
        "total_pipeline": calc_stats(total_durations_us),
        "records_captured": len(driver._pipeline_records),
    }


def main():
    rclpy.init()
    try:
        print("Running benchmark: Instrumentation OFF...")
        off_res = benchmark_mode(enabled=False, n_iterations=2000)

        print("Running benchmark: Instrumentation ON...")
        on_res = benchmark_mode(enabled=True, n_iterations=2000)

        results = {
            "timestamp": time.time(),
            "instrumentation_off": off_res,
            "instrumentation_on": on_res,
            "overhead_delta": {
                "accept_feedback_p50_us": round(on_res["accept_feedback"]["p50_us"] - off_res["accept_feedback"]["p50_us"], 3),
                "publish_telemetry_p50_us": round(on_res["publish_telemetry"]["p50_us"] - off_res["publish_telemetry"]["p50_us"], 3),
                "total_pipeline_p50_us": round(on_res["total_pipeline"]["p50_us"] - off_res["total_pipeline"]["p50_us"], 3),
                "total_pipeline_mean_us": round(on_res["total_pipeline"]["mean_us"] - off_res["total_pipeline"]["mean_us"], 3),
            }
        }

        out_path = Path("/home/lyapunov/bone_drill/results/j14/j14_a1_11_software/overhead_benchmark.json")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)

        print("\n" + "=" * 60)
        print("J14-A1.11 INSTRUMENTATION OVERHEAD BENCHMARK RESULTS")
        print("=" * 60)
        print(f"{'Metric (us)':<25} | {'OFF P50':<10} | {'ON P50':<10} | {'Delta P50':<10} | {'ON Max':<10}")
        print("-" * 60)
        print(f"{'accept_child_feedback':<25} | {off_res['accept_feedback']['p50_us']:<10} | {on_res['accept_feedback']['p50_us']:<10} | {results['overhead_delta']['accept_feedback_p50_us']:<10} | {on_res['accept_feedback']['max_us']:<10}")
        print(f"{'publish_telemetry':<25} | {off_res['publish_telemetry']['p50_us']:<10} | {on_res['publish_telemetry']['p50_us']:<10} | {results['overhead_delta']['publish_telemetry_p50_us']:<10} | {on_res['publish_telemetry']['max_us']:<10}")
        print(f"{'total_pipeline':<25} | {off_res['total_pipeline']['p50_us']:<10} | {on_res['total_pipeline']['p50_us']:<10} | {results['overhead_delta']['total_pipeline_p50_us']:<10} | {on_res['total_pipeline']['max_us']:<10}")
        print("=" * 60)
        print(f"Total pipeline P50 overhead: {results['overhead_delta']['total_pipeline_p50_us']} us")
        print(f"Total pipeline Mean overhead: {results['overhead_delta']['total_pipeline_mean_us']} us")
        print(f"Results written to: {out_path}")

        assert results["overhead_delta"]["total_pipeline_p50_us"] < 50.0, f"Overhead too high: {results['overhead_delta']['total_pipeline_p50_us']} us"
        print("OVERHEAD CRITERIA VERIFIED: NEGLIGIBLE OVERHEAD (PASS)")

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
