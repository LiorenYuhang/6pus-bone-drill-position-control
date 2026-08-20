#!/usr/bin/env python3
"""J5-C2-RT-SW-C layered 200 Hz ROS 2 Mock ablation.

Runs A..F for a fixed wall-clock duration and reports timer callback count,
mean rate, period quantiles, and callback duration quantiles. No motor driver
or UART is instantiated.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
KINEMATICS_SRC = WORKSPACE_ROOT / "src" / "pus6_kinematics"
if str(KINEMATICS_SRC) not in sys.path:
    sys.path.insert(0, str(KINEMATICS_SRC))

import rclpy
from pus6_msgs.msg import TimedJointFeedback
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class CallbackStats:
    def __init__(self) -> None:
        self.starts_ns: list[int] = []
        self.durations_ns: list[int] = []

    def record(self, start_ns: int, end_ns: int) -> None:
        self.starts_ns.append(start_ns)
        self.durations_ns.append(end_ns - start_ns)


class EmptyTimerNode(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.stats = CallbackStats()
        self.timer = self.create_timer(1.0 / 200.0, self._on_timer)

    def _on_timer(self) -> None:
        start = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.stats.record(start, time.clock_gettime_ns(time.CLOCK_MONOTONIC))


class PlainPublisher(Node):
    """30 Hz ROS publisher used by ablation B without FK subscribers."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.publisher = self.create_publisher(
            TimedJointFeedback, "/j5_ablation/plain_feedback", qos_profile_sensor_data
        )
        self.sequence = 0
        self.timer = self.create_timer(1.0 / 30.0, self._on_timer)

    def _on_timer(self) -> None:
        msg = TimedJointFeedback()
        msg.uart_cycle_sequence = self.sequence
        msg.snapshot_start_monotonic_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        msg.snapshot_end_monotonic_ns = msg.snapshot_start_monotonic_ns
        msg.per_axis_rx_done_monotonic_ns = [msg.snapshot_end_monotonic_ns] * 6
        msg.positions_meters = [0.132612268] * 6
        msg.raw_steps = [1000.0] * 6
        msg.fault_codes = [0] * 6
        self.publisher.publish(msg)
        self.sequence += 1


class TelemetryPublisher(Node):
    """30 Hz synthetic timed feedback, with optional deterministic anomalies."""

    def __init__(self, name: str, anomalies: bool) -> None:
        super().__init__(name)
        self.kinematics = Analytical6PusKinematics()
        self.publisher = self.create_publisher(
            TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data
        )
        self.anomalies = anomalies
        self.sequence = 0
        self.started_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.last_end_ns = self.started_ns
        self.timer = self.create_timer(1.0 / 30.0, self._on_timer)

    def _on_timer(self) -> None:
        now_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        cycle_start_ns = max(now_ns - 24_000_000, self.last_end_ns + 500_000)
        rx_stamps = [cycle_start_ns + i * 4_000_000 for i in range(6)]
        self.last_end_ns = rx_stamps[-1]

        t = (cycle_start_ns - self.started_ns) / 1e9
        pose = [
            0.0,
            0.0,
            self.kinematics.nominal_height + 0.0012 * math.sin(2.0 * math.pi * 0.5 * t),
            0.015 * math.sin(2.0 * math.pi * 0.3 * t),
            0.015 * math.cos(2.0 * math.pi * 0.3 * t),
            0.0,
        ]
        sliders = self.kinematics.inverse_kinematics(pose)

        if self.anomalies and self.sequence == 50:
            sliders = [0.050, 0.050, 0.050, 0.500, 0.050, 0.050]
        elif self.anomalies and self.sequence == 70:
            jumped = list(pose)
            jumped[3] += 0.20
            sliders = self.kinematics.inverse_kinematics(jumped)

        msg = TimedJointFeedback()
        msg.uart_cycle_sequence = self.sequence
        msg.snapshot_start_monotonic_ns = cycle_start_ns
        msg.snapshot_end_monotonic_ns = rx_stamps[-1]
        msg.per_axis_rx_done_monotonic_ns = rx_stamps
        msg.positions_meters = sliders
        msg.raw_steps = [s * 40000.0 for s in sliders]
        msg.fault_codes = [0] * 6
        self.publisher.publish(msg)
        self.sequence += 1


def _quantiles_ns(values: list[int]) -> dict[str, float]:
    if not values:
        return {"p50_ms": float("nan"), "p95_ms": float("nan"),
                "p99_ms": float("nan"), "max_ms": float("nan")}
    values_ms = np.asarray(values, dtype=np.float64) / 1e6
    return {
        "p50_ms": float(np.percentile(values_ms, 50)),
        "p95_ms": float(np.percentile(values_ms, 95)),
        "p99_ms": float(np.percentile(values_ms, 99)),
        "max_ms": float(np.max(values_ms)),
    }


def _stats_from_starts(
    starts_ns: list[int], durations_ns: list[int], elapsed_s: float
) -> dict[str, Any]:
    periods_ns = np.diff(np.asarray(starts_ns, dtype=np.int64)).tolist()
    period_q = _quantiles_ns(periods_ns)
    duration_q = _quantiles_ns(durations_ns)
    return {
        "callback_count": len(starts_ns),
        "mean_rate_hz": float(len(starts_ns) / elapsed_s) if elapsed_s > 0 else 0.0,
        "mean_period_ms": (
            float(np.mean(np.asarray(periods_ns, dtype=np.float64)) / 1e6)
            if periods_ns else float("nan")
        ),
        "period": period_q,
        "callback_duration": duration_q,
    }


def run_mode(label: str, duration_s: float, executor_kind: str = "multi") -> dict[str, Any]:
    nodes: list[Node] = []
    stats_node: EmptyTimerNode | None = None
    fk_node: Pus6FkNode | None = None

    if label == "A":
        stats_node = EmptyTimerNode("j5_ablation_a_timer")
        nodes = [stats_node]
    elif label == "B":
        stats_node = EmptyTimerNode("j5_ablation_b_timer")
        nodes = [stats_node, PlainPublisher("j5_ablation_b_publisher")]
    else:
        kwargs = {
            "node_name": f"j5_ablation_{label.lower()}_fk",
            "perform_fk": label != "C",
            "publish_fk": label in ("E", "F"),
            "log_diagnostics": label == "F",
        }
        fk_node = Pus6FkNode(**kwargs)
        nodes = [
            fk_node,
            TelemetryPublisher(
                f"j5_ablation_{label.lower()}_telemetry", anomalies=label == "F"
            ),
        ]

    executor = (
        SingleThreadedExecutor()
        if executor_kind == "single"
        else MultiThreadedExecutor(num_threads=4)
    )
    for node in nodes:
        executor.add_node(node)
    spin_errors: list[BaseException] = []
    stopping = threading.Event()

    def _spin() -> None:
        try:
            executor.spin()
        except BaseException as exc:
            if not (stopping.is_set() and type(exc).__name__ == "InvalidHandle"):
                spin_errors.append(exc)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    started = time.monotonic()
    spin_thread.start()
    time.sleep(duration_s)
    elapsed = time.monotonic() - started
    stopping.set()
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    if spin_errors:
        raise spin_errors[0]

    if fk_node is not None:
        metrics = fk_node.metrics
        starts = [0]
        starts.extend(np.cumsum(metrics["timer_periods_ns"]).astype(np.int64).tolist())
        result = _stats_from_starts(
            starts,
            metrics["callback_durations_ns"],
            elapsed,
        )
        result.update({
            "query_count": metrics["total_queries"],
            "synced_count": metrics["synced_count"],
            "fk_success_count": metrics["fk_success_count"],
            "fk_fail_count": metrics["fk_fail_count"],
            "jump_reject_count": metrics["jump_reject_count"],
            "feedback_callback_duration": _quantiles_ns(metrics["feedback_callback_durations_ns"]),
            "feedback_callback_count": metrics["feedback_callback_count"],
        })
    else:
        result = _stats_from_starts(
            stats_node.stats.starts_ns, stats_node.stats.durations_ns, elapsed
        )

    for node in nodes:
        node.destroy_node()
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--executor", choices=("multi", "single"), default="multi")
    args = parser.parse_args()

    rclpy.init()
    try:
        results = {label: run_mode(label, args.duration, args.executor) for label in "ABCDEF"}
    finally:
        rclpy.shutdown()

    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True, allow_nan=True))
    else:
        print("label count rate_hz mean_period_ms p50 p95 p99 max callback_p50 callback_p95 callback_p99 callback_max")
        for label, result in results.items():
            p = result["period"]
            d = result["callback_duration"]
            print(
                f"{label} {result['callback_count']} {result['mean_rate_hz']:.2f} "
                f"{result['mean_period_ms']:.3f} {p['p50_ms']:.3f} {p['p95_ms']:.3f} "
                f"{p['p99_ms']:.3f} {p['max_ms']:.3f} {d['p50_ms']:.3f} "
                f"{d['p95_ms']:.3f} {d['p99_ms']:.3f} {d['max_ms']:.3f}"
            )
        print(json.dumps(results, indent=2, sort_keys=True, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
