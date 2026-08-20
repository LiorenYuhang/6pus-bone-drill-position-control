import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode


class SyntheticTelemetryPublisher(Node):
    """Generates strictly monotonic 30 Hz feedback with deterministic anomalies."""

    def __init__(self, publish_rate_hz: float = 30.0):
        super().__init__("synthetic_telemetry_publisher")
        self.kin = Analytical6PusKinematics()
        self.z_nom = self.kin.nominal_height
        self.pub = self.create_publisher(
            TimedJointFeedback, "/pus6/joint_feedback/timed", qos_profile_sensor_data
        )
        self.t_start_mono_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.last_published_end_ns = self.t_start_mono_ns
        self.seq = 0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._on_publish_timer)

    def _on_publish_timer(self):
        t_now_mono_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if t_now_mono_ns <= self.last_published_end_ns:
            t_now_mono_ns = self.last_published_end_ns + 1_000_000

        t_cycle_start = t_now_mono_ns - 24_000_000
        if t_cycle_start <= self.last_published_end_ns:
            t_cycle_start = self.last_published_end_ns + 500_000

        rx_stamps = [t_cycle_start + i * 4_000_000 for i in range(6)]
        self.last_published_end_ns = rx_stamps[-1]

        t_sim_s = (t_cycle_start - self.t_start_mono_ns) / 1e9
        pose = [
            0.0,
            0.0,
            self.z_nom + 0.0012 * math.sin(2.0 * math.pi * 0.5 * t_sim_s),
            0.015 * math.sin(2.0 * math.pi * 0.3 * t_sim_s),
            0.015 * math.cos(2.0 * math.pi * 0.3 * t_sim_s),
            0.0,
        ]
        sliders = self.kin.inverse_kinematics(pose)

        if self.seq in (30, 31):
            self.seq += 1
            return
        if self.seq == 50:
            sliders = [0.050, 0.050, 0.050, 0.500, 0.050, 0.050]
        elif self.seq == 70:
            jumped = list(pose)
            jumped[3] += 0.20
            sliders = self.kin.inverse_kinematics(jumped)

        msg = TimedJointFeedback()
        msg.uart_cycle_sequence = self.seq
        msg.snapshot_start_monotonic_ns = t_cycle_start
        msg.snapshot_end_monotonic_ns = rx_stamps[-1]
        msg.per_axis_rx_done_monotonic_ns = rx_stamps
        msg.positions_meters = sliders
        msg.raw_steps = [s * 40000.0 for s in sliders]
        msg.fault_codes = [0] * 6
        self.pub.publish(msg)
        self.seq += 1


@pytest.fixture(scope="module")
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_fk_node_realtime_200hz_isolated_executor(rclpy_session):
    """Acceptance test for the standalone FK process, including anomaly gates."""
    fk_node = Pus6FkNode(
        node_name="realtime_fk_test_node",
        max_residual_m=1e-4,
        max_pos_jump_m=0.015,
        max_ang_jump_rad=0.15,
        query_rate_hz=200.0,
        fixed_lag_delay_ms=50.0,
    )
    telemetry_pub_node = SyntheticTelemetryPublisher(publish_rate_hz=30.0)

    received_fk_states = []
    listener_node = rclpy.create_node("realtime_fk_listener")
    listener_node.create_subscription(
        FkStateStamped,
        "/pus6/fk/state",
        received_fk_states.append,
        qos_profile_sensor_data,
    )

    executor = SingleThreadedExecutor()
    for node in (fk_node, telemetry_pub_node, listener_node):
        executor.add_node(node)

    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
        metrics = fk_node.metrics
    finally:
        executor.shutdown()
        fk_node.destroy_node()
        telemetry_pub_node.destroy_node()
        listener_node.destroy_node()

    timer_periods_ms = np.asarray(metrics["timer_periods_ns"], dtype=np.float64) / 1e6
    callback_durations_ms = np.asarray(metrics["callback_durations_ns"], dtype=np.float64) / 1e6
    mean_rate_hz = 1000.0 / float(np.mean(timer_periods_ms))

    print("\n200 Hz FK isolated-executor acceptance")
    print(
        f"callbacks={metrics['total_queries']} mean_rate_hz={mean_rate_hz:.2f} "
        f"period_p95_ms={np.percentile(timer_periods_ms, 95):.3f} "
        f"period_p99_ms={np.percentile(timer_periods_ms, 99):.3f} "
        f"period_max_ms={np.max(timer_periods_ms):.3f} "
        f"callback_p95_ms={np.percentile(callback_durations_ms, 95):.3f}"
    )

    assert metrics["total_queries"] >= 1900
    assert mean_rate_hz >= 190.0
    assert np.percentile(timer_periods_ms, 95) < 7.5
    assert np.max(timer_periods_ms) < 100.0
    assert np.percentile(callback_durations_ms, 95) < 5.0
    assert metrics["fk_success_count"] >= 1000
    assert metrics["fk_fail_count"] >= 1
    assert metrics["jump_reject_count"] >= 1
    assert abs(len(received_fk_states) - metrics["fk_success_count"]) <= 2
