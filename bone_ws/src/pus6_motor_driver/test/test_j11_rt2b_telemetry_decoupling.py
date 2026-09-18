#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for J11-RT2B Servo Telemetry Publication Decoupling."""

import threading
import time
import pytest
import rclpy

from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode, ServoTelemetrySnapshot
from pus6_motor_driver.safety import HardwareMode
from pus6_motor_driver.fake_bus import MockServoBus


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_default_telemetry_publish_mode_is_inline():
    """Default mode must strictly remain inline for ros_timer, preserving frozen J7/J8 baseline."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="ros_timer",
        telemetry_publish_mode="auto",
        node_name="test_default_telem",
    )
    try:
        assert node.telemetry_publish_mode == "inline"
        assert node._telemetry_thread is None
    finally:
        node.destroy_node()


def test_async_latest_telemetry_thread_lifecycle():
    """dedicated_thread + auto must resolve to async_latest and start Pus6TelemetryThread."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="auto",
        write_rate_hz=50.0,
        node_name="test_async_lifecycle",
    )
    try:
        assert node.telemetry_publish_mode == "async_latest"
        assert node._telemetry_thread is not None
        assert node._telemetry_thread.is_alive()
        assert node._telemetry_thread.name == "Pus6TelemetryThread"
    finally:
        node.destroy_node()

    # Clean shutdown and join
    assert not node._telemetry_thread.is_alive()
    assert not node._servo_thread.is_alive()


def test_servo_cycle_does_not_call_publish_directly_in_async_mode():
    """In async_latest mode, ROS publish must happen in Pus6TelemetryThread, never in Servo Thread."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="async_latest",
        write_rate_hz=50.0,
        node_name="test_thread_isolation",
    )

    publishing_thread_names = []
    orig_pub = node.joint_state_pub.publish

    def mock_pub(msg):
        publishing_thread_names.append(threading.current_thread().name)
        orig_pub(msg)

    node.joint_state_pub.publish = mock_pub

    try:
        time.sleep(0.15)  # Let ~7 cycles run
        assert len(publishing_thread_names) > 0
        # All publishes must originate from Pus6TelemetryThread
        for th_name in publishing_thread_names:
            assert th_name == "Pus6TelemetryThread", f"Publish occurred on unexpected thread: {th_name}"
            assert th_name != "Pus6ServoThread", "Servo Thread illegally executed ROS publish directly!"
    finally:
        node.destroy_node()


def test_simulated_350ms_publish_stall_does_not_stall_servo_thread():
    """Core RT2B Requirement: When publish() blocks for 350ms, Servo Thread continues at ~50Hz."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="async_latest",
        write_rate_hz=50.0,
        node_name="test_350ms_stall",
    )

    stall_injected = False
    stall_dur_ms = 0.0
    servo_cycle_timestamps = []

    # Wrap send_cycle to accurately timestamp servo thread executions
    orig_send_cycle = node.j7_writer.send_cycle

    def wrapped_send_cycle(*args, **kwargs):
        servo_cycle_timestamps.append(time.perf_counter_ns())
        return orig_send_cycle(*args, **kwargs)

    node.j7_writer.send_cycle = wrapped_send_cycle

    orig_pub = node.joint_state_pub.publish

    def blocking_pub(msg):
        nonlocal stall_injected, stall_dur_ms
        if not stall_injected and len(servo_cycle_timestamps) >= 5:
            stall_injected = True
            t0 = time.perf_counter()
            time.sleep(0.350)  # Inject 350ms DDS subscriber matching stall
            stall_dur_ms = (time.perf_counter() - t0) * 1000.0
        orig_pub(msg)

    node.joint_state_pub.publish = blocking_pub

    try:
        # Run for ~600ms total
        time.sleep(0.60)

        assert stall_injected, "Simulated 350ms stall was not injected"
        assert stall_dur_ms >= 340.0, f"Stall duration was {stall_dur_ms:.1f}ms, expected >= 340ms"

        # Calculate inter-cycle gaps in the servo thread
        gaps_ms = []
        for i in range(1, len(servo_cycle_timestamps)):
            gap = (servo_cycle_timestamps[i] - servo_cycle_timestamps[i - 1]) / 1e6
            gaps_ms.append(gap)

        servo_max_gap = max(gaps_ms) if gaps_ms else 0.0

        # Output the required check metrics
        print(f"\n=======================================================")
        print(f"Telemetry publish stall ≈ {stall_dur_ms:.1f}ms")
        print(f"Servo max inter-cycle gap = {servo_max_gap:.1f}ms")
        print(f"Total servo cycles executed = {len(servo_cycle_timestamps)}")
        print(f"=======================================================")

        # Servo thread MUST NOT follow the 350ms stall!
        # Pacing at 50Hz (20ms), even with jitter, max gap must stay well below 50ms (and well below 283.3ms watchdog!)
        assert servo_max_gap < 50.0, (
            f"Servo thread stalled! Max gap was {servo_max_gap:.1f}ms during a {stall_dur_ms:.1f}ms telemetry stall!"
        )
        assert not node._j7_fault_latched, f"Node latched fault: {node._j7_fault_reason}"
        assert node.j7_writer is not None and not node.j7_writer.stopped
    finally:
        node.destroy_node()


def test_latest_only_snapshot_no_backlog_replay():
    """Verify that during telemetry stall, intermediate frames are dropped (zero backlog replay)."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="async_latest",
        write_rate_hz=50.0,
        node_name="test_no_backlog",
    )

    published_seqs = []
    stall_active = threading.Event()
    unblock_event = threading.Event()

    orig_pub = node._publish_telemetry_from_snapshot

    def tracking_pub(snap, record_stall=False):
        published_seqs.append(snap.uart_cycle_seq)
        if not stall_active.is_set():
            stall_active.set()
            # Block telemetry publisher until released
            unblock_event.wait(timeout=1.0)
        orig_pub(snap, record_stall)

    node._publish_telemetry_from_snapshot = tracking_pub

    try:
        # Wait until publisher enters stall on its first publish
        assert stall_active.wait(timeout=0.5), "Telemetry worker did not enter stall"

        first_published_seq = published_seqs[0]

        # Servo thread keeps running while publisher is stalled
        time.sleep(0.25)  # ~12 servo cycles pass
        servo_seq_during_stall = node._uart_cycle_seq
        cycles_elapsed = servo_seq_during_stall - first_published_seq
        assert cycles_elapsed >= 8, f"Expected >= 8 cycles to elapse during stall, got {cycles_elapsed}"

        # Unblock publisher
        unblock_event.set()
        time.sleep(0.04)  # Let publisher process the latest frame

        # In a queue system: published_seqs would contain [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, ...]
        # In latest-only snapshot: published_seqs jumps directly from first_seq to the latest seq!
        assert len(published_seqs) >= 2, "Expected at least 2 publishes after unblock"
        second_published_seq = published_seqs[1]
        skipped_frames = second_published_seq - first_published_seq - 1

        print(f"\nFirst published seq: {first_published_seq}, Second published seq: {second_published_seq}")
        print(f"Intermediate frames dropped (no backlog replay): {skipped_frames} frames")

        # Must have skipped at least 7 frames without replaying them!
        assert skipped_frames >= 7, (
            f"Expected latest-only drop of intermediate frames (skipped >= 7), but only skipped {skipped_frames}"
        )
    finally:
        unblock_event.set()
        node.destroy_node()


def test_stop_latch_immediately_effective_in_servo_thread():
    """STOP latch must immediately stop Servo Thread writes even if telemetry thread is busy."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="async_latest",
        write_rate_hz=50.0,
        node_name="test_stop_latch_rt2b",
    )
    try:
        assert node._servo_thread.is_alive()
        node._latch_j7_fault("test_immediate_safety_trip")
        node._servo_thread.join(timeout=0.5)
        assert not node._servo_thread.is_alive(), "Servo thread did not terminate immediately on STOP latch"
        assert node._j7_fault_latched
    finally:
        node.destroy_node()


def test_motor_command_and_safety_equivalence():
    """Command shaping, limits, and preflight must be 100% equivalent in inline and async_latest modes."""
    bus_inline = MockServoBus(initial_positions=[1000] * 6)
    node_inline = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus_inline,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="inline",
        write_rate_hz=50.0,
        node_name="test_equiv_inline",
    )

    bus_async = MockServoBus(initial_positions=[1000] * 6)
    node_async = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus_async,
        servo_scheduler_mode="dedicated_thread",
        telemetry_publish_mode="async_latest",
        write_rate_hz=50.0,
        node_name="test_equiv_async",
    )

    try:
        time.sleep(0.12)
        # Both should have run ~5-7 cycles without fault
        assert not node_inline._j7_fault_latched
        assert not node_async._j7_fault_latched
        assert abs(node_inline._uart_cycle_seq - node_async._uart_cycle_seq) <= 2
        assert node_inline._last_commanded_steps == node_async._last_commanded_steps
        assert node_inline._latest_positions_steps == node_async._latest_positions_steps
    finally:
        node_inline.destroy_node()
        node_async.destroy_node()
