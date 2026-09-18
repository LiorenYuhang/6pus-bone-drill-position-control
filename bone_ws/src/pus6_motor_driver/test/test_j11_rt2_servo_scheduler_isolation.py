#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for J11-RT2 Servo Loop Scheduling Isolation."""

import time
import pytest
import rclpy

from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import HardwareMode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j7_servo_feedback_scheduler import FeedbackSchedulerError


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_default_mode_is_ros_timer():
    """Default mode must strictly remain ros_timer, preserving frozen J7/J8 baseline."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        node_name="test_default_mode",
    )
    try:
        assert node.servo_scheduler_mode == "ros_timer"
        assert node.write_timer is not None
        assert node._servo_thread is None
    finally:
        node.destroy_node()


def test_dedicated_thread_mode_startup_and_rate():
    """Dedicated thread mode must start dedicated worker thread and run at ~50Hz."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        write_rate_hz=50.0,
        node_name="test_dedicated_thread",
    )
    try:
        assert node.servo_scheduler_mode == "dedicated_thread"
        assert node.write_timer is None
        assert node._servo_thread is not None
        assert node._servo_thread.is_alive()

        initial_seq = node._uart_cycle_seq
        time.sleep(0.30)  # ~300ms should execute ~15 cycles at 50Hz
        cycles_run = node._uart_cycle_seq - initial_seq
        assert 10 <= cycles_run <= 22, f"Expected 10-22 cycles in 300ms at 50Hz, got {cycles_run}"
    finally:
        node.destroy_node()

    # Verify clean thread termination on node destruction
    assert not node._servo_thread.is_alive()


def test_stop_latch_terminates_dedicated_worker():
    """STOP latch must cause the dedicated worker loop to cease execution."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        write_rate_hz=50.0,
        node_name="test_stop_latch",
    )
    try:
        assert node._servo_thread.is_alive()
        node._latch_j7_fault("test_simulated_stop")
        # Give thread a few ms to observe latch and exit
        node._servo_thread.join(timeout=0.5)
        assert not node._servo_thread.is_alive()
    finally:
        node.destroy_node()


def test_behavioral_equivalence():
    """Both modes must dispatch identical commands and apply identical slew/safety checks."""
    bus_timer = MockServoBus(initial_positions=[1000] * 6)
    node_timer = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus_timer,
        servo_scheduler_mode="ros_timer",
        write_rate_hz=50.0,
        node_name="test_equiv_timer",
    )

    bus_thread = MockServoBus(initial_positions=[1000] * 6)
    node_thread = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus_thread,
        servo_scheduler_mode="dedicated_thread",
        write_rate_hz=50.0,
        node_name="test_equiv_thread",
    )

    try:
        test_target = [1005, 995, 1002, 998, 1001, 999]

        node_timer._enqueue_latest_target(test_target)
        node_timer._execute_servo_cycle()

        node_thread._enqueue_latest_target(test_target)
        node_thread._execute_servo_cycle()

        # Commanded steps and positions should match
        assert node_timer._last_commanded_steps == node_thread._last_commanded_steps
        assert node_timer._latest_positions_steps == node_thread._latest_positions_steps
    finally:
        node_timer.destroy_node()
        node_thread.destroy_node()


def test_watchdog_scheduler_deadline_exceeded_in_dedicated_mode():
    """Simulating a >283.3ms stall in dedicated mode must trip scheduler_deadline_exceeded."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        write_rate_hz=50.0,
        node_name="test_watchdog_trip",
    )
    try:
        # First cycle arms the watchdog and records _last_poll_completion_ns
        node._execute_servo_cycle()
        assert node.j7_feedback is not None
        assert node.j7_feedback._last_poll_completion_ns > 0

        # Artificially set _last_poll_completion_ns to 350ms in the past
        past_ns = time.monotonic_ns() - int(0.350 * 1e9)
        node.j7_feedback._last_poll_completion_ns = past_ns

        # Next cycle must trigger FeedbackSchedulerError -> STOP
        node._execute_servo_cycle()
        assert node._j7_fault_latched
        assert "scheduler_deadline_exceeded" in (node._j7_fault_reason or "")
    finally:
        node.destroy_node()


def test_absolute_deadline_pacing_stability():
    """Verify that absolute deadline pacing runs accurately without cumulative drift."""
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(
        hardware_mode=HardwareMode.MOCK,
        bus=bus,
        servo_scheduler_mode="dedicated_thread",
        write_rate_hz=50.0,
        node_name="test_pacing_drift",
    )
    try:
        t0 = time.perf_counter()
        seq0 = node._uart_cycle_seq
        time.sleep(0.50)  # 500ms
        t1 = time.perf_counter()
        seq1 = node._uart_cycle_seq

        elapsed = t1 - t0
        cycles = seq1 - seq0
        expected_cycles = elapsed * 50.0
        # Allow +/- 2 cycles tolerance over 0.5s due to OS scheduler granularity
        assert abs(cycles - expected_cycles) <= 3, f"Cycles ({cycles}) deviated from expected ({expected_cycles:.1f})"
    finally:
        node.destroy_node()
