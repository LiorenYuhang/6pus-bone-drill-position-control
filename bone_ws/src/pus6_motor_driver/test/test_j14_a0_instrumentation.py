#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J14-A0: Instrumentation Unit and Regression Tests."""

import csv
import os
import sys
import time
from pathlib import Path

import pytest
import rclpy

from pus6_motor_driver.stall_diagnostic_recorder import (
    J11StallDiagnosticRecorder,
    CycleTimingRecord,
    TransportPhaseRecord,
)
from pus6_motor_driver.j7_real_uart_adapter import RxDrainMetrics


def test_tier1_summary_legacy_prefix_and_rx_columns_dump(tmp_path: Path):
    rec = J11StallDiagnosticRecorder(buffer_size=10, output_dir=str(tmp_path))
    rec.start_cycle(cycle_seq=1, expected_start_ns=time.perf_counter_ns())
    rec.mark_target_lock_done()
    rec.mark_slew_done()
    rec.mark_preflight_done()
    rec.mark_transport_entry()
    rec.mark_transport_exit()
    rec.mark_ingest_start()
    rec.record_warning_log_duration(1.45)
    rec.mark_ingest_done(scheduler_gap_ms=12.5)
    rec.finish_cycle(fault_reason="", feedback_age_max_ms=15.2)

    rec.dump()

    csv_path = tmp_path / "cycle_summary_full.csv"
    assert csv_path.exists(), "cycle_summary_full.csv must exist after dump()"

    with open(csv_path, "r", newline="") as f:
        reader = list(csv.reader(f))
        assert len(reader) == 2  # header + 1 row
        header = reader[0]
        expected_header = [
            "cycle_seq",
            "t_entry_ns",
            "wake_lateness_ms",
            "inter_gap_ms",
            "cb_total_ms",
            "uart_tx_ms",
            "uart_drain_ms",
            "rx_drain_calls",
            "rx_drain_bytes",
            "ros_pub_total_ms",
            "feedback_age_max_ms",
            "warn_log_dur_ms",
            "fault_reason",
            "rx_max_single_read_wall_ms",
            "rx_max_single_read_thread_cpu_ms",
            "rx_max_single_read_process_cpu_ms",
            "rx_max_read_vol_ctx_delta",
            "rx_max_read_invol_ctx_delta",
            "rx_read_call_count",
            "rx_bytes",
        ]
        assert header == expected_header, f"Header mismatch: {header} vs {expected_header}"
        row = reader[1]
        assert row[0] == "1"
        assert float(row[10]) == 15.2
        assert float(row[11]) == 1.45


def test_host_heartbeat_observer_flush(tmp_path: Path):
    sys.path.insert(0, "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts")
    from j13_ladrc_staged_launcher import HostHeartbeatObserver

    csv_path = tmp_path / "host_heartbeat_stalls.csv"
    observer = HostHeartbeatObserver(csv_path, interval_s=0.005, threshold_delay_ms=0.01)
    observer.start()
    time.sleep(0.02)
    observer.stop_and_flush()

    assert csv_path.exists()
    with open(csv_path, "r", newline="") as f:
        reader = list(csv.reader(f))
        header = reader[0]
        assert header == [
            "heartbeat_seq",
            "timestamp_ns",
            "expected_ns",
            "wake_ns",
            "delay_ms",
            "system_load_1m",
            "process_count",
        ]


def test_runner_zero_hardware_writes_in_dry_run():
    sys.path.insert(0, "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts")
    from j13_ladrc_shadow_runner import J13LadrcShadowRunner

    if not rclpy.ok():
        rclpy.init()

    try:
        runner = J13LadrcShadowRunner(execute_motion=False, dry_run=True, auto_start=False)
        assert runner.hardware_write_count == 0
        assert runner.dry_run is True
        assert runner.execute_motion is False

        # Simulate tick in DISARMED state
        runner._on_control_tick()
        assert runner.hardware_write_count == 0
        assert len(runner.telemetry_history) == 1
        telem = runner.telemetry_history[0]
        assert telem.hardware_write_issued is False
        assert hasattr(telem, "runner_wake_lateness_ms")
        assert hasattr(telem, "console_flush_dur_ms")
        assert hasattr(telem, "delta_t_bus_ms")
        assert hasattr(telem, "delta_t_driver_proc_ms")
        assert hasattr(telem, "delta_t_dds_transport_ms")
        assert hasattr(telem, "timed_feedback_age_at_tick_ms")
        assert telem.delta_t_runner_queue_ms == telem.timed_feedback_age_at_tick_ms
        assert hasattr(telem, "timed_fb_seq")
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_tier1_uses_existing_feedback_sample_age(tmp_path, monkeypatch):
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_motor_driver.fake_bus import MockServoBus
    from pus6_motor_driver.safety import HardwareMode

    if not rclpy.ok():
        rclpy.init()
    import serial
    physical_opens = []
    def forbid_open(*args, **kwargs):
        physical_opens.append(args)
        raise AssertionError("Mock telemetry test must not open serial hardware")
    monkeypatch.setattr(serial, "Serial", forbid_open)
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(hardware_mode=HardwareMode.MOCK, bus=bus)
    try:
        node._stall_recorder = J11StallDiagnosticRecorder(output_dir=str(tmp_path))
        node._execute_servo_cycle()
        time.sleep(0.025)
        node._execute_servo_cycle()
        audit = node.j7_feedback.audit_snapshot().cycles[-1]
        expected = round(max(s.feedback_age_ns for s in audit.samples) / 1e6, 1)
        assert expected >= 25.0
        assert node._stall_recorder.tier1_summary[-1][10] == expected
        assert physical_opens == []
        assert node.mock_bus is bus
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_timed_feedback_age_is_reception_staleness():
    from types import SimpleNamespace
    from j13_ladrc_shadow_runner import J13LadrcShadowRunner

    if not rclpy.ok():
        rclpy.init()
    runner = J13LadrcShadowRunner(execute_motion=False, dry_run=True, auto_start=False)
    try:
        runner.latest_timed_fb = SimpleNamespace(
            snapshot_start_monotonic_ns=1, snapshot_end_monotonic_ns=2,
            per_axis_rx_done_monotonic_ns=[2]*6, uart_cycle_sequence=42,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
        )
        runner.latest_timed_fb_rx_mono_ns = time.perf_counter_ns() - 333_000_000
        runner.latest_timed_fb_rx_ros_ns = 0
        runner._on_control_tick()
        telem = runner.telemetry_history[-1]
        assert 333 <= telem.timed_feedback_age_at_tick_ms < 350
        assert telem.delta_t_runner_queue_ms == telem.timed_feedback_age_at_tick_ms
        assert telem.timed_fb_seq == 42
        assert runner.hardware_write_count == 0
    finally:
        runner.destroy_node()
        rclpy.shutdown()
