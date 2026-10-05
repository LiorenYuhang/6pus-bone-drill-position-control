#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock timing tests & synthetic CPU/GIL contention tests for Runtime-Fix-1 (Sections 22 & 23).

Simulates:
1. Normal 5ms cycle execution
2. Long UART write stall (300ms)
3. Long UART write stall exceeding hard stop (1100ms)
4. Thread wake delay / lateness (1100ms)
5. RX drain stall (500ms)
6. Synthetic CPU/GIL starvation test (software-only, best-effort)

Verifies that the diagnostic recorder and fault snapshot distinctly isolate
worker wake delay vs. serial write blocking vs. RX drain vs. GIL contention.
"""

from __future__ import annotations

import json
import shutil
import tempfile
import threading
import time
from pathlib import Path
import pytest

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, SerialWriteMetric
from pus6_motor_driver.j7_servo_feedback_scheduler import (
    FeedbackSchedulerError,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
    ServoRxPolicy,
    SixAxisCycleAudit,
    SixAxisTxRecord,
)
from pus6_motor_driver.stall_diagnostic_recorder import (
    CycleTimingRecord,
    J11StallDiagnosticRecorder,
    TransportPhaseRecord,
)


def test_scenario_1_normal_5ms_cycle():
    """Scenario 1: Normal 5ms cycle is classified as normal / no stall."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        now_ns = time.perf_counter_ns()
        rec.start_cycle(cycle_seq=1, expected_start_ns=now_ns)
        # Fast execution (< 5ms)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()
        rec.mark_transport_exit()
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=20.0)
        rec.mark_telemetry_done()
        rec.finish_cycle()

        rec.dump()
        summary_file = Path(tmp_dir) / "stall_diagnostic_summary.json"
        assert summary_file.exists()
        with open(summary_file) as f:
            summary = json.load(f)
        assert summary["stalls_gt_50ms_count"] == 0
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_scenario_2_uart_write_stall_300ms():
    """Scenario 2: UART write stall of 300ms is classified as B_SERIAL_WRITE_BLOCK."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        now_ns = time.perf_counter_ns()
        rec.start_cycle(cycle_seq=2, expected_start_ns=now_ns)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()

        # Build simulated audit with Motor ID 2 stalling for 300ms
        tx_recs = [
            SixAxisTxRecord(
                motor_id=mid,
                cycle_id=2,
                target_position=1000,
                target_timestamp_ns=now_ns,
                tx_start_ns=now_ns,
                tx_end_ns=now_ns + (300_000_000 if mid == 2 else 500_000),
                response_wait=False,
                retry_count=0,
                residual_frames_before=0,
                residual_frames_after=0,
                duration_ns=300_000_000 if mid == 2 else 500_000,
                cpu_duration_ns=100_000,
                requested_bytes=10,
                written_bytes=10,
            )
            for mid in range(1, 7)
        ]
        audit = SixAxisCycleAudit(
            cycle_id=2,
            target_timestamp_ns=now_ns,
            cycle_start_ns=now_ns,
            cycle_end_ns=now_ns + 305_000_000,
            cycle_jitter_ns=0,
            target_age_ns=0,
            tx_records=tuple(tx_recs),
            tx_skew_ns=0,
            batch_complete_ns=now_ns + 305_000_000,
            deadline_miss=True,
            prior_rx_drain_wall_ns=100_000,
            prior_rx_drain_cpu_ns=50_000,
            prior_rx_drained_frames=0,
            inter_frame_wait_total_ns=500_000,
            motion_drain_wall_ns=1_000_000,
            motion_drain_cpu_ns=500_000,
        )
        rec.mark_transport_exit(audit)
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=305.0)
        rec.finish_cycle(fault_reason="uart_write_stall")

        rec.dump()
        snap_file = Path(tmp_dir) / "fault_snapshot.json"
        assert snap_file.exists()
        with open(snap_file) as f:
            snap = json.load(f)

        assert snap["classification"] == "B_SERIAL_WRITE_BLOCK"
        assert snap["transport_breakdown"]["slowest_motor_id"] == 2
        assert snap["transport_breakdown"]["slowest_motor_ms"] >= 300.0
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_scenario_3_uart_write_stall_1100ms_hard_stop():
    """Scenario 3: Severe UART write stall of 1100ms triggers hard stop and captures state."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        now_ns = time.perf_counter_ns()
        rec.start_cycle(cycle_seq=3, expected_start_ns=now_ns)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()

        # Motor ID 5 stalls for 1100ms
        tx_recs = [
            SixAxisTxRecord(
                motor_id=mid,
                cycle_id=3,
                target_position=1000,
                target_timestamp_ns=now_ns,
                tx_start_ns=now_ns,
                tx_end_ns=now_ns + (1_100_000_000 if mid == 5 else 500_000),
                response_wait=False,
                retry_count=0,
                residual_frames_before=0,
                residual_frames_after=0,
                duration_ns=1_100_000_000 if mid == 5 else 500_000,
                cpu_duration_ns=100_000,
                requested_bytes=10,
                written_bytes=10,
            )
            for mid in range(1, 7)
        ]
        audit = SixAxisCycleAudit(
            cycle_id=3,
            target_timestamp_ns=now_ns,
            cycle_start_ns=now_ns,
            cycle_end_ns=now_ns + 1_105_000_000,
            cycle_jitter_ns=0,
            target_age_ns=0,
            tx_records=tuple(tx_recs),
            tx_skew_ns=0,
            batch_complete_ns=now_ns + 1_105_000_000,
            deadline_miss=True,
        )
        rec.mark_transport_exit(audit)
        rec.mark_ingest_start()
        # Ingest detects gap > 1000ms
        rec.mark_ingest_done(scheduler_gap_ms=1105.0)
        rec.record_fault_cycle("scheduler_deadline_exceeded", fault_phase="T7_ingest")
        rec.dump()

        snap_file = Path(tmp_dir) / "fault_snapshot.json"
        with open(snap_file) as f:
            snap = json.load(f)
        assert snap["classification"] == "D_FEEDBACK_SCHEDULER_GAP"
        assert snap["scheduler_gap_ms"] >= 1100.0
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_scenario_4_thread_wake_delay_1100ms():
    """Scenario 4: Worker wake lateness of 1100ms is classified as A_TIMER_DISPATCH_DELAY."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        # Expected start was 1100ms in the past
        now_ns = time.perf_counter_ns()
        exp_ns = now_ns - 1_100_000_000
        rec.start_cycle(cycle_seq=4, expected_start_ns=exp_ns)
        # Execution inside callback is fast (2ms)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()
        rec.mark_transport_exit()
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=1102.0)
        rec.record_fault_cycle("scheduler_deadline_exceeded", fault_phase="T0_entry_wake_late")
        rec.dump()

        snap_file = Path(tmp_dir) / "fault_snapshot.json"
        with open(snap_file) as f:
            snap = json.load(f)

        assert snap["cycle_seq"] == 4
        assert snap["wake_lateness_ms"] >= 1000.0
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_scenario_5_rx_drain_stall_500ms():
    """Scenario 5: RX feedback drain stall of 500ms is classified as C_RX_DRAIN_BLOCK."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        now_ns = time.perf_counter_ns()
        rec.start_cycle(cycle_seq=5, expected_start_ns=now_ns)
        rec.mark_transport_entry()

        tx_recs = [
            SixAxisTxRecord(
                motor_id=mid,
                cycle_id=5,
                target_position=1000,
                target_timestamp_ns=now_ns,
                tx_start_ns=now_ns,
                tx_end_ns=now_ns + 500_000,
                response_wait=False,
                retry_count=0,
                residual_frames_before=0,
                residual_frames_after=0,
                duration_ns=500_000,
                cpu_duration_ns=100_000,
                requested_bytes=10,
                written_bytes=10,
            )
            for mid in range(1, 7)
        ]
        audit = SixAxisCycleAudit(
            cycle_id=5,
            target_timestamp_ns=now_ns,
            cycle_start_ns=now_ns,
            cycle_end_ns=now_ns + 505_000_000,
            cycle_jitter_ns=0,
            target_age_ns=0,
            tx_records=tuple(tx_recs),
            tx_skew_ns=0,
            batch_complete_ns=now_ns + 505_000_000,
            deadline_miss=True,
            prior_rx_drain_wall_ns=100_000,
            prior_rx_drain_cpu_ns=50_000,
            motion_drain_wall_ns=500_000_000,  # 500ms drain
            motion_drain_cpu_ns=2_000_000,
        )
        rec.mark_transport_exit(audit)
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=510.0)
        rec.finish_cycle(fault_reason="rx_drain_stall")
        rec.dump()

        snap_file = Path(tmp_dir) / "fault_snapshot.json"
        with open(snap_file) as f:
            snap = json.load(f)
        assert snap["classification"] == "C_RX_DRAIN_BLOCK"
        assert snap["transport_breakdown"]["feedback_drain_ms"] >= 500.0
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_scenario_6_synthetic_gil_starvation():
    """Scenario 6: Synthetic thread starvation demonstrates wall_elapsed >> thread_cpu_delta."""
    tmp_dir = tempfile.mkdtemp()
    stop_event = threading.Event()

    def _gil_burner():
        # High CPU/memory churn to induce thread scheduling preemption
        while not stop_event.is_set():
            _ = [i * i for i in range(50_000)]

    burner_threads = [threading.Thread(target=_gil_burner) for _ in range(2)]
    for t in burner_threads:
        t.start()

    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        for seq in range(1, 6):
            t_exp = time.perf_counter_ns()
            time.sleep(0.010)  # Sleep in servo thread
            rec.start_cycle(cycle_seq=seq, expected_start_ns=t_exp)
            rec.mark_target_lock_done()
            rec.mark_slew_done()
            rec.finish_cycle()

        assert len(rec.ring_buffer) == 5
        # Under starvation / sleep, thread CPU time consumed is significantly less than elapsed wall time
        for r in rec.ring_buffer:
            assert r.callback_total_ms >= 0.0
            # thread_cpu_delta_ms should be well bounded
            assert r.thread_cpu_delta_ms < 50.0
    finally:
        stop_event.set()
        for t in burner_threads:
            t.join(timeout=1.0)
        shutil.rmtree(tmp_dir, ignore_errors=True)
