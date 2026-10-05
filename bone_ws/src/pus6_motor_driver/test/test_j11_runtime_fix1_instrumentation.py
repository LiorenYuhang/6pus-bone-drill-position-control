#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for Runtime-Fix-1 Instrumentation (A through H).

Verifies:
A. Normal cycle ring buffer record correct.
B. Wake lateness single-cycle, not cumulative drift.
C. scheduler_gap real value in diagnostic.
D. Hard-stop faulting cycle exists before dump.
E. Transport phase 6 motor writes distinguished.
F. Serial write exception phase / motor id correct.
G. Ring buffer max length bounded (does not grow indefinitely).
H. Instrumentation disabled: normal runtime behavior unchanged.
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
from pathlib import Path
import pytest

from pus6_motor_driver.fake_bus import FakeBus, MockServoBus
from pus6_motor_driver.j7_real_uart_adapter import J7AdapterError, J7UartAdapter, SerialWriteMetric
from pus6_motor_driver.j7_servo_feedback_scheduler import (
    FeedbackSchedulerError,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
    ServoRxPolicy,
    ServoTransportError,
    SixAxisCycleAudit,
)
from pus6_motor_driver.protocol import StatusFeedback
from pus6_motor_driver.safety import MOTOR_IDS
from pus6_motor_driver.stall_diagnostic_recorder import (
    CycleTimingRecord,
    J11StallDiagnosticRecorder,
    TransportPhaseRecord,
)


class SyntheticClock:
    def __init__(self, start_ns: int = 1_000_000_000) -> None:
        self.now = start_ns

    def now_ns(self) -> int:
        return self.now

    def advance_ms(self, ms: float) -> None:
        self.now += int(ms * 1e6)


class MockHardwareSerial:
    """Serial-like device backed by FakeBus for testing J7UartAdapter."""

    def __init__(self) -> None:
        self.device = FakeBus()
        self.rx = bytearray()
        self.written_chunks: list[bytes] = []

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, data: bytes) -> int:
        self.written_chunks.append(data)
        resp = self.device.request(data)
        if resp:
            self.rx.extend(resp)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data


class FailingMockSerial:
    """Mock serial device that fails on a specific motor ID write."""

    def __init__(self, fail_on_motor_id: int = 4) -> None:
        self.fail_on_motor_id = fail_on_motor_id
        self.rx = bytearray()
        self.written_chunks: list[bytes] = []

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, data: bytes) -> int:
        motor_id = data[3] if len(data) > 3 else 0
        if motor_id == self.fail_on_motor_id:
            raise OSError(f"Simulated write error on motor {motor_id}")
        self.written_chunks.append(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data


def test_a_normal_cycle_ring_buffer_record_correct():
    """Test A: Normal cycle records are correctly populated in ring buffer."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(buffer_size=10, output_dir=tmp_dir)
        now_ns = time.perf_counter_ns()
        rec.start_cycle(cycle_seq=1, expected_start_ns=now_ns)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()
        rec.mark_transport_exit()
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=20.0)
        rec.mark_telemetry_done()
        rec.finish_cycle(fault_reason="")

        assert len(rec.ring_buffer) == 1
        entry = rec.ring_buffer[0]
        assert entry.cycle_seq == 1
        assert entry.callback_total_ms >= 0.0
        assert entry.target_lock_wait_ms >= 0.0
        assert entry.slew_processing_ms >= 0.0
        assert entry.preflight_check_ms >= 0.0
        assert entry.pre_transport_ms >= 0.0
        assert entry.scheduler_gap_ms == 20.0
        assert entry.fault_reason == ""
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_b_wake_lateness_single_cycle_not_cumulative_drift():
    """Test B: Wake lateness is evaluated per-cycle and does not drift cumulatively."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(buffer_size=10, output_dir=tmp_dir)

        # Cycle 1: Expected at 100ms
        t1_exp = 100_000_000
        rec.start_cycle(cycle_seq=1, expected_start_ns=t1_exp)
        rec.finish_cycle()
        r1 = rec.ring_buffer[0]
        assert r1.expected_wake_ns == t1_exp
        assert abs(r1.wake_lateness_ms - (r1.actual_entry_ns - t1_exp) / 1e6) < 1e-3

        # Cycle 50: Expected at 500ms, independent of cycle 1
        t50_exp = 500_000_000
        rec.start_cycle(cycle_seq=50, expected_start_ns=t50_exp)
        rec.finish_cycle()
        r50 = rec.ring_buffer[1]
        assert r50.expected_wake_ns == t50_exp
        assert abs(r50.wake_lateness_ms - (r50.actual_entry_ns - t50_exp) / 1e6) < 1e-3
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_c_scheduler_gap_real_value_enters_diagnostic():
    """Test C: Real scheduler_gap_ns value enters FeedbackCycleAudit and diagnostic record."""
    clk = SyntheticClock(start_ns=1_000_000_000)
    bus = FakeBus()
    writer = J7SixAxisServoTransport(
        bus,
        cycle_rate_hz=50.0,
        clock_ns=clk.now_ns,
    )
    scheduler = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=1e9,
        servo_cycle_period_s=0.020,
        clock_ns=clk.now_ns,
        feedback_loss_policy="age_based",
    )
    # Full scan initializes all axes and arms watchdog
    audit_init = scheduler.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.feedback_age_watchdog_armed

    # Advance clock by 185 ms
    clk.advance_ms(185.0)

    # Ingest cycle feedback
    feedbacks = {
        mid: StatusFeedback(
            target_position=1000,
            current_position=1000,
            current_mA=100,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=0,
        )
        for mid in MOTOR_IDS
    }
    audit = scheduler.ingest_cycle_feedback(
        feedbacks,
        servo_cycle_id=1,
        servo_cycle_timestamp_ns=clk.now_ns(),
    )

    # scheduler_gap_ns MUST be populated with exactly 185ms (185_000_000 ns)
    assert audit.scheduler_gap_ns == 185_000_000
    assert scheduler.last_scheduler_gap_ns == 185_000_000

    # Verify diagnostic recorder correctly accepts and converts this gap
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        rec.start_cycle(cycle_seq=1)
        rec.mark_ingest_start()
        rec.mark_ingest_done(scheduler_gap_ms=audit.scheduler_gap_ns / 1e6)
        rec.finish_cycle()
        assert abs(rec.ring_buffer[0].scheduler_gap_ms - 185.0) < 1e-3
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_d_hard_stop_faulting_cycle_exists_before_dump():
    """Test D: The cycle triggering a fault exists in the ring buffer BEFORE dump."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(buffer_size=10, output_dir=tmp_dir)
        # Cycle 1: normal
        rec.start_cycle(cycle_seq=1)
        rec.finish_cycle()
        assert len(rec.ring_buffer) == 1

        # Cycle 2: encounters fault during execution (in-flight)
        rec.start_cycle(cycle_seq=2)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()
        # Simulated fault happens inside transport!
        rec.record_fault_cycle("motion_write_failure", fault_phase="T4_transport")

        # Cycle 2 MUST already be in ring_buffer before dump is called!
        assert len(rec.ring_buffer) == 2
        faulting_cycle = rec.ring_buffer[-1]
        assert faulting_cycle.cycle_seq == 2
        assert faulting_cycle.fault_reason == "motion_write_failure"
        assert faulting_cycle.fault_phase == "T4_transport"
        assert faulting_cycle.callback_total_ms > 0.0

        # Now call dump()
        rec.dump()

        # Verify fault_snapshot.json was written and contains the faulting cycle
        snap_file = Path(tmp_dir) / "fault_snapshot.json"
        assert snap_file.exists()
        with open(snap_file) as f:
            snap = json.load(f)
        assert snap["cycle_seq"] == 2
        assert snap["fault_reason"] == "motion_write_failure"
        assert snap["fault_phase"] == "T4_transport"

        # Verify cycle_timing.csv was written with cycle 2
        csv_file = Path(tmp_dir) / "cycle_timing.csv"
        assert csv_file.exists()
        content = csv_file.read_text()
        assert "motion_write_failure" in content
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_e_transport_phase_6_motor_writes_distinguished():
    """Test E: Transport phase breakdown correctly distinguishes 6 motor writes."""
    mock_serial = MockHardwareSerial()
    adapter = J7UartAdapter(mock_serial, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0)
    transport.configure_servo(speed=50)

    audit = transport.send_cycle([1000] * 6, cycle_id=10)

    # 6 Tx records
    assert len(audit.tx_records) == 6
    for i, tx in enumerate(audit.tx_records):
        assert tx.motor_id == i + 1
        assert tx.duration_ns >= 0
        assert tx.requested_bytes == 10
        assert tx.written_bytes == 10

    # Feed to diagnostic recorder
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(output_dir=tmp_dir)
        rec.start_cycle(cycle_seq=10)
        rec.mark_transport_entry()
        rec.mark_transport_exit(audit)
        rec.finish_cycle()

        assert len(rec.transport_ring_buffer) == 1
        tr = rec.transport_ring_buffer[0]
        assert tr.cycle_seq == 10
        assert tr.m1_req_bytes == 10
        assert tr.m6_req_bytes == 10
        assert tr.prior_rx_drain_wall_ms >= 0.0

        rec.dump()
        trans_csv = Path(tmp_dir) / "transport_phase_timing.csv"
        assert trans_csv.exists()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_f_serial_write_exception_phase_and_motor_id_correct():
    """Test F: Serial write exception records motor id and phase without altering exception."""
    failing_serial = FailingMockSerial(fail_on_motor_id=4)
    adapter = J7UartAdapter(failing_serial, baudrate=921600, timeout_s=0.010, min_frame_interval_s=0.0001)
    transport = J7SixAxisServoTransport(adapter, cycle_rate_hz=50.0)
    # Manually mark configured to skip configure writes
    transport._configured = tuple()

    with pytest.raises(J7AdapterError, match="Simulated write error on motor 4"):
        transport.send_cycle([1000] * 6, cycle_id=42)

    assert transport.stopped
    assert transport._stop_reason == "motion_write_failure"


def test_g_ring_buffer_bounded_at_max_length():
    """Test G: Ring buffers never grow indefinitely past buffer_size."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(buffer_size=5, output_dir=tmp_dir)
        for i in range(25):
            rec.start_cycle(cycle_seq=i)
            rec.finish_cycle()

        assert len(rec.ring_buffer) == 5
        assert len(rec.transport_ring_buffer) == 5
        # The oldest elements should have been dropped
        assert rec.ring_buffer[0].cycle_seq == 20
        assert rec.ring_buffer[-1].cycle_seq == 24
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_h_instrumentation_disabled_zero_overhead():
    """Test H: When instrumentation is disabled, no records are saved and operations no-op."""
    tmp_dir = tempfile.mkdtemp()
    try:
        rec = J11StallDiagnosticRecorder(buffer_size=10, output_dir=tmp_dir, enabled=False)
        rec.start_cycle(cycle_seq=1)
        rec.mark_target_lock_done()
        rec.mark_slew_done()
        rec.mark_preflight_done()
        rec.mark_transport_entry()
        rec.mark_transport_exit()
        rec.mark_ingest_start()
        rec.mark_ingest_done(20.0)
        rec.finish_cycle()

        assert len(rec.ring_buffer) == 0
        assert len(rec.transport_ring_buffer) == 0
        assert rec._curr is None
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
