#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J11/J12 Runtime Stall & Timing Diagnostic Recorder (Runtime-Fix-1).

Zero-overhead in-memory fixed-size ring buffer measuring exact microsecond
and nanosecond timing breakdowns:
  - Worker wake lateness: actual_entry_ns - expected_wake_ns (single-cycle, NOT cumulative drift)
  - Wall time (perf_counter_ns) vs. Thread CPU time (thread_time_ns) for servo thread
  - Ten segmented timing checkpoints T0..T9 in _execute_servo_cycle():
      T0: Worker cycle entry
      T1: Target lock acquired / latest command obtained
      T2: Slew & vector recovery processing complete
      T3: Safety preflight check complete
      T4: Transport send_cycle entry
      T5: Transport send_cycle exit
      T6: Feedback scheduler ingest entry
      T7: Feedback scheduler ingest exit
      T8: Telemetry snapshot & handoff complete
      T9: Cycle exit
  - Transport sub-phase breakdown (prior RX drain, M1..M6 writes, waits, feedback drain)
  - True per-cycle context-switch deltas via RUSAGE_THREAD
  - Guaranteed faulting-cycle preservation before driver STOP or dump
  - Export artifacts:
      * cycle_timing.csv
      * transport_phase_timing.csv
      * fault_snapshot.json
      * stall_diagnostic_timing.csv (backwards-compatible alias)
      * stall_events_all.csv
      * stall_diagnostic_summary.json

Zero per-cycle disk I/O. In-memory ring buffer (default 300 cycles). Flushes exactly once on fault or shutdown.
"""

from __future__ import annotations

import atexit
import csv
import gc
import json
import os
import resource
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class CycleTimingRecord:
    cycle_seq: int = 0
    expected_wake_ns: int = 0
    actual_entry_ns: int = 0
    wake_lateness_ms: float = 0.0
    t_entry_ns: int = 0
    t_exit_ns: int = 0
    inter_callback_gap_ms: float = 0.0
    timer_dispatch_delay_ms: float = 0.0
    callback_total_ms: float = 0.0

    # T0..T9 Segment Breakdown (ms)
    target_lock_wait_ms: float = 0.0       # T0 -> T1
    slew_processing_ms: float = 0.0        # T1 -> T2
    preflight_check_ms: float = 0.0        # T2 -> T3
    pre_transport_ms: float = 0.0          # T3 -> T4
    transport_total_ms: float = 0.0        # T4 -> T5
    uart_tx_ms: float = 0.0                # within T4 -> T5
    uart_drain_ms: float = 0.0             # within T4 -> T5
    uart_total_ms: float = 0.0             # within T4 -> T5
    post_transport_ms: float = 0.0         # T5 -> T6
    ingest_feedback_ms: float = 0.0        # T6 -> T7
    unpack_samples_ms: float = 0.0         # within T7
    telem_total_ms: float = 0.0            # T7 -> T8
    snapshot_handoff_ms: float = 0.0       # within T8
    ros_pub_joint_state_ms: float = 0.0    # within T8
    ros_pub_timed_fb_ms: float = 0.0       # within T8
    ros_pub_timed_cmd_ms: float = 0.0      # within T8
    ros_pub_diagnostics_ms: float = 0.0    # within T8
    ros_pub_raw_diag_ms: float = 0.0       # within T8
    ros_pub_benchmark_ms: float = 0.0      # within T8
    ros_pub_total_ms: float = 0.0
    post_telem_ms: float = 0.0             # T8 -> T9
    other_in_callback_ms: float = 0.0

    # CPU, Memory, System Metrics
    thread_cpu_delta_ms: float = 0.0
    thread_cpu_time_ns: int = 0
    process_cpu_delta_ms: float = 0.0
    gc_pause_ms: float = 0.0
    vol_ctx_switches_delta: int = 0
    invol_ctx_switches_delta: int = 0
    vol_ctx_switches: int = 0
    invol_ctx_switches: int = 0
    native_thread_id: int = 0

    # Safety & Watchdog Metrics
    scheduler_gap_ms: float = 0.0
    watchdog_scheduler_gap_ms: float = 0.0
    feedback_age_max_ms: float = 0.0
    warn_log_duration_ms: float = 0.0
    fault_reason: str = ""
    fault_phase: str = ""

    # Transport Sub-Phase Summary (ms)
    tx_prior_drain_ms: float = 0.0
    tx_inter_wait_ms: float = 0.0
    tx_feedback_drain_ms: float = 0.0
    tx_slowest_motor_id: int = 0
    tx_slowest_motor_ms: float = 0.0
    tx_drain_in_waiting_calls: int = 0
    tx_drain_read_calls: int = 0
    tx_drain_bytes_read: int = 0
    tx_drain_max_read_ms: float = 0.0
    rx_max_single_read_wall_ms: float = 0.0
    rx_max_single_read_thread_cpu_ms: float = 0.0
    rx_max_single_read_process_cpu_ms: float = 0.0
    rx_max_read_vol_ctx_delta: int = 0
    rx_max_read_invol_ctx_delta: int = 0
    rx_read_call_count: int = 0
    rx_bytes: int = 0


@dataclass
class TransportPhaseRecord:
    cycle_seq: int = 0
    prior_rx_drain_wall_ms: float = 0.0
    prior_rx_drain_cpu_ms: float = 0.0
    prior_rx_drained_frames: int = 0
    m1_write_wall_ms: float = 0.0
    m1_write_cpu_ms: float = 0.0
    m1_req_bytes: int = 0
    m1_ret_bytes: int = 0
    m2_write_wall_ms: float = 0.0
    m2_write_cpu_ms: float = 0.0
    m2_req_bytes: int = 0
    m2_ret_bytes: int = 0
    m3_write_wall_ms: float = 0.0
    m3_write_cpu_ms: float = 0.0
    m3_req_bytes: int = 0
    m3_ret_bytes: int = 0
    m4_write_wall_ms: float = 0.0
    m4_write_cpu_ms: float = 0.0
    m4_req_bytes: int = 0
    m4_ret_bytes: int = 0
    m5_write_wall_ms: float = 0.0
    m5_write_cpu_ms: float = 0.0
    m5_req_bytes: int = 0
    m5_ret_bytes: int = 0
    m6_write_wall_ms: float = 0.0
    m6_write_cpu_ms: float = 0.0
    m6_req_bytes: int = 0
    m6_ret_bytes: int = 0
    inter_frame_wait_total_ms: float = 0.0
    motion_fb_drain_wall_ms: float = 0.0
    motion_fb_drain_cpu_ms: float = 0.0
    motion_fb_in_waiting_calls: int = 0
    motion_fb_read_calls: int = 0
    motion_fb_bytes_read: int = 0
    motion_fb_max_read_latency_ms: float = 0.0
    motion_fb_drain_iterations: int = 0
    feedbacks_extracted: int = 0
    transport_total_wall_ms: float = 0.0
    error_phase: str = ""
    error_motor_id: int = 0
    error_class: str = ""


class J11StallDiagnosticRecorder:
    """High-precision, zero-overhead diagnostic recorder for Pus6ServoThread."""

    def __init__(
        self,
        buffer_size: int = 300,
        output_dir: Optional[str] = None,
        enabled: bool = True,
    ):
        self.enabled = enabled
        self.buffer_size = buffer_size
        self.ring_buffer: deque[CycleTimingRecord] = deque(maxlen=buffer_size)
        self.transport_ring_buffer: deque[TransportPhaseRecord] = deque(maxlen=buffer_size)
        self.tx_queue_ring_buffer: deque = deque(maxlen=buffer_size * 6)
        self._curr_tx_queue_records: list[dict] = []
        self.all_stall_events: List[Dict[str, Any]] = []
        self.rx_slow_read_events: list[dict] = []
        self._rx_representative_notice_emitted = False

        self.output_dir_override = output_dir
        self.tier1_summary: List[Tuple[Any, ...]] = []
        self._t_prev_exit_ns: int = 0
        self._t_prev_entry_ns: int = 0
        self._cycle_count: int = 0
        self._prev_thread_cpu_ns: int = 0
        self._prev_process_cpu_ns: int = 0
        self._prev_vol_ctx: int = -1
        self._prev_invol_ctx: int = -1

        # Checkpoints for current active cycle
        self._curr: Optional[CycleTimingRecord] = None
        self._curr_transport: Optional[TransportPhaseRecord] = None
        self._t_entry_ns: int = 0
        self._t_target_lock_done_ns: int = 0
        self._t_slew_done_ns: int = 0
        self._t_preflight_done_ns: int = 0
        self._t_transport_entry_ns: int = 0
        self._t_transport_exit_ns: int = 0
        self._t_ingest_entry_ns: int = 0
        self._t_ingest_exit_ns: int = 0
        self._t_telem_done_ns: int = 0
        self._fault_snapshot: Optional[Dict[str, Any]] = None

        # GC tracking hook
        self._gc_pause_in_current_cycle_ns: int = 0
        self._gc_start_ns: int = 0
        self._setup_gc_hook()

        self._dumped = False
        atexit.register(self.dump)

    def _setup_gc_hook(self) -> None:
        def _gc_hook(phase: str, info: Any) -> None:
            now = time.perf_counter_ns()
            if phase == "start":
                self._gc_start_ns = now
            elif phase == "stop" and self._gc_start_ns > 0:
                dur = now - self._gc_start_ns
                self._gc_pause_in_current_cycle_ns += dur
                self._gc_start_ns = 0

        gc.callbacks.append(_gc_hook)

    def start_cycle(self, cycle_seq: int, expected_start_ns: int = 0) -> None:
        """T0: Worker cycle entry checkpoint."""
        if not self.enabled:
            return

        t_entry = time.perf_counter_ns()
        self._cycle_count += 1
        t_thread_cpu = time.thread_time_ns()
        t_process_cpu = time.process_time_ns()

        # Inter-callback gap (wall time between previous exit and this entry)
        inter_gap_ms = (t_entry - self._t_prev_exit_ns) / 1e6 if self._t_prev_exit_ns > 0 else 20.0

        # Wake lateness calculation (single-cycle, NOT cumulative drift from cycle 0)
        if expected_start_ns > 0:
            exp_ns = expected_start_ns
        elif self._t_prev_entry_ns > 0:
            exp_ns = self._t_prev_entry_ns + 20_000_000
        else:
            exp_ns = t_entry

        wake_lateness_ns = t_entry - exp_ns
        wake_lateness_ms = wake_lateness_ns / 1e6

        # CPU Deltas
        thread_cpu_delta_ms = (t_thread_cpu - self._prev_thread_cpu_ns) / 1e6 if self._prev_thread_cpu_ns > 0 else 0.0
        process_cpu_delta_ms = (t_process_cpu - self._prev_process_cpu_ns) / 1e6 if self._prev_process_cpu_ns > 0 else 0.0

        self._prev_thread_cpu_ns = t_thread_cpu
        self._prev_process_cpu_ns = t_process_cpu

        # Context switch deltas via RUSAGE_THREAD
        vol_ctx_delta = 0
        invol_ctx_delta = 0
        vol_ctx_total = 0
        invol_ctx_total = 0
        try:
            ru = resource.getrusage(resource.RUSAGE_THREAD)
            vol_ctx_total = ru.ru_nvcsw
            invol_ctx_total = ru.ru_nivcsw
            if self._prev_vol_ctx >= 0:
                vol_ctx_delta = max(0, vol_ctx_total - self._prev_vol_ctx)
                invol_ctx_delta = max(0, invol_ctx_total - self._prev_invol_ctx)
            self._prev_vol_ctx = vol_ctx_total
            self._prev_invol_ctx = invol_ctx_total
        except Exception:
            pass

        try:
            tid = threading.get_native_id()
        except Exception:
            tid = 0

        gc_pause_ms = self._gc_pause_in_current_cycle_ns / 1e6
        self._gc_pause_in_current_cycle_ns = 0

        rec = CycleTimingRecord(
            cycle_seq=cycle_seq,
            expected_wake_ns=exp_ns,
            actual_entry_ns=t_entry,
            wake_lateness_ms=round(wake_lateness_ms, 3),
            t_entry_ns=t_entry,
            inter_callback_gap_ms=round(inter_gap_ms, 3),
            timer_dispatch_delay_ms=round(wake_lateness_ms, 3),  # Updated: true single-cycle lateness
            thread_cpu_delta_ms=round(thread_cpu_delta_ms, 3),
            thread_cpu_time_ns=t_thread_cpu,
            process_cpu_delta_ms=round(process_cpu_delta_ms, 3),
            gc_pause_ms=round(gc_pause_ms, 3),
            vol_ctx_switches_delta=vol_ctx_delta,
            invol_ctx_switches_delta=invol_ctx_delta,
            vol_ctx_switches=vol_ctx_total,
            invol_ctx_switches=invol_ctx_total,
            native_thread_id=tid,
        )
        self._curr = rec
        self._curr_transport = TransportPhaseRecord(cycle_seq=cycle_seq)
        self._curr_tx_queue_records = []
        self._t_entry_ns = t_entry
        self._t_prev_entry_ns = t_entry

    def mark_target_lock_done(self) -> None:
        """T1: Target lock acquired / latest command obtained."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_target_lock_done_ns = now
        self._curr.target_lock_wait_ms = round((now - self._t_entry_ns) / 1e6, 3)

    def mark_slew_done(self) -> None:
        """T2: Slew & vector recovery processing complete."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_slew_done_ns = now
        start_ns = self._t_target_lock_done_ns if self._t_target_lock_done_ns > 0 else self._t_entry_ns
        self._curr.slew_processing_ms = round((now - start_ns) / 1e6, 3)

    def mark_preflight_done(self) -> None:
        """T3: Safety preflight check complete."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_preflight_done_ns = now
        start_ns = self._t_slew_done_ns if self._t_slew_done_ns > 0 else self._t_entry_ns
        self._curr.preflight_check_ms = round((now - start_ns) / 1e6, 3)

    def mark_transport_entry(self) -> None:
        """T4: Transport send_cycle entry checkpoint."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_transport_entry_ns = now
        start_ns = self._t_preflight_done_ns if self._t_preflight_done_ns > 0 else self._t_entry_ns
        self._curr.pre_transport_ms = round((now - start_ns) / 1e6, 3)

    def mark_transport_exit(self, servo_audit: Any = None) -> None:
        """T5: Transport send_cycle exit checkpoint."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_transport_exit_ns = now
        start_ns = self._t_transport_entry_ns if self._t_transport_entry_ns > 0 else self._t_entry_ns
        self._curr.transport_total_ms = round((now - start_ns) / 1e6, 3)

        if servo_audit is not None:
            self._process_servo_audit(servo_audit)

    def _process_servo_audit(self, servo_audit: Any) -> None:
        rec = self._curr
        trans = self._curr_transport
        if rec is None or trans is None:
            return

        tx_records = getattr(servo_audit, "tx_records", ())
        if tx_records:
            tx_start = tx_records[0].tx_start_ns
            tx_end = tx_records[-1].tx_end_ns
            rec.uart_tx_ms = round((tx_end - tx_start) / 1e6, 3)
            cycle_end = getattr(servo_audit, "cycle_end_ns", tx_end)
            rec.uart_drain_ms = round(max(0.0, (cycle_end - tx_end) / 1e6), 3)
            rec.uart_total_ms = round(max(0.0, (cycle_end - tx_start) / 1e6), 3)

            # Per-motor breakdowns
            slowest_mid = 0
            slowest_dur_ms = 0.0
            motor_durations = {}
            for tx in tx_records:
                metric = getattr(tx, "serial_write_metric", None)
                if metric is not None and metric.queue_before is not None:
                    row = dict(
                        cycle_seq=rec.cycle_seq, motor_id=tx.motor_id,
                        configured_timeout_ms=(metric.configured_timeout_s * 1000
                                               if metric.configured_timeout_s is not None else None),
                        serial_write_start_ns=metric.timestamp_start_ns,
                        serial_write_end_ns=metric.timestamp_end_ns,
                        serial_write_wall_ms=metric.wall_duration_ns / 1e6,
                        serial_write_cpu_ms=metric.cpu_duration_ns / 1e6,
                        wrapper_write_wall_ms=tx.duration_ns / 1e6,
                        write_error=metric.error or "",
                    )
                    for name in ('write_seq', 'native_thread_id', 'process_cpu_duration_ns',
                                 'voluntary_ctx_delta', 'involuntary_ctx_delta', 'snapshot_enter_ns',
                                 'snapshot_exit_ns', 'snapshot_overhead_ns', 'clock_sync_monotonic_before_ns',
                                 'clock_sync_epoch_ns', 'clock_sync_monotonic_after_ns', 'requested_bytes', 'written_bytes'):
                        row[name] = getattr(metric, name, None)
                    for suffix, state in (("before", metric.queue_before), ("after", metric.queue_after)):
                        row[f"out_waiting_{suffix}"] = state.out_waiting if state is not None else None
                        row[f"in_waiting_{suffix}"] = state.in_waiting if state is not None else None
                        row[f"queue_read_{suffix}_ms"] = state.wall_duration_ns / 1e6 if state is not None else None
                        row[f"queue_error_{suffix}"] = (state.error or "") if state is not None else "unavailable"
                    self._curr_tx_queue_records.append(row)
                dur_ms = round(getattr(tx, "duration_ns", 0) / 1e6, 3)
                motor_durations[tx.motor_id] = dur_ms
                if dur_ms > slowest_dur_ms:
                    slowest_dur_ms = dur_ms
                    slowest_mid = tx.motor_id

            rec.tx_slowest_motor_id = slowest_mid
            rec.tx_slowest_motor_ms = slowest_dur_ms

            # Fill TransportPhaseRecord
            motor_map = {tx.motor_id: tx for tx in tx_records}
            for mid in range(1, 7):
                tx = motor_map.get(mid)
                if tx is not None:
                    wall_ms = round(getattr(tx, "duration_ns", 0) / 1e6, 3)
                    cpu_ms = round(getattr(tx, "cpu_duration_ns", 0) / 1e6, 3)
                    req_b = getattr(tx, "requested_bytes", 0)
                    ret_b = getattr(tx, "written_bytes", 0)
                else:
                    wall_ms, cpu_ms, req_b, ret_b = 0.0, 0.0, 0, 0
                setattr(trans, f"m{mid}_write_wall_ms", wall_ms)
                setattr(trans, f"m{mid}_write_cpu_ms", cpu_ms)
                setattr(trans, f"m{mid}_req_bytes", req_b)
                setattr(trans, f"m{mid}_ret_bytes", ret_b)

        # Prior drain
        rec.tx_prior_drain_ms = round(getattr(servo_audit, "prior_rx_drain_wall_ns", 0) / 1e6, 3)
        trans.prior_rx_drain_wall_ms = rec.tx_prior_drain_ms
        trans.prior_rx_drain_cpu_ms = round(getattr(servo_audit, "prior_rx_drain_cpu_ns", 0) / 1e6, 3)
        trans.prior_rx_drained_frames = getattr(servo_audit, "prior_rx_drained_frames", 0)

        # Inter-frame wait
        rec.tx_inter_wait_ms = round(getattr(servo_audit, "inter_frame_wait_total_ns", 0) / 1e6, 3)
        trans.inter_frame_wait_total_ms = rec.tx_inter_wait_ms

        # Motion feedback drain
        rec.tx_feedback_drain_ms = round(getattr(servo_audit, "motion_drain_wall_ns", 0) / 1e6, 3)
        trans.motion_fb_drain_wall_ms = rec.tx_feedback_drain_ms
        trans.motion_fb_drain_cpu_ms = round(getattr(servo_audit, "motion_drain_cpu_ns", 0) / 1e6, 3)

        # Detailed RX Drain metrics from adapter
        rx_metrics = getattr(servo_audit, "rx_drain_metrics", None)
        if rx_metrics is not None:
            rec.tx_drain_in_waiting_calls = getattr(rx_metrics, "in_waiting_calls", 0)
            rec.tx_drain_read_calls = getattr(rx_metrics, "read_calls", 0)
            rec.tx_drain_bytes_read = getattr(rx_metrics, "bytes_read", 0)
            rec.tx_drain_max_read_ms = round(getattr(rx_metrics, "max_read_latency_ns", 0) / 1e6, 3)

            trans.motion_fb_in_waiting_calls = rec.tx_drain_in_waiting_calls
            trans.motion_fb_read_calls = rec.tx_drain_read_calls
            trans.motion_fb_bytes_read = rec.tx_drain_bytes_read
            trans.motion_fb_max_read_latency_ms = rec.tx_drain_max_read_ms
            trans.motion_fb_drain_iterations = getattr(rx_metrics, "drain_iterations", 0)
            trans.feedbacks_extracted = getattr(rx_metrics, "feedbacks_extracted", 0)

        trans.transport_total_wall_ms = rec.transport_total_ms
        trans.error_phase = getattr(servo_audit, "error_phase", "") or ""
        trans.error_motor_id = getattr(servo_audit, "error_motor_id", 0) or 0
        trans.error_class = getattr(servo_audit, "error_class", "") or ""

    def mark_uart_done(self, servo_audit: Any) -> None:
        """Backwards-compatible alias for mark_transport_exit."""
        self.mark_transport_exit(servo_audit)

    def mark_ingest_start(self) -> None:
        """T6: Ingest feedback start checkpoint."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_ingest_entry_ns = now
        start_ns = self._t_transport_exit_ns if self._t_transport_exit_ns > 0 else self._t_entry_ns
        self._curr.post_transport_ms = round((now - start_ns) / 1e6, 3)

    def mark_ingest_done(self, scheduler_gap_ms: float = 0.0, gap_ms: float = 0.0) -> None:
        """T7: Ingest feedback done checkpoint."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_ingest_exit_ns = now
        start_ns = self._t_ingest_entry_ns if self._t_ingest_entry_ns > 0 else self._t_entry_ns
        self._curr.ingest_feedback_ms = round((now - start_ns) / 1e6, 3)
        val = scheduler_gap_ms if scheduler_gap_ms > 0 else gap_ms
        self._curr.scheduler_gap_ms = round(val, 3)
        self._curr.watchdog_scheduler_gap_ms = round(val, 3)

    def mark_unpack_done(self) -> None:
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        start_ns = self._t_ingest_exit_ns if self._t_ingest_exit_ns > 0 else self._t_entry_ns
        self._curr.unpack_samples_ms = round((now - start_ns) / 1e6, 3)

    def mark_snapshot_handoff(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.snapshot_handoff_ms = round(dur_ms, 3)

    def mark_pub_joint_state(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_joint_state_ms = round(dur_ms, 3)

    def mark_pub_timed_feedback(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_timed_fb_ms = round(dur_ms, 3)

    def mark_pub_timed_command(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_timed_cmd_ms = round(dur_ms, 3)

    def mark_pub_diagnostics(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_diagnostics_ms = round(dur_ms, 3)

    def mark_pub_raw_diag(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_raw_diag_ms = round(dur_ms, 3)

    def mark_pub_benchmark(self, dur_ms: float) -> None:
        if not self.enabled or self._curr is None:
            return
        self._curr.ros_pub_benchmark_ms = round(dur_ms, 3)

    def mark_telemetry_done(self) -> None:
        """T8: Telemetry publication & caching complete checkpoint."""
        if not self.enabled or self._curr is None:
            return
        now = time.perf_counter_ns()
        self._t_telem_done_ns = now
        start_ns = self._t_ingest_exit_ns if self._t_ingest_exit_ns > 0 else self._t_entry_ns
        self._curr.telem_total_ms = round((now - start_ns) / 1e6, 3)

    def record_warning_log_duration(self, dur_ms: float) -> None:
        """Records duration spent in synchronous logging during warning events."""
        if not self.enabled or self._curr is None:
            return
        self._curr.warn_log_duration_ms = round(self._curr.warn_log_duration_ms + dur_ms, 3)

    def record_single_read(self, metric: Any) -> None:
        """Same-call observational counters; 5ms capture threshold is never a gate."""
        if not self.enabled:
            return
        rec = self._curr
        in_cycle = rec is not None and rec.native_thread_id == metric.native_thread_id
        if in_cycle:
            rec.rx_read_call_count += 1
            rec.rx_bytes += metric.returned_read_bytes or 0
            wall_ms = metric.read_wall_ns / 1e6
            if wall_ms > rec.rx_max_single_read_wall_ms:
                # All associated fields refer to the SAME slowest-wall read.
                rec.rx_max_single_read_wall_ms = wall_ms
                rec.rx_max_single_read_thread_cpu_ms = metric.read_thread_cpu_ns / 1e6
                rec.rx_max_single_read_process_cpu_ms = metric.read_process_cpu_ns / 1e6
                rec.rx_max_read_vol_ctx_delta = metric.read_vol_ctx_delta
                rec.rx_max_read_invol_ctx_delta = metric.read_invol_ctx_delta
        if metric.read_wall_ns > 5_000_000:
            event = dict(cycle_seq=rec.cycle_seq if in_cycle else -1,
                         scope="cycle" if in_cycle else "non_cycle", **asdict(metric))
            self.rx_slow_read_events.append(event)
            if in_cycle and metric.read_wall_ns > 20_000_000 and not self._rx_representative_notice_emitted:
                self._rx_representative_notice_emitted = True
                # One diagnostic notice lets the short-run harness finish promptly.
                # No motion/fault state is changed here; the measured read has returned.
                print("J14_RX_READ_EVENT " + json.dumps(event), flush=True)

    def _finish_tx_queue_records(self) -> None:
        for row in self._curr_tx_queue_records:
            row.update(six_axis_tx_phase_ms=self._curr.uart_tx_ms,
                       callback_total_ms=self._curr.callback_total_ms,
                       fault_reason=self._curr.fault_reason)
            self.tx_queue_ring_buffer.append(row)
        self._curr_tx_queue_records = []

    def record_fault_cycle(self, fault_reason: str, fault_phase: str = "unknown") -> None:
        """Guarantees that the active faulting cycle is preserved in the ring buffer before dump."""
        if self._curr is None:
            return

        now = time.perf_counter_ns()
        rec = self._curr
        rec.t_exit_ns = now
        rec.callback_total_ms = round((now - rec.t_entry_ns) / 1e6, 3)
        rec.fault_reason = str(fault_reason)
        rec.fault_phase = str(fault_phase)

        # Compute other_in_callback_ms
        accounted = (
            rec.target_lock_wait_ms
            + rec.slew_processing_ms
            + rec.preflight_check_ms
            + rec.pre_transport_ms
            + rec.transport_total_ms
            + rec.post_transport_ms
            + rec.ingest_feedback_ms
            + rec.telem_total_ms
        )
        rec.other_in_callback_ms = round(max(0.0, rec.callback_total_ms - accounted), 3)

        self.ring_buffer.append(rec)
        if self._curr_transport is not None:
            if not self._curr_transport.error_phase:
                self._curr_transport.error_phase = fault_phase
            self.transport_ring_buffer.append(self._curr_transport)

        d = asdict(rec)
        self.all_stall_events.append(d)
        self._create_fault_snapshot(rec)
        self._finish_tx_queue_records()

        tier1_tuple = (
            rec.cycle_seq,
            rec.actual_entry_ns,
            rec.wake_lateness_ms,
            rec.inter_callback_gap_ms,
            rec.callback_total_ms,
            rec.uart_tx_ms,
            rec.uart_drain_ms,
            rec.tx_drain_read_calls,
            rec.tx_drain_bytes_read,
            rec.ros_pub_total_ms,
            rec.feedback_age_max_ms,
            rec.warn_log_duration_ms,
            rec.fault_reason,
            rec.rx_max_single_read_wall_ms,
            rec.rx_max_single_read_thread_cpu_ms,
            rec.rx_max_single_read_process_cpu_ms,
            rec.rx_max_read_vol_ctx_delta,
            rec.rx_max_read_invol_ctx_delta,
            rec.rx_read_call_count,
            rec.rx_bytes,
        )
        self.tier1_summary.append(tier1_tuple)

        self._curr = None
        self._curr_transport = None

    def finish_cycle(self, fault_reason: str = "", feedback_age_max_ms: float = 0.0) -> None:
        """T9: Cycle exit checkpoint."""
        if not self.enabled:
            return

        t_exit = time.perf_counter_ns()
        self._t_prev_exit_ns = t_exit
        if self._curr is None:
            return

        rec = self._curr
        rec.t_exit_ns = t_exit
        rec.callback_total_ms = round((t_exit - rec.t_entry_ns) / 1e6, 3)
        rec.fault_reason = str(fault_reason)
        rec.feedback_age_max_ms = round(feedback_age_max_ms, 1)

        start_post_telem = self._t_telem_done_ns if self._t_telem_done_ns > 0 else (
            self._t_ingest_exit_ns if self._t_ingest_exit_ns > 0 else self._t_entry_ns
        )
        rec.post_telem_ms = round((t_exit - start_post_telem) / 1e6, 3)

        rec.ros_pub_total_ms = round(
            rec.snapshot_handoff_ms
            + rec.ros_pub_joint_state_ms
            + rec.ros_pub_timed_fb_ms
            + rec.ros_pub_timed_cmd_ms
            + rec.ros_pub_diagnostics_ms
            + rec.ros_pub_raw_diag_ms
            + rec.ros_pub_benchmark_ms,
            3,
        )

        accounted = (
            rec.target_lock_wait_ms
            + rec.slew_processing_ms
            + rec.preflight_check_ms
            + rec.pre_transport_ms
            + rec.transport_total_ms
            + rec.post_transport_ms
            + rec.ingest_feedback_ms
            + rec.ros_pub_total_ms
            + rec.post_telem_ms
        )
        rec.other_in_callback_ms = round(max(0.0, rec.callback_total_ms - accounted), 3)

        self.ring_buffer.append(rec)
        if self._curr_transport is not None:
            self.transport_ring_buffer.append(self._curr_transport)

        self._finish_tx_queue_records()

        tier1_tuple = (
            rec.cycle_seq,
            rec.actual_entry_ns,
            rec.wake_lateness_ms,
            rec.inter_callback_gap_ms,
            rec.callback_total_ms,
            rec.uart_tx_ms,
            rec.uart_drain_ms,
            rec.tx_drain_read_calls,
            rec.tx_drain_bytes_read,
            rec.ros_pub_total_ms,
            rec.feedback_age_max_ms,
            rec.warn_log_duration_ms,
            rec.fault_reason,
            rec.rx_max_single_read_wall_ms,
            rec.rx_max_single_read_thread_cpu_ms,
            rec.rx_max_single_read_process_cpu_ms,
            rec.rx_max_read_vol_ctx_delta,
            rec.rx_max_read_invol_ctx_delta,
            rec.rx_read_call_count,
            rec.rx_bytes,
        )
        self.tier1_summary.append(tier1_tuple)

        if rec.inter_callback_gap_ms >= 50.0 or rec.callback_total_ms >= 30.0 or fault_reason:
            d = asdict(rec)
            self.all_stall_events.append(d)

        if fault_reason:
            self._create_fault_snapshot(rec)

        self._curr = None
        self._curr_transport = None

    def _create_fault_snapshot(self, rec: CycleTimingRecord) -> None:
        """Builds a comprehensive diagnostic fault snapshot for fault_snapshot.json."""
        # Classify stall type
        gap_ms = rec.inter_callback_gap_ms
        cb_ms = rec.callback_total_ms
        tot_missing_ms = gap_ms + cb_ms

        classification = "H_UNKNOWN"
        explanation = ""

        if rec.scheduler_gap_ms >= 1000.0 or "scheduler_deadline" in rec.fault_reason:
            classification = "D_FEEDBACK_SCHEDULER_GAP"
            explanation = (
                f"J7ServoFeedbackScheduler deadline exceeded: gap {rec.scheduler_gap_ms:.1f}ms > 1000ms. "
                f"Worker wake lateness was {rec.wake_lateness_ms:.1f}ms, thread CPU was {rec.thread_cpu_delta_ms:.1f}ms."
            )
        elif rec.tx_prior_drain_ms >= 50.0 or rec.tx_feedback_drain_ms >= 50.0:
            classification = "C_RX_DRAIN_BLOCK"
            explanation = (
                f"RX drain blocked: prior drain = {rec.tx_prior_drain_ms:.1f}ms, "
                f"motion feedback drain = {rec.tx_feedback_drain_ms:.1f}ms (calls={rec.tx_drain_read_calls}, max_lat={rec.tx_drain_max_read_ms:.1f}ms)."
            )
        elif rec.tx_slowest_motor_ms >= 40.0:
            classification = "B_SERIAL_WRITE_BLOCK"
            explanation = (
                f"UART serial write blocked on Motor ID{rec.tx_slowest_motor_id} for {rec.tx_slowest_motor_ms:.1f}ms."
            )
        elif gap_ms >= 50.0 and gap_ms > cb_ms:
            if rec.process_cpu_delta_ms > 50.0 and rec.thread_cpu_delta_ms < 15.0:
                classification = "A_TIMER_DISPATCH_DELAY_PYTHON_GIL_OR_BUSY"
                explanation = (
                    f"Servo thread wake delayed by {gap_ms:.1f}ms. Process CPU was {rec.process_cpu_delta_ms:.1f}ms "
                    f"while thread CPU was only {rec.thread_cpu_delta_ms:.1f}ms (indicates Python GIL contention or busy thread)."
                )
            else:
                classification = "A_TIMER_DISPATCH_DELAY_OS_OR_WAIT"
                explanation = (
                    f"Servo thread wake delayed by {gap_ms:.1f}ms outside callback. Thread CPU was {rec.thread_cpu_delta_ms:.1f}ms."
                )
        elif cb_ms >= 50.0:
            if rec.thread_cpu_delta_ms < 15.0:
                classification = "B_IN_CALLBACK_DESCHEDULE_OR_WAIT"
                explanation = (
                    f"Servo thread stalled inside callback for {cb_ms:.1f}ms (slew={rec.slew_processing_ms:.1f}ms, "
                    f"transport={rec.transport_total_ms:.1f}ms), but consumed only {rec.thread_cpu_delta_ms:.1f}ms CPU (indicates OS deschedule / lock wait)."
                )
            else:
                classification = "B_IN_CALLBACK_CPU_BOUND"
                explanation = f"Servo thread busy-looped inside callback for {cb_ms:.1f}ms (thread CPU = {rec.thread_cpu_delta_ms:.1f}ms)."

        trans_dict = {}
        if self._curr_transport is not None:
            trans_dict = asdict(self._curr_transport)

        self._fault_snapshot = {
            "timestamp_iso": datetime.now(timezone.utc).isoformat(),
            "fault_reason": rec.fault_reason,
            "fault_phase": rec.fault_phase,
            "cycle_seq": rec.cycle_seq,
            "expected_wake_ns": rec.expected_wake_ns,
            "actual_entry_ns": rec.actual_entry_ns,
            "wake_lateness_ms": rec.wake_lateness_ms,
            "inter_callback_gap_ms": rec.inter_callback_gap_ms,
            "callback_total_ms": rec.callback_total_ms,
            "total_cycle_missing_ms": round(tot_missing_ms, 3),
            "scheduler_gap_ms": rec.scheduler_gap_ms,
            "feedback_age_max_ms": rec.feedback_age_max_ms,
            "classification": classification,
            "explanation": explanation,
            "cpu_metrics": {
                "thread_cpu_delta_ms": rec.thread_cpu_delta_ms,
                "process_cpu_delta_ms": rec.process_cpu_delta_ms,
                "gc_pause_ms": rec.gc_pause_ms,
                "vol_ctx_switches_delta": rec.vol_ctx_switches_delta,
                "invol_ctx_switches_delta": rec.invol_ctx_switches_delta,
                "native_thread_id": rec.native_thread_id,
            },
            "segmented_timing_ms": {
                "t0_to_t1_target_lock_wait": rec.target_lock_wait_ms,
                "t1_to_t2_slew_processing": rec.slew_processing_ms,
                "t2_to_t3_preflight_check": rec.preflight_check_ms,
                "t3_to_t4_pre_transport": rec.pre_transport_ms,
                "t4_to_t5_transport_total": rec.transport_total_ms,
                "t5_to_t6_post_transport": rec.post_transport_ms,
                "t6_to_t7_ingest_feedback": rec.ingest_feedback_ms,
                "t7_to_t8_telem_total": rec.telem_total_ms,
                "t8_to_t9_post_telem": rec.post_telem_ms,
                "other_unaccounted": rec.other_in_callback_ms,
            },
            "transport_breakdown": {
                "prior_rx_drain_ms": rec.tx_prior_drain_ms,
                "inter_frame_wait_ms": rec.tx_inter_wait_ms,
                "feedback_drain_ms": rec.tx_feedback_drain_ms,
                "slowest_motor_id": rec.tx_slowest_motor_id,
                "slowest_motor_ms": rec.tx_slowest_motor_ms,
                "rx_drain_in_waiting_calls": rec.tx_drain_in_waiting_calls,
                "rx_drain_read_calls": rec.tx_drain_read_calls,
                "rx_drain_bytes_read": rec.tx_drain_bytes_read,
                "rx_drain_max_read_latency_ms": rec.tx_drain_max_read_ms,
            },
            "transport_phase_record": trans_dict,
        }

    def get_effective_output_dir(self) -> Path:
        if self.output_dir_override:
            p = Path(self.output_dir_override)
            p.mkdir(parents=True, exist_ok=True)
            return p

        for ptr_path in ("/tmp/j14_latest_run_dir", "/tmp/j11_latest_run_dir"):
            ptr = Path(ptr_path)
            if ptr.exists():
                try:
                    target = ptr.read_text().strip()
                    if target:
                        p = Path(target)
                        p.mkdir(parents=True, exist_ok=True)
                        return p
                except OSError:
                    pass

        default_p = Path("/tmp/j11_stall_diagnostics")
        default_p.mkdir(parents=True, exist_ok=True)
        return default_p

    def dump(self) -> None:
        """Dumps in-memory ring buffers and diagnostic summaries to disk."""
        if self._dumped:
            return
        self._dumped = True

        # CRITICAL: If an active cycle is in flight when dump is called (e.g. from latch), finish it now!
        if self._curr is not None:
            self.record_fault_cycle(
                fault_reason=self._curr.fault_reason or "incomplete_cycle_on_latch",
                fault_phase=self._curr.fault_phase or "active_cycle_on_dump",
            )

        out_dir = self.get_effective_output_dir()
        records = list(self.ring_buffer)
        trans_records = list(self.transport_ring_buffer)
        queue_records = list(self.tx_queue_ring_buffer)
        if self.rx_slow_read_events:
            with open(out_dir / "rx_read_events.csv", "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.rx_slow_read_events[0]))
                writer.writeheader()
                writer.writerows(self.rx_slow_read_events)
        else:
            (out_dir / "rx_read_events.csv").write_text("cycle_seq,scope,read_seq,read_wall_ns\n")
        if queue_records:
            with open(out_dir / "tx_queue_state.csv", "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(queue_records[0]))
                writer.writeheader()
                writer.writerows(queue_records)
            with open(out_dir / 'tx_write_events.jsonl', 'w') as stream:
                for row in queue_records:
                    if row['write_error']:
                        stream.write(json.dumps(row)+'\n')

        # 0. Save cycle_summary_full.csv (Tier 1 full-lifecycle lightweight summary)
        if self.tier1_summary:
            t1_file = out_dir / "cycle_summary_full.csv"
            with open(t1_file, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
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
                ])
                w.writerows(self.tier1_summary)

        # 1. Save cycle_timing.csv (and alias stall_diagnostic_timing.csv for backwards compat)
        if records:
            keys = list(asdict(records[0]).keys())
            for fname in ("cycle_timing.csv", "stall_diagnostic_timing.csv"):
                csv_file = out_dir / fname
                with open(csv_file, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=keys)
                    w.writeheader()
                    for r in records:
                        w.writerow(asdict(r))

        # 2. Save transport_phase_timing.csv
        if trans_records:
            trans_keys = list(asdict(trans_records[0]).keys())
            trans_csv = out_dir / "transport_phase_timing.csv"
            with open(trans_csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=trans_keys)
                w.writeheader()
                for tr in trans_records:
                    w.writerow(asdict(tr))

        # 3. Save stall_events_all.csv
        if self.all_stall_events:
            keys = list(self.all_stall_events[0].keys())
            stall_file = out_dir / "stall_events_all.csv"
            with open(stall_file, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                for ev in self.all_stall_events:
                    w.writerow(ev)

        # 4. Save fault_snapshot.json
        if self._fault_snapshot is not None:
            snapshot_file = out_dir / "fault_snapshot.json"
            with open(snapshot_file, "w") as f:
                json.dump(self._fault_snapshot, f, indent=2)

        # 5. Compute budget analysis for worst stall and save stall_diagnostic_summary.json
        worst_stall_record = None
        max_stall_ms = 0.0

        for ev in self.all_stall_events:
            total_missing = ev["inter_callback_gap_ms"] + ev["callback_total_ms"]
            if total_missing > max_stall_ms:
                max_stall_ms = total_missing
                worst_stall_record = ev

        if worst_stall_record is None and records:
            for r in records:
                tot = r.inter_callback_gap_ms + r.callback_total_ms
                if tot > max_stall_ms:
                    max_stall_ms = tot
                    worst_stall_record = asdict(r)

        budget_analysis = None
        stall_type = "NONE"
        if worst_stall_record is not None:
            gap_ms = worst_stall_record["inter_callback_gap_ms"]
            cb_ms = worst_stall_record["callback_total_ms"]
            tot_missing_ms = gap_ms + cb_ms

            if worst_stall_record.get("scheduler_gap_ms", 0) >= 1000.0:
                stall_type = "D_FEEDBACK_SCHEDULER_GAP"
            elif gap_ms > cb_ms and gap_ms >= 50.0:
                if worst_stall_record["process_cpu_delta_ms"] < 10.0:
                    stall_type = "A_TIMER_DISPATCH_DELAY_OS_OR_WAIT"
                else:
                    stall_type = "A_TIMER_DISPATCH_DELAY_PYTHON_GIL_OR_BUSY"
            elif cb_ms >= 50.0:
                if worst_stall_record["uart_total_ms"] >= 40.0:
                    stall_type = "B_IN_CALLBACK_UART_BLOCK"
                elif worst_stall_record["ros_pub_total_ms"] >= 40.0:
                    stall_type = "E_IN_CALLBACK_ROS_PUBLISH_BLOCK"
                elif worst_stall_record["target_lock_wait_ms"] >= 40.0:
                    stall_type = "D_IN_CALLBACK_LOCK_CONTENTION"
                elif worst_stall_record["gc_pause_ms"] >= 40.0:
                    stall_type = "F_IN_CALLBACK_GC_PAUSE"
                else:
                    stall_type = "B_IN_CALLBACK_OTHER_BLOCK"
            else:
                stall_type = "H_MIXED_OR_NORMAL"

            budget_analysis = {
                "worst_stall_cycle": worst_stall_record["cycle_seq"],
                "total_missing_interval_ms": round(tot_missing_ms, 3),
                "inter_callback_gap_ms": gap_ms,
                "callback_execution_total_ms": cb_ms,
                "wake_lateness_ms": worst_stall_record.get("wake_lateness_ms", 0.0),
                "timer_dispatch_delay_ms": worst_stall_record["timer_dispatch_delay_ms"],
                "scheduler_gap_ms": worst_stall_record.get("scheduler_gap_ms", 0.0),
                "in_callback_breakdown_ms": {
                    "target_lock_wait_ms": worst_stall_record["target_lock_wait_ms"],
                    "slew_processing_ms": worst_stall_record["slew_processing_ms"],
                    "preflight_check_ms": worst_stall_record["preflight_check_ms"],
                    "pre_transport_ms": worst_stall_record.get("pre_transport_ms", 0.0),
                    "transport_total_ms": worst_stall_record.get("transport_total_ms", 0.0),
                    "uart_tx_ms": worst_stall_record["uart_tx_ms"],
                    "uart_drain_ms": worst_stall_record["uart_drain_ms"],
                    "uart_total_ms": worst_stall_record["uart_total_ms"],
                    "post_transport_ms": worst_stall_record.get("post_transport_ms", 0.0),
                    "ingest_feedback_ms": worst_stall_record["ingest_feedback_ms"],
                    "telem_total_ms": worst_stall_record.get("telem_total_ms", 0.0),
                    "ros_pub_total_ms": worst_stall_record["ros_pub_total_ms"],
                    "other_in_callback_ms": worst_stall_record["other_in_callback_ms"],
                },
                "cpu_and_system": {
                    "thread_cpu_delta_ms": worst_stall_record["thread_cpu_delta_ms"],
                    "process_cpu_delta_ms": worst_stall_record["process_cpu_delta_ms"],
                    "gc_pause_ms": worst_stall_record["gc_pause_ms"],
                    "vol_ctx_switches_delta": worst_stall_record.get("vol_ctx_switches_delta", 0),
                    "invol_ctx_switches_delta": worst_stall_record.get("invol_ctx_switches_delta", 0),
                    "vol_ctx_switches": worst_stall_record["vol_ctx_switches"],
                    "invol_ctx_switches": worst_stall_record["invol_ctx_switches"],
                    "native_thread_id": worst_stall_record["native_thread_id"],
                },
                "stall_classification": stall_type,
                "percentage_accounted_for": round((gap_ms + cb_ms) / max(0.001, tot_missing_ms) * 100.0, 1),
            }

        stalls_gt_50 = sum(1 for ev in self.all_stall_events if ev["inter_callback_gap_ms"] >= 50.0 or ev["callback_total_ms"] >= 50.0)
        stalls_gt_100 = sum(1 for ev in self.all_stall_events if ev["inter_callback_gap_ms"] >= 100.0 or ev["callback_total_ms"] >= 100.0)
        stalls_gt_150 = sum(1 for ev in self.all_stall_events if ev["inter_callback_gap_ms"] >= 150.0 or ev["callback_total_ms"] >= 150.0)
        stalls_gt_200 = sum(1 for ev in self.all_stall_events if ev["inter_callback_gap_ms"] >= 200.0 or ev["callback_total_ms"] >= 200.0)

        summary = {
            "total_cycles_recorded": len(records),
            "total_stall_events_recorded": len(self.all_stall_events),
            "stalls_gt_50ms_count": stalls_gt_50,
            "stalls_gt_100ms_count": stalls_gt_100,
            "stalls_gt_150ms_count": stalls_gt_150,
            "stalls_gt_200ms_count": stalls_gt_200,
            "max_missing_interval_ms": round(max_stall_ms, 3),
            "worst_stall_time_budget": budget_analysis,
        }

        summary_file = out_dir / "stall_diagnostic_summary.json"
        with open(summary_file, "w") as f:
            json.dump(summary, f, indent=2)

        print(f"[STALL RECORDER] Timing diagnostics saved to: {out_dir} (Worst stall: {max_stall_ms:.1f}ms, Type: {stall_type})", flush=True)


__all__ = [
    "CycleTimingRecord",
    "J11StallDiagnosticRecorder",
    "TransportPhaseRecord",
]
