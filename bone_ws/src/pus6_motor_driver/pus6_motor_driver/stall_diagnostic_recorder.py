#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""J11-RT1 Runtime Stall Diagnostic Recorder.

Zero-overhead in-memory ring buffer measuring exact microsecond timing breakdowns:
  - inter_callback_gap_ms (time outside callback / executor scheduling delay)
  - timer_dispatch_delay_ms
  - callback_total_ms
  - in-callback segments:
      * target_lock_wait_ms
      * slew_processing_ms
      * preflight_check_ms
      * uart_tx_ms
      * uart_drain_ms
      * ingest_feedback_ms
      * ros_pub_joint_state_ms
      * ros_pub_timed_fb_ms
      * ros_pub_timed_cmd_ms
      * ros_pub_diagnostics_ms
      * ros_pub_raw_diagnostics_ms
      * ros_pub_total_ms
      * other_in_callback_ms
  - CPU & GC metrics:
      * thread_cpu_ms
      * process_cpu_ms
      * gc_pause_ms
      * voluntary_ctx_switches
      * involuntary_ctx_switches

Zero per-cycle I/O. Flushes to disk only on stop/fault.
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
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class CycleTimingRecord:
    cycle_seq: int = 0
    t_entry_ns: int = 0
    t_exit_ns: int = 0
    inter_callback_gap_ms: float = 0.0
    timer_dispatch_delay_ms: float = 0.0
    callback_total_ms: float = 0.0

    # In-callback breakdown (ms)
    target_lock_wait_ms: float = 0.0
    slew_processing_ms: float = 0.0
    preflight_check_ms: float = 0.0
    uart_tx_ms: float = 0.0
    uart_drain_ms: float = 0.0
    uart_total_ms: float = 0.0
    ingest_feedback_ms: float = 0.0
    unpack_samples_ms: float = 0.0
    snapshot_handoff_ms: float = 0.0
    ros_pub_joint_state_ms: float = 0.0
    ros_pub_timed_fb_ms: float = 0.0
    ros_pub_timed_cmd_ms: float = 0.0
    ros_pub_diagnostics_ms: float = 0.0
    ros_pub_raw_diag_ms: float = 0.0
    ros_pub_benchmark_ms: float = 0.0
    ros_pub_total_ms: float = 0.0
    other_in_callback_ms: float = 0.0

    # CPU & System metrics
    thread_cpu_delta_ms: float = 0.0
    process_cpu_delta_ms: float = 0.0
    gc_pause_ms: float = 0.0
    vol_ctx_switches: int = 0
    invol_ctx_switches: int = 0
    native_thread_id: int = 0

    # Safety & Watchdog
    watchdog_scheduler_gap_ms: float = 0.0
    feedback_age_max_ms: float = 0.0
    fault_reason: str = ""


class J11StallDiagnosticRecorder:
    def __init__(self, buffer_size: int = 3000, output_dir: Optional[str] = None):
        self.buffer_size = buffer_size
        self.ring_buffer: deque[CycleTimingRecord] = deque(maxlen=buffer_size)
        self.all_stall_events: List[Dict[str, Any]] = []

        self.output_dir_override = output_dir
        self._t_prev_exit_ns: int = 0
        self._t_first_entry_ns: int = 0
        self._cycle_count: int = 0
        self._prev_thread_cpu_ns: int = 0
        self._prev_process_cpu_ns: int = 0

        # GC tracking hook
        self._gc_pause_in_current_cycle_ns: int = 0
        self._gc_start_ns: int = 0
        self._setup_gc_hook()

        # Checkpoints for current active cycle
        self._curr: Optional[CycleTimingRecord] = None
        self._t_entry_ns: int = 0
        self._t_target_lock_start_ns: int = 0
        self._t_slew_start_ns: int = 0
        self._t_preflight_start_ns: int = 0
        self._t_uart_tx_start_ns: int = 0
        self._t_ingest_start_ns: int = 0
        self._t_unpack_start_ns: int = 0
        self._t_pub_js_start_ns: int = 0
        self._t_pub_tf_start_ns: int = 0
        self._t_pub_tc_start_ns: int = 0
        self._t_pub_diag_start_ns: int = 0
        self._t_pub_raw_start_ns: int = 0
        self._t_pub_bench_start_ns: int = 0

        self._dumped = False
        atexit.register(self.dump)

    def _setup_gc_hook(self):
        def _gc_hook(phase, info):
            now = time.monotonic_ns()
            if phase == "start":
                self._gc_start_ns = now
            elif phase == "stop" and self._gc_start_ns > 0:
                dur = now - self._gc_start_ns
                self._gc_pause_in_current_cycle_ns += dur
                self._gc_start_ns = 0

        gc.callbacks.append(_gc_hook)

    def start_cycle(self, cycle_seq: int) -> None:
        t_entry = time.monotonic_ns()
        self._cycle_count += 1
        if self._t_first_entry_ns == 0:
            self._t_first_entry_ns = t_entry

        t_thread_cpu = time.thread_time_ns()
        t_process_cpu = time.process_time_ns()

        inter_gap_ms = (t_entry - self._t_prev_exit_ns) / 1e6 if self._t_prev_exit_ns > 0 else 20.0
        expected_timer_ns = self._t_first_entry_ns + (self._cycle_count - 1) * 20_000_000
        dispatch_delay_ms = (t_entry - expected_timer_ns) / 1e6

        thread_cpu_delta_ms = (t_thread_cpu - self._prev_thread_cpu_ns) / 1e6 if self._prev_thread_cpu_ns > 0 else 0.0
        process_cpu_delta_ms = (t_process_cpu - self._prev_process_cpu_ns) / 1e6 if self._prev_process_cpu_ns > 0 else 0.0

        self._prev_thread_cpu_ns = t_thread_cpu
        self._prev_process_cpu_ns = t_process_cpu

        try:
            ru = resource.getrusage(resource.RUSAGE_SELF)
            vol_ctx = ru.ru_nvcsw
            invol_ctx = ru.ru_nivcsw
        except Exception:
            vol_ctx = 0
            invol_ctx = 0

        try:
            tid = threading.get_native_id()
        except Exception:
            tid = 0

        gc_pause_ms = self._gc_pause_in_current_cycle_ns / 1e6
        self._gc_pause_in_current_cycle_ns = 0

        rec = CycleTimingRecord(
            cycle_seq=cycle_seq,
            t_entry_ns=t_entry,
            inter_callback_gap_ms=round(inter_gap_ms, 3),
            timer_dispatch_delay_ms=round(dispatch_delay_ms, 3),
            thread_cpu_delta_ms=round(thread_cpu_delta_ms, 3),
            process_cpu_delta_ms=round(process_cpu_delta_ms, 3),
            gc_pause_ms=round(gc_pause_ms, 3),
            vol_ctx_switches=vol_ctx,
            invol_ctx_switches=invol_ctx,
            native_thread_id=tid,
        )
        self._curr = rec
        self._t_entry_ns = t_entry

    def mark_target_lock_done(self) -> None:
        if self._curr is not None:
            now = time.monotonic_ns()
            self._curr.target_lock_wait_ms = round((now - self._t_entry_ns) / 1e6, 3)
            self._t_slew_start_ns = now

    def mark_slew_done(self) -> None:
        if self._curr is not None and self._t_slew_start_ns > 0:
            now = time.monotonic_ns()
            self._curr.slew_processing_ms = round((now - self._t_slew_start_ns) / 1e6, 3)
            self._t_preflight_start_ns = now

    def mark_preflight_done(self) -> None:
        if self._curr is not None and self._t_preflight_start_ns > 0:
            now = time.monotonic_ns()
            self._curr.preflight_check_ms = round((now - self._t_preflight_start_ns) / 1e6, 3)

    def mark_uart_done(self, servo_audit: Any) -> None:
        if self._curr is not None and servo_audit is not None:
            try:
                tx_records = getattr(servo_audit, "tx_records", ())
                if tx_records:
                    tx_start = tx_records[0].tx_start_ns
                    tx_end = tx_records[-1].tx_end_ns
                    self._curr.uart_tx_ms = round((tx_end - tx_start) / 1e6, 3)
                    cycle_end = getattr(servo_audit, "cycle_end_ns", tx_end)
                    self._curr.uart_drain_ms = round(max(0.0, (cycle_end - tx_end) / 1e6), 3)
                    self._curr.uart_total_ms = round(max(0.0, (cycle_end - tx_start) / 1e6), 3)
            except Exception:
                pass

    def mark_ingest_start(self) -> None:
        self._t_ingest_start_ns = time.monotonic_ns()

    def mark_ingest_done(self, scheduler_gap_ms: float = 0.0, gap_ms: float = 0.0) -> None:
        if self._curr is not None and self._t_ingest_start_ns > 0:
            now = time.monotonic_ns()
            self._curr.ingest_feedback_ms = round((now - self._t_ingest_start_ns) / 1e6, 3)
            val = scheduler_gap_ms if scheduler_gap_ms > 0 else gap_ms
            self._curr.watchdog_scheduler_gap_ms = round(val, 3)
            self._t_unpack_start_ns = now

    def mark_unpack_done(self) -> None:
        if self._curr is not None and self._t_unpack_start_ns > 0:
            now = time.monotonic_ns()
            self._curr.unpack_samples_ms = round((now - self._t_unpack_start_ns) / 1e6, 3)

    def mark_snapshot_handoff(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.snapshot_handoff_ms = round(dur_ms, 3)

    def mark_pub_joint_state(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_joint_state_ms = round(dur_ms, 3)

    def mark_pub_timed_feedback(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_timed_fb_ms = round(dur_ms, 3)

    def mark_pub_timed_command(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_timed_cmd_ms = round(dur_ms, 3)

    def mark_pub_diagnostics(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_diagnostics_ms = round(dur_ms, 3)

    def mark_pub_raw_diag(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_raw_diag_ms = round(dur_ms, 3)

    def mark_pub_benchmark(self, dur_ms: float) -> None:
        if self._curr is not None:
            self._curr.ros_pub_benchmark_ms = round(dur_ms, 3)

    def finish_cycle(self, fault_reason: str = "", feedback_age_max_ms: float = 0.0) -> None:
        t_exit = time.monotonic_ns()
        self._t_prev_exit_ns = t_exit
        if self._curr is None:
            return

        rec = self._curr
        rec.t_exit_ns = t_exit
        rec.callback_total_ms = round((t_exit - rec.t_entry_ns) / 1e6, 3)
        rec.fault_reason = str(fault_reason)
        rec.feedback_age_max_ms = round(feedback_age_max_ms, 1)

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
            + rec.uart_total_ms
            + rec.ingest_feedback_ms
            + rec.unpack_samples_ms
            + rec.ros_pub_total_ms
        )
        rec.other_in_callback_ms = round(max(0.0, rec.callback_total_ms - accounted), 3)

        self.ring_buffer.append(rec)
        self._curr = None

        # Record significant stall events (> 50ms total gap or duration)
        if rec.inter_callback_gap_ms >= 50.0 or rec.callback_total_ms >= 30.0 or fault_reason:
            d = asdict(rec)
            self.all_stall_events.append(d)

    def get_effective_output_dir(self) -> Path:
        if self.output_dir_override:
            p = Path(self.output_dir_override)
            p.mkdir(parents=True, exist_ok=True)
            return p

        # Check /tmp/j11_latest_run_dir
        ptr = Path("/tmp/j11_latest_run_dir")
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
        if self._dumped:
            return
        self._dumped = True

        out_dir = self.get_effective_output_dir()

        # 1. Save all records in ring buffer to CSV
        records = list(self.ring_buffer)
        csv_file = out_dir / "stall_diagnostic_timing.csv"
        if records:
            keys = list(asdict(records[0]).keys())
            with open(csv_file, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                for r in records:
                    w.writerow(asdict(r))

        # 2. Save all stall events (>50ms) to CSV
        stall_file = out_dir / "stall_events_all.csv"
        if self.all_stall_events:
            keys = list(self.all_stall_events[0].keys())
            with open(stall_file, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                for ev in self.all_stall_events:
                    w.writerow(ev)

        # 3. Compute detailed time budget analysis for worst stall
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

            # Classify stall type
            if gap_ms > cb_ms and gap_ms >= 50.0:
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
                "timer_dispatch_delay_ms": worst_stall_record["timer_dispatch_delay_ms"],
                "in_callback_breakdown_ms": {
                    "target_lock_wait_ms": worst_stall_record["target_lock_wait_ms"],
                    "slew_processing_ms": worst_stall_record["slew_processing_ms"],
                    "preflight_check_ms": worst_stall_record["preflight_check_ms"],
                    "uart_tx_ms": worst_stall_record["uart_tx_ms"],
                    "uart_drain_ms": worst_stall_record["uart_drain_ms"],
                    "uart_total_ms": worst_stall_record["uart_total_ms"],
                    "ingest_feedback_ms": worst_stall_record["ingest_feedback_ms"],
                    "ros_pub_joint_state_ms": worst_stall_record["ros_pub_joint_state_ms"],
                    "ros_pub_timed_feedback_ms": worst_stall_record["ros_pub_timed_fb_ms"],
                    "ros_pub_timed_command_ms": worst_stall_record["ros_pub_timed_cmd_ms"],
                    "ros_pub_diagnostics_ms": worst_stall_record["ros_pub_diagnostics_ms"],
                    "ros_pub_raw_diagnostics_ms": worst_stall_record["ros_pub_raw_diag_ms"],
                    "ros_pub_total_ms": worst_stall_record["ros_pub_total_ms"],
                    "other_in_callback_ms": worst_stall_record["other_in_callback_ms"],
                },
                "cpu_and_system": {
                    "thread_cpu_delta_ms": worst_stall_record["thread_cpu_delta_ms"],
                    "process_cpu_delta_ms": worst_stall_record["process_cpu_delta_ms"],
                    "gc_pause_ms": worst_stall_record["gc_pause_ms"],
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
