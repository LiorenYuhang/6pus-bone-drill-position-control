#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Benchmark script evaluating Runtime-Fix-1 instrumentation overhead (Section 24).

Compares:
  - Instrumentation OFF (recorder disabled / no-op)
  - Instrumentation ON (recorder active with ring buffer, rusage, T0-T9, transport audit)

Measures:
  - Mean cycle latency (microseconds)
  - P50, P95, P99 latency (microseconds)
  - Memory consumption of 300-element ring buffer
  - Confirms overhead is well below 100 microseconds (< 0.1 ms) per cycle.
"""

from __future__ import annotations

import gc
import json
import os
import sys
import tempfile
import time
from pathlib import Path
import numpy as np

# Ensure pus6_motor_driver is accessible
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
    SixAxisCycleAudit,
    SixAxisTxRecord,
)
from pus6_motor_driver.stall_diagnostic_recorder import (
    CycleTimingRecord,
    J11StallDiagnosticRecorder,
    TransportPhaseRecord,
)


def run_benchmark(num_cycles: int = 5000):
    print(f"=== RUNNING RUNTIME-FIX-1 INSTRUMENTATION BENCHMARK ({num_cycles} cycles) ===")

    # Prepare dummy audit
    now_ns = time.perf_counter_ns()
    tx_recs = [
        SixAxisTxRecord(
            motor_id=mid,
            cycle_id=1,
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
        cycle_id=1,
        target_timestamp_ns=now_ns,
        cycle_start_ns=now_ns,
        cycle_end_ns=now_ns + 3_500_000,
        cycle_jitter_ns=0,
        target_age_ns=0,
        tx_records=tuple(tx_recs),
        tx_skew_ns=0,
        batch_complete_ns=now_ns + 3_500_000,
        deadline_miss=False,
        prior_rx_drain_wall_ns=100_000,
        prior_rx_drain_cpu_ns=50_000,
        motion_drain_wall_ns=1_000_000,
        motion_drain_cpu_ns=500_000,
    )

    # 1. Benchmark OFF mode
    rec_off = J11StallDiagnosticRecorder(buffer_size=300, enabled=False)
    latencies_off_us = []

    # Warmup
    for _ in range(200):
        rec_off.start_cycle(1)
        rec_off.mark_target_lock_done()
        rec_off.mark_slew_done()
        rec_off.mark_preflight_done()
        rec_off.mark_transport_entry()
        rec_off.mark_transport_exit(audit)
        rec_off.mark_ingest_start()
        rec_off.mark_ingest_done(20.0)
        rec_off.mark_telemetry_done()
        rec_off.finish_cycle()

    for i in range(num_cycles):
        t0 = time.perf_counter_ns()
        rec_off.start_cycle(i)
        rec_off.mark_target_lock_done()
        rec_off.mark_slew_done()
        rec_off.mark_preflight_done()
        rec_off.mark_transport_entry()
        rec_off.mark_transport_exit(audit)
        rec_off.mark_ingest_start()
        rec_off.mark_ingest_done(20.0)
        rec_off.mark_telemetry_done()
        rec_off.finish_cycle()
        t1 = time.perf_counter_ns()
        latencies_off_us.append((t1 - t0) / 1000.0)

    # 2. Benchmark ON mode
    tmp_dir = tempfile.mkdtemp()
    rec_on = J11StallDiagnosticRecorder(buffer_size=300, output_dir=tmp_dir, enabled=True)
    latencies_on_us = []

    # Warmup
    for _ in range(200):
        rec_on.start_cycle(1)
        rec_on.mark_target_lock_done()
        rec_on.mark_slew_done()
        rec_on.mark_preflight_done()
        rec_on.mark_transport_entry()
        rec_on.mark_transport_exit(audit)
        rec_on.mark_ingest_start()
        rec_on.mark_ingest_done(20.0)
        rec_on.mark_telemetry_done()
        rec_on.finish_cycle()

    for i in range(num_cycles):
        t0 = time.perf_counter_ns()
        rec_on.start_cycle(i, expected_start_ns=t0)
        rec_on.mark_target_lock_done()
        rec_on.mark_slew_done()
        rec_on.mark_preflight_done()
        rec_on.mark_transport_entry()
        rec_on.mark_transport_exit(audit)
        rec_on.mark_ingest_start()
        rec_on.mark_ingest_done(20.0)
        rec_on.mark_telemetry_done()
        rec_on.finish_cycle()
        t1 = time.perf_counter_ns()
        latencies_on_us.append((t1 - t0) / 1000.0)

    # Calculate statistics
    off_mean = np.mean(latencies_off_us)
    off_p50 = np.percentile(latencies_off_us, 50)
    off_p95 = np.percentile(latencies_off_us, 95)
    off_p99 = np.percentile(latencies_off_us, 99)

    on_mean = np.mean(latencies_on_us)
    on_p50 = np.percentile(latencies_on_us, 50)
    on_p95 = np.percentile(latencies_on_us, 95)
    on_p99 = np.percentile(latencies_on_us, 99)

    delta_mean = on_mean - off_mean
    delta_p95 = on_p95 - off_p95
    delta_p99 = on_p99 - off_p99

    # Memory measurement of ring buffers
    import sys
    record_size_bytes = sys.getsizeof(rec_on.ring_buffer[0])
    trans_record_size_bytes = sys.getsizeof(rec_on.transport_ring_buffer[0])
    total_buffer_mem_bytes = (record_size_bytes + trans_record_size_bytes) * 300

    print("\n--- BENCHMARK RESULTS ---")
    print(f"OFF mode: Mean={off_mean:.3f} us | P50={off_p50:.3f} us | P95={off_p95:.3f} us | P99={off_p99:.3f} us")
    print(f"ON  mode: Mean={on_mean:.3f} us | P50={on_p50:.3f} us | P95={on_p95:.3f} us | P99={on_p99:.3f} us")
    print(f"Added overhead (Mean): {delta_mean:.3f} us ({delta_mean/1000.0:.4f} ms)")
    print(f"Added overhead (P95) : {delta_p95:.3f} us ({delta_p95/1000.0:.4f} ms)")
    print(f"Added overhead (P99) : {delta_p99:.3f} us ({delta_p99/1000.0:.4f} ms)")
    print(f"Memory footprint for 300-cycle ring buffer: ~{total_buffer_mem_bytes / 1024.0:.2f} KB")

    # Assert budget requirement (< 100 us / 0.1 ms)
    assert delta_mean < 100.0, f"Overhead mean {delta_mean} us exceeds 100 us limit!"
    print("\n[VERDICT] Overhead budget PASSED (< 100 us). Zero impact on 50Hz (20,000 us) cycle.")

    results = {
        "off": {"mean_us": off_mean, "p50_us": off_p50, "p95_us": off_p95, "p99_us": off_p99},
        "on": {"mean_us": on_mean, "p50_us": on_p50, "p95_us": on_p95, "p99_us": on_p99},
        "delta": {"mean_us": delta_mean, "p95_us": delta_p95, "p99_us": delta_p99},
        "memory_kb": total_buffer_mem_bytes / 1024.0,
    }
    return results


if __name__ == "__main__":
    run_benchmark(5000)
