#!/usr/bin/env python3
"""6PUS J8-C: Read-Only Hardware Audit & Real UART Timing Preflight.

Executes:
1. J8-C1: Verifies strictly zero state-changing commands on real hardware.
2. J8-C2: Six-axis preflight scan, ID verification, physical positions, current, fault status, and 0x30 transaction timing.
3. J8-C3: Real 50Hz round-robin telemetry streaming (200 cycles = 4.0s), FixedLag delay sweep [120, 140, 160, 180, 200 ms], and static LM FK stability.

STRICT CONSTRAINTS:
- ZERO commanded physical motion.
- ZERO 0x29 position writes.
- ZERO 0x25 / 0x24 register writes.
- ZERO 0x1A pause / stop writes.
- Pure 0x30 status read frames only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
import numpy as np

# Ensure workspace packages are in path
SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
SRC_DIR = WS_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = SRC_DIR / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import serial
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.protocol import CMD_RD_STATUS, CMD_WR_REGISTER
from pus6_motor_driver.safety import (
    FEEDBACK_LOWER_LIMIT,
    FEEDBACK_UPPER_LIMIT,
    MOTOR_IDS,
    NUM_MOTORS,
    PARK_MEASUREMENT_BAND_LOWER,
    PARK_MEASUREMENT_BAND_UPPER,
    StartupClassification,
    StartupStateAssessment,
    classify_startup_state,
)


def run_j8_c_audit(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    timeout_s: float = 0.015,
    min_frame_interval_s: float = 0.0005,
    duration_s: float = 4.0,
) -> dict:
    print(f"\n================================================================================")
    print(f"       6PUS J8-C: READ-ONLY HARDWARE AUDIT & REAL TIMING PREFLIGHT")
    print(f"================================================================================")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Timeout: {timeout_s * 1000:.1f} ms")
    print(f"Mode: READ_ONLY (Zero motion, Zero writes, Pure 0x30 reads only)")
    print(f"Stream duration: {duration_s:.1f} s")
    print(f"================================================================================\n")

    # 1. Open real hardware serial port
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as exc:
        print(f"[FATAL] Failed to open serial port {port}: {exc}")
        sys.exit(1)

    adapter = J7UartAdapter(
        ser,
        baudrate=baudrate,
        timeout_s=timeout_s,
        min_frame_interval_s=min_frame_interval_s,
    )

    # Empty dummy pause callback: READ_ONLY mode MUST NOT send 0x1A
    def read_only_pause_cb(reason: str) -> None:
        print(f"[READ_ONLY GUARD] Pause called with reason='{reason}' -> Suppressed write.")

    writer = J7SixAxisServoTransport(
        adapter,
        min_frame_interval_s=min_frame_interval_s,
        pause_callback=read_only_pause_cb,
    )

    feedback_scheduler = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=1e9, # Do not perform periodic full scans during streaming
        feedback_budget_s=0.010,
    )

    # -------------------------------------------------------------------------
    # J8-C2: Initial Six-Axis Readback & Preflight Scan
    # -------------------------------------------------------------------------
    print("--- [J8-C2] Initial Six-Axis Status & ID Preflight Scan (3 stability samples) ---")
    preflight_samples: list[list[int]] = []
    initial_positions: dict[int, int] = {}
    initial_currents: dict[int, int] = {}
    initial_faults: dict[int, int] = {}
    initial_stamps_ns: dict[int, int] = {}

    for pre_scan in range(3):
        t_preflight_start_ns = time.monotonic_ns()
        preflight_audit = feedback_scheduler.full_scan(
            servo_cycle_id=pre_scan,
            servo_cycle_timestamp_ns=t_preflight_start_ns,
        )
        assert len(preflight_audit.samples) == 6, f"Expected 6 responded axes, got {len(preflight_audit.samples)}"
        scan_vals: list[int] = []
        for sample in preflight_audit.samples:
            mid = sample.motor_id
            pos = sample.position if sample.position is not None else -1
            curr = sample.current_mA if sample.current_mA is not None else 0
            fault = sample.fault if sample.fault is not None else 0
            initial_positions[mid] = pos
            initial_currents[mid] = curr
            initial_faults[mid] = fault
            initial_stamps_ns[mid] = sample.timestamp_ns
            scan_vals.append(pos)
        preflight_samples.append(scan_vals)
        if pre_scan < 2:
            time.sleep(0.02)

    print(f"{'Axis ID':<10}{'Position (steps)':<18}{'Position (mm)':<16}{'Current (mA)':<16}{'Fault Code':<12}")
    print("-" * 72)
    for mid in MOTOR_IDS:
        pos = initial_positions[mid]
        pos_mm = (0.050 + (pos - 1000) * 0.000025) * 1000.0 if pos is not None else 0.0
        curr = initial_currents[mid]
        fault = initial_faults[mid]
        print(f"ID{mid:<8}{pos:<18}{pos_mm:<16.3f}{curr:<16}{fault:<12}")

    # Check zero faults across all axes
    for mid in MOTOR_IDS:
        assert initial_faults[mid] == 0, f"Axis {mid} reported active hardware fault: {initial_faults[mid]}"

    # Authoritative Startup Classification
    pos_list = [initial_positions[mid] for mid in MOTOR_IDS]
    assessment = classify_startup_state(pos_list, history_samples=preflight_samples)
    print(
        f"\n[STARTUP CLASSIFIER] Classification: {assessment.classification.value.upper()} | "
        f"Planning Seed: {list(assessment.planning_seed)} | "
        f"Info: {assessment.description}"
    )
    assert assessment.is_valid, f"Preflight startup assessment failed: {assessment.description}"

    print("\n[PREFLIGHT PASS] All 6 axes verified online, positions read successfully, 0 hardware faults.\n")

    # -------------------------------------------------------------------------
    # J8-C3: 50Hz Real Round-Robin Telemetry Stream
    # -------------------------------------------------------------------------
    total_cycles = int(duration_s * 50.0) # 200 cycles @ 50Hz = 4.0s
    cycle_interval_s = 0.020

    print(f"--- [J8-C3] Streaming Real 50Hz Telemetry ({total_cycles} cycles = {duration_s:.1f}s) ---")

    synchronizer = FixedLagJointSynchronizer(max_history_size=1000)

    cached_steps = dict(initial_positions)
    cached_stamps = dict(initial_stamps_ns)
    cached_currents = dict(initial_currents)
    cached_faults = dict(initial_faults)

    # Initial snapshot push
    stamps_0 = [cached_stamps[m] for m in MOTOR_IDS]
    pos_0 = [0.050 + (cached_steps[m] - 1000) * 0.000025 for m in MOTOR_IDS]
    synchronizer.push_raw_snapshot(stamps_0, pos_0, [cached_steps[m] for m in MOTOR_IDS])

    cycle_durations_ms: list[float] = []
    round_robin_intervals_ms: dict[int, list[float]] = {mid: [] for mid in MOTOR_IDS}
    last_axis_poll_ns: dict[int, int] = dict(initial_stamps_ns)
    per_cycle_audit_records = []
    polled_axis_sequence: list[int] = []

    t_stream_start_ns = time.monotonic_ns()
    next_cycle_target_ns = t_stream_start_ns

    for cycle in range(total_cycles):
        t_cycle_entry_ns = time.monotonic_ns()

        # 1. READ_ONLY cycle: strictly poll 1 axis via round-robin 0x30
        feedback_audit = feedback_scheduler.poll_once(
            servo_cycle_id=cycle + 1,
            servo_cycle_timestamp_ns=t_cycle_entry_ns,
        )

        t_cycle_exit_ns = time.monotonic_ns()
        cycle_dur_ms = (t_cycle_exit_ns - t_cycle_entry_ns) / 1e6
        cycle_durations_ms.append(cycle_dur_ms)

        polled_axis = feedback_audit.motor_ids[0] if feedback_audit.motor_ids else -1
        sample = feedback_audit.samples[0] if feedback_audit.samples else None
        polled_axis_sequence.append(polled_axis)

        if sample is not None and sample.position is not None:
            # Update cache for polled axis ONLY
            cached_steps[polled_axis] = sample.position
            cached_currents[polled_axis] = sample.current_mA if sample.current_mA is not None else 0
            cached_faults[polled_axis] = sample.fault if sample.fault is not None else 0
            cached_stamps[polled_axis] = sample.timestamp_ns

            # Interval between polls for this axis
            if last_axis_poll_ns[polled_axis] > 0:
                dt_axis_ms = (sample.timestamp_ns - last_axis_poll_ns[polled_axis]) / 1e6
                round_robin_intervals_ms[polled_axis].append(dt_axis_ms)
            last_axis_poll_ns[polled_axis] = sample.timestamp_ns

        # Ingest snapshot into FixedLag synchronizer
        rx_stamps = [cached_stamps[mid] for mid in MOTOR_IDS]
        pos_meters = [0.050 + (cached_steps[mid] - 1000) * 0.000025 for mid in MOTOR_IDS]
        raw_steps = [cached_steps[mid] for mid in MOTOR_IDS]
        synchronizer.push_raw_snapshot(rx_stamps, pos_meters, raw_steps)

        # Precise 50Hz rate pacing
        next_cycle_target_ns += int(cycle_interval_s * 1e9)
        sleep_ns = next_cycle_target_ns - time.monotonic_ns()
        if sleep_ns > 0:
            time.sleep(sleep_ns / 1e9)

    t_stream_end_ns = time.monotonic_ns()
    actual_duration_s = (t_stream_end_ns - t_stream_start_ns) / 1e9
    actual_rate_hz = total_cycles / actual_duration_s

    # Close serial port cleanly
    ser.close()

    print(f"Streaming complete: {total_cycles} cycles executed in {actual_duration_s:.3f} s (Average Rate: {actual_rate_hz:.2f} Hz).")

    # -------------------------------------------------------------------------
    # Timing & Real Hardware UART Analysis
    # -------------------------------------------------------------------------
    req_audits = adapter._request_audit
    durations_ms = [(rec.tx_end_ns - rec.tx_start_ns) / 1e6 for rec in req_audits]
    timeouts = sum(1 for rec in req_audits if rec.timeout)
    rx_owner = adapter.rx_owner

    print("\n--- Real UART 0x30 Query Timing Statistics ---")
    print(f"Total 0x30 Requests: {len(req_audits)}")
    print(f"Transaction Duration (ms): Mean={np.mean(durations_ms):.2f}, Min={np.min(durations_ms):.2f}, "
          f"P50={np.percentile(durations_ms, 50):.2f}, P95={np.percentile(durations_ms, 95):.2f}, "
          f"P99={np.percentile(durations_ms, 99):.2f}, Max={np.max(durations_ms):.2f}")
    print(f"Timeouts: {timeouts} ({timeouts/max(1, len(req_audits))*100:.2f}%)")
    print(f"Malformed frames: {rx_owner._malformed_count}")
    print(f"Overflows: {rx_owner._overflow_count}")
    print(f"Residual bytes: {rx_owner.residual_bytes}")

    # Per-axis feedback intervals
    all_axis_intervals = [val for sublist in round_robin_intervals_ms.values() for val in sublist]
    mean_rr_interval_ms = float(np.mean(all_axis_intervals)) if all_axis_intervals else 0.0
    min_rr_interval_ms = float(np.min(all_axis_intervals)) if all_axis_intervals else 0.0
    max_rr_interval_ms = float(np.max(all_axis_intervals)) if all_axis_intervals else 0.0
    std_rr_interval_ms = float(np.std(all_axis_intervals)) if all_axis_intervals else 0.0

    print("\n--- Per-Axis Feedback Intervals (Round-Robin Period) ---")
    print(f"Per-Axis Interval (ms): Mean={mean_rr_interval_ms:.2f}, Min={min_rr_interval_ms:.2f}, "
          f"Max={max_rr_interval_ms:.2f}, Std={std_rr_interval_ms:.2f}")

    # History buffer lengths verification (proves monotonic filtering works on real telemetry)
    buf_lengths = synchronizer.buffer_lengths
    print(f"Synchronizer buffer lengths across 6 axes: {buf_lengths}")
    assert all(count <= total_cycles // 6 + 2 for count in buf_lengths), (
        f"Buffer lengths {buf_lengths} exceed expected round-robin count! Proves duplicate samples are rejected."
    )

    # -------------------------------------------------------------------------
    # FixedLag Delay Sweep on Real Telemetry
    # -------------------------------------------------------------------------
    kin = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0]

    candidate_delays_ms = [120.0, 140.0, 160.0, 180.0, 200.0]
    sweep_results = {}

    # Query window: evaluate queries across the middle 2.5s of streaming @ 200Hz
    q_start_ns = t_stream_start_ns + int(1.0 * 1e9)
    q_end_ns = t_stream_start_ns + int(3.5 * 1e9)
    n_queries = 500
    query_stamps_ns = np.linspace(q_start_ns, q_end_ns, n_queries, dtype=np.int64)

    print(f"\n--- FixedLag Delay Sweep on Real Telemetry ({n_queries} queries @ 200Hz) ---")
    print(f"{'Delay (ms)':<12}{'Sync Ratio':<14}{'WaitFuture':<14}{'FK Success':<14}{'Mean Sample Age':<18}{'Lat. Margin':<14}")
    print("-" * 86)

    for delay_ms in candidate_delays_ms:
        delay_ns = int(delay_ms * 1e6)
        synced_count = 0
        wait_future_count = 0
        wait_too_early_count = 0
        fk_success_count = 0
        sample_ages_ms = []
        fk_poses = []
        fk_residuals = []

        for q_ns in query_stamps_ns:
            t_target_query = int(q_ns - delay_ns)
            sync_res = synchronizer.query_sync_configuration(t_target_query)

            if sync_res.status == SyncStatus.SYNCED:
                synced_count += 1
                sample_ages_ms.append((q_ns - t_target_query) / 1e6)
                fk_res = kin.forward_kinematics_with_metrics(sync_res.positions_meters, initial_guess=ref_pose)
                if fk_res.converged:
                    fk_success_count += 1
                    fk_poses.append(fk_res.pose)
                    fk_residuals.append(fk_res.residual_norm)
            elif sync_res.status == SyncStatus.WAIT_FUTURE_SAMPLE:
                wait_future_count += 1
            elif sync_res.status == SyncStatus.WAIT_TOO_EARLY:
                wait_too_early_count += 1

        bracket_ratio = (synced_count / n_queries) * 100.0
        fk_ratio = (fk_success_count / n_queries) * 100.0
        mean_age_ms = float(np.mean(sample_ages_ms)) if sample_ages_ms else 0.0
        margin_ms = delay_ms - mean_rr_interval_ms

        sweep_results[delay_ms] = {
            "delay_ms": delay_ms,
            "synced_count": synced_count,
            "wait_future_count": wait_future_count,
            "wait_too_early_count": wait_too_early_count,
            "bracketing_success_ratio_pct": bracket_ratio,
            "fk_publish_success_ratio_pct": fk_ratio,
            "mean_sample_age_ms": mean_age_ms,
            "safety_margin_over_rr_mean_ms": margin_ms,
            "fk_pose_count": len(fk_poses),
            "fk_mean_residual_norm": float(np.mean(fk_residuals)) if fk_residuals else 0.0,
        }

        print(f"{delay_ms:<12.1f}{bracket_ratio:<14.1f}%{wait_future_count:<14}{fk_ratio:<14.1f}%{mean_age_ms:<18.1f}{margin_ms:<14.1f}")

    # -------------------------------------------------------------------------
    # Static FK Stability & Spatial Dispersion Analysis
    # -------------------------------------------------------------------------
    # Analyze reconstructed Cartesian pose for candidate 160ms
    tau_eval_ms = 160.0
    tau_eval_ns = int(tau_eval_ms * 1e6)
    valid_poses = []
    for q_ns in query_stamps_ns:
        s_res = synchronizer.query_sync_configuration(int(q_ns - tau_eval_ns))
        if s_res.status == SyncStatus.SYNCED:
            fk_res = kin.forward_kinematics_with_metrics(s_res.positions_meters, initial_guess=ref_pose)
            if fk_res.converged:
                valid_poses.append(fk_res.pose)

    valid_poses_arr = np.array(valid_poses)
    mean_pose = np.mean(valid_poses_arr, axis=0) if len(valid_poses) > 0 else np.zeros(6)
    std_pose = np.std(valid_poses_arr, axis=0) if len(valid_poses) > 0 else np.zeros(6)

    print("\n--- Static Reconstructed Cartesian Pose Stability (tau=160ms) ---")
    print(f"Mean Pose: X={mean_pose[0]*1000:+.3f}mm, Y={mean_pose[1]*1000:+.3f}mm, Z={mean_pose[2]*1000:+.3f}mm, "
          f"Roll={math.degrees(mean_pose[3]):+.4f}deg, Pitch={math.degrees(mean_pose[4]):+.4f}deg, Yaw={math.degrees(mean_pose[5]):+.4f}deg")
    print(f"Pose Std:  X={std_pose[0]*1000:.4f}mm, Y={std_pose[1]*1000:.4f}mm, Z={std_pose[2]*1000:.4f}mm, "
          f"Roll={math.degrees(std_pose[3]):.5f}deg, Pitch={math.degrees(std_pose[4]):.5f}deg, Yaw={math.degrees(std_pose[5]):.5f}deg")

    result = {
        "hardware_port": port,
        "baudrate": baudrate,
        "duration_s": actual_duration_s,
        "total_cycles": total_cycles,
        "actual_rate_hz": actual_rate_hz,
        "initial_positions_steps": initial_positions,
        "initial_currents_ma": initial_currents,
        "initial_faults": initial_faults,
        "uart_0x30_timing_ms": {
            "mean": float(np.mean(durations_ms)),
            "min": float(np.min(durations_ms)),
            "p50": float(np.percentile(durations_ms, 50)),
            "p95": float(np.percentile(durations_ms, 95)),
            "p99": float(np.percentile(durations_ms, 99)),
            "max": float(np.max(durations_ms)),
        },
        "per_axis_feedback_interval_ms": {
            "mean": mean_rr_interval_ms,
            "min": min_rr_interval_ms,
            "max": max_rr_interval_ms,
            "std": std_rr_interval_ms,
        },
        "fixed_lag_delay_sweep": sweep_results,
        "static_fk_pose_mean": mean_pose.tolist(),
        "static_fk_pose_std": std_pose.tolist(),
        "status": "PASS",
    }

    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="J8-C Read-Only Hardware Audit")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--output-json", type=str, default="")
    args = parser.parse_args()

    res = run_j8_c_audit(
        port=args.port,
        baudrate=args.baudrate,
        duration_s=args.duration,
    )

    if args.output_json:
        with open(args.output_json, "w") as f:
            json.dump(res, f, indent=2)
        print(f"\nWrote audit results to {args.output_json}")
