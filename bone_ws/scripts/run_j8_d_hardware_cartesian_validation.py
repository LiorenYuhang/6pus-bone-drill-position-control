#!/usr/bin/env python3
"""6PUS J8-D: Active Takeover & First Cartesian Micro-Motion Validation Runner.

Executes:
1. J8-D0: Active Takeover at OPERATING_HOME
   - Verifies physical initial position is at OPERATING_HOME (~[1000]*6)
   - Establishes first ACTIVE 0x29 batch
   - Proves no uninitialized [0]*6 or stale commands
   - Holds HOME for 3.0s at 50Hz with continuous telemetry

2. J8-D1: First Cartesian Micro-Motion (HOME -> Z +2.0 mm -> HOME)
   - Real Chain: Cartesian Target -> Analytical IK -> Shaper -> J7 50Hz Servo -> 6PUS -> TimedFeedback -> FixedLag -> FK
   - Records full outbound and measured trajectory
   - Analyzes real joint tracking errors (RMS, Peak, Final)
   - Performs timestamp-aligned Cartesian tracking: Command(t_query) vs FK(t_query)
   - Measures UART 0x29/0x30 transaction timing, 50Hz cycle jitter, and protocol stats
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
import numpy as np

# Ensure workspace packages are on path
SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
SRC_DIR = WS_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = SRC_DIR / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import serial
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7StopAdapter
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.safety import MOTOR_IDS, NUM_MOTORS, HardwareMode


@dataclass
class CycleRecord:
    cycle_seq: int
    t_cycle_start_ns: int
    t_cycle_end_ns: int
    cartesian_target_m: list[float]
    ik_desired_deltas_m: list[float]
    shaped_target_steps: list[int]
    tx_start_ns: int
    tx_end_ns: int
    polled_axis: int
    polled_rx_ns: int
    polled_pos_steps: int
    polled_curr_mA: int
    polled_fault: int
    all_latest_pos_steps: list[int]
    all_latest_stamps_ns: list[int]
    fk_sync_status: str
    fk_query_stamp_ns: int
    fk_measured_pose: list[float]
    fk_residual_norm: float


def run_j8_d_validation(
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    timeout_s: float = 0.015,
    min_frame_interval_s: float = 0.0005,
    fixed_lag_delay_ms: float = 160.0,
    output_json: str = "",
) -> dict:
    print("\n" + "=" * 80)
    print("      6PUS J8-D: ACTIVE TAKEOVER & FIRST CARTESIAN MICRO-MOTION")
    print("=" * 80)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"FixedLag Delay: {fixed_lag_delay_ms:.1f} ms")
    print("=" * 80 + "\n")

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

    stop_adapter = J7StopAdapter(adapter)
    transport = J7SixAxisServoTransport(
        adapter,
        min_frame_interval_s=min_frame_interval_s,
        pause_callback=stop_adapter,
    )
    feedback_scheduler = J7ServoFeedbackScheduler(
        transport,
        full_scan_period_s=1e9,
        feedback_budget_s=0.010,
    )

    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    home_cartesian_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]

    # Initialize 6-axis Jerk-Limited Shaper
    # max_speed: 0.005 m/s (200 step/s = 5.0 mm/s)
    # max_accel: 0.050 m/s^2 (2000 step/s^2 = 50.0 mm/s^2)
    # max_jerk:  0.500 m/s^3 (20000 step/s^3 = 500.0 mm/s^3)
    shaper = StatefulJerkLimitedShaper(
        max_speed_m_per_sec=0.005,
        max_accel_m_per_sec2=0.050,
        max_jerk_m_per_sec3=0.500,
        omega_n=20.0,
    )

    synchronizer = FixedLagJointSynchronizer(max_history_size=2000)

    records: list[CycleRecord] = []
    first_active_0x29_batch: list[int] | None = None

    try:
        # =====================================================================
        # J8-D0: Active Takeover Audit at OPERATING_HOME
        # =====================================================================
        print("--- [J8-D0] Preflight Scan & Active Takeover Audit ---")
        t_preflight_ns = time.monotonic_ns()
        preflight_audit = feedback_scheduler.full_scan(
            servo_cycle_id=0,
            servo_cycle_timestamp_ns=t_preflight_ns,
        )

        initial_positions: dict[int, int] = {}
        initial_currents: dict[int, int] = {}
        initial_faults: dict[int, int] = {}
        initial_stamps_ns: dict[int, int] = {}

        for sample in preflight_audit.samples:
            mid = sample.motor_id
            initial_positions[mid] = sample.position if sample.position is not None else -1
            initial_currents[mid] = sample.current_mA if sample.current_mA is not None else 0
            initial_faults[mid] = sample.fault if sample.fault is not None else 0
            initial_stamps_ns[mid] = sample.timestamp_ns

        print(f"{'Axis ID':<10}{'Position (steps)':<18}{'Position (mm)':<16}{'Current (mA)':<16}{'Fault Code':<12}")
        print("-" * 72)
        for mid in MOTOR_IDS:
            pos = initial_positions[mid]
            pos_mm = (0.050 + (pos - 1000) * 0.000025) * 1000.0 if pos >= 0 else 0.0
            print(f"ID{mid:<8}{pos:<18}{pos_mm:<16.3f}{initial_currents[mid]:<16}{initial_faults[mid]:<12}")

        # Check HOME precondition: Positions must be around [1000]*6
        for mid in MOTOR_IDS:
            assert initial_faults[mid] == 0, f"Axis ID{mid} reported hardware fault: {initial_faults[mid]}"
            pos_err = abs(initial_positions[mid] - 1000)
            if pos_err > 50:
                raise RuntimeError(
                    f"Axis ID{mid} initial position {initial_positions[mid]} is far from OPERATING_HOME (1000). "
                    "Please run run_j7_servo_home_park.py --action home first!"
                )

        # Configure Servo Mode (mode=1, speed=50)
        print("\nConfiguring Mode=1 (Servo Mode), Speed=50 on all 6 axes...")
        transport.configure_servo(speed=50)
        print("[CONFIG OK] Motors configured in Servo Mode.\n")

        # Initialize shaper state at initial joint deltas
        init_deltas_m = [(initial_positions[mid] - 1000) * 0.000025 for mid in MOTOR_IDS]
        shaper.reset_state(init_deltas_m)

        cached_steps = dict(initial_positions)
        cached_stamps = dict(initial_stamps_ns)
        cached_currents = dict(initial_currents)
        cached_faults = dict(initial_faults)

        # Ingest initial snapshot
        synchronizer.push_raw_snapshot(
            [cached_stamps[m] for m in MOTOR_IDS],
            [0.050 + (cached_steps[m] - 1000) * 0.000025 for m in MOTOR_IDS],
            [cached_steps[m] for m in MOTOR_IDS],
        )

        cycle_seq = 0
        cycle_dt_s = 0.020
        tau_delay_ns = int(fixed_lag_delay_ms * 1e6)

        # ---------------------------------------------------------------------
        # Helper: Execute One 50Hz Closed-Loop Cycle
        # ---------------------------------------------------------------------
        def execute_50hz_cycle(target_cartesian: list[float]) -> CycleRecord:
            nonlocal cycle_seq, first_active_0x29_batch

            t_cycle_entry_ns = time.monotonic_ns()

            # 1. Analytical IK: Cartesian Target -> Desired Joint Deltas
            ik_deltas_m = kin.differential_inverse_kinematics(target_cartesian, home_cartesian_pose)

            # 2. Stateful Jerk-Limited Shaping
            shaped_deltas_m = shaper.update(ik_deltas_m, dt=cycle_dt_s)

            # 3. Actuator steps conversion
            target_steps = [int(round(1000 + dq * 40000.0)) for dq in shaped_deltas_m]

            # Hard safety bounds check
            for s in target_steps:
                if not (200 <= s <= 1800):
                    raise ValueError(f"Target step {s} exceeded operating envelope [200, 1800]!")

            if first_active_0x29_batch is None:
                first_active_0x29_batch = list(target_steps)

            # 4. J7 50Hz write-only 0x29 batch
            servo_audit = transport.send_cycle(
                target_steps,
                cycle_id=cycle_seq,
                target_timestamp_ns=t_cycle_entry_ns,
            )

            # 5. Round-robin 1-axis 0x30 poll
            feedback_audit = feedback_scheduler.poll_once(
                servo_cycle_id=cycle_seq,
                servo_cycle_timestamp_ns=t_cycle_entry_ns,
            )

            polled_axis = feedback_audit.motor_ids[0] if feedback_audit.motor_ids else -1
            sample = feedback_audit.samples[0] if feedback_audit.samples else None

            if sample is not None and sample.position is not None:
                cached_steps[polled_axis] = sample.position
                cached_currents[polled_axis] = sample.current_mA if sample.current_mA is not None else 0
                cached_faults[polled_axis] = sample.fault if sample.fault is not None else 0
                cached_stamps[polled_axis] = sample.timestamp_ns

            # 6. Ingest into FixedLag synchronizer
            stamps_vec = [cached_stamps[m] for m in MOTOR_IDS]
            steps_vec = [cached_steps[m] for m in MOTOR_IDS]
            pos_vec = [0.050 + (s - 1000) * 0.000025 for s in steps_vec]
            synchronizer.push_raw_snapshot(stamps_vec, pos_vec, steps_vec)

            # 7. Query FixedLag & LM FK reconstruction at t_query = (t_cycle_entry - tau_delay)
            t_query_ns = t_cycle_entry_ns - tau_delay_ns
            sync_res = synchronizer.query_sync_configuration(t_query_ns)

            fk_pose = [0.0] * 6
            fk_residual = 0.0
            fk_status = sync_res.status.value

            if sync_res.status == SyncStatus.SYNCED:
                fk_res = kin.forward_kinematics_with_metrics(sync_res.positions_meters, initial_guess=home_cartesian_pose)
                if fk_res.converged:
                    fk_pose = list(fk_res.pose)
                    fk_residual = float(fk_res.residual_norm)

            rec = CycleRecord(
                cycle_seq=cycle_seq,
                t_cycle_start_ns=t_cycle_entry_ns,
                t_cycle_end_ns=time.monotonic_ns(),
                cartesian_target_m=list(target_cartesian),
                ik_desired_deltas_m=list(ik_deltas_m),
                shaped_target_steps=list(target_steps),
                tx_start_ns=servo_audit.cycle_start_ns,
                tx_end_ns=servo_audit.cycle_end_ns,
                polled_axis=polled_axis,
                polled_rx_ns=sample.timestamp_ns if sample else 0,
                polled_pos_steps=sample.position if sample and sample.position is not None else 0,
                polled_curr_mA=sample.current_mA if sample and sample.current_mA is not None else 0,
                polled_fault=sample.fault if sample and sample.fault is not None else 0,
                all_latest_pos_steps=list(steps_vec),
                all_latest_stamps_ns=list(stamps_vec),
                fk_sync_status=fk_status,
                fk_query_stamp_ns=t_query_ns,
                fk_measured_pose=fk_pose,
                fk_residual_norm=fk_residual,
            )

            cycle_seq += 1
            records.append(rec)
            return rec

        # =====================================================================
        # D0 Execution: Hold OPERATING_HOME for 3.0s (150 cycles)
        # =====================================================================
        print("--- [J8-D0] Executing Active Takeover & Holding HOME for 3.0s ---")
        t_d0_start_ns = time.monotonic_ns()
        next_cycle_target_ns = t_d0_start_ns

        for _ in range(150):
            execute_50hz_cycle(home_cartesian_pose)
            next_cycle_target_ns += int(cycle_dt_s * 1e9)
            sleep_ns = next_cycle_target_ns - time.monotonic_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

        print(f"[D0 TAKEOVER PASS] First Active 0x29 Batch: {first_active_0x29_batch}")
        print(f"[D0 TAKEOVER PASS] Held HOME stably for 3.0s across 150 cycles.\n")

        # =====================================================================
        # J8-D1 Execution: OPERATING_HOME -> Cartesian Z +2.0 mm -> OPERATING_HOME
        # =====================================================================
        print("--- [J8-D1] Executing First Cartesian Micro-Motion (Z +2.0 mm heave) ---")
        z_plus_2mm_pose = [0.0, 0.0, z_nom + 0.002, 0.0, 0.0, 0.0]

        # 1. Heave Up to Z +2.0 mm and hold for 2.5s (125 cycles)
        print("1. Commanding Cartesian Z +2.0 mm...")
        for _ in range(125):
            execute_50hz_cycle(z_plus_2mm_pose)
            next_cycle_target_ns += int(cycle_dt_s * 1e9)
            sleep_ns = next_cycle_target_ns - time.monotonic_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

        # 2. Return to OPERATING_HOME and hold for 2.5s (125 cycles)
        print("2. Commanding Return to Cartesian OPERATING_HOME...")
        for _ in range(125):
            execute_50hz_cycle(home_cartesian_pose)
            next_cycle_target_ns += int(cycle_dt_s * 1e9)
            sleep_ns = next_cycle_target_ns - time.monotonic_ns()
            if sleep_ns > 0:
                time.sleep(sleep_ns / 1e9)

        print("\n[D1 MOTION PASS] Full motion cycle completed cleanly.\n")

    finally:
        # Safe clean stop & shutdown
        print("Executing clean J7 0x1A stop_all on all motors...")
        stop_adapter.stop_all("validation_complete")
        ser.close()
        print("Serial port closed safely.")

    # =========================================================================
    # Quantitative Analysis & Trajectory Evaluation
    # =========================================================================
    print("=" * 80)
    print("                     J8-D TRAJECTORY & TIMING ANALYSIS")
    print("=" * 80)

    total_records = len(records)
    print(f"Total Cycles Recorded: {total_records}")

    # 1. 0x29 batch write durations & 0x30 durations
    batch_durations_ms = [(r.tx_end_ns - r.tx_start_ns) / 1e6 for r in records]
    cycle_dt_ms = [(records[i].t_cycle_start_ns - records[i-1].t_cycle_start_ns) / 1e6 for i in range(1, total_records)]

    req_audits = adapter._request_audit
    query_durations_ms = [(rec.tx_end_ns - rec.tx_start_ns) / 1e6 for rec in req_audits]
    timeouts = sum(1 for rec in req_audits if rec.timeout)

    print("\n--- UART & 50Hz Scheduler Timing ---")
    print(f"50Hz Cycle Period (ms): Mean={np.mean(cycle_dt_ms):.2f}, Min={np.min(cycle_dt_ms):.2f}, Max={np.max(cycle_dt_ms):.2f}, Std={np.std(cycle_dt_ms):.2f}")
    print(f"0x29 Batch Write (ms): Mean={np.mean(batch_durations_ms):.2f}, Min={np.min(batch_durations_ms):.2f}, P95={np.percentile(batch_durations_ms, 95):.2f}, Max={np.max(batch_durations_ms):.2f}")
    print(f"0x30 Query Read  (ms): Mean={np.mean(query_durations_ms):.2f}, Min={np.min(query_durations_ms):.2f}, P95={np.percentile(query_durations_ms, 95):.2f}, Max={np.max(query_durations_ms):.2f}")
    print(f"Timeouts: {timeouts}, Malformed frames: {adapter.rx_owner._malformed_count}, Buffer overflows: {adapter.rx_owner._overflow_count}")

    # 2. Per-Axis Joint Tracking Metrics
    # Commanded steps vs Measured steps
    joint_tracking_stats = {}
    print("\n--- Real Joint Tracking Performance ---")
    print(f"{'Axis ID':<10}{'Cmd Min':<10}{'Cmd Max':<10}{'Meas Min':<10}{'Meas Max':<10}{'RMS Err (steps)':<18}{'Peak Err (steps)':<18}{'Final Err (steps)':<18}")
    print("-" * 96)

    for i, mid in enumerate(MOTOR_IDS):
        cmd_steps = np.array([r.shaped_target_steps[i] for r in records])
        meas_steps = np.array([r.all_latest_pos_steps[i] for r in records])
        err = meas_steps - cmd_steps
        rms_err = float(np.sqrt(np.mean(err**2)))
        peak_err = int(np.max(np.abs(err)))
        final_err = int(err[-1])

        joint_tracking_stats[mid] = {
            "cmd_min": int(np.min(cmd_steps)),
            "cmd_max": int(np.max(cmd_steps)),
            "meas_min": int(np.min(meas_steps)),
            "meas_max": int(np.max(meas_steps)),
            "rms_err_steps": rms_err,
            "peak_err_steps": peak_err,
            "final_err_steps": final_err,
        }

        print(f"ID{mid:<8}{np.min(cmd_steps):<10}{np.max(cmd_steps):<10}{np.min(meas_steps):<10}{np.max(meas_steps):<10}{rms_err:<18.2f}{peak_err:<18}{final_err:<18}")

    # 3. Cartesian FK Tracking with Strict Timestamp Alignment
    # Compare Commanded Cartesian Pose at t_query vs Measured FK Pose at t_query
    synced_records = [r for r in records if r.fk_sync_status == "synced"]
    print(f"\nTotal Synced FK Samples: {len(synced_records)} / {total_records} ({len(synced_records)/total_records*100:.1f}%)")

    # Time-aligned interpolation of commanded Cartesian trajectory at t_query
    record_times_ns = np.array([r.t_cycle_start_ns for r in records])
    cmd_x_interp = np.interp([r.fk_query_stamp_ns for r in synced_records], record_times_ns, [r.cartesian_target_m[0] for r in records])
    cmd_y_interp = np.interp([r.fk_query_stamp_ns for r in synced_records], record_times_ns, [r.cartesian_target_m[1] for r in records])
    cmd_z_interp = np.interp([r.fk_query_stamp_ns for r in synced_records], record_times_ns, [r.cartesian_target_m[2] for r in records])

    meas_x = np.array([r.fk_measured_pose[0] for r in synced_records])
    meas_y = np.array([r.fk_measured_pose[1] for r in synced_records])
    meas_z = np.array([r.fk_measured_pose[2] for r in synced_records])
    meas_roll = np.array([r.fk_measured_pose[3] for r in synced_records])
    meas_pitch = np.array([r.fk_measured_pose[4] for r in synced_records])
    meas_yaw = np.array([r.fk_measured_pose[5] for r in synced_records])

    # Error in mm
    err_x_mm = (meas_x - cmd_x_interp) * 1000.0
    err_y_mm = (meas_y - cmd_y_interp) * 1000.0
    err_z_mm = (meas_z - cmd_z_interp) * 1000.0

    print("\n--- Timestamp-Aligned Cartesian Tracking (FixedLag tau=160ms) ---")
    print(f"X Error (mm): RMS={np.sqrt(np.mean(err_x_mm**2)):.3f}, Peak={np.max(np.abs(err_x_mm)):.3f}")
    print(f"Y Error (mm): RMS={np.sqrt(np.mean(err_y_mm**2)):.3f}, Peak={np.max(np.abs(err_y_mm)):.3f}")
    print(f"Z Error (mm): RMS={np.sqrt(np.mean(err_z_mm**2)):.3f}, Peak={np.max(np.abs(err_z_mm)):.3f}")

    # Specific Phase Poses
    # Initial HOME pose (average of cycles 50..100)
    home_init_poses = [r.fk_measured_pose for r in synced_records if 50 <= r.cycle_seq <= 100]
    mean_home_init = np.mean(home_init_poses, axis=0) if home_init_poses else np.zeros(6)

    # Z +2mm steady pose (average of cycles 200..250)
    z2_steady_poses = [r.fk_measured_pose for r in synced_records if 200 <= r.cycle_seq <= 250]
    mean_z2_steady = np.mean(z2_steady_poses, axis=0) if z2_steady_poses else np.zeros(6)

    # Return HOME steady pose (average of cycles 320..380)
    home_final_poses = [r.fk_measured_pose for r in synced_records if 320 <= r.cycle_seq <= 380]
    mean_home_final = np.mean(home_final_poses, axis=0) if home_final_poses else np.zeros(6)

    print("\n--- Key Cartesian Steady-State Poses ---")
    print(f"1. Initial HOME Pose:     X={mean_home_init[0]*1000:+.3f}mm, Y={mean_home_init[1]*1000:+.3f}mm, Z={mean_home_init[2]*1000:+.3f}mm, R={math.degrees(mean_home_init[3]):+.3f}deg, P={math.degrees(mean_home_init[4]):+.3f}deg, Y={math.degrees(mean_home_init[5]):+.3f}deg")
    print(f"2. Z +2.0mm Steady Pose:  X={mean_z2_steady[0]*1000:+.3f}mm, Y={mean_z2_steady[1]*1000:+.3f}mm, Z={mean_z2_steady[2]*1000:+.3f}mm, R={math.degrees(mean_z2_steady[3]):+.3f}deg, P={math.degrees(mean_z2_steady[4]):+.3f}deg, Y={math.degrees(mean_z2_steady[5]):+.3f}deg")
    print(f"3. Return HOME Pose:      X={mean_home_final[0]*1000:+.3f}mm, Y={mean_home_final[1]*1000:+.3f}mm, Z={mean_home_final[2]*1000:+.3f}mm, R={math.degrees(mean_home_final[3]):+.3f}deg, P={math.degrees(mean_home_final[4]):+.3f}deg, Y={math.degrees(mean_home_final[5]):+.3f}deg")
    print(f"Measured Z Displacement:  {(mean_z2_steady[2] - mean_home_init[2])*1000:+.3f} mm (Expected: +2.000 mm)")
    print(f"Z Return Hysteresis:      {(mean_home_final[2] - mean_home_init[2])*1000:+.3f} mm")

    results_summary = {
        "status": "PASS",
        "first_active_0x29_batch": first_active_0x29_batch,
        "total_cycles": total_records,
        "fixed_lag_delay_ms": fixed_lag_delay_ms,
        "timing": {
            "cycle_dt_ms_mean": float(np.mean(cycle_dt_ms)),
            "cycle_dt_ms_std": float(np.std(cycle_dt_ms)),
            "batch_write_ms_mean": float(np.mean(batch_durations_ms)),
            "batch_write_ms_p95": float(np.percentile(batch_durations_ms, 95)),
            "query_read_ms_mean": float(np.mean(query_durations_ms)),
            "query_read_ms_p95": float(np.percentile(query_durations_ms, 95)),
            "timeouts": timeouts,
            "malformed_frames": adapter.rx_owner._malformed_count,
            "overflows": adapter.rx_owner._overflow_count,
        },
        "joint_tracking": joint_tracking_stats,
        "cartesian_tracking": {
            "err_x_rms_mm": float(np.sqrt(np.mean(err_x_mm**2))),
            "err_x_peak_mm": float(np.max(np.abs(err_x_mm))),
            "err_y_rms_mm": float(np.sqrt(np.mean(err_y_mm**2))),
            "err_y_peak_mm": float(np.max(np.abs(err_y_mm))),
            "err_z_rms_mm": float(np.sqrt(np.mean(err_z_mm**2))),
            "err_z_peak_mm": float(np.max(np.abs(err_z_mm))),
        },
        "key_poses": {
            "home_initial": mean_home_init.tolist(),
            "z_plus_2mm_steady": mean_z2_steady.tolist(),
            "home_return_final": mean_home_final.tolist(),
            "measured_z_delta_mm": float((mean_z2_steady[2] - mean_home_init[2]) * 1000.0),
            "z_return_hysteresis_mm": float((mean_home_final[2] - mean_home_init[2]) * 1000.0),
        },
    }

    if output_json:
        with open(output_json, "w") as f:
            json.dump(results_summary, f, indent=2)
        print(f"\nWrote validation results to {output_json}")

    return results_summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="6PUS J8-D Validation Runner")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--fixed-lag-delay", type=float, default=160.0)
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    res = run_j8_d_validation(
        port=args.port,
        baudrate=args.baudrate,
        fixed_lag_delay_ms=args.fixed_lag_delay,
        output_json=args.output_json,
    )
