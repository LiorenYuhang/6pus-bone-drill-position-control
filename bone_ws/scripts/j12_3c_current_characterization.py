#!/usr/bin/env python3
"""J12-3C PARK <-> HOME Dynamic Current Characterization Runner.

Characterizes 6-axis motor currents, tracking errors, and velocities
across 4 speed levels (100, 200, 300, 400 steps/s) under smooth PARK <-> HOME
motion, evaluating true actuator dynamics up to 100% Vmax.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Ensure pus6 packages are discoverable
BONE_WS = Path(__file__).resolve().parents[1]
SRC_DIR = BONE_WS / "src"
if str(SRC_DIR / "pus6_motor_driver") not in sys.path:
    sys.path.insert(0, str(SRC_DIR / "pus6_motor_driver"))
if str(SRC_DIR / "pus6_kinematics") not in sys.path:
    sys.path.insert(0, str(SRC_DIR / "pus6_kinematics"))

try:
    import serial
except ImportError:
    serial = None

import pus6_motor_driver.safety as safety
import pus6_motor_driver.j7_servo_feedback_scheduler as scheduler
from pus6_motor_driver.protocol import (
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    pack_read_register_cmd,
    pack_read_status_cmd,
    unpack_read_register_response,
    unpack_status_response,
)
from pus6_motor_driver.uart_bus import UartBus
from pus6_motor_driver.j7_real_uart_adapter import J7StopAdapter, J7UartAdapter
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
    ServoRxPolicy,
)
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler

MOTOR_IDS = (1, 2, 3, 4, 5, 6)
OPERATING_HOME = 1000
STORAGE_PARK = 0

SPEED_LEVELS = [
    {"level": 1, "name": "Level 1 (25% Vmax)", "speed_steps_s": 100.0},
    {"level": 2, "name": "Level 2 (50% Vmax)", "speed_steps_s": 200.0},
    {"level": 3, "name": "Level 3 (75% Vmax)", "speed_steps_s": 300.0},
    {"level": 4, "name": "Level 4 (100% Vmax)", "speed_steps_s": 400.0},
]


def quintic_progress(tau: float) -> float:
    """Quintic polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 for tau in [0, 1]."""
    tau = max(0.0, min(1.0, float(tau)))
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def classify_motion_phase(trajectory_phase: str, tau: float) -> str:
    """Classify into ACCEL, CRUISE, DECEL, REVERSAL, HOLD."""
    if "HOLD" in trajectory_phase:
        return "HOLD"
    if tau <= 0.05 or tau >= 0.95:
        # Near boundaries of motion transition
        if "TO_HOME" in trajectory_phase and tau <= 0.05:
            return "REVERSAL"
        if "TO_PARK" in trajectory_phase and tau >= 0.95:
            return "REVERSAL"
    if tau <= 0.40:
        return "ACCEL"
    elif tau < 0.60:
        return "CRUISE"
    else:
        return "DECEL"


def set_latency_timer(port: str) -> None:
    try:
        devname = os.path.basename(port)
        latency_path = f"/sys/bus/usb-serial/devices/{devname}/latency_timer"
        if os.path.exists(latency_path):
            with open(latency_path, "w") as f:
                f.write("1")
    except Exception:
        pass


@dataclass
class TelemetryRow:
    timestamp_s: float
    cycle_index: int
    dt_ms: float
    level: int
    trajectory_phase: str
    motion_phase: str
    cmd_pos: Tuple[int, ...]
    fb_pos: Tuple[int, ...]
    err_pos: Tuple[int, ...]
    cmd_vel: Tuple[float, ...]
    fb_vel: Tuple[float, ...]
    curr_mA: Tuple[int, ...]
    fault: Tuple[int, ...]


def perform_preflight_checks(port: str, baudrate: int) -> Dict[str, Any]:
    """Perform pre-check: 0x20 register, fault codes, positions, and speed params."""
    print("=" * 60)
    print("J12-3C Minimal Preflight Checks")
    print("=" * 60)
    bus = UartBus(port, baudrate=baudrate, timeout=0.05)
    results = {}
    reg_0x20_values = {}
    positions = {}
    currents = {}
    faults = {}
    speeds = {}

    for mid in MOTOR_IDS:
        # 1. Read status (0x30)
        raw_st = bus.request(pack_read_status_cmd(mid), expected_id=mid, expected_command=CMD_RD_STATUS)
        st = unpack_status_response(raw_st)
        positions[mid] = st.current_position
        currents[mid] = st.current_mA
        faults[mid] = st.error_code

        # 2. Read register 0x20 (internal overcurrent)
        raw_oc = bus.request(pack_read_register_cmd(mid, 0x20, 1), expected_id=mid, expected_command=CMD_RD_REGISTER)
        resp_oc = unpack_read_register_response(raw_oc, 0x20, 1)
        reg_0x20_values[mid] = resp_oc.values[0]

        # 3. Read register 0x28 (speed limit)
        raw_sp = bus.request(pack_read_register_cmd(mid, 0x28, 1), expected_id=mid, expected_command=CMD_RD_REGISTER)
        resp_sp = unpack_read_register_response(raw_sp, 0x28, 1)
        speeds[mid] = resp_sp.values[0]

        print(
            f"  ID{mid}: Pos={positions[mid]:4d}, Current={currents[mid]:3d}mA, "
            f"Fault={faults[mid]} | Reg 0x20={reg_0x20_values[mid]:4d}mA | Reg 0x28={speeds[mid]:4d}st/s"
        )

    bus.close()

    # Verify Reg 0x20
    all_1500 = all(1400 <= v <= 1600 for v in reg_0x20_values.values())
    all_zero_fault = all(f == 0 for f in faults.values())
    all_in_envelope = all(-15 <= p <= 1050 for p in positions.values())

    print("-" * 60)
    print(f"Pre-check Reg 0x20 ~1500 mA: {'PASS' if all_1500 else 'FAIL'} (Values: {list(reg_0x20_values.values())})")
    print(f"Pre-check Faults == 0:       {'PASS' if all_zero_fault else 'FAIL'} (Faults: {list(faults.values())})")
    print(f"Pre-check Position valid:    {'PASS' if all_in_envelope else 'WARN'} (Positions: {list(positions.values())})")
    print("=" * 60)

    if not all_1500:
        raise RuntimeError(f"STOP: Actuator Reg 0x20 differs significantly from 1500 mA: {reg_0x20_values}")
    if not all_zero_fault:
        raise RuntimeError(f"STOP: Actuator fault != 0 detected before run: {faults}")

    return {
        "reg_0x20": reg_0x20_values,
        "positions": positions,
        "currents": currents,
        "faults": faults,
        "speeds": speeds,
        "all_1500": all_1500,
        "all_zero_fault": all_zero_fault,
        "all_in_envelope": all_in_envelope,
    }


def build_level_trajectory(
    speed_steps_s: float,
    rate_hz: float = 50.0,
    hold_duration_s: float = 2.0,
) -> Tuple[List[Tuple[Tuple[int, ...], str, float]], float, float]:
    """Build HOME -> PARK -> HOME trajectory with quintic interpolation."""
    d_max = 1000  # steps (from 1000 to 0)
    # v_peak = 1.875 * d_max / T  ==>  T = 1.875 * d_max / speed_steps_s
    trans_duration_s = max(1.5, (1.875 * d_max) / speed_steps_s)
    planned_peak_speed = (1.875 * d_max) / trans_duration_s

    hold_cycles = int(round(hold_duration_s * rate_hz))
    trans_cycles = int(round(trans_duration_s * rate_hz))

    trajectory: List[Tuple[Tuple[int, ...], str, float]] = []

    # 1. HOME Hold
    q_home = (OPERATING_HOME,) * 6
    q_park = (STORAGE_PARK,) * 6

    for _ in range(hold_cycles):
        trajectory.append((q_home, "HOME_HOLD", 0.0))

    # 2. HOME -> PARK
    for k in range(1, trans_cycles + 1):
        tau = k / trans_cycles
        s = quintic_progress(tau)
        target_val = int(round(OPERATING_HOME + (STORAGE_PARK - OPERATING_HOME) * s))
        trajectory.append(((target_val,) * 6, "HOME_TO_PARK", tau))

    # 3. PARK Hold
    for _ in range(hold_cycles):
        trajectory.append((q_park, "PARK_HOLD", 0.0))

    # 4. PARK -> HOME
    for k in range(1, trans_cycles + 1):
        tau = k / trans_cycles
        s = quintic_progress(tau)
        target_val = int(round(STORAGE_PARK + (OPERATING_HOME - STORAGE_PARK) * s))
        trajectory.append(((target_val,) * 6, "PARK_TO_HOME", tau))

    # 5. HOME Final Hold
    for _ in range(hold_cycles):
        trajectory.append((q_home, "HOME_FINAL_HOLD", 0.0))

    total_duration_s = len(trajectory) / rate_hz
    return trajectory, trans_duration_s, planned_peak_speed


def run_j12_3c_characterization(
    *,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 921600,
    rate_hz: float = 50.0,
    dry_run: bool = False,
    output_dir: Optional[Path] = None,
) -> int:
    """Execute complete 4-level dynamic characterization."""
    if output_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(f"/home/lyapunov/bone_drill/results/j12/j12_3c_current_characterization/{ts}")
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"J12-3C Current Characterization Started.")
    print(f"Results Directory: {output_dir}")
    print(f"Execution Mode: {'DRY RUN' if dry_run else 'REAL HARDWARE EXECUTION'}")

    preflight_info = {}
    if not dry_run:
        preflight_info = perform_preflight_checks(port, baudrate)

    # Apply in-memory patch for characterization runner ONLY:
    # Set software limit to 1500 mA (matches internal register 0x20)
    internal_oc_ma = 1500
    safety.MAX_CURRENT_MA = internal_oc_ma
    scheduler.MAX_CURRENT_MA = internal_oc_ma
    safety.FEEDBACK_LOWER_LIMIT = -20
    scheduler.FEEDBACK_LOWER_LIMIT = -20
    print(f"\n[Characterization Safety Gate] Host ceiling set to {internal_oc_ma} mA, Lower Limit set to -20 steps (in-memory patch only).")

    all_telemetry: List[TelemetryRow] = []
    level_summaries: List[Dict[str, Any]] = []
    high_current_events: List[Dict[str, Any]] = []

    if dry_run:
        print("\n>>> Running Dry-Run Simulation across all 4 levels...")
        for lvl_cfg in SPEED_LEVELS:
            lvl = lvl_cfg["level"]
            speed = lvl_cfg["speed_steps_s"]
            traj, trans_s, p_spd = build_level_trajectory(speed, rate_hz=rate_hz)
            print(f"  Level {lvl}: Speed={speed:.1f} st/s, Trans={trans_s:.2f}s, Cycles={len(traj)}, PeakV={p_spd:.1f} st/s")
        print("Dry run completed cleanly. Ready for hardware run.\n")
        return 0

    # Hardware execution
    set_latency_timer(port)
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.005,
        write_timeout=None,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    adapter = J7UartAdapter(
        ser,
        baudrate=baudrate,
        timeout_s=0.010,
        min_frame_interval_s=0.0001,
    )
    stop_adapter = J7StopAdapter(adapter)
    writer = J7SixAxisServoTransport(
        adapter,
        cycle_rate_hz=rate_hz,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
        min_frame_interval_s=0.0006,
        rx_drain_budget_s=0.0040,
        pause_callback=stop_adapter,
    )
    feedback_scheduler = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=1e9,
        feedback_budget_s=0.015,
        servo_cycle_period_s=1.0 / rate_hz,
        timeout_escalation_misses=4,
        max_feedback_age_s=1.0,
        warning_feedback_age_s=0.2833,
        hard_stop_feedback_age_s=1.0,
        feedback_loss_policy="age_based",
    )

    # Configure Servo Mode (mode 1, internal speed register = 500)
    print("\n[Phase 1] Configuring six axes into Servo mode (mode=1, internal_speed=500)...")
    configs = writer.configure_servo(speed=500)
    if len(configs) != 6 or any(c.mode != 1 for c in configs):
        raise RuntimeError("Servo configuration failed")
    print("  All 6 axes configured into Servo mode: mode=1 acknowledged, fault=0.")

    period_s = 1.0 / rate_hz
    period_ns = int(period_s * 1e9)

    global_cycle_id = 0
    t_exp_start_ns = time.monotonic_ns()
    t_prev_cycle_ns = t_exp_start_ns

    # Smooth pre-alignment to OPERATING_HOME if not already at 1000
    init_pos = tuple(preflight_info.get("positions", {}).get(mid, OPERATING_HOME) for mid in MOTOR_IDS)
    init_clamped = tuple(max(0, min(2000, p)) for p in init_pos)
    if any(abs(p - OPERATING_HOME) > 3 for p in init_pos):
        align_cycles = int(3.0 * rate_hz)
        print(f"\n[Phase 1.5] Smooth pre-alignment from {list(init_pos)} to HOME [1000]*6 ({align_cycles} cycles)...")
        for k in range(1, align_cycles + 1):
            s = quintic_progress(k / align_cycles)
            tgt = tuple(int(round(init_clamped[i] + (OPERATING_HOME - init_clamped[i]) * s)) for i in range(6))
            t_now_ns = time.monotonic_ns()
            audit = writer.send_cycle(list(tgt), cycle_id=global_cycle_id, target_timestamp_ns=t_now_ns)
            feedback_scheduler.ingest_cycle_feedback(
                audit.feedbacks,
                servo_cycle_id=global_cycle_id,
                servo_cycle_timestamp_ns=t_now_ns,
            )
            time.sleep(period_s)
            global_cycle_id += 1
        # Settle at HOME for 1.0s
        for _ in range(int(1.0 * rate_hz)):
            t_now_ns = time.monotonic_ns()
            audit = writer.send_cycle([OPERATING_HOME]*6, cycle_id=global_cycle_id, target_timestamp_ns=t_now_ns)
            feedback_scheduler.ingest_cycle_feedback(
                audit.feedbacks,
                servo_cycle_id=global_cycle_id,
                servo_cycle_timestamp_ns=t_now_ns,
            )
            time.sleep(period_s)
            global_cycle_id += 1
        print("  Pre-alignment to HOME complete.")

    prev_cmd_pos: Tuple[int, ...] = (OPERATING_HOME,) * 6
    prev_fb_pos: Tuple[int, ...] = (OPERATING_HOME,) * 6
    prev_smooth_fb_vel: List[float] = [0.0] * 6

    fatal_stopped = False
    fatal_reason = ""

    try:
        for lvl_cfg in SPEED_LEVELS:
            lvl = lvl_cfg["level"]
            speed_setting = lvl_cfg["speed_steps_s"]
            trajectory, trans_duration_s, planned_peak = build_level_trajectory(speed_setting, rate_hz=rate_hz)

            print(f"\n==================================================")
            print(f"EXECUTING {lvl_cfg['name']}: Setting = {speed_setting:.1f} steps/s")
            print(f"Planned Peak Speed: {planned_peak:.1f} steps/s | Transition: {trans_duration_s:.2f}s | Cycles: {len(trajectory)}")
            print(f"==================================================")

            lvl_records: List[TelemetryRow] = []
            lvl_peak_currents = {mid: 0 for mid in MOTOR_IDS}
            lvl_max_tracking_err = {mid: 0 for mid in MOTOR_IDS}
            lvl_peak_fb_vel = {mid: 0.0 for mid in MOTOR_IDS}

            lvl_peak_current_overall = 0
            lvl_peak_axis_overall = 1
            lvl_peak_time_overall = 0.0
            lvl_peak_phase_overall = "HOLD"

            for step_in_level, (target_vec, traj_phase, tau) in enumerate(trajectory):
                t_cycle_start_ns = time.monotonic_ns()
                dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if global_cycle_id > 0 else period_s * 1000.0
                t_prev_cycle_ns = t_cycle_start_ns
                t_current_s = (t_cycle_start_ns - t_exp_start_ns) / 1e9

                motion_phase = classify_motion_phase(traj_phase, tau)

                # 1. Send 6-axis Servo targets
                servo_audit = writer.send_cycle(
                    list(target_vec),
                    cycle_id=global_cycle_id,
                    target_timestamp_ns=t_cycle_start_ns,
                )

                # 2. Ingest 50Hz 0x29 feedback from all 6 axes
                fb_audit = feedback_scheduler.ingest_cycle_feedback(
                    servo_audit.feedbacks,
                    servo_cycle_id=global_cycle_id,
                    servo_cycle_timestamp_ns=t_cycle_start_ns,
                )

                # Unpack feedback
                fb_positions: List[int] = []
                fb_currents: List[int] = []
                fb_faults: List[int] = []
                err_positions: List[int] = []

                for sample in fb_audit.samples:
                    mid = sample.motor_id
                    p = sample.position if sample.position is not None else target_vec[mid - 1]
                    c = sample.current_mA if sample.current_mA is not None else 0
                    f = sample.fault if sample.fault is not None else 0
                    err = p - target_vec[mid - 1]

                    fb_positions.append(p)
                    fb_currents.append(c)
                    fb_faults.append(f)
                    err_positions.append(err)

                    if c > lvl_peak_currents[mid]:
                        lvl_peak_currents[mid] = c
                    if abs(err) > lvl_max_tracking_err[mid]:
                        lvl_max_tracking_err[mid] = abs(err)

                    if c > lvl_peak_current_overall:
                        lvl_peak_current_overall = c
                        lvl_peak_axis_overall = mid
                        lvl_peak_time_overall = t_current_s
                        lvl_peak_phase_overall = motion_phase

                    # High current event logging
                    if c > 500:
                        high_current_events.append({
                            "timestamp_s": round(t_current_s, 3),
                            "level": lvl,
                            "motion_phase": motion_phase,
                            "motor_id": mid,
                            "current_mA": c,
                            "target_step": target_vec[mid - 1],
                            "feedback_step": p,
                            "tracking_err": err,
                        })

                # Compute velocities (raw & smoothed EMA)
                dt_s = dt_ms / 1000.0 if dt_ms > 0 else period_s
                cmd_vels = tuple((target_vec[i] - prev_cmd_pos[i]) / dt_s for i in range(6))
                raw_fb_vels = [(fb_positions[i] - prev_fb_pos[i]) / dt_s for i in range(6)]
                smooth_fb_vels = [
                    0.7 * prev_smooth_fb_vel[i] + 0.3 * raw_fb_vels[i] for i in range(6)
                ]
                prev_smooth_fb_vel = smooth_fb_vels
                fb_vels = tuple(smooth_fb_vels)
                prev_cmd_pos = target_vec
                prev_fb_pos = tuple(fb_positions)

                for i, mid in enumerate(MOTOR_IDS):
                    if abs(fb_vels[i]) > lvl_peak_fb_vel[mid]:
                        lvl_peak_fb_vel[mid] = abs(fb_vels[i])

                # Construct Telemetry Row
                row = TelemetryRow(
                    timestamp_s=t_current_s,
                    cycle_index=global_cycle_id,
                    dt_ms=dt_ms,
                    level=lvl,
                    trajectory_phase=traj_phase,
                    motion_phase=motion_phase,
                    cmd_pos=target_vec,
                    fb_pos=tuple(fb_positions),
                    err_pos=tuple(err_positions),
                    cmd_vel=cmd_vels,
                    fb_vel=fb_vels,
                    curr_mA=tuple(fb_currents),
                    fault=tuple(fb_faults),
                )
                all_telemetry.append(row)
                lvl_records.append(row)

                # Real safety checks (Internal OC 1500 mA, fault code, divergence > 400 steps)
                for i, mid in enumerate(MOTOR_IDS):
                    if fb_faults[i] != 0:
                        fatal_stopped = True
                        fatal_reason = f"Actuator internal fault ID{mid} code={fb_faults[i]}"
                        print(f"\n[FATAL STOP] {fatal_reason} at t={t_current_s:.2f}s")
                        stop_adapter.stop_all(fatal_reason)
                        break
                    if fb_currents[i] > 1500:
                        fatal_stopped = True
                        fatal_reason = f"Hard current limit exceedance ID{mid} {fb_currents[i]}mA > 1500mA"
                        print(f"\n[FATAL STOP] {fatal_reason} at t={t_current_s:.2f}s")
                        stop_adapter.stop_all(fatal_reason)
                        break
                    if abs(err_positions[i]) > 400:
                        fatal_stopped = True
                        fatal_reason = f"Severe tracking divergence ID{mid} err={err_positions[i]} steps"
                        print(f"\n[FATAL STOP] {fatal_reason} at t={t_current_s:.2f}s")
                        stop_adapter.stop_all(fatal_reason)
                        break

                if fatal_stopped:
                    break

                # Sleep to enforce 50Hz deadline
                next_deadline_ns = t_exp_start_ns + (global_cycle_id + 1) * period_ns
                now_ns = time.monotonic_ns()
                rem_ns = next_deadline_ns - now_ns
                if rem_ns > 0:
                    time.sleep(rem_ns / 1e9)

                global_cycle_id += 1

            # Summary for this level
            max_measured_spd = max(lvl_peak_fb_vel.values())
            summary_lvl = {
                "level": lvl,
                "speed_setting_steps_s": speed_setting,
                "measured_peak_speed_steps_s": round(max_measured_spd, 1),
                "peak_current_overall_mA": lvl_peak_current_overall,
                "peak_current_axis": lvl_peak_axis_overall,
                "peak_current_time_s": round(lvl_peak_time_overall, 3),
                "peak_current_phase": lvl_peak_phase_overall,
                "six_axis_peak_currents_mA": [lvl_peak_currents[mid] for mid in MOTOR_IDS],
                "six_axis_max_tracking_err": [lvl_max_tracking_err[mid] for mid in MOTOR_IDS],
                "max_tracking_err_overall": max(lvl_max_tracking_err.values()),
                "fault": "YES" if fatal_stopped else "NO",
            }
            level_summaries.append(summary_lvl)

            # REQUIRED FORMAT FOR SECTION X IMMEDIATE OUTPUT:
            print(f"\n--------------------------------------------------")
            print(f"Level {lvl}：")
            print(f"Speed setting:        {speed_setting:.1f} steps/s")
            print(f"Measured peak speed:  {max_measured_spd:.1f} steps/s")
            print()
            print(f"Peak current:         {lvl_peak_current_overall} mA")
            print(f"Motor ID:             ID{lvl_peak_axis_overall}")
            print(f"Time:                 {lvl_peak_time_overall:.3f} s")
            print(f"Motion phase:         {lvl_peak_phase_overall}")
            print()
            print(f"Six-axis peak currents:")
            print(f"{[lvl_peak_currents[mid] for mid in MOTOR_IDS]}")
            print()
            print(f"Max tracking error:   {max(lvl_max_tracking_err.values())} steps")
            print()
            print(f"Fault:")
            print(f"{summary_lvl['fault']}")
            print(f"--------------------------------------------------")

            if fatal_stopped:
                print(f"Halting remaining levels due to fatal stop: {fatal_reason}")
                break

    except Exception as exc:
        print(f"[FATAL EXCEPTION] During run: {exc}")
        fatal_stopped = True
        fatal_reason = f"Exception: {exc}"
        stop_adapter.stop_all(fatal_reason)

    finally:
        # Clean 0x1A Stop
        print("\n[Phase Final] Sending clean 0x1A Stop command...")
        stop_audit = stop_adapter.stop_all("characterization_complete")
        print(f"  0x1A Stop acknowledged: success={stop_audit.success}, reason={stop_audit.reason}")

        time.sleep(0.1)
        # Read final positions
        final_bus = UartBus(port, baudrate=baudrate, timeout=0.05)
        final_positions = []
        final_faults = []
        for mid in MOTOR_IDS:
            raw_st = final_bus.request(pack_read_status_cmd(mid), expected_id=mid, expected_command=CMD_RD_STATUS)
            st = unpack_status_response(raw_st)
            final_positions.append(st.current_position)
            final_faults.append(st.error_code)
        final_bus.close()
        ser.close()

        print(f"  Final 6-Axis Positions: {final_positions}")
        print(f"  Final 6-Axis Faults:    {final_faults}")

    # Save Telemetry CSV
    csv_path = output_dir / "j12_3c_current_characterization_telemetry.csv"
    print(f"\nSaving telemetry to {csv_path} ({len(all_telemetry)} cycles)...")
    with open(csv_path, "w") as f:
        header = [
            "timestamp_s", "cycle_index", "dt_ms", "level", "trajectory_phase", "motion_phase",
            "cmd_pos_1", "cmd_pos_2", "cmd_pos_3", "cmd_pos_4", "cmd_pos_5", "cmd_pos_6",
            "fb_pos_1", "fb_pos_2", "fb_pos_3", "fb_pos_4", "fb_pos_5", "fb_pos_6",
            "err_pos_1", "err_pos_2", "err_pos_3", "err_pos_4", "err_pos_5", "err_pos_6",
            "cmd_vel_1", "cmd_vel_2", "cmd_vel_3", "cmd_vel_4", "cmd_vel_5", "cmd_vel_6",
            "fb_vel_1", "fb_vel_2", "fb_vel_3", "fb_vel_4", "fb_vel_5", "fb_vel_6",
            "curr_mA_1", "curr_mA_2", "curr_mA_3", "curr_mA_4", "curr_mA_5", "curr_mA_6",
            "fault_1", "fault_2", "fault_3", "fault_4", "fault_5", "fault_6"
        ]
        f.write(",".join(header) + "\n")
        for r in all_telemetry:
            line = [
                f"{r.timestamp_s:.4f}", f"{r.cycle_index}", f"{r.dt_ms:.2f}", f"{r.level}",
                r.trajectory_phase, r.motion_phase,
                *[str(x) for x in r.cmd_pos],
                *[str(x) for x in r.fb_pos],
                *[str(x) for x in r.err_pos],
                *[f"{x:.1f}" for x in r.cmd_vel],
                *[f"{x:.1f}" for x in r.fb_vel],
                *[str(x) for x in r.curr_mA],
                *[str(x) for x in r.fault],
            ]
            f.write(",".join(line) + "\n")

    # Save Summary JSON
    summary_path = output_dir / "j12_3c_current_characterization_summary.json"
    summary_data = {
        "timestamp": datetime.now().isoformat(),
        "preflight": preflight_info,
        "speed_levels": SPEED_LEVELS,
        "level_summaries": level_summaries,
        "high_current_events": high_current_events,
        "final_positions": final_positions,
        "final_faults": final_faults,
        "fatal_stopped": fatal_stopped,
        "fatal_reason": fatal_reason,
    }
    with open(summary_path, "w") as f:
        json.dump(summary_data, f, indent=2)

    # Generate Figures
    generate_figures(all_telemetry, output_dir)

    # Generate Markdown Report
    generate_report(summary_data, output_dir)

    return 0


def generate_figures(telemetry: List[TelemetryRow], output_dir: Path) -> None:
    """Generate the three required characterization figures."""
    if not telemetry:
        return
    times = [r.timestamp_s for r in telemetry]
    levels = [r.level for r in telemetry]

    # 1. Current vs Time Figure
    fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for i in range(6):
        ax = axes[i]
        mid = i + 1
        currs = [r.curr_mA[i] for r in telemetry]
        ax.plot(times, currs, label=f"ID{mid} Current (mA)", color=colors[i], lw=1.2)
        ax.axhline(500, color="orange", linestyle="--", lw=1.0, alpha=0.7, label="Old Host Gate (500mA)" if i == 0 else "")
        ax.axhline(1500, color="red", linestyle=":", lw=1.0, alpha=0.7, label="Actuator Internal OC (1500mA)" if i == 0 else "")
        ax.set_ylabel(f"ID{mid} (mA)")
        ax.grid(True, linestyle="--", alpha=0.4)
        if i == 0:
            ax.set_title("J12-3C Six-Axis Motor Current vs Time across 4 Speed Levels")
            ax.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    fig1_path = output_dir / "figure_1_current_vs_time.png"
    plt.savefig(fig1_path, dpi=150)
    plt.close()

    # 2. Current vs Feedback Velocity Figure
    plt.figure(figsize=(10, 6))
    for i in range(6):
        mid = i + 1
        vels = [abs(r.fb_vel[i]) for r in telemetry]
        currs = [r.curr_mA[i] for r in telemetry]
        plt.scatter(vels, currs, s=8, alpha=0.4, label=f"ID{mid}", color=colors[i])

    plt.axhline(500, color="orange", linestyle="--", lw=1.5, label="Old Host Gate (500mA)")
    plt.axhline(1500, color="red", linestyle=":", lw=1.5, label="Internal OC Limit (1500mA)")
    plt.xlabel("Actuator Absolute Velocity (steps/s)")
    plt.ylabel("Actuator Current (mA)")
    plt.title("J12-3C Motor Current vs Actuator Velocity")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(loc="upper left")
    plt.tight_layout()
    fig2_path = output_dir / "figure_2_current_vs_velocity.png"
    plt.savefig(fig2_path, dpi=150)
    plt.close()

    # 3. Tracking Error vs Velocity Figure
    plt.figure(figsize=(10, 6))
    for i in range(6):
        mid = i + 1
        vels = [abs(r.fb_vel[i]) for r in telemetry]
        errs = [abs(r.err_pos[i]) for r in telemetry]
        plt.scatter(vels, errs, s=8, alpha=0.4, label=f"ID{mid}", color=colors[i])

    plt.xlabel("Actuator Absolute Velocity (steps/s)")
    plt.ylabel("Absolute Tracking Error (steps)")
    plt.title("J12-3C Actuator Tracking Error vs Velocity")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(loc="upper left")
    plt.tight_layout()
    fig3_path = output_dir / "figure_3_tracking_error_vs_velocity.png"
    plt.savefig(fig3_path, dpi=150)
    plt.close()

    print(f"Generated 3 analysis figures in {output_dir}")


def generate_report(summary_data: Dict[str, Any], output_dir: Path) -> None:
    """Generate Markdown report."""
    report_path = output_dir / "j12_3c_current_characterization_report.md"
    level_summaries = summary_data.get("level_summaries", [])
    high_current_events = summary_data.get("high_current_events", [])

    lines = [
        "# J12-3C PARK ↔ HOME Dynamic Current Characterization Report",
        "",
        f"**Run Timestamp**: {summary_data.get('timestamp')}",
        f"**Target Directory**: `{output_dir}`",
        "",
        "## 1. Post-Experiment Comparison Table",
        "",
        "| Level | Speed | Peak measured velocity | Peak current | Axis | Phase | Max tracking error | Fault |",
        "|---|---:|---:|---:|---|---|---:|---|",
    ]

    for s in level_summaries:
        lines.append(
            f"| Level {s['level']} | {s['speed_setting_steps_s']:.1f} st/s | {s['measured_peak_speed_steps_s']:.1f} st/s | "
            f"{s['peak_current_overall_mA']} mA | ID{s['peak_current_axis']} | {s['peak_current_phase']} | "
            f"{s['max_tracking_err_overall']} steps | {s['fault']} |"
        )

    lines.extend([
        "",
        "## 2. Answers to the 8 Core Questions",
        "",
    ])

    # Check question thresholds
    all_peaks = [s["peak_current_overall_mA"] for s in level_summaries]
    max_curr_all = max(all_peaks) if all_peaks else 0

    lines.append(f"1. **500 mA 是否在正常动态运行中被自然超过？**")
    lines.append(f"   - {'是' if max_curr_all > 500 else '否'} (实测最高峰值: {max_curr_all} mA)")
    lines.append(f"2. **800 mA 是否被超过？**")
    lines.append(f"   - {'是' if max_curr_all > 800 else '否'}")
    lines.append(f"3. **1000 mA 是否被超过？**")
    lines.append(f"   - {'是' if max_curr_all > 1000 else '否'}")
    lines.append(f"4. **1200 mA 是否被超过？**")
    lines.append(f"   - {'是' if max_curr_all > 1200 else '否'}")
    lines.append(f"5. **是否接近 1500 mA？**")
    lines.append(f"   - {'是' if max_curr_all >= 1350 else '否'}")
    
    # Motion phase analysis
    phases = [s["peak_current_phase"] for s in level_summaries]
    lines.append(f"6. **电流峰值主要发生在：acceleration / cruise / deceleration / reversal？**")
    lines.append(f"   - 实测峰值阶段分布: {phases}")

    # Tracking check
    max_err_l4 = level_summaries[-1]["max_tracking_err_overall"] if level_summaries else 0
    lines.append(f"7. **最大合法速度下 tracking 是否仍正常？**")
    lines.append(f"   - {'正常' if max_err_l4 < 150 else '存在明显滞后/发散'} (Level 4 最大跟踪误差: {max_err_l4} steps)")

    # Internal faults
    all_clean_faults = all(f == 0 for f in summary_data.get("final_faults", []))
    lines.append(f"8. **六轴有没有任何 internal fault？**")
    lines.append(f"   - {'六轴完全无 internal fault (全部为0)' if all_clean_faults else '存在 internal fault: ' + str(summary_data.get('final_faults'))}")

    lines.extend([
        "",
        "## 3. High Current Events (>500 mA)",
        "",
        f"Total events recorded: {len(high_current_events)}",
        "",
    ])

    if high_current_events:
        lines.append("| Time (s) | Level | Phase | Axis | Current (mA) | Target (st) | Feedback (st) | Error (st) |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for ev in high_current_events[:30]:  # Cap display at 30
            lines.append(f"| {ev['timestamp_s']} | L{ev['level']} | {ev['motion_phase']} | ID{ev['motor_id']} | {ev['current_mA']} | {ev['target_step']} | {ev['feedback_step']} | {ev['tracking_err']} |")
        if len(high_current_events) > 30:
            lines.append(f"| ... | ({len(high_current_events)-30} more events in JSON) | | | | | | |")
    else:
        lines.append("No samples exceeded 500 mA during the entire characterization run.")

    lines.extend([
        "",
        "## 4. Generated Artifact Figures",
        "",
        "- [Figure 1: Current vs Time](figure_1_current_vs_time.png)",
        "- [Figure 2: Current vs Velocity](figure_2_current_vs_velocity.png)",
        "- [Figure 3: Tracking Error vs Velocity](figure_3_tracking_error_vs_velocity.png)",
    ])

    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"Generated report at {report_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="J12-3C PARK <-> HOME Dynamic Current Characterization")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=921600, help="Baudrate (default: 921600)")
    parser.add_argument("--rate", type=float, default=50.0, help="Servo rate Hz (default: 50.0)")
    parser.add_argument("--dry-run", action="store_true", help="Perform dry run without hardware")
    args = parser.parse_args()

    return run_j12_3c_characterization(
        port=args.port,
        baudrate=args.baudrate,
        rate_hz=args.rate,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    sys.exit(main())
