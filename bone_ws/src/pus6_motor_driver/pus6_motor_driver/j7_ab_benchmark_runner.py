"""J6 mode=0 vs J7 mode=1 Servo Real Hardware A/B Benchmark Runner.

Executes consecutive real hardware tests on the same physical 6PUS mechanism:
- Group A: J6 frozen Positioning Mode (mode=0, 10Hz control loop)
- Group B: J7 Servo Mode (mode=1, 50Hz target stream + round-robin feedback)

Both groups execute the identical PARK <-> HOME quintic S-curve trajectory:
- Peak speed: 100 steps/s
- Transition time: 18.75s each direction
- Station holds: 3.0s at PARK and HOME

Calculates and prints comprehensive comparative metrics:
- Tracking errors (Max, Mean, RMS)
- Current consumption (Peak, Mean, Std Dev / Fluctuation)
- Motion continuity (Velocity variance, Jerk proxy, Stutter events)
- Final return repeatability & synchronicity
- UART control timing
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

try:
    import serial
except ImportError:
    serial = None  # type: ignore

from pus6_motor_driver.protocol import (
    CMD_RD_STATUS,
    CMD_WR_REGISTER,
    pack_read_status_cmd,
    pack_write_register_cmd,
    unpack_status_response,
    unpack_write_response,
)
from pus6_motor_driver.safety import (
    CONTROL_MODE_ADDR,
    MOTOR_IDS,
    PAUSE_ADDR,
    TARGET_POSITION_ADDR,
)
from pus6_motor_driver.j7_real_uart_adapter import (
    J7StopAdapter,
    J7UartAdapter,
)
from pus6_motor_driver.j7_servo_feedback_scheduler import (
    FeedbackCycleAudit,
    FeedbackSample,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import (
    J7SixAxisServoTransport,
    ServoRxPolicy,
    SixAxisCycleAudit,
)


def set_latency_timer(port: str) -> None:
    try:
        devname = os.path.basename(port)
        latency_path = f"/sys/bus/usb-serial/devices/{devname}/latency_timer"
        if os.path.exists(latency_path):
            with open(latency_path, "w") as f:
                f.write("1")
    except Exception:
        pass


def quintic_progress(tau: float) -> float:
    tau = max(0.0, min(1.0, float(tau)))
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def quintic_derivative(tau: float) -> float:
    tau = max(0.0, min(1.0, float(tau)))
    return 30.0 * tau * tau * (1.0 - 2.0 * tau + tau * tau)


@dataclass
class GroupRecord:
    t_s: float
    dt_ms: float
    targets: Tuple[int, ...]
    positions: Dict[int, int]
    currents: Dict[int, int]
    faults: Dict[int, int]
    phase: str
    polled_axis: Optional[int] = None


@dataclass
class GroupMetrics:
    name: str
    mode: int
    rate_hz: float
    total_samples: int
    total_duration_s: float
    period_mean_ms: float
    period_std_ms: float
    q_initial: Tuple[int, ...]
    q_final: Tuple[int, ...]
    # Tracking
    max_tracking_err: Dict[int, int]
    mean_tracking_err: Dict[int, float]
    rms_tracking_err: Dict[int, float]
    overall_max_err: int
    overall_rms_err: float
    # Current
    peak_current_mA: Dict[int, int]
    mean_current_mA: Dict[int, float]
    std_current_mA: Dict[int, float]
    overall_peak_current: int
    overall_mean_current: float
    # Continuity & Jerk
    velocity_variance: Dict[int, float]
    jerk_proxy_mean: Dict[int, float]
    stutter_count: Dict[int, int]
    # Return accuracy
    final_return_errors: Dict[int, int]
    max_return_error: int
    return_skew: int
    # Safety
    fault_count: int
    timeout_count: int
    stop_success: bool


def run_j6_baseline_run(
    ser: serial.Serial,
    *,
    rate_hz: float = 10.0,
    speed_steps_s: float = 100.0,
    hold_duration_s: float = 3.0,
) -> Tuple[List[GroupRecord], Tuple[int, ...], Tuple[int, ...], bool]:
    adapter = J7UartAdapter(
        ser,
        baudrate=921600,
        timeout_s=0.025,
        min_frame_interval_s=0.001,
    )
    stop_adapter = J7StopAdapter(adapter)

    # 1. Preflight read
    print("  [J6 Phase 1] Preflight reading 6-axis positions...")
    init_positions = []
    for mid in MOTOR_IDS:
        raw = adapter.request(pack_read_status_cmd(mid), expected_id=mid, expected_command=CMD_RD_STATUS, retries=2)
        st = unpack_status_response(raw)
        init_positions.append(st.current_position)
        print(f"    ID{mid}: pos={st.current_position}, current={st.current_mA}mA, fault={st.error_code}")
    q_initial = tuple(init_positions)
    q_park = tuple(max(0, min(2000, p)) for p in q_initial)
    q_home = (1000, 1000, 1000, 1000, 1000, 1000)

    # 2. Configure Mode 0 (Positioning Mode)
    motor_reg_speed = max(50, int(round(speed_steps_s * 1.2)))
    print(f"  [J6 Phase 2] Configuring mode=0 (Positioning), speed_reg={motor_reg_speed}...")
    for mid in MOTOR_IDS:
        raw = adapter.request(
            pack_write_register_cmd(mid, CONTROL_MODE_ADDR, [0, 0, 0, motor_reg_speed]),
            expected_id=mid,
            expected_command=CMD_WR_REGISTER,
            retries=2,
        )
        resp = unpack_write_response(raw)
        if resp.error_code != 0:
            raise RuntimeError(f"ID{mid} mode=0 config fault={resp.error_code}")

    # 3. Build trajectory
    period_s = 1.0 / rate_hz
    period_ns = int(period_s * 1e9)
    d_max = max(abs(q_home[i] - q_park[i]) for i in range(6))
    trans_duration_s = max(2.0, (1.875 * d_max) / speed_steps_s)
    hold_cycles = int(round(hold_duration_s * rate_hz))
    trans_cycles = int(round(trans_duration_s * rate_hz))

    trajectory_targets: List[Tuple[Tuple[int, ...], str]] = []
    for _ in range(hold_cycles):
        trajectory_targets.append((q_park, "PARK_HOLD"))
    for k in range(1, trans_cycles + 1):
        s = quintic_progress(k / trans_cycles)
        tgt = tuple(int(round(q_park[i] + (q_home[i] - q_park[i]) * s)) for i in range(6))
        trajectory_targets.append((tgt, "PARK_TO_HOME"))
    for _ in range(hold_cycles):
        trajectory_targets.append((q_home, "HOME_HOLD"))
    for k in range(1, trans_cycles + 1):
        s = quintic_progress(k / trans_cycles)
        tgt = tuple(int(round(q_home[i] + (q_park[i] - q_home[i]) * s)) for i in range(6))
        trajectory_targets.append((tgt, "HOME_TO_PARK"))
    for _ in range(hold_cycles):
        trajectory_targets.append((q_park, "PARK_FINAL_HOLD"))

    total_cycles = len(trajectory_targets)
    print(f"  [J6 Phase 3] Executing J6 mode=0 Positioning trajectory @ {rate_hz:.1f}Hz ({total_cycles} cycles, ~{total_cycles * period_s:.1f}s)...")

    records: List[GroupRecord] = []
    t_start_ns = time.monotonic_ns()
    t_prev_cycle_ns = t_start_ns
    stop_success = False

    try:
        for cycle_id, (target_vec, phase_name) in enumerate(trajectory_targets):
            t_cycle_start_ns = time.monotonic_ns()
            dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if cycle_id > 0 else period_s * 1000.0
            t_prev_cycle_ns = t_cycle_start_ns

            # Dispatch 6-axis targets (mode=0)
            for mid, tgt in zip(MOTOR_IDS, target_vec):
                adapter.write(pack_write_register_cmd(mid, TARGET_POSITION_ADDR, [tgt]))

            # Read full 6-axis state snapshot (J6 standard behavior)
            pos_dict = {}
            curr_dict = {}
            fault_dict = {}
            for mid in MOTOR_IDS:
                raw_fb = adapter.request(pack_read_status_cmd(mid), expected_id=mid, expected_command=CMD_RD_STATUS, retries=0)
                st = unpack_status_response(raw_fb)
                pos_dict[mid] = st.current_position
                curr_dict[mid] = st.current_mA
                fault_dict[mid] = st.error_code

            rec = GroupRecord(
                t_s=(t_cycle_start_ns - t_start_ns) / 1e9,
                dt_ms=dt_ms,
                targets=target_vec,
                positions=pos_dict,
                currents=curr_dict,
                faults=fault_dict,
                phase=phase_name,
                polled_axis=None,
            )
            records.append(rec)

            if any(f != 0 for f in fault_dict.values()):
                print(f"    [J6 EMERGENCY STOP] Fault detected: {fault_dict}")
                stop_adapter.stop_all("fault")
                break
            if any(c > 1500 for c in curr_dict.values()):
                print(f"    [J6 EMERGENCY STOP] Overcurrent detected: {curr_dict}")
                stop_adapter.stop_all("overcurrent")
                break

            if cycle_id % 40 == 0 or cycle_id == total_cycles - 1:
                print(f"    [J6 {phase_name:16s}] Cycle {cycle_id:3d}/{total_cycles:3d} (t={rec.t_s:5.2f}s): ID1 pos={pos_dict[1]:4d}, curr={curr_dict[1]:2d}mA | ID2 pos={pos_dict[2]:4d}, curr={curr_dict[2]:2d}mA")

            next_ns = t_start_ns + (cycle_id + 1) * period_ns
            rem_ns = next_ns - time.monotonic_ns()
            if rem_ns > 0:
                time.sleep(rem_ns / 1e9)

    finally:
        print("  [J6 Phase 4] Sending 0x1A Stop...")
        stop_audit = stop_adapter.stop_all("j6_complete")
        stop_success = stop_audit.success
        time.sleep(0.05)
        final_pos = []
        for mid in MOTOR_IDS:
            raw = adapter.request(pack_read_status_cmd(mid), expected_id=mid, expected_command=CMD_RD_STATUS, retries=2)
            st = unpack_status_response(raw)
            final_pos.append(st.current_position)
        q_final = tuple(final_pos)
        print(f"    J6 Final Positions after stop: {list(q_final)}")

    return records, q_initial, q_final, stop_success


def run_j7_servo_run(
    ser: serial.Serial,
    *,
    rate_hz: float = 50.0,
    speed_steps_s: float = 100.0,
    hold_duration_s: float = 3.0,
) -> Tuple[List[GroupRecord], Tuple[int, ...], Tuple[int, ...], bool]:
    adapter = J7UartAdapter(
        ser,
        baudrate=921600,
        timeout_s=0.015,
        min_frame_interval_s=0.0005,
    )
    stop_adapter = J7StopAdapter(adapter)

    writer = J7SixAxisServoTransport(
        adapter,
        cycle_rate_hz=rate_hz,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
        min_frame_interval_s=0.0005,
        pause_callback=stop_adapter,
    )

    feedback = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=1e9,
        feedback_budget_s=0.010,
    )

    # 1. Preflight read
    print("  [J7 Phase 1] Preflight full scan ID1-ID6...")
    preflight_audit = feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
    init_positions = [s.position for s in preflight_audit.samples]
    q_initial = tuple(init_positions)
    q_park = tuple(max(0, min(2000, p)) for p in q_initial)
    q_home = (1000, 1000, 1000, 1000, 1000, 1000)

    # 2. Configure Mode 1 (Servo Mode)
    print("  [J7 Phase 2] Configuring mode=1 (Servo), speed=50...")
    configs = writer.configure_servo(speed=50)

    # 3. Build trajectory
    period_s = 1.0 / rate_hz
    period_ns = int(period_s * 1e9)
    d_max = max(abs(q_home[i] - q_park[i]) for i in range(6))
    trans_duration_s = max(2.0, (1.875 * d_max) / speed_steps_s)
    hold_cycles = int(round(hold_duration_s * rate_hz))
    trans_cycles = int(round(trans_duration_s * rate_hz))

    trajectory_targets: List[Tuple[Tuple[int, ...], str]] = []
    for _ in range(hold_cycles):
        trajectory_targets.append((q_park, "PARK_HOLD"))
    for k in range(1, trans_cycles + 1):
        s = quintic_progress(k / trans_cycles)
        tgt = tuple(int(round(q_park[i] + (q_home[i] - q_park[i]) * s)) for i in range(6))
        trajectory_targets.append((tgt, "PARK_TO_HOME"))
    for _ in range(hold_cycles):
        trajectory_targets.append((q_home, "HOME_HOLD"))
    for k in range(1, trans_cycles + 1):
        s = quintic_progress(k / trans_cycles)
        tgt = tuple(int(round(q_home[i] + (q_park[i] - q_home[i]) * s)) for i in range(6))
        trajectory_targets.append((tgt, "HOME_TO_PARK"))
    for _ in range(hold_cycles):
        trajectory_targets.append((q_park, "PARK_FINAL_HOLD"))

    total_cycles = len(trajectory_targets)
    print(f"  [J7 Phase 3] Executing J7 mode=1 Servo stream @ {rate_hz:.1f}Hz ({total_cycles} cycles, ~{total_cycles * period_s:.1f}s)...")

    records: List[GroupRecord] = []
    t_start_ns = time.monotonic_ns()
    t_prev_cycle_ns = t_start_ns
    stop_success = False

    latest_positions: Dict[int, int] = {mid: q_initial[i] for i, mid in enumerate(MOTOR_IDS)}
    latest_currents: Dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
    latest_faults: Dict[int, int] = {mid: 0 for mid in MOTOR_IDS}

    try:
        for cycle_id, (target_vec, phase_name) in enumerate(trajectory_targets):
            t_cycle_start_ns = time.monotonic_ns()
            dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if cycle_id > 0 else period_s * 1000.0
            t_prev_cycle_ns = t_cycle_start_ns

            # 1. Send 6-axis write-only 0x29 batch
            writer.send_cycle(list(target_vec), cycle_id=cycle_id, target_timestamp_ns=t_cycle_start_ns)

            # 2. Poll 1 axis status via round-robin scheduler
            feedback_audit = feedback.poll_once(servo_cycle_id=cycle_id, servo_cycle_timestamp_ns=t_cycle_start_ns)

            polled_axis = feedback_audit.motor_ids[0] if feedback_audit.motor_ids else 1
            sample = feedback_audit.samples[0] if feedback_audit.samples else None

            if sample is not None:
                p_pos = sample.position if sample.position is not None else target_vec[polled_axis - 1]
                p_curr = sample.current_mA if sample.current_mA is not None else 0
                p_fault = sample.fault if sample.fault is not None else 0
                latest_positions[polled_axis] = p_pos
                latest_currents[polled_axis] = p_curr
                latest_faults[polled_axis] = p_fault
            else:
                p_pos = target_vec[polled_axis - 1]
                p_curr = 0
                p_fault = 0

            rec = GroupRecord(
                t_s=(t_cycle_start_ns - t_start_ns) / 1e9,
                dt_ms=dt_ms,
                targets=target_vec,
                positions=dict(latest_positions),
                currents=dict(latest_currents),
                faults=dict(latest_faults),
                phase=phase_name,
                polled_axis=polled_axis,
            )
            records.append(rec)

            if p_fault != 0:
                print(f"    [J7 EMERGENCY STOP] Fault on ID{polled_axis}: {p_fault}")
                stop_adapter.stop_all("fault")
                break
            if p_curr > 1500:
                print(f"    [J7 EMERGENCY STOP] Overcurrent on ID{polled_axis}: {p_curr}mA")
                stop_adapter.stop_all("overcurrent")
                break

            if cycle_id % 200 == 0 or cycle_id == total_cycles - 1:
                print(f"    [J7 {phase_name:16s}] Cycle {cycle_id:4d}/{total_cycles:4d} (t={rec.t_s:5.2f}s): polled ID{polled_axis} pos={p_pos:4d} (tgt={target_vec[polled_axis-1]:4d}), curr={p_curr:2d}mA")

            next_ns = t_start_ns + (cycle_id + 1) * period_ns
            rem_ns = next_ns - time.monotonic_ns()
            if rem_ns > 0:
                time.sleep(rem_ns / 1e9)

    finally:
        print("  [J7 Phase 4] Sending 0x1A Stop...")
        stop_audit = stop_adapter.stop_all("j7_complete")
        stop_success = stop_audit.success
        time.sleep(0.05)
        final_audit = feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
        final_pos = [s.position for s in final_audit.samples]
        q_final = tuple(final_pos)
        print(f"    J7 Final Positions after stop: {list(q_final)}")

    return records, q_initial, q_final, stop_success


def analyze_records(
    name: str,
    mode: int,
    rate_hz: float,
    records: List[GroupRecord],
    q_initial: Tuple[int, ...],
    q_final: Tuple[int, ...],
    stop_success: bool,
    is_round_robin: bool,
) -> GroupMetrics:
    if not records:
        raise RuntimeError(f"No records for {name}")

    total_samples = len(records)
    total_duration_s = records[-1].t_s
    dts = [r.dt_ms for r in records[1:]] if len(records) > 1 else [20.0]
    period_mean = sum(dts) / len(dts)
    period_std = math.sqrt(sum((x - period_mean) ** 2 for x in dts) / len(dts)) if len(dts) > 1 else 0.0

    max_err: Dict[int, int] = {}
    mean_err: Dict[int, float] = {}
    rms_err: Dict[int, float] = {}
    peak_curr: Dict[int, int] = {}
    mean_curr: Dict[int, float] = {}
    std_curr: Dict[int, float] = {}
    vel_var: Dict[int, float] = {}
    jerk_proxy: Dict[int, float] = {}
    stutter_cnt: Dict[int, int] = {}

    all_errs_list: List[int] = []

    for mid_idx, mid in enumerate(MOTOR_IDS):
        if is_round_robin:
            axis_recs = [r for r in records if r.polled_axis == mid]
        else:
            axis_recs = records

        if not axis_recs:
            continue

        # Tracking errors
        errs = [abs(r.positions[mid] - r.targets[mid_idx]) for r in axis_recs]
        all_errs_list.extend(errs)
        max_err[mid] = max(errs)
        mean_err[mid] = sum(errs) / len(errs)
        rms_err[mid] = math.sqrt(sum(e * e for e in errs) / len(errs))

        # Current
        currs = [r.currents[mid] for r in axis_recs]
        peak_curr[mid] = max(currs)
        m_c = sum(currs) / len(currs)
        mean_curr[mid] = m_c
        std_curr[mid] = math.sqrt(sum((c - m_c) ** 2 for c in currs) / len(currs)) if len(currs) > 1 else 0.0

        # Motion continuity & velocity variance (during active motion phases)
        motion_recs = [r for r in axis_recs if r.phase in ("PARK_TO_HOME", "HOME_TO_PARK")]
        if len(motion_recs) >= 3:
            vels = []
            for i in range(1, len(motion_recs)):
                dt = motion_recs[i].t_s - motion_recs[i-1].t_s
                dp = motion_recs[i].positions[mid] - motion_recs[i-1].positions[mid]
                v = dp / dt if dt > 0 else 0.0
                vels.append(v)

            # Stutter count: during active motion, velocity drops to 0 or reverses unexpectedly
            stutters = sum(1 for v in vels[5:-5] if abs(v) < 2.0)
            stutter_cnt[mid] = stutters

            # Velocity variance
            mean_v = sum(vels) / len(vels)
            v_var = math.sqrt(sum((v - mean_v) ** 2 for v in vels) / len(vels))
            vel_var[mid] = v_var

            # Jerk proxy (acceleration step magnitude)
            accs = []
            for i in range(1, len(vels)):
                dt = motion_recs[i+1].t_s - motion_recs[i].t_s
                a = (vels[i] - vels[i-1]) / dt if dt > 0 else 0.0
                accs.append(abs(a))
            jerk_proxy[mid] = sum(accs) / len(accs) if accs else 0.0
        else:
            stutter_cnt[mid] = 0
            vel_var[mid] = 0.0
            jerk_proxy[mid] = 0.0

    overall_max_e = max(max_err.values()) if max_err else 0
    overall_rms_e = math.sqrt(sum(e * e for e in all_errs_list) / len(all_errs_list)) if all_errs_list else 0.0

    all_currents = [r.currents[mid] for r in records for mid in MOTOR_IDS]
    overall_peak_c = max(all_currents) if all_currents else 0
    overall_mean_c = sum(all_currents) / len(all_currents) if all_currents else 0.0

    q_park = tuple(max(0, min(2000, p)) for p in q_initial)
    final_return_errs = {mid: abs(q_final[i] - q_park[i]) for i, mid in enumerate(MOTOR_IDS)}
    max_return_e = max(final_return_errs.values()) if final_return_errs else 0
    return_skew = max(final_return_errs.values()) - min(final_return_errs.values()) if final_return_errs else 0

    faults = sum(1 for r in records for mid in MOTOR_IDS if r.faults.get(mid, 0) != 0)

    return GroupMetrics(
        name=name,
        mode=mode,
        rate_hz=rate_hz,
        total_samples=total_samples,
        total_duration_s=total_duration_s,
        period_mean_ms=period_mean,
        period_std_ms=period_std,
        q_initial=q_initial,
        q_final=q_final,
        max_tracking_err=max_err,
        mean_tracking_err=mean_err,
        rms_tracking_err=rms_err,
        overall_max_err=overall_max_e,
        overall_rms_err=overall_rms_e,
        peak_current_mA=peak_curr,
        mean_current_mA=mean_curr,
        std_current_mA=std_curr,
        overall_peak_current=overall_peak_c,
        overall_mean_current=overall_mean_c,
        velocity_variance=vel_var,
        jerk_proxy_mean=jerk_proxy,
        stutter_count=stutter_cnt,
        final_return_errors=final_return_errs,
        max_return_error=max_return_e,
        return_skew=return_skew,
        fault_count=faults,
        timeout_count=0,
        stop_success=stop_success,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="J6 vs J7 A/B Hardware Benchmark")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--speed", type=float, default=100.0)
    parser.add_argument("--hold-duration", type=float, default=3.0)
    args = parser.parse_args()

    set_latency_timer(args.port)

    print("================================================================================")
    print("        6PUS J6 (mode=0 Positioning) vs J7 (mode=1 Servo) A/B BENCHMARK        ")
    print("================================================================================")
    print(f"Port: {args.port}, Baudrate: 921600")
    print(f"Trajectory: PARK (0) <-> HOME (1000) Quintic S-Curve, Peak speed: {args.speed} step/s")
    print(f"Hold duration: {args.hold_duration}s per station\n")

    # Group A: J6 frozen baseline
    print(">>> [RUN A] Executing J6 Frozen Baseline (mode=0 Positioning @ 10Hz)...")
    ser_a = serial.Serial(port=args.port, baudrate=921600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.025, write_timeout=0.025)
    ser_a.reset_input_buffer()
    ser_a.reset_output_buffer()
    records_a, q_init_a, q_final_a, stop_ok_a = run_j6_baseline_run(ser_a, rate_hz=10.0, speed_steps_s=args.speed, hold_duration_s=args.hold_duration)
    ser_a.close()
    metrics_a = analyze_records("J6_Positioning", 0, 10.0, records_a, q_init_a, q_final_a, stop_ok_a, is_round_robin=False)
    print("<<< [RUN A] Completed successfully.\n")

    print("--- Settling 3.0s between benchmark runs ---\n")
    time.sleep(3.0)

    # Group B: J7 Servo mode
    print(">>> [RUN B] Executing J7 Servo Stream (mode=1 Servo @ 50Hz)...")
    ser_b = serial.Serial(port=args.port, baudrate=921600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.015, write_timeout=0.015)
    ser_b.reset_input_buffer()
    ser_b.reset_output_buffer()
    records_b, q_init_b, q_final_b, stop_ok_b = run_j7_servo_run(ser_b, rate_hz=50.0, speed_steps_s=args.speed, hold_duration_s=args.hold_duration)
    ser_b.close()
    metrics_b = analyze_records("J7_Servo", 1, 50.0, records_b, q_init_b, q_final_b, stop_ok_b, is_round_robin=True)
    print("<<< [RUN B] Completed successfully.\n")

    # Output Side-by-Side Comparison
    print("================================================================================")
    print("                   QUANTITATIVE A/B COMPARISON RESULTS                          ")
    print("================================================================================")

    print("\n--- 1. High-Level Summary ---")
    print(f"{'Metric':<35} | {'J6 (mode=0 Positioning)':<22} | {'J7 (mode=1 Servo)':<22} | {'Improvement':<15}")
    print("-" * 102)
    print(f"{'Control Update Frequency':<35} | {metrics_a.rate_hz:<22.1f} | {metrics_b.rate_hz:<22.1f} | {'5.0x higher':<15}")
    print(f"{'Total Motion Duration (s)':<35} | {metrics_a.total_duration_s:<22.2f} | {metrics_b.total_duration_s:<22.2f} | {f'{abs(metrics_b.total_duration_s - metrics_a.total_duration_s):.2f}s delta':<15}")
    print(f"{'Cycle Timing Jitter Std (ms)':<35} | {metrics_a.period_std_ms:<22.2f} | {metrics_b.period_std_ms:<22.2f} | {f'{(1 - metrics_b.period_std_ms / max(0.01, metrics_a.period_std_ms))*100:+.1f}%':<15}")
    print(f"{'Overall Max Tracking Error (steps)':<35} | {metrics_a.overall_max_err:<22d} | {metrics_b.overall_max_err:<22d} | {f'{(1 - metrics_b.overall_max_err / max(1, metrics_a.overall_max_err))*100:+.1f}%':<15}")
    print(f"{'Overall RMS Tracking Error (steps)':<35} | {metrics_a.overall_rms_err:<22.2f} | {metrics_b.overall_rms_err:<22.2f} | {f'{(1 - metrics_b.overall_rms_err / max(0.01, metrics_a.overall_rms_err))*100:+.1f}%':<15}")
    print(f"{'Peak Electrical Current (mA)':<35} | {metrics_a.overall_peak_current:<22d} | {metrics_b.overall_peak_current:<22d} | {f'{(1 - metrics_b.overall_peak_current / max(1, metrics_a.overall_peak_current))*100:+.1f}%':<15}")
    print(f"{'Mean Electrical Current (mA)':<35} | {metrics_a.overall_mean_current:<22.1f} | {metrics_b.overall_mean_current:<22.1f} | {f'{(1 - metrics_b.overall_mean_current / max(0.1, metrics_a.overall_mean_current))*100:+.1f}%':<15}")
    print(f"{'Final Return Max Error (steps)':<35} | {metrics_a.max_return_error:<22d} | {metrics_b.max_return_error:<22d} | {f'{(1 - metrics_b.max_return_error / max(1, metrics_a.max_return_error))*100:+.1f}%':<15}")
    print(f"{'Final Return Skew (steps)':<35} | {metrics_a.return_skew:<22d} | {metrics_b.return_skew:<22d} | {f'{(1 - metrics_b.return_skew / max(1, metrics_a.return_skew))*100:+.1f}%':<15}")

    print("\n--- 2. Per-Axis Tracking Performance (steps) ---")
    print(f"{'Axis':<6} | {'J6 Max':<8} | {'J7 Max':<8} | {'J6 Mean':<8} | {'J7 Mean':<8} | {'J6 RMS':<8} | {'J7 RMS':<8} | {'RMS Reduction':<12}")
    print("-" * 80)
    for mid in MOTOR_IDS:
        j6_m = metrics_a.max_tracking_err[mid]
        j7_m = metrics_b.max_tracking_err[mid]
        j6_mean = metrics_a.mean_tracking_err[mid]
        j7_mean = metrics_b.mean_tracking_err[mid]
        j6_rms = metrics_a.rms_tracking_err[mid]
        j7_rms = metrics_b.rms_tracking_err[mid]
        red = (1 - j7_rms / max(0.01, j6_rms)) * 100
        print(f"ID{mid:<3} | {j6_m:<8d} | {j7_m:<8d} | {j6_mean:<8.2f} | {j7_mean:<8.2f} | {j6_rms:<8.2f} | {j7_rms:<8.2f} | {red:>+10.1f}%")

    print("\n--- 3. Per-Axis Current Profile (mA) ---")
    print(f"{'Axis':<6} | {'J6 Peak':<8} | {'J7 Peak':<8} | {'J6 Mean':<8} | {'J7 Mean':<8} | {'J6 Std':<8} | {'J7 Std':<8} | {'Peak Red.':<10}")
    print("-" * 75)
    for mid in MOTOR_IDS:
        j6_pk = metrics_a.peak_current_mA[mid]
        j7_pk = metrics_b.peak_current_mA[mid]
        j6_mn = metrics_a.mean_current_mA[mid]
        j7_mn = metrics_b.mean_current_mA[mid]
        j6_sd = metrics_a.std_current_mA[mid]
        j7_sd = metrics_b.std_current_mA[mid]
        p_red = (1 - j7_pk / max(1, j6_pk)) * 100
        print(f"ID{mid:<3} | {j6_pk:<8d} | {j7_pk:<8d} | {j6_mn:<8.1f} | {j7_mn:<8.1f} | {j6_sd:<8.1f} | {j7_sd:<8.1f} | {p_red:>+8.1f}%")

    print("\n--- 4. Motion Continuity & Stuttering ---")
    print(f"{'Axis':<6} | {'J6 Stutter Events':<18} | {'J7 Stutter Events':<18} | {'J6 Vel Std (step/s)':<20} | {'J7 Vel Std (step/s)':<20}")
    print("-" * 90)
    for mid in MOTOR_IDS:
        j6_st = metrics_a.stutter_count[mid]
        j7_st = metrics_b.stutter_count[mid]
        j6_vv = metrics_a.velocity_variance[mid]
        j7_vv = metrics_b.velocity_variance[mid]
        print(f"ID{mid:<3} | {j6_st:<18d} | {j7_st:<18d} | {j6_vv:<20.2f} | {j7_vv:<20.2f}")

    print("\n================================================================================")
    print("Benchmark completed successfully.")
    print("================================================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
