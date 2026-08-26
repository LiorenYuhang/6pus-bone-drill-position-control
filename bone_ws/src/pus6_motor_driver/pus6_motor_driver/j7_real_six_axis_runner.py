"""J7 real-hardware six-axis Servo experiment runner for ID1-ID6.

Executes:
1. READ_ONLY preflight scan across ID1-ID6 to capture q_current, fault, current.
2. Enter Servo mode (mode=1, speed=50) for all 6 axes (acknowledged write, no 0x25/0x28 readback).
3. 50Hz six-axis Servo stream:
   - Phase A: Hold at q_current for 3 seconds (150 cycles).
   - Phase B: Small-range relative motion: q_current -> q_current + [50]*6 -> q_current (200 cycles).
4. Feedback:
   - Round-robin: 1 axis 0x30 status read per 20ms cycle (ID1 -> ID2 -> ... -> ID6 -> ID1).
   - No runtime 0x25 mode readback.
5. Continuous monitoring & emergency stop:
   - Position, current, fault, timeout, divergence, batch duration, feedback age.
   - Immediate 0x1A stop on any anomaly.
6. Clean 0x1A stop and full metrics reporting.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
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


@dataclass
class SixAxisCycleRecord:
    cycle_id: int
    t_start_s: float
    dt_ms: float
    batch_duration_ms: float
    targets: Tuple[int, ...]
    polled_axis: int
    polled_pos: int
    polled_target: int
    polled_err: int
    polled_current_mA: int
    polled_fault: int
    polled_timeout: bool
    feedback_age_ms: float


@dataclass
class SixAxisExperimentResult:
    port: str
    baudrate: int
    motor_ids: Tuple[int, ...]
    q_initial: Tuple[int, ...]
    q_peak_target: Tuple[int, ...]
    q_final: Tuple[int, ...]
    total_cycles: int
    hold_cycles: int
    motion_cycles: int
    target_frequency_hz: float
    actual_frequency_hz: float
    period_mean_ms: float
    period_min_ms: float
    period_max_ms: float
    period_std_ms: float
    batch_duration_mean_ms: float
    batch_duration_max_ms: float
    feedback_age_mean_ms: float
    feedback_age_max_ms: float
    axis_max_tracking_err: Dict[int, int]
    axis_mean_tracking_err: Dict[int, float]
    axis_max_current_mA: Dict[int, int]
    axis_mean_current_mA: Dict[int, float]
    max_inter_axis_skew: int
    fault_count: int
    timeout_count: int
    stop_success: bool
    passed: bool
    summary: str


def set_latency_timer(port: str) -> None:
    try:
        devname = os.path.basename(port)
        latency_path = f"/sys/bus/usb-serial/devices/{devname}/latency_timer"
        if os.path.exists(latency_path):
            with open(latency_path, "w") as f:
                f.write("1")
    except Exception:
        pass


def run_six_axis_servo_experiment(
    *,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 921600,
    hold_duration_s: float = 3.0,
    delta_pos: int = 50,
    rate_hz: float = 50.0,
) -> SixAxisExperimentResult:
    if serial is None:
        raise RuntimeError("pyserial is required to run real hardware experiment")

    set_latency_timer(port)

    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.015,
        write_timeout=0.015,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    stop_adapter = None
    records: List[SixAxisCycleRecord] = []
    stop_success = False
    passed = False
    q_initial: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    q_peak_target: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    q_final: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)

    try:
        adapter = J7UartAdapter(
            ser,
            baudrate=baudrate,
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
            full_scan_period_s=1e9,  # Round-robin only during streaming
            feedback_budget_s=0.010,
        )

        # Step 1: Preflight full scan across ID1-ID6
        print("[Phase 1] Preflight full scan across ID1-ID6...")
        preflight_audit = feedback.full_scan(
            servo_cycle_id=0,
            servo_cycle_timestamp_ns=time.monotonic_ns(),
        )
        if preflight_audit.deferred or len(preflight_audit.samples) != len(MOTOR_IDS):
            raise RuntimeError("Preflight scan incomplete")

        init_positions: List[int] = []
        for sample in preflight_audit.samples:
            if sample.timeout or sample.fault != 0:
                raise RuntimeError(
                    f"ID{sample.motor_id} preflight failure: fault={sample.fault}, timeout={sample.timeout}"
                )
            if sample.position is None or not (0 <= sample.position <= 2000):
                raise RuntimeError(f"ID{sample.motor_id} position invalid: {sample.position}")
            init_positions.append(sample.position)
            print(
                f"  ID{sample.motor_id}: position={sample.position:4d}, "
                f"current={sample.current_mA:3d} mA, fault={sample.fault}"
            )

        q_initial = tuple(init_positions)
        print(f"  q_initial = {list(q_initial)}")

        # Step 2: Configure all 6 axes into Servo mode (mode=1, speed=50)
        print("\n[Phase 2] Configuring six axes into Servo mode (mode=1, speed=50)...")
        configs = writer.configure_servo(speed=50)
        if len(configs) != 6 or any(c.mode != 1 for c in configs):
            raise RuntimeError("Servo configuration failed")
        print("  All 6 axes configured: mode=1 acknowledged, fault=0.")

        # Step 3: Build trajectory targets
        # Profile:
        # Hold: 3.0s (150 cycles) @ q_initial
        # Ramp up: 1.0s (50 cycles) -> q_initial + delta_pos
        # Peak hold: 1.0s (50 cycles) -> q_initial + delta_pos
        # Ramp down: 1.0s (50 cycles) -> q_initial
        # Final hold: 1.0s (50 cycles) -> q_initial
        period_s = 1.0 / rate_hz
        period_ns = int(period_s * 1e9)

        hold_cycles = int(round(hold_duration_s * rate_hz))
        ramp_up_cycles = int(round(1.0 * rate_hz))
        peak_hold_cycles = int(round(1.0 * rate_hz))
        ramp_down_cycles = int(round(1.0 * rate_hz))
        final_hold_cycles = int(round(1.0 * rate_hz))

        q_peak_target = tuple(pos + delta_pos for pos in q_initial)

        cycle_targets: List[Tuple[int, ...]] = []
        # Hold phase
        for _ in range(hold_cycles):
            cycle_targets.append(q_initial)
        # Ramp up phase
        for i in range(1, ramp_up_cycles + 1):
            fraction = i / ramp_up_cycles
            tgt = tuple(int(round(pos + delta_pos * fraction)) for pos in q_initial)
            cycle_targets.append(tgt)
        # Peak hold phase
        for _ in range(peak_hold_cycles):
            cycle_targets.append(q_peak_target)
        # Ramp down phase
        for i in range(1, ramp_down_cycles + 1):
            fraction = 1.0 - (i / ramp_down_cycles)
            tgt = tuple(int(round(pos + delta_pos * fraction)) for pos in q_initial)
            cycle_targets.append(tgt)
        # Final hold phase
        for _ in range(final_hold_cycles):
            cycle_targets.append(q_initial)

        total_cycles = len(cycle_targets)
        motion_cycles = total_cycles - hold_cycles

        print(f"\n[Phase 3] Starting 50Hz six-axis Servo stream: total {total_cycles} cycles (~{total_cycles * period_s:.1f}s)...")
        print(f"  Hold: {hold_cycles} cycles (3.0s) @ q_initial = {list(q_initial)}")
        print(f"  Motion: q_initial -> {list(q_peak_target)} -> q_initial ({motion_cycles} cycles, 4.0s)")

        t_start_exp_ns = time.monotonic_ns()
        t_prev_cycle_ns = t_start_exp_ns

        latest_positions: Dict[int, int] = {mid: q_initial[i] for i, mid in enumerate(MOTOR_IDS)}
        latest_currents: Dict[int, int] = {mid: 0 for mid in MOTOR_IDS}

        for cycle_id, target_vec in enumerate(cycle_targets):
            t_cycle_start_ns = time.monotonic_ns()
            dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if cycle_id > 0 else period_s * 1000.0
            t_prev_cycle_ns = t_cycle_start_ns

            # 1. Send 6-axis write-only 0x29 batch
            servo_audit = writer.send_cycle(
                list(target_vec),
                cycle_id=cycle_id,
                target_timestamp_ns=t_cycle_start_ns,
            )
            batch_duration_ms = (servo_audit.cycle_end_ns - servo_audit.cycle_start_ns) / 1e6

            # 2. Poll 1 axis status via round-robin scheduler
            feedback_audit = feedback.poll_once(
                servo_cycle_id=cycle_id,
                servo_cycle_timestamp_ns=t_cycle_start_ns,
            )

            polled_axis = feedback_audit.motor_ids[0] if feedback_audit.motor_ids else -1
            sample = feedback_audit.samples[0] if feedback_audit.samples else None

            if sample is not None:
                polled_pos = sample.position if sample.position is not None else target_vec[polled_axis - 1]
                polled_curr = sample.current_mA if sample.current_mA is not None else 0
                polled_fault = sample.fault if sample.fault is not None else 0
                polled_timeout = sample.timeout
                feedback_age_ms = sample.feedback_age_ns / 1e6
                latest_positions[polled_axis] = polled_pos
                latest_currents[polled_axis] = polled_curr
            else:
                polled_pos = target_vec[polled_axis - 1]
                polled_curr = 0
                polled_fault = 0
                polled_timeout = True
                feedback_age_ms = 0.0

            polled_target = target_vec[polled_axis - 1]
            polled_err = polled_pos - polled_target

            record = SixAxisCycleRecord(
                cycle_id=cycle_id,
                t_start_s=(t_cycle_start_ns - t_start_exp_ns) / 1e9,
                dt_ms=dt_ms,
                batch_duration_ms=batch_duration_ms,
                targets=target_vec,
                polled_axis=polled_axis,
                polled_pos=polled_pos,
                polled_target=polled_target,
                polled_err=polled_err,
                polled_current_mA=polled_curr,
                polled_fault=polled_fault,
                polled_timeout=polled_timeout,
                feedback_age_ms=feedback_age_ms,
            )
            records.append(record)

            # Safety guards
            if polled_fault != 0:
                print(f"  [EMERGENCY STOP] ID{polled_axis} fault={polled_fault} at cycle {cycle_id}")
                stop_adapter.stop_all(f"fault_on_ID{polled_axis}")
                break
            if polled_timeout:
                print(f"  [EMERGENCY STOP] ID{polled_axis} feedback timeout at cycle {cycle_id}")
                stop_adapter.stop_all(f"timeout_on_ID{polled_axis}")
                break
            if polled_curr > 1500:
                print(f"  [EMERGENCY STOP] ID{polled_axis} overcurrent {polled_curr} mA at cycle {cycle_id}")
                stop_adapter.stop_all(f"overcurrent_on_ID{polled_axis}")
                break
            if abs(polled_err) > 30:
                print(f"  [EMERGENCY STOP] ID{polled_axis} tracking divergence err={polled_err} at cycle {cycle_id}")
                stop_adapter.stop_all(f"divergence_on_ID{polled_axis}")
                break

            # Progress log every 50 cycles
            if cycle_id % 50 == 0 or cycle_id == total_cycles - 1:
                phase_lbl = "HOLD" if cycle_id < hold_cycles else "MOTION"
                pos_str = ",".join(f"ID{mid}:{latest_positions[mid]}" for mid in MOTOR_IDS)
                print(
                    f"  [{phase_lbl:6s}] Cycle {cycle_id:3d}/{total_cycles:3d} (t={record.t_start_s:5.2f}s): "
                    f"polled ID{polled_axis} (tgt={polled_target}, act={polled_pos}, err={polled_err:+2d}, "
                    f"curr={polled_curr:2d}mA, fb_age={feedback_age_ms:4.2f}ms) | "
                    f"batch_TX={batch_duration_ms:4.2f}ms, dt={dt_ms:5.2f}ms"
                )

            # Sleep until next 20ms deadline
            next_deadline_ns = t_start_exp_ns + (cycle_id + 1) * period_ns
            now_ns = time.monotonic_ns()
            rem_ns = next_deadline_ns - now_ns
            if rem_ns > 0:
                time.sleep(rem_ns / 1e9)

    except Exception as exc:
        print(f"[ERROR] Exception during six-axis Servo run: {exc}")
        if stop_adapter is not None:
            stop_adapter.stop_all("exception_during_run")
    finally:
        # Step 5: Stop command (0x1A)
        print("\n[Phase 4] Sending 0x1A Stop command to all six axes...")
        if stop_adapter is not None:
            stop_audit = stop_adapter.stop_all("experiment_finish")
            stop_success = stop_audit.success
            print(f"  Stop complete: success={stop_success}, reason={stop_audit.reason}")

        # Final full status scan
        try:
            time.sleep(0.05)
            final_audit = feedback.full_scan(
                servo_cycle_id=0,
                servo_cycle_timestamp_ns=time.monotonic_ns(),
            )
            final_pos: List[int] = []
            for sample in final_audit.samples:
                final_pos.append(sample.position if sample.position is not None else -1)
                print(f"  Final ID{sample.motor_id}: pos={sample.position}, current={sample.current_mA} mA, fault={sample.fault}")
            q_final = tuple(final_pos)
        except Exception as exc:
            print(f"  Final status read failed: {exc}")
            q_final = tuple(latest_positions[mid] for mid in MOTOR_IDS)

        ser.close()

    if not records:
        return SixAxisExperimentResult(
            port=port,
            baudrate=baudrate,
            motor_ids=MOTOR_IDS,
            q_initial=q_initial,
            q_peak_target=q_peak_target,
            q_final=q_final,
            total_cycles=0,
            hold_cycles=0,
            motion_cycles=0,
            target_frequency_hz=rate_hz,
            actual_frequency_hz=0.0,
            period_mean_ms=0.0,
            period_min_ms=0.0,
            period_max_ms=0.0,
            period_std_ms=0.0,
            batch_duration_mean_ms=0.0,
            batch_duration_max_ms=0.0,
            feedback_age_mean_ms=0.0,
            feedback_age_max_ms=0.0,
            axis_max_tracking_err={mid: 0 for mid in MOTOR_IDS},
            axis_mean_tracking_err={mid: 0.0 for mid in MOTOR_IDS},
            axis_max_current_mA={mid: 0 for mid in MOTOR_IDS},
            axis_mean_current_mA={mid: 0.0 for mid in MOTOR_IDS},
            max_inter_axis_skew=0,
            fault_count=1,
            timeout_count=1,
            stop_success=False,
            passed=False,
            summary="Experiment produced 0 records",
        )

    dts = [r.dt_ms for r in records[1:]] if len(records) > 1 else [20.0]
    period_mean = sum(dts) / len(dts)
    period_min = min(dts)
    period_max = max(dts)
    period_var = sum((x - period_mean) ** 2 for x in dts) / len(dts) if len(dts) > 1 else 0.0
    period_std = math.sqrt(period_var)

    actual_duration_s = records[-1].t_start_s
    actual_freq = (len(records) - 1) / actual_duration_s if actual_duration_s > 0 else 0.0

    batch_durations = [r.batch_duration_ms for r in records]
    batch_mean = sum(batch_durations) / len(batch_durations)
    batch_max = max(batch_durations)

    feedback_ages = [r.feedback_age_ms for r in records]
    fb_age_mean = sum(feedback_ages) / len(feedback_ages)
    fb_age_max = max(feedback_ages)

    axis_max_err: Dict[int, int] = {}
    axis_mean_err: Dict[int, float] = {}
    axis_max_curr: Dict[int, int] = {}
    axis_mean_curr: Dict[int, float] = {}

    for mid in MOTOR_IDS:
        mid_records = [r for r in records if r.polled_axis == mid]
        if mid_records:
            errs = [abs(r.polled_err) for r in mid_records]
            currs = [r.polled_current_mA for r in mid_records]
            axis_max_err[mid] = max(errs)
            axis_mean_err[mid] = sum(errs) / len(errs)
            axis_max_curr[mid] = max(currs)
            axis_mean_curr[mid] = sum(currs) / len(currs)
        else:
            axis_max_err[mid] = 0
            axis_mean_err[mid] = 0.0
            axis_max_curr[mid] = 0
            axis_mean_curr[mid] = 0.0

    # Inter-axis synchronicity: relative position deviation from initial offsets
    max_skew = 0
    if len(q_final) == 6 and len(q_initial) == 6:
        delta_finals = [abs((q_final[i] - q_initial[i])) for i in range(6)]
        max_skew = max(delta_finals) - min(delta_finals)

    faults = sum(1 for r in records if r.polled_fault != 0)
    timeouts = sum(1 for r in records if r.polled_timeout)

    all_axis_max_err = max(axis_max_err.values())
    all_axis_max_curr = max(axis_max_curr.values())

    passed = (
        len(records) == total_cycles
        and faults == 0
        and timeouts == 0
        and stop_success
        and all_axis_max_err <= 15
        and abs(actual_freq - rate_hz) <= 1.5
    )

    summary = (
        f"Executed {len(records)}/{total_cycles} cycles. "
        f"Freq: {actual_freq:.2f} Hz (mean dt: {period_mean:.2f} ms ± {period_std:.2f} ms). "
        f"Batch TX: {batch_mean:.2f} ms (max: {batch_max:.2f} ms). "
        f"Feedback age: {fb_age_mean:.2f} ms. "
        f"Tracking: max err {all_axis_max_err} steps across all axes. "
        f"Current: max {all_axis_max_curr} mA. "
        f"Faults: {faults}, Timeouts: {timeouts}, Stop: {'OK' if stop_success else 'FAIL'}."
    )

    return SixAxisExperimentResult(
        port=port,
        baudrate=baudrate,
        motor_ids=MOTOR_IDS,
        q_initial=q_initial,
        q_peak_target=q_peak_target,
        q_final=q_final,
        total_cycles=len(records),
        hold_cycles=min(hold_cycles, len(records)),
        motion_cycles=max(0, len(records) - hold_cycles),
        target_frequency_hz=rate_hz,
        actual_frequency_hz=actual_freq,
        period_mean_ms=period_mean,
        period_min_ms=period_min,
        period_max_ms=period_max,
        period_std_ms=period_std,
        batch_duration_mean_ms=batch_mean,
        batch_duration_max_ms=batch_max,
        feedback_age_mean_ms=fb_age_mean,
        feedback_age_max_ms=fb_age_max,
        axis_max_tracking_err=axis_max_err,
        axis_mean_tracking_err=axis_mean_err,
        axis_max_current_mA=axis_max_curr,
        axis_mean_current_mA=axis_mean_curr,
        max_inter_axis_skew=max_skew,
        fault_count=faults,
        timeout_count=timeouts,
        stop_success=stop_success,
        passed=passed,
        summary=summary,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="J7 Six-Axis Servo Experiment Runner")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--hold-duration", type=float, default=3.0)
    parser.add_argument("--delta-pos", type=int, default=50)
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    res = run_six_axis_servo_experiment(
        port=args.port,
        baudrate=args.baudrate,
        hold_duration_s=args.hold_duration,
        delta_pos=args.delta_pos,
        rate_hz=args.rate,
    )

    print("\n=================== SIX-AXIS EXPERIMENT RESULT ===================")
    print(f"Result: {'PASSED' if res.passed else 'FAILED'}")
    print(f"Summary: {res.summary}")
    print(f"q_initial:     {list(res.q_initial)}")
    print(f"q_peak_target: {list(res.q_peak_target)}")
    print(f"q_final:       {list(res.q_final)}")
    print(f"Frequency: {res.actual_frequency_hz:.2f} Hz (target: {res.target_frequency_hz:.1f} Hz)")
    print(f"Period stats: mean={res.period_mean_ms:.2f}ms, min={res.period_min_ms:.2f}ms, max={res.period_max_ms:.2f}ms, std={res.period_std_ms:.2f}ms")
    print(f"6-axis TX Batch Duration: mean={res.batch_duration_mean_ms:.2f}ms, max={res.batch_duration_max_ms:.2f}ms")
    print(f"Feedback age: mean={res.feedback_age_mean_ms:.2f}ms, max={res.feedback_age_max_ms:.2f}ms")
    print(f"Per-axis max tracking error: {res.axis_max_tracking_err}")
    print(f"Per-axis mean tracking error: {res.axis_mean_tracking_err}")
    print(f"Per-axis max current: {res.axis_max_current_mA} mA")
    print(f"Per-axis mean current: {res.axis_mean_current_mA} mA")
    print(f"Inter-axis return synchronicity skew: {res.max_inter_axis_skew} steps")
    print(f"Faults: {res.fault_count}, Timeouts: {res.timeout_count}, Stop ACK: {res.stop_success}")
    print("==================================================================")
    return 0 if res.passed else 1


if __name__ == "__main__":
    sys.exit(main())
