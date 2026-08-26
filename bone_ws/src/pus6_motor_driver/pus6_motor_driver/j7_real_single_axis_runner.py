"""J7 real-hardware single-axis Servo experiment runner for ID1.

Executes:
1. Read initial status (q_current, current, fault).
2. Switch to Servo mode (mode=1) with acknowledged write, no 0x25/0x28 readback.
3. Immediately start 50Hz stream:
   - Hold at q_current for 3 seconds.
   - Smooth trajectory: q_current -> q_current + 50 -> q_current.
   - Runtime monitoring via 0x30 only: position, current, fault, timeout.
4. Stop with 0x1A pause command.
5. Print real execution metrics and pass/fail summary.
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
from typing import List

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    from .protocol import (
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        pack_read_status_cmd,
        pack_write_register_cmd,
        unpack_status_response,
        unpack_write_response,
    )
    from .safety import (
        CONTROL_MODE_ADDR,
        PAUSE_ADDR,
        TARGET_POSITION_ADDR,
    )
    from .j7_real_uart_adapter import J7UartAdapter
except ImportError:
    from pus6_motor_driver.protocol import (  # type: ignore
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        pack_read_status_cmd,
        pack_write_register_cmd,
        unpack_status_response,
        unpack_write_response,
    )
    from pus6_motor_driver.safety import (  # type: ignore
        CONTROL_MODE_ADDR,
        PAUSE_ADDR,
        TARGET_POSITION_ADDR,
    )
    from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter  # type: ignore


@dataclass
class CycleRecord:
    cycle_id: int
    t_start_s: float
    dt_ms: float
    target_pos: int
    actual_pos: int
    tracking_err: int
    current_mA: int
    fault: int
    timeout: bool


@dataclass
class ExperimentResult:
    port: str
    baudrate: int
    motor_id: int
    q_initial: int
    q_target_max: int
    q_final: int
    total_cycles: int
    hold_cycles: int
    motion_cycles: int
    target_frequency_hz: float
    actual_frequency_hz: float
    period_mean_ms: float
    period_min_ms: float
    period_max_ms: float
    period_std_ms: float
    max_tracking_err: int
    mean_tracking_err: float
    current_max_mA: int
    current_mean_mA: float
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


def run_single_axis_servo_experiment(
    *,
    port: str = "/dev/ttyUSB0",
    baudrate: int = 921600,
    motor_id: int = 1,
    hold_duration_s: float = 3.0,
    delta_pos: int = 50,
    rate_hz: float = 50.0,
) -> ExperimentResult:
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

    adapter = J7UartAdapter(
        ser,
        baudrate=baudrate,
        timeout_s=0.015,
        min_frame_interval_s=0.0005,
    )

    records: List[CycleRecord] = []
    stop_success = False
    passed = False

    try:
        # Step 1: Preflight read initial status
        print(f"[Phase 1] Preflight status read on ID{motor_id}...")
        raw_status = adapter.request(
            pack_read_status_cmd(motor_id),
            expected_id=motor_id,
            expected_command=CMD_RD_STATUS,
            retries=2,
        )
        init_status = unpack_status_response(raw_status)
        q_current = init_status.current_position
        print(
            f"  Initial state: position={q_current}, current={init_status.current_mA} mA, "
            f"fault={init_status.error_code}, temp={init_status.temperature}°C"
        )
        if init_status.error_code != 0:
            raise RuntimeError(f"Initial status returned fault={init_status.error_code}")
        if not (0 <= q_current <= 2000):
            raise RuntimeError(f"Initial position {q_current} out of range [0, 2000]")

        # Determine safe delta target
        if q_current + delta_pos <= 2000:
            target_peak = q_current + delta_pos
        else:
            target_peak = q_current - delta_pos

        # Step 2: Switch to Servo mode (mode=1)
        print(f"[Phase 2] Setting Servo mode (mode=1, speed=50)...")
        raw_mode_resp = adapter.request(
            pack_write_register_cmd(motor_id, CONTROL_MODE_ADDR, [1, 0, 0, 50]),
            expected_id=motor_id,
            expected_command=CMD_WR_REGISTER,
            retries=1,
        )
        mode_write_resp = unpack_write_response(raw_mode_resp)
        print(f"  Mode write ACK received: error_code={mode_write_resp.error_code}")
        if mode_write_resp.error_code != 0:
            raise RuntimeError(f"Servo mode configuration returned fault={mode_write_resp.error_code}")

        # Step 3 & 4: Build profile and stream at 50Hz
        period_s = 1.0 / rate_hz
        period_ns = int(period_s * 1e9)

        # Profile planning:
        # 1. Hold at q_current for hold_duration_s (3.0s -> 150 cycles)
        # 2. Ramp to target_peak over 1.0s (50 cycles)
        # 3. Hold at target_peak for 1.0s (50 cycles)
        # 4. Ramp back to q_current over 1.0s (50 cycles)
        # 5. Hold at q_current for 1.0s (50 cycles)
        hold_cycles = int(round(hold_duration_s * rate_hz))
        ramp_up_cycles = int(round(1.0 * rate_hz))
        peak_hold_cycles = int(round(1.0 * rate_hz))
        ramp_down_cycles = int(round(1.0 * rate_hz))
        final_hold_cycles = int(round(1.0 * rate_hz))

        targets: List[int] = []
        for _ in range(hold_cycles):
            targets.append(q_current)
        for i in range(1, ramp_up_cycles + 1):
            pos = int(round(q_current + (target_peak - q_current) * (i / ramp_up_cycles)))
            targets.append(pos)
        for _ in range(peak_hold_cycles):
            targets.append(target_peak)
        for i in range(1, ramp_down_cycles + 1):
            pos = int(round(target_peak + (q_current - target_peak) * (i / ramp_down_cycles)))
            targets.append(pos)
        for _ in range(final_hold_cycles):
            targets.append(q_current)

        total_cycles = len(targets)
        motion_cycles = total_cycles - hold_cycles
        print(f"[Phase 3] Starting 50Hz Servo stream: total {total_cycles} cycles (~{total_cycles * period_s:.1f}s)...")
        print(f"  Hold: {hold_cycles} cycles (3.0s) @ q={q_current}")
        print(f"  Motion: q={q_current} -> {target_peak} -> {q_current} ({motion_cycles} cycles)")

        t_experiment_start_ns = time.monotonic_ns()
        t_prev_cycle_ns = t_experiment_start_ns

        for cycle_id, target_pos in enumerate(targets):
            t_cycle_start_ns = time.monotonic_ns()
            dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if cycle_id > 0 else period_s * 1000.0
            t_prev_cycle_ns = t_cycle_start_ns

            # 1. Write 0x29 target position (write-only, do not wait)
            frame_0x29 = pack_write_register_cmd(motor_id, TARGET_POSITION_ADDR, [target_pos])
            adapter.write(frame_0x29)

            # 2. Query 0x30 status
            timeout = False
            fault = 0
            actual_pos = target_pos
            current_mA = 0
            try:
                raw_fb = adapter.request(
                    pack_read_status_cmd(motor_id),
                    expected_id=motor_id,
                    expected_command=CMD_RD_STATUS,
                    retries=0,
                )
                fb_status = unpack_status_response(raw_fb)
                actual_pos = fb_status.current_position
                current_mA = fb_status.current_mA
                fault = fb_status.error_code
            except Exception as exc:
                timeout = True
                print(f"  [Cycle {cycle_id}] Feedback error: {exc}")

            tracking_err = actual_pos - target_pos
            record = CycleRecord(
                cycle_id=cycle_id,
                t_start_s=(t_cycle_start_ns - t_experiment_start_ns) / 1e9,
                dt_ms=dt_ms,
                target_pos=target_pos,
                actual_pos=actual_pos,
                tracking_err=tracking_err,
                current_mA=current_mA,
                fault=fault,
                timeout=timeout,
            )
            records.append(record)

            # Health checks
            if fault != 0:
                print(f"  [EMERGENCY STOP] Motor returned fault={fault} at cycle {cycle_id}")
                break
            if timeout:
                print(f"  [EMERGENCY STOP] Feedback timeout at cycle {cycle_id}")
                break
            if current_mA > 2000:
                print(f"  [EMERGENCY STOP] Overcurrent {current_mA} mA at cycle {cycle_id}")
                break

            # Progress log every 50 cycles
            if cycle_id % 50 == 0 or cycle_id == total_cycles - 1:
                phase_label = "HOLD" if cycle_id < hold_cycles else "MOTION"
                print(
                    f"  [{phase_label:6s}] Cycle {cycle_id:3d}/{total_cycles:3d} (t={record.t_start_s:5.2f}s): "
                    f"target={target_pos:4d}, actual={actual_pos:4d}, err={tracking_err:+3d}, "
                    f"current={current_mA:3d}mA, dt={dt_ms:5.2f}ms"
                )

            # Precise timing wait to next deadline
            next_deadline_ns = t_experiment_start_ns + (cycle_id + 1) * period_ns
            now_ns = time.monotonic_ns()
            rem_ns = next_deadline_ns - now_ns
            if rem_ns > 0:
                time.sleep(rem_ns / 1e9)

    except Exception as exc:
        print(f"[ERROR] Exception during Servo run: {exc}")
    finally:
        # Step 5: Stop command (0x1A)
        print("[Phase 4] Sending 0x1A Stop command...")
        try:
            raw_stop = adapter.request(
                pack_write_register_cmd(motor_id, PAUSE_ADDR, [1]),
                expected_id=motor_id,
                expected_command=CMD_WR_REGISTER,
                retries=2,
            )
            stop_resp = unpack_write_response(raw_stop)
            stop_success = (stop_resp.error_code == 0)
            print(f"  Stop ACK received: error_code={stop_resp.error_code}")
        except Exception as exc:
            print(f"  Stop command failed: {exc}")
            stop_success = False

        # Final status check
        try:
            raw_final = adapter.request(
                pack_read_status_cmd(motor_id),
                expected_id=motor_id,
                expected_command=CMD_RD_STATUS,
                retries=2,
            )
            final_status = unpack_status_response(raw_final)
            q_final = final_status.current_position
            print(f"  Final state after stop: position={q_final}, fault={final_status.error_code}")
        except Exception:
            q_final = records[-1].actual_pos if records else -1

        ser.close()

    # Compute analysis metrics
    if not records:
        return ExperimentResult(
            port=port,
            baudrate=baudrate,
            motor_id=motor_id,
            q_initial=0,
            q_target_max=0,
            q_final=0,
            total_cycles=0,
            hold_cycles=0,
            motion_cycles=0,
            target_frequency_hz=rate_hz,
            actual_frequency_hz=0.0,
            period_mean_ms=0.0,
            period_min_ms=0.0,
            period_max_ms=0.0,
            period_std_ms=0.0,
            max_tracking_err=0,
            mean_tracking_err=0.0,
            current_max_mA=0,
            current_mean_mA=0.0,
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

    tracking_errors = [abs(r.tracking_err) for r in records]
    max_err = max(tracking_errors)
    mean_err = sum(tracking_errors) / len(tracking_errors)

    currents = [r.current_mA for r in records]
    max_curr = max(currents)
    mean_curr = sum(currents) / len(currents)

    faults = sum(1 for r in records if r.fault != 0)
    timeouts = sum(1 for r in records if r.timeout)

    passed = (
        len(records) == total_cycles
        and faults == 0
        and timeouts == 0
        and stop_success
        and max_err <= 15
        and abs(actual_freq - rate_hz) <= 1.5
    )

    summary = (
        f"Executed {len(records)}/{total_cycles} cycles. "
        f"Freq: {actual_freq:.2f} Hz (mean dt: {period_mean:.2f} ms ± {period_std:.2f} ms). "
        f"Tracking: max err {max_err} steps, mean err {mean_err:.2f} steps. "
        f"Current: max {max_curr} mA, mean {mean_curr:.1f} mA. "
        f"Faults: {faults}, Timeouts: {timeouts}, Stop: {'OK' if stop_success else 'FAIL'}."
    )

    return ExperimentResult(
        port=port,
        baudrate=baudrate,
        motor_id=motor_id,
        q_initial=records[0].actual_pos,
        q_target_max=max(r.target_pos for r in records),
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
        max_tracking_err=max_err,
        mean_tracking_err=mean_err,
        current_max_mA=max_curr,
        current_mean_mA=mean_curr,
        fault_count=faults,
        timeout_count=timeouts,
        stop_success=stop_success,
        passed=passed,
        summary=summary,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="J7 ID1 Servo Experiment Runner")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--hold-duration", type=float, default=3.0)
    parser.add_argument("--delta-pos", type=int, default=50)
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    res = run_single_axis_servo_experiment(
        port=args.port,
        baudrate=args.baudrate,
        motor_id=args.motor_id,
        hold_duration_s=args.hold_duration,
        delta_pos=args.delta_pos,
        rate_hz=args.rate,
    )

    print("\n=================== EXPERIMENT RESULT ===================")
    print(f"Result: {'PASSED' if res.passed else 'FAILED'}")
    print(f"Summary: {res.summary}")
    print(f"Initial Pos: {res.q_initial}, Peak Pos: {res.q_target_max}, Final Pos: {res.q_final}")
    print(f"Frequency: {res.actual_frequency_hz:.2f} Hz (target: {res.target_frequency_hz:.1f} Hz)")
    print(f"Period stats: mean={res.period_mean_ms:.2f}ms, min={res.period_min_ms:.2f}ms, max={res.period_max_ms:.2f}ms, std={res.period_std_ms:.2f}ms")
    print(f"Tracking error: max={res.max_tracking_err} steps, mean={res.mean_tracking_err:.2f} steps")
    print(f"Current: max={res.current_max_mA} mA, mean={res.current_mean_mA:.1f} mA")
    print(f"Faults: {res.fault_count}, Timeouts: {res.timeout_count}, Stop ACK: {res.stop_success}")
    print("=========================================================")
    return 0 if res.passed else 1


if __name__ == "__main__":
    sys.exit(main())
