#!/usr/bin/env python3
"""Production Runner for J7 Six-Axis 50Hz Servo HOME and PARK Operations.

Executes standalone, smooth, 50Hz Servo S-curve motion:
- --action home: q_current -> OPERATING_HOME ([1000]*6) -> Hold -> 0x1A STOP
- --action park: q_current -> STORAGE_PARK ([0]*6)     -> Hold -> 0x1A STOP

Reuses verified J7 transport & feedback scheduler:
- Mode=1 (Servo Mode)
- 50Hz continuous 0x29 write-only batch
- Round-robin 1-axis 0x30 feedback per cycle (~8.33Hz per axis)
- Zero runtime 0x25 mode readback
- Protected by try/finally 0x1A clean stop
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

# Ensure workspace packages are on path
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

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
    parse_frame,
)
from pus6_motor_driver.safety import (
    ABSOLUTE_RAW_LOWER_LIMIT,
    ABSOLUTE_RAW_UPPER_LIMIT,
    CONTROL_MODE_ADDR,
    FEEDBACK_LOWER_LIMIT,
    FEEDBACK_UPPER_LIMIT,
    MOTOR_IDS,
    PAUSE_ADDR,
    TARGET_POSITION_ADDR,
    HardwareMode,
    SafetyViolation,
    StartupClassification,
    StartupStateAssessment,
    classify_startup_state,
)
from pus6_motor_driver.fake_bus import FakeBus
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

HOME_TARGET: Tuple[int, ...] = (1000, 1000, 1000, 1000, 1000, 1000)
PARK_TARGET: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)
QUINTIC_PEAK_DERIVATIVE = 1.875
STANDALONE_WRITE_TIMEOUT_S = 0.025
STANDALONE_RX_DRAIN_BUDGET_S = 0.004
STANDALONE_INCOMPLETE_FEEDBACK_ESCALATION_CYCLES = 4


class MockServoBus(FakeBus):
    """Mock UART bus simulating J7 Servo responses for offline testing."""

    def __init__(self, initial_positions: Optional[Sequence[int]] = None) -> None:
        super().__init__()
        self.uart_lock = threading.RLock()
        self._clock_ns = time.monotonic_ns
        self.rx_queue = deque()
        self.written_frames: list[bytes] = []
        self.timeout_on_axis: Optional[int] = None
        self.fault_on_axis: Optional[int] = None
        if initial_positions is not None:
            for i, mid in enumerate(MOTOR_IDS):
                self.positions[mid] = int(initial_positions[i])
                self.targets[mid] = int(initial_positions[i])

    def write(self, frame: bytes) -> int:
        self.written_frames.append(bytes(frame))
        parsed = parse_frame(frame)
        mid = parsed.motor_id
        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr == 0x25:
                mode_val = int.from_bytes(parsed.data, "little", signed=False)
                self.modes[mid] = mode_val
            elif parsed.register_addr == 0x29:
                pos_val = int.from_bytes(parsed.data, "little", signed=True)
                self.positions[mid] = pos_val
                self.targets[mid] = pos_val
            elif parsed.register_addr == 0x1A:
                pass
            if self.fault_on_axis is not None and mid == 1:
                self.faults[mid] = self.fault_on_axis
            if self.timeout_on_axis != mid:
                resp = super().request(frame)
                self.rx_queue.append(resp)
        return len(frame)

    def drain_motion_feedbacks(self) -> dict[int, StatusFeedback]:
        feedbacks: dict[int, StatusFeedback] = {}
        while self.rx_queue:
            raw = self.rx_queue.popleft()
            try:
                fb = unpack_status_response(raw)
                mid = raw[3]
                feedbacks[mid] = fb
            except Exception:
                pass
        return feedbacks

    def request(self, frame: bytes, **kwargs) -> bytes:
        self.written_frames.append(bytes(frame))
        parsed = parse_frame(frame)
        mid = parsed.motor_id
        if self.timeout_on_axis == mid:
            raise TimeoutError(f"Mock timeout on ID{mid}")
        if parsed.command == CMD_RD_STATUS and self.fault_on_axis is not None and mid == 1:
            self.faults[mid] = self.fault_on_axis
        return super().request(frame)

    def read_available(self) -> bytes:
        data = b"".join(self.rx_queue)
        self.rx_queue.clear()
        return data


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
    """Quintic polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5 for tau in [0, 1]."""
    tau = max(0.0, min(1.0, float(tau)))
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


@dataclass
class ServoHomeParkResult:
    action: str
    target: Tuple[int, ...]
    q_initial: Tuple[int, ...]
    q_final: Tuple[int, ...]
    total_cycles: int
    transition_duration_s: float
    hold_duration_s: float
    planned_peak_speed: float
    actual_frequency_hz: float
    batch_mean_ms: float
    batch_max_ms: float
    max_tracking_err: int
    final_target_err: int
    fault_count: int
    timeout_count: int
    stop_success: bool
    passed: bool
    summary: str
    deadline_miss_count: int = 0
    last_tx_failure_provenance: Optional[Dict[str, Any]] = None
    transport_exception_timestamp_ns: Optional[int] = None
    first_internal_stop_start_ns: Optional[int] = None
    first_internal_stop_end_ns: Optional[int] = None
    first_internal_stop_duration_ms: float = 0.0
    outer_redundant_stop_called: bool = False
    transport_failure_to_first_stop_complete_latency_ms: float = 0.0


def run_servo_home_park(
    action: str,
    *,
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
    speed: float = 100.0,
    rate_hz: float = 50.0,
    hold_duration_s: float = 2.0,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    mock_bus: Optional[MockServoBus] = None,
    sleep_fn: Optional[Any] = None,
) -> ServoHomeParkResult:
    action = action.strip().lower()
    if action not in ("home", "park"):
        raise ValueError(f"Invalid action: {action!r}. Must be 'home' or 'park'.")

    target = HOME_TARGET if action == "home" else PARK_TARGET

    if real_hardware and not confirm_motion:
        raise ValueError("--real-hardware requires --confirm-motion to authorize physical motion.")

    is_real = real_hardware and confirm_motion
    sleeper = sleep_fn if sleep_fn is not None else time.sleep
    ser = None
    stop_adapter = None
    records: List[Dict[str, Any]] = []
    stop_success = False
    passed = False
    q_initial: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    q_final: Tuple[int, ...] = (0, 0, 0, 0, 0, 0)
    latest_positions: Dict[int, int] = {mid: 0 for mid in MOTOR_IDS}
    tx_failure_provenance: Optional[Dict[str, Any]] = None
    transport_exception_timestamp_ns: Optional[int] = None
    first_internal_stop_start_ns: Optional[int] = None
    first_internal_stop_end_ns: Optional[int] = None
    first_internal_stop_duration_ms = 0.0
    outer_redundant_stop_called = False
    failure_deadline_miss = False

    def issue_stop(reason: str, *, is_outer_cleanup: bool = False):
        nonlocal first_internal_stop_start_ns, first_internal_stop_end_ns
        nonlocal first_internal_stop_duration_ms, outer_redundant_stop_called
        was_first_stop = first_internal_stop_start_ns is None
        stop_start_ns = time.monotonic_ns()
        audit = None
        if stop_adapter is not None:
            audit = stop_adapter.stop_all(reason)
        stop_end_ns = time.monotonic_ns()
        if was_first_stop and not is_outer_cleanup:
            first_internal_stop_start_ns = stop_start_ns
            first_internal_stop_end_ns = stop_end_ns
            first_internal_stop_duration_ms = (stop_end_ns - stop_start_ns) / 1e6
        elif is_outer_cleanup and first_internal_stop_start_ns is not None:
            outer_redundant_stop_called = True
        return audit

    try:
        if is_real:
            if serial is None:
                raise RuntimeError("pyserial is required for real hardware execution.")
            set_latency_timer(port)
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.015,
                write_timeout=STANDALONE_WRITE_TIMEOUT_S,
            )
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            adapter = J7UartAdapter(
                ser,
                baudrate=baudrate,
                timeout_s=0.015,
                min_frame_interval_s=0.0005,
            )
        else:
            bus = mock_bus or MockServoBus(initial_positions=(0, 0, 0, 0, 0, 0) if action == "home" else (1000, 1000, 1000, 1000, 1000, 1000))
            adapter = bus

        stop_adapter = J7StopAdapter(adapter)
        writer = J7SixAxisServoTransport(
            adapter,
            cycle_rate_hz=rate_hz,
            rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
            min_frame_interval_s=0.0005,
            rx_drain_budget_s=STANDALONE_RX_DRAIN_BUDGET_S,
            pause_callback=stop_adapter,
        )
        feedback = J7ServoFeedbackScheduler(
            writer,
            full_scan_period_s=1e9,
            feedback_budget_s=0.010,
            timeout_escalation_misses=STANDALONE_INCOMPLETE_FEEDBACK_ESCALATION_CYCLES,
        )

        # Step 1: Preflight 0x30 status scan (3 consecutive scans for stability audit)
        print(f"[{action.upper()} Phase 1] Preflight full status scan across ID1-ID6 (3 stability samples)...")
        samples_history: List[List[int]] = []
        for pre_scan in range(3):
            preflight_audit = feedback.full_scan(
                servo_cycle_id=pre_scan,
                servo_cycle_timestamp_ns=time.monotonic_ns(),
            )
            if preflight_audit.deferred or len(preflight_audit.samples) != len(MOTOR_IDS):
                raise RuntimeError(f"Preflight full scan {pre_scan+1}/3 incomplete")
            for sample in preflight_audit.samples:
                if sample.timeout or sample.fault != 0:
                    raise RuntimeError(
                        f"ID{sample.motor_id} preflight failure: fault={sample.fault}, timeout={sample.timeout}"
                    )
            samples_history.append([s.position for s in preflight_audit.samples if s.position is not None])
            if pre_scan < 2:
                sleeper(0.02)

        init_positions = samples_history[-1]
        for s in preflight_audit.samples:
            if s.position is not None:
                latest_positions[s.motor_id] = s.position
                print(
                    f"  ID{s.motor_id}: position={s.position:4d} step, "
                    f"current={s.current_mA:3d} mA, fault={s.fault}"
                )

        # Authoritative Startup Classification
        assessment = classify_startup_state(init_positions, history_samples=samples_history)
        print(
            f"  [STARTUP CLASSIFIER] Classification: {assessment.classification.value.upper()} | "
            f"Planning Seed: {list(assessment.planning_seed)} | "
            f"Info: {assessment.description}"
        )
        if not assessment.is_valid:
            raise SafetyViolation(
                f"Cannot execute {action.upper()} from invalid startup state: {assessment.description}"
            )

        q_initial = tuple(init_positions)
        q_plan_start = assessment.planning_seed
        print(f"  q_initial = {list(q_initial)}")
        print(f"  q_target  = {list(target)}")

        # Step 2: Configure Servo mode (mode=1, speed=50) with write ACK verification
        print(f"\n[{action.upper()} Phase 2] Configuring six axes into Servo mode (mode=1, motor_reg_speed=50)...")
        configs = writer.configure_servo(speed=50)
        if len(configs) != 6 or any(c.mode != 1 for c in configs):
            raise RuntimeError("Servo mode configuration failed or not acknowledged")
        print("  All 6 axes configured: mode=1 acknowledged, fault=0.")

        # Step 3: Plan quintic smooth trajectory from authoritative planning seed to target
        d_max = max(abs(target[i] - q_plan_start[i]) for i in range(6))
        speed = max(1.0, float(speed))
        transition_duration_s = max(2.0, (QUINTIC_PEAK_DERIVATIVE * float(d_max)) / speed) if d_max > 0 else 0.0
        planned_peak_speed = (QUINTIC_PEAK_DERIVATIVE * float(d_max)) / transition_duration_s if transition_duration_s > 0 else 0.0

        period_s = 1.0 / rate_hz
        period_ns = int(period_s * 1e9)
        trans_cycles = int(round(transition_duration_s * rate_hz))
        hold_cycles = int(round(hold_duration_s * rate_hz))

        cycle_targets: List[Tuple[Tuple[int, ...], str]] = []

        # Transition cycles
        for k in range(1, trans_cycles + 1):
            s = quintic_progress(k / trans_cycles) if trans_cycles > 0 else 1.0
            tgt = tuple(int(round(q_plan_start[i] + (target[i] - q_plan_start[i]) * s)) for i in range(6))
            cycle_targets.append((tgt, f"TRANSITION_TO_{action.upper()}"))

        # Hold cycles
        for _ in range(hold_cycles):
            cycle_targets.append((target, f"{action.upper()}_HOLD"))

        total_cycles = len(cycle_targets)
        total_time_s = total_cycles * period_s

        print(f"\n[{action.upper()} Phase 3] Starting 50Hz Servo motion stream:")
        print(f"  Target: {list(target)} ({action.upper()})")
        print(f"  Total cycles: {total_cycles} (~{total_time_s:.2f}s)")
        print(f"  Transition: {transition_duration_s:.2f}s ({trans_cycles} cycles), Planned peak speed: {planned_peak_speed:.1f} steps/s ({planned_peak_speed * 0.025:.2f} mm/s)")
        print(f"  Hold duration: {hold_duration_s:.1f}s ({hold_cycles} cycles)")

        t_start_exp_ns = time.monotonic_ns()
        t_prev_cycle_ns = t_start_exp_ns

        for cycle_id, (target_vec, phase_name) in enumerate(cycle_targets):
            t_cycle_start_ns = time.monotonic_ns()
            dt_ms = (t_cycle_start_ns - t_prev_cycle_ns) / 1e6 if cycle_id > 0 else period_s * 1000.0
            t_prev_cycle_ns = t_cycle_start_ns

            # 1. 6-axis write 0x32 batch with integrated 0x32 StatusFeedback ingestion
            servo_audit = writer.send_cycle(
                list(target_vec),
                cycle_id=cycle_id,
                target_timestamp_ns=t_cycle_start_ns,
            )
            batch_duration_ms = (servo_audit.cycle_end_ns - servo_audit.cycle_start_ns) / 1e6

            # 2. Design-B direct StatusFeedback ingestion (replaces redundant 0x30 poll)
            feedback_audit = feedback.ingest_cycle_feedback(
                servo_audit.feedbacks,
                servo_cycle_id=cycle_id,
                servo_cycle_timestamp_ns=t_cycle_start_ns,
            )

            # Evaluate all 6 axes feedbacks
            cycle_faults = 0
            cycle_timeouts = feedback_audit.timeout_count
            max_curr = 0
            tracking_errors = []

            for sample in feedback_audit.samples:
                mid = sample.motor_id
                target_step = target_vec[mid - 1]
                if sample.position is not None:
                    act_pos = sample.position
                    latest_positions[mid] = act_pos
                else:
                    act_pos = latest_positions.get(mid, target_step)

                err = act_pos - target_step
                tracking_errors.append(abs(err))

                curr = sample.current_mA if sample.current_mA is not None else 0
                max_curr = max(max_curr, curr)
                if sample.fault is not None and sample.fault != 0:
                    cycle_faults += 1

                # Safety guards per axis
                if sample.fault is not None and sample.fault != 0:
                    print(f"  [SAFETY STOP] ID{mid} fault={sample.fault} at cycle {cycle_id}")
                    issue_stop(f"fault_on_ID{mid}")
                    raise SafetyViolation(f"Motor fault on ID{mid}: {sample.fault}")
                if curr > 1500:
                    print(f"  [SAFETY STOP] ID{mid} overcurrent {curr} mA at cycle {cycle_id}")
                    issue_stop(f"overcurrent_on_ID{mid}")
                    raise SafetyViolation(f"Overcurrent on ID{mid}: {curr} mA")
                if abs(err) > 35:
                    print(f"  [SAFETY STOP] ID{mid} tracking divergence err={err} at cycle {cycle_id}")
                    issue_stop(f"divergence_on_ID{mid}")
                    raise SafetyViolation(f"Tracking divergence on ID{mid}: {err} steps")

            if cycle_timeouts > 0:
                print(f"  [SOFT RT] Incomplete feedback on {cycle_timeouts} axes at cycle {cycle_id}")

            max_cycle_err = max(tracking_errors) if tracking_errors else 0

            # Soft real-time audit only: deadline overruns are recorded, not used
            # as a hard stop condition in this standalone HOME/PARK policy.
            cycle_end_before_sleep_ns = time.monotonic_ns()
            cycle_active_duration_ms = (cycle_end_before_sleep_ns - t_cycle_start_ns) / 1e6
            deadline_miss = cycle_active_duration_ms > (period_s * 1000.0)

            records.append({
                "cycle_id": cycle_id,
                "dt_ms": dt_ms,
                "batch_duration_ms": batch_duration_ms,
                "active_duration_ms": cycle_active_duration_ms,
                "deadline_miss": deadline_miss,
                "max_tracking_err": max_cycle_err,
                "max_current_mA": max_curr,
                "fault_count": cycle_faults,
                "timeout_count": cycle_timeouts,
                "feedback_count": len(feedback_audit.samples) - cycle_timeouts,
                "phase_name": phase_name,
            })

            # Progress logging
            is_phase_start = (cycle_id == 0 or phase_name != cycle_targets[cycle_id - 1][1])
            if cycle_id % 50 == 0 or is_phase_start or cycle_id == total_cycles - 1:
                polled_axis = (cycle_id % len(MOTOR_IDS)) + 1
                polled_target = target_vec[polled_axis - 1]
                polled_pos = latest_positions.get(polled_axis, polled_target)
                polled_err = polled_pos - polled_target
                polled_sample = next((s for s in feedback_audit.samples if s.motor_id == polled_axis), None)
                polled_curr = polled_sample.current_mA if polled_sample and polled_sample.current_mA else 0
                print(
                    f"  [{phase_name:18s}] Cycle {cycle_id:4d}/{total_cycles:4d}: "
                    f"polled ID{polled_axis} (tgt={polled_target:4d}, act={polled_pos:4d}, err={polled_err:+3d}, "
                    f"curr={polled_curr:3d}mA) | batch_TX={batch_duration_ms:4.2f}ms | 6-axis FB=OK"
                )

            # Sleep to next cycle deadline
            next_deadline_ns = t_start_exp_ns + (cycle_id + 1) * period_ns
            now_ns = time.monotonic_ns()
            rem_ns = next_deadline_ns - now_ns
            if rem_ns > 0:
                sleeper(rem_ns / 1e9)

    except KeyboardInterrupt:
        print(f"\n[INTERRUPT] Operator Ctrl+C caught during {action.upper()}. Triggering emergency stop...")
        issue_stop("operator_ctrl_c")
    except Exception as exc:
        t_exc_ns = time.monotonic_ns()
        cycle_elapsed_at_failure_ms = (t_exc_ns - t_cycle_start_ns) / 1e6 if 't_cycle_start_ns' in locals() else 0.0
        deadline_miss_on_failure = cycle_elapsed_at_failure_ms > (period_s * 1000.0) if 'period_s' in locals() else False
        failure_deadline_miss = deadline_miss_on_failure
        print(f"\n[ERROR] Exception during {action.upper()}: {exc} (cycle elapsed: {cycle_elapsed_at_failure_ms:.2f}ms, deadline_miss={deadline_miss_on_failure})")

        tx_failure_provenance = getattr(writer, "last_tx_failure_provenance", None)
        transport_exception_timestamp_ns = t_exc_ns

        issue_stop("exception_during_run")

        if tx_failure_provenance is not None:
            tx_failure_provenance["cycle_elapsed_at_failure_ms"] = cycle_elapsed_at_failure_ms
            tx_failure_provenance["deadline_miss"] = deadline_miss_on_failure
            tx_failure_provenance["emergency_stop_duration_ms"] = first_internal_stop_duration_ms
            tx_failure_provenance["tx_elapsed_time_ms"] = tx_failure_provenance.get("elapsed_write_time_ms")
    finally:
        # Step 4: Stop command (0x1A) and final full status scan
        print(f"\n[{action.upper()} Phase 4] Sending best-effort 0x1A Stop command to all six axes...")
        if stop_adapter is not None:
            stop_audit = issue_stop("motion_finish", is_outer_cleanup=True)
            stop_success = stop_audit.success
            print(f"  0x1A Stop completed: success={stop_success}, reason={stop_audit.reason}")

        final_pos: List[int] = []
        try:
            sleeper(0.05)
            # Query status directly for each axis
            for mid in MOTOR_IDS:
                try:
                    resp = adapter.request(pack_read_status_cmd(mid))
                    status = unpack_status_response(resp)
                    final_pos.append(status.current_position)
                    latest_positions[mid] = status.current_position
                    print(f"  Final ID{mid}: pos={status.current_position:4d} step, current={status.current_mA:3d} mA, fault={status.error_code}")
                except Exception as exc:
                    final_pos.append(latest_positions.get(mid, -1))
            q_final = tuple(final_pos)
        except Exception as exc:
            print(f"  Final status read error: {exc}")
            q_final = tuple(latest_positions.get(mid, -1) for mid in MOTOR_IDS)

        if ser is not None:
            ser.close()

    deadline_miss_count = sum(1 for r in records if r.get("deadline_miss", False))
    if failure_deadline_miss:
        deadline_miss_count += 1

    transport_failure_to_first_stop_complete_latency_ms = 0.0
    if transport_exception_timestamp_ns is not None and first_internal_stop_end_ns is not None:
        transport_failure_to_first_stop_complete_latency_ms = max(
            0.0,
            (first_internal_stop_end_ns - transport_exception_timestamp_ns) / 1e6,
        )

    if not records:
        return ServoHomeParkResult(
            action=action,
            target=target,
            q_initial=q_initial,
            q_final=q_final,
            total_cycles=0,
            transition_duration_s=0.0,
            hold_duration_s=hold_duration_s,
            planned_peak_speed=0.0,
            actual_frequency_hz=0.0,
            batch_mean_ms=0.0,
            batch_max_ms=0.0,
            max_tracking_err=0,
            final_target_err=9999,
            fault_count=1,
            timeout_count=1,
            stop_success=stop_success,
            passed=False,
            summary="0 cycles executed (soft real-time policy; deadline overruns are audited, not hard-stopped)",
            deadline_miss_count=deadline_miss_count,
            last_tx_failure_provenance=tx_failure_provenance,
            transport_exception_timestamp_ns=transport_exception_timestamp_ns,
            first_internal_stop_start_ns=first_internal_stop_start_ns,
            first_internal_stop_end_ns=first_internal_stop_end_ns,
            first_internal_stop_duration_ms=first_internal_stop_duration_ms,
            outer_redundant_stop_called=outer_redundant_stop_called,
            transport_failure_to_first_stop_complete_latency_ms=transport_failure_to_first_stop_complete_latency_ms,
        )

    faults = sum(r.get("fault_count", 0) for r in records)
    timeouts = sum(r.get("timeout_count", 0) for r in records)
    batch_durations = [r["batch_duration_ms"] for r in records]
    batch_mean = sum(batch_durations) / len(batch_durations)
    batch_max = max(batch_durations)
    max_tracking_err = max(r.get("max_tracking_err", 0) for r in records)
    final_errors = [abs(q_final[i] - target[i]) for i in range(min(len(q_final), len(target)))]
    max_final_err = max(final_errors) if final_errors else 9999

    passed = (
        len(records) == total_cycles
        and faults == 0
        and stop_success
        and max_final_err <= 5
        and not (tx_failure_provenance and tx_failure_provenance.get("delivery_uncertain"))
    )

    summary = (
        f"[{action.upper()} {'PASS' if passed else 'FAIL'}] Executed {len(records)}/{total_cycles} cycles. "
        f"Initial: {list(q_initial)} -> Final: {list(q_final)} (Target: {list(target)}, max err: {max_final_err} steps). "
        f"Batch TX: {batch_mean:.2f}ms (max: {batch_max:.2f}ms). Max tracking err: {max_tracking_err} steps. "
        f"Faults: {faults}, Timeouts: {timeouts}, Deadline Misses: {deadline_miss_count}, Soft RT: deadline overruns are audited, not auto-stopped, 0x1A Stop: {'OK' if stop_success else 'FAIL'}."
    )

    return ServoHomeParkResult(
        action=action,
        target=target,
        q_initial=q_initial,
        q_final=q_final,
        total_cycles=len(records),
        transition_duration_s=transition_duration_s,
        hold_duration_s=hold_duration_s,
        planned_peak_speed=planned_peak_speed,
        actual_frequency_hz=rate_hz,
        batch_mean_ms=batch_mean,
        batch_max_ms=batch_max,
        max_tracking_err=max_tracking_err,
        final_target_err=max_final_err,
        fault_count=faults,
        timeout_count=timeouts,
        stop_success=stop_success,
        passed=passed,
        summary=summary,
        deadline_miss_count=deadline_miss_count,
        last_tx_failure_provenance=tx_failure_provenance,
        transport_exception_timestamp_ns=transport_exception_timestamp_ns,
        first_internal_stop_start_ns=first_internal_stop_start_ns,
        first_internal_stop_end_ns=first_internal_stop_end_ns,
        first_internal_stop_duration_ms=first_internal_stop_duration_ms,
        outer_redundant_stop_called=outer_redundant_stop_called,
        transport_failure_to_first_stop_complete_latency_ms=transport_failure_to_first_stop_complete_latency_ms,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Production J7 Six-Axis 50Hz Servo HOME / PARK Runner")
    parser.add_argument(
        "--action",
        required=True,
        choices=["home", "park"],
        help="Action to execute: 'home' (to [1000]*6) or 'park' (to [0]*6)",
    )
    parser.add_argument("--speed", type=float, default=100.0, help="Planned peak speed in step/s (default: 100.0)")
    parser.add_argument("--rate", type=float, default=50.0, help="Servo cycle rate in Hz (default: 50.0)")
    parser.add_argument("--hold-duration", type=float, default=2.0, help="Hold duration at target in seconds (default: 2.0)")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", help="Serial port path")
    parser.add_argument("--baudrate", type=int, default=921600, help="Baudrate (default: 921600)")
    parser.add_argument("--real-hardware", action="store_true", help="Execute on real hardware (defaults to mock mode)")
    parser.add_argument("--confirm-motion", action="store_true", help="Explicit confirmation for real hardware motion")
    args = parser.parse_args()

    res = run_servo_home_park(
        action=args.action,
        port=args.port,
        baudrate=args.baudrate,
        speed=args.speed,
        rate_hz=args.rate,
        hold_duration_s=args.hold_duration,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
    )

    print("\n=================== J7 SERVO ACTION RESULT ===================")
    print(f"Action: {res.action.upper()}")
    print(f"Result: {'PASSED' if res.passed else 'FAILED'}")
    print(f"Summary: {res.summary}")
    print(f"q_initial: {list(res.q_initial)}")
    print(f"q_target:  {list(res.target)}")
    print(f"q_final:   {list(res.q_final)}")
    print(f"Max Final Return Error: {res.final_target_err} steps")
    print(f"Faults: {res.fault_count}, Timeouts: {res.timeout_count}, 0x1A Stop: {res.stop_success}")
    print("==============================================================")
    return 0 if res.passed else 1


if __name__ == "__main__":
    sys.exit(main())
