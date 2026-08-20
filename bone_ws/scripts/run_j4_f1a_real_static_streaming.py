#!/usr/bin/env python3
"""Stage J4-F1A — 25 Hz Real-Hardware Static Target Streaming Runner.

Executes sequential J4-F1A validation steps:
- J4-F1A-0: Pre-flight READ-ONLY 6-axis state snapshot (0 write), captures q_initial.
- J4-F1A-1: 25 Hz static current-position streaming (target=q_initial) for 2.0 s (50 cycles).
- J4-F1A-2: 25 Hz static current-position streaming (target=q_initial) for 10.0 s (250 cycles).

Safety & Operational Compliance:
- Strictly adheres to /home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md
- Adheres to /home/lyapunov/bone_drill/AGENTS.md
- Zero-Retry on UART motion commands
- Target fixed to q_initial (no active Cartesian motion generated)
- Real-time preflight and stop threshold monitoring
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    MOTOR_IDS,
    NUM_MOTORS,
    DriverError,
    HardwareMode,
    SafetyViolation,
)
from pus6_motor_driver.six_motor_controller import SixAxisStateSnapshot, SixMotorController
from pus6_motor_driver.uart_bus import UartBus


class J4F1ARunner:
    """Executes J4-F1A 25Hz static target streaming on real hardware or mock bus."""

    def __init__(
        self,
        *,
        port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        baudrate: int = 921600,
        rate_hz: float = 25.0,
        hardware_mode: HardwareMode = HardwareMode.MOCK,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.rate_hz = rate_hz
        self.target_period_ms = 1000.0 / rate_hz
        self.hardware_mode = hardware_mode
        self.controller: SixMotorController | None = None
        self.q_initial: list[int] | None = None

    def connect(self) -> None:
        if self.hardware_mode == HardwareMode.MOCK:
            bus = FakeBus()
            bus.positions = {i: 1000 for i in MOTOR_IDS}
            bus.target_positions = {i: 1000 for i in MOTOR_IDS}
            self.controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
        else:
            self.controller = SixMotorController(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.05,
                hardware_mode=self.hardware_mode,
            )
        self.controller.connect()
        # Warm-up tty / USB-serial buffer with one read transaction
        try:
            self.controller.read_state_snapshot()
        except Exception:
            pass

    def close(self) -> None:
        if self.controller is not None:
            self.controller.close()

    def run_f1a_0(self) -> dict[str, Any]:
        """J4-F1A-0: Read-only inspection snapshot without motion."""
        print("\n================================================================================")
        print(" [J4-F1A-0] Initial Read-Only State Inspection (0 Write)")
        print("================================================================================")
        if self.controller is None or not self.controller.is_connected:
            raise RuntimeError("Controller is not connected")

        initial_writes = self.controller.write_count

        t0_ns = time.perf_counter_ns()
        snapshot = self.controller.read_state_snapshot()
        t1_ns = time.perf_counter_ns()
        snap_duration_ms = (t1_ns - t0_ns) / 1e6

        assert self.controller.write_count == initial_writes, "Read-only snapshot caused hardware write!"

        self.q_initial = list(snapshot.positions_steps)

        # Verification of OPERATING_HOME neighborhood [900, 1100]
        in_home_neighborhood = True
        for i, pos in enumerate(self.q_initial):
            if not (900 <= pos <= 1100):
                in_home_neighborhood = False
                print(f"  [WARN] Motor {i+1} position {pos} is outside operating home neighborhood [900, 1100]!")

        all_faults_zero = all(f == 0 for f in snapshot.fault_codes)
        max_current = max(abs(c) for c in snapshot.currents_mA)

        res = {
            "timestamp": datetime.now().isoformat(),
            "phase": "J4-F1A-0",
            "snapshot_duration_ms": snap_duration_ms,
            "raw_positions_steps": self.q_initial,
            "currents_mA": list(snapshot.currents_mA),
            "forces_g": list(snapshot.forces_g_raw),
            "temperatures_C": list(snapshot.temperatures_C),
            "fault_codes": list(snapshot.fault_codes),
            "all_faults_zero": all_faults_zero,
            "max_current_mA": max_current,
            "in_home_neighborhood": in_home_neighborhood,
            "hardware_write_count": self.controller.write_count,
            "pass": bool(all_faults_zero and in_home_neighborhood),
        }

        print(f"  Snapshot Duration:    {snap_duration_ms:.2f} ms")
        print(f"  Captured q_initial:   {self.q_initial} steps")
        print(f"  Raw Currents:         {snapshot.currents_mA} mA (max: {max_current} mA)")
        print(f"  Fault Codes:          {snapshot.fault_codes}")
        print(f"  Home Neighborhood:    {'PASS' if in_home_neighborhood else 'WARN'}")
        print(f"  Phase Status:         {'PASS' if res['pass'] else 'FAIL'}")
        return res

    def run_streaming_window(self, duration_s: float, phase_label: str) -> dict[str, Any]:
        """Execute 25 Hz static target streaming with q_initial for duration_s."""
        print("\n================================================================================")
        print(f" [{phase_label}] 25 Hz Static Streaming (Target: {self.q_initial}, Duration: {duration_s}s)")
        print("================================================================================")

        if self.controller is None or not self.controller.is_connected:
            raise RuntimeError("Controller is not connected")
        if self.q_initial is None:
            raise RuntimeError("q_initial has not been established in J4-F1A-0")

        total_cycles = int(self.rate_hz * duration_s)
        period_s = 1.0 / self.rate_hz
        period_ns = int(period_s * 1e9)

        batch_durations_ms: list[float] = []
        target_ages_ms: list[float] = []
        positions_history: list[list[int]] = []
        currents_history: list[list[int]] = []
        faults_history: list[list[int]] = []
        deadline_miss_count = 0
        consecutive_comm_errors = 0
        total_comm_errors = 0
        feedback_completeness_count = 0
        last_feedback_timestamp_ns = time.perf_counter_ns()

        t_window_start_ns = time.perf_counter_ns()
        next_cycle_target_ns = t_window_start_ns
        writes_before = self.controller.write_count

        stop_triggered = False
        stop_reason = ""

        for cycle_idx in range(total_cycles):
            t_cycle_start_ns = time.perf_counter_ns()

            # Target creation (Latest-Value-Wins envelope representation)
            target_stamp_ns = t_cycle_start_ns
            target_steps = list(self.q_initial)

            # Safety Preflight right before dispatch
            for s in target_steps:
                if not (200 <= s <= 1800):
                    stop_triggered = True
                    stop_reason = f"Preflight FAIL: Target {s} outside envelope [200, 1800]"
                    break

            if stop_triggered:
                break

            target_age_ms = (time.perf_counter_ns() - target_stamp_ns) / 1e6
            target_ages_ms.append(target_age_ms)

            # 0x29 Sequential 6-axis write + feedback piggybacking
            t_write_start_ns = time.perf_counter_ns()
            try:
                cmd_result = self.controller.command_positions_with_metrics(target_steps)
                t_write_end_ns = time.perf_counter_ns()
                write_dur_ms = (t_write_end_ns - t_write_start_ns) / 1e6
                batch_durations_ms.append(write_dur_ms)
                consecutive_comm_errors = 0

                if write_dur_ms > self.target_period_ms:
                    deadline_miss_count += 1

                # Parse piggybacked feedback
                snapshot = cmd_result.to_state_snapshot()
                if len(snapshot.positions_steps) == 6:
                    feedback_completeness_count += 1
                    last_feedback_timestamp_ns = t_write_end_ns

                positions_history.append(list(snapshot.positions_steps))
                currents_history.append(list(snapshot.currents_mA))
                faults_history.append(list(snapshot.fault_codes))

                # Real-time STOP Condition Checks
                # 1. Fault code check
                for i, f in enumerate(snapshot.fault_codes):
                    if f != 0:
                        stop_triggered = True
                        stop_reason = f"Hardware Fault: Motor {i+1} returned fault_code=0x{f:02X}"
                        break

                # 2. Position deviation check against q_initial (max allowable deviation 5 steps = 0.125mm)
                for i, (p, p0) in enumerate(zip(snapshot.positions_steps, self.q_initial)):
                    if abs(p - p0) > 5:
                        stop_triggered = True
                        stop_reason = f"Position Deviation STOP: Motor {i+1} drifted by {abs(p - p0)} steps > 5 steps from q_initial"
                        break

                # 3. Peak current check (threshold 400 mA)
                for i, c in enumerate(snapshot.currents_mA):
                    if abs(c) > 400:
                        stop_triggered = True
                        stop_reason = f"Current Overload STOP: Motor {i+1} current {abs(c)} mA > 400 mA"
                        break

                if stop_triggered:
                    break

            except Exception as exc:
                total_comm_errors += 1
                consecutive_comm_errors += 1
                print(f"  [WARN] Cycle {cycle_idx+1}: UART exception (consecutive={consecutive_comm_errors}): {exc}")
                if consecutive_comm_errors >= 3:
                    stop_triggered = True
                    stop_reason = f"Communication Failure STOP: {consecutive_comm_errors} consecutive UART exceptions: {exc}"
                    break

            # Rate regulation
            next_cycle_target_ns += period_ns
            now_ns = time.perf_counter_ns()
            sleep_duration_s = (next_cycle_target_ns - now_ns) / 1e9
            if sleep_duration_s > 0.0005:
                time.sleep(sleep_duration_s)

        t_window_end_ns = time.perf_counter_ns()
        actual_window_duration_s = (t_window_end_ns - t_window_start_ns) / 1e9
        actual_writes = self.controller.write_count - writes_before

        # Analysis
        durations_arr = np.array(batch_durations_ms) if batch_durations_ms else np.array([0.0])
        pos_matrix = np.array(positions_history) if positions_history else np.zeros((1, 6))
        current_matrix = np.array(currents_history) if currents_history else np.zeros((1, 6))

        pos_min = [int(np.min(pos_matrix[:, i])) for i in range(6)]
        pos_max = [int(np.max(pos_matrix[:, i])) for i in range(6)]
        pos_max_offset = [int(max(abs(pos_min[i] - self.q_initial[i]), abs(pos_max[i] - self.q_initial[i]))) for i in range(6)]

        curr_min = [int(np.min(current_matrix[:, i])) for i in range(6)]
        curr_max = [int(np.max(current_matrix[:, i])) for i in range(6)]
        peak_current_mA = int(np.max(np.abs(current_matrix)))

        pass_criteria = bool(
            not stop_triggered
            and deadline_miss_count <= 1
            and total_comm_errors == 0
            and feedback_completeness_count == len(batch_durations_ms)
            and max(pos_max_offset) <= 2  # within deadband
            and peak_current_mA <= 150    # static holding current <= 150 mA
        )

        res = {
            "timestamp": datetime.now().isoformat(),
            "phase": phase_label,
            "duration_requested_s": duration_s,
            "duration_actual_s": actual_window_duration_s,
            "cycles_executed": len(batch_durations_ms),
            "cycles_expected": total_cycles,
            "write_count_increment": actual_writes,
            "batch_write_ms": {
                "mean": float(np.mean(durations_arr)),
                "p50": float(np.percentile(durations_arr, 50)),
                "p95": float(np.percentile(durations_arr, 95)),
                "p99": float(np.percentile(durations_arr, 99)),
                "max": float(np.max(durations_arr)),
                "min": float(np.min(durations_arr)),
            },
            "deadline_miss_count": deadline_miss_count,
            "target_age_ms": {
                "mean": float(np.mean(target_ages_ms)) if target_ages_ms else 0.0,
                "p95": float(np.percentile(target_ages_ms, 95)) if target_ages_ms else 0.0,
                "max": float(np.max(target_ages_ms)) if target_ages_ms else 0.0,
            },
            "feedback_completeness_ratio": feedback_completeness_count / max(1, len(batch_durations_ms)),
            "comm_errors_total": total_comm_errors,
            "consecutive_comm_errors_max": consecutive_comm_errors,
            "positions_min": pos_min,
            "positions_max": pos_max,
            "positions_max_offset_from_q0": pos_max_offset,
            "currents_min_mA": curr_min,
            "currents_max_mA": curr_max,
            "peak_current_mA": peak_current_mA,
            "stop_triggered": stop_triggered,
            "stop_reason": stop_reason,
            "pass": pass_criteria,
        }

        print(f"  Cycles:               {len(batch_durations_ms)} / {total_cycles} in {actual_window_duration_s:.2f}s")
        print(f"  Batch Write (ms):     Mean={res['batch_write_ms']['mean']:.2f}, P95={res['batch_write_ms']['p95']:.2f}, Max={res['batch_write_ms']['max']:.2f}")
        print(f"  Deadline Misses:      {deadline_miss_count} / {len(batch_durations_ms)} (Deadline={self.target_period_ms:.1f}ms)")
        print(f"  Feedback Complete:    {res['feedback_completeness_ratio']*100:.1f}%")
        print(f"  Max Position Offset:  {pos_max_offset} steps (all <= 2 step deadband)")
        print(f"  Peak Current:         {peak_current_mA} mA (threshold <= 150 mA)")
        print(f"  Comm Errors:          {total_comm_errors}")
        if stop_triggered:
            print(f"  [STOP TRIGGERED]      Reason: {stop_reason}")
        print(f"  Phase Status:         {'PASS' if pass_criteria else 'STOP/FAIL'}")

        return res


def main() -> None:
    parser = argparse.ArgumentParser(description="J4-F1A 25Hz Real Hardware Static Streaming Runner")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--phase", choices=["all", "0", "1", "2"], default="all")
    args = parser.parse_args()

    if args.real_hardware:
        if not (args.confirm_motion and args.validated_config):
            print("[ERROR] Real hardware execution requires --confirm-motion and --validated-config")
            sys.exit(1)
        mode = HardwareMode.ACTIVE
    else:
        mode = HardwareMode.MOCK

    print("================================================================================")
    print(" 6PUS J4-F1A: 25 Hz Real-Hardware Static Target Streaming Runner")
    print("================================================================================")
    print(f"Mode:          {mode.value.upper()}")
    print(f"Port:          {args.port}")
    print(f"Baudrate:      {args.baudrate}")
    print(f"Rate:          {args.rate:.1f} Hz (Period = {1000.0/args.rate:.1f} ms)")
    print(f"Phase Target:  {args.phase}")

    runner = J4F1ARunner(
        port=args.port,
        baudrate=args.baudrate,
        rate_hz=args.rate,
        hardware_mode=mode,
    )

    results: dict[str, Any] = {
        "timestamp": datetime.now().isoformat(),
        "stage": "J4-F1A",
        "hardware_mode": mode.value,
        "rate_hz": args.rate,
    }

    try:
        runner.connect()

        # Step 0: Initial state inspection (0 write)
        res0 = runner.run_f1a_0()
        results["J4-F1A-0"] = res0
        if not res0["pass"]:
            print("\n[STOP] J4-F1A-0 failed inspection. Aborting.")
            sys.exit(1)

        if args.phase in ["all", "1"]:
            # Step 1: 25 Hz streaming for 2.0s
            res1 = runner.run_streaming_window(duration_s=2.0, phase_label="J4-F1A-1")
            results["J4-F1A-1"] = res1
            if not res1["pass"]:
                print("\n[STOP] J4-F1A-1 failed or triggered stop. Aborting J4-F1A-2.")
                sys.exit(1)

        if args.phase in ["all", "2"]:
            # Step 2: 25 Hz streaming for 10.0s
            res2 = runner.run_streaming_window(duration_s=10.0, phase_label="J4-F1A-2")
            results["J4-F1A-2"] = res2

    finally:
        runner.close()

    # Save summary
    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"j4_f1a_real_static_streaming_{stamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"\n[SUMMARY] Saved full J4-F1A test results to: {log_file}")


if __name__ == "__main__":
    main()
