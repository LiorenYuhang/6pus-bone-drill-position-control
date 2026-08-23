#!/usr/bin/env python3
"""Stage J2-F0 — ROS 2 + UART 6-Motor Read-Only Real-Time Communication Performance Characterization Runner.

Adheres strictly to:
- /home/lyapunov/bone_drill/agent_docs/stages/J2/J2-F0_ROS2_READONLY_COMMUNICATION_PERFORMANCE_PLAN.md (Rev 2.1.1)
- /home/lyapunov/bone_drill/agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md

Safety Guarantees:
- hardware_mode = read_only (0 hardware write guaranteed at driver & bus levels)
- Fail-closed CLI: default is mock; real hardware requires explicit --real-hardware and explicit single group
- Real-time >3-step position drift monitoring on every cycle (triggers J2F0Stop)
- Pre/Post 0x30 safety snapshots around every F0-B and F0-D timed window
- Independent Auditor OS subprocess for C/D measurements
- High-resolution monotonic timing via time.perf_counter_ns()
- Strict sustainable-rate criteria evaluation
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from collections import Counter
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

# Add package paths
WORKSPACE_ROOT = Path("/home/lyapunov/bone_drill/bone_ws").resolve()
SRC_PATH = WORKSPACE_ROOT / "src"
AUDITOR_SCRIPT = SRC_PATH / "pus6_bringup" / "scripts" / "j2_f0_auditor_node.py"

for p in [
    SRC_PATH / "pus6_motor_driver",
    SRC_PATH / "pus6_kinematics",
    SRC_PATH / "pus6_control",
    SRC_PATH / "pus6_bringup",
]:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

import logging

# Mute high-frequency per-frame UART/protocol INFO logs during benchmark to avoid timing pollution
logging.getLogger("pus6_motor_driver.uart_bus").setLevel(logging.WARNING)
logging.getLogger("pus6_motor_driver.protocol").setLevel(logging.WARNING)
logging.getLogger("pus6_motor_driver.six_motor_controller").setLevel(logging.INFO)

from pus6_motor_driver.benchmark_stats import (
    CycleTimingStats,
    RosEndToEndLatencyStats,
    SweepPointStats,
    compute_cycle_timing_stats,
    compute_e2e_latency_stats,
)
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.protocol import CMD_WR_REGISTER
from pus6_motor_driver.safety import (
    HardwareMode,
    J2F0SafetyViolation,
    J2F0Stop,
    MOTOR_IDS,
    ReadOnlyModeError,
    SafetyViolation,
    steps_to_meters,
)
from pus6_motor_driver.six_motor_controller import (
    SixAxisPositionsSnapshot,
    SixAxisStateSnapshot,
    SixMotorController,
)


class J2F0BenchmarkHarness:
    """Benchmark execution harness managing mock/real controllers and logging."""

    def __init__(
        self,
        port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        baudrate: int = 921600,
        mock: bool = True,
        log_dir: str = "/home/lyapunov/bone_drill/bone_ws/log",
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.mock = mock
        self.hardware_mode = HardwareMode.MOCK if mock else HardwareMode.READ_ONLY
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Enforce ROS environment configuration
        if os.environ.get("ROS_DOMAIN_ID") != "42":
            os.environ["ROS_DOMAIN_ID"] = "42"
        if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
            os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    def _create_controller(self) -> SixMotorController:
        return SixMotorController(
            port=self.port,
            baudrate=self.baudrate,
            hardware_mode=self.hardware_mode,
        )

    def _verify_pre_post_0x30_snapshot(
        self, controller: SixMotorController, stage_label: str
    ) -> dict:
        """Capture and validate a full 0x30 safety snapshot before/after a benchmark window."""
        snap = controller.read_state_snapshot()
        fault_set = [f"0x{f:02X}" for f in set(snap.fault_codes)]
        max_curr = max(snap.currents_mA) if snap.currents_mA else 0

        # Baseline Admission Criteria
        if fault_set != ["0x00"]:
            raise J2F0SafetyViolation(
                f"{stage_label} baseline admission failed: non-zero fault {fault_set}"
            )
        if max_curr > 0:  # Authoritative strict 0 mA baseline admission
            raise J2F0SafetyViolation(
                f"{stage_label} baseline admission failed: non-zero current {max_curr}mA > 0mA"
            )

        return {
            "timestamp": datetime.now().isoformat(),
            "positions_steps": list(snap.positions_steps),
            "currents_mA": list(snap.currents_mA),
            "forces_g_raw": list(snap.forces_g_raw),
            "temperatures_C": list(snap.temperatures_C),
            "fault_set": fault_set,
        }

    def _check_realtime_position_drift(
        self, current_positions: list[int], baselines: list[int], stage_label: str
    ) -> None:
        """Real-time STOP monitoring: abort immediately if any axis drifts > 3 steps."""
        for mid_idx, (cur, base) in enumerate(zip(current_positions, baselines)):
            diff = abs(cur - base)
            if diff > 3:
                mid = MOTOR_IDS[mid_idx]
                raise J2F0Stop(
                    f"[{stage_label}] Real-time position drift STOP: Motor ID{mid} "
                    f"moved from baseline {base} to {cur} (diff={diff} > 3 steps)"
                )

    def _audit_bus_writes(self, controller: SixMotorController) -> dict:
        """Verify that total write count is 0 and CMD 0x32 TX count is 0."""
        ctrl_writes = controller.write_count
        bus = controller.bus

        tx_counts = {}
        tx_wr_count = 0
        if bus is not None:
            if hasattr(bus, "tx_cmd_counts"):
                tx_counts = {f"0x{k:02X}": v for k, v in bus.tx_cmd_counts.items()}
                tx_wr_count = bus.tx_cmd_counts.get(CMD_WR_REGISTER, 0)
            elif hasattr(bus, "events"):  # FakeBus
                tx_wr_count = len([e for e in bus.events if e[1] == CMD_WR_REGISTER])

        if ctrl_writes != 0 or tx_wr_count != 0:
            raise J2F0SafetyViolation(
                f"Authoritative 0-write audit FAILED! ctrl_writes={ctrl_writes}, CMD_0x32_tx={tx_wr_count}"
            )

        return {
            "controller_write_count": ctrl_writes,
            "cmd_0x32_tx_count": tx_wr_count,
            "tx_cmd_counts": tx_counts,
            "pass": True,
        }

    def _wait_for_auditor_match(
        self, driver_node: Pus6MotorDriverNode, executor: SingleThreadedExecutor,
        timeout_sec: float = 5.0,
    ) -> int:
        """Wait until DDS reports the benchmark publisher has a real subscriber."""
        publisher = driver_node.benchmark_stamp_pub
        if publisher is None:
            raise RuntimeError("F0 instrumentation publisher was not created")

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            count = publisher.get_subscription_count()
            if count > 0:
                return count
            executor.spin_once(timeout_sec=0.05)
        return publisher.get_subscription_count()

    # =========================================================================
    # Group A: F0-A (Full-State / No ROS)
    # =========================================================================
    def run_f0_a(self, duration_sec: float = 30.0) -> dict:
        print("\n" + "=" * 80)
        print(f"GROUP A: Stage F0-A (Full-State / No ROS Back-to-Back Baseline, {duration_sec}s)")
        print("=" * 80)

        controller = self._create_controller()
        controller.connect()
        try:
            self._audit_bus_writes(controller)

            # Baseline admission snapshot
            pre_snap = self._verify_pre_post_0x30_snapshot(controller, "Pre-Flight F0-A")
            baseline_positions = list(pre_snap["positions_steps"])

            durations_ns: list[int] = []
            span_id6_id1_ns: list[int] = []
            snapshots: list[SixAxisStateSnapshot] = []

            t_start_ns = time.perf_counter_ns()
            deadline_ns = t_start_ns + int(duration_sec * 1e9)

            while time.perf_counter_ns() < deadline_ns:
                s_t0 = time.perf_counter_ns()
                snap = controller.read_state_snapshot()
                s_t1 = time.perf_counter_ns()

                # Real-time safety checks on every cycle
                self._check_realtime_position_drift(snap.positions_steps, baseline_positions, "F0-A")
                for cur_mA in snap.currents_mA:
                    if cur_mA > 500:
                        raise J2F0SafetyViolation(f"F0-A runtime current {cur_mA}mA > 500mA emergency threshold")

                durations_ns.append(s_t1 - s_t0)
                if 1 in snap.per_axis_timestamps_ns and 6 in snap.per_axis_timestamps_ns:
                    span_id6_id1_ns.append(
                        snap.per_axis_timestamps_ns[6] - snap.per_axis_timestamps_ns[1]
                    )
                snapshots.append(snap)

            t_end_ns = time.perf_counter_ns()
            total_elapsed_sec = (t_end_ns - t_start_ns) / 1e9

            post_snap = self._verify_pre_post_0x30_snapshot(controller, "Post-Flight F0-A")
            write_audit = self._audit_bus_writes(controller)
            uart_telemetry = (
                controller.bus.get_telemetry()
                if hasattr(controller.bus, "get_telemetry")
                else {}
            )

            timing_stats = compute_cycle_timing_stats(
                durations_ns, total_elapsed_sec, theoretical_wire_time_ms=1.823
            )

            # Calculate ID1->ID6 Start Span and ID1->ID6 Completion Span
            start_spans_ms = [
                (s.per_axis_start_timestamps_ns[6] - s.per_axis_start_timestamps_ns[1]) / 1e6
                for s in snapshots
                if 1 in s.per_axis_start_timestamps_ns and 6 in s.per_axis_start_timestamps_ns
            ]
            completion_spans_ms = [
                (s.per_axis_end_timestamps_ns[6] - s.per_axis_end_timestamps_ns[1]) / 1e6
                for s in snapshots
                if 1 in s.per_axis_end_timestamps_ns and 6 in s.per_axis_end_timestamps_ns
            ]

            # Calculate single-axis transaction durations
            per_axis_durations_ms = {}
            for mid in MOTOR_IDS:
                axis_durs = [
                    (s.per_axis_end_timestamps_ns[mid] - s.per_axis_start_timestamps_ns[mid]) / 1e6
                    for s in snapshots
                    if mid in s.per_axis_end_timestamps_ns and mid in s.per_axis_start_timestamps_ns
                ]
                if axis_durs:
                    per_axis_durations_ms[f"motor_{mid}"] = {
                        "min_ms": float(np.min(axis_durs)),
                        "mean_ms": float(np.mean(axis_durs)),
                        "max_ms": float(np.max(axis_durs)),
                    }

            mean_start_span_ms = float(np.mean(start_spans_ms)) if start_spans_ms else 0.0
            mean_comp_span_ms = float(np.mean(completion_spans_ms)) if completion_spans_ms else 0.0
            pos_stats = self._calc_position_histograms([s.positions_steps for s in snapshots])

            result = {
                "subtest": "F0-A",
                "description": "Full-State / No ROS Back-to-Back Baseline",
                "duration_sec": total_elapsed_sec,
                "target_duration_sec": duration_sec,
                "sample_count": len(durations_ns),
                "timing_stats": asdict(timing_stats),
                "mean_id1_to_id6_start_span_ms": mean_start_span_ms,
                "mean_id1_to_id6_feedback_completion_span_ms": mean_comp_span_ms,
                "per_axis_transaction_durations_ms": per_axis_durations_ms,
                "pre_flight_snapshot": pre_snap,
                "post_flight_snapshot": post_snap,
                "position_stats": pos_stats,
                "write_audit": write_audit,
                "uart_telemetry": uart_telemetry,
                "pass": write_audit["pass"] and len(durations_ns) > 0,
            }

            print(f"F0-A Result: Cycles={len(durations_ns)}, Mean T_cycle={timing_stats.mean_ms:.2f}ms, Sustained Rate={timing_stats.sustained_rate_hz:.2f}Hz, Start Span={mean_start_span_ms:.2f}ms, Completion Span={mean_comp_span_ms:.2f}ms, Writes=0")
            return result
        finally:
            controller.close()

    # =========================================================================
    # Group B: F0-B (Position-Only / No ROS)
    # =========================================================================
    def run_f0_b(self, duration_sec: float = 30.0, f0_a_baseline_mean_ms: float | None = None) -> dict:
        print("\n" + "=" * 80)
        print(f"GROUP B: Stage F0-B (Position-Only / No ROS Back-to-Back Baseline, {duration_sec}s)")
        print("=" * 80)

        controller = self._create_controller()
        controller.connect()
        try:
            self._audit_bus_writes(controller)

            # Pre-Flight 0x30 safety snapshot
            pre_snap = self._verify_pre_post_0x30_snapshot(controller, "Pre-Flight F0-B")
            baseline_positions = list(pre_snap["positions_steps"])

            durations_ns: list[int] = []
            span_id6_id1_ns: list[int] = []
            snapshots: list[SixAxisPositionsSnapshot] = []

            t_start_ns = time.perf_counter_ns()
            deadline_ns = t_start_ns + int(duration_sec * 1e9)

            while time.perf_counter_ns() < deadline_ns:
                s_t0 = time.perf_counter_ns()
                snap = controller.read_positions_snapshot()
                s_t1 = time.perf_counter_ns()

                # Real-time position drift STOP monitoring on 0x31 feedback
                self._check_realtime_position_drift(snap.positions_steps, baseline_positions, "F0-B")

                durations_ns.append(s_t1 - s_t0)
                if 1 in snap.per_axis_timestamps_ns and 6 in snap.per_axis_timestamps_ns:
                    span_id6_id1_ns.append(
                        snap.per_axis_timestamps_ns[6] - snap.per_axis_timestamps_ns[1]
                    )
                snapshots.append(snap)

            t_end_ns = time.perf_counter_ns()
            total_elapsed_sec = (t_end_ns - t_start_ns) / 1e9

            # Post-Flight 0x30 safety snapshot
            post_snap = self._verify_pre_post_0x30_snapshot(controller, "Post-Flight F0-B")
            write_audit = self._audit_bus_writes(controller)
            uart_telemetry = (
                controller.bus.get_telemetry()
                if hasattr(controller.bus, "get_telemetry")
                else {}
            )

            timing_stats = compute_cycle_timing_stats(
                durations_ns, total_elapsed_sec, theoretical_wire_time_ms=1.237
            )

            # Calculate ID1->ID6 Start Span and ID1->ID6 Completion Span
            start_spans_ms = [
                (s.per_axis_start_timestamps_ns[6] - s.per_axis_start_timestamps_ns[1]) / 1e6
                for s in snapshots
                if 1 in s.per_axis_start_timestamps_ns and 6 in s.per_axis_start_timestamps_ns
            ]
            completion_spans_ms = [
                (s.per_axis_end_timestamps_ns[6] - s.per_axis_end_timestamps_ns[1]) / 1e6
                for s in snapshots
                if 1 in s.per_axis_end_timestamps_ns and 6 in s.per_axis_end_timestamps_ns
            ]

            # Calculate single-axis transaction durations
            per_axis_durations_ms = {}
            for mid in MOTOR_IDS:
                axis_durs = [
                    (s.per_axis_end_timestamps_ns[mid] - s.per_axis_start_timestamps_ns[mid]) / 1e6
                    for s in snapshots
                    if mid in s.per_axis_end_timestamps_ns and mid in s.per_axis_start_timestamps_ns
                ]
                if axis_durs:
                    per_axis_durations_ms[f"motor_{mid}"] = {
                        "min_ms": float(np.min(axis_durs)),
                        "mean_ms": float(np.mean(axis_durs)),
                        "max_ms": float(np.max(axis_durs)),
                    }

            mean_start_span_ms = float(np.mean(start_spans_ms)) if start_spans_ms else 0.0
            mean_comp_span_ms = float(np.mean(completion_spans_ms)) if completion_spans_ms else 0.0

            scan_time_reduction_ms = (
                (f0_a_baseline_mean_ms - timing_stats.mean_ms)
                if f0_a_baseline_mean_ms is not None
                else None
            )

            pos_stats = self._calc_position_histograms([s.positions_steps for s in snapshots])

            result = {
                "subtest": "F0-B",
                "description": "Position-Only / No ROS Back-to-Back Baseline",
                "duration_sec": total_elapsed_sec,
                "target_duration_sec": duration_sec,
                "sample_count": len(durations_ns),
                "timing_stats": asdict(timing_stats),
                "mean_id1_to_id6_start_span_ms": mean_start_span_ms,
                "mean_id1_to_id6_feedback_completion_span_ms": mean_comp_span_ms,
                "per_axis_transaction_durations_ms": per_axis_durations_ms,
                "empirical_end_to_end_scan_time_reduction_ms": scan_time_reduction_ms,
                "theoretical_wire_time_reduction_ms": 0.586,
                "pre_flight_snapshot": pre_snap,
                "post_flight_snapshot": post_snap,
                "position_stats": pos_stats,
                "write_audit": write_audit,
                "uart_telemetry": uart_telemetry,
                "pass": write_audit["pass"] and len(durations_ns) > 0,
            }

            print(f"F0-B Result: Cycles={len(durations_ns)}, Mean T_cycle={timing_stats.mean_ms:.2f}ms, Sustained Rate={timing_stats.sustained_rate_hz:.2f}Hz, Start Span={mean_start_span_ms:.2f}ms, Completion Span={mean_comp_span_ms:.2f}ms, Writes=0")
            return result
        finally:
            controller.close()

    # =========================================================================
    # Group C: F0-C1 & F0-C2 (Full-State ROS Pipeline with Independent Auditor)
    # =========================================================================
    def run_f0_c(
        self,
        f0_a_baseline_hz: float = 55.0,
        point_duration_sec: float = 30.0,
    ) -> dict:
        print("\n" + "=" * 80)
        print("GROUP C: Stage F0-C (Full-State / ROS Publish Pipeline)")
        print("=" * 80)

        # Baseline-adaptive sweep frequencies anchored on F0-A
        sweep_frequencies = self._generate_adaptive_frequencies(f0_a_baseline_hz, cap_hz=50.0)

        # 1. Subtest F0-C1: 50Hz Fixed Stability Validation
        print(f"\n--- Subtest F0-C1: 50Hz Fixed Stability Validation ({point_duration_sec}s) ---")
        c1_result = self._run_ros_timed_window_with_auditor(
            target_hz=50.0,
            duration_sec=point_duration_sec,
            read_mode="full_state",
            publish_raw=True,
            theoretical_wire_ms=1.823,
            stage_label="F0-C1",
        )

        # 2. Subtest F0-C2: Adaptive Timer Sweep
        print("\n--- Subtest F0-C2: Adaptive Timer Rate Sweep ---")
        sweep_results = []
        sustainable_rate = 0.0
        for freq in sweep_frequencies:
            print(f"Evaluating Sweep Frequency: {freq:.1f} Hz...")
            sp = self._run_ros_timed_window_with_auditor(
                target_hz=freq,
                duration_sec=point_duration_sec,
                read_mode="full_state",
                publish_raw=True,
                theoretical_wire_ms=1.823,
                stage_label=f"F0-C2@{freq:.1f}Hz",
            )
            sweep_results.append(sp)
            if sp.get("is_sustainable") and freq > sustainable_rate:
                sustainable_rate = freq

        return {
            "group": "Group C",
            "f0_c1_stability_50hz": c1_result,
            "f0_c2_sweep_frequencies": sweep_frequencies,
            "f0_c2_sweep_results": sweep_results,
            "maximum_sustainable_ros_rate_hz": sustainable_rate,
            "pass": c1_result.get("is_sustainable", False) or sustainable_rate > 0,
        }

    # =========================================================================
    # Group D: F0-D1 & F0-D2 (Position-Only ROS Pipeline with Per-Window Snapshots)
    # =========================================================================
    def run_f0_d(
        self,
        f0_b_baseline_hz: float = 65.0,
        point_duration_sec: float = 30.0,
    ) -> dict:
        print("\n" + "=" * 80)
        print("GROUP D: Stage F0-D (Position-Only / ROS Publish Pipeline)")
        print("=" * 80)

        sweep_frequencies = self._generate_adaptive_frequencies(f0_b_baseline_hz, cap_hz=60.0)

        # 1. Subtest F0-D1: 50Hz Fixed Stability Validation (Wrapped with Pre/Post 0x30 snapshots)
        print(f"\n--- Subtest F0-D1: 50Hz Fixed Stability Validation ({point_duration_sec}s) ---")
        d1_result = self._run_ros_timed_window_with_auditor(
            target_hz=50.0,
            duration_sec=point_duration_sec,
            read_mode="position_only",
            publish_raw=False,
            theoretical_wire_ms=1.237,
            stage_label="F0-D1",
            wrap_with_0x30_snapshots=True,
        )

        # 2. Subtest F0-D2: Adaptive Timer Sweep (Each frequency window wrapped with Pre/Post 0x30 snapshots)
        print("\n--- Subtest F0-D2: Adaptive Timer Rate Sweep ---")
        sweep_results = []
        sustainable_rate = 0.0
        for freq in sweep_frequencies:
            print(f"Evaluating Sweep Frequency: {freq:.1f} Hz...")
            sp = self._run_ros_timed_window_with_auditor(
                target_hz=freq,
                duration_sec=point_duration_sec,
                read_mode="position_only",
                publish_raw=False,
                theoretical_wire_ms=1.237,
                stage_label=f"F0-D2@{freq:.1f}Hz",
                wrap_with_0x30_snapshots=True,
            )
            sweep_results.append(sp)
            if sp.get("is_sustainable") and freq > sustainable_rate:
                sustainable_rate = freq

        return {
            "group": "Group D",
            "f0_d1_stability_50hz": d1_result,
            "f0_d2_sweep_frequencies": sweep_frequencies,
            "f0_d2_sweep_results": sweep_results,
            "maximum_sustainable_ros_rate_hz": sustainable_rate,
            "pass": d1_result.get("is_sustainable", False) or sustainable_rate > 0,
        }

    # =========================================================================
    # ROS 2 Timed Window Execution with Independent Auditor OS Process
    # =========================================================================
    def _run_ros_timed_window_with_auditor(
        self,
        target_hz: float,
        duration_sec: float,
        read_mode: str,
        publish_raw: bool,
        theoretical_wire_ms: float,
        stage_label: str,
        wrap_with_0x30_snapshots: bool = False,
    ) -> dict:
        target_period_ms = 1000.0 / target_hz
        pre_snap = None
        post_snap = None

        if wrap_with_0x30_snapshots:
            temp_ctrl = self._create_controller()
            temp_ctrl.connect()
            pre_snap = self._verify_pre_post_0x30_snapshot(temp_ctrl, f"Pre-Flight {stage_label}")
            temp_ctrl.close()

        # Temporary path for Auditor process metrics output
        tmp_auditor_json = self.log_dir / f"tmp_auditor_{int(time.time()*1000)}.json"

        # Spawn Independent Auditor in a separate OS process
        auditor_cmd = [
            sys.executable,
            str(AUDITOR_SCRIPT),
            "--output-json",
            str(tmp_auditor_json),
        ]
        auditor_proc = subprocess.Popen(
            auditor_cmd,
            env=dict(os.environ, ROS_DOMAIN_ID="42", RMW_IMPLEMENTATION="rmw_fastrtps_cpp"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.3)  # Let Auditor create its subscriptions before discovery wait.

        if not rclpy.ok():
            rclpy.init()
        try:
            overrides = [
                Parameter("hardware_mode", Parameter.Type.STRING, self.hardware_mode.value),
                Parameter("port", Parameter.Type.STRING, self.port),
                Parameter("baudrate", Parameter.Type.INTEGER, self.baudrate),
                Parameter("auto_connect", Parameter.Type.BOOL, True),
                Parameter("auto_configure", Parameter.Type.BOOL, False),
                Parameter("read_mode", Parameter.Type.STRING, read_mode),
                Parameter("publish_raw_diagnostics", Parameter.Type.BOOL, publish_raw),
                Parameter("enable_j2_f0_instrumentation", Parameter.Type.BOOL, True),
                Parameter("poll_rate_hz", Parameter.Type.DOUBLE, target_hz),
            ]

            driver_node = Pus6MotorDriverNode(parameter_overrides=overrides)
            executor = SingleThreadedExecutor()
            executor.add_node(driver_node)

            auditor_subscription_count = self._wait_for_auditor_match(driver_node, executor)
            if auditor_subscription_count < 1:
                raise RuntimeError(
                    f"Auditor subscription did not match {driver_node.benchmark_stamp_pub.topic_name} "
                    "within 5.0s"
                )

            cycle_durations_ns: list[int] = []
            baseline_positions: list[int] | None = None

            t_start_ns = time.perf_counter_ns()
            deadline_ns = t_start_ns + int(duration_sec * 1e9)

            last_snap_end_ns = 0
            while time.perf_counter_ns() < deadline_ns:
                executor.spin_once(timeout_sec=0.002)

                # Check if driver encountered a fatal safety violation
                if driver_node.runtime_error is not None:
                    raise driver_node.runtime_error

                # Collect completed snapshots
                snap = getattr(driver_node, "_last_state_snapshot", None)
                if snap is None:
                    snap = getattr(driver_node, "_last_positions_snapshot", None)

                if snap is not None and snap.timestamp_end_ns != last_snap_end_ns:
                    last_snap_end_ns = snap.timestamp_end_ns
                    dur_ns = snap.timestamp_end_ns - snap.timestamp_start_ns
                    if dur_ns > 0:
                        cycle_durations_ns.append(dur_ns)

                    # Real-time position drift check
                    if baseline_positions is None:
                        baseline_positions = list(snap.positions_steps)
                    else:
                        self._check_realtime_position_drift(
                            snap.positions_steps, baseline_positions, stage_label
                        )

            t_end_ns = time.perf_counter_ns()
            driver_pub_stamps_ns = [
                stamp for stamp in driver_node.benchmark_publish_timestamps_ns
                if t_start_ns <= stamp <= t_end_ns
            ]

            # Capture transport telemetry before destroying the node/bus.
            uart_telemetry = (
                driver_node.controller.bus.get_telemetry()
                if hasattr(driver_node.controller.bus, "get_telemetry")
                else {}
            )
            write_audit = self._audit_bus_writes(driver_node.controller)

            driver_node.destroy_node()
            executor.shutdown()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

            # Terminate Auditor OS process cleanly
            if auditor_proc.poll() is None:
                auditor_proc.send_signal(signal.SIGINT)
                try:
                    auditor_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    auditor_proc.kill()

        if wrap_with_0x30_snapshots:
            temp_ctrl = self._create_controller()
            temp_ctrl.connect()
            post_snap = self._verify_pre_post_0x30_snapshot(temp_ctrl, f"Post-Flight {stage_label}")
            temp_ctrl.close()

        # Load Auditor metrics from independent process
        auditor_records = []
        auditor_rx_count = 0
        auditor_spin_heartbeat_count = 0
        auditor_stdout = ""
        auditor_stderr = ""
        if tmp_auditor_json.exists():
            try:
                with open(tmp_auditor_json, "r", encoding="utf-8") as f:
                    aud_data = json.load(f)
                    raw_auditor_records = aud_data.get("benchmark_records", [])
                    auditor_spin_heartbeat_count = aud_data.get("spin_heartbeat_count", 0)
                auditor_records = [
                    record for record in raw_auditor_records
                    if "t_pub_start_ns" in record
                    and t_start_ns <= record["t_pub_start_ns"] <= t_end_ns
                ]
                auditor_rx_count = len(auditor_records)
                tmp_auditor_json.unlink(missing_ok=True)
            except Exception:
                pass
        try:
            auditor_stdout, auditor_stderr = auditor_proc.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            auditor_proc.kill()
            auditor_stdout, auditor_stderr = auditor_proc.communicate()

        # Parse timings and compute separate durations
        t_pub_starts_ns = driver_pub_stamps_ns
        e2e_latencies_ns = [r["e2e_latency_ns"] for r in auditor_records if "e2e_latency_ns" in r]
        pub_section_durations_ns = [r["publish_section_ns"] for r in auditor_records if "publish_section_ns" in r]
        cb_durations_ns = [r["callback_duration_ns"] for r in auditor_records if "callback_duration_ns" in r]

        # Calculate pure UART cycle statistics
        cycle_stats = compute_cycle_timing_stats(cycle_durations_ns, duration_sec, theoretical_wire_ms)
        e2e_stats = compute_e2e_latency_stats(e2e_latencies_ns)
        pub_overhead_stats = compute_e2e_latency_stats(pub_section_durations_ns)
        cb_timing_stats = compute_e2e_latency_stats(cb_durations_ns)

        # Calculate ROS publish intervals strictly from continuous t_pub_start_ns
        pub_intervals_ns = list(np.diff(t_pub_starts_ns)) if len(t_pub_starts_ns) > 1 else []
        pub_cadence_stats = compute_e2e_latency_stats(pub_intervals_ns) if pub_intervals_ns else None

        pub_count = len(driver_pub_stamps_ns)
        delivery_ratio = (auditor_rx_count / pub_count) if pub_count > 0 else 0.0

        # Calculate deadline misses and on-time ratio strictly based on ROS publish intervals
        deadline_miss_count = 0
        max_consecutive_miss = 0
        current_consecutive_miss = 0
        on_time_count = 0
        total_intervals = len(pub_intervals_ns)

        if total_intervals > 0:
            pub_intervals_ms = np.array(pub_intervals_ns, dtype=np.float64) / 1e6
            for interval in pub_intervals_ms:
                # On-time criteria: |interval - target_period_ms| < 2.0 ms
                if abs(interval - target_period_ms) < 2.0:
                    on_time_count += 1

                # Deadline miss criteria: interval >= target_period_ms
                if interval >= target_period_ms:
                    deadline_miss_count += 1
                    current_consecutive_miss += 1
                    if current_consecutive_miss > max_consecutive_miss:
                        max_consecutive_miss = current_consecutive_miss
                else:
                    current_consecutive_miss = 0
            on_time_ratio = float(on_time_count / total_intervals)
            deadline_miss_ratio = float(deadline_miss_count / total_intervals)
        else:
            on_time_ratio = 0.0
            deadline_miss_ratio = 0.0

        # Sustainable rate evaluation
        fail_reasons: list[str] = []
        if on_time_ratio < 0.98:
            fail_reasons.append(f"On-time ratio {on_time_ratio*100:.1f}% < 98.0%")
        if total_intervals == 0:
            fail_reasons.append("No publish interval samples")
        if delivery_ratio < 0.99:
            fail_reasons.append(f"Auditor delivery ratio {delivery_ratio*100:.1f}% < 99.0%")
        if deadline_miss_ratio > 0.02:
            fail_reasons.append(f"Deadline miss ratio {deadline_miss_ratio*100:.1f}% > 2.0%")
        if max_consecutive_miss > 2:
            fail_reasons.append(f"Max consecutive deadline miss {max_consecutive_miss} > 2")
        if cycle_stats.p95_ms >= target_period_ms:
            fail_reasons.append(f"T_cycle P95 ({cycle_stats.p95_ms:.2f}ms) >= Target Period ({target_period_ms:.2f}ms)")
        if not write_audit["pass"]:
            fail_reasons.append("Write audit violation: write_count > 0")

        is_sustainable = len(fail_reasons) == 0

        return {
            "stage_label": stage_label,
            "target_frequency_hz": target_hz,
            "target_period_ms": target_period_ms,
            "actual_driver_pub_rate_hz": float(pub_count / duration_sec),
            "driver_publish_timestamp_count": pub_count,
            "auditor_subscription_count": auditor_subscription_count,
            "auditor_stdout": auditor_stdout[-2000:],
            "auditor_stderr": auditor_stderr[-4000:],
            "auditor_receive_rate_hz": float(auditor_rx_count / duration_sec),
            "auditor_spin_heartbeat_count": auditor_spin_heartbeat_count,
            "auditor_delivery_ratio": float(delivery_ratio),
            "on_time_ratio": float(on_time_ratio),
            "deadline_miss_count": deadline_miss_count,
            "deadline_miss_ratio": float(deadline_miss_ratio),
            "max_consecutive_deadline_miss": max_consecutive_miss,
            "t_cycle_stats": asdict(cycle_stats),
            "ros_publish_interval_stats": asdict(pub_cadence_stats) if pub_cadence_stats else None,
            "driver_ros_publish_overhead_stats": asdict(pub_overhead_stats) if pub_overhead_stats else None,
            "total_callback_duration_stats": asdict(cb_timing_stats) if cb_timing_stats else None,
            "ros_e2e_transport_latency_stats": asdict(e2e_stats) if e2e_stats else None,
            "uart_telemetry": uart_telemetry,
            "pre_flight_snapshot": pre_snap,
            "post_flight_snapshot": post_snap,
            "write_audit": write_audit,
            "is_sustainable": is_sustainable,
            "fail_reasons": fail_reasons,
        }

    def _generate_adaptive_frequencies(self, baseline_b2b_hz: float, cap_hz: float) -> list[float]:
        """Generate genuinely adaptive sweep frequencies from measured baseline."""
        target_max = min(baseline_b2b_hz, cap_hz)
        ratios = [0.5, 0.7, 0.85, 0.95, 1.0]
        freqs = [round(r * target_max, 1) for r in ratios if r * target_max >= 10.0]
        return sorted(list(set(freqs)))

    def _calc_position_histograms(self, positions_lists: list[list[int]]) -> dict:
        stats = {}
        if not positions_lists:
            return stats
        for idx, mid in enumerate(MOTOR_IDS):
            vals = [p[idx] for p in positions_lists]
            hist = dict(Counter(vals))
            stats[f"motor_{mid}"] = {
                "min": int(np.min(vals)),
                "max": int(np.max(vals)),
                "span": int(np.max(vals) - np.min(vals)),
                "mean": float(np.mean(vals)),
                "histogram": {str(k): v for k, v in sorted(hist.items())},
            }
        return stats


def main():
    parser = argparse.ArgumentParser(
        description="Stage J2-F0 Benchmark Runner (Strict Fail-Closed CLI)"
    )
    parser.add_argument(
        "--real-hardware",
        action="store_true",
        default=False,
        help="Explicitly enable real hardware access. If not provided, defaults safely to MOCK mode.",
    )
    parser.add_argument(
        "--group",
        choices=["A", "B", "C", "D", "ALL"],
        required=True,
        help="Experiment group to run (A, B, C, D, or ALL). Note: ALL is prohibited on real hardware.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Duration for each timed window in seconds (default: 30.0s)",
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="UART port path",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=921600,
        help="UART baudrate (default: 921600)",
    )
    args = parser.parse_args()

    # Fail-Closed CLI validation
    if args.real_hardware and args.group == "ALL":
        print("\n" + "!" * 80)
        print("FAIL-CLOSED REJECTION: Running '--group ALL' on real hardware is strictly prohibited!")
        print("You must specify an individual authorized group (e.g. '--group A') for real hardware tests.")
        print("!" * 80 + "\n")
        sys.exit(1)

    is_mock = not args.real_hardware
    print(f"J2-F0 Mode: {'MOCK SIMULATION (Safe)' if is_mock else 'REAL HARDWARE (Authorized)'}, Group: {args.group}")

    harness = J2F0BenchmarkHarness(
        port=args.port,
        baudrate=args.baudrate,
        mock=is_mock,
    )

    results = {
        "execution_mode": "mock" if is_mock else "real_hardware",
        "timestamp": datetime.now().isoformat(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
    }
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    f0_a_mean_ms = None
    f0_a_hz = 55.0
    f0_b_hz = 65.0

    if args.group in ["A", "ALL"]:
        res_a = harness.run_f0_a(duration_sec=args.duration)
        results["Group_A"] = res_a
        f0_a_mean_ms = res_a["timing_stats"]["mean_ms"]
        f0_a_hz = res_a["timing_stats"]["sustained_rate_hz"]

    if args.group in ["B", "ALL"]:
        res_b = harness.run_f0_b(duration_sec=args.duration, f0_a_baseline_mean_ms=f0_a_mean_ms)
        results["Group_B"] = res_b
        f0_b_hz = res_b["timing_stats"]["sustained_rate_hz"]

    if args.group in ["C", "ALL"]:
        res_c = harness.run_f0_c(f0_a_baseline_hz=f0_a_hz, point_duration_sec=args.duration)
        results["Group_C"] = res_c

    if args.group in ["D", "ALL"]:
        res_d = harness.run_f0_d(f0_b_baseline_hz=f0_b_hz, point_duration_sec=args.duration)
        results["Group_D"] = res_d

    # Save structured JSON report
    prefix = "mock" if is_mock else "real"
    log_file = harness.log_dir / f"j2f0_{prefix}_{args.group.lower()}_{timestamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\nSaved J2-F0 benchmark report: {log_file}")


if __name__ == "__main__":
    main()
