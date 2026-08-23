#!/usr/bin/env python3
"""Stage J2-A — ROS 2 Formal Driver 6-Motor Read-Only Real Hardware Verification Runner.

Strictly adheres to:
- /home/lyapunov/bone_drill/agent_docs/stages/J2/J2-A_ROS2_READONLY_REAL_HARDWARE_PLAN.md
- /home/lyapunov/bone_drill/agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md
- /home/lyapunov/bone_drill/up/motor/uart_multi/AGENTS.md

Enforces:
- hardware_mode = read_only
- 60 seconds continuous acquisition
- Entire lifecycle hardware write_count = 0 (startup, runtime, shutdown)
"""

from __future__ import annotations

import json
import os
import sys
import time
from collections import Counter
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

# Add package paths
WORKSPACE_ROOT = Path("/home/lyapunov/bone_drill/bone_ws").resolve()
SRC_PATH = WORKSPACE_ROOT / "src"
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
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import HardwareMode, MOTOR_IDS, steps_to_meters


class J2AAuditorNode(Node):
    """Auditor node subscribing to all ROS 2 telemetry topics published by Pus6MotorDriverNode."""

    def __init__(self) -> None:
        super().__init__("j2a_auditor_node")

        self.joint_states_msgs: list[dict] = []
        self.raw_steps_msgs: list[dict] = []
        self.raw_currents_msgs: list[dict] = []
        self.raw_forces_msgs: list[dict] = []
        self.raw_temps_msgs: list[dict] = []
        self.raw_faults_msgs: list[dict] = []

        self.sub_js = self.create_subscription(
            JointState, "/pus6/joint_states", self._on_joint_state, 50
        )
        self.sub_steps = self.create_subscription(
            Int32MultiArray, "/pus6/joint_steps_raw", self._on_steps, 50
        )
        self.sub_currents = self.create_subscription(
            Int32MultiArray, "/pus6/joint_currents_raw", self._on_currents, 50
        )
        self.sub_forces = self.create_subscription(
            Int32MultiArray, "/pus6/joint_forces_raw", self._on_forces, 50
        )
        self.sub_temps = self.create_subscription(
            Int32MultiArray, "/pus6/joint_temperatures_raw", self._on_temps, 50
        )
        self.sub_faults = self.create_subscription(
            Int32MultiArray, "/pus6/joint_faults_raw", self._on_faults, 50
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_states_msgs.append({
            "t": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        })

    def _on_steps(self, msg: Int32MultiArray) -> None:
        self.raw_steps_msgs.append({"t": time.time(), "data": list(msg.data)})

    def _on_currents(self, msg: Int32MultiArray) -> None:
        self.raw_currents_msgs.append({"t": time.time(), "data": list(msg.data)})

    def _on_forces(self, msg: Int32MultiArray) -> None:
        self.raw_forces_msgs.append({"t": time.time(), "data": list(msg.data)})

    def _on_temps(self, msg: Int32MultiArray) -> None:
        self.raw_temps_msgs.append({"t": time.time(), "data": list(msg.data)})

    def _on_faults(self, msg: Int32MultiArray) -> None:
        self.raw_faults_msgs.append({"t": time.time(), "data": list(msg.data)})


def run_j2a_verification(
    duration_sec: float = 60.0,
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    baudrate: int = 921600,
) -> int:
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = Path("/home/lyapunov/bone_drill/bone_ws/log")
    log_dir.mkdir(parents=True, exist_ok=True)
    json_path = log_dir / f"j2a_readonly_{timestamp_str}.json"
    log_path = log_dir / f"j2a_readonly_{timestamp_str}.log"

    print("=" * 80)
    print("STAGE J2-A — ROS 2 Formal Driver 6-Motor Read-Only Real Hardware Verification")
    print(f"Time: {datetime.now().isoformat()}")
    print(f"Target Duration: {duration_sec} s")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Mode: READ_ONLY (0 writes enforced)")
    print("=" * 80)

    rclpy.init()

    # Create driver node with explicit hardware_mode=read_only and real port
    from rclpy.parameter import Parameter
    overrides = [
        Parameter("hardware_mode", Parameter.Type.STRING, "read_only"),
        Parameter("port", Parameter.Type.STRING, port),
        Parameter("baudrate", Parameter.Type.INTEGER, baudrate),
        Parameter("auto_connect", Parameter.Type.BOOL, False),  # We will connect explicitly
    ]
    driver_node = Pus6MotorDriverNode(parameter_overrides=overrides)
    assert driver_node.hardware_mode == HardwareMode.READ_ONLY, f"hardware_mode={driver_node.hardware_mode}"
    assert driver_node.controller._hardware_mode == HardwareMode.READ_ONLY

    # Check 1: Startup Write Audit before connect
    startup_writes_pre = driver_node.controller.write_count
    assert startup_writes_pre == 0, f"FATAL: write_count={startup_writes_pre} before connect"

    print("Connecting to real hardware in READ_ONLY mode...")
    driver_node.controller.connect()

    # Check 2: Startup Write Audit after connect
    startup_writes = driver_node.controller.write_count
    if startup_writes != 0:
        print(f"FATAL STOP: Startup produced {startup_writes} writes! Must be 0.")
        driver_node.destroy_node()
        rclpy.shutdown()
        return 1

    auditor_node = J2AAuditorNode()

    executor = SingleThreadedExecutor()
    executor.add_node(driver_node)
    executor.add_node(auditor_node)

    # Telemetry storage
    snapshots_history = []
    cycle_times = []
    span_id6_id1_list = []
    stop_reason: str | None = None

    t_start = time.time()
    next_poll_time = t_start
    poll_interval = 0.020  # 50 Hz requested

    print(f"Starting 60s continuous read-only telemetry loop at {time.strftime('%H:%M:%S')}...")

    baseline_pos: list[int] | None = None
    drift_history: dict[int, list[int]] = {i: [] for i in MOTOR_IDS}

    try:
        while time.time() - t_start < duration_sec:
            now = time.time()
            # Spin ROS executor to handle timers and pub/sub
            executor.spin_once(timeout_sec=0.005)

            # Check driver controller's last state snapshot
            # Driver timer polls automatically; we inspect snapshots
            if hasattr(driver_node, "_last_state_snapshot") and driver_node._last_state_snapshot is not None:
                snap = driver_node._last_state_snapshot
                # Check if this snapshot is new
                if not snapshots_history or snap.timestamp_end != snapshots_history[-1]["timestamp_end"]:
                    snap_dict = {
                        "timestamp_start": snap.timestamp_start,
                        "timestamp_end": snap.timestamp_end,
                        "t_cycle": snap.timestamp_end - snap.timestamp_start,
                        "per_axis_timestamps": snap.per_axis_timestamps,
                        "positions_steps": list(snap.positions_steps),
                        "positions_meters": list(snap.positions_meters),
                        "currents_mA": list(snap.currents_mA),
                        "forces_g_raw": list(snap.forces_g_raw),
                        "temperatures_C": list(snap.temperatures_C),
                        "fault_codes": list(snap.fault_codes),
                    }
                    snapshots_history.append(snap_dict)
                    cycle_times.append(snap_dict["t_cycle"])
                    if 1 in snap.per_axis_timestamps and 6 in snap.per_axis_timestamps:
                        span_id6_id1_list.append(
                            snap.per_axis_timestamps[6] - snap.per_axis_timestamps[1]
                        )

                    # Baseline check
                    if baseline_pos is None:
                        baseline_pos = list(snap.positions_steps)
                        print(f"Initial 6-motor positions (steps): {baseline_pos}")

                    # Real-time Safety Guards (STOP conditions)
                    # 1. Write count must remain strictly 0
                    if driver_node.controller.write_count != 0:
                        stop_reason = f"Runtime write detected! write_count={driver_node.controller.write_count}"
                        print(f"CRITICAL SAFETY VIOLATION: {stop_reason}")
                        break

                    # 2. Fault codes
                    for idx, m_id in enumerate(MOTOR_IDS):
                        fc = snap.fault_codes[idx]
                        if fc != 0:
                            stop_reason = f"Motor ID{m_id} fault non-zero (0x{fc:02X})"
                            print(f"STOP CONDITION: {stop_reason}")
                            break

                    if stop_reason:
                        break

                    # 3. Current threshold (< 500mA static)
                    for idx, m_id in enumerate(MOTOR_IDS):
                        curr = snap.currents_mA[idx]
                        if curr > 500:
                            stop_reason = f"Motor ID{m_id} overcurrent in static state: {curr} mA"
                            print(f"STOP CONDITION: {stop_reason}")
                            break

                    if stop_reason:
                        break

                    # 4. Position drift check (record position changes)
                    for idx, m_id in enumerate(MOTOR_IDS):
                        p = snap.positions_steps[idx]
                        drift_history[m_id].append(p)
                        # Check monotonic drift over last 100 samples
                        if len(drift_history[m_id]) >= 100:
                            recent = drift_history[m_id][-100:]
                            if (recent[-1] - recent[0]) > 4 or (recent[0] - recent[-1]) > 4:
                                stop_reason = f"Motor ID{m_id} continuous monotonic drift detected: {recent[0]} -> {recent[-1]}"
                                print(f"STOP CONDITION: {stop_reason}")
                                break

                    if stop_reason:
                        break

                    # Periodic console progress report (every 10s)
                    elapsed = time.time() - t_start
                    if len(snapshots_history) % 250 == 0:
                        print(
                            f"[{elapsed:5.1f}s / {duration_sec:.0f}s] Cycles: {len(snapshots_history)}, "
                            f"Positions: {snap.positions_steps}, "
                            f"Mean T_cycle: {np.mean(cycle_times[-250:]) * 1000:.1f}ms, "
                            f"Writes: {driver_node.controller.write_count}"
                        )

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nOperator manually stopped verification.")
        stop_reason = "Manual operator interrupt"

    t_end = time.time()
    runtime_duration = t_end - t_start
    runtime_writes = driver_node.controller.write_count

    print("-" * 80)
    print(f"Acquisition loop finished. Actual duration: {runtime_duration:.2f} s")
    print(f"Total cycles collected: {len(snapshots_history)}")
    print(f"Runtime writes: {runtime_writes}")

    # Destroy node and check shutdown writes
    print("Shutting down nodes...")
    auditor_node.destroy_node()
    driver_node.destroy_node()
    rclpy.shutdown()

    shutdown_writes = driver_node.controller.write_count
    total_writes = driver_node.controller.write_count

    print(f"Shutdown writes: {shutdown_writes - runtime_writes}")
    print(f"Total hardware writes across full lifecycle: {total_writes}")

    # =========================================================================
    # Data Processing & Metrics Calculation
    # =========================================================================
    print("Processing metrics...")

    # A. Timing Stats
    cycle_times_arr = np.array(cycle_times) if cycle_times else np.array([0.0])
    span_arr = np.array(span_id6_id1_list) if span_id6_id1_list else np.array([0.0])

    t_cycle_stats = {
        "min_ms": float(np.min(cycle_times_arr) * 1000),
        "mean_ms": float(np.mean(cycle_times_arr) * 1000),
        "median_p50_ms": float(np.median(cycle_times_arr) * 1000),
        "p95_ms": float(np.percentile(cycle_times_arr, 95) * 1000),
        "p99_ms": float(np.percentile(cycle_times_arr, 99) * 1000),
        "max_ms": float(np.max(cycle_times_arr) * 1000),
        "std_ms": float(np.std(cycle_times_arr) * 1000),
        "effective_acquisition_freq_hz": float(1.0 / np.mean(cycle_times_arr)) if np.mean(cycle_times_arr) > 0 else 0.0,
        "mean_span_id6_id1_ms": float(np.mean(span_arr) * 1000),
    }

    # B. ROS 2 Topic Stats
    js_count = len(auditor_node.joint_states_msgs)
    js_effective_freq = js_count / runtime_duration if runtime_duration > 0 else 0.0

    # SI conversion verification
    si_conversion_passed = True
    si_conversion_errors = []
    for snap in snapshots_history:
        for idx, m_id in enumerate(MOTOR_IDS):
            step_val = snap["positions_steps"][idx]
            meter_val = snap["positions_meters"][idx]
            expected_m = step_val * 0.000025
            if abs(meter_val - expected_m) > 1e-9:
                si_conversion_passed = False
                si_conversion_errors.append(f"ID{m_id}: step={step_val}, m={meter_val}, expected={expected_m}")

    # Check JointState effort/velocity are empty
    js_effort_empty = all(len(m["effort"]) == 0 for m in auditor_node.joint_states_msgs)
    js_velocity_empty = all(len(m["velocity"]) == 0 for m in auditor_node.joint_states_msgs)

    # C. Per-Motor Telemetry Statistics
    per_motor_stats = {}
    for idx, m_id in enumerate(MOTOR_IDS):
        pos_list = [s["positions_steps"][idx] for s in snapshots_history]
        curr_list = [s["currents_mA"][idx] for s in snapshots_history]
        force_list = [s["forces_g_raw"][idx] for s in snapshots_history]
        temp_list = [s["temperatures_C"][idx] for s in snapshots_history]
        fault_list = [s["fault_codes"][idx] for s in snapshots_history]

        pos_hist = dict(Counter(pos_list))
        per_motor_stats[f"motor_{m_id}"] = {
            "motor_id": m_id,
            "position": {
                "initial": pos_list[0] if pos_list else None,
                "final": pos_list[-1] if pos_list else None,
                "min": int(np.min(pos_list)) if pos_list else None,
                "max": int(np.max(pos_list)) if pos_list else None,
                "span": int(np.max(pos_list) - np.min(pos_list)) if pos_list else None,
                "mean": float(np.mean(pos_list)) if pos_list else None,
                "histogram": {str(k): v for k, v in sorted(pos_hist.items())},
            },
            "current_mA": {
                "min": int(np.min(curr_list)) if curr_list else None,
                "max": int(np.max(curr_list)) if curr_list else None,
                "mean": float(np.mean(curr_list)) if curr_list else None,
            },
            "force_g_raw": {
                "min": int(np.min(force_list)) if force_list else None,
                "max": int(np.max(force_list)) if force_list else None,
                "mean": float(np.mean(force_list)) if force_list else None,
            },
            "temperature_C": {
                "min": int(np.min(temp_list)) if temp_list else None,
                "max": int(np.max(temp_list)) if temp_list else None,
            },
            "fault_set": [f"0x{f:02X}" for f in sorted(set(fault_list))],
        }

    # D. Final Pass/Fail Verdict
    pass_conditions = {
        "duration_met": runtime_duration >= (duration_sec - 1.0),
        "zero_startup_writes": startup_writes == 0,
        "zero_runtime_writes": runtime_writes == 0,
        "zero_shutdown_writes": (shutdown_writes - runtime_writes) == 0,
        "zero_total_writes": total_writes == 0,
        "hardware_mode_is_read_only": driver_node.hardware_mode == HardwareMode.READ_ONLY,
        "all_faults_zero": all(s["fault_set"] == ["0x00"] for s in per_motor_stats.values()),
        "si_conversion_valid": si_conversion_passed,
        "js_effort_empty": js_effort_empty,
        "js_velocity_empty": js_velocity_empty,
        "position_span_reasonable": all(s["position"]["span"] <= 3 for s in per_motor_stats.values()),
        "no_stop_condition_triggered": stop_reason is None,
    }

    is_pass = all(pass_conditions.values())
    verdict_str = "J2-A COMPLETE / PASS" if is_pass else "J2-A STOP / NOT PASSED"

    report_data = {
        "verdict": verdict_str,
        "timestamp": datetime.now().isoformat(),
        "runtime_duration_s": runtime_duration,
        "target_duration_s": duration_sec,
        "hardware_mode": driver_node.hardware_mode.value,
        "port": port,
        "baudrate": baudrate,
        "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
        "write_audit": {
            "startup_writes": startup_writes,
            "runtime_writes": runtime_writes,
            "shutdown_writes": shutdown_writes - runtime_writes,
            "total_hardware_writes": total_writes,
        },
        "pass_conditions": pass_conditions,
        "stop_reason": stop_reason,
        "timing_stats": t_cycle_stats,
        "ros2_topic_stats": {
            "joint_states_msg_count": js_count,
            "joint_states_effective_freq_hz": js_effective_freq,
            "raw_steps_msg_count": len(auditor_node.raw_steps_msgs),
            "raw_currents_msg_count": len(auditor_node.raw_currents_msgs),
            "raw_forces_msg_count": len(auditor_node.raw_forces_msgs),
            "raw_temps_msg_count": len(auditor_node.raw_temps_msgs),
            "raw_faults_msg_count": len(auditor_node.raw_faults_msgs),
            "si_unit_m_verified": si_conversion_passed,
            "js_effort_empty": js_effort_empty,
            "js_velocity_empty": js_velocity_empty,
        },
        "per_motor_stats": per_motor_stats,
        "snapshots_sample_count": len(snapshots_history),
    }

    # Save JSON log
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report_data, f, indent=2, ensure_ascii=False)
    print(f"Saved full JSON log: {json_path}")

    # Generate Human-Readable Log & Print
    log_text = f"""
================================================================================
STAGE J2-A VERIFICATION REPORT — {verdict_str}
================================================================================
Timestamp: {report_data['timestamp']}
Duration: {runtime_duration:.2f} s / {duration_sec:.1f} s
Hardware Mode: {report_data['hardware_mode']}
Port: {port} (Baudrate: {baudrate})
ROS 2 Distro: {report_data['ros_distro']}

--------------------------------------------------------------------------------
1. WRITE AUDIT (0-Write Redline)
--------------------------------------------------------------------------------
Startup Writes:        {startup_writes}
Runtime Writes:        {runtime_writes}
Shutdown Writes:       {shutdown_writes - runtime_writes}
TOTAL HARDWARE WRITES: {total_writes} -> {'PASS (0 WRITES)' if total_writes == 0 else 'FAIL'}

--------------------------------------------------------------------------------
2. TIMING & ACQUISITION PERFORMANCE (Ordered Sequential 6-Axis Snapshot)
--------------------------------------------------------------------------------
Total Acquisition Cycles:        {len(snapshots_history)}
T_cycle Min:                     {t_cycle_stats['min_ms']:.2f} ms
T_cycle Mean:                    {t_cycle_stats['mean_ms']:.2f} ms
T_cycle Median (P50):            {t_cycle_stats['median_p50_ms']:.2f} ms
T_cycle P95:                     {t_cycle_stats['p95_ms']:.2f} ms
T_cycle P99:                     {t_cycle_stats['p99_ms']:.2f} ms
T_cycle Max:                     {t_cycle_stats['max_ms']:.2f} ms
T_cycle Std:                     {t_cycle_stats['std_ms']:.2f} ms
Effective Acquisition Frequency: {t_cycle_stats['effective_acquisition_freq_hz']:.2f} Hz
ID1 -> ID6 Sampling Span (Mean): {t_cycle_stats['mean_span_id6_id1_ms']:.2f} ms

--------------------------------------------------------------------------------
3. ROS 2 TOPIC PUBLISHING PERFORMANCE & SI UNIT AUDIT
--------------------------------------------------------------------------------
/pus6/joint_states Msg Count:    {js_count} ({js_effective_freq:.2f} Hz)
SI Position Unit (m) Correct:    {si_conversion_passed} (40,000 steps/m)
JointState Effort Field:         {'EMPTY []' if js_effort_empty else 'NON-EMPTY (VIOLATION)'}
JointState Velocity Field:       {'EMPTY []' if js_velocity_empty else 'NON-EMPTY (VIOLATION)'}
Raw Diagnostic Topics Count:     {len(auditor_node.raw_steps_msgs)} msgs each

--------------------------------------------------------------------------------
4. 6-MOTOR STATIC TELEMETRY & BASELINE SUMMARY
--------------------------------------------------------------------------------
"""
    for m_key, s in per_motor_stats.items():
        m_id = s["motor_id"]
        pos = s["position"]
        curr = s["current_mA"]
        force = s["force_g_raw"]
        temp = s["temperature_C"]
        fault = s["fault_set"]
        log_text += f"""Motor ID{m_id}:
  Position:     Init={pos['initial']} step, Final={pos['final']} step, Span={pos['span']} step (Min={pos['min']}, Max={pos['max']}, Mean={pos['mean']:.2f})
  Histogram:    {pos['histogram']}
  Current (mA): Min={curr['min']}, Max={curr['max']}, Mean={curr['mean']:.1f}
  Force Raw(g): Min={force['min']}, Max={force['max']}, Mean={force['mean']:.1f}
  Temp (degC):  Min={temp['min']}, Max={temp['max']}
  Fault Codes:  {fault}

"""

    log_text += f"""--------------------------------------------------------------------------------
5. FINAL VERDICT: {verdict_str}
--------------------------------------------------------------------------------
All Pass Conditions:
"""
    for cond_k, cond_v in pass_conditions.items():
        log_text += f"  - {cond_k:30s}: {'PASS' if cond_v else 'FAIL'}\n"

    with open(log_path, "w", encoding="utf-8") as f:
        f.write(log_text)
    print(f"Saved human-readable log: {log_path}")
    print(log_text)

    return 0 if is_pass else 1


if __name__ == "__main__":
    sys.exit(run_j2a_verification())
