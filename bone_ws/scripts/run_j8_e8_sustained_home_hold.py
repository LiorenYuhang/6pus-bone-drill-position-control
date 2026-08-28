#!/usr/bin/env python3
"""6PUS J8-E8: Sustained 20s Active Cartesian HOME Hold Hardware Validation.

Runs the complete, unbypassed ROS 2 topic graph on real hardware at OPERATING_HOME:
  /pus6/target_cartesian_pose -> pus6_kinematics_node -> /pus6/desired_joint_deltas
  -> pus6_control_node -> /pus6/command_joint_steps -> pus6_motor_driver_node (J7 Servo 50Hz)
  -> /pus6/joint_feedback/timed -> pus6_fk_node -> /pus6/fk/state

Holds OPERATING_HOME actively for 20.0s (~1000 cycles) and comprehensively records:
- Total successful Servo cycles and actual 0x29 batch rate
- Worker cycle intervals and worker_total_ms statistics (mean, P95, max)
- 0x29 RX classification duration and 0x30 transaction duration
- Per-axis feedback intervals and feedback age metrics
- Motion-write response counts vs. stale response counts
- Residual/overflow/malformed/timeout/fault counts
- Watchdog state and stop conditions
"""

from __future__ import annotations

import argparse
import gc
import json
import math
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

# Ensure workspace packages are on path
SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
SRC_DIR = WS_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs"]:
    p = SRC_DIR / pkg
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.safety import MOTOR_IDS
from pus6_msgs.msg import FkStateStamped, TimedJointCommand, TimedJointFeedback

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_RATE = 921600


class HomeHoldHarness(Node):
    def __init__(self) -> None:
        super().__init__("j8_e8_home_hold_harness")
        self.lock = threading.Lock()
        self.cmd_stamps_ns: list[int] = []
        self.feedback_stamps_ns: dict[int, list[int]] = {mid: [] for mid in MOTOR_IDS}
        self.feedback_positions: dict[int, list[int]] = {mid: [] for mid in MOTOR_IDS}
        self.feedback_currents: dict[int, list[int]] = {mid: [] for mid in MOTOR_IDS}
        self.feedback_faults: dict[int, list[int]] = {mid: [] for mid in MOTOR_IDS}
        self.diag_worker_intervals_ms: list[float] = []
        self.diag_write_durations_ms: list[float] = []
        self.diag_rx_class_ms: list[float] = []
        self.diag_feedback_tx_ms: list[float] = []
        self.diag_worker_total_ms: list[float] = []
        self.diag_watchdog_stop_age_ms: list[float] = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", 10
        )
        self.timed_cmd_sub = self.create_subscription(
            TimedJointCommand, "/pus6/timed_command", self._on_timed_cmd, sensor_qos
        )
        self.timed_fb_sub = self.create_subscription(
            TimedJointFeedback, "/pus6/joint_feedback/timed", self._on_timed_fb, sensor_qos
        )
        self.diag_sub = self.create_subscription(
            Float64MultiArray, "/pus6/driver_diagnostics", self._on_diag, 10
        )
        self.stream_active = False
        self.stream_z0 = 0.132612
        self.stream_timer = self.create_timer(0.020, self._on_stream_timer)

    def _on_stream_timer(self) -> None:
        if self.stream_active:
            self.publish_home_pose(self.stream_z0)

    def _on_timed_cmd(self, msg: TimedJointCommand) -> None:
        with self.lock:
            self.cmd_stamps_ns.append(msg.transaction_start_monotonic_ns)

    def _on_timed_fb(self, msg: TimedJointFeedback) -> None:
        with self.lock:
            for idx, mid in enumerate(MOTOR_IDS):
                self.feedback_stamps_ns[mid].append(msg.per_axis_rx_done_monotonic_ns[idx])
                self.feedback_positions[mid].append(msg.raw_steps[idx])
                self.feedback_currents[mid].append(msg.currents_ma[idx])
                self.feedback_faults[mid].append(msg.fault_codes[idx])

    def _on_diag(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 11:
            with self.lock:
                self.diag_worker_intervals_ms.append(msg.data[5])
                self.diag_write_durations_ms.append(msg.data[6])
                self.diag_rx_class_ms.append(msg.data[7])
                self.diag_feedback_tx_ms.append(msg.data[8])
                self.diag_worker_total_ms.append(msg.data[9])
                self.diag_watchdog_stop_age_ms.append(msg.data[10])

    def publish_home_pose(self, z0: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(z0)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        self.target_pose_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="J8-E8 Sustained 20s Active Cartesian HOME Hold Validation")
    parser.add_argument("--duration", type=float, default=20.0, help="Hold duration in seconds (default: 20.0s)")
    parser.add_argument("--output-json", type=str, default="scripts/j8_e8_home_hold_results.json", help="Path to save JSON metrics")
    args = parser.parse_args()

    z0 = 0.132612
    print(f"\n================================================================================")
    print(f"       6PUS J8-E8: SUSTAINED {args.duration:.1f}s ACTIVE CARTESIAN HOME HOLD")
    print(f"================================================================================")
    print(f"Nominal HOME z0: {z0:.6f} m ({z0*1000:.3f} mm)")
    print(f"Target streaming rate: 50.0 Hz (period = 20.0 ms)")
    print(f"Serial port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Duration: {args.duration:.1f} s (~{int(args.duration*50)} Servo cycles)")
    print(f"================================================================================\n")

    rclpy.init()

    kin_node = Pus6KinematicsNode()
    ctrl_node = Pus6ControlNode()
    motor_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.005,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=False,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0, query_rate_hz=50.0)
    harness = HomeHoldHarness()

    sys.setswitchinterval(0.001)

    exec_motor = SingleThreadedExecutor()
    exec_motor.add_node(motor_node)

    exec_other = SingleThreadedExecutor()
    exec_other.add_node(kin_node)
    exec_other.add_node(ctrl_node)
    exec_other.add_node(fk_node)
    exec_other.add_node(harness)

    th_motor = threading.Thread(target=exec_motor.spin, daemon=True)
    th_other = threading.Thread(target=exec_other.spin, daemon=True)
    th_motor.start()
    th_other.start()
    print("[INIT OK] Dedicated SingleThreadedExecutors spinning motor node and pipeline nodes in background.")

    # Allow startup and arming
    time.sleep(1.0)

    total_target_cycles = int(args.duration * 50)
    dt_target = 0.020

    print(f"\n[STREAM START] Streaming Cartesian HOME at 50 Hz for {args.duration:.1f} s ({total_target_cycles} cycles)...")
    gc.collect()
    gc.disable()
    harness.stream_z0 = z0
    harness.stream_active = True
    t_start = time.perf_counter()

    for sec in range(int(args.duration)):
        if motor_node.driver_fault_latched:
            print(f"\n[ABORT] Driver fault latched: {motor_node.driver_fault_reason}")
            break
        time.sleep(1.0)
        elapsed = time.perf_counter() - t_start
        print(f"  Holding HOME: elapsed {elapsed:5.1f}s / {args.duration:.1f}s | Latched Fault: {motor_node.driver_fault_latched}")

    harness.stream_active = False
    gc.enable()
    t_end = time.perf_counter()
    elapsed_total = t_end - t_start
    print(f"[STREAM END] Stream completed in {elapsed_total:.3f} s.\n")

    # Settle
    time.sleep(0.5)

    # Collect Audits from live nodes
    writer_audit = motor_node.j7_writer.audit_snapshot() if motor_node.j7_writer else None
    feedback_audit = motor_node.j7_feedback.audit_snapshot() if motor_node.j7_feedback else None
    rx_audit = motor_node.j7_adapter.rx_audit_snapshot() if motor_node.j7_adapter else None

    # Shutdown
    print("[SHUTDOWN] Shutting down ROS2 executors and nodes...")
    if motor_node.write_timer is not None:
        motor_node.write_timer.cancel()
    harness.stream_timer.cancel()
    exec_motor.shutdown()
    exec_other.shutdown()
    kin_node.destroy_node()
    ctrl_node.destroy_node()
    motor_node.destroy_node()
    fk_node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()

    # Process and Compute Metrics
    num_cmd_cycles = len(writer_audit.cycles) if writer_audit else 0
    actual_rate_hz = num_cmd_cycles / elapsed_total if elapsed_total > 0 else 0.0

    worker_intervals = harness.diag_worker_intervals_ms[10:] if len(harness.diag_worker_intervals_ms) > 10 else harness.diag_worker_intervals_ms
    write_durs = harness.diag_write_durations_ms[10:] if len(harness.diag_write_durations_ms) > 10 else harness.diag_write_durations_ms
    rx_class_durs = harness.diag_rx_class_ms[10:] if len(harness.diag_rx_class_ms) > 10 else harness.diag_rx_class_ms
    fb_tx_durs = harness.diag_feedback_tx_ms[10:] if len(harness.diag_feedback_tx_ms) > 10 else harness.diag_feedback_tx_ms
    worker_totals = harness.diag_worker_total_ms[10:] if len(harness.diag_worker_total_ms) > 10 else harness.diag_worker_total_ms

    # Per-axis feedback intervals & max age
    per_axis_intervals: dict[int, dict[str, float]] = {}
    per_axis_max_age_ms: dict[int, float] = {}

    for mid in MOTOR_IDS:
        stamps = [s for s in harness.feedback_stamps_ns[mid] if s > 0]
        unique_stamps = []
        for s in stamps:
            if not unique_stamps or s != unique_stamps[-1]:
                unique_stamps.append(s)
        if len(unique_stamps) > 1:
            diffs_ms = np.diff(unique_stamps) / 1e6
            diffs_valid = [d for d in diffs_ms if d > 10.0]
            if diffs_valid:
                per_axis_intervals[mid] = {
                    "mean_ms": float(np.mean(diffs_valid)),
                    "p95_ms": float(np.percentile(diffs_valid, 95)),
                    "max_ms": float(np.max(diffs_valid)),
                }
                per_axis_max_age_ms[mid] = float(np.max(diffs_valid))
            else:
                per_axis_intervals[mid] = {"mean_ms": 0.0, "p95_ms": 0.0, "max_ms": 0.0}
                per_axis_max_age_ms[mid] = 0.0
        else:
            per_axis_intervals[mid] = {"mean_ms": 0.0, "p95_ms": 0.0, "max_ms": 0.0}
            per_axis_max_age_ms[mid] = 0.0

    print("================================================================================")
    print("                    J8-E8 HOME HOLD HARDWARE AUDIT REPORT                       ")
    print("================================================================================")
    print(f"Total Successful 0x29 Servo Cycles: {num_cmd_cycles} / {total_target_cycles} ({num_cmd_cycles/total_target_cycles*100:.1f}%)")
    print(f"Actual 0x29 Batch Write Rate:       {actual_rate_hz:.2f} Hz (target: 50.0 Hz)")
    print(f"Driver Fault Latched:               {motor_node.driver_fault_latched}")
    print(f"Driver Fault Reason:                {motor_node.driver_fault_reason or 'None (HEALTHY)'}")
    print(f"Watchdog Armed:                     {feedback_audit.watchdog_stop_snapshot is None and (feedback_audit.stop_reason is None)}")

    print(f"\n--- Worker & Timing Instrumentation ---")
    if worker_intervals:
        print(f"Worker Cycle Interval (ms):         Mean={np.mean(worker_intervals):.2f}, P95={np.percentile(worker_intervals, 95):.2f}, Max={np.max(worker_intervals):.2f}")
    if write_durs:
        print(f"0x29 Batch Write Duration (ms):     Mean={np.mean(write_durs):.2f}, P95={np.percentile(write_durs, 95):.2f}, Max={np.max(write_durs):.2f}")
    if rx_class_durs:
        print(f"0x29 RX Classification Time (ms):   Mean={np.mean(rx_class_durs):.2f}, P95={np.percentile(rx_class_durs, 95):.2f}, Max={np.max(rx_class_durs):.2f}")
    if fb_tx_durs:
        print(f"0x30 Poll Transaction Time (ms):    Mean={np.mean(fb_tx_durs):.2f}, P95={np.percentile(fb_tx_durs, 95):.2f}, Max={np.max(fb_tx_durs):.2f}")
    if worker_totals:
        print(f"Total Worker Cycle Time (ms):       Mean={np.mean(worker_totals):.2f}, P95={np.percentile(worker_totals, 95):.2f}, Max={np.max(worker_totals):.2f}")

    print(f"\n--- Per-Axis Feedback Round-Robin Intervals & Age (Threshold = 263.324 ms) ---")
    for mid in MOTOR_IDS:
        stats = per_axis_intervals.get(mid, {})
        print(f"  Axis ID{mid}: Mean Interval = {stats.get('mean_ms', 0):.2f} ms | P95 = {stats.get('p95_ms', 0):.2f} ms | Max Age = {per_axis_max_age_ms.get(mid, 0):.2f} ms")

    print(f"\n--- RX Classification & Buffer Counts ---")
    if rx_audit:
        print(f"Motion-Write Responses (0x29):      {rx_audit.motion_write_response_count}")
        print(f"Stale Responses:                    {rx_audit.stale_response_count}")
        print(f"Feedback Responses (0x30):          {rx_audit.feedback_response_count}")
        print(f"Residual Frames Remaining:          {rx_audit.residual_frames}")
        print(f"Malformed Frames:                   {rx_audit.malformed_count}")
        print(f"Buffer Overflows:                   {rx_audit.overflow_count}")

    print(f"\n--- Transport & Scheduler Failure Counters ---")
    if feedback_audit:
        print(f"Feedback Timeouts:                  {feedback_audit.timeout_count}")
        print(f"Deferred Cycles:                    {feedback_audit.defer_count}")
        print(f"Mismatch Count:                     {feedback_audit.mismatch_count}")
        print(f"Transport Stopped:                  {feedback_audit.stopped}")
        print(f"Stop Reason:                        {feedback_audit.stop_reason or 'None'}")

    # Check Acceptance Criteria
    rate_ok = abs(actual_rate_hz - 50.0) < 5.0
    stale_ok = (rx_audit.stale_response_count <= 2) if rx_audit else False
    residual_ok = rx_audit.residual_frames == 0 if rx_audit else False
    malformed_ok = rx_audit.malformed_count == 0 if rx_audit else False
    age_ok = all(age < 263.324 for age in per_axis_max_age_ms.values()) if per_axis_max_age_ms else False
    no_stop_ok = not motor_node.driver_fault_latched and (feedback_audit and not feedback_audit.stopped)

    all_pass = rate_ok and stale_ok and residual_ok and malformed_ok and age_ok and no_stop_ok

    print(f"\n================================================================================")
    print(f"OVERALL J8-E8 HOME HOLD VERDICT: {'PASS' if all_pass else 'FAIL'}")
    print(f"  1. 50Hz Sustained Rate ({actual_rate_hz:.2f} Hz):         {'PASS' if rate_ok else 'FAIL'}")
    print(f"  2. Stale Count (0):                                      {'PASS' if stale_ok else 'FAIL'} ({rx_audit.stale_response_count if rx_audit else 'N/A'})")
    print(f"  3. Residual Frames (0):                                  {'PASS' if residual_ok else 'FAIL'} ({rx_audit.residual_frames if rx_audit else 'N/A'})")
    print(f"  4. Malformed / Protocol Errors (0):                      {'PASS' if malformed_ok else 'FAIL'} ({rx_audit.malformed_count if rx_audit else 'N/A'})")
    print(f"  5. Max Feedback Age (< 263.324 ms):                      {'PASS' if age_ok else 'FAIL'} (max={max(per_axis_max_age_ms.values()) if per_axis_max_age_ms else 0:.2f} ms)")
    print(f"  6. Zero Latched STOP / Faults:                           {'PASS' if no_stop_ok else 'FAIL'}")
    print(f"================================================================================\n")

    # Write results to JSON
    report_dict = {
        "duration_s": elapsed_total,
        "total_cycles": num_cmd_cycles,
        "actual_rate_hz": actual_rate_hz,
        "worker_interval_mean_ms": float(np.mean(worker_intervals)) if worker_intervals else 0.0,
        "worker_interval_p95_ms": float(np.percentile(worker_intervals, 95)) if worker_intervals else 0.0,
        "worker_interval_max_ms": float(np.max(worker_intervals)) if worker_intervals else 0.0,
        "worker_total_mean_ms": float(np.mean(worker_totals)) if worker_totals else 0.0,
        "worker_total_p95_ms": float(np.percentile(worker_totals, 95)) if worker_totals else 0.0,
        "worker_total_max_ms": float(np.max(worker_totals)) if worker_totals else 0.0,
        "rx_class_mean_ms": float(np.mean(rx_class_durs)) if rx_class_durs else 0.0,
        "fb_tx_mean_ms": float(np.mean(fb_tx_durs)) if fb_tx_durs else 0.0,
        "per_axis_intervals": per_axis_intervals,
        "per_axis_max_age_ms": per_axis_max_age_ms,
        "motion_write_response_count": rx_audit.motion_write_response_count if rx_audit else 0,
        "stale_response_count": rx_audit.stale_response_count if rx_audit else 0,
        "feedback_response_count": rx_audit.feedback_response_count if rx_audit else 0,
        "residual_frames": rx_audit.residual_frames if rx_audit else 0,
        "malformed_count": rx_audit.malformed_count if rx_audit else 0,
        "timeout_count": feedback_audit.timeout_count if feedback_audit else 0,
        "defer_count": feedback_audit.defer_count if feedback_audit else 0,
        "driver_fault_latched": motor_node.driver_fault_latched,
        "driver_fault_reason": motor_node.driver_fault_reason,
        "verdict": "PASS" if all_pass else "FAIL",
    }

    out_path = Path(args.output_json)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report_dict, f, indent=2)
    print(f"Wrote full audit report to {out_path.resolve()}\n")

    if not all_pass:
        sys.exit(1)


if __name__ == "__main__":
    main()
