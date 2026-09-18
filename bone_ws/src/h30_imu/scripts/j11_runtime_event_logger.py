#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent high-rate runtime event logger for J11 validation.

Captures raw hardware telemetry:
  - /pus6/joint_currents_raw (mA)
  - /pus6/joint_steps_raw (steps)
  - /pus6/command_joint_steps (steps)
  - /pus6/joint_faults_raw (fault codes)
  - /rosout (driver logs, slew recovery, STOP latches)

Maintains high-resolution timestamps (nanoseconds) and outputs:
  - high_rate_telemetry.csv
  - slew_recovery_events.csv
  - current_spikes.csv (>400mA)
  - runtime_logger_summary.json
"""

import argparse
import csv
import json
import os
import re
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import Int32MultiArray
    from rcl_interfaces.msg import Log
except ImportError:
    rclpy = None
    Node = object
    Log = object


SLEW_RECOVERY_REGEX = re.compile(
    r"original_gap=(\d+).*?allowance=(\d+).*?dt=([\d\.]+)ms.*?alpha=([\d\.]+).*?sent_gap=(\d+).*?remaining=(\d+)"
)


class J11RuntimeEventLogger(Node if Node is not object else object):
    def __init__(self, output_dir: str):
        if Node is not object:
            super().__init__("j11_runtime_event_logger")

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.node_init_time_sec = time.time()

        # Telemetry logs
        self.current_samples: List[Dict] = []
        self.step_samples: List[Dict] = []
        self.cmd_step_samples: List[Dict] = []
        self.fault_samples: List[Dict] = []
        self.rosout_events: List[Dict] = []
        self.slew_events: List[Dict] = []
        self.current_spikes: List[Dict] = []

        self.latest_steps: List[int] = [1000] * 6
        self.latest_cmd_steps: List[int] = [1000] * 6
        self.latest_faults: List[int] = [0] * 6

        self.max_current_per_axis: List[int] = [0] * 6
        self.min_current_per_axis: List[int] = [0] * 6
        self.stop_latched: bool = False
        self.stop_reason: str = ""

        if Node is not object:
            qos_sensor = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            qos_rosout = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

            self.create_subscription(Int32MultiArray, "/pus6/joint_currents_raw", self._on_currents, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_steps_raw", self._on_fb_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/command_joint_steps", self._on_cmd_steps, qos_sensor)
            self.create_subscription(Int32MultiArray, "/pus6/joint_faults_raw", self._on_faults, qos_sensor)
            self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)

    def _on_currents(self, msg: Int32MultiArray):
        if len(msg.data) < 6:
            return
        t_ns = time.perf_counter_ns()
        currents = list(msg.data[:6])

        for i, c in enumerate(currents):
            if c > self.max_current_per_axis[i]:
                self.max_current_per_axis[i] = c
            if c < self.min_current_per_axis[i]:
                self.min_current_per_axis[i] = c

        row = {
            "timestamp_ns": t_ns,
            "current_mA_axis1": currents[0],
            "current_mA_axis2": currents[1],
            "current_mA_axis3": currents[2],
            "current_mA_axis4": currents[3],
            "current_mA_axis5": currents[4],
            "current_mA_axis6": currents[5],
            "max_current_mA": max(currents),
        }
        self.current_samples.append(row)

        if any(c >= 400 for c in currents):
            spike_row = dict(row)
            spike_row["trigger_axes"] = [i + 1 for i, c in enumerate(currents) if c >= 400]
            self.current_spikes.append(spike_row)

    def _on_fb_steps(self, msg: Int32MultiArray):
        if len(msg.data) < 6:
            return
        t_ns = time.perf_counter_ns()
        steps = list(msg.data[:6])
        self.latest_steps = steps
        self.step_samples.append({
            "timestamp_ns": t_ns,
            "fb_step_axis1": steps[0],
            "fb_step_axis2": steps[1],
            "fb_step_axis3": steps[2],
            "fb_step_axis4": steps[3],
            "fb_step_axis5": steps[4],
            "fb_step_axis6": steps[5],
        })

    def _on_cmd_steps(self, msg: Int32MultiArray):
        if len(msg.data) < 6:
            return
        t_ns = time.perf_counter_ns()
        steps = list(msg.data[:6])
        self.latest_cmd_steps = steps
        self.cmd_step_samples.append({
            "timestamp_ns": t_ns,
            "cmd_step_axis1": steps[0],
            "cmd_step_axis2": steps[1],
            "cmd_step_axis3": steps[2],
            "cmd_step_axis4": steps[3],
            "cmd_step_axis5": steps[4],
            "cmd_step_axis6": steps[5],
        })

    def _on_faults(self, msg: Int32MultiArray):
        if len(msg.data) < 6:
            return
        t_ns = time.perf_counter_ns()
        faults = list(msg.data[:6])
        self.latest_faults = faults
        if any(f != 0 for f in faults):
            self.fault_samples.append({
                "timestamp_ns": t_ns,
                "faults": faults,
            })

    def _on_rosout(self, msg: Log):
        msg_t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if msg_t > 0 and msg_t < self.node_init_time_sec:
            return

        t_ns = time.perf_counter_ns()
        text = msg.msg

        # Check for Slew Recovery
        if "Vector slew recovery active" in text:
            m = SLEW_RECOVERY_REGEX.search(text)
            if m:
                ev = {
                    "timestamp_ns": t_ns,
                    "original_gap": int(m.group(1)),
                    "allowance": int(m.group(2)),
                    "dt_ms": float(m.group(3)),
                    "alpha": float(m.group(4)),
                    "sent_gap": int(m.group(5)),
                    "remaining_gap": int(m.group(6)),
                    "log_msg": text,
                }
                self.slew_events.append(ev)

        # Check for Fatal / Error STOP latches
        if msg.level >= 40 or "STOP latched" in text or "scheduler_deadline_exceeded" in text or "overcurrent" in text:
            self.rosout_events.append({
                "timestamp_ns": t_ns,
                "level": msg.level,
                "node": msg.name,
                "msg": text,
            })
            if "STOP latched" in text or "scheduler_deadline_exceeded" in text or "overcurrent" in text:
                self.stop_latched = True
                self.stop_reason = text

    def save_all_records(self):
        # 1. Currents CSV
        if self.current_samples:
            curr_path = self.output_dir / "high_rate_joint_currents_raw.csv"
            keys = list(self.current_samples[0].keys())
            with open(curr_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(self.current_samples)

        # 2. Steps CSV
        if self.step_samples:
            steps_path = self.output_dir / "high_rate_joint_steps_raw.csv"
            keys = list(self.step_samples[0].keys())
            with open(steps_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(self.step_samples)

        # 3. Slew Recovery Events CSV
        slew_path = self.output_dir / "slew_recovery_events.csv"
        with open(slew_path, "w", newline="") as f:
            fields = ["timestamp_ns", "original_gap", "allowance", "dt_ms", "alpha", "sent_gap", "remaining_gap", "log_msg"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            if self.slew_events:
                w.writerows(self.slew_events)

        # 4. Current Spikes CSV
        spike_path = self.output_dir / "current_spikes.csv"
        with open(spike_path, "w", newline="") as f:
            fields = ["timestamp_ns", "current_mA_axis1", "current_mA_axis2", "current_mA_axis3", "current_mA_axis4", "current_mA_axis5", "current_mA_axis6", "max_current_mA", "trigger_axes"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            if self.current_spikes:
                w.writerows(self.current_spikes)

        # 5. Rosout Events CSV
        rosout_path = self.output_dir / "driver_rosout_events.csv"
        with open(rosout_path, "w", newline="") as f:
            fields = ["timestamp_ns", "level", "node", "msg"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            if self.rosout_events:
                w.writerows(self.rosout_events)

        # 6. Summary JSON
        summary = {
            "output_dir": str(self.output_dir),
            "total_current_samples": len(self.current_samples),
            "total_step_samples": len(self.step_samples),
            "total_slew_events": len(self.slew_events),
            "total_current_spikes_ge_400mA": len(self.current_spikes),
            "max_current_per_axis_mA": self.max_current_per_axis,
            "min_current_per_axis_mA": self.min_current_per_axis,
            "driver_stop_latched": self.stop_latched,
            "driver_stop_reason": self.stop_reason,
            "last_step_feedback": self.latest_steps,
            "last_fault_codes": self.latest_faults,
        }
        with open(self.output_dir / "runtime_logger_summary.json", "w") as f:
            json.dump(summary, f, indent=2)

        print(f"[EVENT LOGGER] Data persisted to {self.output_dir} (Currents={len(self.current_samples)}, Slews={len(self.slew_events)}, Spikes={len(self.current_spikes)}).", flush=True)


def main():
    parser = argparse.ArgumentParser(description="J11 High-Rate Runtime Event Logger")
    parser.add_argument("--output-dir", required=True, help="Directory to save runtime event telemetry")
    args = parser.parse_args()

    if rclpy is None:
        print("Error: rclpy not installed", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    logger_node = J11RuntimeEventLogger(args.output_dir)

    stop_requested = False

    def handle_sig(sig, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    print(f"[EVENT LOGGER] Listening for telemetry on /pus6/... saving to {args.output_dir}", flush=True)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(logger_node, timeout_sec=0.05)
            if Path("/tmp/j11_stop_requested").exists():
                break
            if Path("/tmp/j11_driver_stopped").exists():
                time.sleep(0.5)  # allow catching trailing rosout/current messages
                break
    finally:
        logger_node.save_all_records()
        logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
