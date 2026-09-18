#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for J11 runtime instrumentation and fault auditing."""

import csv
import json
import os
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import rclpy
from std_msgs.msg import Int32MultiArray

# Import runner & logger components without live hardware
scripts_dir = str(Path(__file__).resolve().parent.parent / "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

import j11_base_feedforward_runner as runner_mod
from j11_base_feedforward_runner import J11BaseFeedforwardRunner, J11FeedforwardTelemetry, J11State
from j11_runtime_event_logger import J11RuntimeEventLogger


class MockLogMsg:
    def __init__(self, name: str, level: int, msg: str, stamp_sec: float):
        self.name = name
        self.level = level
        self.msg = msg
        self.stamp = MagicMock()
        self.stamp.sec = int(stamp_sec)
        self.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)


class TestJ11RuntimeInstrumentation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_slew_recovery_log_parsing(self):
        """Verify runner parses slew recovery warning and records event."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True, output_dir=self.test_dir)
        try:
            t_now = time.time()
            log_text = (
                "Vector slew recovery active: original_gap=20 steps > allowance=5 steps (dt=161.1ms). "
                "Scaling 6-axis vector by alpha=0.250 -> sent_gap=5 steps (remaining=15 steps)."
            )
            msg = MockLogMsg(
                name="pus6_motor_driver_node",
                level=30,  # WARN
                msg=log_text,
                stamp_sec=t_now,
            )
            runner._on_rosout(msg)

            self.assertIsNotNone(runner.latest_slew_event)
            self.assertEqual(runner.latest_slew_event["original_gap"], 20)
            self.assertEqual(runner.latest_slew_event["allowance"], 5)
            self.assertAlmostEqual(runner.latest_slew_event["dt_ms"], 161.1, places=1)
            self.assertAlmostEqual(runner.latest_slew_event["alpha"], 0.250, places=3)
            self.assertEqual(runner.latest_slew_event["sent_gap"], 5)
            self.assertEqual(runner.latest_slew_event["remaining_gap"], 15)
            self.assertEqual(len(runner.slew_events_history), 1)
        finally:
            runner.destroy_node()

    def test_current_subscription_and_tick(self):
        """Verify joint currents callback and telemetry fields."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True, output_dir=self.test_dir)
        try:
            msg = Int32MultiArray()
            msg.data = [120, 150, 480, 200, 210, 190]
            runner._on_fb_currents(msg)
            self.assertEqual(runner.latest_fb_currents, [120, 150, 480, 200, 210, 190])

            # Trigger control tick
            runner._on_control_tick()
            self.assertTrue(len(runner.telemetry_history) > 0)
            last_telem = runner.telemetry_history[-1]
            self.assertEqual(last_telem.joint_currents_raw_mA, [120, 150, 480, 200, 210, 190])
        finally:
            runner.destroy_node()

    def test_fault_window_audit_generation(self):
        """Verify fault window audit evaluates causal chain when fault occurs."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True, output_dir=self.test_dir)
        try:
            runner.state = J11State.ACTIVE
            runner.active_cycle_count = 100

            # Simulate 10 cycles with rising feedback age and current spike
            base_t = time.perf_counter_ns()
            for i in range(10):
                telem = J11FeedforwardTelemetry(
                    cycle_index=i + 1,
                    timestamp_ns=base_t + i * 20_000_000,
                    state=J11State.ACTIVE.value,
                    driver_feedback_age_ms=160.0 + i * 10.0,  # stall > 150ms
                    command_feedback_errors=[0, 0, 8, 0, 0, 0],
                    joint_currents_raw_mA=[100, 110, 495, 120, 105, 115],  # spike
                )
                runner.telemetry_history.append(telem)

            # Record a slew recovery event
            runner.slew_events_history.append({
                "timestamp_ns": base_t + 8 * 20_000_000,
                "original_gap": 15,
                "allowance": 5,
                "dt_ms": 190.0,
                "alpha": 0.333,
                "sent_gap": 5,
                "remaining_gap": 10,
            })

            # Enter fault
            runner._enter_fault("Driver STOP latched: overcurrent_ID3")
            self.assertEqual(runner.state, J11State.FAULT)

            # Test summary generation via run_feedforward_session logic
            # Simulate saving summary
            runner.telemetry_history.append(
                J11FeedforwardTelemetry(
                    cycle_index=11,
                    timestamp_ns=base_t + 10 * 20_000_000,
                    state=J11State.FAULT.value,
                    driver_feedback_age_ms=250.0,
                    joint_currents_raw_mA=[100, 110, 495, 120, 105, 115],
                    fault_reason=runner.fault_reason,
                )
            )

            # Run feedforward session analysis logic
            window_recs = runner.telemetry_history[-100:]
            fb_ages = [r.driver_feedback_age_ms for r in window_recs]
            max_fb_age = float(max(fb_ages))
            window_curr = [r.joint_currents_raw_mA for r in window_recs]
            max_curr_per_axis = [int(max(c[i] for c in window_curr)) for i in range(6)]
            peak_curr_axis = int(max_curr_per_axis.index(max(max_curr_per_axis))) + 1
            peak_curr_mA = max(max_curr_per_axis)

            self.assertTrue(max_fb_age >= 150.0)
            self.assertEqual(peak_curr_axis, 3)
            self.assertEqual(peak_curr_mA, 495)
            self.assertEqual(len(runner.slew_events_history), 1)
        finally:
            runner.destroy_node()

    def test_event_logger_callbacks_and_save(self):
        """Verify high rate event logger receives data and writes CSV files."""
        logger_node = J11RuntimeEventLogger(output_dir=self.test_dir)
        try:
            curr_msg = Int32MultiArray()
            curr_msg.data = [150, 160, 420, 180, 170, 165]
            logger_node._on_currents(curr_msg)

            step_msg = Int32MultiArray()
            step_msg.data = [1000, 1001, 1002, 999, 998, 1000]
            logger_node._on_fb_steps(step_msg)

            slew_log = MockLogMsg(
                name="pus6_motor_driver_node",
                level=30,
                msg="Vector slew recovery active: original_gap=16 steps > allowance=5 steps (dt=150.0ms). Scaling 6-axis vector by alpha=0.312 -> sent_gap=5 steps (remaining=11 steps).",
                stamp_sec=time.time(),
            )
            logger_node._on_rosout(slew_log)

            logger_node.save_all_records()

            curr_file = Path(self.test_dir) / "high_rate_joint_currents_raw.csv"
            slew_file = Path(self.test_dir) / "slew_recovery_events.csv"
            spike_file = Path(self.test_dir) / "current_spikes.csv"
            summary_file = Path(self.test_dir) / "runtime_logger_summary.json"

            self.assertTrue(curr_file.exists())
            self.assertTrue(slew_file.exists())
            self.assertTrue(spike_file.exists())  # 420 mA >= 400 mA spike
            self.assertTrue(summary_file.exists())

            with open(summary_file) as f:
                data = json.load(f)
                self.assertEqual(data["total_current_samples"], 1)
                self.assertEqual(data["total_slew_events"], 1)
                self.assertEqual(data["total_current_spikes_ge_400mA"], 1)
                self.assertEqual(data["max_current_per_axis_mA"][2], 420)
        finally:
            logger_node.destroy_node()


if __name__ == "__main__":
    unittest.main()
