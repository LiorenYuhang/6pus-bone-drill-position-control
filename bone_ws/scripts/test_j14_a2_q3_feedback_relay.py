#!/usr/bin/env python3
"""Stage J14-A2-Q3H: Software Unit Verification for Legacy Feedback & JointState Relay.

Tests:
  1. /joint_states correctly forwarded to /pus6/joint_states
  2. Header strictly preserved (stamp, frame_id)
  3. Name list strictly preserved
  4. Position list strictly preserved
  5. Velocity list strictly preserved
  6. Effort list strictly preserved
  7. Axis order strictly preserved (1..6)
  8. Units unchanged (SI meters)
  9. Exact 1-to-1 input to output mapping
  10. Zero serial or hardware device access
  11. Existing TimedJointFeedback -> raw_steps functionality without regression
  12. Clean shutdown & artifact recording (relay_lifecycle.jsonl & relay_summary.json)
"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from builtin_interfaces.msg import Time

try:
    from pus6_msgs.msg import TimedJointFeedback
except ImportError:
    TimedJointFeedback = None

import sys
sys.path.insert(0, str(Path(__file__).parent))
from j14_a2_q3_feedback_relay import LegacyFeedbackRelayNode


class TestFeedbackRelay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp(prefix="relay_test_"))
        self.node = LegacyFeedbackRelayNode(out_dir=self.tmp_dir)

    def tearDown(self):
        self.node.close()
        self.node.destroy_node()
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def test_direct_raw_steps_conversion(self):
        """Test criterion 11: TimedJointFeedback -> joint_steps_raw functionality."""
        test_cases = [
            [1000, 1000, 1000, 1000, 1000, 1000],          # Neutral operating home
            [985, 1015, 1050, 950, 1200, 800],             # Arbitrary displacements
            [-5000, 50000, 0, -1, 100000, -99999],         # Stress / extreme (no clamp check)
        ]

        for seq, steps in enumerate(test_cases):
            msg = TimedJointFeedback()
            msg.uart_cycle_sequence = seq
            msg.raw_steps = list(steps)

            orig_pub = self.node.raw_steps_pub.publish
            captured = []
            self.node.raw_steps_pub.publish = lambda m: captured.append(list(m.data))

            self.node._on_feedback(msg)
            self.node.raw_steps_pub.publish = orig_pub

            self.assertEqual(len(captured), 1, "1 input must produce exactly 1 output")
            out_steps = captured[0]
            self.assertEqual(out_steps, steps, f"Mismatch: expected {steps}, got {out_steps}")
            self.assertEqual(len(out_steps), 6, "Must have exactly 6 axes")

        self.assertEqual(self.node.raw_steps_received, len(test_cases))
        self.assertEqual(self.node.raw_steps_published, len(test_cases))
        self.assertEqual(self.node.first_input_seq, 0)
        self.assertEqual(self.node.last_input_seq, len(test_cases) - 1)
        self.assertEqual(self.node.last_raw_steps, test_cases[-1])

    def test_joint_state_forwarding_and_preservation(self):
        """Test criteria 1-9: /joint_states -> /pus6/joint_states 1:1 preservation."""
        test_cases = [
            # Case 1: Operating home (0.025m = 1000 steps * 0.000025)
            dict(
                sec=123, nanosec=456789, frame_id="pus6_base",
                names=[f"motor_{i}" for i in range(1, 7)],
                positions=[0.025] * 6,
                velocities=[0.0] * 6,
                efforts=[0.0] * 6,
            ),
            # Case 2: Arbitrary dynamic displacement
            dict(
                sec=124, nanosec=100000, frame_id="world",
                names=[f"motor_{i}" for i in range(1, 7)],
                positions=[0.024625, 0.025375, 0.026250, 0.023750, 0.030000, 0.020000],
                velocities=[0.001, -0.002, 0.003, -0.001, 0.005, -0.004],
                efforts=[10.0, 12.5, 8.0, 15.0, 9.2, 11.1],
            ),
        ]

        for case in test_cases:
            in_msg = JointState()
            in_msg.header.stamp = Time(sec=case["sec"], nanosec=case["nanosec"])
            in_msg.header.frame_id = case["frame_id"]
            in_msg.name = list(case["names"])
            in_msg.position = list(case["positions"])
            in_msg.velocity = list(case["velocities"])
            in_msg.effort = list(case["efforts"])

            orig_pub = self.node.joint_state_pub.publish
            captured = []
            self.node.joint_state_pub.publish = lambda m: captured.append(m)

            self.node._on_joint_state(in_msg)
            self.node.joint_state_pub.publish = orig_pub

            # 9. Exactly 1 input produces 1 output
            self.assertEqual(len(captured), 1, "1 input must produce exactly 1 output")
            out = captured[0]

            # 2. Header strictly preserved
            self.assertEqual(out.header.stamp.sec, case["sec"])
            self.assertEqual(out.header.stamp.nanosec, case["nanosec"])
            self.assertEqual(out.header.frame_id, case["frame_id"])

            # 3. Name strictly preserved
            self.assertEqual(out.name, case["names"])

            # 4. Position strictly preserved
            self.assertEqual(list(out.position), case["positions"])

            # 5. Velocity strictly preserved
            self.assertEqual(list(out.velocity), case["velocities"])

            # 6. Effort strictly preserved
            self.assertEqual(list(out.effort), case["efforts"])

            # 7. Order preserved (6 axes)
            self.assertEqual(len(out.position), 6)

            # 8. Units unchanged (SI meters)
            for p_in, p_out in zip(case["positions"], out.position):
                self.assertAlmostEqual(p_in, p_out, places=9)

        self.assertEqual(self.node.joint_states_received, len(test_cases))
        self.assertEqual(self.node.joint_states_published, len(test_cases))

    def test_evidence_recording_and_clean_shutdown(self):
        """Test criterion 12: clean shutdown & artifact recording."""
        fb_msg = TimedJointFeedback()
        fb_msg.uart_cycle_sequence = 42
        fb_msg.raw_steps = [1001, 1002, 1003, 1004, 1005, 1006]
        self.node._on_feedback(fb_msg)

        js_msg = JointState()
        js_msg.name = [f"motor_{i}" for i in range(1, 7)]
        js_msg.position = [0.025] * 6
        self.node._on_joint_state(js_msg)

        self.node.close()

        summary_file = self.tmp_dir / "relay_summary.json"
        lifecycle_file = self.tmp_dir / "relay_lifecycle.jsonl"
        self.assertTrue(summary_file.exists(), "relay_summary.json must exist")
        self.assertTrue(lifecycle_file.exists(), "relay_lifecycle.jsonl must exist")

        summary = json.loads(summary_file.read_text())
        self.assertEqual(summary["raw_steps_received"], 1)
        self.assertEqual(summary["raw_steps_published"], 1)
        self.assertEqual(summary["first_input_seq"], 42)
        self.assertEqual(summary["last_raw_steps"], [1001, 1002, 1003, 1004, 1005, 1006])
        self.assertEqual(summary["joint_states_received"], 1)
        self.assertEqual(summary["joint_states_published"], 1)
        self.assertEqual(summary["last_joint_state_positions"], [0.025] * 6)
        self.assertEqual(summary["exit_reason"], "clean_shutdown")

        lifecycle_lines = [json.loads(l) for l in lifecycle_file.read_text().splitlines() if l.strip()]
        events = [l["event"] for l in lifecycle_lines]
        self.assertIn("relay_started", events)
        self.assertIn("relay_stopped", events)

    def test_qos_configuration(self):
        # 1. Raw steps QoS (SensorData profile)
        self.assertEqual(self.node.timed_fb_sub.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(self.node.timed_fb_sub.qos_profile.durability, DurabilityPolicy.VOLATILE)
        self.assertEqual(self.node.raw_steps_pub.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(self.node.raw_steps_pub.qos_profile.durability, DurabilityPolicy.VOLATILE)

        # 2. JointState QoS (Reliable profile)
        self.assertEqual(self.node.joint_state_sub.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(self.node.joint_state_sub.qos_profile.durability, DurabilityPolicy.VOLATILE)
        self.assertEqual(self.node.joint_state_pub.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(self.node.joint_state_pub.qos_profile.durability, DurabilityPolicy.VOLATILE)


if __name__ == "__main__":
    unittest.main()
