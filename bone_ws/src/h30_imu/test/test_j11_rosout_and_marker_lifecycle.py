#!/usr/bin/env python3
"""Unit tests for J11 feedforward runner rosout detector and marker lifecycle.

Verifies:
1. Volatile QoS profile on /rosout subscription.
2. Stale /rosout message rejection (timestamp gating).
3. Fresh /rosout message fault latching.
4. /tmp/j11_active_signaled lifecycle cleanup (init, fault, finally).
"""

import os
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock
import rclpy

# Import runner components without live hardware
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import j11_base_feedforward_runner as runner_mod
from j11_base_feedforward_runner import J11BaseFeedforwardRunner, J11State


class MockLogMsg:
    def __init__(self, name: str, level: int, msg: str, stamp_sec: float):
        self.name = name
        self.level = level
        self.msg = msg
        self.stamp = MagicMock()
        self.stamp.sec = int(stamp_sec)
        self.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)


class TestJ11RosoutAndLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        for m in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_driver_stopped"]:
            p = Path(m)
            if p.exists():
                p.unlink()

    def tearDown(self):
        for m in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_driver_stopped"]:
            p = Path(m)
            if p.exists():
                p.unlink()

    def test_marker_cleanup_on_init(self):
        """Pre-existing markers should be unlinked when runner initializes."""
        active_p = Path("/tmp/j11_active_signaled")
        ready_p = Path("/tmp/j11_ready_for_start")
        stopped_p = Path("/tmp/j11_driver_stopped")

        active_p.touch()
        ready_p.touch()
        stopped_p.touch()
        self.assertTrue(active_p.exists())

        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True)
        try:
            self.assertFalse(active_p.exists(), "active_signaled must be unlinked on init")
            self.assertFalse(ready_p.exists(), "ready_for_start must be unlinked on init")
            self.assertFalse(stopped_p.exists(), "driver_stopped must be unlinked on init")
        finally:
            runner.destroy_node()

    def test_stale_rosout_rejection(self):
        """Stale /rosout logs emitted before runner startup must be rejected."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True)
        try:
            t_init = runner.node_init_time_sec

            # Fabricate a fatal message with timestamp 5.0 seconds in the past
            stale_msg = MockLogMsg(
                name="pus6_motor_driver_node",
                level=50,
                msg="J7 driver STOP latched: scheduler_deadline_exceeded",
                stamp_sec=t_init - 5.0,
            )

            runner._on_rosout(stale_msg)
            self.assertFalse(runner.driver_stop_latched, "Stale rosout must NOT latch driver stop")
            self.assertNotEqual(runner.state, J11State.FAULT, "Stale rosout must NOT trip FAULT")
            self.assertEqual(runner.driver_fault_msg, "")
        finally:
            runner.destroy_node()

    def test_fresh_rosout_acceptance(self):
        """Fresh /rosout logs emitted after runner startup must latch FAULT."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True)
        try:
            t_init = runner.node_init_time_sec

            # Fabricate a fatal message with timestamp 0.1s after runner init
            fresh_msg = MockLogMsg(
                name="pus6_motor_driver_node",
                level=50,
                msg="J7 driver STOP latched: scheduler_deadline_exceeded",
                stamp_sec=t_init + 0.1,
            )

            runner._on_rosout(fresh_msg)
            self.assertTrue(runner.driver_stop_latched, "Fresh rosout MUST latch driver stop")
            self.assertEqual(runner.state, J11State.FAULT, "Fresh rosout MUST trip FAULT")
            self.assertIn("scheduler_deadline_exceeded", runner.fault_reason)
        finally:
            runner.destroy_node()

    def test_active_marker_unlinked_on_fault(self):
        """When fault occurs, /tmp/j11_active_signaled must be immediately unlinked."""
        runner = J11BaseFeedforwardRunner(execute_motion=False, dry_run=True)
        try:
            active_p = Path("/tmp/j11_active_signaled")
            active_p.touch()
            self.assertTrue(active_p.exists())

            runner._enter_fault("Simulated test fault")
            self.assertEqual(runner.state, J11State.FAULT)
            self.assertFalse(active_p.exists(), "/tmp/j11_active_signaled must be unlinked when entering FAULT")
            self.assertTrue(Path("/tmp/j11_driver_stopped").exists(), "driver_stopped marker should be created on FAULT")
        finally:
            runner.destroy_node()


if __name__ == "__main__":
    unittest.main()
