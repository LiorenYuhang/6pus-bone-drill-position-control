#!/usr/bin/env python3
"""Stage J14-A2-Q3H: Legacy Feedback & JointState Compatibility Relay.

Bridging formal production feedback to legacy consumers:
  1. Feedback Steps Relay:
     /pus6/joint_feedback/timed (TimedJointFeedback) -> /pus6/joint_steps_raw (Int32MultiArray)
     Used by legacy J11/J13 Runner preflight gates.
  2. JointState Relay:
     /joint_states (sensor_msgs/msg/JointState) -> /pus6/joint_states (sensor_msgs/msg/JointState)
     Used by pus6_control_node trajectory shaper feedback ingress.

Strictly:
  - 1-to-1 conversion of JointState and raw_steps (no filter, no offset, no clamp, no interpolation)
  - Zero serial or hardware access
  - Feedback authority remains UART Child -> Feedback Bridge
  - Writes audit evidence: relay_lifecycle.jsonl & relay_summary.json
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
import threading
from typing import Any, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

try:
    from pus6_msgs.msg import TimedJointFeedback
except ImportError:
    TimedJointFeedback = None


class LegacyFeedbackRelayNode(Node):
    def __init__(self, out_dir: Optional[Path] = None):
        super().__init__("j14_a2_q3_feedback_relay")
        self.out_dir = out_dir
        if self.out_dir:
            self.out_dir.mkdir(parents=True, exist_ok=True)

        # Metrics for timed feedback -> raw steps
        self.raw_steps_received = 0
        self.raw_steps_published = 0
        self.first_input_seq: Optional[int] = None
        self.last_input_seq: Optional[int] = None
        self.last_raw_steps: List[int] = []

        # Metrics for JointState -> pus6/joint_states
        self.joint_states_received = 0
        self.joint_states_published = 0
        self.last_joint_state_positions: List[float] = []

        # Sensor data QoS for TimedJointFeedback -> /pus6/joint_steps_raw
        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Standard reliable QoS for JointState -> /pus6/joint_states
        qos_joint_state = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 1. Raw steps publisher & subscription
        self.raw_steps_pub = self.create_publisher(
            Int32MultiArray, "/pus6/joint_steps_raw", qos_sensor
        )

        if TimedJointFeedback is None:
            raise RuntimeError("pus6_msgs.msg.TimedJointFeedback is required for relay")

        self.timed_fb_sub = self.create_subscription(
            TimedJointFeedback,
            "/pus6/joint_feedback/timed",
            self._on_feedback,
            qos_sensor,
        )

        # 2. JointState publisher & subscription (Q3H)
        self.joint_state_pub = self.create_publisher(
            JointState, "/pus6/joint_states", qos_joint_state
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_joint_state,
        )

        self._record_lifecycle("relay_started")

    def _record_lifecycle(self, event: str, extra: Optional[dict] = None):
        if not self.out_dir:
            return
        row = {"event": event, "monotonic_ns": time.monotonic_ns(), "pid": os.getpid()}
        if extra:
            row.update(extra)
        path = self.out_dir / "relay_lifecycle.jsonl"
        with path.open("a") as f:
            f.write(json.dumps(row) + "\n")

    def _on_feedback(self, msg: TimedJointFeedback):
        steps = list(msg.raw_steps)
        if len(steps) != 6:
            return

        out_msg = Int32MultiArray()
        # Pure 1-to-1 mapping without filter, clamp or offset
        out_msg.data = [int(s) for s in steps]
        self.raw_steps_pub.publish(out_msg)

        self.raw_steps_received += 1
        self.raw_steps_published += 1
        if self.first_input_seq is None:
            self.first_input_seq = int(msg.uart_cycle_sequence)
        self.last_input_seq = int(msg.uart_cycle_sequence)
        self.last_raw_steps = list(out_msg.data)

    def _on_joint_state(self, msg: JointState):
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = list(msg.name)
        out_msg.position = [float(p) for p in msg.position]
        out_msg.velocity = [float(v) for v in msg.velocity]
        out_msg.effort = [float(e) for e in msg.effort]

        self.joint_state_pub.publish(out_msg)

        self.joint_states_received += 1
        self.joint_states_published += 1
        self.last_joint_state_positions = list(out_msg.position)

    def close(self):
        summary = {
            "raw_steps_received": self.raw_steps_received,
            "raw_steps_published": self.raw_steps_published,
            "first_input_seq": self.first_input_seq,
            "last_input_seq": self.last_input_seq,
            "last_raw_steps": self.last_raw_steps,
            "joint_states_received": self.joint_states_received,
            "joint_states_published": self.joint_states_published,
            "last_joint_state_positions": self.last_joint_state_positions,
            # Backwards compatibility fields
            "total_received": self.raw_steps_received,
            "total_published": self.raw_steps_published,
            "exit_reason": "clean_shutdown",
        }
        if self.out_dir:
            (self.out_dir / "relay_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            self._record_lifecycle("relay_stopped", summary)


def run_relay(out_dir=None, stop_event=None):
    """Keep context destruction outside signal handlers and the wait set."""
    from rclpy.signals import SignalHandlerOptions
    stop_event = stop_event if stop_event is not None else threading.Event()
    previous = {}
    def request_stop(sig, frame):
        stop_event.set()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        node = LegacyFeedbackRelayNode(out_dir=out_dir)
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


def main():
    parser = argparse.ArgumentParser(description="J14-A2-Q3 Legacy Feedback Relay")
    parser.add_argument("--out-dir", type=Path, default=None, help="Directory to record relay artifacts")
    args = parser.parse_args()

    return run_relay(args.out_dir)


if __name__ == "__main__":
    sys.exit(main())
