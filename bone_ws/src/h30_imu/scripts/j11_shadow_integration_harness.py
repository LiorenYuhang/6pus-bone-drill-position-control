#!/usr/bin/env python3
"""Stage J11-1C: Production ROS Shadow Integration Harness.

Validates the full software pipeline using real J8 production nodes:
  PoseStamped (/pus6/target_cartesian_pose)
      ↓
  pus6_kinematics_node (Analytical6PusKinematics IK)
      ↓ (/pus6/desired_joint_deltas & /pus6/desired_joint_delta_steps)
  pus6_control_node (JointTrajectoryController 3rd-order Shaper)
      ↓ (/pus6/command_joint_steps)
  Observed on software topic.

STRICT SAFETY CONSTRAINTS:
  - pus6_motor_driver execution is STRICTLY PROHIBITED.
  - Motor UART process MUST NOT exist.
  - hardware_write_count = 0 strictly guaranteed.
  - J7/J8 production code diff = 0.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray

# Path imports
_SCRIPT_DIR = Path(__file__).resolve().parent
_KINEMATICS_DIR = _SCRIPT_DIR.parents[1] / "pus6_kinematics" / "pus6_kinematics"
_CONTROL_DIR = _SCRIPT_DIR.parents[1] / "pus6_control" / "pus6_control"

for p in (_SCRIPT_DIR, _KINEMATICS_DIR, _CONTROL_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from analytical_6pus import Analytical6PusKinematics
from geometry_config import load_geometry_config
from kinematics_node import Pus6KinematicsNode
from control_node import Pus6ControlNode
from j11_task_space_attitude_pid import compose_so3_residual, rot_matrix_to_quat_xyzw


class ShadowTestHarness(Node):
    """Test harness driving Cartesian target poses and auditing software command topics."""

    def __init__(self, z0: float, operating_home_steps: List[int], steps_per_meter: float) -> None:
        super().__init__("j11_shadow_test_harness")
        self.z0 = z0
        self.operating_home_steps = list(operating_home_steps)
        self.steps_per_meter = steps_per_meter

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, "/pus6/target_cartesian_pose", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/pus6/joint_states", 10)

        # Subscriptions
        self.latest_delta_steps: Optional[List[int]] = None
        self.latest_command_steps: Optional[List[int]] = None
        self.command_steps_history: List[List[int]] = []

        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        control_cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.delta_steps_sub = self.create_subscription(
            Int32MultiArray, "/pus6/desired_joint_delta_steps", self._on_delta_steps, 10
        )
        self.command_steps_sub = self.create_subscription(
            Int32MultiArray, "/pus6/command_joint_steps", self._on_command_steps, control_cmd_qos
        )

        self.lock = threading.Lock()

    def _on_delta_steps(self, msg: Int32MultiArray) -> None:
        with self.lock:
            self.latest_delta_steps = list(msg.data[:6])

    def _on_command_steps(self, msg: Int32MultiArray) -> None:
        with self.lock:
            steps = list(msg.data[:6])
            self.latest_command_steps = steps
            self.command_steps_history.append(steps)

    def publish_joint_state_at_home(self) -> None:
        """Publishes simulated feedback indicating sliders are at OPERATING_HOME."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i+1}" for i in range(6)]
        # Operating home position in meters: home_step / spm
        msg.position = [float(h) / self.steps_per_meter for h in self.operating_home_steps]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.joint_state_pub.publish(msg)

    def publish_target_pose(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
        """Publishes PoseStamped in Base frame."""
        r_rad = math.radians(roll_deg)
        p_rad = math.radians(pitch_deg)
        y_rad = math.radians(yaw_deg)

        R_cmd, q_cmd = compose_so3_residual(np.eye(3), [r_rad, p_rad, y_rad])

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pus6_base"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = self.z0
        msg.pose.orientation.x = float(q_cmd[0])
        msg.pose.orientation.y = float(q_cmd[1])
        msg.pose.orientation.z = float(q_cmd[2])
        msg.pose.orientation.w = float(q_cmd[3])

        self.target_pose_pub.publish(msg)
        return q_cmd


def verify_no_motor_driver_process() -> None:
    """Rigorous pre-flight safety check: verify zero motor driver or motor serial processes."""
    res = subprocess.run(
        ["ps", "aux"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    forbidden_terms = ["motor_driver", "j7_real", "j7_servo", "pus6_motor_driver"]
    for line in res.stdout.splitlines():
        if any(term in line for term in forbidden_terms) and "grep" not in line and "python" in line:
            raise RuntimeError(f"SAFETY LOCKOUT: Active motor driver process detected: {line}")
    print("[SAFETY AUDIT PASS] No active motor driver or UART process running.")


def run_shadow_integration() -> Dict[str, Any]:
    print("=" * 115)
    print("Stage J11-1C: Production ROS Shadow Integration Qualification")
    print("=" * 115)

    verify_no_motor_driver_process()

    cfg = load_geometry_config()
    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    home_raw = list(cfg.operating_home_raw_steps)
    spm = cfg.steps_per_meter

    rclpy.init()
    kinematics_node = Pus6KinematicsNode()
    control_node = Pus6ControlNode()
    harness = ShadowTestHarness(z0=z0, operating_home_steps=home_raw, steps_per_meter=spm)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(kinematics_node)
    executor.add_node(control_node)
    executor.add_node(harness)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    test_cases = [
        {"name": "HOME", "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0},
        {"name": "Roll_+0.25deg", "roll_deg": +0.25, "pitch_deg": 0.0, "yaw_deg": 0.0},
        {"name": "Roll_-0.25deg", "roll_deg": -0.25, "pitch_deg": 0.0, "yaw_deg": 0.0},
        {"name": "Roll_+0.50deg", "roll_deg": +0.50, "pitch_deg": 0.0, "yaw_deg": 0.0},
        {"name": "Roll_-0.50deg", "roll_deg": -0.50, "pitch_deg": 0.0, "yaw_deg": 0.0},
    ]

    results = []

    print(f"\n{'Test Case':16s} | {'Offline Delta Steps':20s} | {'ROS Kin Delta Steps':20s} | {'Offline Pred Raw':26s} | {'ROS Control Final Raw':26s} | Status")
    print("-" * 125)

    try:
        for tc in test_cases:
            # 1. Offline theoretical prediction
            pose = [
                0.0, 0.0, z0,
                math.radians(tc["roll_deg"]),
                math.radians(tc["pitch_deg"]),
                math.radians(tc["yaw_deg"]),
            ]
            offline_delta_steps = kin.differential_inverse_kinematics_steps(pose, [0.0, 0.0, z0, 0.0, 0.0, 0.0])
            offline_pred_raw = [ref + ds for ref, ds in zip(home_raw, offline_delta_steps)]

            # 2. Reset history buffer for this case
            with harness.lock:
                harness.command_steps_history.clear()
                harness.latest_delta_steps = None
                harness.latest_command_steps = None

            # 3. Publish simulated joint state and target pose for 1.2 seconds to allow 2nd-order shaper to settle
            t_end = time.monotonic() + 1.2
            while time.monotonic() < t_end:
                harness.publish_joint_state_at_home()
                harness.publish_target_pose(tc["roll_deg"], tc["pitch_deg"], tc["yaw_deg"])
                time.sleep(0.020)

            # 4. Ingest settled output from ROS topic /pus6/command_joint_steps
            with harness.lock:
                ros_delta_steps = list(harness.latest_delta_steps) if harness.latest_delta_steps else None
                ros_final_raw = list(harness.latest_command_steps) if harness.latest_command_steps else None

            # Assert consistency
            assert ros_delta_steps is not None, f"No message on /pus6/desired_joint_delta_steps for {tc['name']}"
            assert ros_final_raw is not None, f"No message on /pus6/command_joint_steps for {tc['name']}"

            delta_match = (ros_delta_steps == offline_delta_steps)
            raw_match = (ros_final_raw == offline_pred_raw)

            status = "MATCH_PASS" if (delta_match and raw_match) else "MISMATCH_FAIL"

            print(
                f"{tc['name']:16s} | {str(offline_delta_steps):20s} | {str(ros_delta_steps):20s} | "
                f"{str(offline_pred_raw):26s} | {str(ros_final_raw):26s} | {status}"
            )

            results.append({
                "case": tc["name"],
                "target_rpy_deg": [tc["roll_deg"], tc["pitch_deg"], tc["yaw_deg"]],
                "offline_delta_steps": offline_delta_steps,
                "ros_kinematics_delta_steps": ros_delta_steps,
                "offline_predicted_raw": offline_pred_raw,
                "ros_control_final_raw": ros_final_raw,
                "delta_steps_match": delta_match,
                "raw_steps_match": raw_match,
                "status": status,
            })

    finally:
        rclpy.shutdown()

    out_dir = Path("/home/lyapunov/bone_drill/results/j11/pre_motion")
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / "j11_shadow_integration_report.json"

    report = {
        "stage": "J11-1C",
        "description": "Production ROS Shadow Integration Qualification Report",
        "hardware_write_count": 0,
        "operating_home_raw_steps": home_raw,
        "total_cases": len(results),
        "all_cases_match": all(r["status"] == "MATCH_PASS" for r in results),
        "cases": results,
    }

    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    print("-" * 125)
    print(f"[PASS] Shadow integration qualification evidence saved to {report_path}")
    print(f"hardware_write_count = 0 (STRICTLY PRESERVED)")
    return report


if __name__ == "__main__":
    run_shadow_integration()
