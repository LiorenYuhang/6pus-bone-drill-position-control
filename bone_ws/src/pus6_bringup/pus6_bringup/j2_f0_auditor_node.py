"""Stage J2-F0 Independent Auditor Node for measuring ROS 2 delivery and end-to-end transport latency."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, Int64MultiArray


class J2F0AuditorNode(Node):
    """Auditor node running in an isolated OS process to measure ROS 2 delivery and transport latency."""

    def __init__(self, node_name: str = "j2_f0_auditor_node", output_json: str | None = None) -> None:
        super().__init__(node_name)
        self.output_json = output_json

        # Match the benchmark publisher's sensor-data QoS exactly.
        qos_profile = qos_profile_sensor_data

        self.joint_states_count = 0
        self.raw_steps_count = 0
        self.spin_heartbeat_count = 0
        self.benchmark_records: list[dict] = []
        self.heartbeat_timer = self.create_timer(0.1, self._on_heartbeat)

        self.sub_js = self.create_subscription(
            JointState, "/pus6/joint_states", self._on_joint_state, qos_profile
        )
        self.sub_steps = self.create_subscription(
            Int32MultiArray, "/pus6/joint_steps_raw", self._on_steps, qos_profile
        )
        self.sub_bench = self.create_subscription(
            Int64MultiArray, "/pus6/benchmark_stamp", self._on_benchmark_stamp, qos_profile
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_states_count += 1

    def _on_steps(self, msg: Int32MultiArray) -> None:
        self.raw_steps_count += 1

    def _on_heartbeat(self) -> None:
        self.spin_heartbeat_count += 1

    def _on_benchmark_stamp(self, msg: Int64MultiArray) -> None:
        t_recv_ns = time.perf_counter_ns()
        data = list(msg.data)
        if len(data) >= 7:
            seq_id = data[0]
            t_cb_start_ns = data[1]
            t_scan_start_ns = data[2]
            t_scan_end_ns = data[3]
            t_pub_start_ns = data[4]
            t_pub_end_ns = data[5]
            t_cb_end_ns = data[6]

            self.benchmark_records.append({
                "seq_id": seq_id,
                "t_cb_start_ns": t_cb_start_ns,
                "t_scan_start_ns": t_scan_start_ns,
                "t_scan_end_ns": t_scan_end_ns,
                "t_pub_start_ns": t_pub_start_ns,
                "t_pub_end_ns": t_pub_end_ns,
                "t_cb_end_ns": t_cb_end_ns,
                "t_recv_ns": t_recv_ns,
                "e2e_latency_ns": t_recv_ns - t_pub_start_ns,
                "scan_duration_ns": t_scan_end_ns - t_scan_start_ns,
                "publish_section_ns": t_pub_end_ns - t_pub_start_ns,
                "callback_duration_ns": t_cb_end_ns - t_cb_start_ns,
            })
        elif len(data) >= 2:
            seq_id = data[0]
            t_pub_ns = data[1]
            self.benchmark_records.append({
                "seq_id": seq_id,
                "t_pub_start_ns": t_pub_ns,
                "t_recv_ns": t_recv_ns,
                "e2e_latency_ns": t_recv_ns - t_pub_ns,
            })

    def dump_metrics_to_file(self) -> None:
        if self.output_json:
            out_p = Path(self.output_json)
            out_p.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "joint_states_count": self.joint_states_count,
                "raw_steps_count": self.raw_steps_count,
                "spin_heartbeat_count": self.spin_heartbeat_count,
                "benchmark_records_count": len(self.benchmark_records),
                "benchmark_records": self.benchmark_records,
            }
            with open(out_p, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Dumped {len(self.benchmark_records)} auditor records to {out_p}")


def main(args=None):
    parser = argparse.ArgumentParser(description="J2-F0 Auditor Node")
    parser.add_argument("--output-json", default=None, help="Path to write JSON results on exit")
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = J2F0AuditorNode(output_json=parsed_args.output_json)

    def _sigint_handler(sig, frame):
        node.dump_metrics_to_file()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.dump_metrics_to_file()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
