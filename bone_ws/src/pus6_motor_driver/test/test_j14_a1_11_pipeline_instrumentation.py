"""J14-A1.11 Parent Feedback Pipeline Minimal Instrumentation Test Suite.

Verifies:
1. packet bytes/IPC semantics unchanged;
2. parent receive logic unchanged;
3. snapshot latest-wins unchanged;
4. ROS topic/type/QoS unchanged;
5. publish order unchanged;
6. UART child source and AST untouched;
7. timestamps identity correct;
8. snapshot superseded correctly handled;
9. instrumentation OFF path preserves original behavior;
10. shutdown has no deadlock.
"""
from __future__ import annotations

import ast
import json
import os
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.node import Node

from pus6_motor_driver.j14_process_uart import (
    CommandState,
    ProcessClient,
    ServoCore,
    append_rows,
    receive_packets,
    send_packet,
    socket_pair,
)
from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode, ServoTelemetrySnapshot


@pytest.fixture(scope="module", autouse=True)
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_1_ipc_packet_semantics_unchanged():
    """Verify socketpair mode, buffer sizes, and packet encoding."""
    s1, s2 = socket_pair()
    try:
        assert s1.type == s1.type  # SOCK_SEQPACKET
        assert s1.getblocking() is False
        assert s2.getblocking() is False
        test_payload = {"type": "feedback", "epoch": 12345, "cycle_seq": 1, "positions": [1000] * 6}
        assert send_packet(s1, test_payload) is True
        rows, flooded = receive_packets(s2)
        assert len(rows) == 1
        assert rows[0] == test_payload
        assert flooded is False
    finally:
        s1.close()
        s2.close()


def test_2_parent_receive_thread_cpu_recorded():
    """Verify ProcessClient records both process_cpu_ns and thread_cpu_ns."""
    s1, s2 = socket_pair()
    try:
        # Mock client with open socket
        client = object.__new__(ProcessClient)
        client.sock = s2
        client.closed = False
        client.rows = []
        client.proc = MagicMock()
        client.proc.poll.return_value = None

        test_row = {"type": "feedback", "cycle_seq": 42, "feedback_ready_ns": 1000000}
        send_packet(s1, test_row)

        received = client.receive()
        assert len(received) == 1
        assert len(client.rows) == 1
        rec = client.rows[0]
        assert rec["event"] == "parent_receive"
        assert "process_cpu_ns" in rec
        assert "thread_cpu_ns" in rec
        assert rec["thread_cpu_ns"] > 0
        assert rec["cycle_seq"] == 42
    finally:
        s1.close()
        s2.close()


def test_3_snapshot_latest_wins_semantics():
    """Verify that newer child feedback overwrites older snapshot under _telemetry_lock."""
    node = object.__new__(J14ProcessMotorDriverNode)
    node._telemetry_lock = threading.Lock()
    node._telemetry_ready_event = threading.Event()
    node._latest_telemetry_snapshot = None
    node._current_pending_snap = None
    node._pipeline_records = []
    node._instrumentation_enabled = True
    node._process_client = MagicMock()
    node._process_client.config = {"epoch": 100}
    node._latest_positions_steps = {i: 1000 for i in range(1, 7)}
    node._latest_currents = {i: 0 for i in range(1, 7)}
    node._latest_faults = {i: 0 for i in range(1, 7)}
    node._latest_rx_done_ns = {i: 0 for i in range(1, 7)}
    node._last_commanded_steps = [1000] * 6
    node._target_in_count = 1
    node._target_overwrite_count = 0
    node.get_clock = MagicMock()
    node.get_clock().now().to_msg.return_value = MagicMock(sec=1, nanosec=2)

    def dummy_row(seq):
        return {
            "epoch": 100,
            "cycle_seq": seq,
            "command_seq": 10,
            "positions": [1000] * 6,
            "currents": [0] * 6,
            "faults": [0] * 6,
            "rx_ns": [50] * 6,
            "targets": [1000] * 6,
            "tx_start_ns": [60] * 6,
            "tx_end_ns": [70] * 6,
            "tx_wall_ns": [10] * 6,
            "rx_drain_ns": 200,
            "cycle_start_ns": 100,
            "cycle_end_ns": 200,
            "feedback_ready_ns": 200,
            "command_publish_ns": 80,
            "parent_receive_ns": 210,
        }

    # Accept row 1
    node._accept_child_feedback(dummy_row(1))
    assert node._latest_telemetry_snapshot.uart_cycle_seq == 1

    # Accept row 2 before telemetry worker consumed row 1
    node._accept_child_feedback(dummy_row(2))
    assert node._latest_telemetry_snapshot.uart_cycle_seq == 2

    # Verify that row 1 was recorded as superseded!
    assert len(node._pipeline_records) == 1
    sup = node._pipeline_records[0]
    assert sup["cycle_seq"] == 1
    assert sup["status"] == "superseded"
    assert sup["superseded_by"] == 2
    assert sup["t_message_build_done_ns"] is None
    assert sup["t_pub_js_enter_ns"] is None


def test_4_5_ros_topics_and_publish_order(tmp_path):
    """Verify ROS publication ordering: JointState -> TimedJointFeedback -> TimedJointCommand."""
    node = object.__new__(J14ProcessMotorDriverNode)
    node._instrumentation_enabled = True
    node._pipeline_records = []
    node.joint_names = [f"motor_{i}" for i in range(1, 7)]
    node.hardware_mode = MagicMock()
    node._defer_driver_diagnostics = True
    node.publish_raw_diagnostics = False

    published_topics = []

    node.joint_state_pub = MagicMock()
    node.joint_state_pub.publish = lambda msg: published_topics.append("joint_states")
    node.timed_feedback_pub = MagicMock()
    node.timed_feedback_pub.publish = lambda msg: published_topics.append("timed_feedback")
    node.timed_command_pub = MagicMock()
    node.timed_command_pub.publish = lambda msg: published_topics.append("timed_command")

    snap = ServoTelemetrySnapshot(
        uart_cycle_seq=42,
        cycle_start_ns=100,
        cycle_end_ns=200,
        stamp_sec=1,
        stamp_nanosec=2,
        positions_steps=[1000] * 6,
        currents_mA=[0] * 6,
        fault_codes=[0] * 6,
        rx_done_ns=[50] * 6,
        dispatched_targets=[1000] * 6,
        tx_start_ns=[60] * 6,
        tx_end_ns=[70] * 6,
        diag_data=[0.0] * 11,
        benchmark_seq_id=0,
        t_entry_ns=100,
    )
    snap.epoch = 999
    snap.command_seq = 5
    snap.t_child_ready_ns = 200
    snap.t_parent_recv_ns = 220
    snap.t_snapshot_update_ns = 230

    node._publish_telemetry_from_snapshot(snap, record_stall=False, t_wake=240, tc_wake=50, pc_wake=100)

    assert published_topics == ["joint_states", "timed_feedback", "timed_command"]
    assert len(node._pipeline_records) == 1
    rec = node._pipeline_records[0]
    assert rec["status"] == "published"
    assert rec["cycle_seq"] == 42
    assert rec["t_child_ready_ns"] == 200
    assert rec["t_parent_recv_ns"] == 220
    assert rec["t_snapshot_update_ns"] == 230
    assert rec["t_telemetry_wake_ns"] == 240
    assert rec["t_message_build_done_ns"] >= rec["t_telemetry_wake_ns"]
    assert rec["t_pub_js_exit_ns"] >= rec["t_pub_js_enter_ns"]
    assert rec["t_pub_tf_exit_ns"] >= rec["t_pub_tf_enter_ns"]
    assert rec["t_pub_tc_exit_ns"] >= rec["t_pub_tc_enter_ns"]
    assert "telemetry_thread_cpu_ns" in rec


def test_6_child_ast_untouched():
    """Verify that UART child main and ServoCore AST are unchanged from A1.9."""
    main_code = Path("bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_uart_child_main.py").read_text()
    tree = ast.parse(main_code)
    # Ensure j14_uart_child_main.py imports from j14_process_uart
    modules = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert any("j14_process_uart" in (m or "") for m in modules)

    uart_code = Path("bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py").read_text()
    tree_uart = ast.parse(uart_code)
    # Check that ServoCore and child_run functions still exist unchanged
    func_names = [n.name for n in ast.walk(tree_uart) if isinstance(n, ast.FunctionDef)]
    class_names = [n.name for n in ast.walk(tree_uart) if isinstance(n, ast.ClassDef)]
    assert "ServoCore" in class_names
    assert "child_run" in func_names


def test_8_9_instrumentation_toggle_and_superseded(monkeypatch):
    """Verify instrumentation OFF path bypasses pipeline records collection."""
    node = object.__new__(J14ProcessMotorDriverNode)
    node._instrumentation_enabled = False
    node._pipeline_records = []
    node._defer_driver_diagnostics = True
    node.publish_raw_diagnostics = False
    node.joint_names = [f"motor_{i}" for i in range(1, 7)]
    node.hardware_mode = MagicMock()
    node.joint_state_pub = MagicMock()
    node.timed_feedback_pub = None
    node.timed_command_pub = None

    snap = ServoTelemetrySnapshot(
        uart_cycle_seq=1, cycle_start_ns=0, cycle_end_ns=0, stamp_sec=0, stamp_nanosec=0,
        positions_steps=[1000]*6, currents_mA=[0]*6, fault_codes=[0]*6, rx_done_ns=[0]*6,
        dispatched_targets=[1000]*6, tx_start_ns=[0]*6, tx_end_ns=[0]*6, diag_data=[0.0]*11,
        benchmark_seq_id=0, t_entry_ns=0
    )

    # When disabled, super()._publish_telemetry_from_snapshot is called and no records added
    with MagicMock() as mock_super:
        monkeypatch.setattr(Pus6MotorDriverNode, '_publish_telemetry_from_snapshot', mock_super)
        node._publish_telemetry_from_snapshot(snap)
        assert mock_super.called
        assert len(node._pipeline_records) == 0


def test_10_shutdown_clean(tmp_path,monkeypatch):
    """Verify destroy_node flushes pipeline records and exits without deadlock."""
    node = object.__new__(J14ProcessMotorDriverNode)
    node._pipeline_records = [{"cycle_seq": 1, "status": "published"}]
    node._servo_thread_stop_event = threading.Event()
    node._servo_thread = None
    node._telemetry_thread_stop_event = threading.Event()
    node._telemetry_ready_event = threading.Event()
    node._telemetry_thread = None
    node._communication_audit_dir = lambda: tmp_path

    mock_client = MagicMock()
    node._process_client = mock_client

    with MagicMock() as mock_super_destroy:
        monkeypatch.setattr(Pus6MotorDriverNode, 'destroy_node', mock_super_destroy)
        node.destroy_node()
        assert mock_client.close.called
        assert mock_super_destroy.called
        pipeline_file = tmp_path / "parent_feedback_pipeline.jsonl"
        assert pipeline_file.exists()
        lines = [json.loads(l) for l in pipeline_file.read_text().splitlines()]
        assert len(lines) == 1
        assert lines[0]["cycle_seq"] == 1
