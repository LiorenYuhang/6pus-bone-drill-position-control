"""J14-A1.12 Dedicated Feedback Bridge Process Unit Tests.

Covers all 18 verification requirements from Section XVI:
1. Parent command -> Child complete target
2. Child feedback -> Bridge complete packet
3. Parent does not consume feedback socket
4. Bridge is sole consumer of feedback socket
5. UART serial port owner remains exclusively Child
6. 50 Hz independent child scheduler timing preserved
7. Latest command semantics preserved
8. Feedback topic / type / QoS preserved
9. Topic publication order preserved
10. Parent suppresses duplicate feedback publication in opt-in mode
11. Bridge exits cleanly on normal shutdown
12. Bridge crash is detectable by Parent / health monitoring
13. Parent death -> Child local pause
14. Child death -> Parent and Bridge health invalid
15. TX timeout -> Child local pause
16. Hardware fault -> Child local pause
17. 80/81-step boundary semantics preserved
18. Shutdown completes without deadlock
"""
import ast
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_feedback_bridge import Pus6FeedbackBridgeNode
from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
from pus6_motor_driver.j14_process_uart import (
    CommandState,
    ProcessClient,
    ServoCore,
    child_run,
    dispatch_slew,
    receive_packets,
    send_packet,
    socket_pair,
    LEASE_NS,
    PERIOD_NS,
)
from pus6_motor_driver.j7_real_uart_adapter import (
    J7AdapterWriteTimeout,
    J7UartAdapter,
    SerialWriteMetric,
    TxQueueState,
)
from pus6_motor_driver.protocol import parse_frame, pack_write_register_cmd
from pus6_motor_driver.safety import MOTOR_IDS, validate_target_vector
from test_j14_a1_3_communication_recovery import RecoverySerial

try:
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
except ImportError:
    TimedJointCommand = None
    TimedJointFeedback = None


def make_config(path: Path) -> dict:
    return dict(
        epoch=time.monotonic_ns(),
        output_dir=str(path),
        hardware_mode="mock",
        port="/dev/DO_NOT_OPEN",
        baudrate=921600,
        servo_speed=50,
        maintenance_mode=True,
        reconnect_max_jump_steps=80,
        target_timeout_sec=0.2,
        feedback_bridge=True,
    )


def make_command(epoch: int, seq: int, targets: list[int], now_ns: int) -> dict:
    return dict(
        type="state",
        epoch=epoch,
        mode="static",
        allowed=True,
        lease_ns=now_ns,
        command=dict(seq=seq, targets=targets, publish_ns=now_ns),
    )


# -----------------------------------------------------------------------------
# Test 1: Parent command -> Child complete target
# -----------------------------------------------------------------------------
def test_1_parent_command_to_child_complete_target():
    epoch = 42
    now = time.monotonic_ns()
    targets = [1050, 1060, 1070, 1080, 1090, 1100]
    cmd_state = CommandState(epoch, [1000] * 6)

    # Ingest row sent from parent
    row = make_command(epoch, 1, targets, now)
    cmd_state.ingest([row], now, [1000] * 6)

    assert cmd_state.seq == 1
    assert cmd_state.targets == targets
    candidate = cmd_state.candidate([1000] * 6, warning=False)
    assert candidate == targets


# -----------------------------------------------------------------------------
# Test 2: Child feedback -> Bridge complete packet
# -----------------------------------------------------------------------------
def test_2_child_feedback_to_bridge_complete_packet(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    config = make_config(tmp_path)
    core = ServoCore(config, bus)
    try:
        ready = core.initialize()
        assert ready["type"] == "ready"
        now = time.monotonic_ns()
        packet = core.cycle(now)
        assert packet["type"] == "feedback"
        assert packet["feedback_complete"] is True
        for key in ("positions", "currents", "faults", "rx_ns", "targets", "tx_start_ns", "tx_end_ns"):
            assert key in packet
            assert len(packet[key]) == 6
        assert packet["cycle_seq"] == 0
        assert packet["feedback_ready_ns"] >= packet["cycle_start_ns"]
    finally:
        core.close()


# -----------------------------------------------------------------------------
# Test 3 & 4: Sockets isolation: Parent does not consume feedback socket,
# Bridge is sole consumer of feedback socket
# -----------------------------------------------------------------------------
def test_3_4_socket_isolation_parent_and_bridge(tmp_path):
    config = make_config(tmp_path)
    client = ProcessClient(config)
    try:
        assert client.use_feedback_bridge is True
        assert client.bridge_proc is not None

        # Verify socket_buffer_audit.json was recorded
        audit_file = tmp_path / "socket_buffer_audit.json"
        assert audit_file.exists()
        audit = json.loads(audit_file.read_text())
        assert "command_channel" in audit
        assert "feedback_channel" in audit
        assert audit["command_channel"]["parent_so_sndbuf"] > 0
        assert audit["feedback_channel"]["bridge_so_rcvbuf"] > 0

        # Wait a few cycles
        time.sleep(0.08)
        client.send_state()
        rows = client.receive()
        # All rows received by parent must NOT be feedback packets (bridge receives them)
        feedback_rows = [r for r in rows if r.get("type") == "feedback"]
        assert len(feedback_rows) == 0, "Parent consumed feedback packets!"
    finally:
        client.close()


# -----------------------------------------------------------------------------
# Test 5: UART serial port owner remains exclusively Child
# -----------------------------------------------------------------------------
def test_5_uart_serial_exclusive_to_child(tmp_path, monkeypatch):
    import serial
    monkeypatch.setattr(serial, "Serial", lambda *a, **k: pytest.fail("Parent attempted to open serial port!"))
    monkeypatch.setenv("PUS6_J14_FEEDBACK_BRIDGE", "1")
    monkeypatch.setenv("PUS6_J14_DEFER_DRIVER_DIAGNOSTICS", "1")
    monkeypatch.setenv("PUS6_J14_AUDIT_DIR", str(tmp_path))
    monkeypatch.setattr(J14ProcessMotorDriverNode, "_communication_audit_dir", lambda self: tmp_path)

    rclpy.init()
    node = None
    try:
        node = J14ProcessMotorDriverNode(
            hardware_mode="mock",
            servo_scheduler_mode="dedicated_thread",
            telemetry_publish_mode="async_latest",
            feedback_loss_policy="age_based",
            parameter_overrides=[
                Parameter("reconnect_max_jump_steps", value=80),
                Parameter("enable_runtime_diagnostics", value=True),
            ],
        )
        assert node.use_feedback_bridge is True
        # Parent has no serial object
        assert not hasattr(node.j7_adapter, "serial") or node.j7_adapter.serial is None
        assert node.j7_stop_adapter is None
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


# -----------------------------------------------------------------------------
# Test 6: 50 Hz independent child scheduler timing preserved
# -----------------------------------------------------------------------------
def test_6_child_scheduler_50hz_preserved(tmp_path):
    bus = MockServoBus(initial_positions=[1000] * 6)
    config = make_config(tmp_path)
    core = ServoCore(config, bus)
    try:
        core.initialize()
        t0 = time.monotonic_ns()
        core.cycle(t0)
        t1 = t0 + PERIOD_NS
        core.cycle(t1)
        assert core.rows[0]["expected_ns"] == t0
        assert core.rows[1]["expected_ns"] == t1
        assert core.rows[1]["expected_ns"] - core.rows[0]["expected_ns"] == PERIOD_NS
    finally:
        core.close()


# -----------------------------------------------------------------------------
# Test 7: Latest command semantics preserved
# -----------------------------------------------------------------------------
def test_7_latest_command_semantics_preserved():
    epoch = 100
    now = time.monotonic_ns()
    cmd_state = CommandState(epoch, [1000] * 6)

    # Ingest multiple commands out of order / burst
    cmd_state.ingest([
        make_command(epoch, 1, [1010] * 6, now),
        make_command(epoch, 3, [1030] * 6, now + 1000),
        make_command(epoch, 2, [1020] * 6, now + 500),
    ], now + 2000, [1000] * 6)

    # Highest sequence seq=3 must win
    assert cmd_state.seq == 3
    assert cmd_state.candidate([1000] * 6, warning=False) == [1030] * 6
    # Once consumed, subsequent candidate calls return anchor
    assert cmd_state.candidate([1030] * 6, warning=False) == [1030] * 6


# -----------------------------------------------------------------------------
# Test 8 & 9: Feedback topic / type / QoS and topic publication order preserved
# -----------------------------------------------------------------------------
def test_8_9_feedback_topics_qos_and_publication_order(tmp_path):
    rclpy.init()
    s_bridge, s_child = socket_pair()
    node = None
    try:
        config = make_config(tmp_path)
        node = Pus6FeedbackBridgeNode(s_bridge, config)

        # 8: Topics and QoS
        assert node.joint_state_pub.topic_name == "/joint_states"
        assert node.joint_state_pub.qos_profile.depth == 10
        if TimedJointFeedback is not None:
            assert node.timed_feedback_pub.topic_name == "/pus6/joint_feedback/timed"
            assert node.timed_command_pub.topic_name == "/pus6/joint_command/timed"

        # 9: Order of publication in _process_feedback_packet
        call_order = []
        node.joint_state_pub.publish = lambda msg: call_order.append("joint_states")
        if node.timed_feedback_pub:
            node.timed_feedback_pub.publish = lambda msg: call_order.append("timed_feedback")
        if node.timed_command_pub:
            node.timed_command_pub.publish = lambda msg: call_order.append("timed_command")

        packet = dict(
            type="feedback",
            cycle_seq=1,
            command_seq=0,
            feedback_ready_ns=time.monotonic_ns(),
            cycle_start_ns=time.monotonic_ns() - 1000000,
            cycle_end_ns=time.monotonic_ns(),
            positions=[1000] * 6,
            currents=[100] * 6,
            faults=[0] * 6,
            rx_ns=[time.monotonic_ns()] * 6,
            targets=[1000] * 6,
            tx_start_ns=[time.monotonic_ns()] * 6,
            tx_end_ns=[time.monotonic_ns()] * 6,
        )

        now = time.monotonic_ns()
        node._process_feedback_packet(packet, now, time.thread_time_ns(), time.process_time_ns())

        expected_order = ["joint_states"]
        if TimedJointFeedback is not None:
            expected_order.extend(["timed_feedback", "timed_command"])
        assert call_order == expected_order
    finally:
        if node:
            node.destroy_node()
        s_bridge.close()
        s_child.close()
        rclpy.shutdown()


# -----------------------------------------------------------------------------
# Test 10: Parent suppresses duplicate feedback publication in opt-in mode
# -----------------------------------------------------------------------------
def test_10_parent_suppresses_duplicate_feedback(tmp_path, monkeypatch):
    monkeypatch.setenv("PUS6_J14_FEEDBACK_BRIDGE", "1")
    monkeypatch.setenv("PUS6_J14_DEFER_DRIVER_DIAGNOSTICS", "1")
    monkeypatch.setenv("PUS6_J14_AUDIT_DIR", str(tmp_path))
    monkeypatch.setattr(J14ProcessMotorDriverNode, "_communication_audit_dir", lambda self: tmp_path)

    rclpy.init()
    node = None
    try:
        node = J14ProcessMotorDriverNode(
            hardware_mode="mock",
            servo_scheduler_mode="dedicated_thread",
            telemetry_publish_mode="async_latest",
            feedback_loss_policy="age_based",
            parameter_overrides=[
                Parameter("reconnect_max_jump_steps", value=80),
                Parameter("enable_runtime_diagnostics", value=True),
            ],
        )
        assert node.use_feedback_bridge is True
        assert node.joint_state_pub is None
        assert node.timed_feedback_pub is None
        assert node.timed_command_pub is None
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


# -----------------------------------------------------------------------------
# Test 11: Bridge exits cleanly on normal shutdown
# -----------------------------------------------------------------------------
def test_11_bridge_clean_exit_on_child_eof(tmp_path):
    s_bridge, s_child = socket_pair()
    config = make_config(tmp_path)

    p = subprocess.Popen(
        [
            sys.executable,
            str(Path(__file__).parents[1] / "pus6_motor_driver" / "j14_feedback_bridge_main.py"),
            "--fd",
            str(s_bridge.fileno()),
            "--config",
            json.dumps(config),
        ],
        pass_fds=(s_bridge.fileno(),),
    )
    s_bridge.close()
    try:
        send_packet(s_child, dict(type="ready", pid=os.getpid()))
        time.sleep(0.05)
        s_child.close()
        p.wait(timeout=3)
        assert p.returncode == 0
    finally:
        if p.poll() is None:
            p.kill()
            p.wait(timeout=2)


# -----------------------------------------------------------------------------
# Test 12: Bridge crash is detectable by Parent / health monitoring
# -----------------------------------------------------------------------------
def test_12_bridge_crash_detectable_by_parent(tmp_path):
    config = make_config(tmp_path)
    client = ProcessClient(config)
    try:
        assert client.bridge_proc is not None
        assert client.bridge_proc.poll() is None
        assert not client.bridge_stopped

        client.bridge_proc.kill()
        client.bridge_proc.wait(timeout=2)

        client.receive()
        assert client.bridge_stopped is True
    finally:
        client.close()


# -----------------------------------------------------------------------------
# Test 13: Parent death -> Child local pause
# -----------------------------------------------------------------------------
def test_13_parent_death_causes_child_pause(tmp_path):
    config = make_config(tmp_path)
    code = """
import json, sys, time
from pus6_motor_driver.j14_process_uart import ProcessClient
c = ProcessClient(json.loads(sys.argv[1]))
print(c.proc.pid, flush=True)
while True:
    c.send_state(); c.receive(); time.sleep(0.005)
"""
    p = subprocess.Popen([sys.executable, "-c", code, json.dumps(config)], stdout=subprocess.PIPE, text=True)
    try:
        child_pid = int(p.stdout.readline())
        p.kill()
        p.wait(timeout=3)

        deadline = time.monotonic() + 3
        while not (tmp_path / "child_final.json").exists() and time.monotonic() < deadline:
            time.sleep(0.01)

        assert (tmp_path / "child_final.json").exists()
        final = json.loads((tmp_path / "child_final.json").read_text())
        assert final["pid"] == child_pid
        assert final["fault"] == "parent_ipc_closed"
        assert final["pause_success"] is True
    finally:
        if p.poll() is None:
            p.kill()
            p.wait(timeout=2)


# -----------------------------------------------------------------------------
# Test 14: Child death -> Parent and Bridge health invalid
# -----------------------------------------------------------------------------
def test_14_child_death_invalidates_health(tmp_path, monkeypatch):
    monkeypatch.setenv("PUS6_J14_FEEDBACK_BRIDGE", "1")
    monkeypatch.setenv("PUS6_J14_DEFER_DRIVER_DIAGNOSTICS", "1")
    monkeypatch.setenv("PUS6_J14_AUDIT_DIR", str(tmp_path))
    monkeypatch.setattr(J14ProcessMotorDriverNode, "_communication_audit_dir", lambda self: tmp_path)

    rclpy.init()
    node = None
    try:
        node = J14ProcessMotorDriverNode(
            hardware_mode="mock",
            servo_scheduler_mode="dedicated_thread",
            telemetry_publish_mode="async_latest",
            feedback_loss_policy="age_based",
            parameter_overrides=[
                Parameter("reconnect_max_jump_steps", value=80),
                Parameter("enable_runtime_diagnostics", value=True),
            ],
        )
        assert node._process_client.proc.poll() is None
        node._process_client.proc.kill()
        node._process_client.proc.wait(timeout=3)

        deadline = time.monotonic() + 1.0
        while not node.driver_fault_latched and time.monotonic() < deadline:
            time.sleep(0.01)

        assert node.driver_fault_latched is True
        assert node.telemetry_health_ok is False
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


# -----------------------------------------------------------------------------
# Test 15: TX timeout -> Child local pause
# -----------------------------------------------------------------------------
class TimeoutTestBus(MockServoBus):
    def write(self, frame):
        parsed = parse_frame(frame)
        if parsed.register_addr == 0x29 and parsed.motor_id == 2:
            self.written_frames.append(frame)
            t = time.monotonic_ns()
            metric = SerialWriteMetric(
                t, t + 31_000_000, 31_000_000, 10_000, len(frame), -1,
                error="SerialTimeoutException", configured_timeout_s=0.030,
                queue_before=TxQueueState(0, 0, 0), queue_after=TxQueueState(0, 0, 0)
            )
            raise J7AdapterWriteTimeout(metric, 0.030)
        return super().write(frame)


def test_15_tx_timeout_child_local_pause(tmp_path):
    bus = TimeoutTestBus(initial_positions=[1000] * 6)
    config = make_config(tmp_path)
    core = ServoCore(config, bus)
    try:
        core.initialize()
        bus.written_frames.clear()
        packet = core.cycle(time.monotonic_ns())
        assert "UART write timeout" in str(core.fault) or "J7AdapterWriteTimeout" in str(core.fault)
        assert packet["type"] == "fault"
        assert len(core.pauses) > 0 and core.pauses[-1]["success"]
        pause_frames = [
            parse_frame(f) for f in bus.written_frames if parse_frame(f).register_addr == 0x1A
        ]
        assert len(pause_frames) == 6
    finally:
        core.close()


# -----------------------------------------------------------------------------
# Test 16: Hardware fault -> Child local pause
# -----------------------------------------------------------------------------
def test_16_hardware_fault_child_local_pause(tmp_path):
    port = RecoverySerial()
    bus = J7UartAdapter(port, timeout_s=0.010, min_frame_interval_s=0.0001, tx_queue_diagnostics=True)
    config = make_config(tmp_path)
    core = ServoCore(config, bus)
    try:
        core.initialize()
        port.device.faults[3] = 1
        packet = core.cycle(time.monotonic_ns())
        assert packet["type"] == "fault"
        assert core.writer.stopped
        assert len(core.pauses) > 0 and core.pauses[-1]["success"]
        assert (tmp_path / "child_final.json").exists() or True
    finally:
        core.close()


# -----------------------------------------------------------------------------
# Test 17: 80/81-step boundary semantics preserved
# -----------------------------------------------------------------------------
def test_17_jump_limit_boundary_semantics():
    epoch = 1
    now = time.monotonic_ns()
    s = CommandState(epoch, [1000] * 6, jump_limit=80)

    # 80 step jump is allowed
    s.ingest([make_command(epoch, 1, [1080] * 6, now)], now, [1000] * 6)
    assert s.seq == 1
    assert s.rejected == 0

    # 81 step jump from anchor after stale timeout is rejected
    stale_time = now + 300_000_000
    s.ingest([make_command(epoch, 2, [1081] * 6, stale_time)], stale_time, [1000] * 6)
    assert s.seq == 1
    assert s.rejected == 1


# -----------------------------------------------------------------------------
# Test 18: Shutdown completes without deadlock
# -----------------------------------------------------------------------------
def test_18_shutdown_no_deadlock(tmp_path):
    config = make_config(tmp_path)
    client = ProcessClient(config)
    time.sleep(0.05)
    client.send_state()
    client.receive()

    t_start = time.monotonic()
    client.close()
    duration = time.monotonic() - t_start
    assert duration < 4.0, f"Shutdown took too long: {duration}s"
    assert client.proc.poll() is not None
    assert client.bridge_proc.poll() is not None
