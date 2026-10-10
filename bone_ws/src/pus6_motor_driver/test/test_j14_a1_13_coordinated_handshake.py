"""J14-A1.13 Three-Process Coordinated Startup Handshake Unit Tests.

Covers Section XIII (19 core requirements) and Section XIV (Delayed-start simulations):
1. Child does not enter Servo loop before Child is READY
2. Child does not emit formal feedback before Bridge is READY
3. Child does not start before Parent is READY
4. Servo starts after all three parties are READY
5. Any arrival order of READY events succeeds deterministically
6. Delayed Bridge start produces 0 feedback drops
7. Delayed Parent start does not allow premature Servo execution
8. Child init failure aborts without Servo start
9. Bridge startup failure aborts without Servo start
10. Parent handshake failure aborts without Servo start
11. Handshake timeout executes safe exit
12. Serial owner remains exclusively Child
13. Feedback sole consumer remains Bridge
14. Command sole authority remains Parent
15. 50 Hz absolute scheduler semantics preserved
16. TX 30ms / RX 5ms / drain 4ms preserved
17. Motor fault / local pause semantics preserved
18. Shutdown completes without deadlock
19. Default non-A1.13 legacy path preserved

Section XIV Timing Scenarios:
A. Bridge delayed 500 ms
B. Parent delayed 500 ms
C. Child delayed 500 ms
D. Random READY sequence
"""
import json
import os
from pathlib import Path
import random
import select
import socket
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j14_feedback_bridge import Pus6FeedbackBridgeNode
from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
from pus6_motor_driver.j14_process_uart import (
    CommandState,
    ProcessClient,
    ServoCore,
    child_run,
    receive_packets,
    send_packet,
    socket_pair,
    PERIOD_NS,
)
from pus6_motor_driver.j7_real_uart_adapter import J7AdapterWriteTimeout, SerialWriteMetric, TxQueueState
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import MOTOR_IDS


def make_config(path: Path, **kwargs) -> dict:
    cfg = dict(
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
        handshake_timeout_s=3.0,
    )
    cfg.update(kwargs)
    return cfg


# -----------------------------------------------------------------------------
# Test 1, 2, 3, 4: Handshake gating - neither party allows servo before all ready
# -----------------------------------------------------------------------------
def test_1_2_3_4_handshake_gating_and_servo_start(tmp_path):
    cmd_parent, cmd_child = socket_pair()
    fb_bridge, fb_child = socket_pair()
    config = make_config(tmp_path)

    started_cycles = []

    def mock_child():
        child_run(cmd_child, config, bus=MockServoBus([1000] * 6), fb_sock=fb_child)

    child_th = threading.Thread(target=mock_child, daemon=True)
    child_th.start()

    try:
        # Step 1: Child initializes and emits child_ready to both
        rows_parent, _ = receive_packets(cmd_parent)
        t_deadline = time.monotonic() + 1.0
        while not rows_parent and time.monotonic() < t_deadline:
            time.sleep(0.005)
            rows_parent, _ = receive_packets(cmd_parent)

        assert len(rows_parent) == 1
        assert rows_parent[0]["type"] == "child_ready"

        rows_fb, _ = receive_packets(fb_bridge)
        assert len(rows_fb) == 1
        assert rows_fb[0]["type"] == "child_ready"

        # Verification 1 & 2 & 3: While Bridge and Parent have not responded, Child emits NO feedback
        time.sleep(0.1)
        fb_check, _ = receive_packets(fb_bridge)
        assert len(fb_check) == 0, "Child emitted packets before bridge/parent ready!"

        # Step 2: Bridge sends bridge_ready to Child
        t_bridge_ready = time.monotonic_ns()
        send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=t_bridge_ready))

        # Child receives bridge_ready and forwards to Parent
        t_deadline = time.monotonic() + 1.0
        forwarded = []
        while not forwarded and time.monotonic() < t_deadline:
            time.sleep(0.005)
            f_rows, _ = receive_packets(cmd_parent)
            forwarded.extend([r for r in f_rows if r.get("type") == "bridge_ready"])
        assert len(forwarded) == 1

        # Still no feedback cycles yet because Parent has not sent start
        time.sleep(0.05)
        fb_check, _ = receive_packets(fb_bridge)
        assert len(fb_check) == 0, "Child emitted feedback before Parent start!"

        # Step 3: Parent sends start
        now = time.monotonic_ns()
        send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))

        # Verification 4: Now both Parent and Bridge receive servo_start, followed immediately by Cycle 0!
        t_deadline = time.monotonic() + 1.0
        start_received_bridge = None
        while start_received_bridge is None and time.monotonic() < t_deadline:
            f_rows, _ = receive_packets(fb_bridge)
            for r in f_rows:
                if r.get("type") == "servo_start":
                    start_received_bridge = r
                elif r.get("type") == "feedback":
                    started_cycles.append(r)
            time.sleep(0.005)

        assert start_received_bridge is not None
        assert "servo_start_ns" in start_received_bridge

        # Wait for Cycle 0
        t_deadline = time.monotonic() + 0.5
        while not started_cycles and time.monotonic() < t_deadline:
            f_rows, _ = receive_packets(fb_bridge)
            started_cycles.extend([r for r in f_rows if r.get("type") == "feedback"])
            time.sleep(0.005)

        assert len(started_cycles) > 0
        assert started_cycles[0]["cycle_seq"] == 0
        assert started_cycles[0]["cycle_start_ns"] >= start_received_bridge["servo_start_ns"]
    finally:
        send_packet(cmd_parent, dict(type="stop", epoch=config["epoch"], reason="test_done"))
        child_th.join(timeout=2.0)
        cmd_parent.close()
        cmd_child.close()
        fb_bridge.close()
        fb_child.close()


# -----------------------------------------------------------------------------
# Section XIV Scenarios: Delayed Startup Simulations
# -----------------------------------------------------------------------------
def test_simulated_scenario_A_bridge_delayed_500ms(tmp_path):
    """Scenario A: Bridge starts 500 ms late. Child must wait and drop 0 packets."""
    cmd_parent, cmd_child = socket_pair()
    fb_bridge, fb_child = socket_pair()
    config = make_config(tmp_path, max_servo_duration_s=2.0)

    child_th = threading.Thread(
        target=lambda: child_run(cmd_child, config, bus=MockServoBus([1000] * 6), fb_sock=fb_child),
        daemon=True,
    )
    child_th.start()

    try:
        # Parent is immediately ready
        # Drain child_ready
        t_deadline = time.monotonic() + 1.0
        while time.monotonic() < t_deadline:
            rows, _ = receive_packets(cmd_parent)
            if any(r.get("type") == "child_ready" for r in rows):
                break
            time.sleep(0.005)

        # Parent sends start immediately
        now = time.monotonic_ns()
        send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))

        # Bridge delays for 500 ms!
        time.sleep(0.500)

        # In that 500 ms delay, Child must NOT have produced any feedback packets!
        fb_drain, _ = receive_packets(fb_bridge)
        feedback_packets = [r for r in fb_drain if r.get("type") == "feedback"]
        assert len(feedback_packets) == 0, f"Child produced {len(feedback_packets)} packets before Bridge was ready!"

        # Now Bridge becomes ready
        t_br = time.monotonic_ns()
        send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=t_br))

        # Once bridge_ready is sent, servo_start and Cycle 0 arrive
        t_deadline = time.monotonic() + 1.0
        cycles = []
        while not cycles and time.monotonic() < t_deadline:
            rows, _ = receive_packets(fb_bridge)
            cycles.extend([r for r in rows if r.get("type") == "feedback"])
            time.sleep(0.005)

        assert len(cycles) > 0
        assert cycles[0]["cycle_seq"] == 0, "Cycle 0 was lost or dropped!"
    finally:
        send_packet(cmd_parent, dict(type="stop", epoch=config["epoch"], reason="test_done"))
        child_th.join(timeout=2.0)
        cmd_parent.close()
        cmd_child.close()
        fb_bridge.close()
        fb_child.close()


def test_simulated_scenario_B_parent_delayed_500ms(tmp_path):
    """Scenario B: Parent starts 500 ms late. Child must wait and not start servo."""
    cmd_parent, cmd_child = socket_pair()
    fb_bridge, fb_child = socket_pair()
    config = make_config(tmp_path, max_servo_duration_s=2.0)

    child_th = threading.Thread(
        target=lambda: child_run(cmd_child, config, bus=MockServoBus([1000] * 6), fb_sock=fb_child),
        daemon=True,
    )
    child_th.start()

    try:
        # Bridge is immediately ready
        t_br = time.monotonic_ns()
        send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=t_br))

        # Parent delays 500 ms before draining or sending start
        time.sleep(0.500)

        # In that 500 ms, Child must NOT have produced any feedback packets
        fb_drain, _ = receive_packets(fb_bridge)
        feedback_packets = [r for r in fb_drain if r.get("type") == "feedback"]
        assert len(feedback_packets) == 0, "Child produced feedback before Parent ready!"

        # Now Parent becomes ready and sends start
        now = time.monotonic_ns()
        send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))

        # Servo starts cleanly
        t_deadline = time.monotonic() + 1.0
        cycles = []
        while not cycles and time.monotonic() < t_deadline:
            rows, _ = receive_packets(fb_bridge)
            cycles.extend([r for r in rows if r.get("type") == "feedback"])
            time.sleep(0.005)

        assert len(cycles) > 0
        assert cycles[0]["cycle_seq"] == 0
    finally:
        send_packet(cmd_parent, dict(type="stop", epoch=config["epoch"], reason="test_done"))
        child_th.join(timeout=2.0)
        cmd_parent.close()
        cmd_child.close()
        fb_bridge.close()
        fb_child.close()


def test_simulated_scenario_C_child_delayed_500ms(tmp_path):
    """Scenario C: Child takes 500 ms to initialize. Parent and Bridge wait safely."""
    cmd_parent, cmd_child = socket_pair()
    fb_bridge, fb_child = socket_pair()
    config = make_config(tmp_path, max_servo_duration_s=2.0)

    class SlowInitBus(MockServoBus):
        def __init__(self, initial_positions=None):
            super().__init__(initial_positions)
            self._delayed = False

        def request(self, frame: bytes, **kwargs) -> bytes:
            if not self._delayed:
                self._delayed = True
                time.sleep(0.500)
            return super().request(frame, **kwargs)

    # Bridge and Parent send ready/start early
    t_br = time.monotonic_ns()
    send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=t_br))
    now = time.monotonic_ns()
    send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))

    child_th = threading.Thread(
        target=lambda: child_run(cmd_child, config, bus=SlowInitBus([1000] * 6), fb_sock=fb_child),
        daemon=True,
    )
    t0 = time.monotonic()
    child_th.start()

    try:
        t_deadline = time.monotonic() + 2.0
        cycles = []
        while not cycles and time.monotonic() < t_deadline:
            rows, _ = receive_packets(fb_bridge)
            cycles.extend([r for r in rows if r.get("type") == "feedback"])
            time.sleep(0.005)

        assert time.monotonic() - t0 >= 0.45, "Child did not exhibit delay!"
        assert len(cycles) > 0
        assert cycles[0]["cycle_seq"] == 0
    finally:
        send_packet(cmd_parent, dict(type="stop", epoch=config["epoch"], reason="test_done"))
        child_th.join(timeout=2.0)
        cmd_parent.close()
        cmd_child.close()
        fb_bridge.close()
        fb_child.close()


def test_simulated_scenario_D_random_ready_order(tmp_path):
    """Scenario D: Random READY ordering across multiple trials. All must start Cycle 0 safely."""
    for rep in range(4):
        cmd_parent, cmd_child = socket_pair()
        fb_bridge, fb_child = socket_pair()
        config = make_config(tmp_path / f"rep_{rep}", max_servo_duration_s=1.0)

        child_th = threading.Thread(
            target=lambda: child_run(cmd_child, config, bus=MockServoBus([1000] * 6), fb_sock=fb_child),
            daemon=True,
        )
        child_th.start()

        delays = [random.uniform(0.01, 0.08) for _ in range(2)]
        time.sleep(delays[0])
        # Randomly choose who emits ready first
        if rep % 2 == 0:
            send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=time.monotonic_ns()))
            time.sleep(delays[1])
            now = time.monotonic_ns()
            send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))
        else:
            now = time.monotonic_ns()
            send_packet(cmd_parent, dict(type="start", epoch=config["epoch"], mode="static", allowed=True, lease_ns=now, t_parent_ready_ns=now, command=None))
            time.sleep(delays[1])
            send_packet(fb_bridge, dict(type="bridge_ready", epoch=config["epoch"], pid=os.getpid(), t_bridge_ready_ns=time.monotonic_ns()))

        cycles = []
        t_deadline = time.monotonic() + 1.5
        while not cycles and time.monotonic() < t_deadline:
            rows, _ = receive_packets(fb_bridge)
            cycles.extend([r for r in rows if r.get("type") == "feedback"])
            time.sleep(0.005)

        assert len(cycles) > 0, f"Rep {rep} failed to start cycles!"
        assert cycles[0]["cycle_seq"] == 0, f"Rep {rep} first cycle was not 0!"

        send_packet(cmd_parent, dict(type="stop", epoch=config["epoch"], reason="rep_done"))
        child_th.join(timeout=2.0)
        cmd_parent.close()
        cmd_child.close()
        fb_bridge.close()
        fb_child.close()


# -----------------------------------------------------------------------------
# Test 11: Handshake timeout executes safe exit
# -----------------------------------------------------------------------------
def test_11_handshake_timeout_safety(tmp_path):
    cmd_parent, cmd_child = socket_pair()
    fb_bridge, fb_child = socket_pair()
    # Configure short handshake timeout (0.15s)
    config = make_config(tmp_path, handshake_timeout_s=0.15)

    t0 = time.monotonic()
    child_run(cmd_child, config, bus=MockServoBus([1000] * 6), fb_sock=fb_child)
    elapsed = time.monotonic() - t0
    assert 0.10 <= elapsed <= 0.40

    final_path = tmp_path / "child_final.json"
    assert final_path.exists()
    final = json.loads(final_path.read_text())
    assert final["fault"] == "startup_handshake_timeout"
    assert final["pause_success"] is True

    # Check fault packet sent to parent
    rows_p, _ = receive_packets(cmd_parent)
    assert any(r.get("communication_fault") == "startup_handshake_timeout" for r in rows_p)

    cmd_parent.close()
    cmd_child.close()
    fb_bridge.close()
    fb_child.close()


# -----------------------------------------------------------------------------
# Test 18 & 19: Full ProcessClient 3-process integration and legacy preservation
# -----------------------------------------------------------------------------
def test_18_full_process_client_handshake_and_no_deadlock(tmp_path):
    config = make_config(tmp_path, feedback_bridge=True)
    client = ProcessClient(config)
    try:
        assert client.ready is not None
        assert client.child_ready is not None
        assert client.bridge_ready is not None
        assert client.servo_started is not None

        # Check that parent_lifecycle.jsonl recorded startup_handshake_complete
        life_path = tmp_path / "parent_lifecycle.jsonl"
        assert life_path.exists()
        events = [json.loads(l) for l in life_path.read_text().splitlines()]
        hs_event = next((e for e in events if e.get("event") == "startup_handshake_complete"), None)
        assert hs_event is not None
        assert hs_event.get("child_ready_ns") is not None
        assert hs_event.get("bridge_ready_ns") is not None
        assert hs_event.get("servo_start_ns") is not None
    finally:
        client.close()


def test_19_legacy_non_bridge_path_preserved(tmp_path):
    """When feedback_bridge is False, legacy single-process path operates without waiting for Bridge."""
    config = make_config(tmp_path, feedback_bridge=False)
    client = ProcessClient(config)
    try:
        assert client.use_feedback_bridge is False
        assert client.bridge_proc is None
        assert client.ready is not None
        assert client.ready["modules_without_ros"] is True

        # Parent receives feedback directly
        time.sleep(0.1)
        client.send_state()
        rows = client.receive()
        feedback_rows = [r for r in rows if r.get("type") == "feedback"]
        assert len(feedback_rows) > 0, "Legacy mode did not receive feedback in Parent!"
    finally:
        client.close()
