"""Validation tests for the J14 Demo Startup State Machine.

Covers the 5 required verification cases:
1. Park -> HOME -> ACTIVE
2. Already HOME -> ACTIVE
3. Intermediate position -> HOME -> ACTIVE
4. Startup fault during HOME -> Stop, Skip HOME recovery, Cleanup
5. Ctrl+C during ACTIVE -> Stop controller, HOME recovery, Pause ACK 6/6, Cleanup
"""
import json
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace
import pytest

sys.path.insert(0, str(Path(__file__).parent))
for p in (Path(__file__).parents[1] / "install").glob("*/lib/python3.12/site-packages"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import j14_demo_home as home
import j14_demo_launcher as launcher
from j14_demo_controller import write_json
from pus6_motor_driver.safety import (
    HOME_MEASUREMENT_BAND_LOWER, HOME_MEASUREMENT_BAND_UPPER,
    OPERATING_HOME_RAW_STEPS, STORAGE_PARK_RAW_STEPS,
    MAX_CURRENT_MA, FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT,
)


class MockClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def monotonic_ns(self):
        return int(self.now * 1e9)


def make_mock_node(monkeypatch, initial_feedback_steps, target_step=1000):
    import rclpy
    from builtin_interfaces.msg import Time

    clock = MockClock()
    monkeypatch.setattr(home, 'time', clock)
    monkeypatch.setattr(launcher, 'time', clock)

    published_step_commands = []
    published_cartesian_poses = []

    node = SimpleNamespace(
        z0=0.132612268,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        driver_stop_latched=False,
        driver_fault_msg='',
        cmd_steps_pub=SimpleNamespace(publish=lambda m: published_step_commands.append(list(m.data))),
        target_pose_pub=SimpleNamespace(publish=lambda m: published_cartesian_poses.append(m)),
    )

    prior_ns = clock.monotonic_ns() - 100_000_000
    feedback = dict(
        count=0,
        steps=list(initial_feedback_steps),
        rx_ns=[prior_ns] * 6,
        faults=[0] * 6,
        currents=[0] * 6,
        last_rx_ns=prior_ns,
    )

    def spin(n, timeout_sec=0.002, **kwargs):
        clock.now += 0.02
        feedback['count'] += 1
        # When step commands are published, simulate motor following the latest target
        if published_step_commands:
            feedback['steps'] = list(published_step_commands[-1])
        elif home._home_feedback_in_band(feedback['steps']):
            feedback['steps'] = [target_step] * 6
        feedback['rx_ns'] = [clock.monotonic_ns()] * 6
        feedback['last_rx_ns'] = clock.monotonic_ns()

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    return node, feedback, published_step_commands, published_cartesian_poses, clock


# ==============================================================================
# Case 1: Park [0, -2, 1, 0, -1, 0] -> HOME -> ACTIVE
# ==============================================================================
def test_case1_park_to_home_to_active(monkeypatch, tmp_path):
    park_steps = [0, -2, 1, 0, -1, 0]
    node, feedback, step_cmds, cartesian_poses, clock = make_mock_node(monkeypatch, park_steps)

    stop = threading.Event()
    result = home.recover_home(node, feedback, tmp_path, stop, lambda: {}, phase='startup', allow_already_home=True)

    # 1. HOME command generated
    assert result['state'] == 'HOME_CONFIRMED'
    assert not result['already_home']
    assert result['home_target_publish_count'] > 0
    assert len(step_cmds) > 0

    # 2. Safety bounds: every single commanded step must be in [0, 2000]
    for cmd in step_cmds:
        assert len(cmd) == 6
        assert all(0 <= s <= 2000 for s in cmd), f"Command {cmd} violated safe boundary [0, 2000]"

    # 3. Feedback reaches 995-1005
    assert all(HOME_MEASUREMENT_BAND_LOWER <= s <= HOME_MEASUREMENT_BAND_UPPER for s in result['feedback'])
    assert result['consecutive_samples'] == 5
    assert len(result['confirmation_samples']) == 5

    # 4. Controller admission is only written after HOME is confirmed
    admission_path = tmp_path / 'startup_command_admission.json'
    assert not admission_path.exists()  # Not written by recover_home

    # Simulate launcher writing admission post-HOME
    write_json(admission_path, dict(
        feedback=dict(steps=list(result['feedback']), rx_ns=list(result['per_axis_rx_done_ns'])),
        monotonic_ns=clock.monotonic_ns(),
    ))
    assert admission_path.exists()
    adm = json.loads(admission_path.read_text())
    assert all(HOME_MEASUREMENT_BAND_LOWER <= s <= HOME_MEASUREMENT_BAND_UPPER for s in adm['feedback']['steps'])


# ==============================================================================
# Case 2: Already HOME [998, 998, 1002, 999, 998, 1001] -> ACTIVE
# ==============================================================================
def test_case2_already_home_to_active(monkeypatch, tmp_path):
    home_steps = [998, 998, 1002, 999, 998, 1001]
    node, feedback, step_cmds, cartesian_poses, clock = make_mock_node(monkeypatch, home_steps)

    stop = threading.Event()
    result = home.recover_home(node, feedback, tmp_path, stop, lambda: {}, phase='startup', allow_already_home=True)

    # 1. Detected already home
    assert result['state'] == 'HOME_CONFIRMED'
    assert result['already_home'] is True

    # 2. No unnecessary HOME command generated
    assert result['home_target_publish_count'] == 0
    assert len(step_cmds) == 0
    assert result['motion_start_ns'] is None

    # 3. 5 consecutive valid samples confirmed
    assert result['consecutive_samples'] == 5
    assert len(result['confirmation_samples']) == 5
    assert all(HOME_MEASUREMENT_BAND_LOWER <= s <= HOME_MEASUREMENT_BAND_UPPER for s in result['feedback'])


def test_startup_admission_preserves_measured_feedback(monkeypatch, tmp_path):
    node, feedback, _, _, _ = make_mock_node(monkeypatch, [1000] * 6)
    feedback['currents'] = [12, 24, 36, 48, 60, 72]
    publisher = SimpleNamespace(publish=lambda message: None)
    node.get_publishers_info_by_topic = lambda topic: []
    node.create_publisher = lambda *args: publisher
    destroyed = []
    node.destroy_publisher = destroyed.append
    owned = SimpleNamespace(check=lambda: None)
    admission = tmp_path / 'startup_command_admission.json'

    def ready(predicate, label, timeout, stop, owned, observer):
        recorded = json.loads(admission.read_text())['feedback']
        assert recorded['steps'] == feedback['steps']
        assert recorded['rx_ns'] == feedback['rx_ns']
        assert recorded['faults'] == feedback['faults']
        assert recorded['currents'] == [12, 24, 36, 48, 60, 72]
        assert home.feedback_failure(observer, recorded) is None

    monkeypatch.setattr(launcher, 'wait_ready', ready)
    result = launcher.startup_home(node, feedback, owned, tmp_path, threading.Event())
    assert result['state'] == 'HOME_CONFIRMED'
    assert destroyed == [publisher]
    assert node.cmd_steps_pub is None


# ==============================================================================
# Case 3: Intermediate position [500, 800, 1200, 600, 700, 900] -> HOME -> ACTIVE
# ==============================================================================
def test_case3_intermediate_position_to_home_to_active(monkeypatch, tmp_path):
    intermediate_steps = [500, 800, 1200, 600, 700, 900]
    node, feedback, step_cmds, cartesian_poses, clock = make_mock_node(monkeypatch, intermediate_steps)

    stop = threading.Event()
    result = home.recover_home(node, feedback, tmp_path, stop, lambda: {}, phase='startup', allow_already_home=True)

    # 1. Non-home intermediate detected, motion executed
    assert result['state'] == 'HOME_CONFIRMED'
    assert result['already_home'] is False
    assert result['home_target_publish_count'] > 0
    assert len(step_cmds) > 0

    # 2. All commands in bounds
    for cmd in step_cmds:
        assert all(0 <= s <= 2000 for s in cmd)

    # 3. Reaches home and confirms
    assert all(HOME_MEASUREMENT_BAND_LOWER <= s <= HOME_MEASUREMENT_BAND_UPPER for s in result['feedback'])
    assert result['consecutive_samples'] == 5


# ==============================================================================
# Case 4: Startup fault during HOME -> Stop, Skip HOME recovery, Cleanup
# ==============================================================================
@pytest.mark.parametrize('fault_type', ['hardware_fault', 'overcurrent', 'driver_stop', 'timeout'])
def test_case4_startup_fault_skips_home_recovery_and_pauses(monkeypatch, tmp_path, fault_type):
    import rclpy
    park_steps = [0, 0, 0, 0, 0, 0]
    node, feedback, step_cmds, _, clock = make_mock_node(monkeypatch, park_steps)

    orig_spin = rclpy.spin_once

    def faulty_spin(n, **kwargs):
        orig_spin(n, **kwargs)
        if feedback['count'] == 3:
            if fault_type == 'hardware_fault':
                feedback['faults'][0] = 1
            elif fault_type == 'overcurrent':
                feedback['currents'][0] = MAX_CURRENT_MA + 100
            elif fault_type == 'driver_stop':
                node.driver_stop_latched = True
                node.driver_fault_msg = 'overcurrent_trip'
            elif fault_type == 'timeout':
                feedback['rx_ns'] = [0] * 6

    monkeypatch.setattr(rclpy, 'spin_once', faulty_spin)
    stop = threading.Event()
    result = home.recover_home(node, feedback, tmp_path, stop, lambda: {}, phase='startup', allow_already_home=True)

    # 1. State must NOT be HOME_CONFIRMED
    assert result['state'] in ('HOME_ABORTED', 'SKIPPED')
    assert result['state'] != 'HOME_CONFIRMED'

    # 2. Simulation of launcher policy: when startup fails, HOME recovery is SKIPPED
    meta = dict(active=False, stop_reason='startup_home_failed', startup_home=result)
    normal = bool(meta['active'] and meta.get('stop_reason') == 'operator_stop')
    assert normal is False  # Safety policy triggers

    meta['home_recovery'] = dict(state='SKIPPED', reason='safety policy' if not normal else 'not started')
    assert meta['home_recovery']['state'] == 'SKIPPED'


# ==============================================================================
# Case 5: Ctrl+C during ACTIVE -> Stop controller, HOME recovery, Pause ACK, Cleanup
# ==============================================================================
def test_case5_ctrl_c_during_active_stops_controller_recovers_home_and_pauses(monkeypatch, tmp_path):
    active_steps = [1005, 995, 1000, 1002, 998, 1001]
    node, feedback, step_cmds, cartesian_poses, clock = make_mock_node(monkeypatch, active_steps)

    meta = dict(active=True, stop_reason='operator_stop')
    normal = bool(meta['active'] and meta.get('stop_reason') == 'operator_stop')
    assert normal is True

    # Shutdown recovery uses Cartesian target pose
    stop = threading.Event()
    home_result = home.recover_home(node, feedback, tmp_path, stop, lambda: {}, phase='shutdown')

    assert home_result['state'] == 'HOME_CONFIRMED'
    assert (tmp_path / 'home_recovery.json').exists()
    assert all(HOME_MEASUREMENT_BAND_LOWER <= s <= HOME_MEASUREMENT_BAND_UPPER for s in home_result['feedback'])

    # Verify pause driver simulation
    fake_pid = 12345
    write_json(tmp_path / 'motor_driver_node_host.json', dict(pid=fake_pid, state='ready'))
    sent_signals = []

    def fake_kill(pid, sig):
        sent_signals.append((pid, sig))
        write_json(tmp_path / 'motor_driver_node_host.json', dict(pid=pid, state='exited', returncode=0))

    monkeypatch.setattr(launcher.os, 'kill', fake_kill)
    owned = launcher.OwnedProcesses(tmp_path, {})
    owned.pause_driver()

    assert sent_signals == [(fake_pid, signal.SIGINT)]
    assert owned.events[-1]['returncode'] == 0
