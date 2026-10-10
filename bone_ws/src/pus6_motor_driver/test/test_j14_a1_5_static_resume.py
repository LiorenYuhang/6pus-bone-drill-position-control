"""FakeSerial and real ROS handshake tests; no physical serial ports."""
import json
import threading
import time

import pytest

from test_j14_a1_3_communication_recovery import make_recovery, fail, motion
from pus6_motor_driver.communication_recovery import CommunicationRecoveryError
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.protocol import parse_frame, CMD_RD_STATUS
from pus6_motor_driver.safety import PAUSE_ADDR


def make_static(tmp_path, *, jump=0, confirmed=True):
    objs = make_recovery(tmp_path)
    recovery = objs[5]
    synchronized = []
    recovery.static_resume = True
    recovery.reanchor = lambda positions, epoch, deadline: synchronized.extend(positions) or confirmed
    recovery.first_resume_target = lambda: [p+jump for p in synchronized]
    return objs


@pytest.mark.parametrize('axis', [1,3,6])
def test_fresh_state_sync_then_new_complete_static_transaction_no_partial_replay(axis, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path)
    exc = fail(port, writer, axis)
    result = recovery.recover(exc, [1010]*6, old_command=[1100]*6)
    e = result.event
    assert e['old_command'] == [1100]*6
    assert e['reanchor_success'] and not e['resume_success']
    assert e['first_resume_target'] == result.positions != [1100]*6
    assert e['first_resume_delta_per_axis'] == [0]*6
    assert recovery.state == 'STATIC_HOLD_RESUME'
    assert len(motion(port)) == axis  # re-anchor sends no target batch itself.
    for seq in range(41,47):
        audit = writer.send_cycle(e['first_resume_target'], cycle_id=seq)
        fb = feedback.ingest_cycle_feedback(audit.feedbacks, servo_cycle_id=seq, servo_cycle_timestamp_ns=audit.cycle_start_ns)
        recovery.observe_static_cycle(audit, fb)
    resumed = motion(port)[axis:]
    assert [f.motor_id for f in resumed] == list(range(1,7))*6
    assert [int.from_bytes(f.data, 'little') for f in resumed] == e['first_resume_target']*6
    assert e['resume_success'] and e['resumed'] and not e['active_control_enabled']
    assert all(r.tx_start_ns > e['refresh_epoch_ns'] for r in audit.tx_records)
    assert e['static_cycle_count'] == 6 and e['static_incomplete_feedback_cycles'] == 0
    with pytest.raises(ServoTransportError, match='trajectory continuation'):
        writer.send_cycle([1100]*6, cycle_id=47)
    with pytest.raises(ServoTransportError, match='consecutive'):
        writer.send_cycle(e['first_resume_target'], cycle_id=40)
    assert port.hardware_write_count == 0


@pytest.mark.parametrize('jump,allowed', [(80,True),(81,False)])
def test_exact_80_step_resume_admission_boundary(jump, allowed, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path, jump=jump)
    exc = fail(port, writer)
    if allowed:
        e = recovery.recover(exc, [1010]*6).event
        assert e['first_resume_delta_per_axis'] == [80]*6
        assert recovery.state == 'STATIC_HOLD_RESUME'
    else:
        with pytest.raises(CommunicationRecoveryError, match='limit 80'):
            recovery.recover(exc, [1010]*6)
        assert writer.stopped and writer.recovery_static_targets is None
        e = json.loads((tmp_path/'recovery.jsonl').read_text().splitlines()[-1])
        assert e['failure_phase'] == '80_STEP_VALIDATION'
    assert len(motion(port)) == 3 and port.hardware_write_count == 0


@pytest.mark.parametrize('failure', ['baseline_missing','fault_axis','overcurrent_axis'])
def test_incomplete_or_unsafe_fresh_state_never_reanchors(failure, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path)
    exc = fail(port, writer)
    setattr(port, failure, 6)
    recovery.reanchor = lambda *args: pytest.fail('unsafe state must not re-anchor')
    with pytest.raises(CommunicationRecoveryError):
        recovery.recover(exc, [1010]*6)
    assert writer.stopped and writer.recovery_static_targets is None and len(motion(port)) == 3


def test_missing_shaper_sync_ack_never_authorizes_resume(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path, confirmed=False)
    exc = fail(port, writer)
    with pytest.raises(CommunicationRecoveryError, match='confirmation missing'):
        recovery.recover(exc, [1010]*6)
    assert writer.stopped and len(motion(port)) == 3


def test_first_resume_never_writes_after_existing_freshness_deadline(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path)
    result = recovery.recover(fail(port, writer), [1010]*6)
    writer._static_first_deadline_ns = time.monotonic_ns()-1
    count = len(port.frames)
    with pytest.raises(ServoTransportError, match='1000ms'):
        writer.send_cycle(result.event['first_resume_target'], cycle_id=41)
    assert len(port.frames) == count and port.hardware_write_count == 0


def test_first_resume_feedback_must_be_six_new_axes(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path)
    result = recovery.recover(fail(port, writer), [1010]*6)
    port.in_hold=True;port.drop_hold_axis=6
    audit = writer.send_cycle(result.event['first_resume_target'], cycle_id=41)
    fb = feedback.ingest_cycle_feedback(audit.feedbacks, servo_cycle_id=41, servo_cycle_timestamp_ns=audit.cycle_start_ns)
    with pytest.raises(CommunicationRecoveryError, match='6/6'):
        recovery.observe_static_cycle(audit, fb)
    writer.stop('test_first_resume_feedback_failed')
    recovery.fail_static_resume(CommunicationRecoveryError('5/6 first feedback'), 'FIRST_RESUME_FEEDBACK', audit)
    assert not result.event['resume_success'] and writer.stopped and pause.audit_snapshot().success
    assert pause.audit_snapshot().callback_count == 2  # Earlier pause cannot guarantee stop after new motion.


def test_five_second_static_completion_uses_observation_no_active_transition(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = make_static(tmp_path)
    result = recovery.recover(fail(port, writer), [1010]*6)
    for seq in [41,42]:
        audit = writer.send_cycle(result.event['first_resume_target'], cycle_id=seq)
        fb = feedback.ingest_cycle_feedback(audit.feedbacks, servo_cycle_id=seq, servo_cycle_timestamp_ns=audit.cycle_start_ns)
        if seq == 42:
            recovery.clock_ns = lambda: result.event['resume_ns']+5_000_000_000
        recovery.observe_static_cycle(audit, fb)
    assert recovery.state == 'RECOVERY_STABLE' and result.event['success']
    assert result.event['static_hold_duration_ns'] == 5_000_000_000
    assert writer.recovery_static_targets is not None and not result.event['active_control_enabled']


@pytest.mark.parametrize('failure', ['none','first_write','first_feedback','static_write'])
def test_actual_ros_shaper_output_drives_static_resume_and_next_timeout_stops(failure, tmp_path):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float64MultiArray
    from pus6_control.control_node import Pus6ControlNode
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_motor_driver.fake_bus import MockServoBus
    if not rclpy.ok():rclpy.init()
    node=Pus6MotorDriverNode(hardware_mode='mock', bus=MockServoBus(initial_positions=[1000]*6))
    control=Pus6ControlNode()
    node._static_resume=True;control._static_qualification=True
    for timer in [node.write_timer,node.poll_timer]:
        if timer is not None:timer.cancel()
    port, uart, pause, writer, feedback, recovery, anchors=make_static(tmp_path)
    node.j7_adapter,node.j7_stop_adapter,node.j7_writer,node.j7_feedback=uart,pause,writer,feedback
    node._communication_audit_dir=lambda:tmp_path
    control._target_positions=[.01]*6
    control.controller.shaper.q=[.005]*6
    control.controller.shaper.dq=[.001]*6
    control.controller.shaper.ddq=[.002]*6
    control.controller.shaper.dddq=[.003]*6
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node);executor.add_node(control)
    spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
    try:
        node._enqueue_latest_target([1010]*6)
        port.timeout_axis=3
        node._execute_servo_cycle()
        e=node._communication_recovery.static_event
        assert e['reanchor_success'] and not node._j7_fault_latched
        assert e['first_resume_target']==control._static_sync_record['first_shaped_raw_target']==[1010,1010,1000,1000,1000,1000]
        assert control.controller.shaper.dq==control.controller.shaper.ddq==control.controller.shaper.dddq==[0.0]*6
        assert node._latest_target is None and node._uart_cycle_seq==1
        control._handle_desired_deltas(Float64MultiArray(data=[.01]*6))
        node._enqueue_latest_target([1100]*6)
        if failure in ('first_write','static_write','first_feedback'):
            if failure == 'static_write':
                node._execute_servo_cycle()
                assert e['resume_success']
            if failure == 'first_feedback':
                port.device.faults[6] = 1
            else:
                port.timeout_axis=6
            node._execute_servo_cycle()
            phase={'first_write':'FIRST_RESUME_WRITE','static_write':'STATIC_HOLD','first_feedback':'FIRST_RESUME_FEEDBACK'}[failure]
            assert node._j7_fault_latched and e['failure_phase']==phase
            assert node._communication_recovery.recovery_count==1 and pause.audit_snapshot().callback_count==2
        else:
            for _ in range(5):node._execute_servo_cycle()
            assert e['resume_success'] and e['first_resume_delta_per_axis']==[0]*6
            assert e['static_cycle_count']==5 and not node._j7_fault_latched
            assert all([r.target_position for r in a.tx_records]==e['first_resume_target'] for a in writer.audit_snapshot().cycles[1:])
            assert not e['active_control_enabled']
        assert port.hardware_write_count==0
    finally:
        executor.remove_node(node);executor.remove_node(control);executor.shutdown();spinner.join(timeout=2)
        node.destroy_node();control.destroy_node()
        if rclpy.ok():rclpy.shutdown()
