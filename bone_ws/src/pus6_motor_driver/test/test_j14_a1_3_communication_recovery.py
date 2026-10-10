"""Protocol-aware software recovery tests; never open a physical serial device."""
import csv
from dataclasses import asdict
import json
import time

import pytest
import serial

from pus6_motor_driver.communication_recovery import CommunicationRecovery, CommunicationRecoveryError, append_evidence
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7StopAdapter, J7AdapterWriteTimeout
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.protocol import parse_frame, CMD_RD_STATUS
from pus6_motor_driver.safety import PAUSE_ADDR, TARGET_POSITION_ADDR


class RecoverySerial:
    timeout = .005
    write_timeout = .030
    hardware_write_count = 0

    def __init__(self):
        self.device = FakeBus()
        self.device.positions = {m:1000 for m in range(1,7)}
        self.rx = bytearray();self.frames=[]
        self.timeout_axis = None;self.pause_missing=None
        self.drop_hold_axis=None;self.baseline_missing=None
        self.fault_axis=None;self.overcurrent_axis=None
        self.bad_position_axis=None;self.wrong_pause_register=None
        self.in_hold=False;self.non_timeout_axis=None
        self.pause_fault_axis=None
        self.hold_timeout_axis=None

    @property
    def in_waiting(self):return len(self.rx)

    def read(self,size):
        data=bytes(self.rx[:size]);del self.rx[:size];return data

    def close(self):
        self.closed=True

    def write(self,frame):
        self.frames.append(frame);p=parse_frame(frame)
        if p.command==CMD_RD_STATUS:
            if self.fault_axis:self.device.faults[self.fault_axis]=1
            if self.overcurrent_axis:self.device.currents[self.overcurrent_axis]=1501
            if self.bad_position_axis:self.device.positions[self.bad_position_axis]=2006
            if self.baseline_missing==p.motor_id:return len(frame)
        if p.register_addr==TARGET_POSITION_ADDR:
            if p.motor_id==self.non_timeout_axis:raise OSError('fake I/O failure')
            if p.motor_id==self.timeout_axis:
                self.timeout_axis=None
                raise serial.SerialTimeoutException('Write timeout')
            if self.in_hold and self.drop_hold_axis==p.motor_id:return len(frame)
            if self.in_hold and self.hold_timeout_axis==p.motor_id:
                raise serial.SerialTimeoutException('Verification write timeout')
        if p.register_addr==PAUSE_ADDR:
            if self.pause_fault_axis:self.device.faults[self.pause_fault_axis]=1
            if p.motor_id==self.pause_missing:return len(frame)
            if p.motor_id==self.wrong_pause_register:
                from pus6_motor_driver.protocol import pack_write_register_cmd
                self.rx.extend(self.device.request(pack_write_register_cmd(p.motor_id,TARGET_POSITION_ADDR,[1000])))
                return len(frame)
        self.rx.extend(self.device.request(frame));return len(frame)


def make_recovery(tmp_path, *, reanchor=None):
    port=RecoverySerial();uart=J7UartAdapter(port,timeout_s=.001,min_frame_interval_s=0)
    pause=J7StopAdapter(uart,audit_observer=lambda a:append_evidence(tmp_path/'pause.jsonl',asdict(a)))
    writer=J7SixAxisServoTransport(uart,min_frame_interval_s=0,pause_callback=pause,recoverable_write_timeouts=True)
    writer.configure_servo(speed=50)
    feedback=J7ServoFeedbackScheduler(writer,feedback_loss_policy='age_based',max_feedback_age_s=1.0)
    feedback.poll_once(servo_cycle_id=0,servo_cycle_timestamp_ns=time.monotonic_ns(),force_full_scan=True)
    port.frames.clear()
    anchors=[]
    recovery=CommunicationRecovery(writer,pause,feedback,
        reanchor=reanchor or (lambda pos,epoch,deadline:anchors.append(list(pos)) or True),
        audit_path=tmp_path/'recovery.jsonl',recovery_jump_limit=80)
    original=writer.verify_recovery_hold
    def hold(*args,**kwargs):
        port.in_hold=True
        try:return original(*args,**kwargs)
        finally:port.in_hold=False
    writer.verify_recovery_hold=hold
    return port,uart,pause,writer,feedback,recovery,anchors


def fail(port,writer,axis=3):
    port.timeout_axis=axis
    with pytest.raises(J7AdapterWriteTimeout) as caught:writer.send_cycle([1010]*6,cycle_id=40)
    return caught.value


def motion(port):
    return [parse_frame(f) for f in port.frames if parse_frame(f).register_addr==TARGET_POSITION_ADDR]


def test_normal_path_has_no_recovery_or_frame_changes(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    audit=writer.send_cycle([1010]*6,cycle_id=40)
    assert len(audit.feedbacks)==6 and [p.motor_id for p in motion(port)]==list(range(1,7))
    assert all(p.retry_count==0 for p in audit.tx_records)
    assert not pause.called and not writer.communication_transient and not writer.stopped
    assert not anchors and recovery.recovery_count==0 and port.hardware_write_count==0


@pytest.mark.parametrize('axis',[1,3,6])
def test_timeout_aborts_then_fresh_new_transaction_reanchors_and_resumes(axis,tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    exc=fail(port,writer,axis)
    assert [p.motor_id for p in motion(port)]==list(range(1,axis+1))
    assert writer.communication_transient and not writer.stopped
    with pytest.raises(ServoTransportError):writer.send_cycle([1010]*6,cycle_id=41)
    result=recovery.recover(exc,[1010]*6)
    assert result.event['success'] and result.event['sent_motor_ids']==list(range(1,axis))
    assert result.event['unsent_motor_ids']==list(range(axis+1,7))
    assert result.event['pause']['success'] and len(result.event['pause']['records'])==6
    assert result.positions==[1010]*(axis-1)+[1000]*(7-axis)
    assert anchors==[result.positions]
    assert result.verification_audit.cycle_id==41
    assert result.verification_audit.cycle_start_ns>result.event['pause_barrier_ns']
    assert all(s.timestamp_ns>result.event['pause_barrier_ns'] for s in result.feedback_audit.samples)
    later=motion(port)[axis:]
    assert [p.motor_id for p in later]==list(range(1,7))
    assert [int.from_bytes(p.data,'little') for p in later]==result.positions
    # The uncertain axis is not retried at its failed target (1010).
    assert int.from_bytes(later[axis-1].data,'little')==1000
    assert not writer.stopped and not writer.communication_transient and not writer.audit_snapshot().delivery_uncertain
    assert not pause.called and recovery.recovery_count==1
    writer.send_cycle(result.positions,cycle_id=42)
    assert all(r.retry_count==0 for a in writer.audit_snapshot().cycles for r in a.tx_records)
    assert port.hardware_write_count==0


@pytest.mark.parametrize('kind',['pause_missing','wrong_pause_register','baseline_missing','drop_hold_axis','fault_axis','overcurrent_axis','bad_position_axis','pause_fault_axis','hold_timeout_axis'])
def test_unconfirmed_or_unsafe_recovery_never_resumes(kind,tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    exc=fail(port,writer)
    setattr(port,kind,6)
    with pytest.raises(CommunicationRecoveryError):recovery.recover(exc,[1010]*6)
    assert writer.stopped and recovery.state=='HARD_STOP_LATCH' and not anchors
    assert not json.loads((tmp_path/'recovery.jsonl').read_text().splitlines()[-1])['success']
    assert len([f for f in port.frames if parse_frame(f).register_addr==PAUSE_ADDR])==6
    count=len(port.frames)
    with pytest.raises(ServoTransportError):writer.send_cycle([1010]*6,cycle_id=42)
    assert len(port.frames)==count and port.hardware_write_count==0


@pytest.mark.parametrize('jump,success',[(80,True),(81,False)])
def test_existing_recovery_jump_boundary(jump,success,tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    exc=fail(port,writer,1)
    if success:assert recovery.recover(exc,[1000+jump]*6).event['success']
    else:
        with pytest.raises(CommunicationRecoveryError,match='Timeout recovery target rejected'):
            recovery.recover(exc,[1000+jump]*6)
        assert not anchors and len(motion(port))==1


def test_failed_upstream_reset_confirmation_never_resumes(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path,reanchor=lambda *a:False)
    exc=fail(port,writer)
    with pytest.raises(CommunicationRecoveryError,match='re-anchor not confirmed'):recovery.recover(exc,[1010]*6)
    assert writer.stopped


def test_repeated_independent_recoveries_have_new_pause_epochs_no_count_gate(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    for n in range(3):
        port.timeout_axis=3
        with pytest.raises(J7AdapterWriteTimeout) as caught:writer.send_cycle([1010]*6,cycle_id=40+2*n)
        assert recovery.recover(caught.value,[1010]*6).event['success']
    audits=[json.loads(line) for line in (tmp_path/'pause.jsonl').read_text().splitlines()]
    assert len(audits)==3 and all(a['success'] for a in audits)
    assert [a['callback_count'] for a in audits]==[1,2,3]
    assert recovery.recovery_count==3 and not writer.stopped


def test_non_timeout_write_error_still_hard_stops(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    port.non_timeout_axis=3
    with pytest.raises(Exception):writer.send_cycle([1010]*6,cycle_id=40)
    assert writer.stopped and not writer.communication_transient and pause.called and recovery.recovery_count==0


def test_shutdown_pause_ack_is_persisted_and_idempotent(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    first=pause.stop_all('shutdown');second=pause.stop_all('shutdown')
    assert first is second and first.success and first.callback_count==1
    saved=json.loads((tmp_path/'pause.jsonl').read_text())
    assert len(saved['records'])==6
    assert all(r['response_received'] and r['request_hex'] and r['response_hex'] for r in saved['records'])


def test_node_and_real_ros_shaper_handshake_resume_without_latching(tmp_path):
    import threading
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_motor_driver.fake_bus import MockServoBus
    from pus6_control.control_node import Pus6ControlNode
    if not rclpy.ok():rclpy.init()
    node=Pus6MotorDriverNode(hardware_mode='mock',bus=MockServoBus(initial_positions=[1000]*6))
    control=Pus6ControlNode()
    for timer in [node.write_timer,node.poll_timer,control.timer]:
        if timer is not None:timer.cancel()
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    node.j7_adapter,node.j7_stop_adapter,node.j7_writer,node.j7_feedback=uart,pause,writer,feedback
    node._communication_audit_dir=lambda:tmp_path
    control._target_positions=[.01]*6
    control.controller.shaper.dq=[.001]*6
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node);executor.add_node(control)
    spinner=threading.Thread(target=executor.spin,daemon=True);spinner.start()
    try:
        port.timeout_axis=3
        node._execute_servo_cycle()
        assert not node._j7_fault_latched and not writer.stopped
        assert node._communication_recovery.recovery_count==1
        assert node._last_commanded_steps==[1000]*6 and node._latest_target is None
        assert node._uart_cycle_seq==2 and node._stream_timed_out
        assert control._target_positions==[0.0]*6
        assert control.controller.shaper.dq==control.controller.shaper.ddq==[0.0]*6
        node._execute_servo_cycle()
        assert node._uart_cycle_seq==3 and not node._j7_fault_latched
        assert port.hardware_write_count==0
    finally:
        executor.remove_node(node);executor.remove_node(control)
        executor.shutdown();spinner.join(timeout=2)
        node.destroy_node();control.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def test_active_mode_with_injected_mock_bus_never_writes_historical_run_audit():
    import rclpy
    from pathlib import Path
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_motor_driver.fake_bus import MockServoBus
    from pus6_motor_driver.safety import HardwareMode
    if not rclpy.ok():rclpy.init()
    node=Pus6MotorDriverNode(hardware_mode='mock',bus=MockServoBus(initial_positions=[1000]*6))
    node.hardware_mode=HardwareMode.ACTIVE
    try:
        assert node.mock_bus is not None
        assert node._communication_audit_dir().name.startswith('j14_mock_recovery_')
        if Path('/tmp/j14_latest_run_dir').exists():
            assert node._communication_audit_dir()!=Path(Path('/tmp/j14_latest_run_dir').read_text().strip())
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def test_old_transaction_snapshot_is_not_fresh_recovery_feedback(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    old=writer.send_cycle([1000]*6,cycle_id=39)
    exc=fail(port,writer)
    with pytest.raises(ServoTransportError,match='old transaction snapshot'):
        writer.complete_communication_recovery(old)
    writer.verify_recovery_hold=lambda *a,**k:old
    with pytest.raises(CommunicationRecoveryError,match='predates pause'):
        recovery.recover(exc,[1010]*6)
    assert writer.stopped and not anchors


def test_existing_1000ms_scheduler_hard_stop_is_not_reset_by_recovery(tmp_path):
    port,uart,pause,writer,feedback,recovery,anchors=make_recovery(tmp_path)
    exc=fail(port,writer)
    feedback._last_poll_completion_ns=time.monotonic_ns()-1_000_000_001
    with pytest.raises(CommunicationRecoveryError,match='scheduler deadline'):
        recovery.recover(exc,[1010]*6)
    assert writer.stopped and feedback.stopped and not anchors
