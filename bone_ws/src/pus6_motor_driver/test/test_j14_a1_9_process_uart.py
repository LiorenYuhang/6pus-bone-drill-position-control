"""A1.9 A-M mock verification; never opens a hardware port."""
import json
import os
from pathlib import Path
import socket
import sys
import time
from dataclasses import replace
import pytest
from pus6_motor_driver.j14_process_uart import (CommandState, ServoCore, ProcessClient,
    socket_pair,send_packet,receive_packets,dispatch_slew,PERIOD_NS,LEASE_NS)
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame, pack_write_register_cmd
from pus6_motor_driver.j7_real_uart_adapter import J7AdapterWriteTimeout,SerialWriteMetric,TxQueueState


def cfg(path):
    return dict(epoch=time.monotonic_ns(),output_dir=str(path),hardware_mode='mock',
        port='/dev/DO_NOT_OPEN',baudrate=921600,servo_speed=50,maintenance_mode=True,
        reconnect_max_jump_steps=80,target_timeout_sec=.2)


def command(epoch,seq,targets,now):
    return dict(type='state',epoch=epoch,mode='static',allowed=True,lease_ns=now,
                command=dict(seq=seq,targets=targets,publish_ns=now))


def pump(client,seconds=.12):
    until=time.monotonic()+seconds; rows=[]
    while time.monotonic()<until:
        client.send_state(); rows+=client.receive(); time.sleep(.002)
    return rows


def test_A_B_whole_latest_not_trajectory():
    now=time.monotonic_ns(); s=CommandState(7,[1000]*6)
    rows=[command(7,i,[1000+i]*6,now) for i in range(10)]
    s.ingest(rows,now,[1000]*6)
    assert s.seq==9 and s.candidate([1000]*6,False)==[1009]*6
    assert s.candidate([1004]*6,False)==[1004]*6
    with pytest.raises(ValueError): s.ingest([command(7,10,[1000]*5,now)],now,[1000]*6)
    assert s.seq==9


@pytest.mark.parametrize('bad',[[1000]*5,[1000]*7,[1000]*5+[True],[1000]*5+[1000.1]])
def test_A_reject_partial_or_noninteger(bad):
    now=time.monotonic_ns(); s=CommandState(7,[1000]*6)
    with pytest.raises(ValueError): s.ingest([command(7,0,bad,now)],now,[1000]*6)


def test_epoch_permission_replay_warning_and_80_boundary():
    now=time.monotonic_ns(); s=CommandState(7,[1000]*6)
    s.ingest([command(7,0,[1000]*6,now)],now,[1000]*6)
    s.ingest([command(7,1,[1081]*6,now+300_000_000)],now+300_000_000,[1000]*6)
    assert s.seq==0 and s.rejected==1
    s.ingest([command(7,2,[1080]*6,now+300_000_001)],now+300_000_001,[1000]*6)
    assert s.seq==2
    assert s.candidate([1000]*6,True)==[1000]*6
    s.ingest([command(7,1,[999]*6,now+300_000_001)],now+300_000_001,[1000]*6)
    assert s.seq==2
    with pytest.raises(ValueError): s.ingest([command(8,3,[1000]*6,now)],now,[1000]*6)
    bad=command(7,3,[1000]*6,now);bad['allowed']=False
    with pytest.raises(ValueError): s.ingest([bad],now,[1000]*6)


def test_D_exact_order_frames_and_I_feedback(tmp_path):
    b=MockServoBus(initial_positions=[1000]*6);c=ServoCore(cfg(tmp_path),b)
    try:
        c.initialize(); b.written_frames.clear()
        now=time.monotonic_ns();c.command.ingest([command(c.config['epoch'],0,[1001]*6,now)],now,[1000]*6)
        p=c.cycle(now)
        assert b.written_frames==[pack_write_register_cmd(i,0x29,[1001]) for i in range(1,7)]
        assert p['positions']==[1001]*6 and len(p['currents'])==len(p['faults'])==len(p['rx_ns'])==6
        assert p['command_seq']==0 and p['feedback_complete']
        assert len(p['tx_start_ns'])==6 and p['feedback_ready_ns']>=p['cycle_start_ns']
    finally:c.close()


class TimeoutBus(MockServoBus):
    def __init__(self,axis):super().__init__(initial_positions=[1000]*6);self.axis=axis
    def write(self,frame):
        parsed=parse_frame(frame)
        if parsed.register_addr==0x29 and parsed.motor_id==self.axis:
            self.written_frames.append(frame)
            t=time.monotonic_ns()
            metric=SerialWriteMetric(t,t+31_000_000,31_000_000,10_000,len(frame),-1,
                                    error='SerialTimeoutException',configured_timeout_s=.030,
                                    queue_before=TxQueueState(0,0,0),queue_after=TxQueueState(0,0,0))
            raise J7AdapterWriteTimeout(metric,.030)
        return super().write(frame)


@pytest.mark.parametrize('axis',[1,3,6])
def test_E_F_G_timeout_abort_immediate_child_pause(tmp_path,axis):
    b=TimeoutBus(axis); c=ServoCore(cfg(tmp_path),b)
    try:
        c.initialize();b.written_frames.clear();p=c.cycle(time.monotonic_ns())
        frames=[parse_frame(f) for f in b.written_frames]
        motion=[f.motor_id for f in frames if f.register_addr==0x29]
        pause=[f.motor_id for f in frames if f.register_addr==0x1A]
        assert motion==list(range(1,axis+1)) and pause==list(range(1,7))
        assert p['type']=='fault' and c.writer.stopped and c.pauses[-1]['success']
        metric=p['failed_audit']['tx_records'][-1]['serial_write_metric']
        assert metric['configured_timeout_s']==.030
        assert frames[axis].register_addr==0x1A
    finally:c.close()
    assert (tmp_path/'tx_write_events.jsonl').exists()


@pytest.mark.parametrize('kind',['fault','current'])
def test_H_local_motor_safety(tmp_path,kind):
    from test_j14_a1_3_communication_recovery import RecoverySerial
    from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
    port=RecoverySerial(); b=J7UartAdapter(port,timeout_s=.010,min_frame_interval_s=.0001,tx_queue_diagnostics=True)
    c=ServoCore(cfg(tmp_path),b)
    try:
        c.initialize()
        if kind=='fault':port.device.faults[3]=1
        else:port.device.currents[3]=1501
        p=c.cycle(time.monotonic_ns())
        assert p['type']=='fault' and c.writer.stopped and c.pauses[-1]['success']
    finally:c.close()


def test_C_independent_process_50Hz_I_feedback_M_shutdown(tmp_path):
    c=ProcessClient(cfg(tmp_path))
    try:
        assert c.ready['pid']!=os.getpid() and c.ready['modules_without_ros']
        # A single command followed by lease only; never ticks.
        c.update([1000]*6); rows=pump(c,.26)
        assert sum(r['type']=='feedback' for r in rows)>=8 and not c.stopped
    finally:c.close()
    cycles=[json.loads(l) for l in (tmp_path/'child_cycles.jsonl').read_text().splitlines()]
    expected=[r['expected_ns'] for r in cycles]
    assert len(expected)>=8 and all(b-a==PERIOD_NS for a,b in zip(expected,expected[1:]))
    life=[json.loads(l) for l in (tmp_path/'child_lifecycle.jsonl').read_text().splitlines()]
    assert [r['event'] for r in life]==['serial_owner','pause_complete','serial_closed','child_exit']
    assert json.loads((tmp_path/'child_final.json').read_text())['hardware_write_count']==0
    assert c.proc.returncode==0


def test_J_parent_EOF_pauses(tmp_path):
    c=ProcessClient(cfg(tmp_path));c.sock.close();c.proc.wait(timeout=3)
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['fault']=='parent_ipc_closed' and final['pause_success'] and final['closed']


def test_J_parent_lease_expiry(tmp_path):
    c=ProcessClient(cfg(tmp_path))
    # Keep IPC open but do not send any lease. This is not a command-age gate.
    c.proc.wait(timeout=3)
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['fault']=='parent_lease_expired' and final['pause_success']
    c.close()


def test_K_child_death_parent_stop(tmp_path):
    c=ProcessClient(cfg(tmp_path));c.proc.kill();c.proc.wait(timeout=3)
    c.receive();assert c.stopped and not c.proc.poll() is None
    c.close()
    assert not (tmp_path/'child_final.json').exists()  # no invented ACK


def test_L_feedback_backpressure_never_blocks_UART(tmp_path):
    c=ProcessClient(cfg(tmp_path))
    try:
        until=time.monotonic()+.4
        while time.monotonic()<until:
            c.send_state();time.sleep(.005) # deliberately never receive feedback
        c.stop('test_stop');c.sock.shutdown(socket.SHUT_WR);c.proc.wait(timeout=3)
        final=json.loads((tmp_path/'child_final.json').read_text())
        assert final['cycles']>=15 and final['pause_success'] and final['closed']
    finally:c.close()


def test_L_command_socket_bounded_latest_and_terminal_priority():
    a,b=socket_pair();now=time.monotonic_ns();sent=0
    try:
        for i in range(10000):
            if not send_packet(a,command(4,i,[1000+i%20]*6,now)):break
            sent+=1
        assert sent<64
        rows,flood=receive_packets(b);assert not flood
        s=CommandState(4,[1000]*6);s.ingest(rows,now,[1000]*6)
        assert s.seq==sent-1
        s.ingest([dict(type='stop',epoch=4,reason='STOP'),command(4,999,[1001]*6,now)],now,[1000]*6)
        assert s.stop_reason=='STOP' and s.seq==sent-1
    finally:a.close();b.close()


def test_slew_rule_caps_and_proportion():
    assert dispatch_slew([1100,1050,1000,950,900,1000],[1000]*6,0,1)==[1005,1002,1000,998,995,1000]
    assert max(abs(x-1000) for x in dispatch_slew([2000]*6,[1000]*6,1,1_000_000_001))==20


def test_J_actual_parent_process_death(tmp_path):
    import subprocess
    config=cfg(tmp_path)
    code='''import json,time,sys
from pus6_motor_driver.j14_process_uart import ProcessClient
c=ProcessClient(json.loads(sys.argv[1]))
print(c.proc.pid,flush=True)
while True:
 c.send_state();c.receive();time.sleep(.005)
'''
    p=subprocess.Popen([sys.executable,'-c',code,json.dumps(config)],stdout=subprocess.PIPE,text=True)
    try:
        child_pid=int(p.stdout.readline())
        p.kill();p.wait(timeout=3)
        until=time.monotonic()+3
        while not (tmp_path/'child_final.json').exists() and time.monotonic()<until:time.sleep(.005)
        final=json.loads((tmp_path/'child_final.json').read_text())
        assert final['pid']==child_pid and final['fault']=='parent_ipc_closed' and final['pause_success']
    finally:
        if p.poll() is None:p.kill();p.wait(timeout=3)


def test_parent_ROS_integration_no_serial_owner(tmp_path,monkeypatch):
    import rclpy
    from rclpy.parameter import Parameter
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
    from std_msgs.msg import Int32MultiArray
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**k:pytest.fail('parent opened physical serial'))
    monkeypatch.setenv('PUS6_J14_DEFER_DRIVER_DIAGNOSTICS','1')
    monkeypatch.setenv('PUS6_J14_AUDIT_DIR',str(tmp_path))
    # Existing audit resolver uses the recorder pointer; avoid modifying global pointers.
    monkeypatch.setattr(J14ProcessMotorDriverNode,'_communication_audit_dir',lambda self:tmp_path)
    rclpy.init()
    n=None
    try:
        n=J14ProcessMotorDriverNode(hardware_mode='mock',servo_scheduler_mode='dedicated_thread',
            telemetry_publish_mode='async_latest',feedback_loss_policy='age_based',
            parameter_overrides=[Parameter('reconnect_max_jump_steps',value=80),
                                 Parameter('enable_runtime_diagnostics',value=True)])
        assert not hasattr(n.j7_adapter,'serial') and n._process_client.ready['modules_without_ros']
        msg=Int32MultiArray();msg.data=[1001]*6;n._handle_command_joint_steps(msg)
        until=time.monotonic()+.5
        while n._last_commanded_steps!=[1001]*6 and time.monotonic()<until:time.sleep(.005)
        assert n._last_commanded_steps==[1001]*6 and n.telemetry_health_ok
        assert n._latest_telemetry_snapshot is not None
        n._process_client.proc.kill();n._process_client.proc.wait(timeout=3)
        until=time.monotonic()+.3
        while not n.driver_fault_latched and time.monotonic()<until:time.sleep(.005)
        assert n.driver_fault_latched and not n.telemetry_health_ok
    finally:
        if n:n.destroy_node()
        rclpy.shutdown()


def test_E_actual_serial_factory_timeout_unchanged_and_shutdown(tmp_path,monkeypatch):
    import serial
    from test_j14_a1_3_communication_recovery import RecoverySerial
    class Port(RecoverySerial):
        def reset_input_buffer(self):self.rx.clear()
        def reset_output_buffer(self):pass
    port=Port();calls=[]
    def factory(*args,**kwargs):calls.append((args,kwargs));return port
    monkeypatch.setattr(serial,'Serial',factory)
    config=cfg(tmp_path);config['hardware_mode']='active'
    c=ServoCore(config)
    try:
        c.initialize();assert c.cycle(time.monotonic_ns())['type']=='feedback'
        assert calls[0][0]==('/dev/DO_NOT_OPEN',921600)
        assert calls[0][1]['timeout']==.005 and calls[0][1]['write_timeout']==.030
        assert c.writer.rx_drain_budget_s==.004
        assert c.writer.min_frame_interval_ns==600_000
    finally:c.close()
    assert port.closed and port.hardware_write_count==0 and c.pauses[-1]['success']


def test_child_enforces_bounded_experiment_window(tmp_path):
    config=cfg(tmp_path);config['max_servo_duration_s']=.16
    c=ProcessClient(config)
    pump(c,.25)
    c.close()
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['fault']=='window_complete' and final['pause_success'] and 6<=final['cycles']<=9


def test_L_bounded_receive_backlog_holds_without_losing_latest(tmp_path):
    b=MockServoBus(initial_positions=[1000]*6);c=ServoCore(cfg(tmp_path),b)
    try:
        c.initialize();now=time.monotonic_ns()
        c.command.ingest([command(c.config['epoch'],1,[1004]*6,now)],now,[1000]*6)
        assert c.cycle(now,hold_for_ipc_backlog=True)['targets']==[1000]*6
        assert not c.command.consumed
        assert c.cycle(time.monotonic_ns())['targets']==[1004]*6
    finally:c.close()
