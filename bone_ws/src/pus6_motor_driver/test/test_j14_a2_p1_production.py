"""A2-P1 formal configuration, process ownership, safety and bounded telemetry."""
import copy
import json
import os
from pathlib import Path
import threading
import time
import pytest
import rclpy
from rclpy.parameter import Parameter
from pus6_motor_driver.production_driver import (FORMAL_DEFAULTS,THREE_DEFAULTS,
    validate_configuration,create_driver,ProductionDriverNode)
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.j14_process_uart import (ProcessClient,ServoCore,PERIOD_NS,LEASE_NS,
                                               socket_pair,send_packet,receive_packets)
from pus6_motor_driver.fake_bus import MockServoBus


def overrides(tmp_path,architecture='three_process',mode='qualification',duration=.18):
    return [Parameter('driver_architecture',value=architecture),Parameter('driver_audit_directory',value=str(tmp_path)),
        Parameter('servo_run_duration_s',value=float(duration)),Parameter('driver_telemetry_mode',value=mode)]


def config(tmp_path,**changes):
    c=dict(production=True,feedback_bridge=True,enable_coordinated_handshake=True,
        epoch=time.monotonic_ns(),output_dir=str(tmp_path),hardware_mode='mock',port='/dev/DO_NOT_OPEN',
        baudrate=921600,servo_speed=50,maintenance_mode=True,reconnect_max_jump_steps=80,
        target_timeout_sec=.2,max_servo_duration_s=.18,telemetry_mode='qualification',telemetry_capacity_cycles=4096)
    c.update(changes);return c


def read(path):return [json.loads(l) for l in path.read_text().splitlines()]


def pump(c,secs=.2):
    end=time.monotonic()+secs
    while time.monotonic()<end:
        c.send_state();c.receive();time.sleep(.002)


def test_A_default_factory_C_D_E_F_G_H_X_no_environment_flags(tmp_path,monkeypatch):
    for key in list(os.environ):
        if key.startswith('PUS6_J14_'):monkeypatch.delenv(key)
    import serial
    monkeypatch.setattr(serial,'Serial',lambda *a,**k:pytest.fail('parent opened UART'))
    rclpy.init();node=None
    try:
        node=create_driver([p for p in overrides(tmp_path) if p.name!='driver_architecture'])
        assert isinstance(node,ProductionDriverNode)
        c=node._process_client
        assert c.use_feedback_bridge and c.bridge_proc and c.ready['modules_without_ros']
        assert c.sock.fileno()>=0 and not hasattr(c,'fb_sock') and not hasattr(node.j7_adapter,'serial')
        assert not any(p.topic_name in ('/joint_states','/pus6/joint_states','/pus6/joint_feedback/timed',
                                       '/pus6/joint_command/timed') for p in node.publishers)
        with pytest.raises(RuntimeError):node._accept_child_feedback({})
        assert node.telemetry_health_ok
        time.sleep(.05)
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
    cycles=read(tmp_path/'child_cycles.jsonl');hs=[x for x in read(tmp_path/'child_ready.jsonl') if x['type']=='servo_start'][0]
    assert cycles[0]['cycle_seq']==0 and cycles[0]['start_ns']>=hs['servo_start_ns']
    assert hs['servo_start_ns']>=max(hs[k] for k in ('t_child_ready_ns','t_bridge_ready_ns','t_parent_ready_ns'))
    assert json.loads((tmp_path/'child_final.json').read_text())['hardware_write_count']==0


def test_B_legacy_factory_has_no_child_and_remains_original_class(tmp_path):
    rclpy.init();node=None
    try:
        node=create_driver(overrides(tmp_path,'legacy_single_process'))
        assert type(node) is Pus6MotorDriverNode and not hasattr(node,'_process_client')
        assert node.joint_state_pub is not None
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('key,value',[('baudrate',115200),('write_rate_hz',100.),('target_timeout_sec',1.),
    ('reconnect_max_jump_steps',81),('enable_coordinated_handshake',False),('driver_architecture','bad')])
def test_configuration_rejects_drift_before_Uart(key,value):
    opts={**FORMAL_DEFAULTS,**THREE_DEFAULTS};opts[key]=value
    with pytest.raises(ValueError):validate_configuration(opts)


def test_I_K_L_M_N_O_P_Q_frozen_values_and_qualification_capacity(tmp_path):
    from pus6_motor_driver.j7_real_uart_adapter import J14_UART_WRITE_TIMEOUT_S
    from pus6_motor_driver.safety import MAX_CURRENT_MA
    assert PERIOD_NS==20_000_000 and LEASE_NS==1_000_000_000
    assert J14_UART_WRITE_TIMEOUT_S==.030 and MAX_CURRENT_MA==1500
    opts={**FORMAL_DEFAULTS,**THREE_DEFAULTS};opts['servo_run_duration_s']=600.
    validate_configuration(opts) # config capability only, never run 600s
    core=ServoCore(config(tmp_path),MockServoBus(initial_positions=[1000]*6))
    try:
        core.initialize()
        assert core.command.timeout_ns==200_000_000 and core.command.jump_limit==80
        assert core.writer.rx_drain_budget_s==.004 and core.writer.min_frame_interval_ns==600_000
        assert core.feedback.warning_feedback_age_ns==283_300_000 and core.feedback.hard_stop_feedback_age_ns==LEASE_NS
    finally:core.close()


def test_production_telemetry_all_containers_bounded_without_stop_gate(tmp_path):
    from collections import deque
    core=ServoCore(config(tmp_path,telemetry_mode='production'),MockServoBus(initial_positions=[1000]*6))
    try:
        core.initialize()
        for collection in (core.rows,core.write_rows,core.rec.tier1_summary,core.rec.all_stall_events,
                           core.rec.rx_slow_read_events,core.writer._cycles,core.writer._rx_records,core.feedback._cycles):
            assert isinstance(collection,deque) and collection.maxlen is not None
        # Watchlist threshold itself never invokes stop; fake expected wake only.
        p=core.cycle(time.monotonic_ns()-31_000_000)
        assert p['type']=='feedback' and not core.writer.stopped and core.health['wake_watchlist_count']==1
    finally:core.close()
    summary=json.loads((tmp_path/'child_health_summary.json').read_text())
    assert summary['cycles']==1 and summary['motion_writes']==6


def test_U_bridge_death_local_pause_and_health(tmp_path):
    c=ProcessClient(config(tmp_path,max_servo_duration_s=0.))
    try:
        c.bridge_proc.kill();c.bridge_proc.wait(timeout=3)
        pump(c,.12);c.proc.wait(timeout=3)
        assert c.bridge_stopped or c.bridge_proc.poll() is not None
        final=json.loads((tmp_path/'child_final.json').read_text())
        assert final['closed'] and final['pause_success'] and final['fault']=='feedback_ipc_closed'
    finally:c.close()


def test_V_child_death_parent_bridge_invalid(tmp_path):
    c=ProcessClient(config(tmp_path,max_servo_duration_s=0.))
    try:
        c.proc.kill();c.proc.wait(timeout=3);c.receive()
        c.bridge_proc.wait(timeout=3)
        assert c.stopped and c.bridge_proc.poll() is not None
        end=read(tmp_path/'bridge_lifecycle.jsonl')[-1]
        assert end['health_ok'] is False and end['exit_reason']=='child_feedback_eof'
    finally:c.close()


def test_W_normal_three_process_shutdown_order(tmp_path):
    c=ProcessClient(config(tmp_path,max_servo_duration_s=0.));pump(c,.08);c.close()
    life=read(tmp_path/'child_lifecycle.jsonl')
    assert [r['event'] for r in life]==['serial_owner','pause_complete','serial_closed','child_exit']
    assert c.proc.returncode==c.bridge_proc.returncode==0
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['cycles']>=3 and final['pause_success'] and final['hardware_write_count']==0


def test_X_environment_flags_cannot_choose_normal_architecture(tmp_path,monkeypatch):
    monkeypatch.setenv('PUS6_J14_SEPARATE_UART_PROCESS','0');monkeypatch.setenv('PUS6_J14_FEEDBACK_BRIDGE','0')
    monkeypatch.setenv('PUS6_J14_PROCESS_WINDOW_S','bad');monkeypatch.setenv('PUS6_J14_DEFER_DRIVER_DIAGNOSTICS','1')
    rclpy.init();node=None
    try:
        node=create_driver(overrides(tmp_path))
        assert isinstance(node,ProductionDriverNode) and node._process_client.use_feedback_bridge
        assert node._process_client.config['max_servo_duration_s']==.18
        assert node._defer_driver_diagnostics is False
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


def test_J_R_production_order_latest_timeout_abort_pause(tmp_path):
    from test_j14_a1_9_process_uart import TimeoutBus,command
    from pus6_motor_driver.protocol import parse_frame
    bus=TimeoutBus(3);core=ServoCore(config(tmp_path),bus)
    try:
        core.initialize();bus.written_frames.clear();now=time.monotonic_ns()
        core.command.ingest([command(core.config['epoch'],seq,[1000+seq]*6,now) for seq in (1,2,3)],now,[1000]*6)
        packet=core.cycle(now)
        frames=[parse_frame(f) for f in bus.written_frames]
        assert [f.motor_id for f in frames if f.register_addr==0x29]==[1,2,3]
        assert [f.motor_id for f in frames if f.register_addr==0x1A]==list(range(1,7))
        assert packet['type']=='fault' and core.command.seq==3 and core.pauses[-1]['success']
        assert core.health['tx_timeouts']==1 and core.health['fault_cycles']==1
    finally:core.close()


@pytest.mark.parametrize('kind',['fault','overcurrent'])
def test_S_Q_production_motor_fault_current_local_pause(tmp_path,kind):
    from test_j14_a1_3_communication_recovery import RecoverySerial
    from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
    port=RecoverySerial();bus=J7UartAdapter(port,timeout_s=.010,min_frame_interval_s=.0001,tx_queue_diagnostics=True)
    core=ServoCore(config(tmp_path),bus)
    try:
        core.initialize()
        if kind=='fault':port.device.faults[3]=1
        else:port.device.currents[3]=1501
        assert core.cycle(time.monotonic_ns())['type']=='fault'
        assert core.writer.stopped and core.pauses[-1]['success'] and port.hardware_write_count==0
    finally:core.close()


def test_T_actual_production_parent_death_child_pause_bridge_exit(tmp_path):
    import subprocess,sys
    code='''import sys,json,time
from pus6_motor_driver.j14_process_uart import ProcessClient
c=ProcessClient(json.loads(sys.argv[1]))
print('PIDS:'+str(c.proc.pid)+':'+str(c.bridge_proc.pid),flush=True)
while True:
 c.send_state();c.receive();time.sleep(.005)
'''
    p=subprocess.Popen([sys.executable,'-c',code,json.dumps(config(tmp_path,max_servo_duration_s=0.))],stdout=subprocess.PIPE,text=True)
    try:
        while True:
            line=p.stdout.readline()
            assert line
            if line.startswith('PIDS:'):break
        p.kill();p.wait(timeout=3)
        deadline=time.monotonic()+3
        while not (tmp_path/'child_final.json').exists() and time.monotonic()<deadline:time.sleep(.005)
        final=json.loads((tmp_path/'child_final.json').read_text())
        assert final['fault']=='parent_ipc_closed' and final['pause_success'] and final['closed']
    finally:
        if p.poll() is None:p.kill();p.wait(timeout=3)


def test_production_bridge_unique_publishers_actual_graph(tmp_path,monkeypatch):
    # Use an isolated software ROS graph; discovery of names can follow endpoints.
    monkeypatch.setenv('ROS_DOMAIN_ID','177')
    rclpy.init();node=None
    try:
        node=create_driver(overrides(tmp_path,duration=1.2))
        until=time.monotonic()+1
        topics=('/joint_states','/pus6/joint_feedback/timed','/pus6/joint_command/timed')
        infos={}
        while time.monotonic()<until:
            infos={topic:node.get_publishers_info_by_topic(topic) for topic in topics}
            if all(len(rows)==1 and rows[0].node_name=='pus6_feedback_bridge_node' for rows in infos.values()):break
            time.sleep(.01)
        assert all(len(rows)==1 and rows[0].node_name=='pus6_feedback_bridge_node' for rows in infos.values()), {k:[(x.node_name,x.topic_type) for x in v] for k,v in infos.items()}
        assert infos['/joint_states'][0].topic_type=='sensor_msgs/msg/JointState'
        assert infos['/pus6/joint_feedback/timed'][0].topic_type=='pus6_msgs/msg/TimedJointFeedback'
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('mode',['production','qualification','debug'])
def test_bridge_telemetry_profiles_and_watchlist_not_stop(tmp_path,monkeypatch,mode):
    from pus6_motor_driver.j14_feedback_bridge import Pus6FeedbackBridgeNode
    monkeypatch.setenv('ROS_DOMAIN_ID','178')
    rclpy.init();a,b=socket_pair();node=None
    try:
        cfg=config(tmp_path,telemetry_mode=mode)
        node=Pus6FeedbackBridgeNode(a,cfg);now=time.monotonic_ns()
        row=dict(epoch=cfg['epoch'],cycle_seq=0,command_seq=0,positions=[1000]*6,currents=[0]*6,
            faults=[0]*6,rx_ns=[now]*6,targets=[1000]*6,tx_start_ns=[now]*6,tx_end_ns=[now]*6,
            cycle_start_ns=now-200_000_000,cycle_end_ns=now-150_000_000,feedback_ready_ns=now-150_000_000)
        node._process_feedback_packet(row,now,time.thread_time_ns(),time.process_time_ns())
        assert node._running and node._ipc_watchlist_count==1
        assert ('t_message_build_done_ns' in node._records[0])==(mode=='debug')
        assert node._records.maxlen is not None
        assert node.joint_state_pub.qos_profile.depth==10
        assert node.timed_feedback_pub.qos_profile.depth==5
    finally:
        if node:node.destroy_node()
        a.close();b.close();rclpy.shutdown()
