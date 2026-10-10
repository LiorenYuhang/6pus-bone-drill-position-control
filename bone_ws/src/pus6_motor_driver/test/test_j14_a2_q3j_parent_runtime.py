"""Q3J entry selection and real ROS callback/Mock IPC regression."""
import json
import time
from types import SimpleNamespace
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray
from pus6_motor_driver import production_driver as production
from test_j14_a2_p1_production import overrides


@pytest.mark.parametrize('formal', [True, False])
def test_entry_inline_production_preserves_legacy_executor(monkeypatch, formal):
    selected = []
    class Formal:
        def destroy_node(self): selected.append('destroy')
    node = Formal() if formal else SimpleNamespace(destroy_node=lambda:selected.append('destroy'))
    class Executor:
        def add_node(self, value): assert value is node
        def spin(self): raise KeyboardInterrupt
        def shutdown(self, **kwargs): selected.append('shutdown')
    def single(): selected.append('single'); return Executor()
    def multi(**kwargs): selected.append(('multi',kwargs['num_threads'])); return Executor()
    monkeypatch.setattr(production,'ProductionDriverNode',Formal)
    monkeypatch.setattr(production,'create_driver',lambda:node)
    monkeypatch.setattr(production,'SingleThreadedExecutor',single)
    monkeypatch.setattr(production,'MultiThreadedExecutor',multi)
    monkeypatch.setattr(production.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(production.rclpy,'ok',lambda:False)
    production.main()
    assert selected == [('single' if formal else ('multi',3)), 'destroy', 'shutdown']


def test_callback_burst_latest_sequence_and_fault_rejection(tmp_path):
    rclpy.init(); node=None
    try:
        node=production.create_driver(overrides(tmp_path,duration=0.))
        updates=[]
        client=node._process_client
        original=client.update
        def update(targets, **kwargs):
            updates.append((list(targets),kwargs['seq']))
            original(targets,**kwargs)
        client.update=update
        invalid=Int32MultiArray();invalid.data=[1000]*5
        node._handle_command_joint_steps(invalid)
        assert node._target_in_count==0 and not updates
        for i in range(500):
            msg=Int32MultiArray();msg.data=[1000+i%5]*6
            node._handle_command_joint_steps(msg)
        assert node._target_in_count==500 and node._target_overwrite_count==499
        assert node._latest_target.targets_steps==[1004]*6
        assert [seq for _,seq in updates]==list(range(1,501))
        # Stop latch rejects further command ingress before any mailbox update.
        node._j7_fault_latched=True
        node._handle_command_joint_steps(msg)
        assert len(updates)==500 and node._target_in_count==500
    finally:
        if node:node.destroy_node()
        rclpy.shutdown()
    assert json.loads((tmp_path/'child_final.json').read_text())['hardware_write_count']==0


def test_real_ros_sustained_50hz_idle_heartbeat_and_shutdown(tmp_path):
    rclpy.init();node=source=None;executor=SingleThreadedExecutor()
    try:
        node=production.create_driver(overrides(tmp_path,duration=0.))
        source=Node('q3j_unit_command_source')
        publisher=source.create_publisher(Int32MultiArray,'/pus6/command_joint_steps',qos_profile_sensor_data)
        executor.add_node(node);executor.add_node(source)
        until=time.monotonic()+3
        while publisher.get_subscription_count()==0 and time.monotonic()<until:
            executor.spin_once(timeout_sec=.01)
        assert publisher.get_subscription_count()==1
        start=time.monotonic();deadline=start; count=0
        while time.monotonic()-start<1.2:
            if time.monotonic()>=deadline:
                msg=Int32MultiArray();msg.data=[1000+count%4]*6
                publisher.publish(msg);count+=1;deadline+=.02
            executor.spin_once(timeout_sec=.002)
        assert node._target_in_count>=58 and node.telemetry_health_ok
        # Command absence must preserve independent existing 10 Hz lease.
        until=time.monotonic()+.35
        while time.monotonic()<until:executor.spin_once(timeout_sec=.01)
        assert node.telemetry_health_ok
        sent=[r for r in node._process_client.rows if r['event']=='parent_send']
        assert any(r['seq'] is None for r in sent)
        seq=[r['seq'] for r in sent if r['seq'] is not None]
        assert all(b>a for a,b in zip(seq,seq[1:]))
        assert max(b['ns']-a['ns'] for a,b in zip(sent,sent[1:]))<250_000_000
    finally:
        if node:node.destroy_node()
        if source:source.destroy_node()
        executor.shutdown();rclpy.shutdown()
    final=json.loads((tmp_path/'child_final.json').read_text())
    assert final['hardware_write_count']==0 and final['pause_success']
    assert final['fault']=='parent_shutdown'
