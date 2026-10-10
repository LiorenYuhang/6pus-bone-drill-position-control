"""F1 HOME semantics and signal ownership; no UART and no production changes."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import pytest

sys.path.insert(0,str(Path(__file__).parent))
import j14_demo_home as home
from j14_demo_launcher import OwnedProcesses
from j14_demo_controller import write_json


class Clock:
    def __init__(self):self.now=100.
    def monotonic(self):return self.now
    def monotonic_ns(self):return int(self.now*1e9)


def setup_home(monkeypatch,feedback_step=1000,stale_samples=0):
    import rclpy
    from builtin_interfaces.msg import Time
    clock=Clock();monkeypatch.setattr(home,'time',clock)
    published=[]
    node=SimpleNamespace(z0=.132612268,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())),
        target_pose_pub=SimpleNamespace(publish=lambda m:published.append(m)),
        driver_stop_latched=False)
    prior_ns=clock.monotonic_ns()-100_000_000
    feedback=dict(count=0,steps=[1020]*6,rx_ns=[prior_ns]*6,faults=[0]*6,currents=[0]*6,last_rx_ns=0)
    def spin(node,**kwargs):
        clock.now+=.025
        feedback.update(count=feedback['count']+1,steps=[feedback_step]*6,
            rx_ns=[prior_ns]*6 if feedback['count']<stale_samples else [clock.monotonic_ns()]*6)
    monkeypatch.setattr(rclpy,'spin_once',spin)
    return node,feedback,published,clock


def test_home_reuses_only_existing_publisher_and_actual_fresh_feedback(monkeypatch,tmp_path):
    node,feedback,published,_=setup_home(monkeypatch,stale_samples=3)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{})
    assert result['state']=='HOME_CONFIRMED'
    assert result['consecutive_samples']==home.HOME_SAMPLES==5
    assert feedback['count']>=8
    assert all(t>result['start_ns'] for t in result['per_axis_rx_done_ns'])
    assert all(m.pose.position.z==node.z0 and m.pose.orientation.w==1 and m.pose.orientation.x==0 for m in published)
    assert result['target']==[1000]*6


@pytest.mark.parametrize('step,inside',[(994,False),(995,True),(1000,True),(1005,True),(1006,False)])
def test_existing_home_band_inclusive(step,inside):
    assert home._home_feedback_in_band([step]*6)==inside


def test_commanded_home_without_measured_home_times_out(monkeypatch,tmp_path):
    node,feedback,published,_=setup_home(monkeypatch,feedback_step=1020)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{})
    assert published and result['state']=='HOME_TIMEOUT'
    assert result['feedback']==[1020]*6 and result['consecutive_samples']==0
    assert result['timeout_s']==home._home_timeout_from_feedback([1020]*6,[1000]*6)


@pytest.mark.parametrize('cause',['fault','overcurrent','driver_stop','request','feedback_timeout'])
def test_home_aborts_before_any_new_pose_on_safety_failure(monkeypatch,tmp_path,cause):
    node,feedback,published,_=setup_home(monkeypatch)
    request={}
    if cause=='fault':feedback['faults'][0]=1
    if cause=='overcurrent':feedback['currents'][0]=home.MAX_CURRENT_MA+1
    if cause=='driver_stop':node.driver_stop_latched=True
    if cause=='request':request['action']='safety_pause'
    if cause=='feedback_timeout':feedback['rx_ns']=[0]*6
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:request)
    assert result['state']=='HOME_ABORTED' and not published


def test_direct_owned_sensor_leaf_exits_without_orphan(tmp_path):
    # Represents the ownership difference from ros2 run; no actual IMU device.
    code="import signal,time; signal.signal(signal.SIGINT,lambda *a:exit(0)); print('READY',flush=True); time.sleep(30)"
    owned=OwnedProcesses(tmp_path,dict(__import__('os').environ))
    proc=owned.start('base_imu',[sys.executable,'-u','-c',code])
    try:
        deadline=time.monotonic()+3
        while 'READY' not in (tmp_path/'base_imu.log').read_text():
            assert time.monotonic()<deadline
            time.sleep(.02)
        owned.stop('base_imu',1.5)
        assert proc.poll()==0
        assert not Path(f'/proc/{proc.pid}').exists()
        assert not owned.events[-1]['escalation']
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=2)
        for log in owned.logs:log.close()


def test_pause_request_targets_existing_parent_before_teardown(tmp_path,monkeypatch):
    import j14_demo_launcher as launcher
    fake_pid=1_000_000_000  # Cannot refer to a real /proc process on this host.
    write_json(tmp_path/'motor_driver_node_host.json',dict(pid=fake_pid,state='ready'))
    sent=[]
    def send(pid,sig):
        sent.append((pid,sig))
        write_json(tmp_path/'motor_driver_node_host.json',dict(pid=pid,state='exited',returncode=0))
    monkeypatch.setattr(launcher.os,'kill',send)
    owned=OwnedProcesses(tmp_path,{})
    owned.pause_driver()
    assert sent==[(fake_pid,signal.SIGINT)]
    assert owned.events[-1]['returncode']==0


def test_fault_arriving_during_home_stops_further_pose_publication(monkeypatch,tmp_path):
    import rclpy
    node,feedback,published,_=setup_home(monkeypatch,feedback_step=1020)
    spin=rclpy.spin_once
    def fault_after_second_callback(node,**kwargs):
        spin(node,**kwargs)
        if feedback['count']==2:
            feedback['faults'][0]=1
    monkeypatch.setattr(rclpy,'spin_once',fault_after_second_callback)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{})
    assert result['state']=='HOME_ABORTED'
    assert result['home_target_publish_count']==len(published)==2
