"""F2 software startup semantics; no physical serial access."""
import json
import threading
from types import SimpleNamespace
import pytest
import j14_demo_home as home
from test_j14_demo_f1 import setup_home


def test_already_home_requires_five_new_actual_packets_without_publish(monkeypatch,tmp_path):
    node,feedback,published,_=setup_home(monkeypatch)
    feedback['steps']=[1000]*6
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']=='HOME_CONFIRMED' and result['already_home']
    assert not published and result['motion_start_ns'] is None
    assert result['consecutive_samples']==5 and len(result['confirmation_samples'])==5
    assert (tmp_path/'startup_home.json').exists() and not (tmp_path/'home_recovery.json').exists()


def test_nonhome_startup_uses_shutdown_home_path(monkeypatch,tmp_path):
    node,feedback,published,_=setup_home(monkeypatch)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']=='HOME_CONFIRMED' and not result['already_home'] and published
    assert all(t>result['motion_start_ns'] for t in result['per_axis_rx_done_ns'])
    assert result['timeout_s']==home._home_timeout_from_feedback([1020]*6,[1000]*6)


@pytest.mark.parametrize('cause',['fault','overcurrent','missing_steps','missing_faults','missing_currents','missing_timestamps','zero_timestamp','future_timestamp','out_of_range','stale','driver_stop'])
def test_invalid_startup_skips_home_before_any_pose(monkeypatch,tmp_path,cause):
    node,feedback,published,clock=setup_home(monkeypatch)
    if cause=='fault':feedback['faults'][0]=1
    elif cause=='overcurrent':feedback['currents'][0]=home.MAX_CURRENT_MA+1
    elif cause.startswith('missing_'):
        key={'steps':'steps','faults':'faults','currents':'currents','timestamps':'rx_ns'}[cause[8:]]
        feedback[key]=feedback[key][:-1]
    elif cause=='zero_timestamp':feedback['rx_ns'][0]=0
    elif cause=='future_timestamp':feedback['rx_ns'][0]=clock.monotonic_ns()+1
    elif cause=='out_of_range':feedback['steps'][0]=home.FEEDBACK_UPPER_LIMIT+1
    elif cause=='stale':feedback['rx_ns']=[clock.monotonic_ns()-2_000_000_000]*6
    elif cause=='driver_stop':node.driver_stop_latched=True
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']=='SKIPPED' and not published


def test_replayed_packets_cannot_confirm_home(monkeypatch,tmp_path):
    import rclpy
    node,feedback,published,clock=setup_home(monkeypatch)
    feedback['steps']=[1000]*6
    repeated=clock.monotonic_ns()+10_000_000
    def spin(*args,**kwargs):
        clock.now+=.025
        feedback.update(count=feedback['count']+1,rx_ns=[repeated]*6)
    monkeypatch.setattr(rclpy,'spin_once',spin)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']!='HOME_CONFIRMED' and not published


def test_fault_on_fifth_sample_does_not_confirm(monkeypatch,tmp_path):
    import rclpy
    node,feedback,published,_=setup_home(monkeypatch)
    spin=rclpy.spin_once
    def receive(*args,**kwargs):
        spin(*args,**kwargs)
        if feedback['count']==5:feedback['faults'][0]=1
    monkeypatch.setattr(rclpy,'spin_once',receive)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']=='HOME_ABORTED' and result['consecutive_samples']==4


def test_already_home_drift_uses_same_motion_helper(monkeypatch,tmp_path):
    import rclpy
    node,feedback,published,_=setup_home(monkeypatch)
    feedback['steps']=[1000]*6
    spin=rclpy.spin_once
    def receive(*args,**kwargs):
        spin(*args,**kwargs)
        if feedback['count']==2:feedback['steps']=[1020]*6
    monkeypatch.setattr(rclpy,'spin_once',receive)
    result=home.recover_home(node,feedback,tmp_path,threading.Event(),lambda:{},phase='startup',allow_already_home=True)
    assert result['state']=='HOME_CONFIRMED' and not result['already_home'] and published
    assert len(result['confirmation_samples'])==5
