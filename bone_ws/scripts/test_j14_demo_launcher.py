"""Stage-specific Demo selection, failure, interruption and teardown tests. No hardware."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import j14_demo_launcher as demo
from j14_demo_controller import controller_spec, write_json


@pytest.mark.parametrize('mode,clsname', [
    ('feedforward','J11BaseFeedforwardRunner'),
    ('feedforward_pd','J11M2PidResidualRunner'),
    ('feedforward_ladrc','J13LadrcShadowRunner')])
def test_mode_uses_existing_runner_and_config(mode, clsname):
    import yaml
    cls, params, path = controller_spec(mode)
    assert cls.__name__ == clsname
    if mode != 'feedforward':
        data = yaml.safe_load(path.read_text())
        key = 'j11_m2_attitude_pid' if mode == 'feedforward_pd' else 'j13_ladrc_shadow_runner'
        assert all(data[key]['ros__parameters'][k]==v for k,v in params.items() if k!='controller_mode')
        assert [params['ki_'+a] for a in ('roll','pitch','yaw')]==[0,0,0]
        assert params['kd_roll']==0
        if mode=='feedforward_ladrc':
            assert params['controller_mode']=='3axis_active'


def test_invalid_mode():
    with pytest.raises(ValueError):
        controller_spec('invented')


def test_numpy_ros_fixed_array_is_json_serializable(tmp_path):
    state = dict(counts={},last_rx={},child_command_min=None,child_command_max=None)
    for steps in ([1000]*6,[1002,1001,1000,999,998,997]):
        demo.update_observation(state,'child_command',SimpleNamespace(target_steps=np.array(steps,dtype=np.int32)))
    write_json(tmp_path/'observed.json',state)
    data=json.loads((tmp_path/'observed.json').read_text())
    assert data['counts']['child_command']==2
    assert data['child_command_min']==[1000,1000,1000,999,998,997]
    assert data['child_command_max']==[1002,1001,1000,1000,1000,1000]


class FakeProcess:
    def __init__(self, events, role, status=None, timeout=False):
        self.events,self.role,self.status,self.timeout=events,role,status,timeout
        self.pid=100
    def poll(self):
        return self.status
    @property
    def returncode(self):
        return self.status
    def send_signal(self,sig):
        self.events.append((self.role,sig))
    def wait(self,timeout):
        if self.timeout:
            self.timeout=False
            raise subprocess.TimeoutExpired(self.role,timeout)
        self.status=0
        return 0


def test_shutdown_order_stops_output_before_relay_and_driver(tmp_path):
    events=[]
    owned=demo.OwnedProcesses(tmp_path,{})
    owned.procs={role:FakeProcess(events,role) for role in ('controller','relay','robot','mock_imu')}
    owned.cleanup()
    assert [role for role,_ in events]==['controller','relay','mock_imu','robot']
    assert all(sig==signal.SIGINT for _,sig in events)
    assert all(e['returncode']==0 and not e['escalation'] for e in owned.events)


def test_relay_timeout_marks_failure_without_sigkill(tmp_path):
    events=[]
    owned=demo.OwnedProcesses(tmp_path,{})
    owned.procs['relay']=FakeProcess(events,'relay',timeout=True)
    owned.cleanup()
    assert events==[('relay',signal.SIGINT),('relay',signal.SIGTERM)]
    assert owned.events[0]['escalation'] is True


@pytest.mark.parametrize('role', ['robot','relay','controller'])
def test_child_exit_detected_even_returncode_zero(tmp_path,role):
    owned=demo.OwnedProcesses(tmp_path,{})
    owned.procs[role]=FakeProcess([],role,status=0)
    with pytest.raises(RuntimeError,match=role+' exited unexpectedly'):
        owned.check()


def test_startup_timeout():
    owned=SimpleNamespace(check=lambda:None)
    with pytest.raises(TimeoutError,match='Driver startup timeout'):
        demo.wait_ready(lambda:False,'Driver',0,threading.Event(),owned,None)


def test_spawn_failure_releases_context_and_preserves_failure_log(tmp_path,monkeypatch):
    import rclpy
    lifecycle=[]
    monkeypatch.setattr(demo,'runtime_processes',lambda:[])
    monkeypatch.setattr(demo,'serial_status',lambda:dict(free=True))
    monkeypatch.setattr(rclpy,'init',lambda **kw:lifecycle.append('init'))
    monkeypatch.setattr(rclpy,'ok',lambda:True)
    monkeypatch.setattr(rclpy,'shutdown',lambda:lifecycle.append('shutdown'))
    observer=SimpleNamespace(destroy_node=lambda:lifecycle.append('destroy'))
    monkeypatch.setattr(demo,'make_observer',lambda:(observer,dict(counts={},last_rx={})))
    import j14_demo_home
    monkeypatch.setattr(j14_demo_home,'attach_feedback',lambda node:dict(count=0))
    def unavailable(*args,**kwargs):
        raise OSError('injected spawn failure')
    monkeypatch.setattr(demo.subprocess,'Popen',unavailable)
    out=tmp_path/'session'
    assert demo.run_demo('feedforward','mock',out)==1
    assert lifecycle==['init','destroy','shutdown']
    result=json.loads((out/'demo_final.json').read_text())
    assert result['startup']=='NOT_READY' and result['active'] is False
    assert 'injected spawn failure' in result['stop_reason']
    assert result['safe_shutdown'] is False
    assert (out/'robot.log').exists()


def test_relay_failure_aborts_startup(tmp_path):
    owned=demo.OwnedProcesses(tmp_path,{})
    owned.procs['relay']=FakeProcess([],'relay',status=1)
    with pytest.raises(RuntimeError,match='relay exited unexpectedly'):
        demo.wait_ready(lambda:False,'relay',1,threading.Event(),owned,None)


def test_ctrl_c_during_startup_does_not_wait_for_ready():
    stop=threading.Event();stop.set()
    with pytest.raises(InterruptedError):
        demo.wait_ready(lambda:False,'Dual IMU',10,stop,None,None)


def test_driver_stop_aborts_active_session(tmp_path):
    write_json(tmp_path/'child_final.json',{'fault':'mock_failure'})
    with pytest.raises(RuntimeError,match='Driver STOP: mock_failure'):
        demo.OwnedProcesses(tmp_path,{}).check()


def test_historical_wrappers_preserved_and_new_shells_use_exec():
    for mode in demo.MODES:
        path=demo.SCRIPTS/f'run_j14_demo_{mode}.sh'
        text=path.read_text()
        assert 'exec python3' in text and f'--mode {mode}' in text
        assert 'run_j7_servo_home_park' not in text


@pytest.mark.parametrize('method', ['normal','sigint','sigterm'])
def test_relay_graceful_exit_subprocess(tmp_path,method):
    out=tmp_path/method
    if method=='normal':
        script=("import sys, threading; from pathlib import Path; "
                f"sys.path.insert(0,{str(demo.SCRIPTS)!r}); "
                "from j14_a2_q3_feedback_relay import run_relay; "
                "e=threading.Event(); threading.Timer(.3,e.set).start(); "
                f"raise SystemExit(run_relay(Path({str(out)!r}),e))")
        command=[sys.executable,'-c',script]
    else:
        command=[sys.executable,str(demo.SCRIPTS/'j14_a2_q3_feedback_relay.py'),'--out-dir',str(out)]
    proc=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    try:
        import time
        deadline=time.monotonic()+5
        while not (out/'relay_lifecycle.jsonl').exists() and proc.poll() is None:
            assert time.monotonic()<deadline
            time.sleep(.025)
        start=time.monotonic()
        if method!='normal':
            proc.send_signal(signal.SIGINT if method=='sigint' else signal.SIGTERM)
        output,_=proc.communicate(timeout=2)
        assert proc.returncode==0,output.decode()
        assert time.monotonic()-start<1.5
        assert json.loads((out/'relay_summary.json').read_text())['exit_reason']=='clean_shutdown'
    finally:
        if proc.poll() is None:
            proc.terminate();proc.wait(timeout=2)
