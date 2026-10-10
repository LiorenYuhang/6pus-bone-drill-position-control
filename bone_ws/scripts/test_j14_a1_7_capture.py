"""A1.7 harness checks with synthetic files/processes only; no serial opens."""
import json
import pytest
import j14_a1_7_capture as capture


def clean_metadata():
    return dict(stop_reason='no_natural_timeout_within_window',shutdown_exception=False,
                runtime_anomaly=False,telemetry_complete=True,pause_ack_confirmed=True,
                residual_processes=[],serial_after={'free':True})


@pytest.mark.parametrize('field,value',[
    ('shutdown_exception',True),('runtime_anomaly',True),('telemetry_complete',False),
    ('pause_ack_confirmed',False),('residual_processes',[{'pid':1}]),
    ('serial_after',{'free':False}),('stop_reason','first_persisted_natural_TX_timeout'),
    ('shutdown_escalation','SIGTERM')])
def test_r2_requires_every_clean_condition(field,value):
    m=clean_metadata()
    assert capture.r2_admissible(m)
    m[field]=value
    assert not capture.r2_admissible(m)


def test_persisted_timeout_not_partial_record(tmp_path):
    path=tmp_path/'communication_recovery_events.jsonl'
    path.write_text('{"delivery_uncertain":true')
    assert capture.persisted_failure(tmp_path) is None
    row=dict(delivery_uncertain=True,configured_timeout_s=.030,cycle_seq=27)
    path.write_text(json.dumps(row)+'\n{"partial":')
    assert capture.persisted_failure(tmp_path)['record']==row
    (tmp_path/'tx_write_events.jsonl').write_text(json.dumps({'write_error':'SerialTimeoutException','cycle_seq':27})+'\n')
    assert capture.persisted_failure(tmp_path)['source']=='tx_write_events.jsonl'


def test_harness_first_persisted_failure_one_sigint_no_trace(tmp_path,monkeypatch):
    out=tmp_path/'run';signals=[];saved=[]
    monkeypatch.setattr(capture,'processes',lambda:[])
    monkeypatch.setattr(capture,'serial_check',lambda:{'free':True})
    monkeypatch.setattr(capture.subprocess,'check_output',
                        lambda *a,**kw:'mock-git\n' if kw.get('text') else b'mock-git\n')
    # Keep temporary evidence pointers in the test directory as well.
    real_path=capture.Path
    def scoped_path(value):
        return tmp_path/real_path(value).name if str(value).startswith('/tmp/j') else real_path(value)
    monkeypatch.setattr(capture,'Path',scoped_path)
    class Observer:
        def __init__(self,path):self.path=path
        def start(self):pass
        def stop_and_flush(self):self.path.write_text('heartbeat_seq\n')
    monkeypatch.setattr(capture,'HostHeartbeatObserver',Observer)
    class Process:
        returncode=None
        def __init__(self,cmd,**kw):
            saved.append((cmd,kw['env']))
            kw['stdout'].write('UART TX failure: cycle_seq=27 motor_id=3 classification=SerialTimeoutException\n')
            kw['stdout'].flush()
            (out/'communication_recovery_events.jsonl').write_text(json.dumps(dict(
                delivery_uncertain=True,configured_timeout_s=.030,cycle_seq=27))+'\n')
        def poll(self):return self.returncode
        def send_signal(self,sig):signals.append(sig)
        def wait(self,timeout):
            self.returncode=0
            for name in ['cycle_summary_full.csv','stall_diagnostic_timing.csv','transport_phase_timing.csv',
                         'tx_queue_state.csv','rx_read_events.csv']:
                (out/name).write_text('cycle_seq\n27\n')
            (out/'tx_write_events.jsonl').write_text(json.dumps(dict(write_error='SerialTimeoutException',cycle_seq=27))+'\n')
            (out/'stop_pause_audit.jsonl').write_text(json.dumps(dict(success=True,records=[
                dict(response_received=True,timeout=False) for _ in range(6)]))+'\n')
    monkeypatch.setattr(capture.subprocess,'Popen',Process)
    capture.run(out,1)
    m=json.loads((out/'run_metadata.json').read_text())
    assert m['stop_reason']=='first_persisted_natural_TX_timeout' and m['captured_timeout_count']==1
    assert m['failure_persisted_seen_ns']<=m['stop_requested_ns']
    assert signals==[capture.signal.SIGINT] and not m['r2_admissible']
    cmd,env=saved[0]
    assert cmd[:4]==['ros2','launch','pus6_bringup','pus6_system.launch.py']
    assert not any('strace' in x for x in cmd)
    assert env['PUS6_J14_RECOVERY_STAGE']=='fresh_state_only'
    assert 'PUS6_J14_TRACE_DIR' not in env
    # No physical driver process or serial object was constructed.
    assert m['strace'] is False and m['ptrace'] is False
