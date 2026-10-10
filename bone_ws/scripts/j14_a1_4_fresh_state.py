#!/usr/bin/env python3
"""One A1.4 fresh-state qualification, at most 10s; stop at HOLD or failure."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path('/home/lyapunov/bone_drill')
sys.path.insert(0,str(ROOT/'bone_ws/scripts'))
from j14_real_failure_sidecar import processes, serial_check
sys.path.insert(0,str(ROOT/'bone_ws/src/h30_imu/scripts'))
from j13_ladrc_staged_launcher import HostHeartbeatObserver


def run(out):
    if out.exists():raise RuntimeError('Refusing to overwrite evidence')
    before=dict(processes=processes(),serial=serial_check())
    if before['processes'] or not before['serial']['free']:raise RuntimeError(f'Resource conflict: {before}')
    out.mkdir(parents=True)
    previous={p:p.read_bytes() if p.exists() else None for p in
              [Path('/tmp/j14_latest_run_dir'),Path('/tmp/j11_latest_run_dir')]}
    for p in previous:p.write_text(str(out))
    cmd=['ros2','launch','pus6_bringup','pus6_system.launch.py',
         f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
         'hardware_mode:=active','maintenance_mode:=true','servo_scheduler_mode:=dedicated_thread',
         'telemetry_publish_mode:=async_latest','feedback_loss_policy:=age_based','enable_runtime_diagnostics:=true']
    metadata=dict(purpose='J14-A1.4-R1 fresh state only; no re-anchor/resume',command=cmd,
                  preflight=before,max_duration_s=10,start_monotonic_ns=time.monotonic_ns(),start_epoch_ns=time.time_ns())
    observer=HostHeartbeatObserver(out/'host_heartbeat_stalls.csv');observer.start()
    proc=None;reason='no_natural_timeout_within_10s';seen=0
    with (out/'robot.log').open('w') as log:
        try:
            environment=dict(os.environ, PUS6_J14_RECOVERY_STAGE='fresh_state_only')
            metadata['recovery_stage']='fresh_state_only'
            metadata['timeout_injection']=False
            proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=environment)
            start=time.monotonic()
            while proc.poll() is None and time.monotonic()-start<10:
                content=(out/'robot.log').read_text()
                events=[line for line in content.splitlines() if 'J14_COMM_RECOVERY_EVENT ' in line]
                if len(events)>seen:
                    for line in events[seen:]:
                        event=json.loads(line.split('J14_COMM_RECOVERY_EVENT ',1)[1])
                        print(json.dumps({k:event.get(k) for k in ['recovery_count','cycle_seq','failed_motor_id','state','success','failure_reason','recovery_duration_ns']}),flush=True)
                    seen=len(events)
                    reason='fresh_state_hold' if event.get('state')=='HOLD' and event.get('success') else 'recovery_failure'
                    break
                if any(token in content for token in ['STOP latched','Failed to initialize J7 Servo backend','Communication recovery failed:']):
                    reason='driver_or_recovery_failure';break
                time.sleep(.05)
            if proc.poll() is not None:reason='launch_exited'
        finally:
            if proc is not None and proc.poll() is None:
                # One SIGINT to launch itself; no duplicate process-group signal.
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    metadata['shutdown_escalation']='SIGTERM'
                    proc.terminate()
                    try:proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        metadata['shutdown_escalation']='SIGKILL';proc.kill();proc.wait(timeout=2)
            observer.stop_and_flush()
            for p,value in previous.items():
                if value is None:p.unlink(missing_ok=True)
                else:p.write_bytes(value)
    metadata.update(end_monotonic_ns=time.monotonic_ns(),end_epoch_ns=time.time_ns(),stop_reason=reason,
                    launch_returncode=proc.returncode if proc else None,residual_processes=processes(),serial_after=serial_check())
    pauses=out/'stop_pause_audit.jsonl'
    metadata['pause_audits']=[json.loads(s) for s in pauses.read_text().splitlines()] if pauses.exists() else []
    metadata['pause_ack_confirmed']=bool(metadata['pause_audits']) and metadata['pause_audits'][-1]['success']
    metadata['shutdown_exception']=any(token in (out/'robot.log').read_text() for token in ['Traceback','Error during J7 shutdown','Error dumping'])
    (out/'run_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2),flush=True)
    if metadata['residual_processes'] or not metadata['serial_after']['free']:raise RuntimeError('Resource cleanup incomplete')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True,type=Path);p.add_argument('--execute-hardware',action='store_true')
    args=p.parse_args()
    if not args.execute_hardware:p.error('--execute-hardware is required')
    run(args.output_dir.resolve())
