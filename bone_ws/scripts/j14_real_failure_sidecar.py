#!/usr/bin/env python3
"""Observe the unchanged J13 launcher; markers never publish ROS commands."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path('/home/lyapunov/bone_drill')
sys.path.insert(0, str(ROOT/'bone_ws/scripts'))
from j14_a1_1_diagnostics import relevant_processes


def marker(out, event, note):
    path = out/'operator_events.csv'
    rows = list(csv.DictReader(path.open())) if path.exists() else []
    row = dict(monotonic_ns=time.monotonic_ns(), event_index=len(rows), event=event, operator_note=note)
    with path.open('a') as f:
        writer = csv.DictWriter(f, fieldnames=list(row))
        if not rows: writer.writeheader()
        writer.writerow(row)
    print(json.dumps(row), flush=True)
    return row


def processes():
    result = relevant_processes()
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            args = [a for a in p.joinpath('cmdline').read_bytes().decode().split('\0') if a]
            if ('launch' in args and 'pus6_bringup' in args) or any(Path(a).name.startswith('j11_') and 'runner' in a for a in args):
                result.append(dict(pid=int(p.name), args=args))
        except (OSError, UnicodeError): pass
    return list({r['pid']:r for r in result}.values())


def serial_check():
    r = subprocess.run(['fuser','/dev/ttyUSB0','/dev/ttyACM0','/dev/ttyACM1'], capture_output=True, text=True)
    return dict(returncode=r.returncode, owners=r.stdout.strip(), free=r.returncode==1)


def snapshot_logs(out):
    for source, dest in [('/tmp/j13_robot.log','robot.log'),('/tmp/j13_plat_imu.log','platform_imu.log'),('/tmp/j13_base_imu.log','base_imu.log')]:
        path=Path(source)
        if path.exists(): (out/dest).write_bytes(path.read_bytes())


def run(out):
    if out.exists(): raise RuntimeError('Refusing to overwrite evidence')
    pre_processes=processes();pre_serial=serial_check()
    if pre_processes or not pre_serial['free']: raise RuntimeError(f'Resource conflict: {pre_processes} {pre_serial}')
    out.mkdir(parents=True)
    (out/'git_before.txt').write_text(subprocess.check_output(['git','status','--short','--branch'],cwd=ROOT,text=True))
    (out/'git_diff_before.patch').write_bytes(subprocess.check_output(['git','diff'],cwd=ROOT))
    pointers=[Path('/tmp')/f'{stage}_latest_run_dir' for stage in ['j11','j13','j14']]
    previous={p:p.read_bytes() if p.exists() else None for p in pointers}
    # Preserve temporary logs until new child logs have actually been opened.
    prior_logs={p:p.read_bytes() if p.exists() else None for p in [Path('/tmp/j13_robot.log'),Path('/tmp/j13_plat_imu.log'),Path('/tmp/j13_base_imu.log')]}
    for p in prior_logs: p.unlink(missing_ok=True)
    cmd=['python3','-u',str(ROOT/'bone_ws/src/h30_imu/scripts/j13_ladrc_staged_launcher.py'),
         '--mode','3axis_active','--execute-motion','--duration','120',
         '--stage',out.name,'--output-dir',str(out),'--runtime-diagnostics']
    meta=dict(command=cmd,pre_processes=pre_processes,pre_serial=pre_serial,
              start_monotonic_ns=time.monotonic_ns(),start_epoch_ns=time.time_ns(),
              max_elapsed_s=120,active_seen_ns=None,disturbance_window=None,
              first_observed_failure=None,observer_stop_reason=None,cleanup_signals=[])
    (out/'run_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    robot_pid=None;stop_started=None;stop_requested=False;launch_interrupt=False
    with (out/'launcher_runner.log').open('w') as log:
        proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None:
                now=time.monotonic_ns()
                snapshot_logs(out)
                runtime=(out/'robot.log').read_text() if (out/'robot.log').exists() else ''
                launch=(out/'launcher_runner.log').read_text()
                if robot_pid is None:
                    for r in processes():
                        if 'launch' in r['args'] and 'pus6_bringup' in r['args']:
                            robot_pid=r['pid'];meta['robot_launch_pid']=robot_pid;break
                if meta['active_seen_ns'] is None and ('J13-2C1 ACTIVE:' in launch or '[J13-ACTIVE] 3-AXIS LADRC ACTIVE' in launch):
                    meta['active_seen_ns']=now
                    print('OBSERVED 3-AXIS LADRC ACTIVE '+str(now),flush=True)
                if meta['active_seen_ns'] and not meta['disturbance_window'] and not stop_started and now-meta['active_seen_ns']>=20_000_000_000:
                    print('OPERATOR DISTURBANCE WINDOW START',flush=True)
                    meta['disturbance_window']=marker(out,'OPERATOR_DISTURBANCE_WINDOW_START','Window invitation only; physical push is not confirmed by this timestamp')
                if meta['first_observed_failure'] is None:
                    reason=None;detail=None
                    for line in runtime.splitlines():
                        if any(token in line for token in ['STOP latched','Failed to initialize J7 Servo backend','scheduler_deadline_exceeded','overcurrent']):
                            reason='driver_runtime_failure';detail=line;break
                        if 'J14_RX_READ_EVENT ' in line:
                            event=json.loads(line.split('J14_RX_READ_EVENT ',1)[1])
                            if event['read_wall_ns']>50_000_000:
                                reason='pathological_single_read_gt50ms';detail=event;break
                    if reason is None and ('[J13-SHADOW FAULT / STOP DETECTED]' in launch or '[J13 STAGED STARTUP ERROR]' in launch):
                        reason='runner_or_startup_failure';detail=launch.splitlines()[-8:]
                    if reason:
                        meta['first_observed_failure']=dict(observed_monotonic_ns=now,type=reason,detail=detail,active_at_observation=meta['active_seen_ns'] is not None)
                        print('FIRST FAILURE OBSERVED '+json.dumps(meta['first_observed_failure']),flush=True)
                        meta['observer_stop_reason']=reason;stop_started=now
                if stop_started is None and now-meta['start_monotonic_ns']>=120_000_000_000:
                    stop_started=now;meta['observer_stop_reason']='120s_elapsed_limit'
                if stop_started and not launch_interrupt:
                    # Signal only the ROS launch leader; launch forwards one SIGINT.
                    # Never signal its process group and then duplicate SIGINT to nodes.
                    if robot_pid:
                        try:
                            os.kill(robot_pid,signal.SIGINT)
                            meta['cleanup_signals'].append(dict(pid=robot_pid,signal='SIGINT',monotonic_ns=time.monotonic_ns()))
                        except ProcessLookupError: pass
                    launch_interrupt=True
                if stop_started and not stop_requested and now-stop_started>=1_500_000_000:
                    Path('/tmp/j13_stop_requested').touch();stop_requested=True
                if stop_started and now-stop_started>8_000_000_000:
                    os.kill(proc.pid,signal.SIGINT)
                    meta['cleanup_signals'].append(dict(pid=proc.pid,signal='SIGINT',monotonic_ns=time.monotonic_ns()))
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.terminate();proc.wait(timeout=5)
                    break
                time.sleep(.05)
        finally:
            if proc.poll() is None:
                Path('/tmp/j13_stop_requested').touch()
                if robot_pid:
                    try: os.kill(robot_pid,signal.SIGINT)
                    except ProcessLookupError: pass
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.send_signal(signal.SIGINT)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.terminate();proc.wait(timeout=5)
            snapshot_logs(out)
            for p,old in prior_logs.items():
                if old is None:p.unlink(missing_ok=True)
                else:p.write_bytes(old)
            for p,old in previous.items():
                if old is None:p.unlink(missing_ok=True)
                else:p.write_bytes(old)
    meta.update(end_monotonic_ns=time.monotonic_ns(),end_epoch_ns=time.time_ns(),
                launcher_returncode=proc.returncode,residual_processes=processes(),serial_after=serial_check())
    (out/'run_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    (out/'git_after.txt').write_text(subprocess.check_output(['git','status','--short','--branch'],cwd=ROOT,text=True))
    assert subprocess.check_output(['git','diff'],cwd=ROOT)==(out/'git_diff_before.patch').read_bytes(),'Production tracked diff changed'
    print('RUN ENDED '+json.dumps(meta),flush=True)
    if meta['residual_processes'] or not meta['serial_after']['free']:raise RuntimeError('Cleanup requires attention')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--execute-hardware',action='store_true');parser.add_argument('--mark');parser.add_argument('--note',default='Operator report logged now; actual physical onset not measured')
    args=parser.parse_args()
    if args.mark: marker(args.output_dir,args.mark,args.note)
    elif args.execute_hardware: run(args.output_dir)
    else: parser.error('Hardware execution requires --execute-hardware; marker mode requires --mark')
