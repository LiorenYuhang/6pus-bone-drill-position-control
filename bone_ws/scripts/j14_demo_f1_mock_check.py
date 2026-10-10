#!/usr/bin/env python3
"""F1 short Mock-only HOME/safety-stop/ownership qualification."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from j14_demo_controller import MODES,ROOT,write_json


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=MODES,default='feedforward')
    parser.add_argument('--scenario',choices=('normal','home_timeout','hardware_fault','controller_exit','relay_exit'),default='normal')
    parser.add_argument('--out-dir',type=Path,required=True)
    args=parser.parse_args()
    args.out_dir.mkdir(parents=True,exist_ok=False)
    session=(args.out_dir/'session').resolve()
    env=dict(os.environ,ROS_DOMAIN_ID='177',J14_DEMO_MOCK_IMU_AMPLITUDE='.02')
    if args.scenario in ('home_timeout','hardware_fault'):
        env['J14_DEMO_MOCK_INJECTION_DIR']=str(session)
    command=[sys.executable,str(ROOT/'bone_ws/scripts/j14_demo_launcher.py'),
             '--mode',args.mode,'--hardware-mode','mock','--out-dir',str(session)]
    write_json(args.out_dir/'conditions.json',dict(command=command,scenario=args.scenario,
        mock_imu_amplitude_rad=.02,domain=177,hardware_mode='mock',start_epoch_ns=time.time_ns()))
    active_at=None;injected=False
    with (args.out_dir/'launcher.log').open('x') as log:
        proc=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+35
            while proc.poll() is None:
                state=session/'controller_state.json'
                if state.exists() and active_at is None and json.loads(state.read_text())['state']=='ACTIVE':
                    active_at=time.monotonic()
                if active_at is not None and time.monotonic()-active_at>=1.0 and not injected:
                    injected=True
                    if args.scenario in ('home_timeout','hardware_fault'):
                        write_json(session/'mock_injection.json',dict(scenario=args.scenario,ns=time.monotonic_ns()))
                        observed=session/'mock_injection_observed.json'
                        observed_deadline=time.monotonic()+2
                        while not observed.exists() and time.monotonic()<observed_deadline:time.sleep(.01)
                        assert observed.exists(),'Mock bus injection not observed'
                    if args.scenario in ('normal','home_timeout'):
                        proc.send_signal(signal.SIGINT)
                    elif args.scenario in ('controller_exit','relay_exit'):
                        role='controller' if args.scenario=='controller_exit' else 'relay'
                        starts=json.loads((session/'processes.json').read_text())
                        pid=next(e['pid'] for e in starts if e['event']=='start' and e['role']==role)
                        os.kill(pid,signal.SIGTERM)
                if time.monotonic()>deadline:
                    proc.send_signal(signal.SIGTERM)
                    break
                time.sleep(.02)
            proc.wait(timeout=20)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGTERM);proc.wait(timeout=20)
    result=json.loads((session/'demo_final.json').read_text())
    events=json.loads((session/'processes.json').read_text())
    counts=result.get('command_chain',{}).get('counts',{})
    home=result.get('home_recovery',{})
    expected='HOME_CONFIRMED' if args.scenario=='normal' else ('HOME_TIMEOUT' if args.scenario=='home_timeout' else 'SKIPPED')
    pause=next(e for e in events if e['event']=='driver_pause_request')
    resource_stops=[e for e in events if e['event']=='stop' and e['role']!='controller']
    checks=dict(startup=result['startup']=='PASS',active=result['active'],
        command_chain=all(counts.get(k,0)>0 for k in ('target','ik','command','child_command')),
        parent_delivery=result['parent_command_delivery']['max_child_seq']>0,
        home_policy=home['state']==expected,
        pause_ack=sorted(result['pause_ack_ids'])==[1,2,3,4,5,6],
        pause_before_resource_teardown=all(pause['end_ns']<=e['monotonic_ns'] for e in resource_stops),
        shutdown=result['safe_shutdown'],residual=result['residual_processes']==[],
        hardware_write_count=result['hardware_write_count']==0,
        ros_clean=not result['ros_node_exit_errors'] and not result['legacy_ros_shutdown_exception'],
        returncode=proc.returncode==(0 if args.scenario=='normal' else 1))
    if args.scenario=='normal':
        checks.update(actual_feedback=all(995<=s<=1005 for s in home['feedback']),
            fresh_feedback=all(t>home['start_ns'] for t in home['per_axis_rx_done_ns']),
            five_samples=home['consecutive_samples']>=5,
            home_before_pause=home['end_ns']<=pause['ns'],
            starts_away_from_home=any(s<995 or s>1005 for s in home['initial_feedback']))
    if args.scenario in ('controller_exit','relay_exit','hardware_fault'):
        checks['no_home_motion']=not (session/'home_recovery.json').exists()
    if args.scenario=='hardware_fault':
        checks['actual_child_fault']=bool(result['child_final']['fault']) and result['child_final']['fault']!='parent_shutdown'
    if args.scenario=='home_timeout':
        checks['timeout_before_pause']=home['end_ns']<=pause['ns']
        checks['feedback_outside_home']=any(s<995 or s>1005 for s in home['feedback'])
    report=dict(mode=args.mode,scenario=args.scenario,checks=checks,home=home,
                child_fault=result['child_final']['fault'],hardware_write_count=result['hardware_write_count'],
                verdict='PASS' if all(checks.values()) else 'FAIL')
    write_json(args.out_dir/'check.json',report)
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report['verdict']=='PASS' else 1


if __name__=='__main__':
    sys.exit(main())
