#!/usr/bin/env python3
"""F2 startup HOME order and F1 shutdown qualification (Mock only)."""
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
    parser.add_argument('--scenario',choices=('normal','park','already_home','startup_timeout','startup_fault','invalid_feedback','hardware_fault','controller_exit','relay_exit'),default='normal')
    parser.add_argument('--out-dir',type=Path,required=True)
    args=parser.parse_args()
    args.out_dir.mkdir(parents=True,exist_ok=False)
    session=(args.out_dir/'session').resolve()
    env=dict(os.environ,ROS_DOMAIN_ID='177',J14_DEMO_MOCK_IMU_AMPLITUDE='.02')
    env['J14_DEMO_MOCK_INJECTION_DIR']=str(session)
    env['J14_DEMO_F2_SCENARIO']=args.scenario
    command=[sys.executable,str(ROOT/'bone_ws/scripts'/('j14_demo_f2_mock_launcher.py' if args.scenario=='invalid_feedback' else 'j14_demo_launcher.py')),
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
                    if args.scenario in ('hardware_fault',):
                        write_json(session/'mock_injection.json',dict(scenario=args.scenario,ns=time.monotonic_ns()))
                        observed=session/'mock_injection_observed.json'
                        observed_deadline=time.monotonic()+2
                        while not observed.exists() and time.monotonic()<observed_deadline:time.sleep(.01)
                        assert observed.exists(),'Mock bus injection not observed'
                    if args.scenario in ('normal','park','already_home'):
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
    expected='HOME_CONFIRMED' if args.scenario in ('normal','park','already_home') else 'SKIPPED'
    pause=next(e for e in events if e['event']=='driver_pause_request')
    resource_stops=[e for e in events if e['event']=='stop' and e['role']!='controller']
    startup_failure=args.scenario in ('startup_timeout','startup_fault','invalid_feedback')
    startup_home=result['startup_home']
    checks=dict(startup=(result['startup']=='PASS')!=startup_failure,active=result['active']!=startup_failure,
        command_chain=startup_failure or all(counts.get(k,0)>0 for k in ('target','ik','command','child_command')),
        parent_delivery=startup_failure or result['parent_command_delivery']['max_child_seq']>0,
        home_policy=home['state']==expected,
        pause_ack=sorted(result['pause_ack_ids'])==[1,2,3,4,5,6],
        pause_before_resource_teardown=all(pause['end_ns']<=e['monotonic_ns'] for e in resource_stops),
        shutdown=result['safe_shutdown'],residual=result['residual_processes']==[],
        hardware_write_count=result['hardware_write_count']==0,
        no_serial_open=json.loads((session/'mock_serial_guard.json').read_text())['serial_open_attempts']==0,
        ros_clean=not result['ros_node_exit_errors'] and not result['legacy_ros_shutdown_exception'],
        returncode=proc.returncode==(0 if args.scenario in ('normal','park','already_home') else 1))
    if args.scenario in ('normal','park','already_home') and 'feedback' in home:
        checks.update(actual_feedback=all(995<=s<=1005 for s in home['feedback']),
            fresh_feedback=all(t>home['start_ns'] for t in home['per_axis_rx_done_ns']),
            five_samples=home['consecutive_samples']>=5,
            home_before_pause=home['end_ns']<=pause['ns'],
            starts_away_from_home=any(s<995 or s>1005 for s in home['initial_feedback']))
    if args.scenario in ('controller_exit','relay_exit','hardware_fault'):
        checks['no_home_motion']=not (session/'home_recovery.json').exists()
    if args.scenario=='hardware_fault':
        checks['actual_child_fault']=bool(result['child_final']['fault']) and result['child_final']['fault']!='parent_shutdown'
    if startup_failure:
        checks['controller_not_spawned']=not any(e.get('role')=='controller' and e['event']=='start' for e in events)
        checks['imu_not_spawned']=not any(e.get('role')=='mock_imu' and e['event']=='start' for e in events)
        checks['startup_policy']=startup_home['state']==('HOME_TIMEOUT' if args.scenario=='startup_timeout' else 'SKIPPED')
        if args.scenario!='startup_timeout':checks['no_startup_motion']=startup_home['home_target_publish_count']==0
    elif result['active']:
        starts={e['role']:e for e in events if e['event']=='start'}
        transitions=[json.loads(line) for line in (session/'controller_states.jsonl').read_text().splitlines()]
        capture=next(e for e in transitions if e['state']=='CAPTURING_REFERENCE')
        active=next(e for e in transitions if e['state']=='ACTIVE')
        checks.update(startup_home=startup_home['state']=='HOME_CONFIRMED',
            ordering=startup_home['end_ns']<startup_home['authority_released_ns']<starts['controller']['start_call_ns']<capture['monotonic_ns']<active['monotonic_ns'],
            imu_after_home=startup_home['end_ns']<starts['mock_imu']['start_call_ns'],
            startup_fresh=all(t>startup_home['start_ns'] for t in startup_home['per_axis_rx_done_ns']),
            startup_five=startup_home['consecutive_samples']==5)
        samples=startup_home['confirmation_samples']
        origin=startup_home['motion_start_ns'] or startup_home['start_ns']
        checks['all_five_fresh_in_band']=len(samples)==5 and all(
            all(t>origin for t in sample['rx_ns']) and all(995<=s<=1005 for s in sample['steps'])
            for sample in samples)
        checks['all_axes_advance']=all(all(t>prior for t,prior in zip(second['rx_ns'],first['rx_ns']))
            for first,second in zip(samples,samples[1:]))
        checks['already_home_no_pose']=startup_home['already_home'] and startup_home['home_target_publish_count']==0 if args.scenario=='already_home' else not startup_home['already_home'] and startup_home['home_target_publish_count']>0
    report=dict(mode=args.mode,scenario=args.scenario,checks=checks,startup_home=startup_home,home=home,
                child_fault=result['child_final']['fault'],hardware_write_count=result['hardware_write_count'],
                verdict='PASS' if all(checks.values()) else 'FAIL')
    write_json(args.out_dir/'check.json',report)
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report['verdict']=='PASS' else 1


if __name__=='__main__':
    sys.exit(main())
