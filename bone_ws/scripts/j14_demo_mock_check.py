#!/usr/bin/env python3
"""Short explicit MOCK-only integration check of Demo entrypoints and cleanup."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from j14_demo_controller import MODES, ROOT, write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--stop-kind', choices=('ctrl_c', 'controller_exit', 'relay_exit'), default='ctrl_c')
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=False)
    session = args.out_dir/'session'
    env = dict(os.environ, ROS_DOMAIN_ID='173')
    cmd = ['bash', str(ROOT/'bone_ws/scripts'/f'run_j14_demo_{args.mode}.sh'),
           '--hardware-mode', 'mock', '--out-dir', str(session)]
    with (args.out_dir/'launcher.log').open('x') as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+40
        active_at = None
        try:
            while proc.poll() is None:
                state_path = session/'controller_state.json'
                if active_at is None and state_path.exists():
                    if json.loads(state_path.read_text())['state']=='ACTIVE':
                        active_at = time.monotonic()
                if active_at is not None and time.monotonic()-active_at>=6:
                    if args.stop_kind=='ctrl_c':
                        proc.send_signal(signal.SIGINT)
                    else:
                        role = 'controller' if args.stop_kind=='controller_exit' else 'relay'
                        started = json.loads((session/'processes.json').read_text())
                        pid = next(e['pid'] for e in started if e['event']=='start' and e['role']==role)
                        # Only the MOCK process we started; intentional unexpected exit.
                        os.kill(pid, signal.SIGTERM)
                    break
                if time.monotonic()>deadline:
                    proc.send_signal(signal.SIGINT)
                    break
                time.sleep(.05)
            proc.wait(timeout=25)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=15)
    path = session/'demo_final.json'
    result = json.loads(path.read_text()) if path.exists() else {}
    counts = result.get('command_chain', {}).get('counts', {})
    checks = dict(startup=result.get('startup')=='PASS', active=result.get('active') is True,
                  command_chain=all(counts.get(k,0)>0 for k in ('target','ik','command','child_command')),
                  parent_acceptance=result.get('parent_command_delivery',{}).get('max_parent_seq',0)>0,
                  parent_to_child=result.get('parent_command_delivery',{}).get('max_child_seq',0)>0,
                  command_variation=result.get('command_chain',{}).get('command_min') != result.get('command_chain',{}).get('command_max'),
                  child_variation=result.get('command_chain',{}).get('child_command_min') != result.get('command_chain',{}).get('child_command_max'),
                  safe_shutdown=result.get('safe_shutdown') is True,
                  hardware_write_count=result.get('hardware_write_count')==0,
                  residual_processes=result.get('residual_processes')==[],
                  expected_returncode=proc.returncode==(0 if args.stop_kind=='ctrl_c' else 1))
    report = dict(mode=args.mode, stop_kind=args.stop_kind, returncode=proc.returncode,
                  checks=checks, counts=counts,
                  legacy_ros_shutdown_exception=result.get('legacy_ros_shutdown_exception'),
                  verdict='PASS' if all(checks.values()) else 'FAIL')
    write_json(args.out_dir/'check.json', report)
    print(json.dumps(report, indent=2))
    return 0 if report['verdict']=='PASS' else 1


if __name__=='__main__':
    sys.exit(main())
