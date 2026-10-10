#!/usr/bin/env python3
"""A1.8 static observation: no tracing; stop at first persisted natural failure."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path('/home/lyapunov/bone_drill')
sys.path.insert(0, str(ROOT/'bone_ws/scripts'))
from j14_real_failure_sidecar import processes, serial_check
sys.path.insert(0, str(ROOT/'bone_ws/src/h30_imu/scripts'))
from j13_ladrc_staged_launcher import HostHeartbeatObserver


def command():
    return ['ros2', 'launch', 'pus6_bringup', 'pus6_system.launch.py',
            f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
            'hardware_mode:=active', 'maintenance_mode:=true',
            'servo_scheduler_mode:=dedicated_thread', 'telemetry_publish_mode:=async_latest',
            'feedback_loss_policy:=age_based', 'enable_runtime_diagnostics:=true']


def json_lines(path):
    if not path.exists():
        return []
    records = []
    for line in path.read_text().splitlines():
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            # The driver may currently be appending its last line.
            continue
    return records


def persisted_failure(out):
    for row in json_lines(out/'tx_write_events.jsonl'):
        if row.get('write_error') == 'SerialTimeoutException':
            return dict(source='tx_write_events.jsonl', record=row)
    for row in json_lines(out/'communication_recovery_events.jsonl'):
        if row.get('delivery_uncertain') and row.get('configured_timeout_s') == .030:
            return dict(source='communication_recovery_events.jsonl', record=row)
    return None


def r2_admissible(metadata):
    return (metadata['stop_reason'] == 'no_natural_timeout_within_window'
            and not metadata['shutdown_exception'] and not metadata['runtime_anomaly']
            and not metadata.get('shutdown_escalation') and metadata['telemetry_complete']
            and metadata['pause_ack_confirmed'] and not metadata['residual_processes']
            and metadata['serial_after']['free'])


def run(out, run_id):
    if out.exists():
        raise RuntimeError('Refusing to overwrite evidence')
    if run_id == 2:
        previous_run = out.parent/'j14_a1_8_r1_static_no_trace'/'run_metadata.json'
        if not previous_run.exists() or not r2_admissible(json.loads(previous_run.read_text())):
            raise RuntimeError('R2 requires a clean no-timeout R1')
    before = dict(processes=processes(), serial=serial_check())
    if before['processes'] or not before['serial']['free']:
        raise RuntimeError(f'Resource conflict: {before}')
    out.mkdir(parents=True)
    for name, args in [('git_before.txt', ['git','status','--short','--branch']),
                       ('git_diff_before.patch', ['git','diff'])]:
        (out/name).write_bytes(subprocess.check_output(args,cwd=ROOT))
    config = ROOT/'bone_ws/src/h30_imu/config/j13_ladrc_params.yaml'
    (out/'runtime_config.yaml').write_bytes(config.read_bytes())
    limit = 30 if run_id == 1 else 60
    metadata = dict(run_id=f'A1.8-R{run_id}', purpose='Natural TX failure capture; NOT qualification',
                    command=command(), preflight=before, max_duration_s=limit,
                    branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
                    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                    strace=False, ptrace=False, perf_record=False, timeout_injection=False,
                    recovery_stage='fresh_state_only', static_resume=False,
                    tx_write_timeout_s=.030, normal_rx_timeout_s=.005, normal_drain_budget_s=.004,
                    recovery_jump_limit_steps=80, warning_feedback_age_s=.2833,
                    hard_stop_feedback_age_s=1.0, current_limit_mA=1500,
                    application_write_telemetry=True, runtime_diagnostics=True,
                    driver_diagnostics_deferred=True, isolated_workload='driver_diagnostics',
                    heartbeat_period_s=.010, heartbeat_stall_threshold_ms=20,
                    start_monotonic_ns=time.monotonic_ns(), start_epoch_ns=time.time_ns())
    (out/'run_metadata_start.json').write_text(json.dumps(metadata,indent=2)+'\n')
    pointers = [Path('/tmp/j14_latest_run_dir'),Path('/tmp/j11_latest_run_dir')]
    previous = {p:p.read_bytes() if p.exists() else None for p in pointers}
    for p in pointers:
        p.write_text(str(out))
    observer = HostHeartbeatObserver(out/'host_heartbeat_stalls.csv')
    observer.start()
    proc = None
    reason = 'no_natural_timeout_within_window'
    try:
        with (out/'robot.log').open('w') as log:
            environment = dict(os.environ, PUS6_J14_RECOVERY_STAGE='fresh_state_only',
                               PUS6_J14_DEFER_DRIVER_DIAGNOSTICS='1')
            for key in ('PUS6_J14_TRACE_DIR','PUS6_J14_TRACE_TTY'):
                environment.pop(key,None)
            proc = subprocess.Popen(command(),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                                    start_new_session=True,env=environment)
            metadata['launch_start_ns'] = time.monotonic_ns()
            deadline = metadata['launch_start_ns'] + limit*1_000_000_000
            while proc.poll() is None and time.monotonic_ns() < deadline:
                content = (out/'robot.log').read_text()
                if 'UART TX failure:' in content:
                    if 'failure_seen_ns' not in metadata:
                        metadata['failure_seen_ns'] = time.monotonic_ns()
                    evidence = persisted_failure(out)
                    if evidence is not None:
                        metadata['first_persisted_failure'] = evidence
                        metadata['failure_persisted_seen_ns'] = time.monotonic_ns()
                        reason = 'first_persisted_natural_TX_timeout'
                        break
                    # Existing recovery remains motion-inhibited. Do not wait
                    # indefinitely for a broken evidence path or resume motion.
                    if time.monotonic_ns()-metadata['failure_seen_ns'] > 1_000_000_000:
                        reason = 'failure_evidence_persistence_not_confirmed'
                        break
                elif any(t in content for t in ('STOP latched','Failed to initialize J7 Servo backend',
                                                'FATAL safety violation','safety preflight failure')):
                    reason = 'driver_or_safety_failure'
                    break
                time.sleep(.010)
            if proc.poll() is not None:
                reason = 'launch_exited'
    finally:
        metadata['stop_requested_ns'] = time.monotonic_ns()
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                metadata['shutdown_escalation'] = 'SIGTERM'
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    metadata['shutdown_escalation'] = 'SIGKILL'
                    proc.kill()
                    proc.wait(timeout=2)
        observer.stop_and_flush()
        for p,value in previous.items():
            if value is None:
                p.unlink(missing_ok=True)
            else:
                p.write_bytes(value)
    content = (out/'robot.log').read_text()
    pauses = json_lines(out/'stop_pause_audit.jsonl')
    required = ['cycle_summary_full.csv','stall_diagnostic_timing.csv','transport_phase_timing.csv',
                'tx_queue_state.csv','tx_write_events.jsonl','rx_read_events.csv','host_heartbeat_stalls.csv',
                'driver_diagnostics_deferred.jsonl']
    complete = all((out/p).exists() for p in required)
    metadata.update(end_monotonic_ns=time.monotonic_ns(),end_epoch_ns=time.time_ns(),
                    stop_reason=reason,launch_returncode=proc.returncode if proc else None,
                    residual_processes=processes(),serial_after=serial_check(),pause_audits=pauses,
                    pause_ack_confirmed=bool(pauses) and pauses[-1].get('success',False)
                    and len(pauses[-1].get('records',[]))==6
                    and all(x['response_received'] and not x['timeout'] for x in pauses[-1]['records']),
                    shutdown_exception=any(t in content for t in ('Traceback','Error during J7 shutdown',
                                                                 'Error dumping','rcl_shutdown already called')),
                    runtime_anomaly=any(t in content for t in ('STOP latched','Failed to initialize J7 Servo backend',
                                                              'FATAL safety violation','safety preflight failure')),
                    telemetry_complete=complete)
    failures = [r for r in json_lines(out/'tx_write_events.jsonl') if r.get('write_error')=='SerialTimeoutException']
    metadata['captured_timeout_count'] = len(failures)
    if 'failure_seen_ns' in metadata and not failures:
        metadata['telemetry_complete'] = False
    metadata['r2_admissible'] = r2_admissible(metadata)
    (out/'run_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2),flush=True)
    if metadata['residual_processes'] or not metadata['serial_after']['free']:
        raise RuntimeError('Resource cleanup incomplete')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output-dir',required=True,type=Path)
    p.add_argument('--run-id',type=int,choices=(1,2),default=1)
    p.add_argument('--execute-hardware',action='store_true')
    args = p.parse_args()
    if not args.execute_hardware:
        p.error('--execute-hardware is required')
    run(args.output_dir.resolve(),args.run_id)
