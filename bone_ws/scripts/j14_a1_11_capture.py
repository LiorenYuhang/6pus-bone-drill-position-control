#!/usr/bin/env python3
"""Authorized A1.11 static trial with parent feedback pipeline instrumentation."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path('/home/lyapunov/bone_drill')
sys.path.insert(0, str(ROOT / 'bone_ws/scripts'))
from j14_real_failure_sidecar import processes, serial_check
from j14_a1_8_capture import command, json_lines
sys.path.insert(0, str(ROOT / 'bone_ws/src/h30_imu/scripts'))
from j13_ladrc_staged_launcher import HostHeartbeatObserver


def r2_admissible(meta):
    return (
        meta['stop_reason'] == 'no_natural_timeout_within_window'
        and meta['captured_timeout_count'] == 0
        and meta['child_final'].get('fault') == 'window_complete'
        and meta['child_final'].get('closed')
        and meta['child_returncode'] == 0
        and meta['child_final'].get('motion_stop_ns', 0) - meta['child_final'].get('servo_start_ns', 0) >= 30_000_000_000
        and not meta.get('shutdown_escalation')
        and meta['telemetry_complete']
        and meta['pause_ack_confirmed']
        and not meta['residual_processes']
        and meta['serial_after']['free']
    )


def run(out: Path, run_id: int):
    if out.exists():
        raise RuntimeError('Refusing to overwrite evidence')
    if run_id == 2:
        previous = out.parent / 'j14_a1_11_r1_static_separate_process' / 'run_metadata.json'
        if not previous.exists() or not r2_admissible(json.loads(previous.read_text())):
            raise RuntimeError('R2 requires full clean R1')

    before = dict(processes=processes(), serial=serial_check())
    if before['processes'] or not before['serial']['free']:
        raise RuntimeError(f'Resource conflict: {before}')

    out.mkdir(parents=True)
    config = ROOT / 'bone_ws/src/h30_imu/config/j13_ladrc_params.yaml'
    (out / 'runtime_config.yaml').write_bytes(config.read_bytes())
    (out / 'git_before.txt').write_bytes(subprocess.check_output(['git', 'status', '--short', '--branch'], cwd=ROOT))

    limit = 30 if run_id == 1 else 60
    meta = dict(
        run_id=f'A1.11-R{run_id}',
        purpose='Parent feedback pipeline minimal instrumentation static trial',
        max_servo_duration_s=limit,
        command=command(),
        preflight=before,
        branch=subprocess.check_output(['git', 'branch', '--show-current'], cwd=ROOT, text=True).strip(),
        commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        tx_write_timeout_s=0.030,
        normal_rx_timeout_s=0.005,
        normal_drain_budget_s=0.004,
        baud=921600,
        cycle_rate_hz=50,
        recovery_jump_limit_steps=80,
        warning_feedback_age_s=0.2833,
        hard_stop_feedback_age_s=1.0,
        current_limit_mA=1500,
        driver_diagnostics_deferred=True,
        separate_uart_process=True,
        feedback_pipeline_instrumentation=True,
        full_recovery=False,
        strace=False,
        timeout_injection=False,
        heartbeat_period_s=0.010,
        heartbeat_stall_threshold_ms=20,
        start_monotonic_ns=time.monotonic_ns(),
        start_epoch_ns=time.time_ns(),
    )
    (out / 'run_metadata_start.json').write_text(json.dumps(meta, indent=2) + '\n')

    pointers = [Path('/tmp/j14_latest_run_dir'), Path('/tmp/j11_latest_run_dir')]
    prior = {p: p.read_bytes() if p.exists() else None for p in pointers}
    for p in pointers:
        p.write_text(str(out))

    observer = HostHeartbeatObserver(out / 'host_heartbeat_stalls.csv')
    observer.start()

    proc = None
    reason = 'harness_deadline'
    try:
        env = dict(
            os.environ,
            PUS6_J14_SEPARATE_UART_PROCESS='1',
            PUS6_J14_DEFER_DRIVER_DIAGNOSTICS='1',
            PUS6_J14_FEEDBACK_PIPELINE_INSTRUMENTATION='1',
            PUS6_J14_PROCESS_WINDOW_S=str(limit),
        )
        for key in ('PUS6_J14_RECOVERY_STAGE', 'PUS6_J14_TRACE_DIR', 'PUS6_J14_TRACE_TTY'):
            env.pop(key, None)

        with (out / 'robot.log').open('w') as log:
            proc = subprocess.Popen(command(), cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
            meta['launch_start_ns'] = time.monotonic_ns()
            watchdog = time.monotonic() + limit + 5

            while proc.poll() is None and time.monotonic() < watchdog:
                final = out / 'child_final.json'
                if final.exists():
                    child = json.loads(final.read_text())
                    reason = 'no_natural_timeout_within_window' if child.get('fault') == 'window_complete' else 'child_local_fault'
                    break

                ready_path = out / 'child_ready.jsonl'
                if ready_path.exists() and 'child_ready' not in meta:
                    ready = json_lines(ready_path)[0]
                    meta['child_ready'] = ready
                    snapshot = {}
                    for role, pid in [('parent', ready['parent_pid']), ('child', ready['pid'])]:
                        base = Path('/proc') / str(pid)
                        try:
                            snapshot[role] = dict(
                                pid=pid,
                                cmdline=(base / 'cmdline').read_bytes().replace(b'\0', b' ').decode(),
                                status=(base / 'status').read_text(),
                                tasks={p.name: (p / 'comm').read_text().strip() for p in (base / 'task').iterdir()},
                            )
                        except OSError as exc:
                            snapshot[role] = dict(error=repr(exc))
                    snapshot['motor_uart_fuser'] = subprocess.run(['fuser', '/dev/ttyUSB0'], capture_output=True, text=True).__dict__.copy()
                    (out / 'runtime_process_boundary.json').write_text(json.dumps(snapshot, indent=2) + '\n')

                text = (out / 'robot.log').read_text()
                if any(x in text for x in ('Failed to initialize J7 Servo backend', 'STOP latched')):
                    meta.setdefault('fault_seen_ns', time.monotonic_ns())
                    if time.monotonic_ns() - meta['fault_seen_ns'] > 1_000_000_000:
                        reason = 'driver_fault_evidence_incomplete'
                        break
                time.sleep(0.005)

            if proc.poll() is not None:
                reason = 'launch_exited'

    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                meta['shutdown_escalation'] = 'SIGTERM'
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except OSError:
                    proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except OSError:
                        proc.kill()
                    proc.wait(timeout=2)

        # Allow ROS 2 nodes a brief settling time to exit
        time.sleep(1.0)
        for residual in processes():
            try:
                os.kill(residual['pid'], signal.SIGINT)
            except OSError:
                pass
        time.sleep(0.5)

        observer.stop_and_flush()
        for p, value in prior.items():
            if value is None:
                p.unlink(missing_ok=True)
            else:
                p.write_bytes(value)

    robot = (out / 'robot.log').read_text()
    pauses = json_lines(out / 'stop_pause_audit.jsonl')
    child = json.loads((out / 'child_final.json').read_text()) if (out / 'child_final.json').exists() else {}
    parent_life = json_lines(out / 'parent_lifecycle.jsonl')
    joined = [r for r in parent_life if r['event'] == 'child_joined']

    required = [
        'child_cycles.jsonl',
        'child_write_metrics.jsonl',
        'child_ready.jsonl',
        'child_final.json',
        'child_lifecycle.jsonl',
        'cycle_summary_full.csv',
        'tx_queue_state.csv',
        'rx_read_events.csv',
        'parent_ipc.jsonl',
        'parent_lifecycle.jsonl',
        'stop_pause_audit.jsonl',
        'host_heartbeat_stalls.csv',
        'parent_feedback_pipeline.jsonl',
    ]

    writes = json_lines(out / 'child_write_metrics.jsonl')
    timeouts = [r for r in writes if r.get('error') == 'SerialTimeoutException']
    cycles = json_lines(out / 'child_cycles.jsonl')
    complete = all((out / name).exists() for name in required)

    meta.update(
        end_monotonic_ns=time.monotonic_ns(),
        end_epoch_ns=time.time_ns(),
        stop_reason=reason,
        launch_returncode=proc.returncode if proc else None,
        child_final=child,
        child_returncode=joined[-1]['child_returncode'] if joined else None,
        pause_ack_confirmed=bool(pauses) and pauses[-1].get('success', False) and len(pauses[-1]['records']) == 6
        and all(r['response_received'] and not r['timeout'] for r in pauses[-1]['records']),
        captured_timeout_count=len(timeouts),
        cycles=len(cycles),
        first_timeout=timeouts[0] if timeouts else None,
        shutdown_exception=any(x in robot for x in ('Traceback', 'rcl_shutdown already called', 'Error during', 'Error dumping')),
        telemetry_complete=complete,
        residual_processes=processes(),
        serial_after=serial_check(),
    )
    meta['r2_admissible'] = r2_admissible(meta)
    (out / 'run_metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(json.dumps(meta, indent=2), flush=True)

    if meta['residual_processes'] or not meta['serial_after']['free']:
        raise RuntimeError('Cleanup incomplete')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--run-id', type=int, choices=(1, 2), default=1)
    p.add_argument('--execute-hardware', action='store_true')
    a = p.parse_args()
    if not a.execute_hardware:
        p.error('--execute-hardware is required')
    run(a.output_dir.resolve(), a.run_id)
