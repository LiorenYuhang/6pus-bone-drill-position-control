#!/usr/bin/env python3
"""J14-A1.1 software PTY characterization or one authorized Stage A diagnostic.

Production Serial.write, timeout, motion/fault policy and launch file are reused.
PTY is the default. Hardware requires an explicit flag and lasts at most 10 s.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import numpy as np
import serial

ROOT = Path('/home/lyapunov/bone_drill')


def stats(a):
    return dict(n=len(a), mean=float(np.mean(a)), max=float(max(a)),
                **{f'p{q}': float(np.percentile(a, q)) for q in [50, 95, 99, 99.9]})


def pty_characterization(out, trials):
    master, slave = pty.openpty()
    port = serial.Serial(os.ttyname(slave), write_timeout=.030, timeout=.005)
    os.set_blocking(master, False)
    rows = []
    def drain_test_peer():
        while True:
            try:
                if not os.read(master, 65536):
                    break
            except BlockingIOError:
                break
    try:
        # Ordinary 10-byte writes with a consuming test peer.
        started = time.monotonic()
        for _ in range(1000):
            port.write(b'x'*10)
            drain_test_peer()
        normal_elapsed = time.monotonic() - started
        # Backpressure: test peer deliberately does not read during each write.
        for seq in range(trials):
            before = port.out_waiting
            start, cpu = time.perf_counter_ns(), time.thread_time_ns()
            err = ''
            try:
                port.write(b'x'*2_000_000)
            except serial.SerialTimeoutException:
                err = 'SerialTimeoutException'
            wall = (time.perf_counter_ns() - start) / 1e6
            thread_cpu = (time.thread_time_ns() - cpu) / 1e6
            rows.append(dict(seq=seq, configured_timeout_ms=30, observed_return_wall_ms=wall,
                             overshoot_ms=wall-30, thread_cpu_ms=thread_cpu,
                             out_waiting_before=before, out_waiting_after=port.out_waiting, error=err))
            drain_test_peer()
    finally:
        port.close()
        os.close(master)
        os.close(slave)
    with (out/'pty_timeouts.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader(); writer.writerows(rows)
    result = dict(surface='local PTY; not CH340', trials=trials,
                  timeout_count=sum(r['error'] != '' for r in rows),
                  return_wall_ms=stats([r['observed_return_wall_ms'] for r in rows]),
                  overshoot_ms=stats([r['overshoot_ms'] for r in rows]),
                  normal_1000_writes_total_s=normal_elapsed,
                  hardware_write_count=0)
    (out/'pty_summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


def relevant_processes():
    names = {'motor_driver_node','h30_imu_node','kinematics_node','control_node','fk_node',
             'j13_ladrc_shadow_runner.py','j13_ladrc_staged_launcher.py'}
    result = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = proc.joinpath('cmdline').read_bytes().decode().split('\0')
            if any(Path(a).name in names for a in args if a):
                result.append(dict(pid=int(proc.name), args=args))
        except (OSError, UnicodeError):
            pass
    return result


def stage_a(out, duration, tracing):
    if duration <= 0 or duration > 10:
        raise ValueError('Diagnostic duration must be >0 and <=10 s')
    if relevant_processes():
        raise RuntimeError('Existing robot processes active; do not interfere')
    devices = ['/dev/ttyUSB0','/dev/ttyACM0','/dev/ttyACM1']
    busy = subprocess.run(['fuser',*devices], capture_output=True, text=True)
    if busy.returncode == 0:
        raise RuntimeError(f'Serial resource already in use: {busy.stdout}')
    # New evidence destination; preserve A0/A1 pointers and datasets.
    pointers = [Path('/tmp/j14_latest_run_dir'),Path('/tmp/j11_latest_run_dir')]
    previous = {p: p.read_text() if p.exists() else None for p in pointers}
    for p in pointers:
        p.write_text(str(out.resolve()))
    cmd = ['ros2','launch']
    if tracing:
        prefix = f'strace -ff -ttt -T -yy -s 64 -e trace=write,read,pselect6,poll,ppoll,ioctl -o {out.resolve()}/syscalls'
        cmd += ['--launch-prefix',prefix,'--launch-prefix-filter','motor_driver_node']
    cmd += ['pus6_bringup','pus6_system.launch.py',
            f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
            'hardware_mode:=active','maintenance_mode:=true','servo_scheduler_mode:=dedicated_thread',
            'telemetry_publish_mode:=async_latest','feedback_loss_policy:=age_based',
            'enable_runtime_diagnostics:=true']
    metadata = dict(command=cmd, max_duration_s=duration, trace=tracing,
                    purpose='Stage A startup diagnostic only; no runner ACTIVE; not qualification',
                    start_epoch_ns=time.time_ns(), start_perf_ns=time.perf_counter_ns())
    # Import only the existing independent observer; no staged launcher is invoked.
    import sys
    sys.path.insert(0, str(ROOT/'bone_ws/src/h30_imu/scripts'))
    from j13_ladrc_staged_launcher import HostHeartbeatObserver
    observer = HostHeartbeatObserver(out/'host_heartbeat_stalls.csv')
    observer.start()
    proc = None
    stop_reason = 'max_short_diagnostic_duration'
    with (out/'robot.log').open('w') as log:
        try:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            started = time.monotonic()
            while proc.poll() is None and time.monotonic()-started < duration:
                text = (out/'robot.log').read_text()
                if 'STOP latched' in text or 'Failed to initialize J7 Servo backend' in text:
                    stop_reason = 'driver_fault_or_startup_failure'
                    break
                time.sleep(.05)
        finally:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=2)
            observer.stop_and_flush()
            for p, value in previous.items():
                if value is None:
                    p.unlink(missing_ok=True)
                else:
                    p.write_text(value)
    metadata.update(end_epoch_ns=time.time_ns(), end_perf_ns=time.perf_counter_ns(),
                    stop_reason=stop_reason, launch_returncode=proc.returncode if proc else None,
                    residual_processes=relevant_processes())
    free = subprocess.run(['fuser',*devices],capture_output=True,text=True)
    metadata['serial_resource_owners_after'] = free.stdout.strip()
    metadata['serial_resources_free'] = free.returncode == 1
    (out/'run_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))
    if metadata['residual_processes'] or not metadata['serial_resources_free']:
        raise RuntimeError('Diagnostic cleanup incomplete; inspect owned process group')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--pty-trials',type=int,default=40)
    parser.add_argument('--execute-hardware',action='store_true')
    parser.add_argument('--max-seconds',type=float,default=10)
    parser.add_argument('--trace',action='store_true')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.execute_hardware:
        stage_a(args.output_dir,args.max_seconds,args.trace)
    else:
        pty_characterization(args.output_dir,args.pty_trials)


if __name__ == '__main__':
    main()
