#!/usr/bin/env python3
"""J14-A1.2 observation benchmark or authorized 10s Stage A RX diagnostic."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ROOT=Path('/home/lyapunov/bone_drill')
sys.path.insert(0,str(ROOT/'bone_ws/src/pus6_motor_driver'))
from j14_a1_1_diagnostics import relevant_processes


def benchmark(out):
    from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
    from pus6_motor_driver.stall_diagnostic_recorder import J11StallDiagnosticRecorder
    class FakeSerial:
        in_waiting=20
        timeout=.005
        hardware_write_count=0
        def read(self,size):
            return b'x'*size
    rows=[]
    for enabled in [False,True,False,True]:
        rec=J11StallDiagnosticRecorder(enabled=enabled,output_dir=str(out/'benchmark_recorder'))
        rec.start_cycle(0,time.perf_counter_ns())
        adapter=J7UartAdapter(FakeSerial(),read_observer=rec.record_single_read if enabled else None)
        start=time.perf_counter_ns()
        for _ in range(10000):
            assert adapter._read_chunk()==b'x'*20
        elapsed=(time.perf_counter_ns()-start)/1e6
        rec.finish_cycle()
        # Benchmark events are not a robot experiment; no physical hardware.
        rows.append(dict(enabled=enabled,reads=10000,total_ms=elapsed,us_per_read=elapsed/10))
    off=np.mean([r['us_per_read'] for r in rows if not r['enabled']]);on=np.mean([r['us_per_read'] for r in rows if r['enabled']])
    result=dict(surface='fake immediate read + production observer',runs=rows,
                added_us_per_read=float(on-off),estimated_ms_per_cycle_at_two_reads=float((on-off)*2/1000),
                hardware_write_count=0)
    (out/'overhead.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


def stage_a(out,tracing):
    if relevant_processes():
        raise RuntimeError('Existing robot processes; do not interfere')
    devices=['/dev/ttyUSB0','/dev/ttyACM0','/dev/ttyACM1']
    busy=subprocess.run(['fuser',*devices],capture_output=True,text=True)
    if busy.returncode==0:
        raise RuntimeError('Serial resource in use: '+busy.stdout)
    if any(out.glob('cycle*.csv')) or (out/'run_metadata.json').exists():
        raise RuntimeError('Refusing to overwrite diagnostic evidence')
    pointers=[Path('/tmp/j14_latest_run_dir'),Path('/tmp/j11_latest_run_dir')]
    previous={p:p.read_text() if p.exists() else None for p in pointers}
    for p in pointers:p.write_text(str(out.resolve()))
    cmd=['ros2','launch']
    if tracing:
        cmd+=['--launch-prefix',f'strace -ff -ttt -T -yy -s 64 -e trace=read,pselect6,poll,ppoll,ioctl -o {out.resolve()}/syscalls','--launch-prefix-filter','motor_driver_node']
    cmd+=['pus6_bringup','pus6_system.launch.py',
          f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
          'hardware_mode:=active','maintenance_mode:=true','servo_scheduler_mode:=dedicated_thread',
          'telemetry_publish_mode:=async_latest','feedback_loss_policy:=age_based','enable_runtime_diagnostics:=true']
    sys.path.insert(0,str(ROOT/'bone_ws/src/h30_imu/scripts'))
    from j13_ladrc_staged_launcher import HostHeartbeatObserver
    heartbeat=HostHeartbeatObserver(out/'host_heartbeat_stalls.csv');heartbeat.start()
    metadata=dict(command=cmd,trace=tracing,max_duration_s=10,
                  purpose='J14-A1.2-RX-DIAG Stage A only; no runner; not qualification',
                  start_epoch_ns=time.time_ns(),start_monotonic_ns=time.monotonic_ns())
    proc=None;reason='max_10s_diagnostic_duration'
    with (out/'robot.log').open('w') as log:
        try:
            proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            started=time.monotonic()
            while proc.poll() is None and time.monotonic()-started<10:
                content=(out/'robot.log').read_text()
                if 'STOP latched' in content or 'Failed to initialize J7 Servo backend' in content:
                    reason='driver_fault_or_startup_failure';break
                if 'J14_RX_READ_EVENT ' in content:
                    reason='representative_rx_read_gt20ms_captured';break
                time.sleep(.010)
        finally:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2)
            heartbeat.stop_and_flush()
            for p,value in previous.items():
                if value is None:p.unlink(missing_ok=True)
                else:p.write_text(value)
    content=(out/'robot.log').read_text()
    metadata.update(stop_reason=reason,end_epoch_ns=time.time_ns(),end_monotonic_ns=time.monotonic_ns(),
                    launch_returncode=proc.returncode if proc else None,residual_processes=relevant_processes(),
                    hardware_safety_fault_detected=any(token in content for token in
                      ['overcurrent','scheduler_deadline_exceeded','feedback_age_exceeded','safety_preflight_failure','Servo mode command fault=']))
    free=subprocess.run(['fuser',*devices],capture_output=True,text=True)
    metadata.update(serial_resources_free=free.returncode==1,serial_owners_after=free.stdout.strip())
    (out/'run_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n');print(json.dumps(metadata,indent=2))
    if metadata['residual_processes'] or not metadata['serial_resources_free']:
        raise RuntimeError('Cleanup incomplete; inspect owned process group')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--execute-hardware',action='store_true')
    parser.add_argument('--trace',action='store_true')
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.execute_hardware:stage_a(args.output_dir,args.trace)
    else:benchmark(args.output_dir)


if __name__=='__main__':main()
