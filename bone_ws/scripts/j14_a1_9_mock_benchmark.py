#!/usr/bin/env python3
"""Same UART core under identical parent CPU load: thread vs clean process."""
import json
from pathlib import Path
import select
import statistics
import threading
import time
from pus6_motor_driver.j14_process_uart import (ProcessClient, child_run, socket_pair,
                                               send_packet,receive_packets)
ROOT=Path('/home/lyapunov/bone_drill')
OUT=ROOT/'results/j14/j14_a1_9_software'


def stats(v):
    if not v:return None
    v=sorted(v)
    def q(p):
        i=(len(v)-1)*p;lo=int(i);return v[lo]+(v[min(lo+1,len(v)-1)]-v[lo])*(i-lo)
    return dict(n=len(v),P50=q(.5),P95=q(.95),P99=q(.99),Max=v[-1])


def load(stop):
    while not stop.is_set():
        until=time.perf_counter()+.001
        value=0
        while time.perf_counter()<until:value=(value*17+1)%1000003
        stop.wait(.004)


def main():
    records=[]
    for rep in range(3):
        for label in (['same_process','separate_process'] if rep%2==0 else ['separate_process','same_process']):
            out=OUT/f'benchmark_frozen_{rep}_{label}';out.mkdir(exist_ok=False)
            config=dict(epoch=time.monotonic_ns(),output_dir=str(out),hardware_mode='mock',
                port='/dev/DO_NOT_OPEN',baudrate=921600,servo_speed=50,maintenance_mode=True,
                reconnect_max_jump_steps=80,target_timeout_sec=.2)
            stop=threading.Event();loads=[threading.Thread(target=load,args=(stop,)) for _ in range(2)]
            for t in loads:t.start()
            c=None;t=None
            try:
                if label=='separate_process':
                    c=ProcessClient(config);s=c.sock
                else:
                    s,cs=socket_pair();t=threading.Thread(target=child_run,args=(cs,config));t.start()
                    until=time.monotonic()+3;ready=False
                    while time.monotonic()<until and not ready:
                        send_packet(s,dict(type='state',epoch=config['epoch'],mode='static',allowed=True,
                                           lease_ns=time.monotonic_ns(),command=None))
                        rows,_=receive_packets(s);ready=any(r.get('type')=='ready' for r in rows)
                        time.sleep(.003)
                    assert ready
                pc=time.process_time_ns();start=time.monotonic_ns();next_lease=0
                until=time.monotonic()+2
                while time.monotonic()<until:
                    now=time.monotonic_ns()
                    if now>=next_lease:
                        send_packet(s,dict(type='state',epoch=config['epoch'],mode='static',allowed=True,
                                           lease_ns=now,command=None));next_lease=now+100_000_000
                    rows,_=receive_packets(s)
                    if any(r.get('type')=='fault' for r in rows):raise RuntimeError('mock fault')
                    select.select([s],[],[],.005)
                parent_cpu=time.process_time_ns()-pc;duration=time.monotonic_ns()-start
                if c:c.close()
                else:
                    send_packet(s,dict(type='stop',epoch=config['epoch'],reason='benchmark_complete'))
                    t.join(timeout=3);assert not t.is_alive();s.close()
            finally:
                stop.set()
                for th in loads:th.join(timeout=1)
                if c and not c.closed:c.close()
            cycles=[json.loads(l) for l in (out/'child_cycles.jsonl').read_text().splitlines()]
            final=json.loads((out/'child_final.json').read_text());assert final['hardware_write_count']==0
            records.append(dict(rep=rep,condition=label,parent_process_cpu_ns=parent_cpu,
                observed_ns=duration,cycles=len(cycles),fault=final['fault'],
                distributions={k:stats([x[k] for x in cycles]) for k in (
                    'thread_cpu_ns','process_cpu_ns','voluntary_cs','involuntary_cs','wake_lateness_ns')},
                cycle_interval_ns=stats([b['start_ns']-a['start_ns'] for a,b in zip(cycles,cycles[1:])])))
    result=dict(hardware_write_count=0,method='3 alternating pairs x 2 seconds; identical ServoCore / MockServoBus / unchanged 50 Hz and inter-frame behavior; 2 parent Python threads each busy 1ms then sleep 4ms',
                limitations=['Mock measures software scheduling, not physical UART','Same-process parent CPU includes servo thread; separate-process parent CPU excludes UART child','No ROS executor load in this synthetic benchmark'],records=records)
    (OUT/'mock_ab_benchmark_frozen.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
