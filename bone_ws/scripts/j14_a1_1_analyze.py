#!/usr/bin/env python3
"""Offline queue/syscall and historical timing-scope analysis; no serial access."""
import argparse
import csv
from decimal import Decimal
import json
from pathlib import Path
import re

import numpy as np


def read(path):
    return list(csv.DictReader(path.open()))


def stats(values):
    a=np.array(values,dtype=float)
    return dict(n=len(a), mean=float(a.mean()), max=float(a.max()),
                **{f'p{q}':float(np.percentile(a,q)) for q in [50,95,99,99.9]}) if len(a) else dict(n=0)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-dir',required=True,type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args(); p=args.run_dir; out=args.output_dir; out.mkdir(parents=True,exist_ok=True)
    metadata=json.loads((p/'run_metadata.json').read_text())
    queue=read(p/'tx_queue_state.csv'); timing=read(p/'cycle_timing.csv'); full=read(p/'cycle_summary_full.csv')
    tids={r['native_thread_id'] for r in timing}
    events=[]
    for tid in tids:
        for line in (p/f'syscalls.{tid}').read_text().splitlines():
            match=re.match(r'([0-9.]+) (write|pselect6|read|ioctl)\(',line)
            duration=re.search(r'<([0-9.]+)>$',line)
            if not match or not duration or '/dev/ttyUSB0' not in line:
                continue
            start_ns=int(Decimal(match[1])*1_000_000_000)
            duration_ns=int(Decimal(duration[1])*1_000_000_000)
            name=match[2]
            if name=='pselect6':
                name='tx_select' if re.search(r'\], \[\d+</dev/ttyUSB0',line) else 'rx_select'
            events.append(dict(start_ns=start_ns,end_ns=start_ns+duration_ns,
                               duration_ms=duration_ns/1e6,name=name,line=line,tid=tid))
    offset=metadata['start_epoch_ns']-metadata['start_perf_ns']
    correlated=[]
    for row in queue:
        start=int(row['serial_write_start_ns'])+offset;end=int(row['serial_write_end_ns'])+offset
        calls=[r for r in events if r['name'] in ['write','tx_select'] and start-5000<=r['start_ns']<=end+5000]
        entry=dict(cycle_seq=row['cycle_seq'],motor_id=row['motor_id'],wall_ms=float(row['serial_write_wall_ms']),
                   write_syscall_ms=sum(r['duration_ms'] for r in calls if r['name']=='write'),
                   tx_select_syscall_ms=sum(r['duration_ms'] for r in calls if r['name']=='tx_select'))
        if calls:
            entry.update(pre_first_syscall_ms=(min(r['start_ns'] for r in calls)-start)/1e6,
                         post_last_syscall_ms=(end-max(r['end_ns'] for r in calls))/1e6,
                         syscall_count=len(calls))
        correlated.append(entry)
    with (out/'write_syscall_alignment.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=correlated[0]);writer.writeheader();writer.writerows(correlated)
    maxcalls={k: max((r for r in events if r['name']==k),key=lambda r:r['duration_ms'],default=None)
              for k in ['write','tx_select','read','rx_select','ioctl']}
    summary=dict(run=metadata,retained_queue_writes=len(queue),full_cycle_count=len(full),
                 retained_cycle_range=[timing[0]['cycle_seq'],timing[-1]['cycle_seq']],
                 timeout_count_retained=sum(bool(r['write_error']) for r in queue),
                 out_waiting_before_counts={v:sum(r['out_waiting_before']==v for r in queue) for v in set(r['out_waiting_before'] for r in queue)},
                 out_waiting_after_counts={v:sum(r['out_waiting_after']==v for r in queue) for v in set(r['out_waiting_after'] for r in queue)},
                 queue_errors=sum(bool(r['queue_error_before'] or r['queue_error_after']) for r in queue),
                 write_and_observer_ms={k:stats([float(r[k]) for r in queue]) for k in
                                        ['serial_write_wall_ms','serial_write_cpu_ms','queue_read_before_ms','queue_read_after_ms']},
                 full_cycle_ms={k:stats([float(r[k]) for r in full]) for k in
                                ['uart_tx_ms','uart_drain_ms','cb_total_ms','inter_gap_ms','feedback_age_max_ms']},
                 syscall_maxima=maxcalls,
                 top_write_alignment=sorted(correlated,key=lambda r:r['wall_ms'],reverse=True)[:10],
                 clock_offset_drift_us=((metadata['end_epoch_ns']-metadata['end_perf_ns'])-offset)/1000,
                 limitations=['Tracing perturbs normal timing','No physical timeout reproduced','Queue snapshots do not cover every instant or physical UART/device queues','Only retained last 300 cycles have full Python queue rows'],
                 hardware_write_count=0)
    (out/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    old=p.parent/'j14_a1_r1_static_10s'
    c=read(old/'cycle_timing.csv'); m={r['cycle_seq']:r for r in c}
    timeline=[]
    for seq in [14,15,16,89,90,91,198,199]:
        r=m[str(seq)];prev=m.get(str(seq-1))
        record=dict(cycle_seq=seq,entry_ns=int(r['t_entry_ns']),exit_ns=int(r['t_exit_ns']),
                    callback_wall_ms=float(r['callback_total_ms']),inter_gap_ms=float(r['inter_callback_gap_ms']),
                    recorded_thread_cpu_entry_to_entry_ms=float(r['thread_cpu_delta_ms']),
                    recorded_process_cpu_entry_to_entry_ms=float(r['process_cpu_delta_ms']),
                    voluntary_entry_to_entry=int(r['vol_ctx_switches_delta']),involuntary_entry_to_entry=int(r['invol_ctx_switches_delta']))
        if prev:record['cpu_sampling_wall_entry_to_entry_ms']=(int(r['t_entry_ns'])-int(prev['t_entry_ns']))/1e6
        timeline.append(record)
    (out/'historical_event_timeline.json').write_text(json.dumps(timeline,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['retained_queue_writes','full_cycle_count','timeout_count_retained','out_waiting_before_counts','out_waiting_after_counts','top_write_alignment']},indent=2))
    print('hardware_write_count = 0 (offline analysis)')


if __name__=='__main__':
    main()
