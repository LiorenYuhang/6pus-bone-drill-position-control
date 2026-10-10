#!/usr/bin/env python3
"""Offline same-read attribution matrix. Labels are evidence support, not gates."""
import argparse
import csv
import json
from pathlib import Path


def read(path):
    return list(csv.DictReader(path.open())) if path.exists() else []


def classify(event,heartbeat):
    wall=int(event['read_wall_ns']);thread=int(event['read_thread_cpu_ns']);process=int(event['read_process_cpu_ns'])
    vol=int(event['read_vol_ctx_delta']);invol=int(event['read_invol_ctx_delta'])
    lo=int(event['t_read_enter_ns']);hi=int(event['t_read_exit_ns'])
    overlapping=[r for r in heartbeat if int(r['expected_ns'])<=hi and int(r['wake_ns'])>=lo]
    low=thread/wall<.2
    tags=[]
    if low and vol>0:tags.append('R1 blocking/sleep/syscall-wait support')
    if low and invol>0:tags.append('R2 preemption support')
    if low and process/wall>=.5 and not overlapping:tags.append('R3 process-local workload/contention support; GIL UNKNOWN')
    if thread/wall>=.7:tags.append('R4 CPU execution support; parser is outside this wrapper')
    if overlapping:tags.append('R5 heartbeat-overlap support')
    overhead=int(event['snapshot_overhead_ns'])/wall
    if overhead>.1:tags.append('snapshot boundary overhead significant: attribution limited')
    return dict(support_classes=tags or ['insufficient classification evidence'],
                thread_cpu_fraction=thread/wall,process_cpu_fraction=process/wall,
                snapshot_overhead_fraction=overhead,overlapping_heartbeat_events=len(overlapping),
                wall_ms=wall/1e6,thread_cpu_ms=thread/1e6,process_cpu_ms=process/1e6,
                evidence_level='INFERRED for root mechanism; measured same-call values CONFIRMED')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output-dir',required=True,type=Path);args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    reports=[];events_all=[]
    for run in args.runs:
        events=read(run/'rx_read_events.csv');heartbeat=read(run/'host_heartbeat_stalls.csv')
        full=read(run/'cycle_summary_full.csv');cycle={r['cycle_seq']:r for r in read(run/'cycle_timing.csv')}
        for event in events:
            result=dict(run=run.name,**event,**classify(event,heartbeat))
            row=cycle.get(event['cycle_seq'],{})
            result['cycle_timing']={k:row.get(k) for k in ['callback_total_ms','uart_tx_ms','uart_drain_ms','inter_callback_gap_ms','scheduler_gap_ms','wake_lateness_ms']}
            events_all.append(result)
        metadata=json.loads((run/'run_metadata.json').read_text())
        idle=max(cycle.values(),key=lambda r:float(r['inter_callback_gap_ms']),default={})
        largest_callback=max(cycle.values(),key=lambda r:float(r['callback_total_ms']),default={})
        idle_interval=(int(idle['t_entry_ns'])-round(float(idle['inter_callback_gap_ms'])*1e6),int(idle['t_entry_ns'])) if idle else None
        reports.append(dict(run=run.name,metadata=metadata,cycles=len(full),
                            read_calls=sum(int(r['rx_read_call_count']) for r in full),
                            read_bytes=sum(int(r['rx_bytes']) for r in full),
                            events_gt_ms={str(t):sum(int(r['read_wall_ns'])>t*1e6 for r in events) for t in [5,10,20,50,100]},
                            cycle_events_gt_ms={str(t):sum(int(r['read_wall_ns'])>t*1e6 and r['scope']=='cycle' for r in events) for t in [5,10,20,50,100]},
                            max_single_read_wall_ms=max((float(r['rx_max_single_read_wall_ms']) for r in full),default=None),
                            largest_idle_cycle={k:idle.get(k) for k in ['cycle_seq','t_entry_ns','inter_callback_gap_ms','scheduler_gap_ms','wake_lateness_ms','rx_max_single_read_wall_ms']},
                            largest_idle_interval_ns_approx=idle_interval,
                            slow_reads_overlapping_largest_idle=sum(int(e['t_read_enter_ns'])<=idle_interval[1] and int(e['t_read_exit_ns'])>=idle_interval[0] for e in events) if idle_interval else 0,
                            largest_callback_cycle={k:largest_callback.get(k) for k in ['cycle_seq','callback_total_ms','uart_drain_ms','rx_max_single_read_wall_ms']},
                            heartbeat_stalls=len(heartbeat),
                            tx_timeout_rows=[r for r in read(run/'transport_phase_timing.csv') if r['error_class']=='SerialTimeoutException']))
    (args.output_dir/'analysis.json').write_text(json.dumps(dict(runs=reports,slow_events=events_all,
       rubric='R1/R2 CS>0 with thread/wall<0.2; R3 process/wall>=0.5 and no overlapping recorded heartbeat; R4 thread/wall>=0.7; R5 heartbeat overlap. Non-exclusive diagnostic support, never root proof or safety gates.',
       offline_analysis_hardware_write_count=0),indent=2)+'\n')
    with (args.output_dir/'all_slow_read_events.csv').open('w') as f:
        if events_all:
            # Stable event fields, excluding nested classification objects.
            cols=['run','cycle_seq','scope','read_seq','native_thread_id','t_read_enter_ns','t_read_exit_ns','read_wall_ns','serial_call_wall_ns','snapshot_overhead_ns','read_thread_cpu_ns','read_process_cpu_ns','read_vol_ctx_delta','read_invol_ctx_delta','in_waiting_before','requested_read_bytes','returned_read_bytes','configured_read_timeout_s','read_error']
            writer=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');writer.writeheader();writer.writerows(events_all)
        else:f.write('run,cycle_seq,read_seq,read_wall_ns\n')
    print(json.dumps(reports,indent=2));print('hardware_write_count = 0 (offline analysis)')


if __name__=='__main__':main()
