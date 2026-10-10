#!/usr/bin/env python3
"""Offline A1.9 artifact analysis; no serial or ROS access."""
import csv
import json
from pathlib import Path
ROOT=Path('/home/lyapunov/bone_drill');OUT=ROOT/'results/j14/j14_a1_9_software'


def lines(path):
    return [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []


def csvrows(path):
    return list(csv.DictReader(path.open())) if path.exists() else []


def stats(v):
    v=sorted(x for x in v if x is not None)
    if not v:return None
    def q(p):
        i=(len(v)-1)*p;lo=int(i);return v[lo]+(v[min(lo+1,len(v)-1)]-v[lo])*(i-lo)
    return dict(n=len(v),P50=q(.5),P95=q(.95),P99=q(.99),Max=v[-1])


def analyze(out):
    meta=json.loads((out/'run_metadata.json').read_text())
    cycles=lines(out/'child_cycles.jsonl');writes=lines(out/'child_write_metrics.jsonl')
    metrics=[r['serial_metric'] for r in writes if not r['error'] and r['serial_metric']]
    failed=[r for r in writes if r['error']=='SerialTimeoutException']
    ipc=lines(out/'parent_ipc.jsonl')
    recv=[r for r in ipc if r['event']=='parent_receive' and r.get('type')=='feedback']
    cmd_by_seq={r['seq']:r for r in recv if r.get('command_receive_ns') and r.get('seq') is not None}
    reads=csvrows(out/'rx_read_events.csv');phases=csvrows(out/'transport_phase_timing.csv')
    full=csvrows(out/'cycle_summary_full.csv')
    snapshot=json.loads((out/'runtime_process_boundary.json').read_text())
    result=dict(timeout_count=len(failed),cycles=len(cycles),successful_writes=len(metrics),all_motion_write_records=len(writes),
        servo_start_ns=meta['child_final']['servo_start_ns'],
        servo_window_s=(meta['child_final']['motion_stop_ns']-meta['child_final']['servo_start_ns'])/1e9,
        first_cycle_to_last_exit_s=(cycles[-1]['end_ns']-cycles[0]['start_ns'])/1e9,
        launch_to_stop_s=(meta['stop_requested_ns']-meta['launch_start_ns'])/1e9,
        total_with_cleanup_s=(meta['end_monotonic_ns']-meta['start_monotonic_ns'])/1e9,
        first_timeout=failed[0] if failed else None,
        first_failure_s=(failed[0]['serial_metric']['timestamp_end_ns']-cycles[0]['start_ns'])/1e9 if failed else None,
        serial_write_ms={name:stats([r[field]/1e6 for r in metrics]) for name,field in
            [('wall','wall_duration_ns'),('thread_cpu','cpu_duration_ns'),('child_process_cpu','process_cpu_duration_ns')]},
        wrapper_wall_ms=stats([r['wrapper_wall_ns']/1e6 for r in writes if not r['error']]),
        write_voluntary_CS=stats([r['voluntary_ctx_delta'] for r in metrics]),
        write_involuntary_CS=stats([r['involuntary_ctx_delta'] for r in metrics]),
        child_cycle_ms={name:stats([(r[b]-r[a])/1e6 if a else r[b]/1e6 for r in cycles]) for name,a,b in
            [('wall','start_ns','end_ns'),('thread_cpu',None,'thread_cpu_ns'),('process_cpu',None,'process_cpu_ns'),('wake_lateness',None,'wake_lateness_ns')]},
        child_cycle_voluntary_CS=stats([r['voluntary_cs'] for r in cycles]),
        child_cycle_involuntary_CS=stats([r['involuntary_cs'] for r in cycles]),
        observed_cycle_interval_ms=stats([(b['start_ns']-a['start_ns'])/1e6 for a,b in zip(cycles,cycles[1:])]),
        cycle_interval_minus_20_ms=stats([(b['start_ns']-a['start_ns'])/1e6-20 for a,b in zip(cycles,cycles[1:])]),
        parent_cpu_sample_span_s=(ipc[-1]['ns']-ipc[0]['ns'])/1e9,
        parent_cpu_sample_delta_s=(ipc[-1]['process_cpu_ns']-ipc[0]['process_cpu_ns'])/1e9,
        child_cycle_cpu_sum_s=sum(r['process_cpu_ns'] for r in cycles)/1e9,
        feedback_received=len(recv),distinct_command_sequences=len(cmd_by_seq),
        parent_to_child_command_age_ms=stats([(r['command_receive_ns']-r['command_publish_ns'])/1e6 for r in cmd_by_seq.values()]),
        feedback_IPC_age_ms=stats([(r['ns']-r['child_ready_ns'])/1e6 for r in recv]),
        feedback_gap_ms=stats([(b['ns']-a['ns'])/1e6 for a,b in zip(recv,recv[1:])]),
        rx_recorded_read_wall_ms=stats([float(r['read_wall_ns'])/1e6 for r in reads]),
        rx_recorded_read_cpu_ms=stats([float(r['read_thread_cpu_ns'])/1e6 for r in reads]),
        rx_drain_last_300_cycles_ms=stats([float(r['motion_fb_drain_wall_ms']) for r in phases]),
        rx_cycle_max_read_ms=stats([float(r['rx_max_single_read_wall_ms']) for r in full]),
        rx_cycle_max_read_thread_cpu_ms=stats([float(r['rx_max_single_read_thread_cpu_ms']) for r in full]),
        rx_cycle_max_read_process_cpu_ms=stats([float(r['rx_max_single_read_process_cpu_ms']) for r in full]),
        rx_drain_all_cycles_ms=stats([float(r['uart_drain_ms']) for r in full]),
        rx_read_call_count_full=sum(int(r['rx_read_call_count']) for r in full),
        rx_bytes_full=sum(int(r['rx_bytes']) for r in full),
        feedback_max_age_by_cycle_ms=stats([float(r['feedback_age_max_ms']) for r in full]),
        heartbeat_stalls=len(csvrows(out/'host_heartbeat_stalls.csv')),
        child_warnings=lines(out/'child_warnings.jsonl'),
        task_counts={k:len(snapshot[k]['tasks']) for k in ('parent','child')},
        exclusive_motor_uart_owner=snapshot['motor_uart_fuser']['stdout'].strip(),
        all_out_waiting_before_zero=all(r['queue_before']['out_waiting']==0 for r in metrics),
        pause_ack_confirmed=meta['pause_ack_confirmed'],serial_free=meta['serial_after']['free'],
        residual_processes=meta['residual_processes'],shutdown_exception=meta['shutdown_exception'],
        child_returncode=meta['child_returncode'],telemetry_complete=meta['telemetry_complete'])
    if failed:
        t0=failed[0]['serial_metric']['timestamp_start_ns'];t1=failed[0]['serial_metric']['timestamp_end_ns']
        before=[r for r in ipc if r['ns']<=t0];after=[r for r in ipc if r['ns']>=t1]
        result['parent_cpu_fault_bracket']=dict(start=before[-1] if before else None,end=after[0] if after else None,
            exact_serial_interval_cpu='NOT RECORDED')
    result['scope_notes']=['Strict serial wall/thread CPU and slightly wider paired process CPU/CS are unchanged adapter measurements',
        'All successful write metrics and child cycle scalars cover full trial; legacy detailed drain ring covers final 300 cycles only',
        'RX read export is selected slow/representative records, not every read distribution',
        'Parent CPU cumulative sample delta is parent IPC sampling span, not a strict serial-write interval',
        'No reliable failure probability from one/two short samples; completion is right-censored first-failure time',
        'No heartbeat >20ms does not mean every host scheduling delay was zero']
    (out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    baseline=json.loads((ROOT/'results/j14/j14_a1_8_r1_static_no_trace/a1_7_a1_8_comparison.json').read_text())
    result=dict(baseline=baseline,new_runs={})
    for num in (1,2):
        out=ROOT/f'results/j14/j14_a1_9_r{num}_static_separate_process'
        if (out/'run_metadata.json').exists():
            result['new_runs'][f'A1.9-R{num}']=analyze(out)
    configs=[(ROOT/'results/j14/j14_a1_7_r1_static_no_trace/runtime_config.yaml').read_bytes(),
             (ROOT/'results/j14/j14_a1_8_r1_static_no_trace/runtime_config.yaml').read_bytes()]
    for num in (1,2):
        p=ROOT/f'results/j14/j14_a1_9_r{num}_static_separate_process/runtime_config.yaml'
        if p.exists():configs.append(p.read_bytes())
    result['all_config_yaml_bytes_equal']=all(x==configs[0] for x in configs)
    (OUT/'a1_7_a1_8_a1_9_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,r in result['new_runs'].items():
        print(name,json.dumps(r,indent=2))


if __name__=='__main__':main()
