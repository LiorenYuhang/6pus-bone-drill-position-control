#!/usr/bin/env python3
"""Offline joins of existing A1.7 application evidence; no hardware imports."""
import argparse
import csv
import json
from pathlib import Path
import statistics


def csv_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def json_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def event_matches_csv(event,row):
    for field,value in event.items():
        actual = row[field]
        if isinstance(value,int):
            if int(actual) != value:
                return False
        elif isinstance(value,float):
            if float(actual) != value:
                return False
        elif actual != str(value):
            return False
    return True


def overlaps(start,end,heartbeat):
    return [x for x in heartbeat if int(x['expected_ns']) < end and int(x['wake_ns']) > start]


def join_timeline(cycles,phases,writes,heartbeat,end_cycle,count=50):
    indexed = {int(x['cycle_seq']):x for x in phases}
    result = []
    for cycle in cycles:
        seq = int(cycle['cycle_seq'])
        if not end_cycle-count <= seq <= end_cycle:
            continue
        phase = indexed.get(seq,{})
        axis = [x for x in writes if int(x['cycle_seq'])==seq]
        result.append(dict(
            cycle_seq=seq,entry_ns=int(cycle['t_entry_ns']),exit_ns=int(cycle['t_exit_ns']),
            expected_wake_ns=int(cycle['expected_wake_ns']),
            callback_wall_ms=float(cycle['callback_total_ms']),
            inter_callback_idle_gap_ms=float(cycle['inter_callback_gap_ms']),
            wake_lateness_ms=float(cycle['wake_lateness_ms']),
            feedback_scheduler_gap_ms=float(cycle['scheduler_gap_ms']),
            feedback_age_ms=float(cycle['feedback_age_max_ms']),
            six_axis_tx_phase_ms=float(cycle['uart_tx_ms']),
            uart_drain_wall_ms=float(cycle['uart_drain_ms']),
            prior_drain_wall_ms=float(phase.get('prior_rx_drain_wall_ms',0)),
            feedback_drain_wall_ms=float(phase.get('motion_fb_drain_wall_ms',0)),
            max_read_wall_ms=float(cycle['rx_max_single_read_wall_ms']),
            entry_interval_process_cpu_ms=float(cycle['process_cpu_delta_ms']),
            entry_interval_servo_thread_cpu_ms=float(cycle['thread_cpu_delta_ms']),
            entry_interval_voluntary_cs=int(cycle['vol_ctx_switches_delta']),
            entry_interval_involuntary_cs=int(cycle['invol_ctx_switches_delta']),
            per_axis_write_metrics=axis,
            write_interval_thread_cpu_sum_ms=sum(float(x['serial_write_cpu_ms']) for x in axis),
            write_interval_process_cpu_sum_ms=sum(int(x['process_cpu_duration_ns'])/1e6 for x in axis),
            heartbeat_overlaps=overlaps(int(cycle['t_entry_ns']),int(cycle['t_exit_ns']),heartbeat)))
    return result


def summarize(rows,fields):
    return {field:dict(median=statistics.median(float(x[field]) for x in rows),
                       max=max(float(x[field]) for x in rows)) for field in fields}


def analyze(run):
    cycles = csv_rows(run/'stall_diagnostic_timing.csv')
    phases = csv_rows(run/'transport_phase_timing.csv')
    writes = csv_rows(run/'tx_queue_state.csv')
    heartbeat = csv_rows(run/'host_heartbeat_stalls.csv')
    events = json_rows(run/'tx_write_events.jsonl')
    failures = [x for x in events if x['write_error']=='SerialTimeoutException']
    for event in failures:
        paired = [x for x in writes if int(x['cycle_seq'])==event['cycle_seq']
                  and int(x['write_seq'])==event['write_seq'] and int(x['motor_id'])==event['motor_id']]
        assert len(paired)==1, 'Failed write identity must pair exactly once'
        assert event_matches_csv(event,paired[0]), 'Event and CSV telemetry differ'
    end = failures[0]['cycle_seq'] if failures else int(cycles[-1]['cycle_seq'])
    timeline = join_timeline(cycles,phases,writes,heartbeat,end)
    (run/'failure_timeline_50_cycles.json').write_text(json.dumps(timeline,indent=2)+'\n')
    scalar_fields = [k for k in timeline[0] if k not in ('per_axis_write_metrics','heartbeat_overlaps')]
    with (run/'failure_timeline_50_cycles.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=scalar_fields)
        writer.writeheader()
        writer.writerows({k:x[k] for k in scalar_fields} for x in timeline)
    with (run/'failure_timeline_per_axis.csv').open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(writes[0]))
        writer.writeheader()
        writer.writerows(x for row in timeline for x in row['per_axis_write_metrics'])
    pre = [x for x in timeline if x['cycle_seq']<end]
    pre_writes = [x for row in pre for x in row['per_axis_write_metrics']]
    good = [x for x in writes if not x['write_error']]
    result = dict(outcome='CAPTURED' if failures else 'NOT REPRODUCED',
                  total_cycles=len(cycles),failed_writes=failures,
                  failed_write_csv_identity_verified=True,
                  preceding_cycles=len(pre),preceding_cycle_range=[pre[0]['cycle_seq'],pre[-1]['cycle_seq']] if pre else [],
                  pre50_stats=summarize(pre,['callback_wall_ms','wake_lateness_ms','inter_callback_idle_gap_ms',
                      'six_axis_tx_phase_ms','uart_drain_wall_ms','max_read_wall_ms','entry_interval_process_cpu_ms',
                      'entry_interval_servo_thread_cpu_ms']) if pre else {},
                  pre50_successful_write_stats=summarize(pre_writes,['serial_write_wall_ms']) if pre_writes else {},
                  successful_write_median_ms=statistics.median(float(x['serial_write_wall_ms']) for x in good),
                  max_successful_write=max(good,key=lambda x:float(x['serial_write_wall_ms'])),
                  max_wake_cycle=max(cycles,key=lambda x:float(x['wake_lateness_ms'])),
                  max_idle_gap_cycle=max(cycles,key=lambda x:float(x['inter_callback_gap_ms'])),
                  max_read_cycle=max(cycles,key=lambda x:float(x['rx_max_single_read_wall_ms'])),
                  max_drain_cycle=max(cycles,key=lambda x:float(x['uart_drain_ms'])),
                  heartbeat_stalls=heartbeat,
                  failure_write_heartbeat_overlaps=overlaps(failures[0]['serial_write_start_ns'],
                      failures[0]['serial_write_end_ns'],heartbeat) if failures else [],
                  legacy_cpu_caveat='entry_interval CPU/CS span adjacent entries, include idle, and do not measure failed write or exact callback CPU. Failed-write metrics are paired separately.',
                  failure_cycle_caveat='Fresh-state-only recovery completes inside failure callback. Callback wall/read aggregates may include pause/fresh operations; missing post-TX drain/feedback age zeros are not observations of normal completed phases.')
    (run/'application_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('max_wake_cycle','max_idle_gap_cycle','max_read_cycle','max_drain_cycle')},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    analyze(parser.parse_args().run.resolve())
