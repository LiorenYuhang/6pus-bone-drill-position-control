#!/usr/bin/env python3
"""Offline J14 evidence analysis; never imports ROS or opens serial devices."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def read(path):
    return list(csv.DictReader(path.open())) if path.exists() else []


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'sample_count': 0}
    return dict(sample_count=len(a), mean=float(a.mean()),
                **{name: float(np.percentile(a, q)) for name, q in
                   [('p50', 50), ('p95', 95), ('p99', 99), ('p99_9', 99.9)]},
                max=float(a.max()),
                **{f'ge_{t}_ms_count': int((a >= t).sum()) for t in [5, 10, 20, 50, 100]})


def analyze(run):
    transport = read(run / 'transport_phase_timing.csv')
    timing = {r['cycle_seq']: r for r in read(run / 'cycle_timing.csv')}
    events = read(run / 'stall_events_all.csv')
    summary = read(run / 'cycle_summary_full.csv')
    telemetry = read(next(iter(run.glob('*telemetry_powered.csv')), run / 'missing.csv'))
    top = []
    writes = {str(m): [] for m in range(1, 7)}
    present = set()
    for r in transport:
        c = timing.get(r['cycle_seq'], {})
        for m in range(1, 7):
            # Aborted transaction's unsent axes have req_bytes == 0; not zero-latency writes.
            if not int(r[f'm{m}_req_bytes']):
                continue
            present.add((r['cycle_seq'], m))
            wall = float(r[f'm{m}_write_wall_ms'])
            writes[str(m)].append(wall)
            top.append(dict(run=run.name, cycle_seq=r['cycle_seq'], motor_id=m,
                            write_wall_ms=wall, write_cpu_ms=float(r[f'm{m}_write_cpu_ms']),
                            uart_tx_ms=c.get('uart_tx_ms'), callback_total_ms=c.get('callback_total_ms'),
                            rx_drain_ms=r['motion_fb_drain_wall_ms'], prior_rx_drain_ms=r['prior_rx_drain_wall_ms'],
                            source='transport window'))
    # Preserve known extremes outside ring without manufacturing six-axis samples or per-write CPU.
    for c in events:
        m = int(c['tx_slowest_motor_id'])
        if m and (c['cycle_seq'], m) not in present:
            top.append(dict(run=run.name, cycle_seq=c['cycle_seq'], motor_id=m,
                            write_wall_ms=float(c['tx_slowest_motor_ms']), write_cpu_ms=None,
                            uart_tx_ms=c['uart_tx_ms'], callback_total_ms=c['callback_total_ms'],
                            rx_drain_ms=c['tx_feedback_drain_ms'], prior_rx_drain_ms=c['tx_prior_drain_ms'],
                            source='full-lifecycle event; per-write CPU unavailable'))
    full = {key: stats([float(r[key]) for r in summary
                        if key != 'feedback_age_max_ms' or not r['fault_reason']]) for key in
            ['uart_tx_ms', 'uart_drain_ms', 'cb_total_ms', 'inter_gap_ms', 'wake_lateness_ms', 'feedback_age_max_ms']}
    fields = ['driver_feedback_age_ms', 'timed_feedback_age_at_tick_ms', 'delta_t_runner_queue_ms']
    active = [r for r in telemetry if r['state'] == 'ACTIVE']
    runner = {key: stats([float(r[key]) for r in active]) for key in fields if active and key in active[0]}
    runner['scope'] = 'ACTIVE runner ticks only; excludes missing-feedback startup sentinel'
    runner['active_ticks'] = len(active)
    timeouts = [r for r in transport if 'Timeout' in r['error_class']]
    result = dict(run=run.name, transport_window_cycles=len(transport),
                  cycle_seq_range=[transport[0]['cycle_seq'], transport[-1]['cycle_seq']] if transport else [],
                  full_lifecycle_cycles=len(summary), per_motor={k: stats(v) for k, v in writes.items()},
                  all_motors_combined=stats(sum(writes.values(), [])), full_lifecycle=full,
                  runner=runner, heartbeat_stalls=len(read(run / 'host_heartbeat_stalls.csv')),
                  timeout_count=len(timeouts), timeout_motor_ids=[r['error_motor_id'] for r in timeouts],
                  callback_cpu_window={k: stats([float(r[k]) for r in timing.values()]) for k in
                                       ['thread_cpu_delta_ms', 'process_cpu_delta_ms']},
                  summaries=[json.loads(p.read_text()) for p in run.glob('*summary_powered.json')])
    return result, top, writes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results, top, combined = [], [], []
    for run in args.runs:
        result, events, writes = analyze(run)
        results.append(result)
        top.extend(events)
        combined.extend(sum(writes.values(), []))
    out = dict(runs=results, combined_retained_windows=stats(combined),
               note='Per-axis distributions cover retained transport windows only; full lifecycle metrics use Tier 1. Top 20 adds outside-window slowest-axis events; unavailable CPU is blank.')
    (args.output_dir / 'analysis.json').write_text(json.dumps(out, indent=2) + '\n')
    with (args.output_dir / 'write_distribution.csv').open('w') as f:
        records = [dict(run=r['run'], motor_id=m, **s) for r in results for m, s in
                   list(r['per_motor'].items()) + [('combined', r['all_motors_combined'])]]
        w = csv.DictWriter(f, fieldnames=records[0].keys())
        w.writeheader()
        w.writerows(records)
    with (args.output_dir / 'top20_write_long_tail.csv').open('w') as f:
        ordered = sorted(top, key=lambda r: r['write_wall_ms'], reverse=True)[:20]
        if ordered:
            w = csv.DictWriter(f, fieldnames=ordered[0].keys())
            w.writeheader()
            w.writerows(ordered)
    print(args.output_dir / 'analysis.json')
    print('hardware_write_count = 0 (offline analysis)')


if __name__ == '__main__':
    main()
