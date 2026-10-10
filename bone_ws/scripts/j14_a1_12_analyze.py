#!/usr/bin/env python3
"""Offline A1.12 artifact analysis and Dedicated Feedback Bridge latency attribution."""
import csv
import json
from pathlib import Path
import sys

ROOT = Path('/home/lyapunov/bone_drill')


def lines(path: Path) -> list[dict]:
    if not path.exists():
        return []
    records = []
    for line in path.read_text().splitlines():
        if line.strip():
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


def csvrows(path: Path) -> list[dict]:
    return list(csv.DictReader(path.open())) if path.exists() else []


def stats(v: list[float | int | None]) -> dict | None:
    valid = sorted(x for x in v if x is not None)
    if not valid:
        return None
    n = len(valid)

    def q(p: float) -> float:
        idx = (n - 1) * p
        lo = int(idx)
        return valid[lo] + (valid[min(lo + 1, n - 1)] - valid[lo]) * (idx - lo)

    return {
        'n': n,
        'P50': round(q(0.50), 3),
        'P95': round(q(0.95), 3),
        'P99': round(q(0.99), 3),
        'Max': round(valid[-1], 3),
        'Min': round(valid[0], 3),
    }


def analyze(out: Path) -> dict:
    meta = json.loads((out / 'run_metadata.json').read_text())
    cycles = lines(out / 'child_cycles.jsonl')
    writes = lines(out / 'child_write_metrics.jsonl')
    metrics = [r['serial_metric'] for r in writes if not r.get('error') and r.get('serial_metric')]
    failed = [r for r in writes if r.get('error') == 'SerialTimeoutException']
    parent_ipc = lines(out / 'parent_ipc.jsonl')
    bridge_pipeline = lines(out / 'bridge_feedback_pipeline.jsonl')
    bridge_life = lines(out / 'bridge_lifecycle.jsonl')
    reads = csvrows(out / 'rx_read_events.csv')
    heartbeats = csvrows(out / 'host_heartbeat_stalls.csv')
    snapshot = json.loads((out / 'runtime_process_boundary.json').read_text()) if (out / 'runtime_process_boundary.json').exists() else {}
    sock_audit = json.loads((out / 'socket_buffer_audit.json').read_text()) if (out / 'socket_buffer_audit.json').exists() else {}

    # Bridge timing metrics (in ms)
    bridge_ipc_age_ms = [r['ipc_age_ms'] for r in bridge_pipeline if 'ipc_age_ms' in r]
    bridge_gaps_ms = [r['feedback_gap_ms'] for r in bridge_pipeline if 'feedback_gap_ms' in r and r['feedback_gap_ms'] > 0]
    message_build_wall_ms = [
        (r['t_message_build_done_ns'] - r['t_bridge_recv_ns']) / 1e6
        for r in bridge_pipeline
        if r.get('t_message_build_done_ns') and r.get('t_bridge_recv_ns')
    ]
    pub_js_wall_ms = [
        (r['t_pub_js_exit_ns'] - r['t_pub_js_enter_ns']) / 1e6
        for r in bridge_pipeline
        if r.get('t_pub_js_exit_ns') and r.get('t_pub_js_enter_ns')
    ]
    pub_tf_wall_ms = [
        (r['t_pub_tf_exit_ns'] - r['t_pub_tf_enter_ns']) / 1e6
        for r in bridge_pipeline
        if r.get('t_pub_tf_exit_ns') and r.get('t_pub_tf_enter_ns')
    ]
    pub_tc_wall_ms = [
        (r['t_pub_tc_exit_ns'] - r['t_pub_tc_enter_ns']) / 1e6
        for r in bridge_pipeline
        if r.get('t_pub_tc_exit_ns') and r.get('t_pub_tc_enter_ns')
    ]
    pub_total_wall_ms = [
        (r['t_pub_tc_exit_ns'] - r['t_pub_js_enter_ns']) / 1e6
        for r in bridge_pipeline
        if r.get('t_pub_tc_exit_ns') and r.get('t_pub_js_enter_ns')
    ]
    bridge_thread_cpu_ms = [
        r['bridge_thread_cpu_ns'] / 1e6
        for r in bridge_pipeline
        if r.get('bridge_thread_cpu_ns') is not None
    ]

    # Drops from bridge lifecycle
    bridge_stop_ev = next((e for e in bridge_life if e.get('event') == 'bridge_stopped'), {})
    drop_count = bridge_stop_ev.get('drop_count', 0)
    total_received = bridge_stop_ev.get('total_received', len(bridge_pipeline))

    # Child UART write metrics
    write_wall_ms = [r['wrapper_wall_ns'] / 1e6 for r in writes if r.get('wrapper_wall_ns') is not None]
    raw_write_ms = [m['wall_duration_ns'] / 1e6 for m in metrics if m.get('wall_duration_ns') is not None]

    # Child loop scheduling
    wake_lateness_ms = [c['wake_lateness_ns'] / 1e6 for c in cycles]
    intervals_ms = [
        (b['start_ns'] - a['start_ns']) / 1e6
        for a, b in zip(cycles, cycles[1:])
    ]

    # Heartbeat stalls
    stalls = [float(r['stall_duration_ms']) for r in heartbeats if 'stall_duration_ms' in r]

    # Compile result summary
    summary = {
        'run_id': meta.get('run_id'),
        'purpose': meta.get('purpose'),
        'duration_s': meta.get('max_servo_duration_s'),
        'stop_reason': meta.get('stop_reason'),
        'pause_ack_confirmed': meta.get('pause_ack_confirmed'),
        'captured_timeout_count': meta.get('captured_timeout_count'),
        'child_cycles': len(cycles),
        'bridge_received_cycles': total_received,
        'bridge_drop_count': drop_count,
        'socket_buffers': sock_audit,
        'bridge_ipc_age_ms': stats(bridge_ipc_age_ms),
        'bridge_feedback_gap_ms': stats(bridge_gaps_ms),
        'message_build_wall_ms': stats(message_build_wall_ms),
        'pub_js_wall_ms': stats(pub_js_wall_ms),
        'pub_tf_wall_ms': stats(pub_tf_wall_ms),
        'pub_tc_wall_ms': stats(pub_tc_wall_ms),
        'pub_total_wall_ms': stats(pub_total_wall_ms),
        'bridge_thread_cpu_ms': stats(bridge_thread_cpu_ms),
        'child_write_wall_ms': stats(write_wall_ms),
        'child_raw_write_ms': stats(raw_write_ms),
        'child_wake_lateness_ms': stats(wake_lateness_ms),
        'child_cycle_interval_ms': stats(intervals_ms),
        'host_heartbeat_stalls': stats(stalls),
        'r2_admissible': meta.get('r2_admissible'),
    }

    out_file = out / 'a1_12_analysis_summary.json'
    out_file.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 j14_a1_12_analyze.py <output_directory>")
        sys.exit(1)
    analyze(Path(sys.argv[1]))
