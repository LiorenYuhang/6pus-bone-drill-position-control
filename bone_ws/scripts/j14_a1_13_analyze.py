#!/usr/bin/env python3
"""Offline A1.13 artifact analysis and Three-Process Coordinated Startup Handshake validation."""
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
    parent_life = lines(out / 'parent_lifecycle.jsonl')
    bridge_pipeline = lines(out / 'bridge_feedback_pipeline.jsonl')
    bridge_life = lines(out / 'bridge_lifecycle.jsonl')
    child_ready = lines(out / 'child_ready.jsonl')
    reads = csvrows(out / 'rx_read_events.csv')
    heartbeats = csvrows(out / 'host_heartbeat_stalls.csv')
    snapshot = json.loads((out / 'runtime_process_boundary.json').read_text()) if (out / 'runtime_process_boundary.json').exists() else {}
    sock_audit = json.loads((out / 'socket_buffer_audit.json').read_text()) if (out / 'socket_buffer_audit.json').exists() else {}

    # Startup Handshake Timeline extraction
    hs_start_packet = next((r for r in child_ready if r.get('type') == 'servo_start'), {})
    hs_ready_packet = next((r for r in child_ready if r.get('type') == 'child_ready'), {})
    hs_parent_event = next((r for r in parent_life if r.get('event') == 'startup_handshake_complete'), {})
    bridge_ready_event = next((r for r in bridge_life if r.get('event') == 'bridge_ready'), {})
    bridge_start_event = next((r for r in bridge_life if r.get('event') == 'servo_start_received'), {})

    t_child_ready_ns = hs_start_packet.get('t_child_ready_ns') or hs_ready_packet.get('t_child_ready_ns')
    t_bridge_ready_ns = hs_start_packet.get('t_bridge_ready_ns') or bridge_ready_event.get('ns')
    t_parent_ready_ns = hs_start_packet.get('t_parent_ready_ns') or hs_parent_event.get('t_parent_ready_ns')
    servo_start_ns = hs_start_packet.get('servo_start_ns') or bridge_start_event.get('servo_start_ns')
    cycle_0_start_ns = cycles[0]['start_ns'] if cycles else None

    # Readiness order
    ready_times = []
    if t_child_ready_ns: ready_times.append(('child', t_child_ready_ns))
    if t_bridge_ready_ns: ready_times.append(('bridge', t_bridge_ready_ns))
    if t_parent_ready_ns: ready_times.append(('parent', t_parent_ready_ns))
    ready_times.sort(key=lambda x: x[1])
    ready_sequence = [role for role, _ in ready_times]

    handshake_timeline = {
        'ready_sequence': ready_sequence,
        't_child_ready_ns': t_child_ready_ns,
        't_bridge_ready_ns': t_bridge_ready_ns,
        't_parent_ready_ns': t_parent_ready_ns,
        'servo_start_ns': servo_start_ns,
        'cycle_0_start_ns': cycle_0_start_ns,
        'child_wait_for_servo_start_ms': round((servo_start_ns - t_child_ready_ns) / 1e6, 3) if (servo_start_ns and t_child_ready_ns) else None,
        'bridge_wait_for_servo_start_ms': round((servo_start_ns - t_bridge_ready_ns) / 1e6, 3) if (servo_start_ns and t_bridge_ready_ns) else None,
        'parent_wait_for_servo_start_ms': round((servo_start_ns - t_parent_ready_ns) / 1e6, 3) if (servo_start_ns and t_parent_ready_ns) else None,
        'servo_start_to_cycle_0_ms': round((cycle_0_start_ns - servo_start_ns) / 1e6, 3) if (cycle_0_start_ns and servo_start_ns) else None,
    }

    # Drops and cycle sequence audit
    bridge_stop_ev = next((e for e in bridge_life if e.get('event') == 'bridge_stopped'), {})
    drop_count = bridge_stop_ev.get('drop_count', 0)
    total_received = bridge_stop_ev.get('total_received', len(bridge_pipeline))
    first_cycle_seq = bridge_pipeline[0].get('cycle_seq', 0) if bridge_pipeline else 0
    cycle_seqs = [r.get('cycle_seq') for r in bridge_pipeline if 'cycle_seq' in r]
    steady_drops = sum(b - a - 1 for a, b in zip(cycle_seqs, cycle_seqs[1:]) if b > a + 1)

    # Bridge timing metrics (in ms)
    bridge_ipc_age_ms = [r['ipc_age_ms'] for r in bridge_pipeline if 'ipc_age_ms' in r]
    cycle_0_ipc_age_ms = bridge_ipc_age_ms[0] if bridge_ipc_age_ms else None
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
        'first_cycle_seq': first_cycle_seq,
        'startup_drop_count': first_cycle_seq,
        'steady_drop_count': steady_drops,
        'total_drop_count': drop_count,
        'handshake_timeline': handshake_timeline,
        'cycle_0_ipc_age_ms': cycle_0_ipc_age_ms,
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

    out_file = out / 'a1_13_analysis_summary.json'
    out_file.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: j14_a1_13_analyze.py <output_directory>", file=sys.stderr)
        sys.exit(1)
    analyze(Path(sys.argv[1]).resolve())
