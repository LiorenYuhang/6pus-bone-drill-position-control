#!/usr/bin/env python3
"""Offline A1.11 artifact analysis and feedback pipeline latency attribution."""
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
    ipc = lines(out / 'parent_ipc.jsonl')
    recv = [r for r in ipc if r.get('event') == 'parent_receive' and r.get('type') == 'feedback']
    pipeline = lines(out / 'parent_feedback_pipeline.jsonl')
    reads = csvrows(out / 'rx_read_events.csv')
    phases = csvrows(out / 'transport_phase_timing.csv')
    full = csvrows(out / 'cycle_summary_full.csv')
    snapshot = json.loads((out / 'runtime_process_boundary.json').read_text()) if (out / 'runtime_process_boundary.json').exists() else {}

    # Separate published vs superseded pipeline records
    published = [r for r in pipeline if r.get('status') == 'published']
    superseded = [r for r in pipeline if r.get('status') == 'superseded']

    # Stage durations in ms
    ipc_transfer_age_ms = [
        (r['t_parent_recv_ns'] - r['t_child_ready_ns']) / 1e6
        for r in pipeline
        if r.get('t_parent_recv_ns') and r.get('t_child_ready_ns')
    ]
    snapshot_update_delay_ms = [
        (r['t_snapshot_update_ns'] - r['t_parent_recv_ns']) / 1e6
        for r in pipeline
        if r.get('t_snapshot_update_ns') and r.get('t_parent_recv_ns')
    ]
    telemetry_wake_delay_ms = [
        (r['t_telemetry_wake_ns'] - r['t_snapshot_update_ns']) / 1e6
        for r in published
        if r.get('t_telemetry_wake_ns') and r.get('t_snapshot_update_ns')
    ]
    message_build_wall_ms = [
        (r['t_message_build_done_ns'] - r['t_telemetry_wake_ns']) / 1e6
        for r in published
        if r.get('t_message_build_done_ns') and r.get('t_telemetry_wake_ns')
    ]
    pub_js_wall_ms = [
        (r['t_pub_js_exit_ns'] - r['t_pub_js_enter_ns']) / 1e6
        for r in published
        if r.get('t_pub_js_exit_ns') and r.get('t_pub_js_enter_ns')
    ]
    pub_tf_wall_ms = [
        (r['t_pub_tf_exit_ns'] - r['t_pub_tf_enter_ns']) / 1e6
        for r in published
        if r.get('t_pub_tf_exit_ns') and r.get('t_pub_tf_enter_ns')
    ]
    pub_tc_wall_ms = [
        (r['t_pub_tc_exit_ns'] - r['t_pub_tc_enter_ns']) / 1e6
        for r in published
        if r.get('t_pub_tc_exit_ns') and r.get('t_pub_tc_enter_ns')
    ]
    pub_all_wall_ms = [
        (r['t_pub_tc_exit_ns'] - r['t_pub_js_enter_ns']) / 1e6
        for r in published
        if r.get('t_pub_tc_exit_ns') and r.get('t_pub_js_enter_ns')
    ]
    e2e_feedback_age_ms = [
        (r['t_pub_tc_exit_ns'] - r['t_child_ready_ns']) / 1e6
        for r in published
        if r.get('t_pub_tc_exit_ns') and r.get('t_child_ready_ns')
    ]

    # CPU metrics in ms
    telemetry_thread_cpu_ms = [
        r['telemetry_thread_cpu_ns'] / 1e6
        for r in published
        if r.get('telemetry_thread_cpu_ns') is not None
    ]
    message_build_thread_cpu_ms = [
        r['message_build_thread_cpu_ns'] / 1e6
        for r in published
        if r.get('message_build_thread_cpu_ns') is not None
    ]
    pub_js_thread_cpu_ms = [
        r['pub_js_thread_cpu_ns'] / 1e6
        for r in published
        if r.get('pub_js_thread_cpu_ns') is not None
    ]
    pub_tf_thread_cpu_ms = [
        r['pub_tf_thread_cpu_ns'] / 1e6
        for r in published
        if r.get('pub_tf_thread_cpu_ns') is not None
    ]
    pub_tc_thread_cpu_ms = [
        r['pub_tc_thread_cpu_ns'] / 1e6
        for r in published
        if r.get('pub_tc_thread_cpu_ns') is not None
    ]

    # Parent IPC receive thread CPU
    parent_recv_thread_cpu_ms = [
        r['thread_cpu_ns'] / 1e6
        for r in recv
        if r.get('thread_cpu_ns') is not None
    ]

    # Feedback intervals / gaps
    feedback_gap_ms = [(b['ns'] - a['ns']) / 1e6 for a, b in zip(recv, recv[1:])]

    # Worst feedback age event
    worst_pub_rec = max(published, key=lambda r: (r['t_pub_tc_exit_ns'] - r['t_child_ready_ns'])) if published else None
    worst_ipc_rec = max(pipeline, key=lambda r: (r['t_parent_recv_ns'] - r['t_child_ready_ns'])) if pipeline else None
    worst_gap_idx = max(range(len(feedback_gap_ms)), key=lambda i: feedback_gap_ms[i]) if feedback_gap_ms else None

    result = dict(
        timeout_count=len(failed),
        cycles=len(cycles),
        successful_writes=len(metrics),
        all_motion_write_records=len(writes),
        servo_start_ns=meta['child_final'].get('servo_start_ns', 0),
        servo_window_s=(meta['child_final'].get('motion_stop_ns', 0) - meta['child_final'].get('servo_start_ns', 0)) / 1e9,
        serial_write_ms={
            name: stats([r[field] / 1e6 for r in metrics])
            for name, field in [
                ('wall', 'wall_duration_ns'),
                ('thread_cpu', 'cpu_duration_ns'),
                ('child_process_cpu', 'process_cpu_duration_ns'),
            ]
        },
        child_cycle_ms={
            name: stats([(r[b] - r[a]) / 1e6 if a else r[b] / 1e6 for r in cycles])
            for name, a, b in [
                ('wall', 'start_ns', 'end_ns'),
                ('thread_cpu', None, 'thread_cpu_ns'),
                ('process_cpu', None, 'process_cpu_ns'),
                ('wake_lateness', None, 'wake_lateness_ns'),
            ]
        },
        feedback_received=len(recv),
        feedback_gap_ms=stats(feedback_gap_ms),
        # Pipeline stages
        pipeline_total_records=len(pipeline),
        published_count=len(published),
        superseded_count=len(superseded),
        stage1_ipc_transfer_age_ms=stats(ipc_transfer_age_ms),
        stage2_snapshot_update_delay_ms=stats(snapshot_update_delay_ms),
        stage3_telemetry_wake_delay_ms=stats(telemetry_wake_delay_ms),
        stage4_message_build_wall_ms=stats(message_build_wall_ms),
        stage5a_pub_js_wall_ms=stats(pub_js_wall_ms),
        stage5b_pub_tf_wall_ms=stats(pub_tf_wall_ms),
        stage5c_pub_tc_wall_ms=stats(pub_tc_wall_ms),
        stage5_all_pub_wall_ms=stats(pub_all_wall_ms),
        e2e_feedback_age_ms=stats(e2e_feedback_age_ms),
        # CPU statistics
        stage4_message_build_thread_cpu_ms=stats(message_build_thread_cpu_ms),
        stage5a_pub_js_thread_cpu_ms=stats(pub_js_thread_cpu_ms),
        stage5b_pub_tf_thread_cpu_ms=stats(pub_tf_thread_cpu_ms),
        stage5c_pub_tc_thread_cpu_ms=stats(pub_tc_thread_cpu_ms),
        telemetry_worker_thread_cpu_ms=stats(telemetry_thread_cpu_ms),
        parent_recv_thread_cpu_ms=stats(parent_recv_thread_cpu_ms),
        # Heartbeat stalls
        heartbeat_stalls=len(csvrows(out / 'host_heartbeat_stalls.csv')),
        pause_ack_confirmed=meta.get('pause_ack_confirmed', False),
        serial_free=meta.get('serial_after', {}).get('free', False),
        residual_processes=meta.get('residual_processes', []),
        child_returncode=meta.get('child_returncode'),
        telemetry_complete=meta.get('telemetry_complete', False),
        worst_published_event=worst_pub_rec,
        worst_ipc_event=worst_ipc_rec,
        worst_gap_ms=feedback_gap_ms[worst_gap_idx] if worst_gap_idx is not None else None,
    )

    (out / 'a1_11_analysis.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    if len(sys.argv) > 1:
        out_dir = Path(sys.argv[1])
    else:
        out_dir = ROOT / 'results/j14/j14_a1_11_r1_static_separate_process'
    res = analyze(out_dir)
    print(json.dumps(res, indent=2))
