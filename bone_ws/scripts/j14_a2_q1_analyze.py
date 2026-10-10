#!/usr/bin/env python3
"""Offline Q1 qualification analysis and reporting."""
import csv
import json
from pathlib import Path
import sys

ROOT = Path('/home/lyapunov/bone_drill')


def rows(path: Path) -> list[dict]:
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


def stats(values: list[float | int]) -> dict | None:
    v = sorted(float(x) for x in values if x is not None)
    if not v:
        return None
    n = len(v)

    def q(p: float) -> float:
        x = (n - 1) * p
        i = int(x)
        return round(v[i] + (v[min(i + 1, n - 1)] - v[i]) * (x - i), 4)

    return {
        'n': n,
        'P50': q(0.50),
        'P95': q(0.95),
        'P99': q(0.99),
        'P99_9': q(0.999),
        'Max': round(v[-1], 4),
        'Min': round(v[0], 4),
    }


def analyze(out: Path) -> dict:
    meta = json.loads((out / 'run_metadata.json').read_text())
    bridge = rows(out / 'bridge_feedback_pipeline.jsonl')
    cycles = rows(out / 'child_cycles.jsonl')
    writes = rows(out / 'child_write_metrics.jsonl')
    healthy = [r for r in writes if not r.get('error')]
    pipeline = rows(out / 'bridge_lifecycle.jsonl')
    parent = rows(out / 'parent_lifecycle.jsonl')
    heartbeats = csvrows(out / 'host_heartbeat_stalls.csv')

    # Metrics computation
    raw_writes_ms = [r['serial_metric']['wall_duration_ns'] / 1e6 for r in healthy if r.get('serial_metric')]
    wrapper_writes_ms = [r['wrapper_wall_ns'] / 1e6 for r in healthy if r.get('wrapper_wall_ns') is not None]
    ipc_ages_ms = [r['ipc_age_ms'] for r in bridge if 'ipc_age_ms' in r]
    feedback_gaps_ms = [r['feedback_gap_ms'] for r in bridge[1:] if 'feedback_gap_ms' in r and r['feedback_gap_ms'] > 0]
    wake_latenesses_ms = [c['wake_lateness_ns'] / 1e6 for c in cycles if 'wake_lateness_ns' in c]
    cycle_intervals_ms = [(b['start_ns'] - a['start_ns']) / 1e6 for a, b in zip(cycles, cycles[1:])]
    bridge_cpu_ms = [r['bridge_thread_cpu_ns'] / 1e6 for r in bridge if r.get('bridge_thread_cpu_ns') is not None]

    # Stalls and warnings
    hb_stalls = [float(r['stall_duration_ms']) for r in heartbeats if 'stall_duration_ms' in r]
    warning_count = sum(1 for g in feedback_gaps_ms if g > 283.3)
    hard_stop_count = sum(1 for g in feedback_gaps_ms if g > 1000.0)

    expected = meta.get('expected_cycles', int(meta['duration_s'] * 50))
    actual_child = meta.get('cycles', len(cycles))
    actual_bridge = meta.get('bridge_cycles', len(bridge))

    pass_hard = bool(
        meta.get('stop_reason') == 'window_complete'
        and abs(actual_child - expected) <= 5
        and actual_child == actual_bridge
        and meta.get('tx_timeout_count', 0) == 0
        and meta.get('hardware_fault_count', 0) == 0
        and meta.get('packet_drop_count', 0) == 0
        and meta.get('first_cycle_seq') == 0
        and meta.get('duplicate_cycles', 0) == 0
        and meta.get('pause_ack_confirmed') is True
        and meta.get('child_returncode') == 0
        and meta.get('bridge_returncode') == 0
        and meta.get('serial_after', {}).get('free') is True
        and not meta.get('residual_processes')
        and meta.get('handshake_valid') is True
        and meta.get('production_path_confirmed') is True
        and meta.get('unique_publishers_confirmed') is True
        and not meta.get('shutdown_escalation')
    )

    summary = {
        'run_id': meta.get('run_id'),
        'duration_s': meta.get('duration_s'),
        'expected_cycles': expected,
        'child_cycles': actual_child,
        'bridge_cycles': actual_bridge,
        'motion_writes': len(writes),
        'tx_timeout_count': meta.get('tx_timeout_count', 0),
        'hardware_fault_count': meta.get('hardware_fault_count', 0),
        'startup_drop_count': meta.get('first_cycle_seq'),
        'total_missing_cycles': meta.get('packet_drop_count', 0),
        'duplicate_cycles': meta.get('duplicate_cycles', 0),
        'handshake_valid': meta.get('handshake_valid'),
        'production_path_confirmed': meta.get('production_path_confirmed'),
        'unique_publishers_confirmed': meta.get('unique_publishers_confirmed'),
        'pause_ack_confirmed': meta.get('pause_ack_confirmed'),
        'child_returncode': meta.get('child_returncode'),
        'bridge_returncode': meta.get('bridge_returncode'),
        'serial_free': meta.get('serial_after', {}).get('free'),
        'residual_processes': meta.get('residual_processes', []),
        'shutdown_exception': meta.get('shutdown_exception'),
        'cycle0_ipc_age_ms': ipc_ages_ms[0] if ipc_ages_ms else None,
        'stats': {
            'serial_write_ms': stats(raw_writes_ms),
            'wrapper_write_ms': stats(wrapper_writes_ms),
            'feedback_ipc_age_ms': stats(ipc_ages_ms),
            'feedback_gap_ms': stats(feedback_gaps_ms),
            'child_wake_lateness_ms': stats(wake_latenesses_ms),
            'child_cycle_interval_ms': stats(cycle_intervals_ms),
            'bridge_thread_cpu_ms': stats(bridge_cpu_ms),
            'host_heartbeat_stalls_gt20ms': stats(hb_stalls),
        },
        'watchlist': {
            'ipc_age_gt100ms': sum(1 for x in ipc_ages_ms if x > 100.0),
            'gap_gt100ms': sum(1 for x in feedback_gaps_ms if x > 100.0),
            'wake_gt30ms': sum(1 for x in wake_latenesses_ms if x > 30.0),
            'warning_283_3ms_count': warning_count,
            'hard_stop_1000ms_count': hard_stop_count,
        },
        'verdict': 'PASS' if pass_hard else 'FAIL'
    }

    out_file = out / 'q1_analysis.json'
    out_file.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: j14_a2_q1_analyze.py <output_directory>", file=sys.stderr)
        sys.exit(1)
    analyze(Path(sys.argv[1]).resolve())
