#!/usr/bin/env python3
"""Q1F offline inspection. Write only new Q1F outputs, never pre-fix evidence."""
import argparse
import json
from pathlib import Path

ROOT = Path('/home/lyapunov/bone_drill')
PERIOD_NS = 20_000_000


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def stats(values):
    values = sorted(values)
    if not values:
        return None
    def quantile(p):
        index = (len(values) - 1) * p
        i = int(index)
        return round(values[i] + (values[min(i + 1, len(values) - 1)] - values[i]) * (index - i), 6)
    return dict(n=len(values), P50=quantile(.5), P95=quantile(.95), P99=quantile(.99),
                P99_9=quantile(.999), Max=round(values[-1], 6))


def deadline_resets(cycles):
    # Preserve the pre-fix forensic convention: nearest nominal slots of phase
    # displacement. This is an estimate, distinct from nominal - executed.
    result = []
    for before, after in zip(cycles, cycles[1:]):
        shift = after['expected_ns'] - before['expected_ns'] - PERIOD_NS
        if shift:
            result.append(dict(cycle_seq=before['cycle_seq'], next_cycle_seq=after['cycle_seq'],
                phase_shift_ns=shift, skipped_slots_estimate=round(shift / PERIOD_NS)))
    return result


def inspect_run(out):
    meta = json.loads((out / 'run_metadata.json').read_text())
    final = json.loads((out / 'child_final.json').read_text())
    cycles = rows(out / 'child_cycles.jsonl')
    bridge = rows(out / 'bridge_feedback_pipeline.jsonl')
    writes = rows(out / 'child_write_metrics.jsonl')
    pauses = rows(out / 'stop_pause_audit.jsonl')
    nominal = round(meta['duration_s'] * 50)
    reset = deadline_resets(cycles)
    cycle_sequences = [r['cycle_seq'] for r in cycles]
    bridge_sequences = [r['cycle_seq'] for r in bridge]
    measurements = dict(
        serial_write_ms=stats([r['serial_metric']['wall_duration_ns'] / 1e6 for r in writes if r.get('serial_metric')]),
        wrapper_wall_ms=stats([r['wrapper_wall_ns'] / 1e6 for r in writes]),
        cycle_wall_ms=stats([(r['end_ns'] - r['start_ns']) / 1e6 for r in cycles]),
        cycle_interval_ms=stats([(b['start_ns'] - a['start_ns']) / 1e6 for a, b in zip(cycles, cycles[1:])]),
        wake_lateness_ms=stats([r['wake_lateness_ns'] / 1e6 for r in cycles]),
        thread_cpu_ms=stats([r['thread_cpu_ns'] / 1e6 for r in cycles]),
        feedback_ipc_age_ms=stats([r['ipc_age_ms'] for r in bridge]),
        feedback_gap_ms=stats([r['feedback_gap_ms'] for r in bridge[1:]]))
    missing = sorted(set(range(final['cycles'])) - set(bridge_sequences))
    duplicate = len(bridge_sequences) - len(set(bridge_sequences))
    timeout_count = sum(r.get('error') == 'SerialTimeoutException' for r in writes)
    fault_cycles = sum(bool(r.get('fault')) for r in cycles)
    queue_count = sum(any((r.get('serial_metric') or {}).get(k) is not None for k in ('queue_before', 'queue_after')) for r in writes)
    raw_pause_ok = bool(pauses) and pauses[-1]['success'] and len(pauses[-1]['records']) == 6 and all(
        r['response_received'] and not r['timeout'] and not r['error'] for r in pauses[-1]['records'])
    # Offline qualification admission only; no new driver STOP or watchdog.
    obvious_stall = bool(measurements['wrapper_wall_ms'] and measurements['wrapper_wall_ms']['Max'] >= 20) or bool(
        measurements['cycle_wall_ms'] and measurements['cycle_wall_ms']['Max'] >= 40)
    passed = bool(meta['stop_reason'] == 'window_complete' and len(cycles) == final['cycles'] == nominal
        and cycle_sequences == list(range(nominal)) and bridge_sequences == cycle_sequences
        and len(writes) == nominal * 6 and timeout_count == fault_cycles == 0
        and meta.get('hardware_fault_count') == 0 and not missing and not duplicate
        and not reset and queue_count == 0 and raw_pause_ok and meta['pause_ack_confirmed']
        and meta['child_returncode'] == meta['bridge_returncode'] == 0
        and meta['serial_after']['free'] and not meta['residual_processes']
        and meta['handshake_valid'] and meta['production_path_confirmed'] and meta['unique_publishers_confirmed']
        and not meta.get('shutdown_escalation') and not obvious_stall)
    return dict(run_directory=str(out), nominal_slots=nominal, executed_cycles=len(cycles), bridge_cycles=len(bridge),
        nominal_minus_executed=nominal-len(cycles), deadline_reset_count=len(reset), deadline_resets=reset,
        skipped_nominal_slots_estimate=sum(r['skipped_slots_estimate'] for r in reset),
        phase_shift_slots=sum(r['phase_shift_ns'] for r in reset) / PERIOD_NS,
        tx_timeout_count=timeout_count, fault_cycles=fault_cycles, ipc_missing_sequences=missing,
        duplicate_cycles=duplicate, motion_write_records=len(writes), queue_snapshot_records=queue_count,
        pause_ack_confirmed=raw_pause_ok, serial_free=meta['serial_after']['free'], residual_processes=meta['residual_processes'],
        shutdown_exception=meta['shutdown_exception'], measurements=measurements,
        obvious_hot_path_stall=obvious_stall, Q1F_PASS=passed)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_directory', type=Path)
    args = parser.parse_args()
    out = args.run_directory.resolve()
    if 'j14_a2_q1f_' not in out.name:
        parser.error('writes restricted to new Q1F run directories')
    dest = out / 'q1f_analysis.json'
    if dest.exists():
        parser.error('refusing to overwrite analysis')
    result = inspect_run(out)
    dest.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not result['Q1F_PASS']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
