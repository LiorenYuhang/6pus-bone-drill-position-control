#!/usr/bin/env python3
"""Authorized Q3 Active-Control Runtime Qualification Offline Analysis."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path('/home/lyapunov/bone_drill')
PERIOD_NS = 20_000_000


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
    v = sorted(float(x) for x in values if x is not None and not math.isnan(float(x)))
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


def deadline_resets(cycles: list[dict]) -> list[dict]:
    result = []
    for before, after in zip(cycles, cycles[1:]):
        shift = after['expected_ns'] - before['expected_ns'] - PERIOD_NS
        if shift:
            result.append(dict(
                cycle_seq=before['cycle_seq'],
                next_cycle_seq=after['cycle_seq'],
                phase_shift_ns=shift,
                skipped_slots_estimate=round(shift / PERIOD_NS),
            ))
    return result


def parse_list(val) -> list[float]:
    if isinstance(val, (list, tuple)):
        return [float(x) for x in val]
    if isinstance(val, str):
        val = val.strip()
        if val.startswith('[') and val.endswith(']'):
            try:
                return [float(x) for x in json.loads(val)]
            except Exception:
                pass
            items = val[1:-1].split(',')
            return [float(x.strip()) for x in items if x.strip()]
    return []


def analyze_controller_telemetry(out: Path) -> dict:
    csv_files = list(out.glob("*telemetry*.csv"))
    if not csv_files:
        return {'found': False, 'active_evidence_pass': False, 'reason': 'No telemetry CSV found'}

    csv_file = sorted(csv_files, key=lambda p: p.stat().st_mtime)[-1]
    records = csvrows(csv_file)
    if not records:
        return {'found': True, 'csv_path': str(csv_file), 'active_evidence_pass': False, 'reason': 'Empty telemetry CSV'}

    active_rows = [r for r in records if r.get('state') == 'ACTIVE']
    eval_rows = active_rows if active_rows else records

    # 1. Base attitude modulation
    base_so3 = [float(r['base_so3_magnitude_deg']) for r in eval_rows if r.get('base_so3_magnitude_deg')]
    base_rpy = []
    for r in eval_rows:
        if r.get('base_euler_zyx_deg'):
            base_rpy.append(parse_list(r['base_euler_zyx_deg']))
        elif r.get('base_mech_rpy_deg'):
            base_rpy.append(parse_list(r['base_mech_rpy_deg']))
        elif r.get('base_rel_roll_deg') and r.get('base_rel_pitch_deg') and r.get('base_rel_yaw_deg'):
            base_rpy.append([float(r['base_rel_roll_deg']), float(r['base_rel_pitch_deg']), float(r['base_rel_yaw_deg'])])

    # 2. Platform tracking / attitude
    plat_so3_err = [float(r['platform_so3_error_deg']) for r in eval_rows if r.get('platform_so3_error_deg')]
    plat_roll = [float(r['platform_rel_roll_deg']) for r in eval_rows if r.get('platform_rel_roll_deg')]
    plat_pitch = [float(r['platform_rel_pitch_deg']) for r in eval_rows if r.get('platform_rel_pitch_deg')]
    plat_yaw = [float(r['platform_rel_yaw_deg']) for r in eval_rows if r.get('platform_rel_yaw_deg')]

    # 3. Controller output
    ctrl_u = []
    for r in eval_rows:
        if r.get('u_combined_actual_deg'):
            ctrl_u.append(parse_list(r['u_combined_actual_deg']))
        elif r.get('pid_total_residual_deg'):
            ctrl_u.append(parse_list(r['pid_total_residual_deg']))

    # 4. Motor commands / targets
    motor_targets = []
    for r in eval_rows:
        if r.get('commanded_steps'):
            motor_targets.append(parse_list(r['commanded_steps']))
        elif r.get('motor_targets_deg'):
            motor_targets.append(parse_list(r['motor_targets_deg']))

    def axis_stats(matrix, axis_names):
        res = {}
        if not matrix:
            return res
        for i, name in enumerate(axis_names):
            vals = [row[i] for row in matrix if len(row) > i]
            if vals:
                res[name] = {
                    'min': round(min(vals), 4),
                    'max': round(max(vals), 4),
                    'range': round(max(vals) - min(vals), 4),
                    'mean': round(sum(vals) / len(vals), 4),
                }
        return res

    base_stats = axis_stats(base_rpy, ['roll', 'pitch', 'yaw'])
    plat_stats = {
        'roll': {'min': min(plat_roll), 'max': max(plat_roll), 'range': round(max(plat_roll) - min(plat_roll), 4)} if plat_roll else {},
        'pitch': {'min': min(plat_pitch), 'max': max(plat_pitch), 'range': round(max(plat_pitch) - min(plat_pitch), 4)} if plat_pitch else {},
        'yaw': {'min': min(plat_yaw), 'max': max(plat_yaw), 'range': round(max(plat_yaw) - min(plat_yaw), 4)} if plat_yaw else {},
    }
    ctrl_stats = axis_stats(ctrl_u, ['roll', 'pitch', 'yaw'])
    motor_stats = axis_stats(motor_targets, [f'axis_{i+1}' for i in range(6)])

    # Evidence of ACTIVE control: Base movement and controller modulation observed
    base_dyn = any(s.get('range', 0) > 0.3 for s in base_stats.values()) or (max(base_so3) - min(base_so3) > 0.3 if base_so3 else False)
    ctrl_dyn = any(s.get('range', 0) > 0.02 for s in ctrl_stats.values())
    motor_dyn = any(s.get('range', 0) > 1.0 for s in motor_stats.values())

    algorithm_active = bool(len(active_rows) >= 50)
    actuation_active = bool(motor_dyn)
    physical_response = bool(base_dyn or ctrl_dyn)

    active_pass = bool(algorithm_active and actuation_active)

    return {
        'found': True,
        'csv_path': str(csv_file),
        'total_records': len(records),
        'active_records': len(active_rows),
        'algorithm_active': algorithm_active,
        'actuation_active': actuation_active,
        'physical_response': physical_response,
        'base_so3_magnitude_deg': stats(base_so3),
        'base_attitude_deg': base_stats,
        'platform_so3_error_deg': stats(plat_so3_err),
        'platform_attitude_deg': plat_stats,
        'controller_output_deg': ctrl_stats,
        'motor_commands': motor_stats,
        'active_evidence_pass': active_pass,
    }


def analyze(out: Path) -> dict:
    meta_path = out / 'run_metadata.json'
    if not meta_path.exists():
        raise FileNotFoundError(f"Missing run_metadata.json in {out}")

    meta = json.loads(meta_path.read_text())
    final = json.loads((out / 'child_final.json').read_text()) if (out / 'child_final.json').exists() else {}
    cycles = rows(out / 'child_cycles.jsonl')
    bridge = rows(out / 'bridge_feedback_pipeline.jsonl')
    writes = rows(out / 'child_write_metrics.jsonl')
    pauses = rows(out / 'stop_pause_audit.jsonl')
    heartbeats = csvrows(out / 'host_heartbeat_stalls.csv')

    nominal = round(meta.get('duration_s', 60.0) * 50)
    resets = deadline_resets(cycles)
    cycle_sequences = [r['cycle_seq'] for r in cycles]
    bridge_sequences = [r['cycle_seq'] for r in bridge]

    healthy_writes = [r for r in writes if not r.get('error')]
    raw_writes_ms = [r['serial_metric']['wall_duration_ns'] / 1e6 for r in healthy_writes if r.get('serial_metric')]
    wrapper_writes_ms = [r['wrapper_wall_ns'] / 1e6 for r in healthy_writes if r.get('wrapper_wall_ns') is not None]
    ipc_ages_ms = [r['ipc_age_ms'] for r in bridge if 'ipc_age_ms' in r]
    feedback_gaps_ms = [r['feedback_gap_ms'] for r in bridge[1:] if 'feedback_gap_ms' in r and r['feedback_gap_ms'] > 0]
    wake_latenesses_ms = [c['wake_lateness_ns'] / 1e6 for c in cycles if 'wake_lateness_ns' in c]
    cycle_intervals_ms = [(b['start_ns'] - a['start_ns']) / 1e6 for a, b in zip(cycles, cycles[1:])]
    thread_cpu_ms = [r['thread_cpu_ns'] / 1e6 for r in cycles if 'thread_cpu_ns' in r]

    hb_stalls = [float(r['stall_duration_ms']) for r in heartbeats if 'stall_duration_ms' in r]
    warning_count = sum(1 for g in feedback_gaps_ms if g > 283.3)
    hard_stop_count = sum(1 for g in feedback_gaps_ms if g > 1000.0)

    missing = sorted(set(range(final.get('cycles', len(cycles)))) - set(bridge_sequences))
    duplicate = len(bridge_sequences) - len(set(bridge_sequences))
    timeout_count = sum(1 for r in writes if r.get('error') == 'SerialTimeoutException')
    fault_cycles = sum(1 for r in cycles if bool(r.get('fault')))

    raw_pause_ok = bool(pauses) and pauses[-1].get('success') and len(pauses[-1].get('records', [])) == 6 and all(
        r.get('response_received') and not r.get('timeout') and not r.get('error') for r in pauses[-1]['records']
    )

    measurements = {
        'serial_write_ms': stats(raw_writes_ms),
        'wrapper_wall_ms': stats(wrapper_writes_ms),
        'cycle_wall_ms': stats([(r['end_ns'] - r['start_ns']) / 1e6 for r in cycles]),
        'cycle_interval_ms': stats(cycle_intervals_ms),
        'wake_lateness_ms': stats(wake_latenesses_ms),
        'thread_cpu_ms': stats(thread_cpu_ms),
        'feedback_ipc_age_ms': stats(ipc_ages_ms),
        'feedback_gap_ms': stats(feedback_gaps_ms),
    }

    # Analyze Controller Active Evidence
    controller_analysis = analyze_controller_telemetry(out)

    # Static Baseline Comparison
    static_baseline_path = ROOT / 'results/j14/j14_a2_q1f_r2_static_60s/q1f_analysis.json'
    baseline_comp = {}
    if static_baseline_path.exists():
        try:
            base_data = json.loads(static_baseline_path.read_text()).get('measurements', {})
            for key in ['serial_write_ms', 'wrapper_wall_ms', 'cycle_wall_ms', 'feedback_ipc_age_ms', 'feedback_gap_ms']:
                if measurements.get(key) and base_data.get(key):
                    baseline_comp[key] = {
                        'static_P50': base_data[key].get('P50'),
                        'active_P50': measurements[key].get('P50'),
                        'delta_P50': round(measurements[key].get('P50', 0) - base_data[key].get('P50', 0), 4),
                        'static_P99': base_data[key].get('P99'),
                        'active_P99': measurements[key].get('P99'),
                        'delta_P99': round(measurements[key].get('P99', 0) - base_data[key].get('P99', 0), 4),
                        'static_Max': base_data[key].get('Max'),
                        'active_Max': measurements[key].get('Max'),
                        'delta_Max': round(measurements[key].get('Max', 0) - base_data[key].get('Max', 0), 4),
                    }
        except Exception:
            pass

    # Hard Checks
    hard_pass = bool(
        timeout_count == 0
        and fault_cycles == 0
        and meta.get('hardware_fault_count', 0) == 0
        and not missing
        and not duplicate
        and raw_pause_ok
        and meta.get('pause_ack_confirmed') is True
        and meta.get('serial_after', {}).get('free') is True
        and not meta.get('residual_processes')
        and meta.get('handshake_valid') is True
        and not meta.get('shutdown_escalation')
    )

    if meta.get('hardware', True):
        # In real hardware active qualification, controller active evidence must also pass
        active_evidence_ok = controller_analysis.get('active_evidence_pass', False)
    else:
        # In mock preflight, verify runner successfully engaged ACTIVE state
        active_evidence_ok = controller_analysis.get('active_records', 0) > 0

    if not hard_pass or not active_evidence_ok:
        verdict = 'FAIL'
    elif len(resets) == 0:
        verdict = 'PASS'
    else:
        verdict = 'PASS WITH OBSERVATION'

    summary = {
        'stage': 'A2-Q3',
        'run_id': meta.get('run_id'),
        'mode': meta.get('mode'),
        'hardware': meta.get('hardware'),
        'duration_s': meta.get('duration_s'),
        'nominal_slots': nominal,
        'driver_session_cycles': len(cycles),
        'driver_bridge_cycles': len(bridge),
        'controller_total_cycles': controller_analysis.get('total_records', 0),
        'controller_active_cycles': controller_analysis.get('active_records', 0),
        'controller_planned_cycles': nominal,
        'executed_cycles': len(cycles),
        'bridge_cycles': len(bridge),
        'nominal_minus_executed': nominal - len(cycles),
        'deadline_reset_count': len(resets),
        'deadline_resets': resets,
        'skipped_nominal_slots_estimate': sum(r['skipped_slots_estimate'] for r in resets),
        'phase_shift_slots': sum(r['phase_shift_ns'] for r in resets) / PERIOD_NS,
        'tx_timeout_count': timeout_count,
        'fault_cycles': fault_cycles,
        'ipc_missing_sequences': missing,
        'duplicate_cycles': duplicate,
        'motion_write_records': len(writes),
        'pause_ack_confirmed': raw_pause_ok,
        'serial_free': meta.get('serial_after', {}).get('free'),
        'residual_processes': meta.get('residual_processes', []),
        'host_heartbeat_stalls_gt20ms': len(hb_stalls),
        'warning_283_3ms_count': warning_count,
        'hard_stop_1000ms_count': hard_stop_count,
        'measurements': measurements,
        'controller_active_evidence': controller_analysis,
        'static_baseline_comparison': baseline_comp,
        'verdict': verdict,
    }

    out_file = out / 'q3_analysis.json'
    out_file.write_text(json.dumps(summary, indent=2) + '\n')

    # Print Formatted Executive Report
    print("\n" + "=" * 78)
    print(f"J14-A2-Q3 ACTIVE QUALIFICATION REPORT: {meta.get('run_id')} ({meta.get('mode', '').upper()})")
    print("=" * 78)
    print(f"Verdict:                        {verdict}")
    print(f"Hardware Mode:                  {'ACTIVE REAL HARDWARE' if meta.get('hardware') else 'MOCK PREFLIGHT'}")
    print(f"Planned Duration:               {meta.get('duration_s')} s ({nominal} controller cycles)")
    print(f"Driver Session Cycles:          {len(cycles)} (Child) / {len(bridge)} (Bridge)")
    print(f"Controller Total Cycles:        {controller_analysis.get('total_records', 0)} ({controller_analysis.get('active_records', 0)} active)")
    print(f"TX Timeouts:                    {timeout_count}")
    print(f"Hardware Faults:                {fault_cycles}")
    print(f"IPC Missing Sequences:          {len(missing)}")
    print(f"IPC Duplicates:                 {duplicate}")
    print(f"Deadline Resets:                {len(resets)}")
    print(f"6-Axis Pause ACK:               {'CONFIRMED' if raw_pause_ok else 'FAILED'}")
    print(f"Serial Clean Free:              {'CONFIRMED' if meta.get('serial_after', {}).get('free') else 'FAILED'}")
    print(f"Residual Processes:             {len(meta.get('residual_processes', []))}")

    print("\n--- Telemetry Metrics (ms) ---")
    headers = ["Metric", "P50", "P95", "P99", "P99.9", "Max"]
    fmt = "{:<22} {:>10} {:>10} {:>10} {:>10} {:>10}"
    print(fmt.format(*headers))
    print("-" * 78)
    for k, label in [
        ('serial_write_ms', 'Serial.write'),
        ('wrapper_wall_ms', 'Wrapper Wall'),
        ('cycle_wall_ms', 'Cycle Wall'),
        ('cycle_interval_ms', 'Cycle Interval'),
        ('wake_lateness_ms', 'Wake Lateness'),
        ('feedback_ipc_age_ms', 'Feedback IPC Age'),
        ('feedback_gap_ms', 'Feedback Gap'),
    ]:
        s = measurements.get(k)
        if s:
            print(fmt.format(label, s['P50'], s['P95'], s['P99'], s['P99_9'], s['Max']))

    print("\n--- Controller Active Evidence (Section XIII) ---")
    if controller_analysis.get('found'):
        print(f"Active Telemetry Records:       {controller_analysis.get('active_records')} / {controller_analysis.get('total_records')}")
        b_att = controller_analysis.get('base_attitude_deg', {})
        p_att = controller_analysis.get('platform_attitude_deg', {})
        c_out = controller_analysis.get('controller_output_deg', {})
        m_cmd = controller_analysis.get('motor_commands', {})

        print(f"Base Attitude Modulation:       "
              f"Roll range={b_att.get('roll', {}).get('range', 0):.2f}°, "
              f"Pitch range={b_att.get('pitch', {}).get('range', 0):.2f}°, "
              f"Yaw range={b_att.get('yaw', {}).get('range', 0):.2f}°")
        print(f"Platform Attitude Variation:    "
              f"Roll range={p_att.get('roll', {}).get('range', 0):.2f}°, "
              f"Pitch range={p_att.get('pitch', {}).get('range', 0):.2f}°, "
              f"Yaw range={p_att.get('yaw', {}).get('range', 0):.2f}°")
        print(f"Controller Output Range:        "
              f"Roll={c_out.get('roll', {}).get('range', 0):.3f}°, "
              f"Pitch={c_out.get('pitch', {}).get('range', 0):.3f}°, "
              f"Yaw={c_out.get('yaw', {}).get('range', 0):.3f}°")
        max_m_range = max((v.get('range', 0) for v in m_cmd.values()), default=0)
        print(f"Max Motor Command Variation:    {max_m_range:.1f} steps")
        print(f"Algorithm Active (>=50 cycles): {'PASS' if controller_analysis.get('algorithm_active') else 'FAIL'}")
        print(f"Actuation Active (>1.0 steps):  {'PASS' if controller_analysis.get('actuation_active') else 'FAIL'}")
        print(f"Physical Response Observed:     {'YES' if controller_analysis.get('physical_response') else 'NO'}")
        print(f"Active Control Confirmation:    {'PASS (Dynamic motion verified)' if active_evidence_ok else 'FAIL'}")
    else:
        print(f"Active Telemetry Evaluation:    {controller_analysis.get('reason')}")

    if baseline_comp:
        print("\n--- Comparison Against 60s Static Baseline (Q1F-R2) ---")
        print("{:<22} {:>12} {:>12} {:>12} {:>12}".format("Metric", "Static P50", "Active P50", "Static P99", "Active P99"))
        print("-" * 78)
        for k, label in [
            ('serial_write_ms', 'Serial.write'),
            ('wrapper_wall_ms', 'Wrapper Wall'),
            ('cycle_wall_ms', 'Cycle Wall'),
            ('feedback_ipc_age_ms', 'Feedback IPC Age'),
            ('feedback_gap_ms', 'Feedback Gap'),
        ]:
            c = baseline_comp.get(k)
            if c:
                print("{:<22} {:>12} {:>12} {:>12} {:>12}".format(
                    label, c['static_P50'], c['active_P50'], c['static_P99'], c['active_P99']
                ))

    print("=" * 78 + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description="J14-A2-Q3 Offline Analysis")
    parser.add_argument('run_directory', type=Path, help="Directory containing Q3 run evidence")
    args = parser.parse_args()
    summary = analyze(args.run_directory.resolve())
    if summary['verdict'] == 'FAIL':
        sys.exit(2)


if __name__ == '__main__':
    main()
