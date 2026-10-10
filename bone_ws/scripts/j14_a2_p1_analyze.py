#!/usr/bin/env python3
"""Offline P1 production regression evidence; no ROS/serial activity."""
import json
from pathlib import Path
ROOT=Path('/home/lyapunov/bone_drill')


def rows(path):return [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []


def stats(values):
    v=sorted(values)
    if not v:return None
    def q(p):
        x=(len(v)-1)*p;i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
    return dict(n=len(v),P50=q(.5),P95=q(.95),P99=q(.99),Max=v[-1])


def analyze(out):
    meta=json.loads((out/'run_metadata.json').read_text())
    bridge=rows(out/'bridge_feedback_pipeline.jsonl');cycles=rows(out/'child_cycles.jsonl')
    writes=rows(out/'child_write_metrics.jsonl');healthy=[r for r in writes if not r['error']]
    pipeline=rows(out/'bridge_lifecycle.jsonl')
    summary=dict(run_id=meta['run_id'],duration_s=meta['duration_s'],cycles=meta['cycles'],
        bridge_cycles=meta['bridge_cycles'],motion_writes=len(writes),
        tx_timeout_count=meta['tx_timeout_count'],hardware_fault_count=meta['hardware_fault_count'],
        startup_drop_count=meta['first_cycle_seq'],total_missing_cycles=meta['packet_drop_count'],
        duplicate_cycles=meta['duplicate_cycles'],handshake_valid=meta['handshake_valid'],
        production_path_confirmed=meta['production_path_confirmed'],unique_publishers_confirmed=meta['unique_publishers_confirmed'],
        rmw=meta['rmw'],experimental_environment_flags=meta['experimental_environment_flags'],
        feedback_ipc_age_ms=stats([r['ipc_age_ms'] for r in bridge]),
        feedback_gap_ms=stats([r['feedback_gap_ms'] for r in bridge[1:]]),
        child_wake_lateness_ms=stats([r['wake_lateness_ns']/1e6 for r in cycles]),
        serial_write_ms=stats([r['serial_metric']['wall_duration_ns']/1e6 for r in healthy]),
        wrapper_write_ms=stats([r['wrapper_wall_ns']/1e6 for r in healthy]),
        cycle0_ipc_age_ms=bridge[0]['ipc_age_ms'] if bridge else None,
        pause_ack_confirmed=meta['pause_ack_confirmed'],child_returncode=meta['child_returncode'],
        bridge_returncode=meta['bridge_returncode'],serial_free=meta['serial_after']['free'],
        residual_processes=meta['residual_processes'],shutdown_exception=meta['shutdown_exception'],
        bridge_exit=pipeline[-1] if pipeline else None)
    summary['watchlist']=dict(ipc_age_gt100ms=sum(r['ipc_age_ms']>100 for r in bridge),
        gap_gt100ms=sum(r['feedback_gap_ms']>100 for r in bridge),wake_gt30ms=sum(r['wake_lateness_ns']>30_000_000 for r in cycles))
    summary['short_static_PASS']=bool(summary['production_path_confirmed'] and summary['unique_publishers_confirmed']
        and summary['handshake_valid'] and summary['cycles']==summary['bridge_cycles']
        and not summary['tx_timeout_count'] and not summary['hardware_fault_count'] and not summary['total_missing_cycles']
        and not summary['duplicate_cycles'] and summary['pause_ack_confirmed'] and summary['serial_free']
        and not summary['residual_processes'] and summary['child_returncode']==summary['bridge_returncode']==0)
    (out/'p1_analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    result=dict(baseline_A1_13_R2=json.loads((ROOT/'results/j14/j14_a1_13_r2_static_handshake/a1_13_analysis_summary.json').read_text()),production_runs={})
    for n in (1,2):
        out=ROOT/f'results/j14/j14_a2_p1_r{n}_static_production'
        if (out/'run_metadata.json').exists():result['production_runs'][f'P1-R{n}']=analyze(out)
    (ROOT/'results/j14/j14_a2_p1_software/production_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['production_runs'],indent=2))


if __name__=='__main__':main()
