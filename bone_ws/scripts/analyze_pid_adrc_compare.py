#!/usr/bin/env python3
"""Offline J14 PID/P/PD vs three-axis LADRC representative comparison.

Example (input order is arbitrary):
  python3 bone_ws/scripts/analyze_pid_adrc_compare.py PID_RUN ADRC_RUN

Only ACTIVE rows are used. Each time axis starts at its first ACTIVE tick;
the default window ends at the shorter run's last ACTIVE tick. RMS is the
unweighted sample RMS. Platform Euler errors are measured minus the captured
zero reference, not the components of the SO(3) rotation vector e_R_W.
No ROS imports, runtime launch, controller changes, or writes to input files.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
AXES = ('roll', 'pitch', 'yaw')
FIGURES = ('fig1_base_disturbance.png', 'fig2_platform_error.png',
           'fig3_error_rms_compare.png')
MODE_IDENTITIES = {'feedforward_pd': 'pid', 'pid': 'pid',
                   'feedforward_ladrc': 'adrc', 'adrc': 'adrc', 'ladrc': 'adrc'}
CLASS_IDENTITIES = {'J11M2PidResidualRunner': 'pid', 'J13LadrcShadowRunner': 'adrc'}
ROW_MODES = {'pid': 'pid', 'adrc': '3axis_active'}


def read_run(directory):
    """Identify from saved metadata and confirm actual ACTIVE control authority."""
    directory = Path(directory).resolve()
    metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
    if not isinstance(metadata, dict):
        raise ValueError(f'{directory}: metadata.json must contain a JSON object')
    mode = metadata.get('mode')
    identity = MODE_IDENTITIES.get(mode)
    class_identity = CLASS_IDENTITIES.get(metadata.get('controller_class'))
    if mode is not None and identity is None:
        raise ValueError(f'{directory}: unsupported metadata mode {mode!r}')
    if identity and class_identity and identity != class_identity:
        raise ValueError(f'{directory}: conflicting controller mode and class')
    identity = identity or class_identity
    if identity is None:
        raise ValueError(f'{directory}: controller identity is not recorded/recognized')

    fields = [f'{prefix}_rel_{axis}_deg' for prefix in ('base', 'platform') for axis in AXES]
    required = {'timestamp_ns', 'cycle_index', 'state', 'controller_mode', *fields}
    with (directory / 'telemetry.csv').open(newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        header = reader.fieldnames or []
        if len(header) != len(set(header)) or not required.issubset(header):
            raise ValueError(f'{directory}: duplicate or missing columns: {sorted(required - set(header))}')
        rows = list(reader)
    if not rows:
        raise ValueError(f'{directory}: empty telemetry.csv')
    if metadata.get('rows_written') is not None and metadata['rows_written'] != len(rows):
        raise ValueError(f'{directory}: metadata rows_written differs from CSV row count')

    timestamps, cycles = [], []
    active_times, active_cycles, values = [], [], []
    for line, row in enumerate(rows, 2):
        try:
            timestamp, cycle = int(row['timestamp_ns']), int(row['cycle_index'])
            if timestamp <= 0 or cycle <= 0:
                raise ValueError('timestamp and cycle must be positive')
            if timestamps and (timestamp <= timestamps[-1] or cycle <= cycles[-1]):
                raise ValueError('timestamps/cycles must strictly increase')
            timestamps.append(timestamp)
            cycles.append(cycle)
            if row['state'] != 'ACTIVE':
                continue
            if row['controller_mode'] != ROW_MODES[identity]:
                raise ValueError(f'ACTIVE controller_mode must be {ROW_MODES[identity]!r}')
            sample = [float(row[field]) for field in fields]
            if not all(math.isfinite(value) for value in sample):
                raise ValueError('ACTIVE attitude contains non-finite values')
            active_times.append(timestamp)
            active_cycles.append(cycle)
            values.append(sample)
        except (ValueError, TypeError, KeyError) as exc:
            raise ValueError(f'{directory}: CSV line {line}: {exc}') from exc
    if len(values) < 2:
        raise ValueError(f'{directory}: need at least two ACTIVE rows')
    if any(b != a + 1 for a, b in zip(active_cycles, active_cycles[1:])):
        raise ValueError(f'{directory}: ACTIVE cycle gap; select a complete recording')

    # Subtract integers before conversion, preserving nanosecond differences.
    relative_time = np.array([(t - active_times[0]) / 1e9 for t in active_times])
    attitudes = np.array(values, dtype=float)
    return dict(directory=directory, metadata=metadata, controller=identity,
                hardware_mode=metadata.get('hardware_mode', 'UNKNOWN'),
                time=relative_time, base=attitudes[:, :3], error=attitudes[:, 3:],
                total_rows=len(rows), active_rows=len(values))


def select_window(runs, start_s=0.0, duration_s=None):
    if not math.isfinite(start_s) or start_s < 0:
        raise ValueError('--start-s must be finite and nonnegative')
    available_end = min(float(run['time'][-1]) for run in runs)
    end_s = available_end if duration_s is None else start_s + duration_s
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('--duration-s must be finite and positive')
    if end_s > available_end or end_s <= start_s:
        raise ValueError('requested window exceeds the shared ACTIVE duration')
    selected = []
    for run in runs:
        mask = (run['time'] >= start_s) & (run['time'] <= end_s)
        if np.count_nonzero(mask) < 2:
            raise ValueError('comparison window needs at least two samples from each run')
        selected.append(dict(run, time=run['time'][mask], base=run['base'][mask],
                             error=run['error'][mask]))
    return selected, end_s


def summarize(run, start_s, end_s):
    base, error = run['base'], run['error']
    base_rms = np.sqrt(np.mean(base ** 2, axis=0))
    error_rms = np.sqrt(np.mean(error ** 2, axis=0))
    base_range = np.ptp(base, axis=0)
    base_peak = np.max(np.abs(base), axis=0)
    error_peak = np.max(np.abs(error), axis=0)
    return [dict(controller=run['controller'], hardware_mode=run['hardware_mode'],
                 axis=axis, samples=len(base), window_start_s=start_s, window_end_s=end_s,
                 first_sample_s=float(run['time'][0]), last_sample_s=float(run['time'][-1]),
                 base_source_field=f'base_rel_{axis}_deg',
                 platform_error_source_field=f'platform_rel_{axis}_deg',
                 base_rms_deg=float(base_rms[i]), base_range_deg=float(base_range[i]),
                 base_peak_deg=float(base_peak[i]), platform_error_rms_deg=float(error_rms[i]),
                 platform_error_max_abs_deg=float(error_peak[i]),
                 run_directory=str(run['directory'])) for i, axis in enumerate(AXES)]


def plot_comparison(runs, summary, out, start_s, end_s):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    gains = runs[0]['metadata'].get('constructor_arguments', {})
    pid_label = 'PID / Classical P-PD' if all(gains.get(f'ki_{a}') == 0 for a in AXES) else 'PID'
    labels = (pid_label, 'ADRC / 3-axis LADRC')
    colors = ('#2066a8', '#ce4c36')
    scope = 'MOCK' if any(r['hardware_mode'] == 'mock' for r in runs) else 'Representative'
    for key, title, filename in (
        ('base', 'Base disturbance comparison', FIGURES[0]),
        ('error', 'Platform Euler attitude error comparison', FIGURES[1]),
    ):
        fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
        for i, axis in enumerate(AXES):
            for run, label, color in zip(runs, labels, colors):
                axes[i].plot(run['time'], run[key][:, i], label=label, color=color, linewidth=1.3)
            axes[i].set_ylabel(f'{axis.title()} (deg)')
            axes[i].grid(alpha=0.25)
        axes[0].legend(loc='best')
        axes[-1].set_xlabel('Time since each run entered ACTIVE (s)')
        axes[-1].set_xlim(start_s, end_s)
        fig.suptitle(f'{scope}: {title}')
        fig.savefig(out / filename, dpi=160)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
    positions = np.arange(3)
    for i, (identity, label, color) in enumerate(zip(('pid', 'adrc'), labels, colors)):
        values = [row['platform_error_rms_deg'] for row in summary if row['controller'] == identity]
        bars = ax.bar(positions + (i - 0.5) * 0.36, values, width=0.36, label=label, color=color)
        ax.bar_label(bars, fmt='%.4g', padding=3)
    ax.set_xticks(positions, [axis.title() for axis in AXES])
    ax.set_ylabel('Platform Euler error RMS (deg)')
    ax.set_ylim(bottom=0)
    ax.set_title(f'{scope}: PID vs ADRC error RMS')
    ax.legend()
    ax.grid(axis='y', alpha=0.25)
    fig.savefig(out / FIGURES[2], dpi=160)
    plt.close(fig)


def write_summary(runs, summary, out, start_s, end_s):
    with (out / 'summary.csv').open('x', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    lines = ['# PID vs ADRC representative comparison', '',
             f'Window: {start_s:.9g} to {end_s:.9g} s after each run first entered ACTIVE.',
             'Only ACTIVE rows are used; native samples are retained without resampling or time warping.',
             'RMS = sqrt(mean(x^2)); range = max(x) - min(x); peak/max absolute error = max(abs(x)).',
             'All reported angles are degrees. RMS is sample-weighted, not time-weighted.', '',
             'Platform errors use platform_rel_roll/pitch/yaw_deg: measured Euler attitude minus the',
             'captured zero reference. These are not e_R_W SO(3) rotation-vector components.',
             'Base angles also use each run\'s captured reference and existing mechanical-frame transform.', '']
    for run in runs:
        meta = run['metadata']
        lines += [f"- {run['controller'].upper()}: `{run['directory']}`",
                  f"  Metadata mode: `{meta.get('mode', 'NOT RECORDED')}`; hardware mode: `{run['hardware_mode']}`.",
                  f"  Rows: {run['total_rows']} total, {run['active_rows']} ACTIVE, {len(run['time'])} selected.",
                  f"  Selected samples span {run['time'][0]:.9g} to {run['time'][-1]:.9g} s.",
                  f"  Recording state: `{meta.get('recording_state', 'NOT RECORDED')}`; stop reason: `{meta.get('stop_reason', 'NOT RECORDED')}`."]
        gains = meta.get('constructor_arguments', {})
        if run['controller'] == 'pid':
            ki = [gains.get(f'ki_{axis}') for axis in AXES]
            if all(value == 0 for value in ki):
                lines.append('  Integral gains are all zero: this PID-family run uses Classical P/PD.')
            else:
                lines.append(f'  Recorded integral gains (roll, pitch, yaw): `{ki}`; missing values are unknown.')
        else:
            lines.append('  ACTIVE telemetry confirms three-axis LADRC authority (controller_mode=3axis_active).')
    lines += ['', '## Base disturbance', '', '| Controller | Axis | RMS (deg) | Range (deg) | Peak abs (deg) |',
              '| --- | --- | ---: | ---: | ---: |']
    for row in summary:
        lines.append(f"| {row['controller'].upper()} | {row['axis'].title()} | {row['base_rms_deg']:.6g} | {row['base_range_deg']:.6g} | {row['base_peak_deg']:.6g} |")
    lines += ['', '## Platform Euler attitude error', '', '| Controller | Axis | RMS (deg) | Max absolute (deg) |',
              '| --- | --- | ---: | ---: |']
    for row in summary:
        lines.append(f"| {row['controller'].upper()} | {row['axis'].title()} | {row['platform_error_rms_deg']:.6g} | {row['platform_error_max_abs_deg']:.6g} |")
    lines += ['', '## Interpretation boundary', '',
              'Human-applied disturbances cannot be perfectly identical. Similar Base RMS/range/peak',
              'alone do not establish identical input waveforms; inspect the Base comparison plot.',
              'This is a representative comparison for the recorded runs and selected window.',
              'It does not establish universal ADRC superiority, global superiority, or external absolute accuracy.']
    if any(run['hardware_mode'] == 'mock' for run in runs):
        lines += ['', '**MOCK DATA: calculation/output validation only. These values do not demonstrate',
                  'real-hardware controller performance or justify a PID/ADRC hardware ranking.**']
    elif not all(run['hardware_mode'] == 'active' for run in runs):
        lines += ['', '**Hardware evidence classification is UNKNOWN for at least one input.**']
    lines += ['', '## Figures', '', *[f'![{name}]({out / name})' for name in FIGURES], '']
    (out / 'summary.md').write_text('\n'.join(lines), encoding='utf-8')


def analyze(run_a, run_b, out, start_s=0.0, duration_s=None):
    runs = [read_run(run_a), read_run(run_b)]
    if {run['controller'] for run in runs} != {'pid', 'adrc'}:
        raise ValueError('provide one PID-family run and one three-axis ADRC run')
    runs.sort(key=lambda run: run['controller'] != 'pid')
    runs, end_s = select_window(runs, start_s, duration_s)
    summary = [row for run in runs for row in summarize(run, start_s, end_s)]
    out = Path(out).resolve()
    if any(out == run['directory'] or run['directory'] in out.parents for run in runs):
        raise ValueError('output must be outside both input run directories')
    if any((out / name).exists() for name in (*FIGURES, 'summary.csv', 'summary.md')):
        raise ValueError('comparison outputs already exist; use a new --out-dir')
    out.mkdir(parents=True, exist_ok=True)
    plot_comparison(runs, summary, out, start_s, end_s)
    write_summary(runs, summary, out, start_s, end_s)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_a', type=Path, help='run directory containing telemetry.csv and metadata.json')
    parser.add_argument('run_b', type=Path, help='second run directory; controller order is arbitrary')
    parser.add_argument('--out-dir', type=Path, default=ROOT / 'results/j14/controller_compare/comparison')
    parser.add_argument('--start-s', type=float, default=0.0, help='window start relative to first ACTIVE tick')
    parser.add_argument('--duration-s', type=float, help='window duration; default: remaining shared ACTIVE duration')
    args = parser.parse_args()
    try:
        summary = analyze(args.run_a, args.run_b, args.out_dir, args.start_s, args.duration_s)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f'Analysis failed: {exc}\n')
    print(f'Comparison saved to {args.out_dir.resolve()} (hardware_write_count = 0; offline analysis only)')
    for identity in ('pid', 'adrc'):
        rows = [row for row in summary if row['controller'] == identity]
        print(f"{identity.upper()}: {rows[0]['samples']} samples; Platform Euler RMS deg = "
              + ', '.join(f"{row['axis']}:{row['platform_error_rms_deg']:.6g}" for row in rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
