#!/usr/bin/env python3
"""Offline A1.6 interval/tty-trace alignment. Never imports a hardware driver."""
import argparse
import csv
from decimal import Decimal
import json
from pathlib import Path
import re
import statistics


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def ns(seconds):
    return int(Decimal(seconds) * 1_000_000_000)


def analyze(run):
    rows = read_csv(run / 'tx_queue_state.csv')
    traces = {}
    pattern = re.compile(r'^(\d+\.\d+) (write|select|pselect6|poll|ppoll|ioctl|read)\(.* <(\d+\.\d+)>$')
    for path in run.glob('driver_trace.*'):
        entries = []
        for line in path.read_text().splitlines():
            if 'unfinished ...' in line or 'resumed>' in line:
                raise ValueError('Fragmented syscall needs explicit reconstruction: ' + str(path))
            match = pattern.match(line)
            if match:
                start, name, duration = match.groups()
                entries.append(dict(start_epoch_ns=ns(start), duration_ns=ns(duration), syscall=name, raw=line))
        traces[int(path.suffix[1:])] = entries
    aligned = []
    for row in rows:
        before = int(row['clock_sync_monotonic_before_ns'])
        after = int(row['clock_sync_monotonic_after_ns'])
        offset = int(row['clock_sync_epoch_ns']) - (before + after) // 2
        # Mapping bracket plus strace's microsecond print resolution.
        uncertainty = (after - before + 1) // 2 + 1000
        enter = int(row['serial_write_start_ns']) + offset
        exit_ = int(row['serial_write_end_ns']) + offset
        selected = [x for x in traces[int(row['native_thread_id'])]
                    if enter - uncertainty <= x['start_epoch_ns'] <= exit_ + uncertainty]
        total = sum(x['duration_ns'] for x in selected)
        write = sum(x['duration_ns'] for x in selected if x['syscall'] == 'write')
        wait = sum(x['duration_ns'] for x in selected if x['syscall'] in ('select', 'pselect6', 'poll', 'ppoll'))
        serial_wall = int(row['serial_write_end_ns']) - int(row['serial_write_start_ns'])
        wrapper_wall = ns(row['wrapper_write_wall_ms']) // 1000
        gaps = []
        cursor = enter
        for call in selected:
            gaps.append(dict(before_syscall=call['syscall'], duration_ns=call['start_epoch_ns']-cursor))
            cursor = call['start_epoch_ns'] + call['duration_ns']
        gaps.append(dict(before_syscall='application_exit', duration_ns=exit_-cursor))
        aligned.append(dict(application=row, epoch_enter_ns=enter, epoch_exit_ns=exit_,
                            clock_mapping_uncertainty_ns=uncertainty,
                            os_write_total_ns=write, readiness_total_ns=wait,
                            tty_syscall_total_ns=total,
                            serial_non_syscall_elapsed_ns=serial_wall-total,
                            wrapper_minus_tty_syscalls_ns=wrapper_wall-total,
                            gaps=gaps, syscalls=selected))
    failed = [x for x in aligned if x['application']['write_error']]
    longest = max(aligned, key=lambda x:float(x['application']['serial_write_wall_ms']))
    cycles = read_csv(run/'cycle_summary_full.csv')
    heartbeat = read_csv(run/'host_heartbeat_stalls.csv')
    result = dict(failed_write_count=len(failed), failed_writes=failed,
                  application_retained_cycles=[int(rows[0]['cycle_seq']), int(rows[-1]['cycle_seq'])],
                  application_retained_write_count=len(rows), full_cycle_count=len(cycles),
                  serial_wall_median_ms=statistics.median(float(x['serial_write_wall_ms']) for x in rows),
                  wrapper_wall_median_ms=statistics.median(float(x['wrapper_write_wall_ms']) for x in rows),
                  write_syscall_median_ns=statistics.median(x['os_write_total_ns'] for x in aligned),
                  longest_retained_successful_write=longest if not longest['application']['write_error'] else None,
                  heartbeat_stall_count=len(heartbeat), heartbeat_stalls=heartbeat,
                  max_wake_lateness_cycle=max(cycles,key=lambda x:float(x['wake_lateness_ms'])),
                  max_inter_gap_cycle=max(cycles,key=lambda x:float(x['inter_gap_ms'])),
                  scheduler_stall_events=read_csv(run/'stall_events_all.csv'),
                  caveat='Tracing perturbs timing; syscall elapsed includes tracing/descheduling. Residual is not pure Python/GIL time. Successful sample cannot explain an uncaptured timeout.')
    (run/'trace_alignment.json').write_text(json.dumps(result,indent=2)+'\n')
    with (run/'write_syscall_alignment.jsonl').open('w') as stream:
        for x in aligned:
            stream.write(json.dumps(x)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('scheduler_stall_events','heartbeat_stalls')},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    analyze(parser.parse_args().run.resolve())
