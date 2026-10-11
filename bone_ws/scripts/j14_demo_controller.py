#!/usr/bin/env python3
"""Demo lifecycle host for unchanged J11/J13 runner nodes (no control law here)."""
from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import asdict
import inspect
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'bone_ws/src/h30_imu/scripts'))
CONFIG_DIR = ROOT / 'bone_ws/src/h30_imu/config'
MODES = {
    'feedforward': ('j11_feedforward_params.yaml', 'Base SO(3) Feedforward only'),
    'feedforward_pd': ('j11_m2_pid_params.yaml', 'Base SO(3) Feedforward + Classical P/PD'),
    'feedforward_ladrc': ('j13_ladrc_params.yaml', 'Base SO(3) Feedforward + 3-Axis First-Order LADRC'),
}


def write_json(path, value):
    # Readers see either the previous complete state or the new complete state.
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def controller_spec(mode):
    import yaml
    if mode not in MODES:
        raise ValueError(f'Invalid mode: {mode}')
    config = CONFIG_DIR / MODES[mode][0]
    document = yaml.safe_load(config.read_text())
    if mode == 'feedforward':
        from j11_base_feedforward_runner import J11BaseFeedforwardRunner
        cls, params = J11BaseFeedforwardRunner, {}
    elif mode == 'feedforward_pd':
        from j11_m2_pid_residual_runner import J11M2PidResidualRunner
        cls = J11M2PidResidualRunner
        params = document['j11_m2_attitude_pid']['ros__parameters']
    else:
        from j13_ladrc_shadow_runner import J13LadrcShadowRunner
        cls = J13LadrcShadowRunner
        params = document['j13_ladrc_shadow_runner']['ros__parameters']
    # The frozen YAML is the only parameter source; never the runner CLI defaults.
    names = inspect.signature(cls).parameters
    kwargs = {key: value for key, value in params.items() if key in names}
    if mode == 'feedforward_ladrc':
        kwargs['controller_mode'] = '3axis_active'
    return cls, kwargs, config


class TelemetryRecorder:
    """Queue snapshots without waiting for disk; one worker owns CSV writes."""

    def __init__(self, out, mode, hardware_mode, cls, kwargs, config):
        self.path = out / 'metadata.json'
        self.file = (out / 'telemetry.csv').open('x', newline='', encoding='utf-8')
        self.writer = None
        self.last_cycle = None
        self.last_flush = time.monotonic()
        self.pending = queue.Queue(maxsize=250)
        self.stopping = threading.Event()
        self.error = None
        self.final_info = {}
        self.worker = threading.Thread(target=self._write_pending,
                                       name='j14-telemetry-writer', daemon=True)
        self.meta = dict(
            schema_version=1, mode=mode, controller_description=MODES[mode][1],
            hardware_mode=hardware_mode, controller_class=cls.__name__,
            config=str(config), constructor_arguments=dict(
                execute_motion=True, dry_run=False, auto_start=True,
                output_dir=str(out), **kwargs),
            recording_state='recording', start_epoch_ns=time.time_ns(),
            timestamp_clock='time.perf_counter_ns (host control tick; not sensor or UTC time)',
            csv_file='telemetry.csv', columns=[], rows_written=0,
            first_cycle_index=None, last_cycle_index=None,
            first_timestamp_ns=None, last_timestamp_ns=None,
            structured_cells='JSON arrays/objects; other cells use CSV scalar values',
            field_conventions=dict(
                angles='*_deg: degrees; *_rad: radians',
                relative_attitude='startup reference with existing mechanical-frame transform',
                e_R_W='World-frame SO(3) rotation vector, not Euler-angle error',
                controller_output=('u_combined_actual_rad/deg' if mode == 'feedforward_ladrc'
                                   else 'pid_total_residual_rad/deg' if mode == 'feedforward_pd'
                                   else 'feedforward command attitude; no residual controller'),
                commanded_steps='latest /pus6/command_joint_steps; not UART TX acknowledgement',
                feedback_steps='latest relayed feedback; not synchronous with controller output'),
            coverage='all emitted telemetry rows, including preflight; fault ticks may return before append',
            flush_interval_s=1.0,
            recording_buffer_rows=self.pending.maxsize,
            recording_io='dedicated writer thread; non-waiting enqueue; errors are reported',
        )
        try:
            git = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                                 capture_output=True, text=True, check=False)
            self.meta['git_head'] = git.stdout.strip() if git.returncode == 0 else None
            write_json(self.path, self.meta)
            self.worker.start()
        except Exception:
            self.file.close()
            raise

    def capture(self, history):
        # Single-threaded spin_once executes at most one control callback.
        if self.error is not None:
            raise RuntimeError('Telemetry recording failed') from self.error
        if history and history[-1].cycle_index != self.last_cycle:
            row = asdict(history[-1])
            try:
                self.pending.put_nowait(row)
            except queue.Full as exc:
                self.error = exc
                raise RuntimeError('Telemetry recording buffer full; recording is incomplete') from exc
            self.last_cycle = row['cycle_index']

    def _write_pending(self):
        try:
            while not self.stopping.is_set() or not self.pending.empty():
                try:
                    row = self.pending.get(timeout=0.05)
                except queue.Empty:
                    row = None
                if row is not None:
                    if self.writer is None:
                        self.meta['columns'] = list(row)
                        self.writer = csv.DictWriter(self.file, fieldnames=list(row))
                        self.writer.writeheader()
                    self.writer.writerow({key: json.dumps(value) if isinstance(value, (list, tuple, dict))
                                          else value for key, value in row.items()})
                    if self.meta['rows_written'] == 0:
                        self.meta.update(first_cycle_index=row['cycle_index'],
                                         first_timestamp_ns=row['timestamp_ns'])
                    self.meta.update(rows_written=self.meta['rows_written'] + 1,
                                     last_cycle_index=row['cycle_index'],
                                     last_timestamp_ns=row['timestamp_ns'])
                if time.monotonic() - self.last_flush >= self.meta['flush_interval_s']:
                    self.file.flush()
                    self.last_flush = time.monotonic()
            self.file.close()
            self.meta.update(self.final_info, recording_state='closed' if self.error is None else 'failed')
            if self.error is not None:
                self.meta['recording_error'] = repr(self.error)
                self.meta['controller_returncode'] = 1
            write_json(self.path, self.meta)
        except Exception as exc:
            self.error = exc
            self.meta.update(self.final_info, recording_state='failed', recording_error=repr(exc),
                             controller_returncode=1)
            try:
                write_json(self.path, self.meta)
            except OSError:
                pass  # Initial metadata remains 'recording', never a false 'closed'.
        finally:
            try:
                self.file.close()
            except Exception as exc:
                self.error = self.error or exc

    def close(self, node, reason, code):
        try:
            if node is not None:
                self.capture(node.telemetry_history)
        finally:
            self.final_info = dict(stop_epoch_ns=time.time_ns(), stop_reason=reason,
                                   controller_returncode=code,
                                   final_state=node.state.value if node is not None else None,
                                   controller_cycles=node.cycle_index if node is not None else 0)
            self.stopping.set()
            # Do not let a stalled filesystem hold controller teardown indefinitely.
            self.worker.join(timeout=1.0)
            if self.worker.is_alive():
                self.error = TimeoutError('Telemetry writer did not drain; recording is incomplete')
                raise self.error
            if self.error is not None:
                raise RuntimeError('Telemetry recording failed') from self.error


def run_controller(mode, out, hardware_mode, record_telemetry=False):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    prefix = 'j13' if mode == 'feedforward_ladrc' else 'j11'
    markers = [Path('/tmp') / f'{prefix}_{s}' for s in
               ('active_signaled', 'ready_for_start', 'driver_stopped', 'start_signal')]
    previous_markers = {p: p.read_bytes() if p.exists() else None for p in markers}
    node = None
    recorder = None
    home_result = None
    reason, code, active_seen = 'requested_stop', 0, False
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        cls, kwargs, config = controller_spec(mode)
        if record_telemetry:
            recorder = TelemetryRecorder(out, mode, hardware_mode, cls, kwargs, config)
        # execute_motion enables Cartesian publication, even with the MOCK driver.
        # The legacy runner's hardware_write_count counts pose publications, not UART writes.
        node = cls(execute_motion=True, dry_run=False, auto_start=True,
                   output_dir=str(out), **kwargs)
        # Demo runs indefinitely. Bound diagnostic retention, not controller state.
        node.telemetry_history = deque(maxlen=250)
        if hasattr(node, 'slew_events_history'):
            node.slew_events_history = deque(maxlen=250)
        from j14_demo_home import attach_feedback, feedback_failure, recover_home
        feedback = attach_feedback(node)
        def request_reader():
            path=out/'stop_request.json'
            return json.loads(path.read_text()) if path.exists() else {}
        last_state = None
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if recorder is not None:
                recorder.capture(node.telemetry_history)
            request=request_reader()
            if request.get('action')=='safety_pause':
                reason=request.get('reason','safety_stop');break
            if request.get('action')=='normal_home' and home_result is None:
                node.timer.cancel()
                write_json(out/'controller_state.json',dict(state='HOME_RECOVERY',mode=mode,
                    active_seen=active_seen,active_output_stopped_ns=time.monotonic_ns()))
                home_result=recover_home(node,feedback,out,stop,request_reader)
                if home_result['state']!='HOME_CONFIRMED':
                    code=1;reason=home_result.get('reason',home_result['state'])
            state = home_result['state'] if home_result is not None else node.state.value
            if state != last_state:
                active_seen = active_seen or state == 'ACTIVE'
                with (out/'controller_states.jsonl').open('a') as audit:
                    audit.write(json.dumps(dict(state=state,monotonic_ns=time.monotonic_ns()))+'\n')
                write_json(out / 'controller_state.json', dict(
                    state=state, mode=mode, controller_class=cls.__name__,
                    config=str(config), parameters=kwargs,
                    preflight_pass=active_seen, active_seen=active_seen, home_recovery=home_result,
                    monotonic_ns=time.monotonic_ns(), fault_reason=node.fault_reason))
                last_state = state
            if state == 'FAULT':
                reason, code = node.fault_reason, 1
                break
    except Exception as exc:
        reason, code = repr(exc), 1
        raise
    finally:
        if node is not None:
            # No further executor calls after cancellation: ACTIVE output is stopped first.
            node.timer.cancel()
        try:
            if node is not None:
                last_telem = asdict(node.telemetry_history[-1]) if node.telemetry_history else None
                write_json(out / 'controller_final.json', dict(
                    mode=mode, state=node.state.value, active_seen=active_seen,
                    output_stopped_ns=time.monotonic_ns(), stop_reason=reason,
                    cartesian_pose_publish_count=node.hardware_write_count,
                    hardware_write_count=0 if hardware_mode == 'mock' else None,
                    hardware_write_count_source='driver child_final.json; runner counter is pose publication',
                    active_cycles=node.active_cycle_count, cycles=node.cycle_index, home_recovery=home_result,
                    last_telemetry=last_telem, retained_rows=len(node.telemetry_history)))
        finally:
            try:
                if recorder is not None:
                    recorder.close(node, reason, code)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                for p, contents in previous_markers.items():
                    if contents is None:
                        p.unlink(missing_ok=True)
                    else:
                        p.write_bytes(contents)
    return code


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--hardware-mode', choices=('mock', 'active'), required=True)
    parser.add_argument('--record-telemetry', action='store_true',
                        help='Continuously save telemetry.csv and metadata.json (default: off)')
    args = parser.parse_args()
    sys.exit(run_controller(args.mode, args.out_dir, args.hardware_mode, args.record_telemetry))
