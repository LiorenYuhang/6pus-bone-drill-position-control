"""Diagnostic-only queue snapshots and installed PySerial logical budget semantics."""
import csv
import os
import pty
import time
from unittest.mock import patch

import pytest
import serial
from serial import serialposix, serialutil

from test_j14_a1_bounded_write import DeadlineSerial
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7AdapterWriteTimeout
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.stall_diagnostic_recorder import J11StallDiagnosticRecorder


class QueueSerial(DeadlineSerial):
    def __init__(self):
        super().__init__()
        self.queue_reads = 0
        self.queue_error = False
        self.queue_delay_s = 0

    @property
    def out_waiting(self):
        self.queue_reads += 1
        if self.queue_delay_s:
            time.sleep(self.queue_delay_s)
        if self.queue_error:
            raise OSError('unsupported queue ioctl')
        return 10 if self.writes else 0

    def flush(self):
        raise AssertionError('telemetry must not flush')

    def reset_output_buffer(self):
        raise AssertionError('telemetry must not reset output')


@pytest.mark.parametrize('enabled', [False, True])
def test_queue_observer_preserves_write_bytes_and_is_optional(enabled):
    port = QueueSerial()
    adapter = J7UartAdapter(port, min_frame_interval_s=0, tx_queue_diagnostics=enabled)
    frame = bytes.fromhex('55aa0501322900e8034c')
    try:
        assert adapter.write(frame) == len(frame)
        assert port.writes == [frame]
        assert port.queue_reads == (2 if enabled else 0)
        metric = adapter.last_write_metric
        if enabled:
            assert metric.queue_before.out_waiting == 0
            assert metric.queue_after.out_waiting == 10
            assert metric.queue_after.in_waiting == 20
            assert metric.configured_timeout_s == .030
        else:
            assert metric.queue_before is None and metric.queue_after is None
        assert port.hardware_write_count == 0
    finally:
        port.close()


def test_queue_error_remains_unknown_and_does_not_mask_write_timeout():
    port = QueueSerial()
    port.queue_error = True
    port.timeout_motor = 1
    adapter = J7UartAdapter(port, min_frame_interval_s=0, tx_queue_diagnostics=True)
    try:
        with pytest.raises(J7AdapterWriteTimeout) as exc:
            adapter.write(bytes.fromhex('55aa0501322900e8034c'))
        assert isinstance(exc.value.__cause__, serial.SerialTimeoutException)
        metric = exc.value.write_metric
        assert metric.queue_before.out_waiting is None
        assert metric.queue_after.out_waiting is None
        assert metric.queue_after.in_waiting == 0
        assert metric.queue_before.error == 'out_waiting:OSError'
        assert len(port.writes) == 1
        assert port.hardware_write_count == 0
    finally:
        port.close()


def test_queue_observation_time_is_separate_from_serial_write_wall():
    port = QueueSerial()
    port.queue_delay_s = .004
    adapter = J7UartAdapter(port, min_frame_interval_s=0, tx_queue_diagnostics=True)
    try:
        adapter.write(bytes.fromhex('55aa0501322900e8034c'))
        metric = adapter.last_write_metric
        assert metric.queue_before.wall_duration_ns >= 4_000_000
        assert metric.queue_after.wall_duration_ns >= 4_000_000
        assert metric.wall_duration_ns < metric.queue_before.wall_duration_ns
    finally:
        port.close()


@pytest.mark.parametrize('timeout_axis', [None, 3])
def test_queue_csv_is_correlated_with_cycle_and_preserves_timeout_snapshot(tmp_path, timeout_axis):
    port = QueueSerial()
    adapter = J7UartAdapter(port, min_frame_interval_s=0, tx_queue_diagnostics=True)
    writer = J7SixAxisServoTransport(adapter, min_frame_interval_s=0)
    writer.configure_servo(speed=50)
    port.timeout_motor = timeout_axis
    rec = J11StallDiagnosticRecorder(output_dir=str(tmp_path))
    rec.start_cycle(10, time.perf_counter_ns())
    rec.mark_transport_entry()
    try:
        if timeout_axis:
            with pytest.raises(J7AdapterWriteTimeout) as exc:
                writer.send_cycle([1000]*6, cycle_id=10)
            audit = exc.value.cycle_audit
            rec.mark_transport_exit(audit)
            rec.record_fault_cycle('motion_write_failure')
        else:
            rec.mark_transport_exit(writer.send_cycle([1000]*6, cycle_id=10))
            rec.finish_cycle()
        rec.dump()
        rows = list(csv.DictReader((tmp_path/'tx_queue_state.csv').open()))
        assert len(rows) == (timeout_axis or 6)
        assert all(r['cycle_seq'] == '10' and float(r['configured_timeout_ms']) == 30 for r in rows)
        assert all(float(r['callback_total_ms']) > 0 for r in rows)
        assert all(float(r['six_axis_tx_phase_ms']) > 0 for r in rows)
        assert rows[-1]['write_error'] == ('SerialTimeoutException' if timeout_axis else '')
        assert rows[-1]['out_waiting_after'] == '10'
        assert port.hardware_write_count == 0
    finally:
        port.close()


@pytest.mark.parametrize('slow_layer', ['os_write', 'select'])
def test_installed_pyserial_uses_logical_budget_not_preemptive_wall_deadline(slow_layer):
    """Execute installed Serial.write with virtual syscall/clock injection; no package edit."""
    now = [0.0]
    calls = []
    port = serial.Serial(port=None, write_timeout=.030)
    port.is_open = True
    port.fd = 111
    port.pipe_abort_write_r = 112
    def write(fd, data):
        calls.append(('write', len(data)))
        now[0] += .0406 if slow_layer == 'os_write' else .001
        return len(data)
    def select(read, write, error, timeout):
        calls.append(('select', timeout))
        now[0] += timeout + .0106
        return [], [], []
    try:
        with patch.object(serialutil.Timeout, 'TIME', new=staticmethod(lambda: now[0])), \
             patch.object(serialposix.os, 'write', new=write), \
             patch.object(serialposix.select, 'select', new=select):
            with pytest.raises(serial.SerialTimeoutException):
                port.write(b'x'*10)
        assert now[0] == pytest.approx(.0406)
        assert calls[0] == ('write', 10)
        if slow_layer == 'os_write':
            assert len(calls) == 1  # Deadline checked only after os.write returns.
        else:
            assert calls[1][1] == pytest.approx(.029)  # Remaining budget, not a fresh 30ms.
    finally:
        port.is_open = False  # Sentinel fds are never closed or used by cleanup.
        port.fd = None
