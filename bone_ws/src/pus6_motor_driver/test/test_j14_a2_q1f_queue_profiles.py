"""Exercise real adapter factory with protocol-aware FakeSerial, never a UART."""
import json
import time
from collections import deque

import pytest
import serial

from pus6_motor_driver.j14_process_uart import ServoCore
from pus6_motor_driver.protocol import parse_frame
from test_j14_a1_3_communication_recovery import RecoverySerial
from test_j14_a2_p1_production import config


class ProfileSerial(RecoverySerial):
    def __init__(self):
        super().__init__()
        self.queue_reads = 0

    @property
    def out_waiting(self):
        self.queue_reads += 1
        return 0

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass


def make_core(tmp_path, monkeypatch, mode, **changes):
    port = ProfileSerial()
    def factory(path, baud, **kwargs):
        assert path == '/dev/DO_NOT_OPEN' and baud == 921600
        assert kwargs['write_timeout'] == .030 and kwargs['timeout'] == .005
        return port
    monkeypatch.setattr(serial, 'Serial', factory)
    core = ServoCore(config(tmp_path, hardware_mode='active', telemetry_mode=mode, **changes))
    return core, port


@pytest.mark.parametrize('mode,enabled', [('production', False), ('qualification', False), ('debug', True)])
def test_factory_profiles_preserve_serial_rx_wrapper_and_frames(tmp_path, monkeypatch, mode, enabled):
    core, port = make_core(tmp_path, monkeypatch, mode)
    try:
        assert core.bus.tx_queue_diagnostics is enabled
        core.initialize()
        port.frames.clear(); port.queue_reads = 0
        # Detect any hidden fallback probe, including startup/pause/error handling.
        if not enabled:
            monkeypatch.setattr(core.bus, '_read_tx_queue_state', lambda: pytest.fail('TX queue probe in formal hot path'))
        packet = core.cycle(time.monotonic_ns())
        assert packet['type'] == 'feedback' and packet['feedback_complete']
        assert [parse_frame(f).motor_id for f in port.frames if parse_frame(f).register_addr == 0x29] == list(range(1, 7))
        audit = core.writer.audit_snapshot().cycles[-1]
        assert len(audit.feedbacks) == 6 and audit.rx_drain_metrics.read_calls > 0
        assert core.bus.read_observer == core.rec.record_single_read
        for record in audit.tx_records:
            m = record.serial_write_metric
            assert m.timestamp_end_ns >= m.timestamp_start_ns and m.wall_duration_ns >= 0
            assert m.requested_bytes == m.written_bytes == 10 and m.error is None
            assert record.duration_ns >= m.wall_duration_ns
            assert (m.queue_before is not None) is enabled and (m.queue_after is not None) is enabled
        assert port.queue_reads == (12 if enabled else 0)
        assert isinstance(core.rows, deque) and core.rows.maxlen is not None
        assert core.health['motion_writes'] == 6 and port.hardware_write_count == 0
    finally:
        core.close()
    health = json.loads((tmp_path / 'child_health_summary.json').read_text())
    assert health['closed'] and health['cycles'] == 1


@pytest.mark.parametrize('mode', ['production', 'qualification'])
@pytest.mark.parametrize('axis', [1, 3, 6])
def test_off_profiles_keep_timeout_metric_abort_pause_and_fault_file(tmp_path, monkeypatch, mode, axis):
    core, port = make_core(tmp_path, monkeypatch, mode)
    try:
        core.initialize(); port.frames.clear(); port.queue_reads = 0
        port.timeout_axis = axis
        packet = core.cycle(time.monotonic_ns())
        assert packet['type'] == 'fault' and core.writer.stopped and core.pauses[-1]['success']
        records = packet['failed_audit']['tx_records']; m = records[-1]['serial_write_metric']
        assert m['error'] == 'SerialTimeoutException' and m['configured_timeout_s'] == .030
        assert m['written_bytes'] == -1 and m['wall_duration_ns'] >= 0
        assert m['queue_before'] is None and m['queue_after'] is None
        assert [parse_frame(f).motor_id for f in port.frames if parse_frame(f).register_addr == 0x29] == list(range(1, axis + 1))
        assert [parse_frame(f).motor_id for f in port.frames if parse_frame(f).register_addr == 0x1A] == list(range(1, 7))
        assert core.health['tx_timeouts'] == 1 and port.queue_reads == 0 and port.hardware_write_count == 0
    finally:
        core.close()
    assert json.loads((tmp_path / 'child_fault.jsonl').read_text())['type'] == 'fault'
    writes = [json.loads(l) for l in (tmp_path / 'child_write_metrics.jsonl').read_text().splitlines()]
    assert any(r['error'] == 'SerialTimeoutException' and r['serial_metric']['wall_duration_ns'] >= 0 for r in writes)


@pytest.mark.parametrize('mode', ['production', 'qualification'])
@pytest.mark.parametrize('kind', ['fault', 'overcurrent'])
def test_off_profiles_keep_motor_safety_rx_and_pause(tmp_path, monkeypatch, mode, kind):
    core, port = make_core(tmp_path, monkeypatch, mode)
    try:
        core.initialize(); port.queue_reads = 0
        if kind == 'fault':
            port.device.faults[3] = 1
        else:
            port.device.currents[3] = 1501
        assert core.cycle(time.monotonic_ns())['type'] == 'fault'
        assert core.writer.stopped and core.pauses[-1]['success']
        assert core.health['fault_cycles'] == 1 and port.queue_reads == 0 and port.hardware_write_count == 0
    finally:
        core.close()


def test_historical_nonproduction_factory_retains_queue_diagnostics(tmp_path, monkeypatch):
    core, port = make_core(tmp_path, monkeypatch, 'qualification', production=False)
    try:
        assert core.bus.tx_queue_diagnostics
        core.initialize()
        assert core.cycle(time.monotonic_ns())['type'] == 'feedback'
        assert port.queue_reads > 0 and port.hardware_write_count == 0
    finally:
        core.close()
