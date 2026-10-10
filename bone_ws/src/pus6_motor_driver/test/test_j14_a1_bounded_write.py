"""J14-A1 software-only serial TX and existing fault/shutdown path tests."""
import csv
import os
import pty
import time

import pytest
import rclpy
import serial

from pus6_motor_driver.fake_bus import FakeBus, MockServoBus
from pus6_motor_driver.j7_real_uart_adapter import (
    J7UartAdapter, J7StopAdapter, J7AdapterWriteTimeout, J14_UART_WRITE_TIMEOUT_S,
)
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.protocol import parse_frame, pack_write_register_cmd
from pus6_motor_driver.safety import HardwareMode, TARGET_POSITION_ADDR, PAUSE_ADDR
from pus6_motor_driver.stall_diagnostic_recorder import J11StallDiagnosticRecorder


class DeadlineSerial:
    """Protocol-aware fake; delayed writes honor the same finite timeout contract."""
    def __init__(self):
        self.device = FakeBus()
        self.device.positions = {m: 1000 for m in range(1, 7)}
        self.rx = bytearray()
        self.writes = []
        self.write_timeout = J14_UART_WRITE_TIMEOUT_S
        self.timeout_motor = None
        self.delay_s = 0
        self.is_open = True
        self.close_count = 0
        self.hardware_write_count = 0

    @property
    def in_waiting(self):
        return len(self.rx)

    def write(self, frame):
        assert self.is_open
        self.writes.append(frame)
        parsed = parse_frame(frame)
        if parsed.register_addr == TARGET_POSITION_ADDR:
            if parsed.motor_id == self.timeout_motor:
                time.sleep(self.write_timeout)
                raise serial.SerialTimeoutException('Write timeout')
            if self.delay_s:
                time.sleep(self.delay_s)
        self.rx.extend(self.device.request(frame))
        return len(frame)

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def close(self):
        self.is_open = False
        self.close_count += 1

    def reset_input_buffer(self):
        self.rx.clear()

    def reset_output_buffer(self):
        pass


def make_transport():
    port = DeadlineSerial()
    adapter = J7UartAdapter(port, min_frame_interval_s=0)
    stop = J7StopAdapter(adapter)
    writer = J7SixAxisServoTransport(adapter, min_frame_interval_s=0, pause_callback=stop)
    writer.configure_servo(speed=50)
    port.writes.clear()
    return port, adapter, stop, writer


@pytest.mark.parametrize('delay_s', [0, 0.002])
def test_normal_and_delayed_success_preserve_frames_order_and_no_fault(delay_s):
    port, adapter, stop, writer = make_transport()
    port.delay_s = delay_s
    targets = [1000, 1001, 1002, 1003, 1004, 1005]
    try:
        audit = writer.send_cycle(targets, cycle_id=12)
        assert port.writes == [pack_write_register_cmd(m, TARGET_POSITION_ADDR, [t])
                               for m, t in zip(range(1, 7), targets)]
        assert [r.motor_id for r in audit.tx_records] == list(range(1, 7))
        assert all(r.error is None and r.retry_count == 0 and not r.response_wait for r in audit.tx_records)
        assert audit.error_class is None and len(audit.feedbacks) == 6
        assert not writer.stopped and not stop.called
        assert port.hardware_write_count == 0
    finally:
        port.close()


@pytest.mark.parametrize('motor_id', [1, 3, 6])
def test_timeout_aborts_batch_records_axis_cycle_and_pauses_once(motor_id, tmp_path):
    port, adapter, stop, writer = make_transport()
    port.timeout_motor = motor_id
    rec = J11StallDiagnosticRecorder(output_dir=str(tmp_path))
    rec.start_cycle(42, expected_start_ns=time.perf_counter_ns())
    rec.mark_transport_entry()
    try:
        t_start = time.monotonic()
        with pytest.raises(J7AdapterWriteTimeout) as caught:
            writer.send_cycle([1000] * 6, cycle_id=42)
        elapsed = time.monotonic() - t_start
        exc = caught.value
        assert isinstance(exc.__cause__, serial.SerialTimeoutException)
        # Ubuntu userspace timing allows scheduler overshoot, never an infinite wait.
        assert J14_UART_WRITE_TIMEOUT_S <= elapsed < J14_UART_WRITE_TIMEOUT_S + 0.050
        assert exc.write_metric.error == 'SerialTimeoutException'
        assert exc.write_metric.written_bytes == -1  # Unknown partial delivery.
        audit = exc.cycle_audit
        assert audit.cycle_id == 42 and audit.error_motor_id == motor_id
        assert audit.error_class == 'SerialTimeoutException'
        assert audit.error_phase == f'write_id{motor_id}'
        assert audit.tx_records[-1].error == 'SerialTimeoutException'
        assert writer.audit_snapshot().delivery_uncertain and writer.stopped
        motion = [parse_frame(f).motor_id for f in port.writes if parse_frame(f).register_addr == TARGET_POSITION_ADDR]
        assert motion == list(range(1, motor_id + 1))
        pauses = [parse_frame(f).motor_id for f in port.writes if parse_frame(f).register_addr == PAUSE_ADDR]
        assert pauses == list(range(1, 7))
        assert stop.stop_all('again').callback_count == 1
        assert stop.stop_all('again').success
        with pytest.raises(ServoTransportError):
            writer.send_cycle([1000] * 6, cycle_id=43)
        assert len(port.writes) == motor_id + 6
        assert writer.audit_snapshot().motion_retry_count == 0
        rec.mark_transport_exit(audit)
        rec.record_fault_cycle('motion_write_failure', fault_phase='worker_exception')
        rec.dump()
        row = list(csv.DictReader((tmp_path / 'transport_phase_timing.csv').open()))[-1]
        assert row['error_motor_id'] == str(motor_id)
        assert row['error_phase'] == f'write_id{motor_id}'
        assert row['error_class'] == 'SerialTimeoutException'
        assert float(row[f'm{motor_id}_write_wall_ms']) >= 30
        assert row[f'm{motor_id}_ret_bytes'] == '-1'
        for later in range(motor_id + 1, 7):
            assert row[f'm{later}_req_bytes'] == '0'
        assert port.hardware_write_count == 0
    finally:
        port.close()
    assert not port.is_open and port.close_count == 1


def test_actual_pyserial_posix_backpressure_has_finite_timeout():
    """Real PySerial on a local PTY, with no consumer; no physical serial hardware."""
    master, slave = pty.openpty()
    port = serial.Serial(os.ttyname(slave), write_timeout=J14_UART_WRITE_TIMEOUT_S, timeout=0)
    try:
        started = time.monotonic()
        with pytest.raises(serial.SerialTimeoutException):
            port.write(b'x' * 2_000_000)
        elapsed = time.monotonic() - started
        assert J14_UART_WRITE_TIMEOUT_S <= elapsed < J14_UART_WRITE_TIMEOUT_S + 0.050
    finally:
        port.close()
        os.close(master)
        os.close(slave)
    assert not port.is_open


def test_node_timeout_latches_existing_fault_and_closes_serial(tmp_path, monkeypatch):
    if not rclpy.ok():
        rclpy.init()
    node = Pus6MotorDriverNode(hardware_mode=HardwareMode.MOCK, bus=MockServoBus(initial_positions=[1000]*6))
    port, adapter, stop, writer = make_transport()
    node.j7_adapter, node.j7_stop_adapter, node.j7_writer = adapter, stop, writer
    node.j7_feedback.writer = writer
    node._stall_recorder = J11StallDiagnosticRecorder(output_dir=str(tmp_path))
    port.timeout_motor = 3
    try:
        node._execute_servo_cycle()
        assert node._j7_fault_latched
        assert node._j7_fault_reason == 'motion_write_failure'
        assert node.runtime_error is not None
        assert writer.stopped and stop.called and stop.stop_all('again').callback_count == 1
        row = list(csv.DictReader((tmp_path / 'transport_phase_timing.csv').open()))[-1]
        assert row['error_class'] == 'SerialTimeoutException' and row['error_motor_id'] == '3'
        prior_count = len(port.writes)
        node._execute_servo_cycle()
        assert len(port.writes) == prior_count
        assert port.hardware_write_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert not port.is_open and port.close_count == 1


def test_production_serial_open_uses_selected_finite_timeout(monkeypatch):
    """Exercise the production open call using an injected fake serial factory."""
    calls = []
    port = DeadlineSerial()
    def fake_serial(**kwargs):
        calls.append(kwargs)
        port.write_timeout = kwargs['write_timeout']
        return port
    monkeypatch.setattr(serial, 'Serial', fake_serial)
    if not rclpy.ok():
        rclpy.init()
    # READ_ONLY reaches production open while issuing no motion/configuration write.
    node = Pus6MotorDriverNode(hardware_mode=HardwareMode.READ_ONLY)
    try:
        assert len(calls) == 1
        assert calls[0]['write_timeout'] == 0.030
        assert calls[0]['timeout'] == 0.005
        assert port.hardware_write_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert not port.is_open
