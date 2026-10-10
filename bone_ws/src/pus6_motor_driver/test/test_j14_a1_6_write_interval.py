"""Same-call write telemetry; FakeSerial only, never open physical hardware."""
import csv
import json
import resource
import threading
import time
from types import SimpleNamespace

import pytest
import serial

from test_j14_a1_1_timeout_semantics import QueueSerial
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7AdapterWriteTimeout
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.stall_diagnostic_recorder import J11StallDiagnosticRecorder


@pytest.mark.parametrize('behavior', ['normal','delayed','timeout'])
def test_same_call_interval_counters_and_one_unchanged_write(behavior):
    class Port:
        write_timeout=.030;out_waiting=0;in_waiting=0;hardware_write_count=0
        frames=[]
        def write(self, data):
            self.frames.append(data)
            if behavior!='normal':time.sleep(.008)
            if behavior=='timeout':raise serial.SerialTimeoutException('Write timeout')
            return len(data)
    port=Port();port.frames=[]
    uart=J7UartAdapter(port,min_frame_interval_s=0,tx_queue_diagnostics=True)
    frame=b'0123456789'
    if behavior=='timeout':
        with pytest.raises(J7AdapterWriteTimeout):uart.write(frame)
    else:assert uart.write(frame)==10
    m=uart.last_write_metric
    assert m.write_seq==1 and m.native_thread_id==threading.get_native_id()
    assert m.snapshot_enter_ns<=m.timestamp_start_ns<m.timestamp_end_ns<=m.snapshot_exit_ns
    assert m.wall_duration_ns==m.timestamp_end_ns-m.timestamp_start_ns
    assert m.process_cpu_duration_ns>=0 and m.cpu_duration_ns>=0
    assert m.voluntary_ctx_delta>=0 and m.involuntary_ctx_delta>=0
    if behavior!='normal':assert m.wall_duration_ns>=8_000_000 and m.voluntary_ctx_delta>0
    assert m.snapshot_overhead_ns==(m.snapshot_exit_ns-m.snapshot_enter_ns)-m.wall_duration_ns
    assert m.clock_sync_monotonic_before_ns<=m.clock_sync_monotonic_after_ns
    assert m.clock_sync_epoch_ns>0
    assert m.written_bytes==(-1 if behavior=='timeout' else 10)
    assert port.frames==[frame] and port.hardware_write_count==0


def test_context_switches_use_rusage_thread_with_paired_snapshots(monkeypatch):
    calls=[]
    def usage(who):
        calls.append(who)
        return SimpleNamespace(ru_nvcsw=10+3*len(calls),ru_nivcsw=20+2*len(calls))
    monkeypatch.setattr(resource,'getrusage',usage)
    port=QueueSerial();uart=J7UartAdapter(port,min_frame_interval_s=0,tx_queue_diagnostics=True)
    uart.write(bytes.fromhex('55aa0501322900e8034c'))
    m=uart.last_write_metric
    assert calls==[resource.RUSAGE_THREAD,resource.RUSAGE_THREAD]
    assert m.voluntary_ctx_delta==3 and m.involuntary_ctx_delta==2
    assert port.hardware_write_count==0


def test_disabled_diagnostics_leave_extra_counters_unknown(monkeypatch):
    monkeypatch.setattr(resource,'getrusage',lambda *_:pytest.fail('disabled counter collection'))
    port=QueueSerial();uart=J7UartAdapter(port,min_frame_interval_s=0,tx_queue_diagnostics=False)
    uart.write(bytes.fromhex('55aa0501322900e8034c'))
    m=uart.last_write_metric
    assert m.process_cpu_duration_ns is None and m.native_thread_id is None
    assert m.voluntary_ctx_delta is None and m.queue_before is None


def test_failed_csv_and_event_identity_match_exact_metric(tmp_path):
    port=QueueSerial();uart=J7UartAdapter(port,min_frame_interval_s=0,tx_queue_diagnostics=True)
    writer=J7SixAxisServoTransport(uart,min_frame_interval_s=0)
    writer.configure_servo(speed=50);port.timeout_motor=3
    rec=J11StallDiagnosticRecorder(output_dir=str(tmp_path));rec.start_cycle(17,time.perf_counter_ns());rec.mark_transport_entry()
    with pytest.raises(J7AdapterWriteTimeout) as caught:writer.send_cycle([1000]*6,cycle_id=17)
    metric=caught.value.write_metric;rec.mark_transport_exit(caught.value.cycle_audit);rec.record_fault_cycle('motion_write_failure');rec.dump()
    row=list(csv.DictReader((tmp_path/'tx_queue_state.csv').open()))[-1]
    event=json.loads((tmp_path/'tx_write_events.jsonl').read_text())
    assert row['cycle_seq']=='17' and row['motor_id']=='3'
    assert int(row['write_seq'])==event['write_seq']==metric.write_seq
    assert int(row['native_thread_id'])==event['native_thread_id']==metric.native_thread_id
    assert int(row['process_cpu_duration_ns'])==event['process_cpu_duration_ns']==metric.process_cpu_duration_ns
    assert int(row['written_bytes'])==event['written_bytes']==-1
    assert port.hardware_write_count==0
