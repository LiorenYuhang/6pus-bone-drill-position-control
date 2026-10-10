"""Same-call read attribution, bytes/policy preservation, no physical hardware."""
import csv
import os
import pty
import time

import pytest
import serial

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.protocol import pack_write_register_cmd
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter
from pus6_motor_driver.stall_diagnostic_recorder import J11StallDiagnosticRecorder

FRAME = FakeBus().request(pack_write_register_cmd(1, 0x29, [1000]))


class ReadSerial:
    timeout = .005
    write_timeout = .030
    hardware_write_count = 0
    def __init__(self, delay_s=0, returned=FRAME, fail=False):
        self.delay_s = delay_s
        self.returned = returned
        self.fail = fail
        self.read_sizes = []
        self.availability_calls = 0
    @property
    def in_waiting(self):
        self.availability_calls += 1
        return len(FRAME)
    def read(self, size):
        self.read_sizes.append(size)
        if self.delay_s:
            time.sleep(self.delay_s)
        if self.fail:
            raise OSError('fake read failure')
        return self.returned


@pytest.mark.parametrize('delay,returned', [(0,FRAME),(.002,FRAME),(.005,b''),(.025,FRAME)])
def test_same_call_wall_cpu_context_and_return_values(delay,returned):
    port = ReadSerial(delay,returned)
    observed=[]
    adapter=J7UartAdapter(port,read_observer=observed.append)
    assert adapter._read_chunk() == returned
    event=observed[0]
    assert port.read_sizes == [20] and port.availability_calls == 1
    assert event.in_waiting_before == event.requested_read_bytes == 20
    assert event.returned_read_bytes == len(returned)
    assert event.configured_read_timeout_s == .005 and port.timeout == .005
    assert event.t_read_exit_ns-event.t_read_enter_ns == event.read_wall_ns
    assert event.thread_cpu_after_ns-event.thread_cpu_before_ns == event.read_thread_cpu_ns
    assert event.process_cpu_after_ns-event.process_cpu_before_ns == event.read_process_cpu_ns
    assert event.vol_ctx_after-event.vol_ctx_before == event.read_vol_ctx_delta
    assert event.invol_ctx_after-event.invol_ctx_before == event.read_invol_ctx_delta
    assert event.serial_call_wall_ns <= event.read_wall_ns
    assert event.snapshot_overhead_ns == event.t_snapshot_exit_ns-event.t_snapshot_enter_ns-event.read_wall_ns
    if delay:
        assert event.read_wall_ns >= delay*1e9
        assert event.read_thread_cpu_ns < event.read_wall_ns
        assert event.read_vol_ctx_delta >= 1
    assert port.hardware_write_count == 0


def test_diagnostic_observer_error_does_not_change_data_or_original_failure():
    def observer(event):
        raise RuntimeError('fake diagnostic sink error')
    port=ReadSerial()
    adapter=J7UartAdapter(port,read_observer=observer)
    assert adapter._read_chunk() == FRAME
    assert adapter.last_read_observer_error == 'RuntimeError'
    port.fail=True
    with pytest.raises(OSError,match='fake read failure'):
        adapter._read_chunk()


def test_tier1_is_scalar_summary_and_tier2_only_keeps_slow_reads(tmp_path,capsys):
    rec=J11StallDiagnosticRecorder(output_dir=str(tmp_path))
    rec.start_cycle(77,time.perf_counter_ns())
    port=ReadSerial()
    adapter=J7UartAdapter(port,read_observer=rec.record_single_read)
    adapter._read_chunk()
    assert not rec.rx_slow_read_events
    port.delay_s=.025
    adapter._read_chunk()
    assert len(rec.rx_slow_read_events)==1
    assert rec._curr.rx_read_call_count==2 and rec._curr.rx_bytes==40
    event=rec.rx_slow_read_events[0]
    assert event['scope']=='cycle' and event['cycle_seq']==77
    assert rec._curr.rx_max_single_read_thread_cpu_ms==event['read_thread_cpu_ns']/1e6
    assert rec._curr.rx_max_single_read_process_cpu_ms==event['read_process_cpu_ns']/1e6
    assert 'J14_RX_READ_EVENT ' in capsys.readouterr().out
    rec.finish_cycle();rec.dump()
    tier1=list(csv.DictReader((tmp_path/'cycle_summary_full.csv').open()))[-1]
    assert tier1['rx_read_call_count']=='2' and tier1['rx_bytes']=='40'
    tier2=list(csv.DictReader((tmp_path/'rx_read_events.csv').open()))
    assert len(tier2)==1 and tier2[0]['requested_read_bytes']=='20'
    assert 'thread_cpu_before_ns' not in tier1


def test_parser_and_feedback_frames_unchanged():
    class FiniteSerial(ReadSerial):
        def __init__(self):
            super().__init__()
            self.rx=bytearray(FRAME)
        @property
        def in_waiting(self):
            return len(self.rx)
        def read(self,size):
            self.read_sizes.append(size)
            data=bytes(self.rx[:size]);del self.rx[:size];return data
    port=FiniteSerial();observed=[]
    adapter=J7UartAdapter(port,read_observer=observed.append)
    feedback=adapter.drain_motion_feedbacks()
    assert feedback[1].current_position==1000
    assert port.read_sizes==[20] and len(observed)==1
    assert adapter.last_rx_drain_metrics.bytes_read==20


def test_real_pyserial_pty_5ms_timeout_unchanged_and_observed():
    master,slave=pty.openpty()
    port=serial.Serial(os.ttyname(slave),timeout=.005,write_timeout=.030)
    class PtySerial:
        timeout=.005
        # Test stale availability, forcing actual Serial.read's empty timeout path.
        in_waiting=20
        def read(self,size):
            return port.read(size)
    observed=[]
    adapter=J7UartAdapter(PtySerial(),read_observer=observed.append)
    try:
        assert adapter._read_chunk()==b''
        event=observed[0]
        assert event.returned_read_bytes==0 and event.requested_read_bytes==20
        assert event.read_wall_ns>=5_000_000
        assert event.read_thread_cpu_ns<event.read_wall_ns
        assert port.timeout==.005 and port.write_timeout==.030
    finally:
        port.close();os.close(master);os.close(slave)


def test_slow_counter_snapshot_cannot_masquerade_as_slow_serial_read(monkeypatch):
    import pus6_motor_driver.j7_real_uart_adapter as module
    observed=[];original=module.resource.getrusage;calls=[]
    def sample(who):
        calls.append(who)
        if len(calls)==1:
            time.sleep(.025)
        return original(who)
    monkeypatch.setattr(module.resource,'getrusage',sample)
    adapter=J7UartAdapter(ReadSerial(),read_observer=observed.append)
    assert adapter._read_chunk()==FRAME
    metric=observed[0]
    assert calls==[module.resource.RUSAGE_THREAD]*2
    assert metric.read_wall_ns<5_000_000
    assert metric.snapshot_overhead_ns>=25_000_000


def test_process_cpu_includes_other_thread_work_while_reader_sleeps():
    import threading
    started=threading.Event();stop=threading.Event()
    def worker():
        started.set()
        while not stop.is_set():
            sum(range(1000))
    thread=threading.Thread(target=worker);thread.start();started.wait(timeout=1)
    observed=[]
    try:
        adapter=J7UartAdapter(ReadSerial(.025),read_observer=observed.append)
        assert adapter._read_chunk()==FRAME
        metric=observed[0]
        assert metric.read_process_cpu_ns>metric.read_thread_cpu_ns+1_000_000
    finally:
        stop.set();thread.join(timeout=1)
    assert not thread.is_alive()
