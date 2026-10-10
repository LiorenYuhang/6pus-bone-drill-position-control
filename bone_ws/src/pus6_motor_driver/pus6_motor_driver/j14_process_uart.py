"""A1.9 opt-in UART process experiment. No ROS imports; no automatic recovery."""
from __future__ import annotations
import argparse
from collections import deque
from dataclasses import asdict
import json
import os
from pathlib import Path
import resource
import select
import signal
import socket
import subprocess
import sys
import threading
import time

from .fake_bus import MockServoBus
from .j7_real_uart_adapter import J7UartAdapter, J7StopAdapter, J14_UART_WRITE_TIMEOUT_S
from .j7_six_axis_servo_transport import J7SixAxisServoTransport, ServoRxPolicy
from .j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from .stall_diagnostic_recorder import J11StallDiagnosticRecorder
from .safety import MOTOR_IDS, validate_target_vector

PERIOD_NS = 20_000_000
LEASE_NS = 1_000_000_000
PACKET_MAX = 16384
DRAIN_MAX = 64


def socket_pair():
    pair = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    for s in pair:
        s.setblocking(False)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
    return pair


def send_packet(sock, row):
    payload = json.dumps(row, separators=(',', ':')).encode()
    if len(payload) > PACKET_MAX:
        raise ValueError('IPC packet too large')
    try:
        if sock.send(payload) != len(payload):
            raise RuntimeError('partial seqpacket send')
        return True
    except BlockingIOError:
        return False


def receive_packets(sock):
    rows = []
    for _ in range(DRAIN_MAX):
        try:
            data, _, flags, _ = sock.recvmsg(PACKET_MAX)
        except BlockingIOError:
            return rows, False
        if not data:
            if rows:
                return rows, False
            raise EOFError('IPC peer exited')
        if flags & socket.MSG_TRUNC:
            raise ValueError('truncated IPC packet')
        row = json.loads(data)
        if not isinstance(row, dict):
            raise ValueError('IPC object required')
        rows.append(row)
    # A flooded receiver does bounded work; no old vector may be dispatched.
    return rows, True


def append_rows(path, rows):
    with Path(path).open('a') as f:
        for row in rows:
            f.write(json.dumps(row) + '\n')


class CommandState:
    """A single whole-vector reference and separate parent lease."""
    def __init__(self, epoch, initial, target_timeout_s=.2, jump_limit=80):
        self.epoch = epoch
        self.seq = -1
        self.targets = list(initial)
        self.command_ns = 0
        self.receive_ns = 0
        self.lease_ns = time.monotonic_ns()
        self.timeout_ns = int(target_timeout_s * 1e9)
        self.jump_limit = jump_limit
        self.consumed = True
        self.stop_reason = None
        self.rejected = 0

    def ingest(self, rows, now_ns, anchor):
        newest = None
        for row in rows:
            if row.get('epoch') != self.epoch:
                raise ValueError('IPC epoch mismatch')
            if row.get('type') == 'stop':
                self.stop_reason = row.get('reason', 'parent_stop')
                continue
            if row.get('type') not in ('state', 'start') or row.get('mode') != 'static' or row.get('allowed') is not True:
                raise ValueError('IPC mode/permission invalid')
            lease = row.get('lease_ns')
            if not isinstance(lease, int) or lease > now_ns:
                raise ValueError('IPC lease invalid')
            self.lease_ns = max(self.lease_ns, lease)
            cmd = row.get('command')
            if cmd is None:
                continue
            targets = cmd.get('targets')
            if not isinstance(targets, list) or len(targets) != 6 or any(type(v) is not int for v in targets):
                raise ValueError('whole six integer targets required')
            validate_target_vector(targets)
            seq, stamp = cmd.get('seq'), cmd.get('publish_ns')
            if type(seq) is not int or type(stamp) is not int or stamp > now_ns or stamp <= 0:
                raise ValueError('IPC sequence/timestamp invalid')
            if seq > self.seq and (newest is None or seq > newest['seq']):
                newest = cmd
        if self.stop_reason or newest is None:
            return
        stale = self.command_ns > 0 and now_ns - self.command_ns > self.timeout_ns
        if stale and max(abs(a-b) for a,b in zip(newest['targets'], anchor)) > self.jump_limit:
            self.rejected += 1
            return
        self.seq = newest['seq']
        self.targets = list(newest['targets'])
        self.command_ns = newest['publish_ns']
        self.receive_ns = now_ns
        self.consumed = False

    def candidate(self, anchor, warning):
        if warning or self.consumed:
            return list(anchor)
        self.consumed = True
        return list(self.targets)


def dispatch_slew(candidate, anchor, last_write_ns, now_ns):
    """Same driver final-dispatch vector rule; upstream ROS shaper is unchanged."""
    dt = max(.010, (now_ns-last_write_ns)/1e9) if last_write_ns else .020
    allowance = min(20, max(5, int(200.0*dt)))
    gaps = [c-a for c,a in zip(candidate, anchor)]
    gap = max(map(abs, gaps))
    if gap <= allowance:
        return list(candidate)
    alpha = allowance / gap
    result = [int(round(a+alpha*g)) for a,g in zip(anchor, gaps)]
    return [a+min(20,max(-20,d-a)) for a,d in zip(anchor,result)]


class ServoCore:
    """Single-thread UART/safety owner shared by thread/process mock A/B."""
    def __init__(self, config, bus=None):
        self.config = config
        self.out = Path(config['output_dir'])
        self.out.mkdir(parents=True, exist_ok=True)
        self.rec = J11StallDiagnosticRecorder(output_dir=str(self.out), enabled=True)
        self.lifecycle = []
        self.pauses = []
        self.warnings = deque(maxlen=64)
        self.rows = []  # timing scalars only; trial is bounded to 60 seconds
        self.write_rows = []
        self.production = bool(config.get('production'))
        self.telemetry_mode = config.get('telemetry_mode','debug')
        self.health = dict(cycles=0,motion_writes=0,tx_timeouts=0,fault_cycles=0,
            max_serial_write_ns=0,max_rx_drain_ns=0,max_wake_lateness_ns=0,
            wake_watchlist_count=0)
        if self.production:
            capacity = 300 if self.telemetry_mode=='production' else config['telemetry_capacity_cycles']
            self.rows=deque(maxlen=capacity); self.write_rows=deque(maxlen=capacity*6)
            self.rec.tier1_summary=deque(maxlen=capacity)
            self.rec.all_stall_events=deque(maxlen=capacity)
            self.rec.rx_slow_read_events=deque(maxlen=capacity)
        self.serial = None
        if bus is not None:
            self.bus = bus
        elif config['hardware_mode'] == 'mock':
            self.bus = MockServoBus(initial_positions=[1000]*6)
        elif config['hardware_mode'] == 'active':
            import serial
            self.serial = serial.Serial(config['port'], config['baudrate'],
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=.005, write_timeout=J14_UART_WRITE_TIMEOUT_S)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.bus = J7UartAdapter(self.serial, baudrate=config['baudrate'], timeout_s=.010,
                min_frame_interval_s=.0001,
                tx_queue_diagnostics=(not self.production or self.telemetry_mode == 'debug'),
                read_observer=self.rec.record_single_read)
        else:
            raise ValueError('trial permits mock or active only')
        self.stop_adapter = J7StopAdapter(self.bus, audit_observer=self._pause_audit)
        self.writer = J7SixAxisServoTransport(self.bus, cycle_rate_hz=50,
            rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX, min_frame_interval_s=.0006,
            rx_drain_budget_s=.004, pause_callback=self.stop_adapter,
            recoverable_write_timeouts=True)
        self.feedback = J7ServoFeedbackScheduler(self.writer, full_scan_period_s=1e9,
            feedback_budget_s=.015, servo_cycle_period_s=.020, timeout_escalation_misses=4,
            max_feedback_age_s=1., warning_feedback_age_s=.2833, hard_stop_feedback_age_s=1.,
            feedback_loss_policy='age_based', warn_callback=self.warnings.append)
        if self.production:
            # Bound instance-owned evidence, not the frozen transport implementations.
            self.writer._cycles=deque(maxlen=300)
            self.writer._rx_records=deque(maxlen=300)
            self.feedback._cycles=deque(maxlen=300)
        self.positions = [1000]*6
        self.currents = [0]*6
        self.faults = [0]*6
        self.rx_ns = [0]*6
        self.last_command = None
        self.last_write_ns = 0
        self.cycle_seq = 0
        self.fault = None
        self.last_packet = None
        self.closed = False
        self.servo_start_ns=0
        self.motion_stop_ns=0

    def _pause_audit(self, audit):
        # Existing one-shot audit persists after all six pause requests.
        row = asdict(audit)
        self.pauses.append(row)
        append_rows(self.out/'stop_pause_audit.jsonl', [row])
        self.lifecycle.append(dict(event='pause_complete', ns=time.monotonic_ns(), success=row['success']))

    def update(self, audit):
        for s in audit.samples:
            if s.position is not None:
                i=s.motor_id-1
                self.positions[i]=s.position
                self.currents[i]=s.current_mA or 0
                self.faults[i]=s.fault or 0
                self.rx_ns[i]=s.timestamp_ns

    def initialize(self):
        self.lifecycle.append(dict(event='serial_owner', ns=time.monotonic_ns(), pid=os.getpid(),
                                   hardware_mode=self.config['hardware_mode']))
        scan = self.feedback.full_scan(servo_cycle_id=0, servo_cycle_timestamp_ns=time.monotonic_ns())
        self.update(scan)
        if any(s.timeout or s.position is None for s in scan.samples) or len(scan.samples)!=6:
            raise RuntimeError('incomplete startup status')
        if self.feedback.stopped:
            raise RuntimeError('unsafe startup status')
        self.writer.configure_servo(speed=self.config.get('servo_speed',50))
        self.command = CommandState(self.config['epoch'], self.positions,
            self.config.get('target_timeout_sec',.2),self.config.get('reconnect_max_jump_steps',80))
        return dict(type='ready', epoch=self.config['epoch'], pid=os.getpid(),
                    parent_pid=os.getppid(),positions=self.positions, currents=self.currents, faults=self.faults, rx_ns=self.rx_ns,
                    modules_without_ros=not any(x=='rclpy' or x.startswith(('rclpy.','pus6_msgs','sensor_msgs')) for x in sys.modules),
                    serial_write_timeout_s=.030, serial_read_timeout_s=.005)

    def stop(self, reason):
        if not self.fault:
            self.fault = reason
        if not self.motion_stop_ns:
            self.motion_stop_ns = time.monotonic_ns()
        self.writer.stop(reason)  # pause before any fault persistence / IPC

    def cycle(self, expected_ns, hold_for_ipc_backlog=False):
        start=time.monotonic_ns(); tc=time.thread_time_ns(); pc=time.process_time_ns()
        ru=resource.getrusage(resource.RUSAGE_THREAD)
        self.rec.start_cycle(self.cycle_seq,expected_start_ns=expected_ns)
        audit=None
        try:
            anchor=self.last_command or self.positions
            candidate=self.command.candidate(anchor,hold_for_ipc_backlog or self.feedback.is_feedback_stale_warning(start))
            self.rec.mark_target_lock_done()
            dispatched=dispatch_slew(candidate, self.last_command or candidate,self.last_write_ns,start)
            self.rec.mark_slew_done()
            validate_target_vector(dispatched, limits=None if self.config.get('maintenance_mode',True) else {mid:(200,1800) for mid in MOTOR_IDS})
            if self.last_command and max(abs(a-b) for a,b in zip(dispatched,self.last_command))>20:
                raise RuntimeError('inter-frame delta >20')
            if any(self.faults):
                raise RuntimeError('cached motor fault')
            self.rec.mark_preflight_done(); self.rec.mark_transport_entry()
            audit=self.writer.send_cycle(dispatched,cycle_id=self.cycle_seq,target_timestamp_ns=start)
            self.last_command=list(dispatched); self.last_write_ns=time.monotonic_ns()
            self.rec.mark_transport_exit(audit); self.rec.mark_ingest_start()
            fb=self.feedback.ingest_cycle_feedback(audit.feedbacks,servo_cycle_id=self.cycle_seq,
                                                   servo_cycle_timestamp_ns=start)
            self.rec.mark_ingest_done(fb.scheduler_gap_ns/1e6)
            if self.feedback.stopped or self.writer.stopped:
                raise RuntimeError(self.feedback.audit_snapshot().stop_reason or 'transport stopped')
            self.update(fb); self.rec.mark_unpack_done()
            end=time.monotonic_ns()
            packet=dict(type='feedback',epoch=self.config['epoch'],cycle_seq=self.cycle_seq,
                command_seq=self.command.seq, command_publish_ns=self.command.command_ns,
                command_receive_ns=self.command.receive_ns,cycle_start_ns=start,cycle_end_ns=end,
                feedback_ready_ns=end, positions=list(self.positions),currents=list(self.currents),
                faults=list(self.faults),rx_ns=list(self.rx_ns), targets=dispatched,
                tx_start_ns=[r.tx_start_ns for r in audit.tx_records],
                tx_end_ns=[r.tx_end_ns for r in audit.tx_records],
                tx_wall_ns=[r.duration_ns for r in audit.tx_records],
                rx_drain_ns=audit.motion_drain_wall_ns,
                feedback_age_ns=[end-n for n in self.rx_ns],
                feedback_complete=len(audit.feedbacks)==6,
                scheduler_gap_ns=fb.scheduler_gap_ns, communication_fault=None,
                rejected_commands=self.command.rejected)
            self.last_packet=packet
            self.rec.mark_telemetry_done()
            self.rec.finish_cycle(feedback_age_max_ms=max((s.feedback_age_ns/1e6 for s in fb.samples),default=0))
        except Exception as exc:
            self.stop(f'{type(exc).__name__}: {exc}')
            failed=getattr(exc,'cycle_audit',None)
            if failed is not None:
                self.rec.mark_transport_exit(failed)
            self.rec.record_fault_cycle(self.fault,fault_phase='child_local_fault')
            self.rec.finish_cycle(fault_reason=self.fault)
            packet=dict(type='fault',epoch=self.config['epoch'],cycle_seq=self.cycle_seq,
                        communication_fault=self.fault, feedback_ready_ns=time.monotonic_ns(),
                        failed_audit=asdict(failed) if failed else None)
            self.last_packet=packet
        # Keep all strict write scalar measurements, not just the legacy 6-second ring.
        if locals().get('failed') is not None:
            audit = failed
        if audit is not None:
            for r in audit.tx_records:
                m=r.serial_write_metric
                self.health['motion_writes']+=1
                self.health['tx_timeouts']+=int(r.error=='SerialTimeoutException')
                if m is not None:
                    self.health['max_serial_write_ns']=max(self.health['max_serial_write_ns'],m.wall_duration_ns)
                if self.production and self.telemetry_mode=='production' and not r.error and (m is None or m.wall_duration_ns<1_000_000):
                    continue  # recent full TX evidence remains in the existing bounded recorder
                self.write_rows.append(dict(cycle_seq=r.cycle_id,motor_id=r.motor_id,
                    wrapper_wall_ns=r.duration_ns,wrapper_thread_cpu_ns=r.cpu_duration_ns,
                    error=r.error,serial_metric=asdict(m) if m is not None else None))
        self.health['cycles']+=1
        self.health['fault_cycles']+=int(self.fault is not None)
        self.health['max_wake_lateness_ns']=max(self.health['max_wake_lateness_ns'],start-expected_ns)
        self.health['wake_watchlist_count']+=int(start-expected_ns>30_000_000)
        if audit is not None:self.health['max_rx_drain_ns']=max(self.health['max_rx_drain_ns'],audit.motion_drain_wall_ns)
        end=time.monotonic_ns(); after=resource.getrusage(resource.RUSAGE_THREAD)
        self.rows.append(dict(cycle_seq=self.cycle_seq, expected_ns=expected_ns,start_ns=start,end_ns=end,
            wake_lateness_ns=start-expected_ns, thread_cpu_ns=time.thread_time_ns()-tc,
            process_cpu_ns=time.process_time_ns()-pc,voluntary_cs=after.ru_nvcsw-ru.ru_nvcsw,
            involuntary_cs=after.ru_nivcsw-ru.ru_nivcsw,
            command_seq=self.command.seq, command_publish_ns=self.command.command_ns,
            command_receive_ns=self.command.receive_ns, fault=self.fault))
        self.cycle_seq+=1
        return packet

    def close(self, reason='shutdown'):
        if self.closed:
            return
        try:
            self.writer.stop(reason)
        finally:
            if self.serial is not None:
                self.serial.close()
            else:
                getattr(self.bus, 'serial', self.bus).close()
            self.closed=True
            self.lifecycle.append(dict(event='serial_closed',ns=time.monotonic_ns()))
            self.rec.dump()
            if self.production:
                self.health.update(telemetry_mode=self.telemetry_mode,retained_cycles=len(self.rows),
                    retained_write_records=len(self.write_rows),closed=True)
                (self.out/'child_health_summary.json').write_text(json.dumps(self.health,indent=2)+'\n')
            append_rows(self.out/'child_cycles.jsonl',self.rows)
            append_rows(self.out/'child_write_metrics.jsonl',self.write_rows)
            append_rows(self.out/'child_warnings.jsonl',list(self.warnings))
            append_rows(self.out/'child_lifecycle.jsonl',self.lifecycle)
            if self.last_packet and self.last_packet['type']=='fault':
                append_rows(self.out/'child_fault.jsonl',[self.last_packet])
            (self.out/'child_final.json').write_text(json.dumps(dict(pid=os.getpid(),cycles=self.cycle_seq,
                fault=self.fault,closed=self.closed,servo_start_ns=self.servo_start_ns,
                motion_stop_ns=self.motion_stop_ns, hardware_write_count=0 if self.config['hardware_mode']=='mock' else None,
                pause_success=bool(self.pauses) and self.pauses[-1]['success']),indent=2)+'\n')


def child_run(sock, config, bus=None, fb_sock=None):
    core=None; signal_stop=[False]
    out_fb = fb_sock if fb_sock is not None else sock
    # Only actual child main thread installs handlers (thread A/B calls this too).
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT,signal.SIGTERM):
            signal.signal(sig,lambda *_: signal_stop.__setitem__(0,True))
    try:
        core=ServoCore(config,bus)
        ready=core.initialize()
        t_child_ready_ns = time.monotonic_ns()
        ready['t_child_ready_ns'] = t_child_ready_ns
        ready['type'] = 'child_ready'
        if not send_packet(sock,ready):
            raise RuntimeError('startup IPC full')
        if out_fb is not sock:
            send_packet(out_fb, ready)

            # J14-A1.13 Coordinated Three-Process Handshake
            parent_ready = False
            bridge_ready = False
            t_bridge_ready_ns = None
            t_parent_ready_ns = None
            handshake_timeout_s = config.get('handshake_timeout_s', 5.0)
            handshake_deadline = time.monotonic() + handshake_timeout_s

            while not (parent_ready and bridge_ready) and not signal_stop[0]:
                remaining = handshake_deadline - time.monotonic()
                if remaining <= 0:
                    core.stop('startup_handshake_timeout')
                    fault_pkt = dict(
                        type='fault',
                        epoch=config['epoch'],
                        communication_fault='startup_handshake_timeout',
                    )
                    send_packet(sock, fault_pkt)
                    send_packet(out_fb, fault_pkt)
                    return

                rlist, _, _ = select.select([sock, out_fb], [], [], min(0.020, max(0.001, remaining)))
                if sock in rlist:
                    rows, _ = receive_packets(sock)
                    for row in rows:
                        if row.get('type') in ('start', 'state') and row.get('allowed'):
                            parent_ready = True
                            t_parent_ready_ns = row.get('t_parent_ready_ns') or time.monotonic_ns()
                            core.command.ingest([row], time.monotonic_ns(), core.last_command or core.positions)
                        elif row.get('type') == 'stop':
                            core.stop(row.get('reason', 'parent_stop'))
                            return
                        elif row.get('type') == 'fault':
                            core.stop(row.get('communication_fault', 'parent_fault'))
                            return
                if out_fb in rlist:
                    rows, _ = receive_packets(out_fb)
                    for row in rows:
                        if row.get('type') == 'bridge_ready':
                            bridge_ready = True
                            t_bridge_ready_ns = row.get('t_bridge_ready_ns') or time.monotonic_ns()
                            send_packet(sock, dict(
                                type='bridge_ready',
                                epoch=config['epoch'],
                                pid=row.get('pid'),
                                t_bridge_ready_ns=t_bridge_ready_ns,
                            ))
                        elif row.get('type') == 'fault':
                            core.stop(row.get('communication_fault', 'bridge_fault'))
                            return

            if signal_stop[0]:
                core.stop('signal_stop')
                return

            servo_start_ns = time.monotonic_ns()
            ready['servo_start_ns'] = servo_start_ns
            core.servo_start_ns = servo_start_ns
            start_packet = dict(
                type='servo_start',
                epoch=config['epoch'],
                servo_start_ns=servo_start_ns,
                t_child_ready_ns=t_child_ready_ns,
                t_bridge_ready_ns=t_bridge_ready_ns,
                t_parent_ready_ns=t_parent_ready_ns,
            )
            send_packet(sock, start_packet)
            send_packet(out_fb, start_packet)
            append_rows(core.out / 'child_ready.jsonl', [ready, start_packet])
            deadline = servo_start_ns
        else:
            deadline=time.monotonic_ns()
            ready['servo_start_ns']=deadline
            core.servo_start_ns=deadline
            append_rows(core.out/'child_ready.jsonl',[ready])

        duration=config.get('max_servo_duration_s',60)
        if config.get('production'):
            if duration<0:raise ValueError('production duration must be nonnegative')
            trial_end_ns=deadline+int(duration*1e9) if duration else None
        else:
            if not 0<duration<=60: raise ValueError('A1.9 duration must be <=60 s')
            trial_end_ns=deadline+int(duration*1e9)
        flooded=False
        while not signal_stop[0]:
            rows,flooded=receive_packets(sock)
            core.command.ingest(rows,time.monotonic_ns(),core.last_command or core.positions)
            if core.command.stop_reason:
                core.stop(core.command.stop_reason); break
            if time.monotonic_ns()-core.command.lease_ns>LEASE_NS:
                core.stop('parent_lease_expired'); break
            now=time.monotonic_ns()
            if trial_end_ns is not None and now>=trial_end_ns:
                core.stop('window_complete'); break
            if now<deadline:
                select.select([sock],[],[],(deadline-now)/1e9)
                continue
            # Bounded receive work cannot starve UART. Hold last safe vector if
            # a backlog remains; admit its newest vector only after drain completes.
            packet=core.cycle(deadline,hold_for_ipc_backlog=flooded)
            try:
                send_packet(out_fb,packet)  # bounded, no waiting for feedback consumer
                if out_fb is not sock and packet.get('type') == 'fault':
                    send_packet(sock, packet)
            except (BrokenPipeError,ConnectionResetError,EOFError):
                core.stop('feedback_ipc_closed' if out_fb is not sock else 'parent_ipc_closed'); break
            if core.fault:
                break
            deadline+=PERIOD_NS
            now=time.monotonic_ns()
            if now-deadline>PERIOD_NS:
                deadline=now
    except (EOFError,BrokenPipeError,ConnectionResetError) as exc:
        if core: core.stop('parent_ipc_closed')
    except Exception as exc:
        if core:
            core.stop(f'child_exception: {type(exc).__name__}: {exc}')
        else:
            Path(config['output_dir']).mkdir(parents=True,exist_ok=True)
            append_rows(Path(config['output_dir'])/'child_start_failure.jsonl',[dict(error=repr(exc))])
    finally:
        if core: core.close()
        sock.close()
        if out_fb is not sock:
            out_fb.close()
        if core:
            append_rows(core.out/'child_lifecycle.jsonl',[dict(event='child_exit',ns=time.monotonic_ns())])


class ProcessClient:
    """ROS-independent parent endpoint; no serial factory or fallback UART path."""
    max_feedback_age_ns=LEASE_NS
    def __init__(self, config):
        self.config=config
        self.use_feedback_bridge = bool(
            config.get('feedback_bridge') or not config.get('production') and os.environ.get('PUS6_J14_FEEDBACK_BRIDGE') == '1'
        )
        self.sock,child_sock=socket_pair()
        self.bridge_proc = None
        self.bridge_stopped = False

        # Section XIV: Capture actual socket buffer values without altering them
        buffer_audit = {
            'command_channel': {
                'parent_so_sndbuf': self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'parent_so_rcvbuf': self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                'child_so_sndbuf': child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'child_so_rcvbuf': child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
            }
        }

        if self.use_feedback_bridge:
            fb_child_sock, fb_bridge_sock = socket_pair()
            buffer_audit['feedback_channel'] = {
                'child_so_sndbuf': fb_child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'child_so_rcvbuf': fb_child_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                'bridge_so_sndbuf': fb_bridge_sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                'bridge_so_rcvbuf': fb_bridge_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
            }
            out_dir = Path(config['output_dir'])
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / 'socket_buffer_audit.json').write_text(json.dumps(buffer_audit, indent=2) + '\n')

            argv_child = [
                sys.executable, str(Path(__file__).with_name('j14_uart_child_main.py')),
                '--fd-cmd', str(child_sock.fileno()),
                '--fd-fb', str(fb_child_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.proc = subprocess.Popen(argv_child, pass_fds=(child_sock.fileno(), fb_child_sock.fileno()), start_new_session=True)
            finally:
                child_sock.close()
                fb_child_sock.close()

            argv_bridge = [
                sys.executable, str(Path(__file__).with_name('j14_feedback_bridge_main.py')),
                '--fd', str(fb_bridge_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.bridge_proc = subprocess.Popen(argv_bridge, pass_fds=(fb_bridge_sock.fileno(),), start_new_session=True)
            finally:
                fb_bridge_sock.close()
        else:
            out_dir = Path(config['output_dir'])
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / 'socket_buffer_audit.json').write_text(json.dumps(buffer_audit, indent=2) + '\n')

            argv = [
                sys.executable, str(Path(__file__).with_name('j14_uart_child_main.py')),
                '--fd', str(child_sock.fileno()),
                '--config', json.dumps(config)
            ]
            try:
                self.proc = subprocess.Popen(argv, pass_fds=(child_sock.fileno(),), start_new_session=True)
            finally:
                child_sock.close()

        self.lock=threading.Lock()
        self.latest=None; self.seq=0; self.terminal=None
        self.stopped=False; self.stop_reason=None
        self.rows=deque(maxlen=config['telemetry_capacity_cycles']) if config.get('production') else []; self.last_lease_sent_ns=0; self.closed=False
        self.ready=None
        self.child_ready=None
        self.bridge_ready=None
        self.servo_started=None
        handshake_timeout_s = config.get('handshake_timeout_s', 5.0)
        until = time.monotonic() + handshake_timeout_s
        while time.monotonic() < until:
            if not self.use_feedback_bridge:
                self.send_state()
                for row in self.receive():
                    if row.get('type') in ('ready', 'child_ready'): self.ready = row
                    elif row.get('type') == 'fault': self.stopped = True; self.stop_reason = row['communication_fault']
                if self.ready: return
                if self.stopped: break
            else:
                for row in self.receive():
                    if row.get('type') in ('ready', 'child_ready'):
                        self.child_ready = row
                        self.ready = row
                    elif row.get('type') == 'bridge_ready':
                        self.bridge_ready = row
                    elif row.get('type') == 'servo_start':
                        self.servo_started = row
                    elif row.get('type') == 'fault':
                        self.stopped = True
                        self.stop_reason = row.get('communication_fault')
                if self.stopped:
                    break
                if self.child_ready and self.bridge_ready and not self.servo_started:
                    now = time.monotonic_ns()
                    send_packet(self.sock, dict(
                        type='start',
                        epoch=self.config['epoch'],
                        mode='static',
                        allowed=True,
                        lease_ns=now,
                        t_parent_ready_ns=now,
                        command=None,
                    ))
                if self.servo_started:
                    append_rows(Path(self.config['output_dir']) / 'parent_lifecycle.jsonl', [
                        dict(
                            event='startup_handshake_complete',
                            ns=time.monotonic_ns(),
                            child_ready_ns=self.child_ready.get('t_child_ready_ns'),
                            bridge_ready_ns=self.bridge_ready.get('t_bridge_ready_ns'),
                            servo_start_ns=self.servo_started.get('servo_start_ns'),
                        )
                    ])
                    return
            time.sleep(0.005)
        self.close()
        raise RuntimeError(f'UART child / bridge startup failed: {self.stop_reason or "handshake_timeout"}')

    def update(self, targets, publish_ns=None, seq=None):
        if len(targets)!=6 or any(type(v) is not int for v in targets):
            raise ValueError('whole six integer target required')
        with self.lock:
            if self.terminal or self.stopped: return
            self.latest=dict(seq=self.seq if seq is None else seq,targets=list(targets),
                             publish_ns=publish_ns or time.monotonic_ns())
            self.seq=max(self.seq+1,(seq+1) if seq is not None else 0)

    def send_state(self):
        now=time.monotonic_ns()
        with self.lock:
            latest=self.latest
            if self.terminal:
                row=dict(type='stop',epoch=self.config['epoch'],reason=self.terminal)
            elif latest is not None or now-self.last_lease_sent_ns>=100_000_000:
                row=dict(type='state',epoch=self.config['epoch'],mode='static',allowed=True,
                         lease_ns=now,command=latest)
            else: return
            try:
                sent=send_packet(self.sock,row)
            except (BrokenPipeError,ConnectionResetError,OSError):
                self.stopped=True; self.stop_reason='child_ipc_closed'; return
            if sent:
                if self.latest is latest: self.latest=None
                self.last_lease_sent_ns=now
                self.rows.append(dict(event='parent_send',ns=now,process_cpu_ns=time.process_time_ns(),
                                      thread_cpu_ns=time.thread_time_ns(),
                                      seq=latest['seq'] if latest else None))

    def receive(self):
        if self.closed: return []
        try:
            rows,_=receive_packets(self.sock)
        except (EOFError,ConnectionResetError,OSError):
            self.stopped=True; self.stop_reason=self.stop_reason or 'child_exit_or_EOF'
            final_path=Path(self.config['output_dir'])/'child_final.json'
            if final_path.exists():
                final=json.loads(final_path.read_text())
                self.stop_reason=final.get('fault') or self.stop_reason
            return []
        for row in rows:
            now=time.monotonic_ns(); row['parent_receive_ns']=now
            cpu=time.process_time_ns()
            tcpu=time.thread_time_ns()
            self.rows.append(dict(event='parent_receive',ns=now,process_cpu_ns=cpu,
                thread_cpu_ns=tcpu,
                cycle_seq=row.get('cycle_seq'),seq=row.get('command_seq'),type=row.get('type'),
                child_ready_ns=row.get('feedback_ready_ns'),command_publish_ns=row.get('command_publish_ns'),
                command_receive_ns=row.get('command_receive_ns')))
            if row.get('type')=='fault':
                self.stopped=True; self.stop_reason=row['communication_fault']
        if self.proc.poll() is not None:
            self.stopped=True; self.stop_reason=self.stop_reason or 'child_exited'
        bridge = getattr(self, 'bridge_proc', None)
        if bridge is not None and bridge.poll() is not None:
            self.bridge_stopped = True
        return rows

    def stop(self, reason):
        with self.lock: self.terminal=str(reason)
        self.send_state()

    def close(self):
        if self.closed: return
        self.stop('parent_shutdown')
        # EOF is independent of a full parent->child buffer.
        try:
            if self.proc.poll() is None: self.sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            self.proc.wait(timeout=3)
            self.rows.append(dict(event='shutdown_escalated_SIGTERM',ns=time.monotonic_ns()))
        if self.bridge_proc is not None:
            try:
                self.bridge_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bridge_proc.terminate()
                try:
                    self.bridge_proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.bridge_proc.kill()
                    self.bridge_proc.wait(timeout=1)
            append_rows(Path(self.config['output_dir'])/'parent_lifecycle.jsonl',[
                dict(event='bridge_joined',ns=time.monotonic_ns(),bridge_returncode=self.bridge_proc.returncode)])
        self.closed=True; self.sock.close()
        append_rows(Path(self.config['output_dir'])/'parent_ipc.jsonl',self.rows)
        append_rows(Path(self.config['output_dir'])/'parent_lifecycle.jsonl',[
            dict(event='child_joined',ns=time.monotonic_ns(),child_returncode=self.proc.returncode)])


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--fd',type=int,required=False)
    p.add_argument('--fd-cmd',type=int,required=False)
    p.add_argument('--fd-fb',type=int,required=False)
    p.add_argument('--config',required=True)
    a=p.parse_args()
    cmd_fd = a.fd_cmd if a.fd_cmd is not None else a.fd
    if cmd_fd is None:
        raise ValueError('Either --fd or --fd-cmd is required')
    cmd_s = socket.socket(fileno=cmd_fd); cmd_s.setblocking(False)
    if a.fd_fb is not None:
        fb_s = socket.socket(fileno=a.fd_fb); fb_s.setblocking(False)
    else:
        fb_s = None
    child_run(cmd_s, json.loads(a.config), fb_sock=fb_s)


if __name__=='__main__': main()
