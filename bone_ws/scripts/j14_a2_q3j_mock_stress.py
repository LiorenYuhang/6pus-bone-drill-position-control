#!/usr/bin/env python3
"""Q3J software-only workload and transparent, stage-local runtime observation.

Uses the actual production entry, Mock Child, Bridge and unchanged Q3 relay.
All instrumentation is test-local; no production Child source is changed.
"""
import argparse
import cProfile
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def rows(path, values):
    with path.open('w') as f:
        for value in values:
            f.write(json.dumps(value) + '\n')


def percentile(values):
    values = sorted(values)
    if not values:
        return None
    def q(p):
        x = (len(values)-1)*p
        i = int(x)
        return values[i] + (values[min(i+1, len(values)-1)]-values[i])*(x-i)
    return dict(n=len(values), p50=q(.5), p95=q(.95), p99=q(.99), p999=q(.999), max=values[-1])


def child_probe():
    import runpy
    config = json.loads(sys.argv[sys.argv.index('--config')+1])
    if config.get('hardware_mode') != 'mock':
        raise RuntimeError('Q3J software harness permits Mock Child only')
    bootstrap = Path(__file__).resolve().parents[1]/'src/pus6_motor_driver/pus6_motor_driver/j14_uart_child_main.py'
    runpy.run_path(str(bootstrap), run_name='q3j_bootstrap')
    from pus6_motor_driver import j14_process_uart as uart
    original = uart.ServoCore.cycle
    original_stop = uart.ServoCore.stop
    samples = []
    stop_samples = []
    def cycle(core, *args, **kwargs):
        now = time.monotonic_ns()
        samples.append(dict(ns=now, lease_age_ns=now-core.command.lease_ns,
                            seq=core.command.seq))
        return original(core, *args, **kwargs)
    uart.ServoCore.cycle = cycle
    def stop(core, reason):
        now = time.monotonic_ns()
        stop_samples.append(dict(ns=now, reason=reason, lease_age_ns=now-core.command.lease_ns))
        return original_stop(core, reason)
    uart.ServoCore.stop = stop
    try:
        uart.main()
    finally:
        config = json.loads(sys.argv[sys.argv.index('--config')+1])
        rows(Path(config['output_dir'])/'mock_child_lease_samples.jsonl', samples)
        rows(Path(config['output_dir'])/'mock_child_stop_samples.jsonl', stop_samples)


def parent_probe(out):
    from pus6_motor_driver import production_driver as production
    from pus6_motor_driver import j14_process_uart as uart
    from pus6_motor_driver.j14_process_driver import J14ProcessMotorDriverNode
    original_popen = subprocess.Popen
    def popen(argv, *args, **kwargs):
        if isinstance(argv, list) and len(argv)>1 and argv[1].endswith('j14_uart_child_main.py'):
            argv = [argv[0], str(Path(__file__).resolve()), '--child-probe', *argv[2:]]
        return original_popen(argv, *args, **kwargs)
    subprocess.Popen = popen
    callback = J14ProcessMotorDriverNode._enqueue_latest_target
    ingress = []
    def enqueue(node, targets):
        start = time.monotonic_ns()
        callback(node, targets)
        ingress.append(dict(ns=start, done_ns=time.monotonic_ns(), seq=node._target_in_count,
                            targets=list(targets)))
    J14ProcessMotorDriverNode._enqueue_latest_target = enqueue
    send = uart.send_packet
    packets = []
    def send_packet(sock, packet):
        result = send(sock, packet)
        if packet.get('type')=='state':
            packets.append(dict(ns=time.monotonic_ns(), lease_ns=packet['lease_ns'], sent=result,
                seq=(packet.get('command') or {}).get('seq')))
        return result
    uart.send_packet = send_packet
    factory = production.create_driver
    holder = {}
    def create(*args, **kwargs):
        if production.read_configuration()['hardware_mode'] != 'mock':
            raise RuntimeError('Q3J software harness permits Mock Parent only')
        node = factory(*args, **kwargs)
        holder['node'] = node
        save(out/'probe_ready.json', dict(parent_pid=os.getpid(), ns=time.monotonic_ns(),
            child_pid=node._process_client.proc.pid, bridge_pid=node._process_client.bridge_proc.pid))
        return node
    production.create_driver = create
    samples = []
    finished = threading.Event()
    def sample():
        while not finished.wait(.5):
            node = holder.get('node')
            if node is None:
                continue
            ex = node.executor
            pool = getattr(ex, '_executor', None)
            queue = getattr(pool, '_work_queue', None)
            row = dict(ns=time.monotonic_ns(), cpu_ns=time.process_time_ns(),
                ingress=node._target_in_count, executor=type(ex).__name__,
                pending=len(getattr(ex, '_pending_tasks', {})),
                ready=len(getattr(ex, '_ready_tasks', [])),
                pool_queue=queue.qsize() if queue is not None else 0)
            row['executor_futures'] = len(getattr(ex, '_futures', []))
            row['native_thread_ticks'] = {}
            for task in Path(f'/proc/{os.getpid()}/task').glob('*'):
                try:
                    stat = (task/'stat').read_text().split()
                    row['native_thread_ticks'][task.name] = dict(name=stat[1], ticks=int(stat[13])+int(stat[14]))
                except FileNotFoundError:
                    pass
            # Sampling only: no callback work, no file IO in the runtime loop.
            row['stacks'] = {str(tid):traceback.format_stack(frame)[-5:]
                             for tid, frame in sys._current_frames().items()}
            samples.append(row)
    thread = threading.Thread(target=sample, daemon=True)
    thread.start()
    profile = cProfile.Profile() if os.getenv('Q3J_PROFILE') == '1' else None
    event_counts = {}
    if profile:
        from rclpy.event_handler import EventHandler
        original_ready = EventHandler.is_ready
        def is_ready(event, wait_set):
            result = original_ready(event, wait_set)
            key = f'{event.event_type}:{result}'
            event_counts[key] = event_counts.get(key, 0)+1
            return result
        EventHandler.is_ready = is_ready
    try:
        if profile:
            profile.enable()
        production.main()
    finally:
        if profile:
            profile.disable(); profile.dump_stats(str(out/'parent_main.prof'))
            save(out/'executor_event_counts.json', event_counts)
            import pstats
            with (out/'parent_main_profile.txt').open('w') as f:
                pstats.Stats(profile, stream=f).sort_stats('tottime').print_stats(60)
        finished.set(); thread.join(timeout=2)
        rows(out/'parent_cpu_samples.jsonl', samples)
        rows(out/'command_ingress.jsonl', ingress)
        rows(out/'parent_state_packets.jsonl', packets)


def analyze(out, start, end, published):
    def read(name):
        return [json.loads(x) for x in (out/name).read_text().splitlines()]
    cpu = read('parent_cpu_samples.jsonl')
    cpu_pct = [(b['cpu_ns']-a['cpu_ns'])/(b['ns']-a['ns'])*100
               for a,b in zip(cpu,cpu[1:]) if start<=a['ns']<b['ns']<=end]
    packets = [r for r in read('parent_state_packets.jsonl') if r['sent'] and start<=r['ns']<=end]
    gap = [(b['lease_ns']-a['lease_ns'])/1e6 for a,b in zip(packets,packets[1:])]
    seq = [r['seq'] for r in packets if r['seq'] is not None]
    ingress = [r for r in read('command_ingress.jsonl') if start<=r['ns']<=end]
    lease = [r for r in read('mock_child_lease_samples.jsonl') if start<=r['ns']<=end]
    cycles = [r for r in read('child_cycles.jsonl') if start<=r['start_ns']<=end]
    final = json.loads((out/'child_final.json').read_text())
    health = json.loads((out/'child_health_summary.json').read_text())
    dynamic = len({tuple(r['targets']) for r in ingress})
    summary = dict(duration_s=(end-start)/1e9, parent_cpu_percent=percentile(cpu_pct),
        ipc_gap_ms=percentile(gap), ipc_gaps_over_250ms=sum(x>250 for x in gap),
        command_ingress_count=len(ingress), command_ingress_hz=len(ingress)/((end-start)/1e9),
        published_commands=len(published), distinct_command_vectors=dynamic,
        command_seq_first=seq[0] if seq else None, command_seq_last=seq[-1] if seq else None,
        ipc_command_duplicate_or_regression=sum(b<=a for a,b in zip(seq,seq[1:])),
        ipc_send_not_accepted=sum(not r['sent'] for r in read('parent_state_packets.jsonl')),
        child_lease_age_ms=percentile([r['lease_age_ns']/1e6 for r in lease]),
        child_cycle_gap_ms=percentile([(b['start_ns']-a['start_ns'])/1e6 for a,b in zip(cycles,cycles[1:])]),
        child_cycle_duration_ms=percentile([(r['end_ns']-r['start_ns'])/1e6 for r in cycles]),
        child_final=final, child_health=health,
        parent_executor_pool_queue_max=max((r['pool_queue'] for r in cpu), default=0),
        hardware_write_count=final['hardware_write_count'])
    summary['pass'] = (final['fault'] == 'parent_shutdown' and final['hardware_write_count']==0
        and final['pause_success'] and health['fault_cycles']==0
        and summary['ipc_command_duplicate_or_regression']==0 and summary['ipc_send_not_accepted']==0
        and summary['ipc_gaps_over_250ms']==0
        and (not published or (48<=summary['command_ingress_hz']<=52 and dynamic>10)))
    save(out/'summary.json', summary)
    return summary


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Int32MultiArray
    from sensor_msgs.msg import JointState
    out = args.out.resolve(); out.mkdir(parents=True, exist_ok=False)
    if any(k.startswith('PUS6_J14_') for k in os.environ):
        raise RuntimeError('Remove legacy PUS6_J14 environment flags before stress')
    save(out/'conditions.json', dict(hardware_mode='mock', ros_domain_id=os.getenv('ROS_DOMAIN_ID'),
        start_wall=time.strftime('%Y-%m-%dT%H:%M:%S%z'), duration_s=args.duration,
        static=args.static, command_rate_hz=0 if args.static else 50,
        producer='software sine steps +/- 8; phase shifted six axes',
        entry='production_driver.main', instrumentation='transparent stage-local wrappers'))
    logs = [(out/'parent.log').open('w'), (out/'relay.log').open('w')]
    parent = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--parent-probe', str(out),
        '--ros-args', '-p', 'hardware_mode:=mock', '-p', 'driver_telemetry_mode:=qualification',
        '-p', 'driver_telemetry_capacity_cycles:=32768', '-p', f'driver_audit_directory:={out}',
        '-p', 'servo_run_duration_s:=0.0'], stdout=logs[0], stderr=subprocess.STDOUT)
    relay = None; node = None; published = []; start=end=0
    compatibility = dict(raw_steps=0, joint_states=0)
    try:
        until=time.monotonic()+15
        while not (out/'probe_ready.json').exists():
            if parent.poll() is not None or time.monotonic()>until:
                raise RuntimeError('Mock parent failed to initialize')
            time.sleep(.05)
        relay = subprocess.Popen([sys.executable, str(Path(__file__).with_name('j14_a2_q3_feedback_relay.py')),
                                  '--out-dir', str(out)], stdout=logs[1], stderr=subprocess.STDOUT)
        rclpy.init(); node=Node('q3j_mock_command_source')
        def raw(msg):
            if len(msg.data)==6: compatibility['raw_steps']+=1
        def joint(msg):
            if len(msg.position)==6: compatibility['joint_states']+=1
        node.create_subscription(Int32MultiArray, '/pus6/joint_steps_raw', raw, qos_profile_sensor_data)
        node.create_subscription(JointState, '/pus6/joint_states', joint, 10)
        pub=node.create_publisher(Int32MultiArray, '/pus6/command_joint_steps', qos_profile_sensor_data)
        until=time.monotonic()+5
        while pub.get_subscription_count()==0 and time.monotonic()<until:
            time.sleep(.05)
        time.sleep(.5)
        start=time.monotonic_ns(); deadline=start; stop_at=start+int(args.duration*1e9)
        while time.monotonic_ns()<stop_at:
            # Drain a bounded number of observer callbacks; publisher deadline
            # remains 20 ms and the production Parent is a separate process.
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0)
            if parent.poll() is not None or (out/'child_final.json').exists():
                break
            if not args.static:
                phase=(deadline-start)/1e9*2*math.pi/4
                values=[1000+round(8*math.sin(phase+i*.3)) for i in range(6)]
                msg=Int32MultiArray(); msg.data=values; pub.publish(msg)
                published.append(dict(ns=time.monotonic_ns(), targets=values))
            deadline+=20_000_000
            time.sleep(max(0,(deadline-time.monotonic_ns())/1e9))
        end=time.monotonic_ns()
    finally:
        rows(out/'published_commands.jsonl', published)
        save(out/'measurement_window.json', dict(start_ns=start, end_ns=end))
        save(out/'compatibility_observer.json', compatibility)
        if node is not None:
            node.destroy_node(); rclpy.shutdown()
        if parent.poll() is None:
            parent.send_signal(signal.SIGINT)
        try:
            parent.wait(timeout=15)
        except subprocess.TimeoutExpired:
            parent.terminate(); parent.wait(timeout=5)
        if relay is not None:
            relay.send_signal(signal.SIGINT)
            try:
                relay.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # This is the known stage relay cleanup hang; preserve it, only
                # reap the subprocess created by this software harness.
                save(out/'relay_cleanup_timeout.json', dict(pid=relay.pid, signal='SIGINT'))
                relay.kill(); relay.wait(timeout=5)
        for f in logs:
            f.close()
        save(out/'exit_status.json', dict(parent_returncode=parent.returncode,
             relay_returncode=relay.returncode if relay else None))
    summary=analyze(out,start,end,published)
    summary['requested_duration_s'] = args.duration
    summary['completed_requested_duration'] = (end-start)/1e9 >= args.duration
    summary['compatibility_observer'] = compatibility
    summary['pass'] = (summary['pass'] and summary['completed_requested_duration']
        and compatibility['raw_steps']>0 and compatibility['joint_states']>0)
    save(out/'summary.json', summary)
    print(json.dumps(summary,indent=2), flush=True)


if __name__=='__main__':
    if '--child-probe' in sys.argv:
        sys.argv.remove('--child-probe'); child_probe()
    elif '--parent-probe' in sys.argv:
        idx=sys.argv.index('--parent-probe'); directory=Path(sys.argv[idx+1])
        del sys.argv[idx:idx+2]; parent_probe(directory)
    else:
        parser=argparse.ArgumentParser()
        parser.add_argument('--out', type=Path, required=True)
        parser.add_argument('--duration', type=float, required=True)
        parser.add_argument('--static', action='store_true')
        run(parser.parse_args())
