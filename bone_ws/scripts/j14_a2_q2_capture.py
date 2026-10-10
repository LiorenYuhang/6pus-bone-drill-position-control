#!/usr/bin/env python3
"""Authorized Q2 static qualification capture via production ROS 2 launch/parameters."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier

ROOT = Path('/home/lyapunov/bone_drill')
sys.path.insert(0, str(ROOT / 'bone_ws/scripts'))
from j14_real_failure_sidecar import processes, serial_check
from j14_a1_13_analyze import lines
sys.path.insert(0, str(ROOT / 'bone_ws/src/h30_imu/scripts'))
from j13_ladrc_staged_launcher import HostHeartbeatObserver


def command(out: Path, duration: float, hardware: str = 'active'):
    return [
        'ros2', 'launch', 'pus6_bringup', 'pus6_system.launch.py',
        f'hardware_mode:={hardware}',
        f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
        f'driver_audit_directory:={out}',
        f'servo_run_duration_s:={float(duration)}',
        'driver_telemetry_mode:=qualification',
        'driver_telemetry_capacity_cycles:=32768',
    ]


def graph_snapshot(observer):
    return {
        topic: [
            dict(
                node_name=x.node_name,
                namespace=x.node_namespace,
                type=x.topic_type,
                qos=dict(
                    reliability=x.qos_profile.reliability.name,
                    durability=x.qos_profile.durability.name,
                    history=x.qos_profile.history.name,
                    depth=x.qos_profile.depth,
                )
            )
            for x in observer.get_publishers_info_by_topic(topic)
        ]
        for topic in ('/joint_states', '/pus6/joint_feedback/timed', '/pus6/joint_command/timed')
    }


def run(out: Path, run_id: int, duration: float, hardware: bool = True):
    if out.exists():
        raise RuntimeError(f'Refusing to overwrite existing evidence: {out}')

    preflight = dict(processes=processes(), serial=serial_check())
    if preflight['processes'] or not preflight['serial']['free']:
        raise RuntimeError(f'Resource conflict before Q2 run: {preflight}')

    out.mkdir(parents=True)
    env = {k: v for k, v in os.environ.items() if not k.startswith('PUS6_J14_')}

    cmd = command(out, duration, 'active' if hardware else 'mock')
    meta = dict(
        stage='A2-Q2',
        run_id=f'Q2-R{run_id}',
        hardware=hardware,
        command=cmd,
        duration_s=duration,
        expected_cycles=int(duration * 50),
        driver_telemetry_capacity_cycles=32768,
        preflight=preflight,
        hardware_write_count=None if hardware else 0,
        start_ns=time.monotonic_ns(),
        start_epoch_ns=time.time_ns(),
    )
    (out / 'j13_runtime_config.yaml').write_bytes((ROOT / 'bone_ws/src/h30_imu/config/j13_ladrc_params.yaml').read_bytes())
    (out / 'production_defaults.yaml').write_bytes((ROOT / 'bone_ws/src/pus6_motor_driver/config/three_process.yaml').read_bytes())
    (out / 'run_metadata_start.json').write_text(json.dumps(meta, indent=2) + '\n')

    # Start host heartbeat observer
    heartbeat_path = out / 'host_heartbeat_stalls.csv'
    observer_hb = HostHeartbeatObserver(heartbeat_path)
    observer_hb.start()

    rclpy.init()
    observer_ros = Node('j14_a2_q2_observer', enable_rosout=False, start_parameter_services=False)
    meta['rmw'] = get_rmw_implementation_identifier()

    proc = None
    snapshot = None
    reason = 'harness_deadline'
    graph = {}
    try:
        with (out / 'robot.log').open('w') as log:
            proc = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            # Give a safety buffer above nominal duration for hardware init + handshake
            deadline = time.monotonic() + duration + 15.0
            graph_deadline = 0

            while proc.poll() is None and time.monotonic() < deadline:
                ready = lines(out / 'child_ready.jsonl')
                if ready and snapshot is None:
                    child = ready[0]
                    b = lines(out / 'bridge_lifecycle.jsonl')
                    bridge_pid = b[0]['pid'] if b else None
                    snapshot = {}
                    for role, pid in [('parent', child['parent_pid']), ('child', child['pid']), ('bridge', bridge_pid)]:
                        base = Path('/proc') / str(pid)
                        try:
                            snapshot[role] = dict(
                                pid=pid,
                                cmdline=(base / 'cmdline').read_bytes().replace(b'\0', b' ').decode(),
                                tasks={p.name: (p / 'comm').read_text().strip() for p in (base / 'task').iterdir()}
                            )
                        except OSError as exc:
                            snapshot[role] = dict(error=repr(exc))
                    fuser = subprocess.run(['fuser', '/dev/ttyUSB0'], capture_output=True, text=True)
                    snapshot['motor_uart_fuser'] = dict(returncode=fuser.returncode, stdout=fuser.stdout, stderr=fuser.stderr)
                    (out / 'runtime_process_boundary.json').write_text(json.dumps(snapshot, indent=2) + '\n')
                    graph_deadline = time.monotonic() + 3.0

                if snapshot is not None and time.monotonic() < graph_deadline:
                    graph = graph_snapshot(observer_ros)
                    if all(len(v) == 1 and v[0]['node_name'] == 'pus6_feedback_bridge_node' for v in graph.values()):
                        graph_deadline = 0
                        (out / 'feedback_ros_graph.json').write_text(json.dumps(graph, indent=2) + '\n')

                final_path = out / 'child_final.json'
                if final_path.exists():
                    reason = json.loads(final_path.read_text()).get('fault', 'unknown')
                    break

                text = (out / 'robot.log').read_text()
                if 'Failed to initialize J7 Servo backend' in text or 'STOP latched' in text:
                    reason = 'startup_or_hardware_fault'
                    break

                time.sleep(0.010)

    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                meta['shutdown_escalation'] = 'SIGTERM'
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except OSError:
                    proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except OSError:
                        proc.kill()
                    proc.wait(timeout=2)

        observer_ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        observer_hb.stop_and_flush()

    # Settle down and collect evidence
    time.sleep(0.5)

    robot = (out / 'robot.log').read_text()
    child = json.loads((out / 'child_final.json').read_text()) if (out / 'child_final.json').exists() else {}
    cycles = lines(out / 'child_cycles.jsonl')
    writes = lines(out / 'child_write_metrics.jsonl')
    bridge = lines(out / 'bridge_feedback_pipeline.jsonl')
    bl = lines(out / 'bridge_lifecycle.jsonl')
    parent = lines(out / 'parent_lifecycle.jsonl')
    pause = lines(out / 'stop_pause_audit.jsonl')
    health = json.loads((out / 'child_health_summary.json').read_text()) if (out / 'child_health_summary.json').exists() else {}
    joins = {r['event']: r for r in parent if r['event'] in ('child_joined', 'bridge_joined')}
    startup = lines(out / 'child_ready.jsonl')
    start = next((r for r in startup if r.get('type') == 'servo_start'), {})
    handshake = bool(start and cycles) and cycles[0]['start_ns'] >= start['servo_start_ns'] >= max(start[k] for k in ('t_child_ready_ns', 't_bridge_ready_ns', 't_parent_ready_ns'))
    seqs = [r['cycle_seq'] for r in bridge]
    missing = sorted(set(range(child.get('cycles', 0))) - set(seqs))
    config = json.loads((out / 'production_configuration.json').read_text()) if (out / 'production_configuration.json').exists() else {}

    meta.update(
        end_ns=time.monotonic_ns(),
        end_epoch_ns=time.time_ns(),
        stop_reason=reason,
        launch_returncode=proc.returncode if proc else None,
        child_final=child,
        child_returncode=joins.get('child_joined', {}).get('child_returncode'),
        bridge_returncode=joins.get('bridge_joined', {}).get('bridge_returncode'),
        cycles=child.get('cycles'),
        bridge_cycles=len(bridge),
        tx_timeout_count=health.get('tx_timeouts', 0),
        hardware_fault_count=health.get('fault_cycles', 0),
        packet_drop_count=len(missing),
        missing_cycle_sequences=missing,
        first_cycle_seq=seqs[0] if seqs else None,
        duplicate_cycles=len(seqs) - len(set(seqs)),
        handshake_valid=handshake,
        production_path_confirmed=config.get('parameters', {}).get('driver_architecture') == 'three_process'
            and config.get('child_configuration', {}).get('production') is True,
        unique_publishers_confirmed=bool(graph) and all(len(v) == 1 and v[0]['node_name'] == 'pus6_feedback_bridge_node' for v in graph.values()),
        pause_ack_confirmed=bool(pause) and pause[-1].get('success') and len(pause[-1].get('records', [])) == 6
            and all(r.get('response_received') and not r.get('timeout') for r in pause[-1]['records']),
        residual_processes=processes(),
        serial_after=serial_check(),
        shutdown_exception=any(x in robot for x in ('Traceback', 'rcl_shutdown already called', 'Error dumping')),
    )

    (out / 'run_metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(json.dumps(meta, indent=2))

    if meta['residual_processes'] or not meta['serial_after']['free']:
        raise RuntimeError('Cleanup incomplete: residual processes or locked serial')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--run-id', type=int, choices=(1, 2, 3), default=1)
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--execute-hardware', action='store_true')
    a = p.parse_args()
    if not a.execute_hardware:
        p.error('--execute-hardware is required')
    run(a.output_dir.resolve(), a.run_id, a.duration, hardware=True)
