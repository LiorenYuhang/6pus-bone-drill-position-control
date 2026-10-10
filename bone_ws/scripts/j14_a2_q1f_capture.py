#!/usr/bin/env python3
"""Q1F 30/60s capture, preserving historical P1/Q1 capture scripts."""
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
ROOT=Path('/home/lyapunov/bone_drill')
sys.path.insert(0,str(ROOT/'bone_ws/scripts'))
from j14_real_failure_sidecar import processes,serial_check
from j14_a1_13_analyze import lines
from j14_a2_q1f_analyze import inspect_run


def command(out,duration,hardware='active',architecture=None):
    args=['ros2','launch','pus6_bringup','pus6_system.launch.py',f'hardware_mode:={hardware}',
        f'config_file:={ROOT}/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml',
        f'driver_audit_directory:={out}',f'servo_run_duration_s:={float(duration)}',
        'driver_telemetry_mode:=qualification']
    if architecture:args.append(f'driver_architecture:={architecture}')
    return args


def admission(meta):
    return (meta.get('stop_reason')=='window_complete' and meta.get('cycles')==1500
        and meta.get('tx_timeout_count')==0 and meta.get('hardware_fault_count')==0
        and meta.get('packet_drop_count')==0 and meta.get('first_cycle_seq')==0
        and meta.get('pause_ack_confirmed') and meta.get('child_returncode')==0
        and meta.get('bridge_returncode')==0 and meta.get('serial_after',{}).get('free')
        and not meta.get('residual_processes') and meta.get('handshake_valid')
        and meta.get('production_path_confirmed') and meta.get('unique_publishers_confirmed')
        and not meta.get('shutdown_escalation'))


def graph_snapshot(observer):
    return {topic:[dict(node_name=x.node_name,namespace=x.node_namespace,type=x.topic_type,
        qos=dict(reliability=x.qos_profile.reliability.name,durability=x.qos_profile.durability.name,
                 history=x.qos_profile.history.name,depth=x.qos_profile.depth))
        for x in observer.get_publishers_info_by_topic(topic)]
        for topic in ('/joint_states','/pus6/joint_feedback/timed','/pus6/joint_command/timed')}


def run(out,run_id,hardware=True,architecture=None):
    if out.exists():raise RuntimeError('Refusing to overwrite evidence')
    if hardware and run_id==2:
        p=out.parent/'j14_a2_q1f_r1_static_30s/run_metadata.json'
        if not p.exists() or not admission(json.loads(p.read_text())) or not inspect_run(p.parent)['Q1F_PASS']:raise RuntimeError('R2 requires Q1F-R1 PASS, zero resets/skipped slots and queue probes')
    preflight=dict(processes=processes(),serial=serial_check())
    if preflight['processes'] or not preflight['serial']['free']:raise RuntimeError(f'Resource conflict: {preflight}')
    duration=(30 if run_id==1 else 60) if hardware else 2
    out.mkdir(parents=True)
    env={k:v for k,v in os.environ.items() if not k.startswith('PUS6_J14_')}
    meta=dict(stage='A2-Q1F',run_id=run_id,hardware=hardware,command=command(out,duration,'active' if hardware else 'mock',architecture),
        duration_s=duration,preflight=preflight,hardware_write_count=None if hardware else 0,experimental_environment_flags={},start_ns=time.monotonic_ns())
    (out/'j13_runtime_config.yaml').write_bytes((ROOT/'bone_ws/src/h30_imu/config/j13_ladrc_params.yaml').read_bytes())
    (out/'production_defaults.yaml').write_bytes((ROOT/'bone_ws/src/pus6_motor_driver/config/three_process.yaml').read_bytes())
    (out/'run_metadata_start.json').write_text(json.dumps(meta,indent=2)+'\n')
    rclpy.init();observer=Node('j14_a2_p1_read_only_observer',enable_rosout=False,start_parameter_services=False)
    meta['rmw']=get_rmw_implementation_identifier()
    proc=None;snapshot=None;reason='harness_deadline';graph={}
    try:
        with (out/'robot.log').open('w') as log:
            proc=subprocess.Popen(meta['command'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            deadline=time.monotonic()+duration+7
            graph_deadline=0
            while proc.poll() is None and time.monotonic()<deadline:
                ready=lines(out/'child_ready.jsonl')
                if ready and snapshot is None:
                    child=ready[0];config=json.loads((out/'production_configuration.json').read_text())
                    b=lines(out/'bridge_lifecycle.jsonl');bridge_pid=b[0]['pid'] if b else None
                    snapshot={}
                    for role,pid in [('parent',child['parent_pid']),('child',child['pid']),('bridge',bridge_pid)]:
                        base=Path('/proc')/str(pid)
                        try:snapshot[role]=dict(pid=pid,cmdline=(base/'cmdline').read_bytes().replace(b'\0',b' ').decode(),
                            tasks={p.name:(p/'comm').read_text().strip() for p in (base/'task').iterdir()})
                        except OSError as exc:snapshot[role]=dict(error=repr(exc))
                    fuser=subprocess.run(['fuser','/dev/ttyUSB0'],capture_output=True,text=True)
                    snapshot['motor_uart_fuser']=dict(returncode=fuser.returncode,stdout=fuser.stdout,stderr=fuser.stderr)
                    (out/'runtime_process_boundary.json').write_text(json.dumps(snapshot,indent=2)+'\n')
                    graph_deadline=time.monotonic()+3
                if snapshot is not None and time.monotonic()<graph_deadline:
                    graph=graph_snapshot(observer)
                    if all(len(v)==1 and v[0]['node_name']=='pus6_feedback_bridge_node' for v in graph.values()):
                        graph_deadline=0
                        (out/'feedback_ros_graph.json').write_text(json.dumps(graph,indent=2)+'\n')
                if (out/'child_final.json').exists():
                    reason=json.loads((out/'child_final.json').read_text())['fault'];break
                text=(out/'robot.log').read_text()
                if 'Failed to initialize J7 Servo backend' in text:
                    reason='startup_failed';break
                if not hardware and architecture=='legacy_single_process' and time.monotonic()>deadline-7:
                    reason='legacy_mock_observation_complete'
                    (out/'legacy_feedback_ros_graph.json').write_text(json.dumps(graph_snapshot(observer),indent=2)+'\n')
                    break
                time.sleep(.005)
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                meta['shutdown_escalation']='SIGTERM';proc.terminate();proc.wait(timeout=3)
        observer.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    robot=(out/'robot.log').read_text()
    child=json.loads((out/'child_final.json').read_text()) if (out/'child_final.json').exists() else {}
    cycles=lines(out/'child_cycles.jsonl');writes=lines(out/'child_write_metrics.jsonl')
    bridge=lines(out/'bridge_feedback_pipeline.jsonl');bl=lines(out/'bridge_lifecycle.jsonl')
    parent=lines(out/'parent_lifecycle.jsonl');pause=lines(out/'stop_pause_audit.jsonl')
    health=json.loads((out/'child_health_summary.json').read_text()) if (out/'child_health_summary.json').exists() else {}
    joins={r['event']:r for r in parent if r['event'] in ('child_joined','bridge_joined')}
    startup=lines(out/'child_ready.jsonl');start=next((r for r in startup if r.get('type')=='servo_start'),{})
    handshake=bool(start and cycles) and cycles[0]['start_ns']>=start['servo_start_ns']>=max(start[k] for k in ('t_child_ready_ns','t_bridge_ready_ns','t_parent_ready_ns'))
    seqs=[r['cycle_seq'] for r in bridge]
    missing=sorted(set(range(child.get('cycles',0)))-set(seqs))
    config=json.loads((out/'production_configuration.json').read_text()) if (out/'production_configuration.json').exists() else {}
    meta.update(end_ns=time.monotonic_ns(),stop_reason=reason,launch_returncode=proc.returncode if proc else None,
        child_final=child,child_returncode=joins.get('child_joined',{}).get('child_returncode'),
        bridge_returncode=joins.get('bridge_joined',{}).get('bridge_returncode'),cycles=child.get('cycles'),bridge_cycles=len(bridge),
        tx_timeout_count=health.get('tx_timeouts'),hardware_fault_count=health.get('fault_cycles'),
        packet_drop_count=len(missing),missing_cycle_sequences=missing,first_cycle_seq=seqs[0] if seqs else None,
        duplicate_cycles=len(seqs)-len(set(seqs)),handshake_valid=handshake,
        production_path_confirmed=config.get('parameters',{}).get('driver_architecture')=='three_process'
            and config.get('child_configuration',{}).get('production') is True,
        unique_publishers_confirmed=bool(graph) and all(len(v)==1 and v[0]['node_name']=='pus6_feedback_bridge_node' for v in graph.values()),
        pause_ack_confirmed=bool(pause) and pause[-1]['success'] and len(pause[-1]['records'])==6
            and all(r['response_received'] and not r['timeout'] for r in pause[-1]['records']),
        residual_processes=processes(),serial_after=serial_check(),
        shutdown_exception=any(x in robot for x in ('Traceback','rcl_shutdown already called','Error dumping')))
    meta['r2_admissible']=admission(meta)
    (out/'run_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2))
    if meta['residual_processes'] or not meta['serial_after']['free']:raise RuntimeError('Cleanup incomplete')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--run-id',type=int,choices=(1,2),default=1);p.add_argument('--execute-hardware',action='store_true')
    p.add_argument('--mock',action='store_true');p.add_argument('--legacy-rollback',action='store_true')
    a=p.parse_args()
    if 'j14_a2_q1f_' not in a.output_dir.name:p.error('new Q1F directory required')
    if not a.execute_hardware and not a.mock:p.error('--execute-hardware or --mock required')
    if a.legacy_rollback and a.execute_hardware:p.error('legacy regression is software-only')
    run(a.output_dir.resolve(),a.run_id,hardware=a.execute_hardware,architecture='legacy_single_process' if a.legacy_rollback else None)
