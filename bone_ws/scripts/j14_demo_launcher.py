#!/usr/bin/env python3
"""One lifecycle for three J14 demos; production driver and control remain unchanged."""
from __future__ import annotations

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import threading
import time

from j14_demo_controller import CONFIG_DIR, MODES, ROOT, write_json
SCRIPTS = ROOT / 'bone_ws/scripts'
PORTS = {
    'motor': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    'base': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00',
    'platform': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00',
}


def runtime_processes():
    names = {'motor_driver_node', 'kinematics_node', 'control_node', 'fk_node',
             'j14_uart_child_main.py', 'j14_feedback_bridge_main.py',
             'h30_imu_node', 'j14_a2_q3_feedback_relay.py', 'j14_demo_controller.py',
             'j14_demo_mock_imu.py', 'j14_demo_node_host.py', 'j14_demo_f1_mock_child.py', 'j14_demo_f2_mock_child.py', 'j14_demo_relay_host.py',
             'j11_base_feedforward_runner.py',
             'j11_m2_pid_residual_runner.py', 'j13_ladrc_shadow_runner.py',
             'j11_staged_launcher.py', 'j11_m2_staged_launcher.py', 'j13_ladrc_staged_launcher.py'}
    found = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            args = [s for s in (p / 'cmdline').read_bytes().decode().split('\0') if s]
            if any(Path(a).name in names for a in args) or ('launch' in args and 'pus6_system.launch.py' in args):
                found.append(dict(pid=int(p.name), args=args))
        except (OSError, UnicodeError):
            pass
    return found


def serial_status():
    result = subprocess.run(['fuser', *PORTS.values()], capture_output=True, text=True)
    return dict(free=result.returncode == 1, returncode=result.returncode,
                owners=result.stdout.strip(), detail=result.stderr.strip())


def rows(path):
    if not path.exists():
        return []
    result = []
    for line in path.read_text().splitlines():
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            # A writer may still be appending the last row.
            pass
    return result


class OwnedProcesses:
    def __init__(self, out, env):
        self.out, self.env = out, env
        self.procs, self.logs, self.events = {}, [], []

    def start(self, role, command):
        log = (self.out / f'{role}.log').open('x')
        self.logs.append(log)
        begin_ns = time.monotonic_ns()
        proc = subprocess.Popen(command, cwd=ROOT, env=self.env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        self.procs[role] = proc
        self.events.append(dict(event='start', role=role, pid=proc.pid,
                                command=command, start_call_ns=begin_ns, monotonic_ns=time.monotonic_ns()))
        write_json(self.out / 'processes.json', self.events)
        return proc

    def check(self):
        for role, proc in self.procs.items():
            if proc.poll() is not None:
                raise RuntimeError(f'{role} exited unexpectedly (returncode={proc.returncode})')
        final = self.out / 'child_final.json'
        if final.exists():
            raise RuntimeError(f'Driver STOP: {json.loads(final.read_text()).get("fault")}')

    def request_controller(self, action, reason):
        write_json(self.out/'stop_request.json',dict(action=action,reason=reason,ns=time.monotonic_ns()))

    def pause_driver(self):
        """Ask the existing Parent to close its Child, never write UART here."""
        path=self.out/'motor_driver_node_host.json'
        if not path.exists():
            return
        status=json.loads(path.read_text())
        event=dict(event='driver_pause_request',pid=status['pid'],ns=time.monotonic_ns(),signal=None)
        if status['state']!='exited':
            os.kill(status['pid'],signal.SIGINT)
            event['signal']='SIGINT'
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                status=json.loads(path.read_text())
                if status['state']=='exited':
                    break
                time.sleep(.025)
        # A host state file precedes Python interpreter exit. Wait for the actual
        # process to finish before ROS launch can forward a second SIGINT.
        deadline=time.monotonic()+10
        process_path=Path('/proc')/str(status['pid'])/'stat'
        while process_path.exists():
            try:
                if process_path.read_text().rsplit(')',1)[1].split()[0]=='Z':
                    break
            except FileNotFoundError:
                break
            if time.monotonic()>deadline:
                raise TimeoutError('Parent host did not exit after Pause')
            time.sleep(.01)
        event.update(state=status['state'],returncode=status.get('returncode'),end_ns=time.monotonic_ns(),process_exit_confirmed=True)
        self.events.append(event)
        write_json(self.out/'processes.json',self.events)

    def stop(self, role, timeout):
        proc = self.procs.get(role)
        if proc is None:
            return
        event = dict(event='stop', role=role, pid=proc.pid,
                     monotonic_ns=time.monotonic_ns(), signal=None, escalation=False)
        if proc.poll() is None:
            # Signal only the launch leader; ROS launch forwards the signal once.
            event['signal'] = 'SIGINT'
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                event['escalation'] = True
                event['signal'] = 'SIGTERM'
                proc.send_signal(signal.SIGTERM)
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    event['still_alive'] = True
        event.update(returncode=proc.poll(), elapsed_s=(time.monotonic_ns()-event['monotonic_ns'])/1e9)
        self.events.append(event)
        write_json(self.out / 'processes.json', self.events)

    def cleanup(self):
        # Parent pause is requested before this final resource teardown.
        for role, timeout in [('controller', 3.0), ('relay', 1.5),
                              ('base_imu', 3.0), ('platform_imu', 3.0), ('mock_imu', 3.0),
                              ('robot', 10.0)]:
            try:
                self.stop(role, timeout)
            except Exception as exc:
                self.events.append(dict(event='cleanup_error', role=role, error=repr(exc)))
        for log in self.logs:
            log.close()


def update_observation(state, key, msg):
    state['counts'][key] = state['counts'].get(key, 0) + 1
    state['last_rx'][key] = time.monotonic()
    if key in ('command', 'child_command'):
        steps = [int(s) for s in (msg.data if key == 'command' else msg.target_steps)]
        lo, hi = key + '_min', key + '_max'
        state[lo] = steps if state[lo] is None else [min(a,b) for a,b in zip(state[lo],steps)]
        state[hi] = steps if state[hi] is None else [max(a,b) for a,b in zip(state[hi],steps)]
    if key=='timed_feedback':
        state['feedback']=dict(steps=[int(s) for s in msg.raw_steps],
            faults=[int(s) for s in msg.fault_codes],currents=[int(s) for s in msg.currents_ma],
            rx_ns=[int(s) for s in msg.per_axis_rx_done_monotonic_ns])

def make_observer():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Int32MultiArray, Float64MultiArray
    from pus6_msgs.msg import TimedJointCommand, TimedJointFeedback
    node = rclpy.create_node('j14_demo_observer', enable_rosout=False, start_parameter_services=False)
    state = dict(counts={}, last_rx={}, command_min=None, command_max=None,
                 child_command_min=None, child_command_max=None)

    interfaces = [
        ('raw_feedback', Int32MultiArray, '/pus6/joint_steps_raw', qos_profile_sensor_data),
        ('joint_feedback', JointState, '/pus6/joint_states', 10),
        ('base', Imu, '/pus6/imu_base/data', qos_profile_sensor_data),
        ('platform', Imu, '/pus6/imu_platform/data', qos_profile_sensor_data),
        ('target', PoseStamped, '/pus6/target_cartesian_pose', 10),
        ('ik', Float64MultiArray, '/pus6/desired_joint_deltas', 10),
        ('command', Int32MultiArray, '/pus6/command_joint_steps', qos_profile_sensor_data),
        ('child_command', TimedJointCommand, '/pus6/joint_command/timed', qos_profile_sensor_data),
        ('timed_feedback', TimedJointFeedback, '/pus6/joint_feedback/timed', qos_profile_sensor_data),
    ]
    for key, typ, topic, qos in interfaces:
        node.create_subscription(typ, topic, lambda m, k=key: update_observation(state,k,m), qos)
    from rcl_interfaces.msg import Log
    node.driver_stop_latched=False
    def driver_log(msg):
        if msg.name=='motor_driver' or 'motor_driver' in msg.name:
            if any(word in msg.msg for word in ('STOP latched','scheduler_deadline_exceeded','overcurrent','Timeout recovery target rejected','reconnect_max_jump_steps')):
                node.driver_stop_latched=True;node.driver_fault_msg=msg.msg
    node.create_subscription(Log,'/rosout',driver_log,10)
    return node, state


def wait_ready(predicate, label, timeout, stop, owned, observer):
    import rclpy
    deadline = time.monotonic() + timeout
    while not stop.is_set():
        owned.check()
        if predicate():
            print(f'{label}: READY', flush=True)
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f'{label} startup timeout ({timeout}s)')
        rclpy.spin_once(observer, timeout_sec=0.05)
    raise InterruptedError('operator_stop_during_startup')


def graph(node):
    topics = ('/pus6/target_cartesian_pose', '/pus6/desired_joint_deltas',
              '/pus6/command_joint_steps', '/pus6/joint_feedback/timed', '/pus6/joint_states')
    return {t: dict(publishers=[x.node_name for x in node.get_publishers_info_by_topic(t)],
                    subscribers=[x.node_name for x in node.get_subscriptions_info_by_topic(t)]) for t in topics}


def startup_home(observer, feedback, owned, out, stop):
    from geometry_msgs.msg import PoseStamped
    from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
    from j14_demo_home import feedback_failure, recover_home
    print('Checking OPERATING_HOME...', flush=True)
    owned.check()
    reason=feedback_failure(observer,feedback)
    if reason:
        result=dict(state='SKIPPED',phase='startup',reason=reason,
                    home_target_publish_count=0,end_ns=time.monotonic_ns())
        write_json(out/'startup_home.json',result)
        return result
    # Sole Cartesian authority until HOME completion. Destroy the endpoint before
    # spawning the controller, which then owns the same normal command path.
    if observer.get_publishers_info_by_topic('/pus6/target_cartesian_pose'):
        raise RuntimeError('Cartesian command authority already in use')
    write_json(out/'startup_command_admission.json',dict(feedback=feedback,monotonic_ns=time.monotonic_ns()))
    wait_ready(lambda:(out/'startup_shaper_ready.json').exists(),
               'Feedback-anchored shaper',5.0,stop,owned,observer)
    # Recheck live feedback after the lifecycle handshake, before any HOME pose.
    owned.check()
    reason=feedback_failure(observer,feedback)
    if reason:
        result=dict(state='SKIPPED',phase='startup',reason=reason,
                    home_target_publish_count=0,end_ns=time.monotonic_ns())
        write_json(out/'startup_home.json',result)
        return result
    observer.z0=Analytical6PusKinematics().nominal_height
    from types import SimpleNamespace
    publishers=[]
    def publish(msg):
        if not publishers:
            print('Returning to OPERATING_HOME...',flush=True)
            publishers.append(observer.create_publisher(PoseStamped,'/pus6/target_cartesian_pose',10))
        publishers[0].publish(msg)
    observer.target_pose_pub=SimpleNamespace(publish=publish)
    try:
        result=recover_home(observer,feedback,out,stop,lambda:(owned.check() or {}),
                            phase='startup',allow_already_home=True)
    finally:
        if publishers:
            observer.destroy_publisher(publishers[0])
        observer.target_pose_pub=None
    result['authority_released_ns']=time.monotonic_ns()
    write_json(out/'startup_home.json',result)
    return result


def await_home(owned, observer, out, interrupted):
    import rclpy
    from j14_demo_home import _home_timeout_from_feedback, OPERATING_HOME_RAW_STEPS
    owned.request_controller('normal_home','operator_ctrl_c')
    # Reuse the same feedback-derived deadline; allow the existing runner-stop wait
    # (3 s) for lifecycle acknowledgement, not as an extra motion budget.
    deadline=time.monotonic()+3+_home_timeout_from_feedback([],OPERATING_HOME_RAW_STEPS)
    while True:
        owned.check()
        path=out/'home_recovery.json'
        if path.exists():
            home=json.loads(path.read_text())
            deadline=home['start_ns']/1e9+home['timeout_s']+3
            if home['state']!='HOME_RECOVERY':
                return home
        if interrupted():
            raise InterruptedError('safety interrupt during HOME')
        if time.monotonic()>deadline:
            raise TimeoutError('HOME lifecycle response timeout')
        rclpy.spin_once(observer,timeout_sec=.01)


def run_demo(mode, hardware_mode='mock', out=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    if mode not in MODES or hardware_mode not in ('mock', 'active'):
        raise ValueError('invalid Demo selection')
    stop = threading.Event()
    signal_state=dict(count=0,signal=None)
    def request_stop(sig,frame):
        signal_state['count']+=1;signal_state['signal']=sig;stop.set()
    previous_signals = {sig: signal.signal(sig, request_stop)
                        for sig in (signal.SIGINT, signal.SIGTERM)}
    # One demo owns the historical runner marker namespace and the robot resources.
    lock = open('/tmp/j14_demo.lock', 'a')
    owned = observer = None
    initialized = False
    meta = dict(mode=mode, hardware_mode=hardware_mode, startup='NOT_READY', active=False,
                start_epoch_ns=time.time_ns(), start_monotonic_ns=time.monotonic_ns())
    code = 1
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        conflicts = runtime_processes()
        if conflicts:
            raise RuntimeError(f'Required runtime already in use: {conflicts}')
        meta['pre_serial'] = serial_status()
        if hardware_mode == 'active' and not meta['pre_serial']['free']:
            raise RuntimeError(f'Serial resource conflict: {meta["pre_serial"]}')
        out = out or ROOT / 'results/j14/demo' / f'{mode}_{datetime.now():%Y%m%d_%H%M%S_%f}'
        out.mkdir(parents=True, exist_ok=False)
        print(f'Mode: {MODES[mode][1]} ({hardware_mode})\nLog: {out}', flush=True)
        env = {k:v for k,v in os.environ.items() if not k.startswith('PUS6_J14_')}
        env['ROS_LOG_DIR'] = str(out / 'ros_logs')
        env['PYTHONUNBUFFERED'] = '1'
        env['J14_DEMO_LOG_DIR']=str(out.resolve())
        owned = OwnedProcesses(out, env)
        write_json(out / 'demo_start.json', meta)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        observer, counts = make_observer()
        from j14_demo_home import attach_feedback
        home_feedback=attach_feedback(observer)
        config = CONFIG_DIR / MODES[mode][0]
        owned.start('robot', ['ros2', 'launch', '--launch-prefix',
                             shlex.join([sys.executable,str(SCRIPTS/'j14_demo_node_host.py')]),
                             '--launch-prefix-filter','^(motor_driver_node|kinematics_node|control_node|fk_node)$',
                             'pus6_bringup', 'pus6_system.launch.py',
                             f'hardware_mode:={hardware_mode}', f'config_file:={config}',
                             f'driver_audit_directory:={out}', 'driver_telemetry_mode:=production',
                             'servo_run_duration_s:=0.0'])
        wait_ready(lambda: any(r.get('type') == 'servo_start' for r in rows(out/'child_ready.jsonl')),
                   'Driver', 15.0, stop, owned, observer)
        owned.start('relay', [sys.executable, str(SCRIPTS/'j14_demo_relay_host.py'), '--out-dir', str(out)])
        wait_ready(lambda: all(counts['counts'].get(k,0)>0 for k in ('raw_feedback','joint_feedback')),
                   'Compatibility Relay / Feedback', 5.0, stop, owned, observer)
        wait_ready(lambda: home_feedback['count']>0,'Timed six-axis feedback',5.0,stop,owned,observer)
        meta['startup_home']=startup_home(observer,home_feedback,owned,out,stop)
        if meta['startup_home']['state']!='HOME_CONFIRMED':
            print('STARTUP HOME: '+('SKIPPED (safety policy)' if meta['startup_home']['state']=='SKIPPED' else 'FAIL / '+meta['startup_home']['state']),flush=True)
            raise RuntimeError('Startup HOME failed: '+meta['startup_home'].get('reason','unknown'))
        print('Startup HOME: PASS'+(' (already within tolerance)' if meta['startup_home']['already_home'] else '')+
              '\nFeedback: '+str(meta['startup_home']['feedback']),flush=True)
        if hardware_mode == 'mock':
            owned.start('mock_imu', [sys.executable, str(SCRIPTS/'j14_demo_mock_imu.py'), '--out-dir', str(out),
                                   '--amplitude-rad',env.get('J14_DEMO_MOCK_IMU_AMPLITUDE','.005')])
        else:
            from ros2run.api import get_executable_path
            imu_executable=get_executable_path(package_name='h30_imu',executable_name='h30_imu_node')
            if imu_executable is None:
                raise RuntimeError('Installed H30 executable unavailable')
            for role in ('base', 'platform'):
                owned.start(role+'_imu', [imu_executable, '--ros-args',
                                         '-r', f'__ns:=/pus6/imu_{role}', '-p', f'port:={PORTS[role]}',
                                         '-p', 'baud_rate:=460800', '-p', f'frame_id:=pus6_{role}_imu_link',
                                         '-p', 'world_frame:=world', '-p', f'imu_topic:=/pus6/imu_{role}/data'])
        wait_ready(lambda: all(time.monotonic()-counts['last_rx'].get(k,0)<0.1 for k in ('base','platform')),
                   'Dual IMU', 10.0, stop, owned, observer)
        owned.start('controller', [sys.executable, str(SCRIPTS/'j14_demo_controller.py'), '--mode', mode,
                                   '--out-dir', str(out), '--hardware-mode', hardware_mode])
        state_path = out / 'controller_state.json'
        wait_ready(lambda: state_path.exists() and json.loads(state_path.read_text())['state']=='ACTIVE',
                   'Controller preflight / ACTIVE', 15.0, stop, owned, observer)
        meta.update(startup='PASS', active=True, active_monotonic_ns=time.monotonic_ns())
        write_json(out/'demo_start.json', meta)
        write_json(out/'command_graph.json', graph(observer))
        print('\n'+'='*60+'\n6PUS ACTIVE STABILIZATION DEMO\n'
              f'Mode: {MODES[mode][1]}\n'
              'Driver: READY\nFeedback: READY\nBase IMU: READY\nPlatform IMU: READY\n'
              'Controller: ACTIVE\nPress Ctrl+C to stop safely.\n'+'='*60, flush=True)
        while not stop.is_set():
            owned.check()
            if json.loads(state_path.read_text())['state'] == 'FAULT':
                raise RuntimeError(json.loads(state_path.read_text())['fault_reason'])
            rclpy.spin_once(observer, timeout_sec=0.05)
        # Faults and unexpected exits win even when SIGINT arrives in the same tick.
        owned.check()
        state=json.loads(state_path.read_text())
        if state['state']=='FAULT':
            raise RuntimeError(state['fault_reason'])
        from types import SimpleNamespace
        from j14_demo_home import feedback_failure
        feedback=counts.get('feedback',dict(steps=[],rx_ns=[],faults=[],currents=[]))
        reason=feedback_failure(SimpleNamespace(driver_stop_latched=False),feedback)
        if reason:
            raise RuntimeError(reason)
        if signal_state['signal']!=signal.SIGINT:
            raise InterruptedError('safety_signal')
        meta['stop_reason'], code = 'operator_stop', 0
    except InterruptedError as exc:
        meta['stop_reason']=str(exc)
        code=1
    except Exception as exc:
        meta['stop_reason'] = repr(exc)
        print(f'DEMO STOP: {exc}', file=sys.stderr, flush=True)
    finally:
        try:
            if owned is not None:
                if 'startup_home' not in meta:
                    meta['startup_home']=dict(state='SKIPPED',phase='startup',reason=meta.get('stop_reason','startup interrupted'),home_target_publish_count=0,end_ns=time.monotonic_ns())
                    write_json(out/'startup_home.json',meta['startup_home'])
                    print('STARTUP HOME: SKIPPED (safety policy)',flush=True)
                normal=bool(meta['active'] and meta.get('stop_reason')=='operator_stop')
                meta['home_recovery']=dict(state='SKIPPED',reason='safety policy' if not normal else 'not started')
                if normal:
                    print('Stopping active stabilization...\nReturning to OPERATING_HOME...',flush=True)
                    try:
                        home=await_home(owned,observer,out,lambda:signal_state['count']>1 or signal_state['signal']!=signal.SIGINT)
                        meta['home_recovery']=home
                        if home['state']=='HOME_CONFIRMED':
                            print(f"HOME recovery: PASS\nFeedback: {home['feedback']} within tolerance",flush=True)
                        else:
                            code=1
                            print(f"HOME recovery: FAIL / {home['state']}",flush=True)
                    except Exception as exc:
                        code=1
                        meta['home_recovery']=dict(state='HOME_ABORTED',reason=repr(exc))
                        print(f'HOME recovery: FAIL / {exc}',flush=True)
                else:
                    print(f"SAFETY STOP\nReason: {meta.get('stop_reason')}\nHOME recovery: SKIPPED (safety policy)",flush=True)
                # Faults and HOME timeout request Pause before any waits on other processes.
                owned.request_controller('safety_pause',meta.get('stop_reason','shutdown'))
                try:
                    owned.pause_driver()
                except Exception as exc:
                    code=1;meta['pause_request_error']=repr(exc)
                pause_rows=rows(out/'stop_pause_audit.jsonl')
                ack=[r for r in (pause_rows[-1].get('records',[]) if pause_rows else [])
                     if r.get('response_received') and not r.get('timeout') and not r.get('error')]
                print(f'6-axis Pause: {len(ack)}/6 ACK\nStopping relay / IMU / ROS runtime...',flush=True)
                owned.cleanup()
                if observer is not None:
                    meta['command_chain'] = counts
                    observer.destroy_node()
                final_path = out/'child_final.json'
                child = json.loads(final_path.read_text()) if final_path.exists() else {}
                pause = rows(out/'stop_pause_audit.jsonl')
                records = pause[-1].get('records', []) if pause else []
                meta.update(child_final=child, pause_ack_ids=[r['motor_id'] for r in records
                            if r.get('response_received') and not r.get('timeout') and not r.get('error')],
                            serial_release=serial_status(), residual_processes=runtime_processes(),
                            hardware_write_count=child.get('hardware_write_count'),
                            stop_monotonic_ns=time.monotonic_ns(), stop_epoch_ns=time.time_ns())
                stops = [e for e in owned.events if e['event']=='stop']
                packets = rows(out/'parent_ipc.jsonl')
                cycles = rows(out/'child_cycles.jsonl')
                parent_sequences = [r['seq'] for r in packets
                    if r.get('event')=='parent_send' and (r.get('seq') or 0)>0]
                child_sequences = [r['command_seq'] for r in cycles if (r.get('command_seq') or 0)>0]
                meta['parent_command_delivery'] = dict(
                    source=['parent_ipc.jsonl','child_cycles.jsonl'],
                    retained_parent_sends=len(parent_sequences),
                    max_parent_seq=max(parent_sequences,default=0),
                    retained_child_command_cycles=len(child_sequences),
                    max_child_seq=max(child_sequences,default=0))
                joined = rows(out/'parent_lifecycle.jsonl')
                meta['driver_join_status'] = {r['event']:r for r in joined
                    if r.get('event') in ('child_joined','bridge_joined')}
                robot_log=out/'robot.log'
                robot_text = robot_log.read_text() if robot_log.exists() else ''
                meta['ros_node_exit_errors'] = [line for line in robot_text.splitlines() if 'process has died' in line]
                meta['legacy_ros_shutdown_exception'] = 'rcl_shutdown already called' in robot_text
                hosts={p.stem:json.loads(p.read_text()) for p in out.glob('*_node_host.json')}
                meta['runtime_host_status']=hosts
                expected_controller_failure=not normal or meta['home_recovery']['state']!='HOME_CONFIRMED'
                meta['safe_shutdown'] = bool(child.get('closed') and child.get('pause_success')
                    and sorted(meta['pause_ack_ids'])==[1,2,3,4,5,6]
                    and meta['driver_join_status'].get('child_joined',{}).get('child_returncode')==0
                    and meta['driver_join_status'].get('bridge_joined',{}).get('bridge_returncode')==0
                    and not any(r.get('event')=='shutdown_escalated_SIGTERM' for r in packets)
                    and not meta['residual_processes'] and meta['serial_release']['free']
                    and all((e.get('returncode')==0 or
                             e['role']=='controller' and e.get('returncode')==1 and expected_controller_failure)
                            and not e['escalation'] for e in stops)
                    and len(hosts)==4 and all(h.get('state')=='exited' and h.get('returncode')==0 for h in hosts.values())
                    and not meta['ros_node_exit_errors'] and not meta['legacy_ros_shutdown_exception'])
                if not meta['safe_shutdown']:
                    code = 1
                write_json(out/'demo_final.json', meta)
                meta['demo_shutdown_pass']=bool(meta['safe_shutdown'] and code==0
                    and (not normal or meta['home_recovery']['state']=='HOME_CONFIRMED'))
                write_json(out/'demo_final.json',meta)
                print(f"Serial release: {'PASS' if meta['serial_release']['free'] else 'FAIL'}\n"
                      f"Residual processes: {len(meta['residual_processes'])}\n"
                      f"Demo shutdown: {'PASS' if meta['demo_shutdown_pass'] else 'FAIL'}",flush=True)
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()
            lock.close()
            for sig, handler in previous_signals.items():
                signal.signal(sig, handler)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--hardware-mode', choices=('mock','active'), default='mock')
    parser.add_argument('--out-dir', type=Path)
    args = parser.parse_args()
    return run_demo(args.mode, args.hardware_mode, args.out_dir)


if __name__ == '__main__':
    sys.exit(main())
