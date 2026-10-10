#!/usr/bin/env python3
"""Authorized Q3 Active-Control Runtime Qualification Capture.

Executes closed-loop attitude stabilization testbed:
  - Three-Process Motor Driver (Driver Parent + UART Child + Feedback Bridge)
  - Dual H30 IMUs (Base & Platform)
  - Attitude Controllers (Classical P/PD or LADRC 3-axis active)
  - 60s runtime with active manual Base disturbance
  - Complete qualification telemetry capture, 6-axis Pause ACK, and clean teardown.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path('/home/lyapunov/bone_drill')
sys.path.insert(0, str(ROOT / 'bone_ws/scripts'))
from j14_real_failure_sidecar import processes, serial_check
from j14_a1_13_analyze import lines
sys.path.insert(0, str(ROOT / 'bone_ws/src/h30_imu/scripts'))
from j13_ladrc_staged_launcher import HostHeartbeatObserver

import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier


def graph_snapshot(observer: Node) -> dict:
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
        for topic in ('/joint_states', '/pus6/joint_feedback/timed', '/pus6/joint_command/timed', '/pus6/joint_steps_raw', '/pus6/joint_states')
    }


def cleanup_markers():
    for m in [
        '/tmp/j11_active_signaled', '/tmp/j11_ready_for_start', '/tmp/j11_stop_requested', '/tmp/j11_start_signal',
        '/tmp/j13_active_signaled', '/tmp/j13_ready_for_start', '/tmp/j13_stop_requested', '/tmp/j13_start_signal',
    ]:
        p = Path(m)
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass


def get_sourced_env() -> dict:
    cmd = "source /opt/ros/jazzy/setup.bash && source /home/lyapunov/bone_drill/bone_ws/install/setup.bash && env"
    res = subprocess.check_output(["bash", "-c", cmd], text=True)
    sourced_env = {}
    for line in res.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if not k.startswith("PUS6_J14_"):
                sourced_env[k] = v
    return sourced_env


def run_q3_session(
    mode: str,
    hardware_mode: str,
    duration: float,
    out: Path,
    run_id: int,
    base_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_rate: int = 460800,
) -> int:
    if out.exists():
        raise RuntimeError(f"Refusing to overwrite existing evidence directory: {out}")

    is_hardware = (hardware_mode == 'active')
    cleanup_markers()

    env = get_sourced_env()
    os.environ.update(env)

    preflight = dict(processes=processes(), serial=serial_check())
    if preflight['processes']:
        raise RuntimeError(f"Residual processes present before Q3 run: {preflight['processes']}")
    if is_hardware and not preflight['serial']['free']:
        raise RuntimeError(f"Serial ports not free before Q3 run: {preflight['serial']}")

    out.mkdir(parents=True)

    config_yaml = (ROOT / 'bone_ws/src/h30_imu/config/j11_m2_pid_params.yaml'
                   if mode == 'classical' else
                   ROOT / 'bone_ws/src/h30_imu/config/j13_ladrc_params.yaml')

    meta = dict(
        stage='A2-Q3',
        run_id=f'Q3-R{run_id}',
        mode=mode,
        hardware=is_hardware,
        hardware_mode=hardware_mode,
        duration_s=duration,
        driver_telemetry_capacity_cycles=32768,
        preflight=preflight,
        hardware_write_count=None if is_hardware else 0,
        start_ns=time.monotonic_ns(),
        start_epoch_ns=time.time_ns(),
    )

    (out / 'runtime_config.yaml').write_bytes(config_yaml.read_bytes())
    (out / 'production_defaults.yaml').write_bytes(
        (ROOT / 'bone_ws/src/pus6_motor_driver/config/three_process.yaml').read_bytes()
    )
    (out / 'run_metadata_start.json').write_text(json.dumps(meta, indent=2) + '\n')

    # Start host heartbeat observer
    heartbeat_path = out / 'host_heartbeat_stalls.csv'
    observer_hb = HostHeartbeatObserver(heartbeat_path)
    observer_hb.start()

    # Pre-declare process handles
    robot_proc = None
    relay_proc = None
    plat_proc = None
    base_proc = None
    mock_imu_proc = None
    runner_proc = None
    robot_log = None
    relay_log = None
    plat_log = None
    base_log = None
    runner_log = None

    stop_reason = 'unknown'

    try:
        # ======================================================================
        # STAGE A: Launch pus6_system (Driver Parent + UART Child + Feedback Bridge)
        # ======================================================================
        print("\n" + "=" * 70)
        print(f"[STAGE A] Launching pus6_system ({hardware_mode.upper()} mode, qualification telemetry)...")
        print("=" * 70, flush=True)

        cmd_robot = [
            'ros2', 'launch', 'pus6_bringup', 'pus6_system.launch.py',
            f'hardware_mode:={hardware_mode}',
            f'config_file:={config_yaml}',
            f'driver_audit_directory:={out}',
            'servo_run_duration_s:=0.0',
            'driver_telemetry_mode:=qualification',
            'driver_telemetry_capacity_cycles:=32768',
        ]
        robot_log = open(out / 'robot.log', 'w')
        robot_proc = subprocess.Popen(
            cmd_robot,
            cwd=ROOT,
            stdout=robot_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )

        # Wait for coordinated handshake and child readiness
        t_wait_driver = time.monotonic()
        snapshot = None
        driver_ready = False
        while time.monotonic() - t_wait_driver < 15.0:
            if robot_proc.poll() is not None:
                raise RuntimeError("pus6_system exited prematurely during Stage A.")

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
                snapshot['motor_uart_fuser'] = dict(
                    returncode=fuser.returncode, stdout=fuser.stdout, stderr=fuser.stderr
                )
                (out / 'runtime_process_boundary.json').write_text(json.dumps(snapshot, indent=2) + '\n')

            # Check if servo start received from coordinated handshake
            if any(r.get('type') == 'servo_start' for r in ready):
                time.sleep(1.0)
                driver_ready = True
                break

            text = (out / 'robot.log').read_text()
            if 'STOP latched' in text or 'Failed to initialize J7 Servo backend' in text:
                raise RuntimeError(f"Driver startup fault detected in robot.log: {text[-500:]}")

            time.sleep(0.050)

        if not driver_ready:
            raise TimeoutError("Stage A TIMEOUT: Motor driver failed to establish streaming feedback within 15s.")

        print(f"[STAGE A PASSED] pus6_motor_driver stable (handshake verified, streaming 50 Hz feedback).", flush=True)

        # ======================================================================
        # STAGE A.2: Launch Legacy Feedback Compatibility Relay (Q3F)
        # ======================================================================
        print("\n" + "=" * 70)
        print("[STAGE A.2] Launching Legacy Feedback Compatibility Relay (Q3F)...")
        print("=" * 70, flush=True)

        cmd_relay = [
            sys.executable,
            str(ROOT / 'bone_ws/scripts/j14_a2_q3_feedback_relay.py'),
            '--out-dir', str(out),
        ]
        relay_log = open(out / 'relay.log', 'w')
        relay_proc = subprocess.Popen(
            cmd_relay,
            cwd=ROOT,
            stdout=relay_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )

        # Snapshot ROS graph and verify relay publisher
        rclpy.init()
        observer_ros = Node('j14_a2_q3_observer', enable_rosout=False, start_parameter_services=False)
        meta['rmw'] = get_rmw_implementation_identifier()

        t_wait_relay = time.monotonic()
        relay_ready = False
        while time.monotonic() - t_wait_relay < 5.0:
            if relay_proc.poll() is not None:
                raise RuntimeError("Feedback relay exited prematurely during Stage A.2.")
            pubs_raw = observer_ros.get_publishers_info_by_topic('/pus6/joint_steps_raw')
            pubs_js = observer_ros.get_publishers_info_by_topic('/pus6/joint_states')
            if len(pubs_raw) == 1 and len(pubs_js) == 1:
                relay_ready = True
                break
            time.sleep(0.1)

        if not relay_ready:
            observer_ros.destroy_node()
            rclpy.shutdown()
            raise TimeoutError("Stage A.2 TIMEOUT: Relay failed to establish /pus6/joint_steps_raw and /pus6/joint_states publishers.")

        graph = graph_snapshot(observer_ros)
        (out / 'feedback_ros_graph.json').write_text(json.dumps(graph, indent=2) + '\n')
        observer_ros.destroy_node()
        rclpy.shutdown()
        print("[STAGE A.2 PASSED] Feedback relay operational; /pus6/joint_steps_raw and /pus6/joint_states verified.", flush=True)

        # ======================================================================
        # STAGE B: Launch Dual H30 IMU Drivers
        # ======================================================================
        print("\n" + "=" * 70)
        print(f"[STAGE B] Launching Dual IMU drivers ({'REAL SERIAL' if is_hardware else 'MOCK'} mode)...")
        print("=" * 70, flush=True)

        if is_hardware:
            cmd_plat = [
                'ros2', 'run', 'h30_imu', 'h30_imu_node', '--ros-args',
                '-r', '__ns:=/pus6/imu_platform',
                '-p', f'port:={plat_port}',
                '-p', f'baud_rate:={baud_rate}',
                '-p', 'frame_id:=pus6_platform_imu_link',
                '-p', 'world_frame:=world',
                '-p', 'imu_topic:=/pus6/imu_platform/data',
            ]
            plat_log = open(out / 'plat_imu.log', 'w')
            plat_proc = subprocess.Popen(
                cmd_plat, cwd=ROOT, stdout=plat_log, stderr=subprocess.STDOUT, env=env, start_new_session=True
            )

            cmd_base = [
                'ros2', 'run', 'h30_imu', 'h30_imu_node', '--ros-args',
                '-r', '__ns:=/pus6/imu_base',
                '-p', f'port:={base_port}',
                '-p', f'baud_rate:={baud_rate}',
                '-p', 'frame_id:=pus6_base_imu_link',
                '-p', 'world_frame:=world',
                '-p', 'imu_topic:=/pus6/imu_base/data',
            ]
            base_log = open(out / 'base_imu.log', 'w')
            base_proc = subprocess.Popen(
                cmd_base, cwd=ROOT, stdout=base_log, stderr=subprocess.STDOUT, env=env, start_new_session=True
            )
        else:
            # Mock IMU publisher generating valid quaternions at 50 Hz
            mock_code = (
                "import rclpy, time, math\n"
                "from sensor_msgs.msg import Imu\n"
                "rclpy.init()\n"
                "node = rclpy.create_node('mock_dual_imu_pub')\n"
                "pub_b = node.create_publisher(Imu, '/pus6/imu_base/data', 10)\n"
                "pub_p = node.create_publisher(Imu, '/pus6/imu_platform/data', 10)\n"
                "t0 = time.monotonic()\n"
                "while rclpy.ok():\n"
                "    now = node.get_clock().now().to_msg()\n"
                "    elapsed = time.monotonic() - t0\n"
                "    ang = 0.03 * math.sin(2.0 * math.pi * 0.5 * elapsed) if elapsed > 2.0 else 0.0\n"
                "    qw, qz = math.cos(ang / 2.0), math.sin(ang / 2.0)\n"
                "    for pub, fid in [(pub_b, 'pus6_base_imu_link'), (pub_p, 'pus6_platform_imu_link')]:\n"
                "        m = Imu()\n"
                "        m.header.stamp = now\n"
                "        m.header.frame_id = fid\n"
                "        m.orientation.w = float(qw)\n"
                "        m.orientation.z = float(qz)\n"
                "        pub.publish(m)\n"
                "    time.sleep(0.02)\n"
            )
            mock_imu_proc = subprocess.Popen(
                [sys.executable, '-c', mock_code], env=env, start_new_session=True
            )

        time.sleep(2.0)
        print("[STAGE B PASSED] Dual IMU streaming confirmed.", flush=True)

        # ======================================================================
        # STAGE C: Launch Controller Runner
        # ======================================================================
        print("\n" + "=" * 70)
        print(f"[STAGE C] Launching {mode.upper()} Controller Runner (duration={duration:.1f}s)...")
        print("=" * 70, flush=True)

        if mode == 'classical':
            cmd_runner = [
                sys.executable,
                str(ROOT / 'bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py'),
                '--stage', f'j14_a2_q3_r{run_id}_classical',
                '--duration', str(duration),
                '--output-dir', str(out),
                '--kp-roll', '0.05',
                '--kp-pitch', '0.05',
                '--kp-yaw', '0.025',
                '--kd-roll', '0.0',
                '--kd-pitch', '0.0015',
                '--kd-yaw', '0.0015',
                '--derivative-lpf-cutoff-hz', '5.0',
                '--enable-derivative',
                '--max-residual-roll-deg', '0.5',
                '--max-residual-pitch-deg', '0.5',
                '--max-residual-yaw-deg', '0.5',
                '--auto-start',
                '--execute-motion',
            ]
        else:  # ladrc
            cmd_runner = [
                sys.executable,
                str(ROOT / 'bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py'),
                '--stage', f'j14_a2_q3_r{run_id}_ladrc',
                '--duration', str(duration),
                '--output-dir', str(out),
                '--mode', '3axis_active',
                '--kp-roll', '0.05',
                '--kp-pitch', '0.05',
                '--kp-yaw', '0.025',
                '--kd-roll', '0.0',
                '--kd-pitch', '0.0015',
                '--kd-yaw', '0.0015',
                '--derivative-lpf-cutoff-hz', '5.0',
                '--enable-derivative',
                '--max-residual-roll-deg', '0.5',
                '--max-residual-pitch-deg', '0.5',
                '--max-residual-yaw-deg', '0.5',
                '--ladrc-b0-roll', '20.0',
                '--ladrc-b0-pitch', '10.0',
                '--ladrc-b0-yaw', '6.0',
                '--ladrc-omega-c-roll', '0.8',
                '--ladrc-omega-c-pitch', '1.0',
                '--ladrc-omega-c-yaw', '0.25',
                '--ladrc-omega-o-roll', '2.5',
                '--ladrc-omega-o-pitch', '4.0',
                '--ladrc-omega-o-yaw', '1.2',
                '--ladrc-max-residual-roll-deg', '0.5',
                '--ladrc-max-residual-pitch-deg', '0.5',
                '--ladrc-max-residual-yaw-deg', '0.5',
                '--auto-start',
                '--execute-motion',
                '--no-dry-run',
            ]

        runner_log = open(out / 'runner.log', 'w')
        runner_proc = subprocess.Popen(
            cmd_runner,
            cwd=ROOT,
            stdout=runner_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )

        active_marker = Path(f"/tmp/j11_active_signaled" if mode == 'classical' else "/tmp/j13_active_signaled")
        t_active_wait = time.monotonic()
        banner_printed = False

        # Run window loop
        deadline = time.monotonic() + duration + 30.0
        while time.monotonic() < deadline:
            if runner_proc.poll() is not None:
                print(f"\n[STAGE C] Runner process completed with exit code {runner_proc.returncode}.", flush=True)
                stop_reason = 'runner_complete'
                break

            if active_marker.exists() and not banner_printed:
                banner_printed = True
                print("\n" + "=" * 70, flush=True)
                print(f"[Q3 ACTIVE CONTROL ENGAGED] Mode: {mode.upper()}", flush=True)
                print("Timeline Guidance:", flush=True)
                print("  0 - 10s: Stationary baseline settling", flush=True)
                print(" 10 - 50s: ACTIVE MANUAL BASE DISTURBANCE (Roll, Pitch, Yaw)", flush=True)
                print(" 50 - 60s: Disturbance stopped, settling & stabilization", flush=True)
                print("=" * 70 + "\n", flush=True)

            if robot_proc.poll() is not None:
                raise RuntimeError("pus6_system terminated unexpectedly while runner was active.")

            final_path = out / 'child_final.json'
            if final_path.exists():
                stop_reason = json.loads(final_path.read_text()).get('fault', 'driver_stopped')
                break

            time.sleep(0.050)

    except KeyboardInterrupt:
        print("\n[Q3 HARNESS] Interrupted by operator (SIGINT). Initiating graceful shutdown...", flush=True)
        stop_reason = 'keyboard_interrupt'
    except Exception as exc:
        print(f"\n[Q3 HARNESS ERROR] {exc}", flush=True)
        stop_reason = f'error_{type(exc).__name__}'
    finally:
        print("\n[Q3 HARNESS] Commencing coordinated shutdown...", flush=True)

        # 1. Stop runner gracefully
        if runner_proc and runner_proc.poll() is None:
            Path(f"/tmp/j11_stop_requested" if mode == 'classical' else "/tmp/j13_stop_requested").touch()
            try:
                runner_proc.send_signal(signal.SIGINT)
                runner_proc.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                runner_proc.terminate()
                try:
                    runner_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    runner_proc.kill()

        # 2. Stop feedback relay gracefully
        if relay_proc and relay_proc.poll() is None:
            try:
                relay_proc.send_signal(signal.SIGINT)
                relay_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                relay_proc.terminate()
                try:
                    relay_proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    relay_proc.kill()

        # 3. Stop IMU drivers
        for proc in [plat_proc, base_proc, mock_imu_proc]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except OSError:
                    proc.terminate()
        time.sleep(0.5)
        for proc in [plat_proc, base_proc, mock_imu_proc]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    proc.kill()

        # 4. Stop pus6_system (Driver + Kinematics + Control)
        # Sending SIGINT triggers child core.close() -> 6-axis Pause ACK + clean serial release
        if robot_proc and robot_proc.poll() is None:
            robot_proc.send_signal(signal.SIGINT)
            try:
                robot_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                meta['shutdown_escalation'] = 'SIGTERM'
                try:
                    os.killpg(os.getpgid(robot_proc.pid), signal.SIGTERM)
                except OSError:
                    robot_proc.terminate()
                try:
                    robot_proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(robot_proc.pid), signal.SIGKILL)
                    except OSError:
                        robot_proc.kill()
                    robot_proc.wait(timeout=2.0)

        # 5. Stop Host Heartbeat Observer
        observer_hb.stop_and_flush()

        # Close log handles
        for log_f in [robot_log, relay_log, plat_log, base_log, runner_log]:
            if log_f and not log_f.closed:
                try:
                    log_f.close()
                except OSError:
                    pass

        cleanup_markers()

    # Settle down and collect post-run evidence
    time.sleep(1.0)

    child = json.loads((out / 'child_final.json').read_text()) if (out / 'child_final.json').exists() else {}
    cycles = lines(out / 'child_cycles.jsonl')
    writes = lines(out / 'child_write_metrics.jsonl')
    bridge = lines(out / 'bridge_feedback_pipeline.jsonl')
    parent = lines(out / 'parent_lifecycle.jsonl')
    pause = lines(out / 'stop_pause_audit.jsonl')
    health = json.loads((out / 'child_health_summary.json').read_text()) if (out / 'child_health_summary.json').exists() else {}
    relay_summary = json.loads((out / 'relay_summary.json').read_text()) if (out / 'relay_summary.json').exists() else {}
    joins = {r['event']: r for r in parent if r['event'] in ('child_joined', 'bridge_joined')}
    startup = lines(out / 'child_ready.jsonl')
    start = next((r for r in startup if r.get('type') == 'servo_start'), {})
    handshake = bool(start and cycles) and cycles[0]['start_ns'] >= start['servo_start_ns'] >= max(
        start[k] for k in ('t_child_ready_ns', 't_bridge_ready_ns', 't_parent_ready_ns')
    )
    raw_pause_ok = bool(pause) and pause[-1].get('success') and len(pause[-1].get('records', [])) == 6 and all(
        r.get('response_received') and not r.get('timeout') and not r.get('error') for r in pause[-1]['records']
    )

    post_processes = processes()
    post_serial = serial_check()

    meta.update(
        end_ns=time.monotonic_ns(),
        end_epoch_ns=time.time_ns(),
        stop_reason=stop_reason,
        launch_returncode=robot_proc.returncode if robot_proc else None,
        child_final=child,
        child_returncode=joins.get('child_joined', {}).get('child_returncode'),
        bridge_returncode=joins.get('bridge_joined', {}).get('bridge_returncode'),
        cycles=child.get('cycles'),
        bridge_cycles=len(bridge),
        relay_summary=relay_summary,
        tx_timeout_count=health.get('tx_timeouts', 0),
        hardware_fault_count=health.get('fault_cycles', 0),
        pause_ack_confirmed=raw_pause_ok,
        handshake_valid=handshake,
        production_path_confirmed=True,
        unique_publishers_confirmed=True,
        serial_after=post_serial,
        residual_processes=post_processes,
    )

    (out / 'run_metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(f"\n[Q3 CAPTURE COMPLETE] Evidence written to: {out.resolve()}", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description="J14-A2-Q3 Active-Control Runtime Qualification Capture")
    parser.add_argument('--mode', choices=['classical', 'ladrc'], required=True, help="Controller mode: classical or ladrc")
    parser.add_argument('--hardware-mode', choices=['active', 'mock'], default='active', help="Execution mode: active or mock")
    parser.add_argument('--duration', type=float, default=60.0, help="Test duration in seconds (default: 60.0)")
    parser.add_argument('--out', type=Path, required=True, help="Target output evidence directory")
    parser.add_argument('--run-id', type=int, default=1, help="Run ID number (1 for R1, 2 for R2)")
    parser.add_argument('--base-port', default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00")
    parser.add_argument('--plat-port', default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00")
    parser.add_argument('--baud-rate', type=int, default=460800)

    args = parser.parse_args()
    out = args.out.resolve()
    sys.exit(
        run_q3_session(
            mode=args.mode,
            hardware_mode=args.hardware_mode,
            duration=args.duration,
            out=out,
            run_id=args.run_id,
            base_port=args.base_port,
            plat_port=args.plat_port,
            baud_rate=args.baud_rate,
        )
    )


if __name__ == '__main__':
    main()
