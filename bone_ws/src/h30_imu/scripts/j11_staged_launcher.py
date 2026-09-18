#!/usr/bin/env python3
"""Stage J11: Staged Startup Orchestrator for Base-IMU Feedforward Attitude Hold.

Implements staged startup separation:
Stage A: Launch pus6_system -> Wait for driver stable (feedback continuous >= 3.0s, no STOP latch, 0 faults)
Stage B: Launch Dual H30 IMUs -> Wait for dual IMU stable (age < 100ms, continuous >= 2.0s)
Verify:  Confirm mechanical OPERATING_HOME within 15 steps
Stage C: Launch j11_base_feedforward_runner (without --start-robot/--start-imu) -> Wait for ACTIVE

Displays banner:
========================================
J11 ACTIVE
现在可以开始录像并移动 Base
========================================
"""

import argparse
import datetime
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.msg import Log


class StagePreflightMonitor(Node):
    def __init__(self):
        super().__init__("j11_stage_preflight_monitor")
        self.node_start_time = time.time()
        self.latest_steps: Optional[List[int]] = None
        self.latest_faults: Optional[List[int]] = None
        self.last_fb_time: float = 0.0
        self.driver_stop_latched: bool = False
        self.driver_stop_reason: str = ""

        self.last_base_imu_time: float = 0.0
        self.last_plat_imu_time: float = 0.0
        self.has_base_imu: bool = False
        self.has_plat_imu: bool = False

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_rosout = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Int32MultiArray, "/pus6/joint_steps_raw", self._on_steps, qos_sensor)
        self.create_subscription(Int32MultiArray, "/pus6/joint_faults_raw", self._on_faults, qos_sensor)
        self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_sensor)
        self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_sensor)
        self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)

    def _on_steps(self, msg: Int32MultiArray):
        if len(msg.data) >= 6:
            self.latest_steps = list(msg.data[:6])
            self.last_fb_time = time.time()

    def _on_faults(self, msg: Int32MultiArray):
        if len(msg.data) >= 6:
            self.latest_faults = list(msg.data[:6])

    def _on_base_imu(self, msg: Imu):
        self.last_base_imu_time = time.time()
        self.has_base_imu = True

    def _on_plat_imu(self, msg: Imu):
        self.last_plat_imu_time = time.time()
        self.has_plat_imu = True

    def _on_rosout(self, msg: Log):
        if "pus6_motor_driver" in msg.name and msg.level >= 40:
            msg_t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if msg_t > 0 and msg_t < self.node_start_time:
                return
            if any(k in msg.msg for k in ["STOP latched", "scheduler_deadline_exceeded"]):
                self.driver_stop_latched = True
                self.driver_stop_reason = msg.msg


def cleanup_stale_processes_and_markers():
    """Clean up any old J11 or pus6 processes and markers before starting."""
    print("[STAGED STARTUP] Cleaning up stale markers and processes...", flush=True)
    for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_driver_stopped", "/tmp/j11_stop_requested", "/tmp/j11_start_signal"]:
        p = Path(marker)
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass
    subprocess.run(["pkill", "-9", "-f", "j11_base"], stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def run_staged_session(
    duration: float = 300.0,
    stage: str = "j11_video_manual",
    output_dir: Optional[str] = None,
    config_file: str = "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j11_feedforward_params.yaml",
    base_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_rate: int = 460800,
    dry_run: bool = False,
    execute_motion: bool = True,
    servo_scheduler_mode: str = "dedicated_thread",
    telemetry_publish_mode: str = "async_latest",
    feedback_loss_policy: str = "age_based",
):
    cleanup_stale_processes_and_markers()

    if output_dir is None or output_dir == "results/j11/j11_video_manual":
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"results/j11/runtime_validation/{timestamp}"

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    try:
        Path("/tmp/j11_latest_run_dir").write_text(str(out_path.resolve()))
    except OSError:
        pass
    print(f"[STAGED STARTUP] Output directory: {out_path.resolve()}", flush=True)

    setup_bash = "/home/lyapunov/bone_drill/bone_ws/install/setup.bash"
    robot_proc = None
    plat_proc = None
    base_proc = None
    runner_proc = None
    event_logger_proc = None

    rclpy.init()
    monitor = StagePreflightMonitor()

    def kill_all_subprocesses():
        for marker in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_stop_requested"]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass
        for proc, name in [
            (runner_proc, "runner"),
            (plat_proc, "plat_imu"),
            (base_proc, "base_imu"),
            (robot_proc, "robot"),
            (event_logger_proc, "event_logger"),
        ]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=2.0)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass

    try:
        # Launch independent high-rate event logger
        cmd_logger = [
            "python3",
            "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_runtime_event_logger.py",
            "--output-dir", str(out_path.resolve()),
        ]
        logger_log = open(out_path / "event_logger_stdout.log", "w")
        event_logger_proc = subprocess.Popen(cmd_logger, stdout=logger_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        # ==========================================================
        # STAGE A: Launch pus6_system and wait for driver stable
        # ==========================================================
        print("\n" + "=" * 70)
        print("[STAGE A] Launching pus6_system (hardware_mode=active)...")
        print("=" * 70, flush=True)

        cmd_robot = [
            "bash", "-c",
            f"source {setup_bash} && "
            f"exec ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=active config_file:={config_file} servo_scheduler_mode:={servo_scheduler_mode} telemetry_publish_mode:={telemetry_publish_mode} feedback_loss_policy:={feedback_loss_policy}"
        ]
        robot_log = open("/tmp/j11_robot_launch.log", "w")
        robot_proc = subprocess.Popen(cmd_robot, stdout=robot_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        t_start_a = time.monotonic()
        stable_a_start: Optional[float] = None
        driver_ok = False

        while time.monotonic() - t_start_a < 15.0:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now = time.time()
            if monitor.driver_stop_latched:
                raise RuntimeError(f"Stage A FAIL: Driver STOP latched during startup: {monitor.driver_stop_reason}")

            fb_fresh = (monitor.last_fb_time > 0 and (now - monitor.last_fb_time) < 0.200)
            faults_ok = (monitor.latest_faults is not None and all(f == 0 for f in monitor.latest_faults))

            if fb_fresh and faults_ok:
                if stable_a_start is None:
                    stable_a_start = time.monotonic()
                elif time.monotonic() - stable_a_start >= 3.0:
                    driver_ok = True
                    break
            else:
                stable_a_start = None

        if not driver_ok:
            raise TimeoutError("Stage A TIMEOUT: Motor driver feedback failed to achieve 3.0s sustained stability within 15s.")

        print(f"[STAGE A PASSED] Robot driver stable (feedback continuous >= 3.0s, latest steps={monitor.latest_steps}).", flush=True)

        # ==========================================================
        # STAGE B: Launch Dual H30 IMUs and wait for stable streaming
        # ==========================================================
        print("\n" + "=" * 70)
        print("[STAGE B] Launching Platform and Base H30 IMU nodes...")
        print("=" * 70, flush=True)

        if not Path(plat_port).exists() or not Path(base_port).exists():
            raise FileNotFoundError(f"IMU serial port missing: base={base_port}, plat={plat_port}")

        cmd_plat = [
            "bash", "-c",
            f"source {setup_bash} && "
            f"exec ros2 run h30_imu h30_imu_node --ros-args "
            f"-r __ns:=/pus6/imu_platform "
            f"-p port:={plat_port} "
            f"-p baud_rate:={baud_rate} "
            f"-p frame_id:=pus6_platform_imu_link "
            f"-p world_frame:=world "
            f"-p imu_topic:=/pus6/imu_platform/data"
        ]
        plat_log = open("/tmp/j11_plat_imu.log", "w")
        plat_proc = subprocess.Popen(cmd_plat, stdout=plat_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        cmd_base = [
            "bash", "-c",
            f"source {setup_bash} && "
            f"exec ros2 run h30_imu h30_imu_node --ros-args "
            f"-r __ns:=/pus6/imu_base "
            f"-p port:={base_port} "
            f"-p baud_rate:={baud_rate} "
            f"-p frame_id:=pus6_base_imu_link "
            f"-p world_frame:=world "
            f"-p imu_topic:=/pus6/imu_base/data"
        ]
        base_log = open("/tmp/j11_base_imu.log", "w")
        base_proc = subprocess.Popen(cmd_base, stdout=base_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        t_start_b = time.monotonic()
        stable_b_start: Optional[float] = None
        imu_ok = False

        while time.monotonic() - t_start_b < 10.0:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now = time.time()
            if monitor.driver_stop_latched:
                raise RuntimeError(f"Stage B FAIL: Driver STOP latched while starting IMUs: {monitor.driver_stop_reason}")

            base_fresh = (monitor.has_base_imu and (now - monitor.last_base_imu_time) < 0.100)
            plat_fresh = (monitor.has_plat_imu and (now - monitor.last_plat_imu_time) < 0.100)

            if base_fresh and plat_fresh:
                if stable_b_start is None:
                    stable_b_start = time.monotonic()
                elif time.monotonic() - stable_b_start >= 2.0:
                    imu_ok = True
                    break
            else:
                stable_b_start = None

        if not imu_ok:
            raise TimeoutError("Stage B TIMEOUT: Dual IMUs failed to achieve 2.0s sustained streaming (<100ms age) within 10s.")

        print("[STAGE B PASSED] Dual H30 IMUs streaming stably (Base & Plat age < 100ms for >= 2.0s).", flush=True)

        # ==========================================================
        # Mechanical OPERATING_HOME Verification
        # ==========================================================
        if monitor.latest_steps is None:
            raise RuntimeError("No feedback steps received from driver.")
        errors = [abs(s - 1000) for s in monitor.latest_steps]
        max_err = max(errors)
        if max_err > 15:
            raise ValueError(f"Preflight FAIL: Motors not at OPERATING_HOME [1000]*6 (actual={monitor.latest_steps}, max err={max_err} > 15 steps)")

        print(f"[PREFLIGHT OK] Verified 6 axes at OPERATING_HOME: {monitor.latest_steps} (max error={max_err} steps <= 15 steps).", flush=True)

        # Destroy monitor node before launching runner so no competing subscribers interfere
        monitor.destroy_node()
        rclpy.shutdown()

        # ==========================================================
        # STAGE C: Launch J11 Base Feedforward Runner
        # ==========================================================
        print("\n" + "=" * 70)
        print("[STAGE C] Launching J11 Feedforward Runner...")
        print("=" * 70, flush=True)

        cmd_runner = [
            "python3",
            "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py",
            "--stage", stage,
            "--duration", str(duration),
            "--output-dir", output_dir,
            "--auto-start",
        ]
        if execute_motion and not dry_run:
            cmd_runner.append("--execute-motion")
        else:
            cmd_runner.append("--dry-run")

        runner_proc = subprocess.Popen(cmd_runner, preexec_fn=os.setsid)

        # Wait for runner to signal ACTIVE
        active_marker = Path("/tmp/j11_active_signaled")
        t_wait_active = time.monotonic()
        banner_printed = False

        while runner_proc.poll() is None:
            if active_marker.exists() and not banner_printed:
                banner_printed = True
                print("\n" + "=" * 50, flush=True)
                print("J11 ACTIVE: STATIC HOME HOLD IN PROGRESS", flush=True)
                print("请保持 Base 完全静止 (DIAGNOSTIC RUNNING)", flush=True)
                print("=" * 50 + "\n", flush=True)
            if not banner_printed and (time.monotonic() - t_wait_active > 15.0):
                print("[WARNING] Active marker not seen after 15s. Continuing to monitor runner...", flush=True)
            time.sleep(0.1)

        ret = runner_proc.poll()
        print(f"\n[STAGED STARTUP] J11 runner exited with code {ret}.", flush=True)

    except KeyboardInterrupt:
        print("\n[STAGED STARTUP] Interrupted by user.", flush=True)
    except Exception as exc:
        print(f"\n[STAGED STARTUP ERROR] {exc}", flush=True)
    finally:
        print("[STAGED STARTUP] Shutting down subprocesses...", flush=True)
        kill_all_subprocesses()
        print("[STAGED STARTUP] All processes cleanly terminated.", flush=True)


def main():
    parser = argparse.ArgumentParser(description="J11 Staged Startup Orchestrator")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--stage", default="j11_video_manual")
    parser.add_argument("--output-dir", default=None, help="Directory to save results (defaults to unique timestamped dir)")
    parser.add_argument("--config-file", default="/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j11_feedforward_params.yaml")
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--execute-motion", action="store_true", default=False)
    parser.add_argument("--servo-scheduler-mode", default="dedicated_thread", choices=["ros_timer", "dedicated_thread"], help="Servo loop scheduling mode")
    parser.add_argument("--telemetry-publish-mode", default="async_latest", choices=["auto", "inline", "async_latest"], help="Telemetry publication mode")
    parser.add_argument("--feedback-loss-policy", default="age_based", choices=["default", "consecutive_misses", "age_based"], help="Feedback loss safety policy")
    args = parser.parse_args()

    run_staged_session(
        duration=args.duration,
        stage=args.stage,
        output_dir=args.output_dir,
        config_file=args.config_file,
        dry_run=args.dry_run,
        execute_motion=args.execute_motion,
        servo_scheduler_mode=args.servo_scheduler_mode,
        telemetry_publish_mode=args.telemetry_publish_mode,
        feedback_loss_policy=args.feedback_loss_policy,
    )


if __name__ == "__main__":
    main()
