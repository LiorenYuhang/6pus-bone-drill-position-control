#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J13-2A2: Staged Startup Orchestrator for Real-Hardware LADRC Shadow Run.

Orchestration sequence:
  Stage A: Launch pus6_system -> Wait for driver stable (feedback continuous >= 3.0s, 0 faults)
  Stage B: Launch Dual H30 IMUs -> Wait for dual IMU stable (age < 100ms, continuous >= 2.0s)
  Verify:  Confirm mechanical OPERATING_HOME within 15 steps of [1000]*6
  Stage C: Launch j13_ladrc_shadow_runner.py in powered execution or dry-run -> Wait for ACTIVE

Safety:
  - 100% Classical P/PD real control authority.
  - LADRC runs in Shadow Mode (LOG ONLY, ZERO CONTROL AUTHORITY).
  - Clean subprocess lifecycle management and signal handlers.
"""

from __future__ import annotations

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
    """Monitors motor driver and dual IMUs before launching the attitude runner."""

    def __init__(self) -> None:
        super().__init__("j13_preflight_monitor")
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

        self.create_subscription(
            Int32MultiArray, "/pus6/joint_steps_raw", self._on_steps, qos_sensor
        )
        self.create_subscription(
            Int32MultiArray, "/pus6/joint_faults_raw", self._on_faults, qos_sensor
        )
        self.create_subscription(Imu, "/pus6/imu_base/data", self._on_base_imu, qos_sensor)
        self.create_subscription(Imu, "/pus6/imu_platform/data", self._on_plat_imu, qos_sensor)
        self.create_subscription(Log, "/rosout", self._on_rosout, qos_rosout)

    def _on_steps(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_steps = list(msg.data[:6])
            self.last_fb_time = time.time()

    def _on_faults(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_faults = list(msg.data[:6])

    def _on_base_imu(self, msg: Imu) -> None:
        self.last_base_imu_time = time.time()
        self.has_base_imu = True

    def _on_plat_imu(self, msg: Imu) -> None:
        self.last_plat_imu_time = time.time()
        self.has_plat_imu = True

    def _on_rosout(self, msg: Log) -> None:
        if "pus6_motor_driver" in msg.name and msg.level >= 40:
            msg_t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if 0 < msg_t < self.node_start_time:
                return
            if any(
                k in msg.msg
                for k in [
                    "STOP latched",
                    "scheduler_deadline_exceeded",
                    "overcurrent",
                    "Timeout recovery target rejected",
                    "reconnect_max_jump_steps",
                ]
            ):
                self.driver_stop_latched = True
                self.driver_stop_reason = msg.msg


def cleanup_stale_processes_and_markers() -> None:
    """Clean up stale markers and runner processes."""
    print("[J13 STAGED STARTUP] Cleaning up stale markers and processes...", flush=True)
    for marker in [
        "/tmp/j13_active_signaled",
        "/tmp/j13_ready_for_start",
        "/tmp/j13_driver_stopped",
        "/tmp/j13_stop_requested",
        "/tmp/j13_start_signal",
        "/tmp/j11_active_signaled",
        "/tmp/j11_ready_for_start",
        "/tmp/j11_driver_stopped",
        "/tmp/j11_stop_requested",
        "/tmp/j11_start_signal",
    ]:
        p = Path(marker)
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass
    subprocess.run(["pkill", "-9", "-f", "j13_ladrc_shadow_runner"], stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def run_j13_staged_session(
    duration: float = 30.0,
    stage: str = "j13_2a2_static_shadow",
    output_dir: Optional[str] = None,
    config_file: str = "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml",
    base_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00",
    plat_port: str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00",
    baud_rate: int = 460800,
    dry_run: bool = False,
    execute_motion: bool = True,
    servo_scheduler_mode: str = "dedicated_thread",
    telemetry_publish_mode: str = "async_latest",
    feedback_loss_policy: str = "age_based",
    # Classical PID Gains
    kp_roll: float = 0.05,
    kp_pitch: float = 0.05,
    kp_yaw: float = 0.025,
    kd_roll: float = 0.0,
    kd_pitch: float = 0.0015,
    kd_yaw: float = 0.0015,
    derivative_enabled: bool = True,
    derivative_lpf_cutoff_hz: float = 5.0,
    max_residual_roll_deg: float = 0.5,
    max_residual_pitch_deg: float = 0.5,
    max_residual_yaw_deg: float = 0.5,
    # LADRC Parameters
    ladrc_b0_roll: float = 10.0,
    ladrc_b0_pitch: float = 10.0,
    ladrc_b0_yaw: float = 3.0,
    ladrc_omega_c_roll: float = 1.0,
    ladrc_omega_c_pitch: float = 1.0,
    ladrc_omega_c_yaw: float = 0.35,
    ladrc_omega_o_roll: float = 4.0,
    ladrc_omega_o_pitch: float = 4.0,
    ladrc_omega_o_yaw: float = 1.8,
) -> None:
    cleanup_stale_processes_and_markers()

    if output_dir is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"/home/lyapunov/bone_drill/results/j13/j13_2a2_shadow/{timestamp}/{stage}"

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    try:
        Path("/tmp/j13_latest_run_dir").write_text(str(out_path.resolve()))
    except OSError:
        pass
    print(f"[J13 STAGED STARTUP] Output directory: {out_path.resolve()}", flush=True)

    setup_bash = "/home/lyapunov/bone_drill/bone_ws/install/setup.bash"
    robot_proc = None
    plat_proc = None
    base_proc = None
    runner_proc = None

    if not rclpy.ok():
        rclpy.init()
    monitor = StagePreflightMonitor()

    def kill_all_subprocesses():
        for marker in [
            "/tmp/j13_active_signaled",
            "/tmp/j13_ready_for_start",
            "/tmp/j13_stop_requested",
        ]:
            try:
                Path(marker).unlink(missing_ok=True)
            except OSError:
                pass
        for proc, name in [
            (runner_proc, "J13 Runner"),
            (plat_proc, "Platform IMU"),
            (base_proc, "Base IMU"),
            (robot_proc, "Pus6 System"),
        ]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except OSError:
                    pass
        time.sleep(1.0)
        for proc, name in [
            (runner_proc, "J13 Runner"),
            (plat_proc, "Platform IMU"),
            (base_proc, "Base IMU"),
            (robot_proc, "Pus6 System"),
        ]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass

    try:
        # ==========================================================
        # STAGE A: Launch pus6_system & Validate Driver Stability
        # ==========================================================
        print("\n" + "=" * 70)
        print("[STAGE A] Launching pus6_system (Motor Driver, Kinematics, Control)...")
        print("=" * 70, flush=True)

        cmd_robot = [
            "bash",
            "-c",
            f"source {setup_bash} && "
            f"exec ros2 launch pus6_bringup pus6_system.launch.py "
            f"config_file:={config_file} "
            f"hardware_mode:={'active' if (execute_motion and not dry_run) else 'mock'} "
            f"maintenance_mode:=true "
            f"servo_scheduler_mode:={servo_scheduler_mode} "
            f"telemetry_publish_mode:={telemetry_publish_mode} "
            f"feedback_loss_policy:={feedback_loss_policy}",
        ]
        robot_log = open("/tmp/j13_robot.log", "w")
        robot_proc = subprocess.Popen(
            cmd_robot, stdout=robot_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid
        )

        t_start_a = time.monotonic()
        stable_a_start: Optional[float] = None
        driver_ok = False

        while time.monotonic() - t_start_a < 15.0:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now = time.time()
            if monitor.driver_stop_latched:
                raise RuntimeError(
                    f"Stage A FAIL: Driver STOP latched during startup: {monitor.driver_stop_reason}"
                )

            fb_fresh = monitor.last_fb_time > 0 and (now - monitor.last_fb_time) < 0.200
            no_faults = monitor.latest_faults is not None and all(
                f == 0 for f in monitor.latest_faults
            )

            if fb_fresh and no_faults:
                if stable_a_start is None:
                    stable_a_start = time.monotonic()
                elif time.monotonic() - stable_a_start >= 3.0:
                    driver_ok = True
                    break
            else:
                stable_a_start = None

        if not driver_ok:
            raise TimeoutError(
                "Stage A TIMEOUT: Motor driver failed to achieve 3.0s sustained zero-fault feedback within 15s."
            )

        print(
            "[STAGE A PASSED] pus6_motor_driver stable (feedback age < 200ms, 0 faults, >= 3.0s continuous).",
            flush=True,
        )

        # ==========================================================
        # STAGE B: Launch Dual H30 IMUs & Validate Streaming Stability
        # ==========================================================
        print("\n" + "=" * 70)
        print("[STAGE B] Launching Dual H30 IMU drivers (Base & Platform)...")
        print("=" * 70, flush=True)

        cmd_plat = [
            "bash",
            "-c",
            f"source {setup_bash} && "
            f"exec ros2 run h30_imu h30_imu_node --ros-args "
            f"-r __ns:=/pus6/imu_platform "
            f"-p port:={plat_port} "
            f"-p baud_rate:={baud_rate} "
            f"-p frame_id:=pus6_platform_imu_link "
            f"-p world_frame:=world "
            f"-p imu_topic:=/pus6/imu_platform/data",
        ]
        plat_log = open("/tmp/j13_plat_imu.log", "w")
        plat_proc = subprocess.Popen(
            cmd_plat, stdout=plat_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid
        )

        cmd_base = [
            "bash",
            "-c",
            f"source {setup_bash} && "
            f"exec ros2 run h30_imu h30_imu_node --ros-args "
            f"-r __ns:=/pus6/imu_base "
            f"-p port:={base_port} "
            f"-p baud_rate:={baud_rate} "
            f"-p frame_id:=pus6_base_imu_link "
            f"-p world_frame:=world "
            f"-p imu_topic:=/pus6/imu_base/data",
        ]
        base_log = open("/tmp/j13_base_imu.log", "w")
        base_proc = subprocess.Popen(
            cmd_base, stdout=base_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid
        )

        t_start_b = time.monotonic()
        stable_b_start: Optional[float] = None
        imu_ok = False

        while time.monotonic() - t_start_b < 10.0:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            now = time.time()
            if monitor.driver_stop_latched:
                raise RuntimeError(
                    f"Stage B FAIL: Driver STOP latched while starting IMUs: {monitor.driver_stop_reason}"
                )

            base_fresh = monitor.has_base_imu and (now - monitor.last_base_imu_time) < 0.100
            plat_fresh = monitor.has_plat_imu and (now - monitor.last_plat_imu_time) < 0.100

            if base_fresh and plat_fresh:
                if stable_b_start is None:
                    stable_b_start = time.monotonic()
                elif time.monotonic() - stable_b_start >= 2.0:
                    imu_ok = True
                    break
            else:
                stable_b_start = None

        if not imu_ok:
            raise TimeoutError(
                "Stage B TIMEOUT: Dual IMUs failed to achieve 2.0s sustained streaming (<100ms age) within 10s."
            )

        print(
            "[STAGE B PASSED] Dual H30 IMUs streaming stably (Base & Plat age < 100ms for >= 2.0s).",
            flush=True,
        )

        # ==========================================================
        # Mechanical OPERATING_HOME Verification
        # ==========================================================
        if monitor.latest_steps is None:
            raise RuntimeError("No feedback steps received from driver.")
        errors = [abs(s - 1000) for s in monitor.latest_steps]
        max_err = max(errors)
        if max_err > 15:
            raise ValueError(
                f"Preflight FAIL: Motors not at OPERATING_HOME [1000]*6 "
                f"(actual={monitor.latest_steps}, max err={max_err} > 15 steps)"
            )

        print(
            f"[PREFLIGHT OK] Verified 6 axes at OPERATING_HOME: {monitor.latest_steps} "
            f"(max error={max_err} steps <= 15 steps).",
            flush=True,
        )

        monitor.destroy_node()
        rclpy.shutdown()

        # ==========================================================
        # STAGE C: Launch J13 LADRC Shadow Runner
        # ==========================================================
        print("\n" + "=" * 70)
        print(f"[STAGE C] Launching J13 LADRC Shadow Runner (Stage: {stage})...")
        print(
            f"[STAGE C] Classical P/PD: Roll Kp={kp_roll:.3f}, Pitch Kp={kp_pitch:.3f}, Yaw Kp={kp_yaw:.3f}"
        )
        print(
            f"[STAGE C] LADRC Roll:     b0={ladrc_b0_roll:.1f}, wc={ladrc_omega_c_roll:.2f}, wo={ladrc_omega_o_roll:.2f}"
        )
        print(
            f"[STAGE C] LADRC Pitch:    b0={ladrc_b0_pitch:.1f}, wc={ladrc_omega_c_pitch:.2f}, wo={ladrc_omega_o_pitch:.2f}"
        )
        print(
            f"[STAGE C] LADRC Yaw:      b0={ladrc_b0_yaw:.1f}, wc={ladrc_omega_c_yaw:.2f}, wo={ladrc_omega_o_yaw:.2f}"
        )
        print(
            f"[STAGE C] Residual Clamp: +/-{max_residual_roll_deg:.1f}° (Roll/Pitch/Yaw) | Duration: {duration:.1f}s"
        )
        print("=" * 70, flush=True)

        cmd_runner = [
            "python3",
            "/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py",
            "--stage",
            stage,
            "--duration",
            str(duration),
            "--output-dir",
            str(out_path.resolve()),
            "--kp-roll",
            str(kp_roll),
            "--kp-pitch",
            str(kp_pitch),
            "--kp-yaw",
            str(kp_yaw),
            "--kd-roll",
            str(kd_roll),
            "--kd-pitch",
            str(kd_pitch),
            "--kd-yaw",
            str(kd_yaw),
            "--derivative-lpf-cutoff-hz",
            str(derivative_lpf_cutoff_hz),
            "--max-residual-roll-deg",
            str(max_residual_roll_deg),
            "--max-residual-pitch-deg",
            str(max_residual_pitch_deg),
            "--max-residual-yaw-deg",
            str(max_residual_yaw_deg),
            "--ladrc-b0-roll",
            str(ladrc_b0_roll),
            "--ladrc-b0-pitch",
            str(ladrc_b0_pitch),
            "--ladrc-b0-yaw",
            str(ladrc_b0_yaw),
            "--ladrc-omega-c-roll",
            str(ladrc_omega_c_roll),
            "--ladrc-omega-c-pitch",
            str(ladrc_omega_c_pitch),
            "--ladrc-omega-c-yaw",
            str(ladrc_omega_c_yaw),
            "--ladrc-omega-o-roll",
            str(ladrc_omega_o_roll),
            "--ladrc-omega-o-pitch",
            str(ladrc_omega_o_pitch),
            "--ladrc-omega-o-yaw",
            str(ladrc_omega_o_yaw),
            "--ladrc-max-residual-roll-deg",
            str(max_residual_roll_deg),
            "--ladrc-max-residual-pitch-deg",
            str(max_residual_pitch_deg),
            "--ladrc-max-residual-yaw-deg",
            str(max_residual_yaw_deg),
            "--auto-start",
        ]
        if derivative_enabled:
            cmd_runner.append("--enable-derivative")
        else:
            cmd_runner.append("--disable-derivative")

        if execute_motion and not dry_run:
            cmd_runner.append("--execute-motion")
            cmd_runner.append("--no-dry-run")
        else:
            cmd_runner.append("--dry-run")

        runner_proc = subprocess.Popen(cmd_runner, preexec_fn=os.setsid)

        active_marker = Path("/tmp/j13_active_signaled")
        t_wait_active = time.monotonic()
        banner_printed = False

        while runner_proc.poll() is None:
            if active_marker.exists() and not banner_printed:
                banner_printed = True
                print("\n" + "=" * 60, flush=True)
                print(f"J13-2A2 ACTIVE: Classical P/PD Control + LADRC Shadow Mode Engaged")
                print(f"Stage: {stage} | Planned Duration: {duration:.1f}s")
                print(f"LADRC Shadow Observing (LOG ONLY, NO REAL CONTROL AUTHORITY)")
                print("=" * 60 + "\n", flush=True)
            if not banner_printed and (time.monotonic() - t_wait_active > 15.0):
                print(
                    "[WARNING] Active marker not seen after 15s. Continuing to monitor runner...",
                    flush=True,
                )
            time.sleep(0.1)

        ret = runner_proc.poll()
        print(f"\n[J13 STAGED STARTUP] J13 runner exited with code {ret}.", flush=True)

    except KeyboardInterrupt:
        print("\n[J13 STAGED STARTUP] Interrupted by user.", flush=True)
    except Exception as exc:
        print(f"\n[J13 STAGED STARTUP ERROR] {exc}", flush=True)
    finally:
        print("[J13 STAGED STARTUP] Shutting down subprocesses...", flush=True)
        kill_all_subprocesses()
        print("[J13 STAGED STARTUP] All processes cleanly terminated.", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="J13-2A2 Real-Hardware LADRC Shadow Staged Startup Orchestrator"
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--stage", default="j13_2a2_static_shadow")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument(
        "--config-file",
        default="/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml",
    )
    parser.add_argument("--dry-run", action="store_true", default=False)
    parser.add_argument("--execute-motion", action="store_true", default=True)
    parser.add_argument(
        "--servo-scheduler-mode",
        default="dedicated_thread",
        choices=["ros_timer", "dedicated_thread"],
    )
    parser.add_argument(
        "--telemetry-publish-mode",
        default="async_latest",
        choices=["auto", "inline", "async_latest"],
    )
    parser.add_argument(
        "--feedback-loss-policy",
        default="age_based",
        choices=["default", "consecutive_misses", "age_based"],
    )

    # Classical PID
    parser.add_argument("--kp-roll", type=float, default=0.05)
    parser.add_argument("--kp-pitch", type=float, default=0.05)
    parser.add_argument("--kp-yaw", type=float, default=0.025)
    parser.add_argument("--kd-roll", type=float, default=0.0)
    parser.add_argument("--kd-pitch", type=float, default=0.0015)
    parser.add_argument("--kd-yaw", type=float, default=0.0015)
    parser.add_argument("--enable-derivative", action="store_true", default=True)
    parser.add_argument("--disable-derivative", action="store_false", dest="enable_derivative")
    parser.add_argument("--derivative-lpf-cutoff-hz", type=float, default=5.0)
    parser.add_argument("--max-residual-roll-deg", type=float, default=0.5)
    parser.add_argument("--max-residual-pitch-deg", type=float, default=0.5)
    parser.add_argument("--max-residual-yaw-deg", type=float, default=0.5)

    # LADRC
    parser.add_argument("--ladrc-b0-roll", type=float, default=10.0)
    parser.add_argument("--ladrc-b0-pitch", type=float, default=10.0)
    parser.add_argument("--ladrc-b0-yaw", type=float, default=3.0)
    parser.add_argument("--ladrc-omega-c-roll", type=float, default=1.0)
    parser.add_argument("--ladrc-omega-c-pitch", type=float, default=1.0)
    parser.add_argument("--ladrc-omega-c-yaw", type=float, default=0.35)
    parser.add_argument("--ladrc-omega-o-roll", type=float, default=4.0)
    parser.add_argument("--ladrc-omega-o-pitch", type=float, default=4.0)
    parser.add_argument("--ladrc-omega-o-yaw", type=float, default=1.8)

    args = parser.parse_args()

    run_j13_staged_session(
        duration=args.duration,
        stage=args.stage,
        output_dir=args.output_dir,
        config_file=args.config_file,
        dry_run=args.dry_run,
        execute_motion=args.execute_motion,
        servo_scheduler_mode=args.servo_scheduler_mode,
        telemetry_publish_mode=args.telemetry_publish_mode,
        feedback_loss_policy=args.feedback_loss_policy,
        kp_roll=args.kp_roll,
        kp_pitch=args.kp_pitch,
        kp_yaw=args.kp_yaw,
        kd_roll=args.kd_roll,
        kd_pitch=args.kd_pitch,
        kd_yaw=args.kd_yaw,
        derivative_enabled=args.enable_derivative,
        derivative_lpf_cutoff_hz=args.derivative_lpf_cutoff_hz,
        max_residual_roll_deg=args.max_residual_roll_deg,
        max_residual_pitch_deg=args.max_residual_pitch_deg,
        max_residual_yaw_deg=args.max_residual_yaw_deg,
        ladrc_b0_roll=args.ladrc_b0_roll,
        ladrc_b0_pitch=args.ladrc_b0_pitch,
        ladrc_b0_yaw=args.ladrc_b0_yaw,
        ladrc_omega_c_roll=args.ladrc_omega_c_roll,
        ladrc_omega_c_pitch=args.ladrc_omega_c_pitch,
        ladrc_omega_c_yaw=args.ladrc_omega_c_yaw,
        ladrc_omega_o_roll=args.ladrc_omega_o_roll,
        ladrc_omega_o_pitch=args.ladrc_omega_o_pitch,
        ladrc_omega_o_yaw=args.ladrc_omega_o_yaw,
    )


if __name__ == "__main__":
    main()
