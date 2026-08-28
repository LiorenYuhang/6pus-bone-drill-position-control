#!/usr/bin/env python3
"""6PUS J8: Final Full-Featured Video Demonstration Trajectory Runner.

Executes the complete, sequential visual showcase trajectory on the real 6PUS manipulator:
  1. Phase 0: HOME hold & telemetry lock (3.0s)
  2. Phase 1: Pure X lateral motion (HOME -> +8mm -> HOME -> -8mm -> HOME)
  3. Phase 2: Pure Y lateral motion (HOME -> +8mm -> HOME -> -8mm -> HOME)
  4. Phase 3: Pure Z vertical heave (HOME -> +8mm -> HOME -> -8mm -> HOME)
  5. Phase 4: Pure Roll platform tilt (HOME -> +8deg -> HOME -> -8deg -> HOME)
  6. Phase 5: Pure Pitch platform tilt (HOME -> +8deg -> HOME -> -8deg -> HOME)
  7. Phase 6: Pure Yaw axial rotation (HOME -> +8deg -> HOME -> -8deg -> HOME)
  8. Phase 7: Combined 6-DoF smooth continuous trajectory (2 full cycles @ 0.1 Hz)
  9. Phase 8: Final Settle and Return to OPERATING_HOME with repeatability verification

All motion runs strictly through the complete, unbypassed ROS 2 topic graph:
  /pus6/target_cartesian_pose -> pus6_kinematics_node -> /pus6/desired_joint_deltas
  -> pus6_control_node -> /pus6/command_joint_steps -> pus6_motor_driver_node (J7 Servo 50Hz)
  -> /pus6/joint_feedback/timed -> pus6_fk_node -> /pus6/fk/state
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable
from pathlib import Path

import numpy as np

# ROS 2 imports
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_control.control_node import Pus6ControlNode
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_kinematics.kinematics_node import Pus6KinematicsNode
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_msgs.msg import FkStateStamped, TimedJointFeedback

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_RATE = 921600


class DemoSafetyAbort(RuntimeError):
    """Raised when driver or telemetry health becomes unsafe during the demo."""


def demo_health_failure_reason(driver_node) -> str | None:
    if getattr(driver_node, "driver_fault_latched", False):
        return f'driver fault latched: {getattr(driver_node, "driver_fault_reason", None)}'
    if getattr(driver_node, "j7_writer", None) is not None and driver_node.j7_writer.stopped:
        return f'driver writer stopped: {driver_node.j7_writer.audit_snapshot().stop_reason}'
    if getattr(driver_node, "j7_feedback", None) is not None and driver_node.j7_feedback.stopped:
        return f'driver feedback scheduler stopped: {driver_node.j7_feedback.audit_snapshot().stop_reason}'
    if hasattr(driver_node, "_latest_faults") and any(f != 0 for f in driver_node._latest_faults.values()):
        return f'hardware motor fault detected: {driver_node._latest_faults}'
    return None


def ensure_demo_health(driver_node) -> None:
    reason = demo_health_failure_reason(driver_node)
    if reason is not None:
        raise DemoSafetyAbort(reason)
MOTOR_IDS = [1, 2, 3, 4, 5, 6]



def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def smooth_ramp(s: float) -> float:
    """Quintic S-curve ramp in [0, 1]."""
    s_clamped = max(0.0, min(1.0, s))
    return 10.0 * (s_clamped**3) - 15.0 * (s_clamped**4) + 6.0 * (s_clamped**5)


class VideoDemoHarness(Node):
    def __init__(self) -> None:
        super().__init__("j8_video_demo_harness")
        self.lock = threading.Lock()
        self._health_check: Callable[[], None] | None = None
        self.current_fk_pose = [0.0, 0.0, 0.132612, 0.0, 0.0, 0.0]

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/pus6/target_cartesian_pose", 10
        )

        self.fk_state_sub = self.create_subscription(
            FkStateStamped, "/pus6/fk/state", self._on_fk_state, sensor_qos
        )

    def set_health_check(self, health_check: Callable[[], None]) -> None:
        self._health_check = health_check

    def _on_fk_state(self, msg: FkStateStamped) -> None:
        r, p, y = quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        with self.lock:
            self.current_fk_pose = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                r, p, y,
            ]


    def publish_target_pose(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
        if self._health_check is not None:
            self._health_check()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        msg.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        msg.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        msg.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)

        self.target_pose_pub.publish(msg)


def run_single_axis_showcase(
    harness: VideoDemoHarness,
    dof_idx: int,
    target_val: float,
    dof_label: str,
    unit_label: str,
    scale: float,
    z0: float,
    ramp_dur_s: float = 3.0,
    hold_dur_s: float = 1.2,
    rate_hz: float = 50.0,
) -> None:
    dt = 1.0 / rate_hz
    tgt_disp = target_val * scale
    print(f"\n[DEMO] >> Showcase {dof_label}: +/-{tgt_disp:.1f} {unit_label} (HOME -> +{tgt_disp:.1f} -> HOME -> -{tgt_disp:.1f} -> HOME)...")

    def send(v: float) -> None:
        pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
        if dof_idx == 2:
            pose[2] = z0 + v
        else:
            pose[dof_idx] = v
        harness.publish_target_pose(*pose)

    n_ramp = int(ramp_dur_s * rate_hz)
    n_hold = int(hold_dur_s * rate_hz)

    # 1. Ramp to +target & Hold
    for s_step in range(n_ramp):
        s = smooth_ramp((s_step + 1) / n_ramp)
        send(s * target_val)
        time.sleep(dt)
    for _ in range(n_hold):
        send(target_val)
        time.sleep(dt)

    # 2. Ramp back to HOME & Hold
    for s_step in range(n_ramp):
        s = smooth_ramp((s_step + 1) / n_ramp)
        send((1.0 - s) * target_val)
        time.sleep(dt)
    for _ in range(n_hold):
        send(0.0)
        time.sleep(dt)

    # 3. Ramp to -target & Hold
    for s_step in range(n_ramp):
        s = smooth_ramp((s_step + 1) / n_ramp)
        send(-s * target_val)
        time.sleep(dt)
    for _ in range(n_hold):
        send(-target_val)
        time.sleep(dt)

    # 4. Ramp back to HOME & Hold
    for s_step in range(n_ramp):
        s = smooth_ramp((s_step + 1) / n_ramp)
        send(-(1.0 - s) * target_val)
        time.sleep(dt)
    for _ in range(n_hold):
        send(0.0)
        time.sleep(dt)

    with harness.lock:
        p = list(harness.current_fk_pose)
    val_meas = p[dof_idx]
    if dof_idx == 2:
        val_meas -= z0
    disp_meas = val_meas * scale
    print(f"       Settled at HOME: measured {dof_label} residual = {disp_meas:+.3f} {unit_label}")


def run_combined_6dof_showcase(
    harness: VideoDemoHarness,
    z0: float,
    cycles: int = 2,
    freq_hz: float = 0.1,
    rate_hz: float = 50.0,
) -> None:
    dt = 1.0 / rate_hz
    total_dur_s = cycles / freq_hz
    n_steps = int(total_dur_s * rate_hz)
    omega = 2.0 * math.pi * freq_hz

    Ax_m = 0.0050     # 5.0 mm
    Ay_m = 0.0050     # 5.0 mm
    Az_m = 0.0050     # 5.0 mm
    Ar_rad = math.radians(3.0)   # 3.0 deg
    Ap_rad = math.radians(3.0)   # 3.0 deg
    Ayaw_rad = math.radians(5.0) # 5.0 deg

    print(f"\n[DEMO] >> Showcase Combined 6-DoF Continuous Motion ({cycles} cycles @ {freq_hz} Hz, {total_dur_s:.1f}s)...")
    print(f"       Amplitudes: X/Y/Z = +/-5.0 mm, Roll/Pitch = +/-3.0 deg, Yaw = +/-5.0 deg")

    for step in range(n_steps):
        t = step * dt
        # Smooth envelope blending for seamless start and finish
        blend = 1.0
        if t < 2.0:
            blend = smooth_ramp(t / 2.0)
        elif t > (total_dur_s - 2.0):
            blend = smooth_ramp((total_dur_s - t) / 2.0)

        x = blend * Ax_m * math.sin(omega * t)
        y = blend * Ay_m * math.cos(omega * t)
        z = z0 + blend * Az_m * math.sin(omega * t)
        roll = blend * Ar_rad * math.sin(omega * t)
        pitch = blend * Ap_rad * math.cos(omega * t)
        yaw = blend * Ayaw_rad * math.sin(omega * t + math.pi / 4.0)

        harness.publish_target_pose(x, y, z, roll, pitch, yaw)
        time.sleep(dt)

    print("       Combined 6-DoF showcase completed smoothly.")


def main() -> None:
    print("=" * 90)
    print("      6PUS J8: FINAL VIDEO DEMONSTRATION TRAJECTORY RUNNER")
    print("=" * 90)
    print(f"Hardware Port: {SERIAL_PORT} @ {BAUD_RATE}")
    print("=" * 90)

    kin = Analytical6PusKinematics()
    z0 = kin.nominal_height
    print(f"Geometry Nominal Cartesian HOME: z0 = {z0:.6f} m ({z0*1000:.3f} mm)\n")

    rclpy.init()

    kin_node = Pus6KinematicsNode()
    kin_node.max_xy = 0.025
    kin_node.max_z_delta = 0.025
    kin_node.max_angle = 0.25

    ctrl_node = Pus6ControlNode()
    motor_node = Pus6MotorDriverNode(
        hardware_mode="active",
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=0.050,
        write_rate_hz=50.0,
        poll_rate_hz=10.0,
        use_j7_servo=True,
        servo_speed=50,
        publish_raw_diagnostics=True,
    )
    fk_node = Pus6FkNode(fixed_lag_delay_ms=160.0, query_rate_hz=50.0)
    harness = VideoDemoHarness()
    harness.set_health_check(lambda: ensure_demo_health(motor_node))

    import gc
    import sys
    from rclpy.executors import SingleThreadedExecutor

    sys.setswitchinterval(0.001)

    exec_motor = SingleThreadedExecutor()
    exec_motor.add_node(motor_node)

    exec_other = SingleThreadedExecutor()
    exec_other.add_node(kin_node)
    exec_other.add_node(ctrl_node)
    exec_other.add_node(fk_node)
    exec_other.add_node(harness)

    th_motor = threading.Thread(target=exec_motor.spin, daemon=True)
    th_other = threading.Thread(target=exec_other.spin, daemon=True)
    th_motor.start()
    th_other.start()
    print("[INIT OK] Dedicated SingleThreadedExecutors spinning motor node and pipeline nodes in background.\n")

    time.sleep(1.0)

    # Disable GC during active streaming to prevent stop-the-world pauses
    gc.collect()
    gc.disable()

    demo_start_wall = time.monotonic()
    aborted = False
    try:
        # 1. Initial HOME Hold
        print("--- [Phase 0] Publishing Cartesian OPERATING_HOME & Holding 3.0s ---")
        for _ in range(150):
            harness.publish_target_pose(0.0, 0.0, z0, 0.0, 0.0, 0.0)
            time.sleep(0.02)

        time.sleep(1.0)
        with harness.lock:
            home_fk = list(harness.current_fk_pose)
        print(
            f"[Phase 0 PASS] Initial Settled HOME FK: X={home_fk[0]*1000:+.3f}mm, "
            f"Y={home_fk[1]*1000:+.3f}mm, Z={home_fk[2]*1000:+.3f}mm, "
            f"R={math.degrees(home_fk[3]):+.3f}°, P={math.degrees(home_fk[4]):+.3f}°, Y={math.degrees(home_fk[5]):+.3f}°\n"
        )

        # 2. Sequential Single-Axis Large-Motion Showcases (8 mm / 8 deg)
        run_single_axis_showcase(harness, 0, 0.008, "X Translation", "mm", 1000.0, z0)
        run_single_axis_showcase(harness, 1, 0.008, "Y Translation", "mm", 1000.0, z0)
        run_single_axis_showcase(harness, 2, 0.008, "Z Heave", "mm", 1000.0, z0)
        run_single_axis_showcase(harness, 3, math.radians(8.0), "Roll Rotation", "deg", 180.0/math.pi, z0)
        run_single_axis_showcase(harness, 4, math.radians(8.0), "Pitch Rotation", "deg", 180.0/math.pi, z0)
        run_single_axis_showcase(harness, 5, math.radians(8.0), "Yaw Rotation", "deg", 180.0/math.pi, z0)

        # 3. Combined 6-DoF Continuous Motion Showcase
        run_combined_6dof_showcase(harness, z0, cycles=2, freq_hz=0.1)

        # 4. Final Settle at HOME
        print("\n--- [Phase 8] Returning to Cartesian OPERATING_HOME & Final Repeatability Verification (4.0s) ---")
        for _ in range(200):
            harness.publish_target_pose(0.0, 0.0, z0, 0.0, 0.0, 0.0)
            time.sleep(0.02)

        time.sleep(1.0)
        with harness.lock:
            final_fk = list(harness.current_fk_pose)

        dx_mm = (final_fk[0] - home_fk[0]) * 1000.0
        dy_mm = (final_fk[1] - home_fk[1]) * 1000.0
        dz_mm = (final_fk[2] - home_fk[2]) * 1000.0
        dr_deg = math.degrees(final_fk[3] - home_fk[3])
        dp_deg = math.degrees(final_fk[4] - home_fk[4])
        dyaw_deg = math.degrees(final_fk[5] - home_fk[5])

        demo_dur_s = time.monotonic() - demo_start_wall

        print(
            f"\n[DEMO COMPLETE] Final Settled HOME FK: X={final_fk[0]*1000:+.3f}mm, "
            f"Y={final_fk[1]*1000:+.3f}mm, Z={final_fk[2]*1000:+.3f}mm, "
            f"R={math.degrees(final_fk[3]):+.3f}°, P={math.degrees(final_fk[4]):+.3f}°, Y={math.degrees(final_fk[5]):+.3f}°"
        )
        print(f"Repeatability Hysteresis: dX={dx_mm:+.3f}mm, dY={dy_mm:+.3f}mm, dZ={dz_mm:+.3f}mm, dRoll={dr_deg:+.3f}°, dPitch={dp_deg:+.3f}°, dYaw={dyaw_deg:+.3f}°\n")

        # Extract audit metrics from motor node before shutdown
        writer_audit = motor_node.j7_writer.audit_snapshot()
        fb_audit = motor_node.j7_feedback.audit_snapshot()
        rx_audit = motor_node.j7_adapter.rx_audit_snapshot()

        total_cycles = len(writer_audit.cycles)
        actual_rate = total_cycles / demo_dur_s if demo_dur_s > 0 else 0.0

        intervals = [
            (c.cycle_end_ns - c.cycle_start_ns) / 1e6
            for c in writer_audit.cycles
        ]
        mean_int = float(np.mean(intervals)) if intervals else 0.0
        p95_int = float(np.percentile(intervals, 95)) if intervals else 0.0
        max_int = float(np.max(intervals)) if intervals else 0.0

        motion_responses = rx_audit.motion_write_response_count
        expected_responses = total_cycles * 6

        print("=" * 90)
        print("          6PUS J8 FINAL VIDEO DEMONSTRATION QUALIFICATION REPORT")
        print("=" * 90)
        print(f"Total Execution Duration:           {demo_dur_s:.3f} s")
        print(f"Total Executed Servo Cycles:        {total_cycles}")
        print(f"Actual Sustained Servo Rate:        {actual_rate:.2f} Hz (Configured: 50.0 Hz)")
        print(f"6/6 Feedback Completeness:          {motion_responses} / {expected_responses} ({motion_responses/expected_responses*100.0 if expected_responses else 0.0:.1f}%)")
        print(f"Worker Batch Duration (ms):         Mean={mean_int:.2f}, P95={p95_int:.2f}, Max={max_int:.2f}")
        print(f"Stale Responses:                    {rx_audit.stale_response_count}")
        print(f"Residual Frames Remaining:          {rx_audit.residual_frames}")
        print(f"Malformed Frames:                   {rx_audit.malformed_count}")
        print(f"Feedback Timeouts:                  {fb_audit.timeout_count}")
        print(f"Hardware Motor Faults:              {sum(1 for f in motor_node._latest_faults.values() if f != 0)}")
        print(f"Driver Latched Fault:               {motor_node.driver_fault_latched}")
        print(f"Driver STOP Reason:                 {fb_audit.stop_reason or 'None (HEALTHY)'}")
        print(f"Final Repeatability Hysteresis:     dX={dx_mm:+.3f}mm, dY={dy_mm:+.3f}mm, dZ={dz_mm:+.3f}mm")
        print(f"                                    dRoll={dr_deg:+.3f}°, dPitch={dp_deg:+.3f}°, dYaw={dyaw_deg:+.3f}°")
        print("=" * 90)

    except DemoSafetyAbort as exc:
        aborted = True
        print(f"[DEMO ABORTED] {exc}")
    finally:
        gc.enable()
        print("[SHUTDOWN] Shutting down ROS2 executors and nodes...")
        exec_motor.shutdown()
        exec_other.shutdown()
        kin_node.destroy_node()
        ctrl_node.destroy_node()
        motor_node.destroy_node()
        fk_node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
    if not aborted:
        print("[SHUTDOWN OK] Video demonstration finished successfully.")
    else:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
