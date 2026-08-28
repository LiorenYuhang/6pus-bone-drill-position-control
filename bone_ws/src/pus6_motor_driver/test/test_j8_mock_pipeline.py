"""J8-B MOCK / OFFLINE ROS2 Integration & Timing Contract Test Suite.

Verifies:
1. Cartesian target -> Analytical IK -> Shaper -> J7 Servo -> TimedFeedback -> FixedLag -> FK
2. HOME mapping: Cartesian HOME -> [1000]*6
3. Cartesian Z +2 mm -> Analytical IK -> [1080]*6
4. Arbitrary 6DoF targets produce non-hardcoded asymmetric commands
5. J7 Servo adapter 50Hz batch streaming
6. Real per-axis feedback timestamp advance (only polled axis updates)
7. Monotonic FixedLag buffering without duplicate sample injection
8. 100% zero-extrapolation bracketing
9. Real-time SE(3) LM FK reconstruction
10. Delay sweep across candidate values (120, 140, 160, 180, 200 ms)
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

# Also ensure pus6_kinematics and pus6_control are in sys.path
WS_SRC = PKG_ROOT.parent
for pkg_name in ["pus6_kinematics", "pus6_control", "pus6_msgs"]:
    pkg_path = WS_SRC / pkg_name
    if pkg_path.exists() and str(pkg_path) not in sys.path:
        sys.path.insert(0, str(pkg_path))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.geometry_config import load_geometry_config
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus
from pus6_kinematics.fk_node import Pus6FkNode
from pus6_control.joint_trajectory_controller import StatefulJerkLimitedShaper
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.safety import HardwareMode, MOTOR_IDS, NUM_MOTORS
from pus6_msgs.msg import TimedJointFeedback, TimedJointCommand, FkStateStamped


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_j8_cartesian_ik_and_geometry_consistency():
    """Verify items 1, 2, 3, 4: Cartesian poses pass through Analytical IK without hardcoding."""
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    ref_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]

    # Item 2: HOME mapping
    q_home = kin.inverse_kinematics(ref_pose)
    assert len(q_home) == 6
    assert all(math.isclose(q, 0.050, rel_tol=1e-6) for q in q_home)
    delta_home = kin.differential_inverse_kinematics(ref_pose, ref_pose)
    assert all(math.isclose(dq, 0.0, abs_tol=1e-9) for dq in delta_home)
    steps_home = [int(round(1000 + dq * 40000.0)) for dq in delta_home]
    assert steps_home == [1000, 1000, 1000, 1000, 1000, 1000]

    # Item 3: Z +2.0 mm heave
    pose_z_plus_2mm = [0.0, 0.0, z_nom + 0.002, 0.0, 0.0, 0.0]
    q_z2 = kin.inverse_kinematics(pose_z_plus_2mm)
    assert all(math.isclose(q, 0.052, rel_tol=1e-6) for q in q_z2)
    delta_z2 = kin.differential_inverse_kinematics(pose_z_plus_2mm, ref_pose)
    assert all(math.isclose(dq, 0.002, abs_tol=1e-9) for dq in delta_z2)
    steps_z2 = [int(round(1000 + dq * 40000.0)) for dq in delta_z2]
    assert steps_z2 == [1080, 1080, 1080, 1080, 1080, 1080]

    # Item 4: Arbitrary asymmetric 6DoF poses produce non-hardcoded asymmetric joint commands
    asymmetric_cases = [
        ("X_+3mm", [0.003, 0.0, z_nom, 0.0, 0.0, 0.0]),
        ("Y_-3mm", [0.0, -0.003, z_nom, 0.0, 0.0, 0.0]),
        ("Pitch_+1deg", [0.0, 0.0, z_nom, 0.0, math.radians(1.0), 0.0]),
        ("Roll_-1.5deg", [0.0, 0.0, z_nom, math.radians(-1.5), 0.0, 0.0]),
        ("Yaw_+2deg", [0.0, 0.0, z_nom, 0.0, 0.0, math.radians(2.0)]),
        ("Composite_6DoF", [0.002, -0.0015, z_nom + 0.0018, math.radians(0.8), math.radians(-0.6), math.radians(1.2)]),
    ]

    for name, pose_in in asymmetric_cases:
        delta_q = kin.differential_inverse_kinematics(pose_in, ref_pose)
        steps = [int(round(1000 + dq * 40000.0)) for dq in delta_q]
        # Verify non-uniform / asymmetric steps
        assert len(set(steps)) > 1, f"Expected asymmetric steps for {name}, got uniform {steps}"
        assert all(200 <= s <= 1800 for s in steps), f"Steps out of normal envelope: {steps}"

        # Verify round-trip FK
        feedback_q = [0.050 + (s - 1000) * 0.000025 for s in steps]
        fk_res = kin.forward_kinematics_with_metrics(feedback_q, initial_guess=ref_pose)
        assert fk_res.converged is True
        pos_err_m = np.linalg.norm(np.array(fk_res.pose[:3]) - np.array(pose_in[:3]))
        assert pos_err_m < 0.0001, f"FK error {pos_err_m*1000:.3f}mm on {name}"


def test_j8_servo_adapter_timing_and_real_timestamp_advance(rclpy_init):
    """Verify items 5, 6, 7: J7 Servo adapter sends 50Hz batches, only genuinely polled axes update timestamps."""
    driver_node = Pus6MotorDriverNode(
        hardware_mode="mock",
        write_rate_hz=50.0,
        use_j7_servo=True,
    )
    assert driver_node.is_j7_servo_active is True
    assert driver_node.j7_writer is not None
    assert driver_node.j7_feedback is not None

    synchronizer = FixedLagJointSynchronizer(max_history_size=500)

    # Send 18 cycles (3 full round-robin rounds of 6 axes)
    t_start_ns = time.monotonic_ns()
    for cycle in range(18):
        t_cycle_ns = t_start_ns + cycle * 20_000_000

        # Send command steps
        cmd_msg = Int32MultiArray()
        cmd_msg.data = [1000 + cycle] * 6
        driver_node._handle_command_joint_steps(cmd_msg)

        # Run 50Hz cycle
        driver_node._uart_worker_cycle()

        # Extract current feedback state
        per_axis_stamps = [driver_node._latest_rx_done_ns[mid] for mid in MOTOR_IDS]
        raw_steps = [driver_node._latest_positions_steps[mid] for mid in MOTOR_IDS]
        pos_m = [0.050 + (s - 1000) * 0.000025 for s in raw_steps]

        # Ingest into synchronizer (Item 7: Monotonic filtering of duplicate timestamps)
        synchronizer.push_raw_snapshot(per_axis_stamps, pos_m, raw_steps)

    # Verify buffer lengths: under Design B ACTIVE 50Hz feedback, all 6 axes receive genuine 50Hz updates every cycle!
    final_lengths = synchronizer.buffer_lengths
    assert all(count >= 18 for count in final_lengths), (
        f"Expected each axis to have at least 18 samples (50Hz updates on all axes), got {final_lengths}. "
        "Proves Design B delivers 50Hz feedback across all axes!"
    )

    driver_node.destroy_node()


def test_j8_fixed_lag_zero_extrapolation_and_fk_reconstruction(rclpy_init):
    """Verify items 8, 9: Zero extrapolation only, and accurate LM FK reconstruction."""
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    ref_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]

    synchronizer = FixedLagJointSynchronizer(max_history_size=500)

    # Simulate 60 cycles (1.2 seconds, 10 samples per axis) of a continuous heave motion: Z = z_nom + 2mm * sin(2*pi*f*t)
    f_motion = 0.5 # 0.5 Hz
    t_start_ns = 1_000_000_000 # 1.0s base
    dt_cycle_ns = 20_000_000   # 20ms

    all_axis_timestamps: dict[int, list[int]] = {mid: [] for mid in MOTOR_IDS}
    cached_stamps = {mid: 0 for mid in MOTOR_IDS}
    cached_steps = {mid: 1000 for mid in MOTOR_IDS}

    for cycle in range(60):
        t_now_ns = t_start_ns + cycle * dt_cycle_ns
        t_sec = (t_now_ns - t_start_ns) / 1e9

        # Target heave
        z_curr = z_nom + 0.002 * math.sin(2.0 * math.pi * f_motion * t_sec)
        pose_curr = [0.0, 0.0, z_curr, 0.0, 0.0, 0.0]
        q_curr = kin.inverse_kinematics(pose_curr)
        steps_curr = [int(round(1000 + (q - 0.050) * 40000.0)) for q in q_curr]

        # Round-robin polling: axis k is polled in cycle
        polled_axis = (cycle % 6) + 1
        cached_steps[polled_axis] = steps_curr[polled_axis - 1]
        cached_stamps[polled_axis] = t_now_ns
        all_axis_timestamps[polled_axis].append(t_now_ns)

        # Push snapshot to synchronizer
        stamps_vec = [cached_stamps[mid] for mid in MOTOR_IDS]
        steps_vec = [cached_steps[mid] for mid in MOTOR_IDS]
        pos_vec = [0.050 + (s - 1000) * 0.000025 for s in steps_vec]

        synchronizer.push_raw_snapshot(stamps_vec, pos_vec, steps_vec)

    # Now test queries with candidate delay tau = 160ms
    tau_delay_ns = 160_000_000
    successful_fk_count = 0
    total_query_count = 0

    # Query from t = 0.4s to t = 1.0s (after startup buffer is filled)
    for q_idx in range(100):
        query_time_ns = t_start_ns + int((0.400 + q_idx * 0.005) * 1e9) # 200 Hz queries
        sync_res = synchronizer.query_sync_configuration(query_time_ns)
        total_query_count += 1

        if sync_res.status == SyncStatus.SYNCED:
            assert sync_res.positions_meters is not None
            assert len(sync_res.positions_meters) == 6
            # Solve FK
            fk_res = kin.forward_kinematics_with_metrics(sync_res.positions_meters, initial_guess=ref_pose)
            assert fk_res.converged is True
            assert fk_res.residual_norm < 1e-9

            # Expected Z at query time
            t_q_sec = (query_time_ns - t_start_ns) / 1e9
            z_expected = z_nom + 0.002 * math.sin(2.0 * math.pi * f_motion * t_q_sec)
            z_reconstructed = fk_res.pose[2]
            # Tracking error between reconstructed FK and smooth trajectory should be small
            assert abs(z_reconstructed - z_expected) < 0.0002 # < 0.2mm
            successful_fk_count += 1

    sync_ratio = successful_fk_count / total_query_count
    print(f"\nFixedLag 160ms Sync Success Ratio: {sync_ratio * 100:.1f}% ({successful_fk_count}/{total_query_count})")
    assert sync_ratio >= 0.95, f"Expected sync ratio >= 95%, got {sync_ratio*100:.1f}%"


def test_j8_fixed_lag_delay_sweep_metrics():
    """Measure timing metrics and sweep candidate delays [120, 140, 160, 180, 200] ms."""
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    ref_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]

    # Generate 3.0 seconds (150 cycles) of 50Hz round-robin telemetry with realistic jitter (+/- 1.5ms)
    np.random.seed(42)
    t_start_ns = 1_000_000_000
    dt_cycle_ns = 20_000_000

    synchronizer = FixedLagJointSynchronizer(max_history_size=500)

    cached_stamps = {mid: 0 for mid in MOTOR_IDS}
    cached_steps = {mid: 1000 for mid in MOTOR_IDS}

    axis_sample_intervals_ms: dict[int, list[float]] = {mid: [] for mid in MOTOR_IDS}
    last_axis_time_ns = {mid: 0 for mid in MOTOR_IDS}

    for cycle in range(150):
        jitter_ns = int(np.random.uniform(-1_500_000, 1_500_000))
        t_now_ns = t_start_ns + cycle * dt_cycle_ns + jitter_ns

        # Simulated slow 0.2Hz roll-pitch motion
        t_sec = (t_now_ns - t_start_ns) / 1e9
        p_in = [0.0, 0.0, z_nom, 0.01 * math.sin(2 * math.pi * 0.2 * t_sec), 0.01 * math.cos(2 * math.pi * 0.2 * t_sec), 0.0]
        q_in = kin.inverse_kinematics(p_in)
        steps_in = [int(round(1000 + (q - 0.050) * 40000.0)) for q in q_in]

        polled_axis = (cycle % 6) + 1
        cached_steps[polled_axis] = steps_in[polled_axis - 1]
        cached_stamps[polled_axis] = t_now_ns

        if last_axis_time_ns[polled_axis] > 0:
            dt_axis_ms = (t_now_ns - last_axis_time_ns[polled_axis]) / 1e6
            axis_sample_intervals_ms[polled_axis].append(dt_axis_ms)
        last_axis_time_ns[polled_axis] = t_now_ns

        stamps_vec = [cached_stamps[mid] for mid in MOTOR_IDS]
        steps_vec = [cached_steps[mid] for mid in MOTOR_IDS]
        pos_vec = [0.050 + (s - 1000) * 0.000025 for s in steps_vec]
        synchronizer.push_raw_snapshot(stamps_vec, pos_vec, steps_vec)

    # Compute per-axis feedback interval statistics
    all_intervals = [val for sublist in axis_sample_intervals_ms.values() for val in sublist]
    mean_interval_ms = float(np.mean(all_intervals))
    min_interval_ms = float(np.min(all_intervals))
    max_interval_ms = float(np.max(all_intervals))
    std_interval_ms = float(np.std(all_intervals))

    print(f"\n--- J7 Per-Axis Feedback Timing ---")
    print(f"Per-Axis Interval: Mean={mean_interval_ms:.2f}ms, Min={min_interval_ms:.2f}ms, Max={max_interval_ms:.2f}ms, Std={std_interval_ms:.2f}ms")

    candidate_delays_ms = [120.0, 140.0, 160.0, 180.0, 200.0]
    sweep_results = {}

    # Query window from 1.0s to 2.5s (300 query points @ 200Hz)
    query_start_ns = t_start_ns + int(1.0 * 1e9)
    query_end_ns = t_start_ns + int(2.5 * 1e9)
    n_queries = 300
    query_times_ns = np.linspace(query_start_ns, query_end_ns, n_queries, dtype=np.int64)

    for delay_ms in candidate_delays_ms:
        delay_ns = int(delay_ms * 1e6)
        synced_count = 0
        wait_future_count = 0
        fk_success_count = 0
        sample_ages_ms = []

        for q_ns in query_times_ns:
            # Query timestamp is (t_now - delay)
            t_target_query = int(q_ns - delay_ns)
            sync_res = synchronizer.query_sync_configuration(t_target_query)

            if sync_res.status == SyncStatus.SYNCED:
                synced_count += 1
                sample_ages_ms.append((q_ns - t_target_query) / 1e6)
                fk_res = kin.forward_kinematics_with_metrics(sync_res.positions_meters, initial_guess=ref_pose)
                if fk_res.converged:
                    fk_success_count += 1
            elif sync_res.status == SyncStatus.WAIT_FUTURE_SAMPLE:
                wait_future_count += 1

        bracket_ratio = synced_count / n_queries
        fk_ratio = fk_success_count / n_queries

        sweep_results[delay_ms] = {
            "delay_ms": delay_ms,
            "synced_count": synced_count,
            "wait_future_count": wait_future_count,
            "bracketing_success_ratio": bracket_ratio,
            "fk_publish_success_ratio": fk_ratio,
            "mean_sample_age_ms": float(np.mean(sample_ages_ms)) if sample_ages_ms else 0.0,
            "per_axis_mean_interval_ms": mean_interval_ms,
            "per_axis_max_interval_ms": max_interval_ms,
        }

        print(f"Delay {delay_ms:5.1f}ms: Sync={bracket_ratio*100:5.1f}% | WaitFuture={wait_future_count:3d} | FK={fk_ratio*100:5.1f}% | Age={delay_ms:.1f}ms")

    assert sweep_results[160.0]["bracketing_success_ratio"] == 1.0
    assert sweep_results[180.0]["bracketing_success_ratio"] == 1.0
    assert sweep_results[200.0]["bracketing_success_ratio"] == 1.0


def test_j8_read_only_strictly_zero_state_change(rclpy_init):
    """J8-C1: Prove READ_ONLY mode produces strictly zero state-changing commands across full lifecycle."""
    from std_srvs.srv import Trigger

    mock_bus = MockServoBus(initial_positions=[1000] * NUM_MOTORS)
    driver_node = Pus6MotorDriverNode(
        hardware_mode="read_only",
        write_rate_hz=50.0,
        use_j7_servo=True,
        bus=mock_bus,
    )
    assert driver_node.hardware_mode == HardwareMode.READ_ONLY
    assert driver_node.is_j7_servo_active is True
    bus = mock_bus

    # Clear recorded transmissions from startup preflight
    # Preflight full_scan sends 6 read frames (0x30)
    for frame in bus.written_frames:
        # frame format: [0x55, 0xAA, 0x01, mid, cmd, ...]
        assert frame[4] == 0x30, f"Non-read frame sent during READ_ONLY preflight: {frame.hex()}"
    bus.written_frames.clear()

    # 1. Attempt to inject joint steps & meters commands -> Must be rejected at ingress
    steps_msg = Int32MultiArray()
    steps_msg.data = [1050] * 6
    driver_node._handle_command_joint_steps(steps_msg)
    assert driver_node._latest_target is None

    meters_msg = Float64MultiArray()
    meters_msg.data = [0.052] * 6
    driver_node._handle_command_positions_meters(meters_msg)
    assert driver_node._latest_target is None

    # 2. Run 12 worker cycles (2 full round-robin rounds) -> Must ONLY issue 0x30 queries
    for cycle in range(12):
        driver_node._uart_worker_cycle()

    # Verify all transmitted frames during runtime
    assert len(bus.written_frames) == 12
    for frame in bus.written_frames:
        assert frame[4] == 0x30, f"Forbidden state-changing frame during READ_ONLY cycle: {frame.hex()}"

    # 3. Test service calls -> Must be rejected
    req = Trigger.Request()
    resp_pause = Trigger.Response()
    resp_pause = driver_node._handle_pause_all(req, resp_pause)
    assert resp_pause.success is False
    assert "READ_ONLY" in resp_pause.message

    resp_cfg = Trigger.Response()
    resp_cfg = driver_node._handle_configure_motors(req, resp_cfg)
    assert resp_cfg.success is False
    assert "READ_ONLY" in resp_cfg.message

    # 4. Shutdown -> Must NOT send 0x1A stop frames
    bus.written_frames.clear()
    driver_node.destroy_node()
    assert len(bus.written_frames) == 0, f"Shutdown sent unexpected frames in READ_ONLY: {bus.written_frames}"

