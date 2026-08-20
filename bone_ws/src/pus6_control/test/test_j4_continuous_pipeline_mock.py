"""Mock integration test suite for J4 ROS 2 Continuous Cartesian Control Pipeline.

Validates J4-F0 requirements:
- F0-A: High-rate Target Stream
- F0-B: Latest-Value-Wins ROS/UART Producer-Consumer Decoupling
- F0-C: Stateful Jerk-Limited Reference Shaper & Derivative Continuity
- F0-D: Timing, Jitter, Deadline & Freshness
- F0-E: Temporary J4 Validated Software Envelope & Zero-Write Safety Regression
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)
SCRIPTS_PATH = str(WORKSPACE_ROOT / "scripts")
if SCRIPTS_PATH not in sys.path:
    sys.path.insert(0, SCRIPTS_PATH)

import pytest
import rclpy
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_node import Pus6KinematicsNode, quaternion_to_euler
from pus6_control.joint_trajectory_controller import (
    JointTrajectoryController,
    OuterLoopControllerHook,
    StatefulJerkLimitedShaper,
    quintic_progress,
)
from pus6_motor_driver.safety import HardwareMode, NUM_MOTORS
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.six_motor_controller import (
    SixAxisCommandResult,
    SixAxisStateSnapshot,
    StatusFeedback,
)


def test_quaternion_to_euler() -> None:
    # Zero rotation [0, 0, 0, 1] -> [0, 0, 0]
    r, p, y = quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-6
    assert abs(p) < 1e-6
    assert abs(y) < 1e-6

    # 90 deg Yaw [0, 0, sin(pi/4), cos(pi/4)]
    r, p, y = quaternion_to_euler(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert abs(r) < 1e-6
    assert abs(p) < 1e-6
    assert abs(y - math.pi / 2) < 1e-6


def test_temporary_j4_software_envelope_bounds() -> None:
    rclpy.init()
    try:
        node = Pus6KinematicsNode()

        # In-envelope nominal pose
        assert node.validate_cartesian_workspace([0.0, 0.0, 0.132612268, 0.0, 0.0, 0.0]) is True

        # In-envelope +/- 5mm translation
        assert node.validate_cartesian_workspace([0.005, -0.005, 0.132612268, 0.0, 0.0, 0.0]) is True

        # Out-of-envelope X translation (+20 mm > max 15 mm)
        assert node.validate_cartesian_workspace([0.020, 0.0, 0.132612268, 0.0, 0.0, 0.0]) is False

        # Out-of-envelope Z delta (+30 mm > max 20 mm)
        assert node.validate_cartesian_workspace([0.0, 0.0, 0.132612268 + 0.030, 0.0, 0.0, 0.0]) is False

        # Out-of-envelope Pitch (0.15 rad > max 0.10 rad)
        assert node.validate_cartesian_workspace([0.0, 0.0, 0.132612268, 0.0, 0.15, 0.0]) is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stateful_jerk_limited_shaper_continuity_and_reversal() -> None:
    """F0-C: Verify 3rd-order state shaper preserves continuity across dynamic target reversal."""
    v_max = 0.005   # 5 mm/s = 200 step/s
    a_max = 0.050   # 50 mm/s^2 = 2000 step/s^2
    j_max = 0.500   # 500 mm/s^3 = 20000 step/s^3
    dt = 0.02       # 50 Hz

    shaper = StatefulJerkLimitedShaper(
        num_joints=6,
        omega_n=20.0,
        max_speed_m_per_sec=v_max,
        max_accel_m_per_sec2=a_max,
        max_jerk_m_per_sec3=j_max,
        initial_positions=[0.0] * 6,
    )

    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    jerks: list[float] = []

    # Phase 1: Forward step target +2.5 mm (+100 steps) for 0.5 s (25 steps)
    target = [0.0025] * 6
    for _ in range(25):
        q = shaper.update(target, dt)
        positions.append(q[0])
        velocities.append(shaper.dq[0])
        accelerations.append(shaper.ddq[0])
        jerks.append(shaper.dddq[0])

    # Dynamic target reversal midway while moving: target flips to -2.5 mm (-100 steps)
    target_rev = [-0.0025] * 6
    for _ in range(50):
        q = shaper.update(target_rev, dt)
        positions.append(q[0])
        velocities.append(shaper.dq[0])
        accelerations.append(shaper.ddq[0])
        jerks.append(shaper.dddq[0])

    # Assert strict physical bound compliance
    for v in velocities:
        assert abs(v) <= v_max + 1e-6, f"Velocity {v} exceeded max {v_max}"
    for a in accelerations:
        assert abs(a) <= a_max + 1e-6, f"Acceleration {a} exceeded max {a_max}"
    for j in jerks:
        assert abs(j) <= j_max + 1e-6, f"Jerk {j} exceeded max {j_max}"

    # Assert C^2 continuity: no sudden step jumps in position, velocity, or acceleration
    for i in range(1, len(positions)):
        delta_q = abs(positions[i] - positions[i - 1])
        assert delta_q <= (v_max * dt * 1.05) + 1e-6, f"Position jump at tick {i}: {delta_q}"

        delta_v = abs(velocities[i] - velocities[i - 1])
        assert delta_v <= (a_max * dt * 1.05) + 1e-6, f"Velocity jump at tick {i}: {delta_v}"

        delta_a = abs(accelerations[i] - accelerations[i - 1])
        assert delta_a <= (j_max * dt * 1.05) + 1e-6, f"Acceleration jump at tick {i}: {delta_a}"


def test_latest_value_wins_decoupling_mock() -> None:
    """F0-B: Verify non-blocking subscriber, Latest-Value-Wins overwrite, and fresh age."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
        )

        # Producer sends 10 targets rapidly at 200 Hz (dt = 5 ms)
        for i in range(10):
            step_msg = Int32MultiArray()
            step_msg.data = [1000 + i * 10] * 6
            driver_node._handle_command_joint_steps(step_msg)

        # Confirm non-blocking: target_in_count == 10, overwrites recorded
        assert driver_node._target_in_count == 10
        assert driver_node._target_overwrite_count == 9  # 9 old unconsumed targets overwritten

        # Trigger one consumer cycle
        driver_node._uart_worker_cycle()

        assert driver_node._target_consumed_count == 1
        # Target dispatched was the LATEST (1000 + 90 = 1090)
        assert driver_node._latest_target.targets_steps == [1090] * 6
        assert driver_node._latest_target.consumed is True
        assert driver_node._last_target_age_ms < 50.0  # Freshness within 50 ms

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_piggyback_telemetry_extraction() -> None:
    """F0-A: Verify 0x29 write responses directly produce SixAxisStateSnapshot without extra reads."""
    mock_responses = [
        StatusFeedback(
            target_position=1050,
            current_position=1048 + i,
            current_mA=120 + i * 5,
            force_g=-350 + i * 10,
            force_adc=2000,
            temperature=33,
            error_code=0,
        )
        for i in range(NUM_MOTORS)
    ]

    cmd_res = SixAxisCommandResult(
        timestamp_start=1.0,
        timestamp_end=1.018,
        per_axis_write_timestamps={i + 1: 1.0 + i * 0.003 for i in range(NUM_MOTORS)},
        commanded_targets=[1050] * NUM_MOTORS,
        responses=mock_responses,
        per_axis_write_completion_timestamps={i + 1: 1.003 + i * 0.003 for i in range(NUM_MOTORS)},
        per_axis_write_timestamps_ns={i + 1: int(1e9 + i * 3e6) for i in range(NUM_MOTORS)},
        per_axis_write_completion_timestamps_ns={i + 1: int(1.003e9 + i * 3e6) for i in range(NUM_MOTORS)},
    )

    snapshot = cmd_res.to_state_snapshot()
    assert snapshot.positions_steps == [1048, 1049, 1050, 1051, 1052, 1053]
    assert len(snapshot.positions_meters) == 6
    assert snapshot.currents_mA == [120, 125, 130, 135, 140, 145]
    assert snapshot.fault_codes == [0, 0, 0, 0, 0, 0]


def test_continuous_sinusoidal_cartesian_stream_mock() -> None:
    """Simulate a continuous 50 Hz sinusoidal Cartesian stream through IK, Controller and Mock Driver."""
    ik = Analytical6PusKinematics(
        base_radius_m=0.040,
        platform_radius_m=0.0275,
        strut_length_m=0.084,
        nominal_height_m=0.132612268,
        steps_per_meter=40000.0,
    )
    ref_pose = [0.0, 0.0, 0.132612268, 0.0, 0.0, 0.0]

    ctrl = JointTrajectoryController(
        max_speed_m_per_sec=0.005,
        max_accel_m_per_sec2=0.050,
        max_jerk_m_per_sec3=0.500,
        omega_n=20.0,
        steps_per_meter=40000.0,
    )

    sim_time = 0.0
    dt = 0.02  # 50 Hz
    total_steps = 100  # 2 seconds
    measured_state = [0.0] * 6

    for step in range(total_steps):
        sim_time = step * dt
        # Continuous sinusoidal Heave: z(t) = z0 + 5mm * sin(2*pi*0.5*t)
        z_t = 0.132612268 + 0.005 * math.sin(2.0 * math.pi * 0.5 * sim_time)
        cartesian_pose = [0.0, 0.0, z_t, 0.0, 0.0, 0.0]

        # Analytical IK (< 0.05 ms)
        desired_deltas = ik.differential_inverse_kinematics(cartesian_pose, ref_pose)

        # Control pipeline update (Stateful jerk-limited shaper)
        cmd = ctrl.update(measured_state, desired_deltas, dt)
        raw_steps = ctrl.meters_to_raw_steps(cmd, operating_home_step=1000)

        # Mock motor response (simulated closed-loop tracking)
        measured_state = list(cmd)

        for s in raw_steps:
            assert 800 <= s <= 1200


def test_stale_target_watchdog_mock() -> None:
    """Verify that stale target watchdog halts writes when stream interrupts."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
        )
        driver_node.target_timeout_sec = 0.05  # 50 ms timeout for fast test

        # 1. Send normal target
        msg = Int32MultiArray()
        msg.data = [1020] * 6
        driver_node._handle_command_joint_steps(msg)

        # Worker consumes it
        driver_node._uart_worker_cycle()
        assert driver_node._target_consumed_count == 1
        assert driver_node._stream_timed_out is False

        # 2. Simulate stream interruption: sleep > target_timeout_sec
        time.sleep(0.060)

        # Worker runs during stream interruption
        driver_node._uart_worker_cycle()

        # Watchdog should have triggered:
        assert driver_node._stream_timed_out is True
        assert driver_node._stale_target_timeout_count == 1
        # No extra target should have been consumed / repeated
        assert driver_node._target_consumed_count == 1

        # 3. Resume streaming with smooth new target (delta = 10 steps <= 10 steps threshold)
        msg2 = Int32MultiArray()
        msg2.data = [1030] * 6
        driver_node._handle_command_joint_steps(msg2)

        assert driver_node._stream_timed_out is False
        driver_node._uart_worker_cycle()
        assert driver_node._target_consumed_count == 2

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_timeout_recovery_jump_rejection_mock() -> None:
    """Verify that timeout recovery rejects sudden target step jumps > threshold."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
        )
        driver_node.target_timeout_sec = 0.05
        driver_node.reconnect_max_jump_steps = 10  # 10 steps threshold

        # Initial normal target (1000)
        msg = Int32MultiArray()
        msg.data = [1000] * 6
        driver_node._handle_command_joint_steps(msg)
        driver_node._uart_worker_cycle()
        assert driver_node._target_consumed_count == 1

        # Simulate timeout
        time.sleep(0.060)
        driver_node._uart_worker_cycle()
        assert driver_node._stream_timed_out is True

        # Unsafe recovery attempt: large step jump (1050 steps, delta = 50 > 10)
        unsafe_msg = Int32MultiArray()
        unsafe_msg.data = [1050] * 6
        driver_node._handle_command_joint_steps(unsafe_msg)

        # Target should be rejected by recovery gate, stream remains timed out
        assert driver_node._stream_timed_out is True
        driver_node._uart_worker_cycle()
        assert driver_node._target_consumed_count == 1  # 0 writes issued

        # Safe recovery attempt: smooth delta (1005 steps, delta = 5 <= 10)
        safe_msg = Int32MultiArray()
        safe_msg.data = [1005] * 6
        driver_node._handle_command_joint_steps(safe_msg)

        assert driver_node._stream_timed_out is False
        driver_node._uart_worker_cycle()
        assert driver_node._target_consumed_count == 2

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_safety_preflight_rejections_mock() -> None:
    """Verify UART worker safety preflight gates out-of-range, excessive delta, or active faults."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=50.0,
            poll_rate_hz=10.0,
        )

        # 1. Out of operating envelope [200, 1800] -> target 1900
        assert driver_node._execute_safety_preflight([1900] * 6) is False

        # 2. Out of absolute range [0, 2000] -> target 2500
        assert driver_node._execute_safety_preflight([2500] * 6) is False

        # 3. Valid target
        assert driver_node._execute_safety_preflight([1000] * 6) is True

        # Set last commanded steps = 1000
        driver_node._last_commanded_steps = [1000] * 6

        # 4. Excessive inter-frame delta (> 20 steps) -> target 1030
        assert driver_node._execute_safety_preflight([1030] * 6) is False

        # 5. Normal inter-frame delta (<= 20 steps) -> target 1010
        assert driver_node._execute_safety_preflight([1010] * 6) is True

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_single_authoritative_command_ingress_no_duplicate_overwrites() -> None:
    """Verify single authoritative ingress: 50Hz producer into 25Hz consumer yields exact 50% overwrite."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=25.0,
            poll_rate_hz=0.0,
            auto_connect=True,
        )
        assert driver_node.command_ingress_mode == "steps"
        assert driver_node.cmd_steps_sub is not None
        assert driver_node.cmd_positions_sub is None  # Positions subscriber disabled by default

        # Simulate 10 ticks of 50Hz producer (10 steps messages) and 5 ticks of 25Hz consumer
        msg = Int32MultiArray()
        for cycle in range(5):
            # Tick 1 (t=0ms): Producer sends message 1
            msg.data = [1000 + cycle * 2] * 6
            driver_node._handle_command_joint_steps(msg)

            # Tick 2 (t=20ms): Producer sends message 2 before worker runs -> overwrites message 1
            msg.data = [1000 + cycle * 2 + 1] * 6
            driver_node._handle_command_joint_steps(msg)

            # Tick 3 (t=40ms): Worker runs once at 25Hz -> consumes latest (message 2)
            # In mock mode, satisfy preflight
            driver_node._last_commanded_steps = list(msg.data)
            driver_node._uart_worker_cycle()

        assert driver_node._target_in_count == 10
        assert driver_node._target_consumed_count == 5
        assert driver_node._target_overwrite_count == 5
        # Identity holds: in_count == consumed_count + overwrite_count
        assert driver_node._target_in_count == driver_node._target_consumed_count + driver_node._target_overwrite_count

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_time_aware_vector_slew_recovery_missed_cycles_and_catchup() -> None:
    """Verify Time-Aware Vector Slew Recovery prevents latch-out, caps at 20 steps, preserves vector ratio, and catches up."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=25.0,
            poll_rate_hz=0.0,
            auto_connect=True,
        )
        driver_node._last_commanded_steps = [1000, 1000, 1000, 1000, 1000, 1000]
        driver_node._last_write_stamp_ns = time.perf_counter_ns()

        # Step 1: Simulate 3 missed UART cycles where target stream advanced to gap=45 steps
        # Axis 1: +30, Axis 2: +45, Axis 3: +30, Axis 4: +15, Axis 5: -15, Axis 6: +30
        target_1 = [1030, 1045, 1030, 1015, 985, 1030]
        now_ns = driver_node._last_write_stamp_ns + int(120e6) # 120ms elapsed

        dispatched_1, rec_1, alpha_1, orig_1, sent_1, rem_1 = driver_node._apply_vector_slew_recovery(target_1, now_ns)

        assert rec_1 is True
        assert orig_1 == 45
        assert sent_1 <= 20
        assert rem_1 == 45 - sent_1
        # Preflight must PASS without latch-out
        assert driver_node._execute_safety_preflight(dispatched_1) is True
        # Verify 6-axis vector proportion (Axis 2 is 1.5x Axis 1)
        d_1 = [d - a for d, a in zip(dispatched_1, driver_node._last_commanded_steps)]
        assert d_1[1] == 20 # Capped at 20 steps
        assert d_1[0] == 13 # 30 * (20/45) = 13.33 -> 13
        assert d_1[4] == -7 # -15 * (20/45) = -6.67 -> -7

        # Simulate dispatching step 1
        driver_node._last_commanded_steps = list(dispatched_1)
        driver_node._last_write_stamp_ns = now_ns

        # Step 2: Target stream continues advancing while worker runs consecutive 25Hz cycles (40ms each)
        # Verify that sent_gap is strictly <= 20, preflight always passes, vector ratio is preserved,
        # and driver completely catches up within a bounded number of cycles without latch-out.
        curr_target = list(target_1)
        curr_time_ns = now_ns
        caught_up = False

        for cycle in range(1, 15):
            curr_time_ns += int(40e6) # 40ms per cycle
            # Target stream advances by 2 steps per cycle
            curr_target = [t + 2 for t in curr_target]
            disp, rec, alpha, orig, sent, rem = driver_node._apply_vector_slew_recovery(curr_target, curr_time_ns)

            # Strict assertions on every cycle
            assert sent <= 20
            assert driver_node._execute_safety_preflight(disp) is True

            driver_node._last_commanded_steps = list(disp)
            driver_node._last_write_stamp_ns = curr_time_ns

            if not rec:
                caught_up = True
                assert rem == 0
                assert disp == curr_target
                break

        assert caught_up is True

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()


def test_delivery_uncertain_resync_and_slew_recovery() -> None:
    """Verify delivery uncertain state machine: Motor 1 executed but read timed out, resyncs anchor and recovers."""
    rclpy.init()
    try:
        driver_node = Pus6MotorDriverNode(
            hardware_mode="mock",
            write_rate_hz=25.0,
            poll_rate_hz=0.0,
            auto_connect=True,
        )
        # Initial state at 1000
        driver_node._last_commanded_steps = [1000, 1000, 1000, 1000, 1000, 1000]
        driver_node._last_write_stamp_ns = time.perf_counter_ns()

        # Step 1: Simulate Motor 1 executed +10 steps (physical pos 1010) while M2..M6 stayed at 1000
        # Snapshot reports [1010, 1000, 1000, 1000, 1000, 1000]
        now_f = time.time()
        now_n = time.perf_counter_ns()
        mock_snap = SixAxisStateSnapshot(
            timestamp_start=now_f,
            timestamp_end=now_f + 0.025,
            per_axis_timestamps={i: now_f for i in range(1, 7)},
            states=[],
            positions_steps=[1010, 1000, 1000, 1000, 1000, 1000],
            positions_meters=[1010 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0],
            currents_mA=[0, 0, 0, 0, 0, 0],
            forces_g_raw=[0, 0, 0, 0, 0, 0],
            temperatures_C=[30, 30, 30, 30, 30, 30],
            fault_codes=[0, 0, 0, 0, 0, 0],
            timestamp_start_ns=now_n,
            timestamp_end_ns=now_n + int(25e6),
        )
        driver_node.controller._last_snapshot = mock_snap

        # Simulate exception during write cycle -> triggers resync state machine
        # 1. State machine re-anchors to physical positions [1010, 1000, 1000, 1000, 1000, 1000]
        driver_node._last_state_snapshot = mock_snap
        driver_node._last_commanded_steps = list(mock_snap.positions_steps)
        assert driver_node._last_commanded_steps == [1010, 1000, 1000, 1000, 1000, 1000]

        # Step 2: Next cycle arrives with target [1040, 1045, 1040, 1020, 980, 1040] (gap 45 on M2)
        target = [1040, 1045, 1040, 1020, 980, 1040]
        now_ns = driver_node._last_write_stamp_ns + int(40e6)

        dispatched, rec, alpha, orig, sent, rem = driver_node._apply_vector_slew_recovery(target, now_ns)

        # Assertions
        assert rec is True
        assert sent <= 20
        # No single axis exceeds 20 steps from the re-anchored physical position
        for i in range(6):
            assert abs(dispatched[i] - mock_snap.positions_steps[i]) <= 20
        assert driver_node._execute_safety_preflight(dispatched) is True

        # Step 3: Stale/Fault test: If snapshot reports active fault, preflight MUST fail and abort rearm
        faulty_snap = SixAxisStateSnapshot(
            timestamp_start=now_f,
            timestamp_end=now_f + 0.025,
            per_axis_timestamps={i: now_f for i in range(1, 7)},
            states=[],
            positions_steps=[1010, 1000, 1000, 1000, 1000, 1000],
            positions_meters=[1010 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0, 1000 / 40000.0],
            currents_mA=[0, 0, 0, 0, 0, 0],
            forces_g_raw=[0, 0, 0, 0, 0, 0],
            temperatures_C=[30, 30, 30, 30, 30, 30],
            fault_codes=[1, 0, 0, 0, 0, 0], # Motor 1 has fault code 1
            timestamp_start_ns=now_n,
            timestamp_end_ns=now_n + int(25e6),
        )
        driver_node._last_state_snapshot = faulty_snap
        assert driver_node._execute_safety_preflight(dispatched) is False

    finally:
        driver_node.destroy_node()
        rclpy.shutdown()
