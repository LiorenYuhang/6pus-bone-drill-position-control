#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for Stage J13-2A1: LADRC Shadow Runner (j13_ladrc_shadow_runner.py).

Verifies:
  - Test 1: Shadow LADRC receives same SO(3) error as Classical controller
  - Test 2: Classical output enters attitude composition
  - Test 3: LADRC shadow output NEVER enters published target pose or kinematics
  - Test 4: Fault isolation: LADRC anomaly/NaN does not affect Classical control path
  - Test 5: LESO warm-start initialization upon entering ACTIVE (z1=e0, z2=0)
  - Test 6: LESO reset upon exiting ACTIVE or encountering fault
  - Test 7: Telemetry fields completeness and schema compliance
  - Test 8: Delta comparison metric (u_shadow - u_classical) accuracy
  - Test 9: hardware_write_count strictly 0 in dry-run mode
"""

import math
from pathlib import Path
import sys
import time
from dataclasses import asdict
import numpy as np
import pytest

# Add scripts directory to path
_SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

try:
    import rclpy
except ImportError:
    rclpy = None

from j13_ladrc_shadow_runner import (
    J13LadrcShadowRunner,
    J13ShadowState,
    J13LadrcTelemetry,
    compose_so3_pid_residual,
)


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initializes rclpy session for nodes used in tests."""
    if rclpy is not None and not rclpy.ok():
        rclpy.init()
    yield
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


def _create_mock_runner(
    execute_motion: bool = False,
    dry_run: bool = True,
    controller_mode: str = "shadow",
    safe_takeover_jump_threshold_deg: float = 0.05,
) -> J13LadrcShadowRunner:
    runner = J13LadrcShadowRunner(
        execute_motion=execute_motion,
        dry_run=dry_run,
        auto_start=True,
        controller_mode=controller_mode,
        safe_takeover_jump_threshold_deg=safe_takeover_jump_threshold_deg,
    )
    now = time.perf_counter_ns()
    # Establish stationary reference at identity
    runner.q0_base_raw = (0.0, 0.0, 0.0, 1.0)
    runner.q0_plat_raw = (0.0, 0.0, 0.0, 1.0)
    runner.latest_base_q_raw = (0.0, 0.0, 0.0, 1.0)
    runner.latest_plat_q_raw = (0.0, 0.0, 0.0, 1.0)
    runner.latest_base_rx_time_ns = now
    runner.latest_plat_rx_time_ns = now
    runner.latest_fb_rx_time_ns = now
    runner.has_received_real_feedback = True
    runner.last_tick_time_ns = now - 20_000_000
    return runner


def _tick(runner: J13LadrcShadowRunner) -> None:
    """Invokes control tick with fresh synthetic timestamps (<100ms age)."""
    now = time.perf_counter_ns()
    runner.latest_base_rx_time_ns = now
    runner.latest_plat_rx_time_ns = now
    runner.latest_fb_rx_time_ns = now
    runner.last_tick_time_ns = now - 20_000_000
    runner._on_control_tick()


def test_01_same_so3_error_to_both_controllers():
    """Test 1: Classical P/PD and LADRC Shadow receive identical SO(3) tracking error."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE

        # Apply 0.3 deg roll rotation to platform
        roll_rad = math.radians(0.3)
        runner.latest_plat_q_raw = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))

        for _ in range(50):
            _tick(runner)
        telem = runner.telemetry_history[-1]

        # Verify tracking error is non-zero
        assert np.linalg.norm(telem.e_R_W_rad) > 0.001

        # Verify LADRC observer state z1 has converged to this error across all axes
        assert np.allclose(np.array(telem.ladrc_z1_rad), np.array(telem.e_R_W_rad), atol=0.0005)
    finally:
        runner.destroy_node()


def test_02_classical_output_enters_attitude_composition():
    """Test 2: Real attitude composition strictly uses classical_u_actual."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE

        roll_rad = math.radians(0.2)
        runner.latest_plat_q_raw = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))

        _tick(runner)
        telem = runner.telemetry_history[-1]

        # Recompute authoritative composition using classical_u_actual
        R_WB = np.array(telem.R_WB, dtype=np.float64)
        R_BP_FF = np.array(telem.R_BP_FF, dtype=np.float64)
        u_classical = telem.classical_u_actual_rad

        _, _, expected_R_BP_cmd, expected_quat = compose_so3_pid_residual(
            R_WB, R_BP_FF, u_classical
        )

        assert np.allclose(np.array(telem.R_BP_cmd), expected_R_BP_cmd, atol=1e-12)
        assert np.allclose(np.array(telem.cmd_quat_xyzw), np.array(expected_quat), atol=1e-12)
    finally:
        runner.destroy_node()


def test_03_ladrc_shadow_output_never_enters_target_pose():
    """Test 3: LADRC shadow output has zero influence on commanded attitude or target pose."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE

        roll_rad = math.radians(0.4)
        runner.latest_plat_q_raw = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))

        _tick(runner)
        telem = runner.telemetry_history[-1]

        # u_ladrc_shadow is different from u_classical_actual
        u_shadow = telem.ladrc_u_shadow_rad
        u_classical = telem.classical_u_actual_rad
        assert u_shadow[0] != u_classical[0]

        # Recompute composition using shadow output to prove it DOES NOT match telem.R_BP_cmd
        R_WB = np.array(telem.R_WB, dtype=np.float64)
        R_BP_FF = np.array(telem.R_BP_FF, dtype=np.float64)
        _, _, shadow_R_BP_cmd, shadow_quat = compose_so3_pid_residual(
            R_WB, R_BP_FF, u_shadow
        )

        # Prove telem.cmd_quat_xyzw differs from shadow_quat
        assert not np.allclose(np.array(telem.cmd_quat_xyzw), np.array(shadow_quat), atol=1e-5)
    finally:
        runner.destroy_node()


def test_04_ladrc_fault_isolation():
    """Test 4: LADRC anomaly or NaN in shadow channel does not affect Classical control path."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE

        # Force LADRC channel states to corrupted values
        runner.ladrc.channels[0].z1 = float("nan")
        runner.ladrc.channels[0].z2 = float("nan")

        _tick(runner)
        telem = runner.telemetry_history[-1]

        # Classical control path remains fully finite and unaffected
        assert all(math.isfinite(x) for x in telem.classical_u_actual_rad)
        assert all(math.isfinite(x) for x in telem.cmd_quat_xyzw)
        assert runner.state == J13ShadowState.ACTIVE
    finally:
        runner.destroy_node()


def test_05_leso_warm_start_initialization():
    """Test 5: Transitioning from READY to ACTIVE initializes LESO with current error (z1=e0, z2=0)."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.READY

        # Preset platform orientation with 0.15 deg error
        err_rad = math.radians(0.15)
        runner.latest_plat_q_raw = (math.sin(err_rad / 2.0), 0.0, 0.0, math.cos(err_rad / 2.0))

        _tick(runner)

        # State transitioned to ACTIVE
        assert runner.state == J13ShadowState.ACTIVE
        # LESO z1 should be initialized close to the measured error
        assert math.isclose(runner.ladrc.channels[0].z1, err_rad, abs_tol=0.01)
        # Disturbance estimate z2 must be zero at cold/warm start
        assert abs(runner.ladrc.channels[0].z2) < 0.01
    finally:
        runner.destroy_node()


def test_06_leso_reset_on_fault():
    """Test 6: Entering FAULT state triggers complete reset of LESO."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE
        runner.ladrc.channels[0].z1 = 0.05
        runner.ladrc.channels[0].z2 = 0.10

        runner._enter_fault("Test fault trigger")

        assert runner.state == J13ShadowState.FAULT
        assert runner.ladrc.channels[0].z1 == 0.0
        assert runner.ladrc.channels[0].z2 == 0.0
    finally:
        runner.destroy_node()


def test_07_telemetry_completeness():
    """Test 7: All mandatory telemetry fields for J13 shadow mode exist in telemetry record."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE
        _tick(runner)

        telem = runner.telemetry_history[-1]
        d = asdict(telem)

        required_keys = [
            "timestamp_ns",
            "cycle_index",
            "dt_ms",
            "state",
            "controller_mode",
            "active_axis",
            "takeover_cycle_index",
            "command_jump_at_takeover_rad",
            "command_jump_at_takeover_deg",
            "u_combined_actual_rad",
            "u_combined_actual_deg",
            "e_R_W_rad",
            "e_R_W_deg",
            "classical_u_raw_rad",
            "classical_u_raw_deg",
            "classical_u_actual_rad",
            "classical_u_actual_deg",
            "classical_saturated_axes",
            "ladrc_z1_rad",
            "ladrc_z1_deg",
            "ladrc_z2_rad_s",
            "ladrc_z2_deg_s",
            "ladrc_u_raw_rad",
            "ladrc_u_raw_deg",
            "ladrc_u_shadow_rad",
            "ladrc_u_shadow_deg",
            "ladrc_saturated_axes",
            "observer_previous_plant_input_rad",
            "observer_previous_plant_input_deg",
            "delta_u_shadow_vs_classical_rad",
            "delta_u_shadow_vs_classical_deg",
            "b0",
            "omega_c",
            "omega_o",
            "base_rel_quat_xyzw",
            "platform_rel_quat_xyzw",
            "R_BP_cmd",
            "cmd_quat_xyzw",
            "predicted_steps",
            "commanded_steps",
            "feedback_steps",
        ]

        for k in required_keys:
            assert k in d, f"Missing telemetry key: {k}"
    finally:
        runner.destroy_node()


def test_08_delta_u_shadow_vs_classical():
    """Test 8: delta_u_shadow_vs_classical is accurately computed as (u_shadow - u_classical)."""
    runner = _create_mock_runner()
    try:
        runner.state = J13ShadowState.ACTIVE

        roll_rad = math.radians(0.25)
        runner.latest_plat_q_raw = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))

        _tick(runner)
        telem = runner.telemetry_history[-1]

        for i in range(3):
            diff = telem.ladrc_u_shadow_rad[i] - telem.classical_u_actual_rad[i]
            assert math.isclose(telem.delta_u_shadow_vs_classical_rad[i], diff, abs_tol=1e-12)
    finally:
        runner.destroy_node()


def test_09_dry_run_hardware_write_count_zero():
    """Test 9: In dry-run mode, hardware_write_count remains strictly 0."""
    runner = _create_mock_runner(execute_motion=False, dry_run=True)
    try:
        runner.state = J13ShadowState.ACTIVE

        for _ in range(50):
            _tick(runner)

        assert runner.hardware_write_count == 0
        assert all(not t.hardware_write_issued for t in runner.telemetry_history)
    finally:
        runner.destroy_node()


def test_10_roll_active_residual_routing():
    """Test 10: In roll_active mode, Roll residual strictly matches LADRC, Pitch/Yaw match Classical."""
    runner = _create_mock_runner(controller_mode="roll_active")
    try:
        runner.state = J13ShadowState.ACTIVE
        # Set 0.2 deg roll and 0.1 deg pitch on platform
        roll_rad = math.radians(0.2)
        pitch_rad = math.radians(0.1)
        runner.latest_plat_q_raw = (
            math.sin(roll_rad / 2.0),
            math.sin(pitch_rad / 2.0),
            0.0,
            math.cos(roll_rad / 2.0) * math.cos(pitch_rad / 2.0),
        )

        _tick(runner)
        telem = runner.telemetry_history[-1]

        assert telem.controller_mode == "roll_active"
        assert telem.active_axis == "roll"

        # Roll: strictly matches LADRC roll output
        assert math.isclose(telem.u_combined_actual_rad[0], telem.ladrc_u_shadow_rad[0], abs_tol=1e-12)
        # Pitch & Yaw: strictly match Classical P/PD outputs
        assert math.isclose(telem.u_combined_actual_rad[1], telem.classical_u_actual_rad[1], abs_tol=1e-12)
        assert math.isclose(telem.u_combined_actual_rad[2], telem.classical_u_actual_rad[2], abs_tol=1e-12)

        # Authoritative composition must match u_combined_actual
        R_WB = np.array(telem.R_WB, dtype=np.float64)
        R_BP_FF = np.array(telem.R_BP_FF, dtype=np.float64)
        _, _, expected_R_BP_cmd, expected_quat = compose_so3_pid_residual(
            R_WB, R_BP_FF, telem.u_combined_actual_rad
        )
        assert np.allclose(np.array(telem.R_BP_cmd), expected_R_BP_cmd, atol=1e-12)
        assert np.allclose(np.array(telem.cmd_quat_xyzw), np.array(expected_quat), atol=1e-12)
    finally:
        runner.destroy_node()


def test_11_roll_active_observer_previous_input_routing():
    """Test 11: In roll_active mode, Roll LESO input uses actual Roll LADRC input, Pitch/Yaw use Classical."""
    runner = _create_mock_runner(controller_mode="roll_active")
    try:
        runner.state = J13ShadowState.ACTIVE
        roll_rad = math.radians(0.2)
        runner.latest_plat_q_raw = (math.sin(roll_rad / 2.0), 0.0, 0.0, math.cos(roll_rad / 2.0))

        # Tick 1
        _tick(runner)
        telem1 = runner.telemetry_history[-1]
        u_combined_tick1 = list(telem1.u_combined_actual_rad)

        # Tick 2
        _tick(runner)
        telem2 = runner.telemetry_history[-1]

        # On Tick 2, observer_previous_plant_input must match Tick 1's combined actual input!
        assert math.isclose(telem2.observer_previous_plant_input_rad[0], u_combined_tick1[0], abs_tol=1e-12)
        assert math.isclose(telem2.observer_previous_plant_input_rad[1], u_combined_tick1[1], abs_tol=1e-12)
        assert math.isclose(telem2.observer_previous_plant_input_rad[2], u_combined_tick1[2], abs_tol=1e-12)

        # In particular, Roll input was LADRC, not Classical
        assert math.isclose(telem2.observer_previous_plant_input_rad[0], telem1.ladrc_u_shadow_rad[0], abs_tol=1e-12)
    finally:
        runner.destroy_node()


def test_12_roll_active_bumpless_takeover_check():
    """Test 12: Takeover cycle index and command jump are recorded correctly."""
    runner = _create_mock_runner(controller_mode="roll_active", safe_takeover_jump_threshold_deg=0.05)
    try:
        runner.state = J13ShadowState.ACTIVE
        _tick(runner)
        telem = runner.telemetry_history[-1]

        assert telem.takeover_cycle_index == 1
        assert telem.command_jump_at_takeover_deg >= 0.0
        assert telem.command_jump_at_takeover_deg < 0.05
        assert runner.state == J13ShadowState.ACTIVE
    finally:
        runner.destroy_node()


def test_13_roll_active_takeover_jump_threshold_exceeded():
    """Test 13: If command jump at takeover exceeds safe threshold, runner halts into FAULT."""
    runner = _create_mock_runner(controller_mode="roll_active", safe_takeover_jump_threshold_deg=1e-8)
    try:
        runner.state = J13ShadowState.ACTIVE
        roll_rad = math.radians(0.1)
        runner.latest_plat_q_raw = (0.0, math.sin(roll_rad / 2.0), 0.0, math.cos(roll_rad / 2.0))

        _tick(runner)
        assert runner.state == J13ShadowState.FAULT
        assert "Bumpless takeover check FAILED" in runner.fault_reason
    finally:
        runner.destroy_node()


def test_14_roll_active_ladrc_fault_fallback():
    """Test 14: In roll_active mode, corrupted LADRC Roll triggers fault and safe fallback."""
    runner = _create_mock_runner(controller_mode="roll_active")
    try:
        runner.state = J13ShadowState.ACTIVE
        # Force corruption on LADRC Roll
        runner.ladrc.channels[0].z1 = float("nan")

        _tick(runner)
        assert runner.state == J13ShadowState.FAULT
        assert "LADRC Roll fault" in runner.fault_reason
    finally:
        runner.destroy_node()
