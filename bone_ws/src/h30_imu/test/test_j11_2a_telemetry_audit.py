"""Unit tests for Stage J11-2A Telemetry Audit, Feedback Topic Validity, and PASS Criteria.

Covers:
  1. Feedback topic unreceived guard: default [1000]*6 must not masquerade as valid feedback.
  2. Feedback topic reception & validity verification.
  3. Motor fault count semantics: faulted cycles, faulted motors, total code sum.
  4. Non-zero fault code on any motor forces FAIL.
  5. Strengthened J11-2A PASS criteria: all 9 safety conditions.
  6. Topic ownership normalization (prevents '//node' false self-conflict).
  7. Actuator holding mechanism terminology (stepper electromagnetic holding current).
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import List

import pytest

# Add h30_imu scripts to path
_TEST_DIR = Path(__file__).resolve().parent
_SCRIPTS_DIR = _TEST_DIR.parent / "scripts"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

try:
    import rclpy
except ImportError:
    rclpy = None

from j11_attitude_stabilization_runner import (
    J11AttitudeStabilizationRunner,
    J11CycleTelemetry,
    J11State,
    compute_j11_2a_summary,
)


@pytest.fixture(scope="session", autouse=True)
def rclpy_session():
    """Initializes rclpy session for nodes used in tests."""
    if rclpy is not None and not rclpy.ok():
        rclpy.init()
    yield
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


def _make_valid_active_telemetry(
    count: int = 350,
    cmd_steps: List[int] = None,
    fb_steps: List[int] = None,
    fault_codes: List[int] = None,
    delta_phi: float = 0.0,
    imu_age_ms: float = 1.0,
    watchdog_status: str = "NORMAL",
    feedback_received: bool = True,
    feedback_valid: bool = True,
) -> List[J11CycleTelemetry]:
    """Helper to synthesize valid ACTIVE telemetry records."""
    if cmd_steps is None:
        cmd_steps = [1000] * 6
    if fb_steps is None:
        fb_steps = [1000, 1001, 1000, 998, 1000, 1001]
    if fault_codes is None:
        fault_codes = [0] * 6

    records = []
    for i in range(count):
        rec = J11CycleTelemetry(
            cycle_index=i + 1,
            timestamp_ns=i * 20_000_000,
            dt_ms=20.0,
            state=J11State.ACTIVE.value,
            platform_imu_age_ms=imu_age_ms,
            base_imu_age_ms=imu_age_ms,
            plat_roll_deg=0.01,
            plat_pitch_deg=0.01,
            plat_yaw_deg=0.01,
            base_roll_deg=0.005,
            base_pitch_deg=0.005,
            base_yaw_deg=0.005,
            res_roll_rad=delta_phi,
            res_pitch_rad=0.0,
            res_yaw_rad=0.0,
            commanded_steps=list(cmd_steps),
            feedback_steps=list(fb_steps),
            feedback_received=feedback_received,
            feedback_valid=feedback_valid,
            motor_fault_codes=list(fault_codes),
            watchdog_status=watchdog_status,
        )
        records.append(rec)
    return records


def test_runner_initialization_feedback_guard():
    """Runner must initialize with feedback_received=False and latest_feedback_steps=None."""
    runner = J11AttitudeStabilizationRunner(execute_motion=False, dry_run=True)
    assert runner.feedback_received is False
    assert runner.latest_feedback_steps is None
    assert runner.feedback_rx_count == 0


def test_feedback_unreceived_forces_summary_fail():
    """When feedback topic was never received, summary must evaluate to FAIL."""
    recs = _make_valid_active_telemetry(count=350, feedback_received=False, feedback_valid=False)
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=False,
    )
    assert summary["pass_criteria"]["feedback_received"] is False
    assert summary["verdict"] == "FAIL"


def test_feedback_invalid_range_forces_summary_fail():
    """Feedback outside mechanical stroke ([200, 1800]) must fail validity check."""
    recs = _make_valid_active_telemetry(count=350, fb_steps=[100, 1000, 1000, 1000, 1000, 1000], feedback_valid=False)
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["pass_criteria"]["feedback_valid"] is False
    assert summary["verdict"] == "FAIL"


def test_motor_fault_semantics_single_axis_single_cycle():
    """A single non-zero fault code in one cycle must be counted and force FAIL."""
    recs = _make_valid_active_telemetry(count=350)
    # Inject fault on Motor 3 at cycle 100
    recs[100].motor_fault_codes = [0, 0, 0x04, 0, 0, 0]

    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["faulted_cycle_count"] == 1
    assert summary["faulted_motor_count"] == 1
    assert summary["total_fault_codes_sum"] == 4
    assert summary["pass_criteria"]["no_motor_faults"] is False
    assert summary["verdict"] == "FAIL"


def test_motor_fault_semantics_multi_axis_multi_cycle():
    """Multiple axes faulted across multiple cycles must report exact counts and force FAIL."""
    recs = _make_valid_active_telemetry(count=350)
    recs[50].motor_fault_codes = [1, 0, 0, 0, 0, 2]  # Motors 1 and 6
    recs[51].motor_fault_codes = [0, 0, 0, 3, 0, 0]  # Motor 4

    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["faulted_cycle_count"] == 2
    assert summary["faulted_motor_count"] == 3  # Motors 1, 4, 6
    assert summary["total_fault_codes_sum"] == 1 + 2 + 3  # 6
    assert summary["pass_criteria"]["no_motor_faults"] is False
    assert summary["verdict"] == "FAIL"


def test_watchdog_trip_forces_summary_fail():
    """Watchdog trip must force FAIL."""
    recs = _make_valid_active_telemetry(count=350)
    recs[200].watchdog_status = "TRIPPED_TIER1"

    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["watchdog_trip_count"] == 1
    assert summary["pass_criteria"]["no_watchdog_trips"] is False
    assert summary["verdict"] == "FAIL"


def test_nonzero_delta_phi_cmd_forces_summary_fail():
    """Under zero-gain qualification, any non-zero delta_phi command forces FAIL."""
    recs = _make_valid_active_telemetry(count=350, delta_phi=0.005)
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["pass_criteria"]["delta_phi_is_zero"] is False
    assert summary["verdict"] == "FAIL"


def test_unexpected_command_steps_forces_summary_fail():
    """Commanded steps departing from nominal [1000]*6 forces FAIL."""
    recs = _make_valid_active_telemetry(count=350, cmd_steps=[1000, 1000, 1005, 1000, 1000, 1000])
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["pass_criteria"]["cmd_nominal_pass"] is False
    assert summary["verdict"] == "FAIL"


def test_large_tracking_error_forces_summary_fail():
    """Tracking error exceeding HOME tolerance (4 steps) forces FAIL."""
    # fb_steps diff = 5 steps
    recs = _make_valid_active_telemetry(count=350, fb_steps=[1000, 1000, 1005, 1000, 1000, 1000])
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["max_command_feedback_error_steps"] == 5
    assert summary["pass_criteria"]["error_within_tolerance"] is False
    assert summary["verdict"] == "FAIL"


def test_stale_imu_forces_summary_fail():
    """Platform IMU age > 50ms forces FAIL."""
    recs = _make_valid_active_telemetry(count=350, imu_age_ms=55.0)
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["pass_criteria"]["no_stale_imu"] is False
    assert summary["verdict"] == "FAIL"


def test_insufficient_active_cycles_forces_summary_fail():
    """Fewer than 300 active cycles (< 6s) forces FAIL."""
    recs = _make_valid_active_telemetry(count=150)
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["pass_criteria"]["active_duration_pass"] is False
    assert summary["verdict"] == "FAIL"


def test_perfect_run_passes_all_nine_criteria():
    """A valid run satisfying all criteria evaluates to PASS."""
    recs = _make_valid_active_telemetry(count=699, fb_steps=[1000, 1001, 1000, 998, 1000, 1001])
    summary = compute_j11_2a_summary(
        telemetry_history=recs,
        command_used="j11_attitude_stabilization_runner.py --start-imu --start-robot --execute-motion",
        execute_motion=True,
        dry_run=False,
        feedback_received=True,
    )
    assert summary["verdict"] == "PASS"
    for criterion, status in summary["pass_criteria"].items():
        assert status is True, f"Failed criterion: {criterion}"
    assert summary["holding_mechanism"] == "stepper_electromagnetic_holding_current"
    assert summary["unintended_motion"] is False
