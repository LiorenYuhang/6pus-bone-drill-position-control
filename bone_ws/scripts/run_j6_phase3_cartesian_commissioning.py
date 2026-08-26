#!/usr/bin/env python3
"""J6 Phase 3 coordinated Cartesian commissioning wrapper.

The wrapper is an upper-level commissioning state machine.  It reuses the
existing kinematics model and quintic trajectory helper, while keeping HOME,
Cartesian forward motion, observation hold, return HOME, and optional PARK as
separate explicitly authorized stages.  It is not a replacement for the
low-level motor driver and never performs an automatic return or retry.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
SRC_ROOT = WORKSPACE_ROOT / "src"
for path in (SCRIPT_ROOT, SRC_ROOT / "pus6_motor_driver", SRC_ROOT / "pus6_kinematics", SRC_ROOT / "pus6_control"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from pus6_control.joint_trajectory_controller import quintic_progress  # noqa: E402
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import (  # noqa: E402
    ABSOLUTE_RAW_LOWER_LIMIT,
    ABSOLUTE_RAW_UPPER_LIMIT,
    DEFAULT_SERIAL_PORT,
    MAX_CURRENT_MA,
    MOTOR_IDS,
    OPERATING_HOME_RAW_STEPS,
    PAUSE_ADDR,
    STORAGE_PARK_RAW_STEPS,
    TARGET_POSITION_ADDR,
    HardwareMode,
    SafetyViolation,
    TargetValidationError,
)
from pus6_motor_driver.six_motor_controller import SixAxisStateSnapshot, SixMotorController  # noqa: E402


HOME_RAW_STEPS = list(OPERATING_HOME_RAW_STEPS)
PARK_RAW_STEPS = list(STORAGE_PARK_RAW_STEPS)
COMMISSIONING_SPEED_STEP_S = 50.0
DEFAULT_CONTROL_RATE_HZ = 10.0
DEFAULT_HOLD_SAMPLES = 2
DEFAULT_HOLD_INTERVAL_S = 0.05
DEFAULT_MAX_INCREMENT_STEPS = 40
DEFAULT_HOME_LOCAL_LIMITS = {mid: (900, 1100) for mid in MOTOR_IDS}
RETURN_TOLERANCE_STEPS = 2

# Pose order is [x, y, z, roll, pitch, yaw].  Yaw is deliberately absent.
DOF_OFFSETS: dict[str, tuple[int, float, str]] = {
    "z_pos": (2, 0.0005, "Z+"),
    "z_neg": (2, -0.0005, "Z-"),
    "x_pos": (0, 0.0005, "X+"),
    "x_neg": (0, -0.0005, "X-"),
    "y_pos": (1, 0.0005, "Y+"),
    "y_neg": (1, -0.0005, "Y-"),
    "roll_pos": (3, 0.004363323129985824, "Roll+"),
    "roll_neg": (3, -0.004363323129985824, "Roll-"),
    "pitch_pos": (4, 0.004363323129985824, "Pitch+"),
    "pitch_neg": (4, -0.004363323129985824, "Pitch-"),
}
ALLOWED_TESTS = tuple(DOF_OFFSETS)

OBSERVATION_DEFAULTS = {
    "platform_motion": "unrecorded",
    "ball_joint_condition": "unrecorded",
    "interference": "unrecorded",
    "abnormal_sound": "unrecorded",
    "observer_notes": None,
}
OBSERVATION_VALUES = {
    "platform_motion": {"unrecorded", "expected", "unexpected"},
    "ball_joint_condition": {"unrecorded", "clear", "abnormal"},
    "interference": {"unrecorded", "none", "present"},
    "abnormal_sound": {"unrecorded", "none", "present"},
}


class CommissioningState(str, Enum):
    CREATED = "CREATED"
    BASELINE_RECORDED = "BASELINE_RECORDED"
    HOME_REACHED = "HOME_REACHED"
    HOME_CONFIRMED = "HOME_CONFIRMED"
    PREVIEWED = "PREVIEWED"
    FORWARD_COMPLETE = "FORWARD_COMPLETE"
    OBSERVATION_HOLD = "OBSERVATION_HOLD"
    RETURN_HOME_COMPLETE = "RETURN_HOME_COMPLETE"
    PARK_COMPLETE = "PARK_COMPLETE"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


def _normalise_observations(observations: Mapping[str, Any] | None) -> dict[str, Any]:
    values = dict(OBSERVATION_DEFAULTS)
    if observations is None:
        return values
    for key, value in observations.items():
        if key not in values:
            raise TargetValidationError(f"unknown operator observation field: {key}")
        if key in OBSERVATION_VALUES and value not in OBSERVATION_VALUES[key]:
            allowed = ", ".join(sorted(OBSERVATION_VALUES[key]))
            raise TargetValidationError(f"{key} must be one of: {allowed}")
        values[key] = value
    return values


def _observations_complete(observations: Mapping[str, Any]) -> bool:
    return all(observations[key] != "unrecorded" for key in OBSERVATION_VALUES)


def _observations_safe(observations: Mapping[str, Any]) -> bool:
    return (
        observations.get("platform_motion") == "expected"
        and observations.get("ball_joint_condition") == "clear"
        and observations.get("interference") == "none"
        and observations.get("abnormal_sound") == "none"
    )


def _snapshot_dict(snapshot: SixAxisStateSnapshot) -> dict[str, Any]:
    return {
        "timestamp_start_ns": int(snapshot.timestamp_start_ns),
        "timestamp_end_ns": int(snapshot.timestamp_end_ns),
        "per_axis_timestamps_ns": {str(mid): int(ts) for mid, ts in snapshot.per_axis_timestamps_ns.items()},
        "motor_ids": list(MOTOR_IDS),
        "target_positions_steps": [int(status.target_position) for status in snapshot.states],
        "actual_positions_steps": [int(value) for value in snapshot.positions_steps],
        "positions_meters": [float(value) for value in snapshot.positions_meters],
        "currents_mA": [int(value) for value in snapshot.currents_mA],
        "temperatures_C": [int(value) for value in snapshot.temperatures_C],
        "fault_codes": [int(value) for value in snapshot.fault_codes],
    }


def _write_records(controller: SixMotorController) -> list[dict[str, Any]]:
    bus = controller.bus
    raw_audit = getattr(bus, "write_audit", None)
    if raw_audit is not None:
        return [dict(record) for record in raw_audit]
    return []


def _serialise_fk(result: Mapping[str, Any] | None) -> dict[str, Any] | None:
    return dict(result) if result is not None else None


def _evidence_fields(evidence: Mapping[str, Any] | None) -> dict[str, Any]:
    """Keep raw H30/camera evidence opaque and preserve required field names."""
    fields: dict[str, Any] = {
        "imu_base": None,
        "imu_platform": None,
        "relative_attitude": None,
        "camera": {
            "enabled": False,
            "snapshot_timestamp_monotonic_ns": None,
            "camera_id": None,
            "frame_id": None,
            "path": None,
        },
    }
    if evidence is None:
        return fields
    for key in ("imu_base", "imu_platform", "relative_attitude", "camera"):
        if key in evidence:
            fields[key] = evidence[key]
    return fields


class CartesianCommissioningSession:
    """Explicit PARK -> HOME -> Cartesian -> hold -> authorized return state machine."""

    def __init__(
        self,
        controller: SixMotorController,
        *,
        test_name: str,
        kinematics: Analytical6PusKinematics | None = None,
        commissioning_limits: Mapping[int, tuple[int, int]] | None = None,
        max_increment_steps: int = DEFAULT_MAX_INCREMENT_STEPS,
        speed_step_s: float = COMMISSIONING_SPEED_STEP_S,
        control_rate_hz: float = DEFAULT_CONTROL_RATE_HZ,
        sleep_fn: Callable[[float], None] = time.sleep,
        session_id: str | None = None,
        motion_authorized: bool = False,
    ) -> None:
        if test_name not in DOF_OFFSETS:
            raise TargetValidationError(f"unsupported test {test_name!r}; allowed: {list(ALLOWED_TESTS)}")
        if controller.hardware_mode == HardwareMode.READ_ONLY:
            raise TargetValidationError("Cartesian commissioning requires MOCK or authorized ACTIVE mode")
        if controller.hardware_mode == HardwareMode.ACTIVE and not motion_authorized:
            raise TargetValidationError("ACTIVE commissioning requires explicit motion authorization")
        if getattr(controller, "retries", 0) != 0:
            raise TargetValidationError("commissioning requires controller retries=0; retries are forbidden")
        if speed_step_s <= 0 or speed_step_s > COMMISSIONING_SPEED_STEP_S:
            raise TargetValidationError(f"speed_step_s must be in (0,{COMMISSIONING_SPEED_STEP_S:g}]")
        if control_rate_hz <= 0 or max_increment_steps < 1:
            raise TargetValidationError("control_rate_hz and max_increment_steps must be positive")

        self.controller = controller
        self.kinematics = kinematics or Analytical6PusKinematics()
        self.test_name = test_name
        self.speed_step_s = float(speed_step_s)
        self.control_rate_hz = float(control_rate_hz)
        self.sleep_fn = sleep_fn
        self.max_increment_steps = int(max_increment_steps)
        self.home_pose = [0.0, 0.0, self.kinematics.nominal_height, 0.0, 0.0, 0.0]
        self.home_sliders_m = self.kinematics.inverse_kinematics(self.home_pose)
        self.commissioning_limits = {
            mid: tuple(commissioning_limits[mid]) if commissioning_limits and mid in commissioning_limits
            else DEFAULT_HOME_LOCAL_LIMITS[mid]
            for mid in MOTOR_IDS
        }
        for mid, bounds in self.commissioning_limits.items():
            if len(bounds) != 2 or bounds[0] > bounds[1]:
                raise TargetValidationError(f"invalid commissioning limits for ID{mid}: {bounds}")

        self.state = CommissioningState.CREATED
        self.failures: list[str] = []
        self.limits: dict[int, tuple[int, int]] = {}
        self.baseline_snapshot: SixAxisStateSnapshot | None = None
        self.pending_target_steps: list[int] | None = None
        self.pending_target_pose: list[float] | None = None
        self.observations = _normalise_observations(None)
        self._expected_target_vectors = 0
        self._stop_requested = False
        self._return_authorized = False
        self._home_authorized = False
        self._park_authorized = False
        self._snapshots_logged = 0
        self.session_id = session_id
        self.motion_authorized = bool(motion_authorized)
        self.report: dict[str, Any] = {
            "schema_version": 1,
            "tool": "j6_phase3_cartesian_commissioning",
            "phase": "J6_PHASE3",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "session_id": session_id,
            "hardware_mode": controller.hardware_mode.value,
            "port": controller.port,
            "test_name": test_name,
            "allowed_tests": list(ALLOWED_TESTS),
            "yaw_allowed": False,
            "home_target_steps": list(HOME_RAW_STEPS),
            "park_target_steps": list(PARK_RAW_STEPS),
            "speed_step_s": self.speed_step_s,
            "control_rate_hz": self.control_rate_hz,
            "commissioning_limits_steps": {str(mid): list(bounds) for mid, bounds in self.commissioning_limits.items()},
            "max_increment_steps": self.max_increment_steps,
            "hardware_motion_executed": False,
            "hardware_write_count": 0,
            "mock_write_count": 0,
            "home_executed": False,
            "park_executed": False,
            "automatic_return": False,
            "stages": {
                "baseline_static": None,
                "home_transition": None,
                "home_static_confirmation": None,
                "cartesian_preview": None,
                "forward_perturbation": None,
                "observation_hold": None,
                "return_home_authorized": None,
                "park_authorized": None,
            },
            "failure_reasons": [],
            "evidence_policy": {
                "six_axis_feedback": True,
                "fk_result": True,
                "dual_h30_fields": True,
                "camera_timestamp_field": True,
                "operator_observation": True,
            },
        }

    def _set_state(self, state: CommissioningState) -> None:
        self.state = state
        self.report["state"] = state.value

    def _fail(self, message: str, *, stopped: bool = False) -> None:
        self.failures.append(message)
        self._set_state(CommissioningState.STOPPED if stopped else CommissioningState.FAILED)

    def _health_failures(self, snapshot: SixAxisStateSnapshot, phase: str) -> list[str]:
        failures: list[str] = []
        if len(snapshot.states) != len(MOTOR_IDS):
            return [f"{phase}: expected six-axis feedback, got {len(snapshot.states)}"]
        for index, status in enumerate(snapshot.states, start=1):
            if status.error_code != 0:
                failures.append(f"{phase}: ID{index} fault=0x{status.error_code:02X}")
            if status.current_mA > MAX_CURRENT_MA:
                failures.append(f"{phase}: ID{index} current={status.current_mA}mA exceeds {MAX_CURRENT_MA}mA")
        return failures

    def _hardware_target_failures(self, targets: Sequence[int], phase: str) -> list[str]:
        failures: list[str] = []
        for index, target in enumerate(targets, start=1):
            lower, upper = self.limits.get(index, (ABSOLUTE_RAW_LOWER_LIMIT, ABSOLUTE_RAW_UPPER_LIMIT))
            if not lower <= target <= upper:
                failures.append(f"{phase}: ID{index} target={target} outside hardware [{lower},{upper}]")
        return failures

    def _commissioning_target_failures(self, targets: Sequence[int], phase: str) -> list[str]:
        return [
            f"{phase}: ID{index} target={target} outside commissioning [{bounds[0]},{bounds[1]}]"
            for index, target in enumerate(targets, start=1)
            for bounds in [self.commissioning_limits[index]]
            if not bounds[0] <= target <= bounds[1]
        ]

    def _fk_from_raw(self, raw_steps: Sequence[int], *, initial_guess: Sequence[float] | None = None) -> dict[str, Any]:
        sliders_m = [
            float(home_m + (int(raw) - int(HOME_RAW_STEPS[index])) / self.kinematics.steps_per_meter)
            for index, (home_m, raw) in enumerate(zip(self.home_sliders_m, raw_steps))
        ]
        try:
            result = self.kinematics.forward_kinematics_with_metrics(
                sliders_m,
                initial_guess=initial_guess or self.home_pose,
            )
            return {
                "available": True,
                "input_slider_positions_m": sliders_m,
                "pose": list(result.pose),
                "rotation_matrix": result.rotation_matrix,
                "converged": bool(result.converged),
                "iterations": int(result.iterations),
                "residual_norm_m": float(result.residual_norm),
            }
        except Exception as exc:  # FK is evidence; motion gates still use IK and target checks.
            return {
                "available": False,
                "input_slider_positions_m": sliders_m,
                "error": f"{type(exc).__name__}: {exc}",
            }

    def _feedback(self, snapshot: SixAxisStateSnapshot, *, fk: Mapping[str, Any] | None = None) -> dict[str, Any]:
        record = _snapshot_dict(snapshot)
        record["fk"] = _serialise_fk(fk)
        self._snapshots_logged += 1
        return record

    def _stop(self) -> None:
        self._stop_requested = True
        try:
            self.controller.pause_all()
        except Exception as exc:
            self.failures.append(f"pause_all failed during safety stop: {exc}")

    def _read_snapshot_or_stop(self, phase: str) -> SixAxisStateSnapshot:
        """Convert driver feedback validation errors into a latched STOP."""
        try:
            return self.controller.read_state_snapshot()
        except Exception as exc:
            message = f"{phase}: feedback read/validation failed: {type(exc).__name__}: {exc}"
            self.failures.append(message)
            self._stop()
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation(message) from exc

    def _execute_segment(
        self,
        *,
        start_steps: Sequence[int],
        target_steps: Sequence[int],
        stage: str,
        duration_s: float | None,
        commissioning_check: bool,
    ) -> dict[str, Any]:
        max_disp = max(abs(int(t) - int(s)) for s, t in zip(start_steps, target_steps))
        duration = float(duration_s) if duration_s is not None else max(0.2, 1.875 * max_disp / self.speed_step_s)
        duration = max(0.01, duration)
        sample_count = max(2, int(math.ceil(duration * self.control_rate_hz)) + 1)
        trajectory: list[dict[str, Any]] = []
        segment_failures: list[str] = []
        for sample_index in range(sample_count):
            tau = sample_index / float(sample_count - 1)
            progress = quintic_progress(tau)
            command = [
                int(round(int(start) + progress * (int(target) - int(start))))
                for start, target in zip(start_steps, target_steps)
            ]
            if command != list(start_steps) or sample_index == sample_count - 1:
                if command != getattr(self, "_last_command", None):
                    try:
                        self.controller.command_positions(command)
                    except Exception as exc:
                        message = f"{stage}: target dispatch failed: {type(exc).__name__}: {exc}"
                        segment_failures.append(message)
                        self.failures.append(message)
                        self._stop()
                        self._set_state(CommissioningState.STOPPED)
                        break
                    self._expected_target_vectors += 1
                    self._last_command = list(command)
            if commissioning_check:
                segment_failures.extend(self._commissioning_target_failures(command, stage))
            segment_failures.extend(self._hardware_target_failures(command, stage))
            try:
                snapshot = self._read_snapshot_or_stop(stage)
            except SafetyViolation as exc:
                segment_failures.append(str(exc))
                break
            health = self._health_failures(snapshot, stage)
            segment_failures.extend(health)
            trajectory.append({
                "sample_index": sample_index,
                "tau": tau,
                "target_steps": list(command),
                "feedback": self._feedback(snapshot, fk=self._fk_from_raw(snapshot.positions_steps)),
            })
            if segment_failures:
                self.failures.extend(segment_failures)
                if not self._stop_requested:
                    self._stop()
                self._set_state(CommissioningState.STOPPED)
                break
            if sample_index + 1 < sample_count:
                self.sleep_fn(1.0 / self.control_rate_hz)
        return {
            "start_steps": list(start_steps),
            "target_steps": list(target_steps),
            "duration_s": duration,
            "planned_peak_speed_step_s": 1.875 * max_disp / duration,
            "sample_count": len(trajectory),
            "trajectory": trajectory,
            "safety_pass": not segment_failures,
        }

    def record_baseline(self, *, evidence: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if self.state != CommissioningState.CREATED:
            raise SafetyViolation("baseline must be the first commissioning stage")
        self.controller.connect()
        self.limits = self.controller.read_limits()
        try:
            snapshot = self.controller.read_state_snapshot()
        except Exception as exc:
            message = f"baseline: feedback read/validation failed: {type(exc).__name__}: {exc}"
            self.failures.append(message)
            self._stop()
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation(message) from exc
        failures = self._health_failures(snapshot, "baseline")
        failures.extend(self._hardware_target_failures(snapshot.positions_steps, "baseline actual"))
        if self.controller.write_count != 0:
            failures.append("baseline started with non-zero write_count")
        self.baseline_snapshot = snapshot
        self.report["stages"]["baseline_static"] = {
            "feedback": self._feedback(snapshot, fk=self._fk_from_raw(snapshot.positions_steps)),
            "hardware_limits_steps": {str(mid): list(bounds) for mid, bounds in self.limits.items()},
            "evidence": _evidence_fields(evidence),
            "write_count_at_stage": int(self.controller.write_count),
            "health_pass": not failures,
        }
        if failures:
            self.failures.extend(failures)
            self._set_state(CommissioningState.FAILED)
            raise SafetyViolation("baseline safety gate failed")
        self._set_state(CommissioningState.BASELINE_RECORDED)
        return self.report["stages"]["baseline_static"]

    def home_transition(self, *, home_authorized: bool = False, duration_s: float | None = None) -> dict[str, Any]:
        if self.state != CommissioningState.BASELINE_RECORDED:
            raise SafetyViolation("HOME transition requires a recorded baseline")
        if not home_authorized:
            raise SafetyViolation("HOME transition requires explicit authorization")
        assert self.baseline_snapshot is not None
        self._home_authorized = True
        self.report["home_executed"] = True
        self._last_command = None
        stage = self._execute_segment(
            start_steps=self.baseline_snapshot.positions_steps,
            target_steps=HOME_RAW_STEPS,
            stage="home_transition",
            duration_s=duration_s,
            commissioning_check=False,
        )
        stage["authorized"] = True
        stage["evidence"] = _evidence_fields(None)
        self.report["stages"]["home_transition"] = stage
        if not stage["safety_pass"]:
            raise SafetyViolation("HOME transition safety gate failed")
        self._set_state(CommissioningState.HOME_REACHED)
        return stage

    def confirm_home(
        self,
        *,
        observations: Mapping[str, Any],
        evidence: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self.state != CommissioningState.HOME_REACHED:
            raise SafetyViolation("HOME confirmation requires a completed HOME transition")
        values = _normalise_observations(observations)
        snapshot = self._read_snapshot_or_stop("HOME confirmation")
        failures = self._health_failures(snapshot, "HOME confirmation")
        errors = [abs(int(actual) - int(target)) for actual, target in zip(snapshot.positions_steps, HOME_RAW_STEPS)]
        if max(errors, default=0) > RETURN_TOLERANCE_STEPS:
            failures.append(f"HOME residual exceeds {RETURN_TOLERANCE_STEPS} steps: {errors}")
        safe = _observations_complete(values) and _observations_safe(values)
        if not safe:
            failures.append("HOME operator observations are incomplete or unsafe")
        stage = {
            "feedback": self._feedback(snapshot, fk=self._fk_from_raw(snapshot.positions_steps, initial_guess=self.home_pose)),
            "residual_steps": errors,
            "operator_observations": values,
            "evidence": _evidence_fields(evidence),
            "pass": not failures,
            "write_count_at_stage": int(self.controller.write_count),
        }
        self.report["stages"]["home_static_confirmation"] = stage
        self.observations = values
        if failures:
            self.failures.extend(failures)
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation("HOME static confirmation failed")
        self._set_state(CommissioningState.HOME_CONFIRMED)
        return stage

    def preview_cartesian(self) -> dict[str, Any]:
        if self.state != CommissioningState.HOME_CONFIRMED:
            raise SafetyViolation("Cartesian preview requires HOME static confirmation")
        index, delta, label = DOF_OFFSETS[self.test_name]
        target_pose = list(self.home_pose)
        target_pose[index] += delta
        try:
            ik_sliders_m = self.kinematics.inverse_kinematics(target_pose)
            delta_steps = self.kinematics.differential_inverse_kinematics_steps(target_pose, self.home_pose)
        except Exception as exc:
            self._fail(f"IK preview failed: {type(exc).__name__}: {exc}")
            raise SafetyViolation(self.failures[-1]) from exc
        target_steps = [int(home + delta_step) for home, delta_step in zip(HOME_RAW_STEPS, delta_steps)]
        failures = self._hardware_target_failures(target_steps, "Cartesian preview")
        failures.extend(self._commissioning_target_failures(target_steps, "Cartesian preview"))
        failures.extend(
            f"Cartesian preview: ID{mid} increment={abs(int(delta_step))} exceeds {self.max_increment_steps} steps"
            for mid, delta_step in zip(MOTOR_IDS, delta_steps)
            if abs(int(delta_step)) > self.max_increment_steps
        )
        fk = self._fk_from_raw(target_steps, initial_guess=target_pose)
        if not fk.get("available", False):
            failures.append("Cartesian preview FK did not converge")
        stage = {
            "test_name": self.test_name,
            "label": label,
            "target_pose_cartesian": target_pose,
            "ik_output_slider_positions_m": [float(value) for value in ik_sliders_m],
            "ik_delta_steps": [int(value) for value in delta_steps],
            "target_steps": target_steps,
            "hardware_limits_pass": not self._hardware_target_failures(target_steps, "hardware"),
            "commissioning_limits_pass": not self._commissioning_target_failures(target_steps, "commissioning"),
            "fk": fk,
            "pass": not failures,
            "failure_reasons": failures,
        }
        self.report["stages"]["cartesian_preview"] = stage
        if failures:
            self.failures.extend(failures)
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation("Cartesian target preview failed")
        self.pending_target_steps = target_steps
        self.pending_target_pose = target_pose
        self._set_state(CommissioningState.PREVIEWED)
        return stage

    def forward_motion(self, *, forward_authorized: bool = False, duration_s: float | None = None) -> dict[str, Any]:
        if self.state != CommissioningState.PREVIEWED or self.pending_target_steps is None:
            raise SafetyViolation("forward motion requires a successful Cartesian preview")
        if not forward_authorized:
            raise SafetyViolation("forward Cartesian motion requires explicit authorization")
        snapshot = self._read_snapshot_or_stop("forward preflight")
        health = self._health_failures(snapshot, "forward preflight")
        home_errors = [abs(int(actual) - int(target)) for actual, target in zip(snapshot.positions_steps, HOME_RAW_STEPS)]
        if max(home_errors, default=0) > RETURN_TOLERANCE_STEPS:
            health.append(f"forward preflight is not at HOME: {home_errors}")
        if health:
            self.failures.extend(health)
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation("forward preflight failed")
        self._last_command = None
        stage = self._execute_segment(
            start_steps=snapshot.positions_steps,
            target_steps=self.pending_target_steps,
            stage="forward_perturbation",
            duration_s=duration_s,
            commissioning_check=True,
        )
        stage.update({
            "authorized": True,
            "target_pose_cartesian": list(self.pending_target_pose or []),
            "ik_output_target_steps": list(self.pending_target_steps),
            "fk_target": self._fk_from_raw(self.pending_target_steps, initial_guess=self.pending_target_pose),
            "automatic_return": False,
        })
        self.report["stages"]["forward_perturbation"] = stage
        if not stage["safety_pass"]:
            raise SafetyViolation("forward Cartesian motion safety gate failed")
        self._set_state(CommissioningState.FORWARD_COMPLETE)
        return stage

    def observation_hold(
        self,
        *,
        observations: Mapping[str, Any],
        evidence: Mapping[str, Any] | None = None,
        hold_samples: int = DEFAULT_HOLD_SAMPLES,
        hold_interval_s: float = DEFAULT_HOLD_INTERVAL_S,
    ) -> dict[str, Any]:
        if self.state != CommissioningState.FORWARD_COMPLETE:
            raise SafetyViolation("observation hold requires completed forward motion")
        if hold_samples < 1 or hold_interval_s < 0:
            raise TargetValidationError("hold_samples must be >=1 and hold_interval_s >=0")
        values = _normalise_observations(observations)
        samples: list[dict[str, Any]] = []
        failures: list[str] = []
        for index in range(hold_samples):
            try:
                snapshot = self._read_snapshot_or_stop(f"hold[{index}]")
            except SafetyViolation as exc:
                failures.append(str(exc))
                break
            failures.extend(self._health_failures(snapshot, f"hold[{index}]"))
            samples.append({
                "sample_index": index,
                "feedback": self._feedback(snapshot, fk=self._fk_from_raw(snapshot.positions_steps, initial_guess=self.pending_target_pose)),
            })
            if index + 1 < hold_samples:
                self.sleep_fn(hold_interval_s)
        complete = _observations_complete(values)
        safe = _observations_safe(values)
        stage = {
            "samples": samples,
            "sample_count": len(samples),
            "operator_observations": values,
            "evidence": _evidence_fields(evidence),
            "operator_observations_complete": complete,
            "operator_observations_safe": safe,
            "automatic_return": False,
            "health_pass": not failures,
            "write_count_at_stage": int(self.controller.write_count),
        }
        self.report["stages"]["observation_hold"] = stage
        self.observations = values
        self.failures.extend(failures)
        if failures or (complete and not safe):
            if not self._stop_requested:
                self._stop()
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation("observation hold safety gate failed")
        self._set_state(CommissioningState.OBSERVATION_HOLD)
        return stage

    def authorize_return_home(
        self,
        *,
        return_authorized: bool = False,
        duration_s: float | None = None,
        evidence: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if self.state != CommissioningState.OBSERVATION_HOLD:
            raise SafetyViolation("return HOME requires completed observation hold")
        if not return_authorized:
            raise SafetyViolation("return HOME requires explicit authorization")
        if not _observations_complete(self.observations) or not _observations_safe(self.observations):
            raise SafetyViolation("return HOME requires complete and safe operator observations")
        snapshot = self._read_snapshot_or_stop("return HOME preflight")
        self._last_command = None
        stage = self._execute_segment(
            start_steps=snapshot.positions_steps,
            target_steps=HOME_RAW_STEPS,
            stage="return_home_authorized",
            duration_s=duration_s,
            commissioning_check=False,
        )
        final_snapshot = self._read_snapshot_or_stop("return HOME final")
        residual = [abs(int(actual) - int(target)) for actual, target in zip(final_snapshot.positions_steps, HOME_RAW_STEPS)]
        stage.update({
            "authorized": True,
            "feedback_final": self._feedback(final_snapshot, fk=self._fk_from_raw(final_snapshot.positions_steps, initial_guess=self.home_pose)),
            "residual_steps": residual,
            "return_pass": stage["safety_pass"] and max(residual, default=0) <= RETURN_TOLERANCE_STEPS,
            "evidence": _evidence_fields(evidence),
            "automatic_return": False,
        })
        self.report["stages"]["return_home_authorized"] = stage
        self._return_authorized = True
        if not stage["return_pass"]:
            self.failures.append(f"return HOME residual exceeds tolerance: {residual}")
            if not self._stop_requested:
                self._stop()
            self._set_state(CommissioningState.STOPPED)
            raise SafetyViolation("return HOME safety gate failed")
        self._set_state(CommissioningState.RETURN_HOME_COMPLETE)
        return stage

    def authorize_park(self, *, park_authorized: bool = False, duration_s: float | None = None) -> dict[str, Any]:
        if self.state != CommissioningState.RETURN_HOME_COMPLETE:
            raise SafetyViolation("PARK requires completed authorized return HOME")
        if not park_authorized:
            raise SafetyViolation("PARK requires explicit authorization")
        self._park_authorized = True
        self._last_command = None
        snapshot = self._read_snapshot_or_stop("PARK preflight")
        stage = self._execute_segment(
            start_steps=snapshot.positions_steps,
            target_steps=PARK_RAW_STEPS,
            stage="park_authorized",
            duration_s=duration_s,
            commissioning_check=False,
        )
        stage["authorized"] = True
        self.report["stages"]["park_authorized"] = stage
        self.report["park_executed"] = True
        if not stage["safety_pass"]:
            raise SafetyViolation("PARK safety gate failed")
        self._set_state(CommissioningState.PARK_COMPLETE)
        return stage

    def finalise(self) -> dict[str, Any]:
        records = _write_records(self.controller)
        target_records = [record for record in records if int(record.get("register_address", -1)) == TARGET_POSITION_ADDR]
        unexpected = [
            record for record in records
            if int(record.get("register_address", -1)) != TARGET_POSITION_ADDR
            and not (int(record.get("register_address", -1)) == PAUSE_ADDR and self._stop_requested)
        ]
        if len(target_records) != self._expected_target_vectors * len(MOTOR_IDS):
            self.failures.append(
                f"target write count {len(target_records)} != expected {self._expected_target_vectors * len(MOTOR_IDS)}"
            )
        bus_telemetry = self.controller.bus.get_telemetry() if hasattr(self.controller.bus, "get_telemetry") else {}
        no_retry = getattr(self.controller, "retries", 0) == 0 and int(bus_telemetry.get("retry_count", 0)) == 0
        h30_present = bool(
            (self.report["stages"]["observation_hold"] or {}).get("evidence", {}).get("imu_base")
            is not None
            and (self.report["stages"]["observation_hold"] or {}).get("evidence", {}).get("imu_platform") is not None
        )
        required = {
            "home_completed": self.state in {CommissioningState.RETURN_HOME_COMPLETE, CommissioningState.PARK_COMPLETE},
            "home_static_confirmed": self.report["stages"]["home_static_confirmation"] is not None,
            "preview_completed": self.report["stages"]["cartesian_preview"] is not None,
            "forward_completed": self.report["stages"]["forward_perturbation"] is not None,
            "observation_hold_recorded": self.report["stages"]["observation_hold"] is not None,
            "return_explicitly_authorized": self._return_authorized,
            "no_automatic_return": not self.report["automatic_return"],
            "no_automatic_retry": no_retry,
            "no_unexpected_writes": not unexpected,
            "full_six_axis_feedback": self._snapshots_logged >= 5,
            "dual_h30_data_recorded": h30_present,
        }
        successful = all(required.values()) and not self.failures
        if self.state == CommissioningState.PARK_COMPLETE and successful:
            outcome = "PASS"
        elif self.state == CommissioningState.RETURN_HOME_COMPLETE and successful:
            outcome = "PASS"
        elif self.state in {CommissioningState.BASELINE_RECORDED, CommissioningState.PREVIEWED, CommissioningState.OBSERVATION_HOLD, CommissioningState.FORWARD_COMPLETE, CommissioningState.HOME_CONFIRMED, CommissioningState.HOME_REACHED}:
            outcome = "HOLD"
        elif self.state == CommissioningState.STOPPED:
            outcome = "STOP"
        else:
            outcome = "FAIL"
        self.report.update({
            "state": self.state.value,
            "outcome": outcome,
            "pass": successful,
            "hardware_motion_executed": self.controller.hardware_mode == HardwareMode.ACTIVE and self._expected_target_vectors > 0,
            "hardware_write_count": int(self.controller.write_count) if self.controller.hardware_mode == HardwareMode.ACTIVE else 0,
            "mock_write_count": int(self.controller.write_count) if self.controller.hardware_mode == HardwareMode.MOCK else 0,
            "write_audit": records,
            "unexpected_writes": unexpected,
            "pass_conditions": required,
            "failure_reasons": list(dict.fromkeys(self.failures)),
            "write_audit_summary": {
                "expected_target_vectors": self._expected_target_vectors,
                "target_write_count": len(target_records),
                "unexpected_write_count": len(unexpected),
                "retry_count": int(bus_telemetry.get("retry_count", 0)),
            },
        })
        return self.report

    def finalize(self) -> dict[str, Any]:
        """US spelling alias for callers using finalize()."""
        return self.finalise()


Phase3CartesianCommissioningSession = CartesianCommissioningSession


def make_mock_controller(*, start_steps: Sequence[int] | None = None) -> SixMotorController:
    bus = FakeBus()
    if start_steps is not None:
        if len(start_steps) != len(MOTOR_IDS):
            raise ValueError("start_steps must contain six values")
        bus.positions = {mid: int(start_steps[index]) for index, mid in enumerate(MOTOR_IDS)}
        bus.targets = dict(bus.positions)
    bus.limits = {mid: (ABSOLUTE_RAW_LOWER_LIMIT, ABSOLUTE_RAW_UPPER_LIMIT) for mid in MOTOR_IDS}
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK, retries=0)
    controller.motion_authorized = True  # Explicitly local to this Mock controller only.
    return controller


def run_j6_phase3_cartesian_commissioning(
    *,
    test_name: str,
    home_authorized: bool = False,
    home_confirmed: bool = False,
    forward_authorized: bool = False,
    return_home_authorized: bool = False,
    park_authorized: bool = False,
    observations: Mapping[str, Any] | None = None,
    evidence: Mapping[str, Any] | None = None,
    controller: SixMotorController | None = None,
    duration_s: float | None = None,
    hold_samples: int = DEFAULT_HOLD_SAMPLES,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """Convenience runner; missing authorizations deliberately leave HOLD."""
    active_controller = controller or make_mock_controller(start_steps=PARK_RAW_STEPS)
    session = CartesianCommissioningSession(
        active_controller,
        test_name=test_name,
        sleep_fn=sleep_fn,
        motion_authorized=(active_controller.hardware_mode != HardwareMode.ACTIVE
                           or bool(getattr(active_controller, "motion_authorized", False))),
    )
    try:
        session.record_baseline(evidence=evidence)
        if home_authorized:
            session.home_transition(home_authorized=True, duration_s=duration_s)
        if home_confirmed:
            session.confirm_home(observations=observations or {}, evidence=evidence)
        if session.state == CommissioningState.HOME_CONFIRMED:
            session.preview_cartesian()
        if forward_authorized:
            session.forward_motion(forward_authorized=True, duration_s=duration_s)
        if session.state == CommissioningState.FORWARD_COMPLETE and observations is not None:
            session.observation_hold(observations=observations, evidence=evidence, hold_samples=hold_samples)
        if return_home_authorized:
            session.authorize_return_home(return_authorized=True, duration_s=duration_s, evidence=evidence)
        if park_authorized:
            session.authorize_park(park_authorized=True, duration_s=duration_s)
    except (SafetyViolation, TargetValidationError) as exc:
        if not session.failures:
            session.failures.append(str(exc))
    return session.finalise()



# Stable functional alias matching the other J6 commissioning wrappers.
run_phase3_cartesian_commissioning = run_j6_phase3_cartesian_commissioning


def _write_report(report: Mapping[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {output}")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J6 Phase 3 coordinated Cartesian commissioning")
    parser.add_argument("--test", choices=ALLOWED_TESTS, required=True)
    parser.add_argument("--authorize-home", action="store_true")
    parser.add_argument("--confirm-home", action="store_true")
    parser.add_argument("--authorize-forward", action="store_true")
    parser.add_argument("--authorize-return-home", action="store_true")
    parser.add_argument("--authorize-park", action="store_true")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--hold-samples", type=int, default=DEFAULT_HOLD_SAMPLES)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.real_hardware and not (args.confirm_motion and args.validated_config):
        raise SystemExit("real hardware requires --confirm-motion and --validated-config")
    if args.real_hardware:
        controller = SixMotorController(
            hardware_mode=HardwareMode.ACTIVE,
            port=args.port,
            retries=0,
        )
        controller.motion_authorized = True
    else:
        controller = make_mock_controller(start_steps=PARK_RAW_STEPS)
    report = run_j6_phase3_cartesian_commissioning(
        test_name=args.test,
        home_authorized=args.authorize_home,
        home_confirmed=args.confirm_home,
        forward_authorized=args.authorize_forward,
        return_home_authorized=args.authorize_return_home,
        park_authorized=args.authorize_park,
        controller=controller,
        duration_s=args.duration,
        hold_samples=args.hold_samples,
    )
    output = args.output or WORKSPACE_ROOT / "log" / f"j6_phase3_cartesian_{args.test}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    _write_report(report, output)
    print(json.dumps({"pass": report["pass"], "outcome": report["outcome"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
