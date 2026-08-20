#!/usr/bin/env python3
"""Fail-closed J2-C six-axis vector & multi-cycle cyclic motion runner.

The default mode is Mock. A real run requires explicit stage gates and separate
human authorization. All six references and targets are admitted before the first
configuration write; configuration, target, and pause dispatch are sequential and
never retried.

Supports:
- Single vector step motion (cycles=1, return_to_ref=False, 18 writes);
- Multi-cycle round-trip motion (cycles=5, return_to_ref=True, 72 writes for 5 cycles).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
if str(DRIVER_ROOT) not in sys.path:
    sys.path.insert(0, str(DRIVER_ROOT))

from pus6_motor_driver.safety import (  # noqa: E402
    DEFAULT_SPEED,
    HardwareMode,
    MOTOR_IDS,
    PAUSE_ADDR,
    SPEED_MODE,
    STATIC_POSITION_TOLERANCE_STEPS,
    SafetyViolation,
    TargetValidationError,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


DELTA_STEPS: tuple[int, ...] = (10, 10, 10, 10, 10, 10)
DEFAULT_CYCLES = 5
TIMEOUT_PER_PHASE_SEC = 5.0
POLL_PERIOD_SEC = 0.05
SETTLE_SAMPLES = 2
CONFIRMED_MOTION_CONSECUTIVE_SAMPLES = 2
BASELINE_SAMPLE_COUNT = 10
BASELINE_SAMPLE_PERIOD_SEC = 0.005


class J2CStop(SafetyViolation):
    """Controlled J2-C safety stop; the runner preserves the audit and does not retry."""


def _json_value(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _json_value(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if hasattr(value, "value") and not isinstance(value, (str, bytes)):
        return value.value
    return value


def _calc_stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "samples": []}
    mean_val = sum(values) / len(values)
    if len(values) > 1:
        var = sum((x - mean_val) ** 2 for x in values) / (len(values) - 1)
        std_val = math.sqrt(var)
    else:
        std_val = 0.0
    return {
        "mean": round(mean_val, 4),
        "std": round(std_val, 4),
        "min": round(min(values), 4),
        "max": round(max(values), 4),
        "samples": [round(x, 4) for x in values],
    }


class J2CVectorRunner:
    """Harness for 6-axis coordinated/synchronous motion verification and safety audit."""

    def __init__(
        self,
        controller: SixMotorController,
        *,
        cycles: int = DEFAULT_CYCLES,
        return_to_ref: bool = True,
        log_file: str | Path | None = None,
        validated_config: bool = False,
    ) -> None:
        if controller.hardware_mode == HardwareMode.ACTIVE and not validated_config:
            raise TargetValidationError(
                "real J2-C requires explicit validated_config authorization"
            )
        self.controller = controller
        self.cycles = max(1, cycles)
        self.return_to_ref = return_to_ref
        self.validated_config = validated_config
        self.log_file = Path(log_file) if log_file else None
        self.connected = False
        self.configuration_started = False
        self.target_dispatch_started = False
        self.pause_attempted = False
        self._validated_forward_targets: list[int] | None = None
        self._validated_return_targets: list[int] | None = None
        self._target_dispatch_start_ns: int | None = None

        stage_name = "J2-C-CYCLIC" if (self.cycles > 1 or self.return_to_ref) else "J2-C"
        self._result: dict[str, Any] = {
            "stage": stage_name,
            "timestamp": datetime.now().isoformat(),
            "hardware_mode": controller.hardware_mode.value,
            "delta_steps": list(DELTA_STEPS),
            "cycles": self.cycles,
            "return_to_ref": self.return_to_ref,
            "timeout_per_phase_sec": TIMEOUT_PER_PHASE_SEC,
            "poll_period_sec": POLL_PERIOD_SEC,
            "settle_samples_required": SETTLE_SAMPLES,
            "confirmed_motion_consecutive_samples": CONFIRMED_MOTION_CONSECUTIVE_SAMPLES,
            "static_position_tolerance_steps": STATIC_POSITION_TOLERANCE_STEPS,
            "validated_config": validated_config,
            "outcome": "NOT_STARTED",
            "pre_flight": None,
            "baseline_analysis": None,
            "references": None,
            "forward_targets": None,
            "return_targets": None,
            "target_validation": None,
            "configuration_dispatch": [],
            "configuration_readback": None,
            "phases_summary": [],
            "cycles_data": [],
            "feedback_samples": [],
            "forward_stats": {},
            "return_stats": {},
            "repeatability": {},
            "cumulative_drift": {},
            "target_dispatch_spread": {},
            "tracking": {
                "motion_timing_definition": {
                    "first_raw_positive_sample": "first feedback position moving away from phase start",
                    "first_confirmed_motion": (
                        "first feedback position moving beyond static deadband (2 step) "
                        "for two consecutive samples"
                    ),
                },
                "first_raw_positive_sample_latency_ms": {},
                "first_confirmed_motion_latency_ms": {},
                "first_error_lte_3_ms": {},
                "first_error_lte_2_ms": {},
                "peak_current_mA": {str(mid): 0 for mid in MOTOR_IDS},
                "sync_metrics": {
                    "start_latencies_ms": {},
                    "start_skew_ms": None,
                    "arrival_latencies_ms": {},
                    "arrival_skew_ms": None,
                    "settled_axes": [],
                    "all_axes_settled": False,
                    "final_positions_steps": {},
                    "final_errors_steps": {},
                    "final_displacements_steps": {},
                },
            },
            "pause": None,
            "stop_confirmation": None,
            "post_flight": None,
            "uart_telemetry": {},
            "write_audit": None,
            "error": None,
        }

    @staticmethod
    def _drift_reason(samples: list[int]) -> str | None:
        if len(samples) < 4:
            return None
        if all(b >= a for a, b in zip(samples, samples[1:])) and samples[-1] > samples[0]:
            return f"monotonic_increasing start={samples[0]} end={samples[-1]}"
        if all(b <= a for a, b in zip(samples, samples[1:])) and samples[-1] < samples[0]:
            return f"monotonic_decreasing start={samples[0]} end={samples[-1]}"
        midpoint = len(samples) // 2
        first_half = samples[:midpoint]
        second_half = samples[midpoint:]
        if min(second_half) > max(first_half) or max(second_half) < min(first_half):
            return (
                f"disjoint_halves first=[{min(first_half)},{max(first_half)}] "
                f"second=[{min(second_half)},{max(second_half)}]"
            )
        return None

    @staticmethod
    def _stats(samples: list[int]) -> dict[str, Any]:
        histogram = Counter(samples)
        mode_count = max(histogram.values())
        candidates = sorted(value for value, count in histogram.items() if count == mode_count)
        minimum = min(samples)
        maximum = max(samples)
        return {
            "samples": list(samples),
            "sample_count": len(samples),
            "histogram": dict(sorted(histogram.items())),
            "mode": candidates[0],
            "mode_candidates": candidates,
            "min": minimum,
            "max": maximum,
            "span": maximum - minimum,
        }

    def _stop(self, reason: str) -> None:
        raise J2CStop(reason)

    def _collect_baseline(self, pre_flight: Any) -> tuple[dict[int, int], dict[int, Any]]:
        samples: dict[int, dict[str, list[int]]] = {
            mid: {"actual": [], "target": [], "current": [], "fault": []}
            for mid in MOTOR_IDS
        }
        for sample_index in range(BASELINE_SAMPLE_COUNT):
            for mid in MOTOR_IDS:
                configuration = self.controller.read_single_axis_preflight(mid)
                data = samples[mid]
                data["actual"].append(int(configuration["actual_position"]))
                data["target"].append(int(configuration["target_position"]))
                data["current"].append(int(configuration["current_mA"]))
                data["fault"].append(int(configuration["fault"]))
                if configuration["current_mA"] != 0:
                    self._result["baseline_analysis"] = self._partial_baseline_analysis(samples)
                    self._stop(f"ID{mid} baseline current={configuration['current_mA']}mA != 0")
                if configuration["fault"] != 0:
                    self._result["baseline_analysis"] = self._partial_baseline_analysis(samples)
                    self._stop(f"ID{mid} baseline fault=0x{configuration['fault']:02X}")
            if sample_index + 1 < BASELINE_SAMPLE_COUNT:
                time.sleep(BASELINE_SAMPLE_PERIOD_SEC)

        references: dict[int, int] = {}
        analysis: dict[int, Any] = {}
        for mid in MOTOR_IDS:
            actual_stats = self._stats(samples[mid]["actual"])
            target_stats = self._stats(samples[mid]["target"])
            reference = int(actual_stats["mode"])
            actual_drift = self._drift_reason(samples[mid]["actual"])
            target_drift = self._drift_reason(samples[mid]["target"])
            actual_deviation = max(abs(value - reference) for value in samples[mid]["actual"])
            target_deviation = max(abs(value - reference) for value in samples[mid]["target"])
            analysis[mid] = {
                "actual": actual_stats,
                "target_0x29": target_stats,
                "reference": reference,
                "reference_band": [
                    reference - STATIC_POSITION_TOLERANCE_STEPS,
                    reference + STATIC_POSITION_TOLERANCE_STEPS,
                ],
                "actual_max_reference_deviation": actual_deviation,
                "target_max_reference_deviation": target_deviation,
                "actual_drift": actual_drift,
                "target_drift": target_drift,
                "current_samples": list(samples[mid]["current"]),
                "fault_samples": list(samples[mid]["fault"]),
            }
            if actual_deviation > STATIC_POSITION_TOLERANCE_STEPS:
                self._result["baseline_analysis"] = _json_value(analysis)
                self._stop(
                    f"ID{mid} actual/reference deviation={actual_deviation} "
                    f"exceeds +/-{STATIC_POSITION_TOLERANCE_STEPS}"
                )
            if target_deviation > STATIC_POSITION_TOLERANCE_STEPS:
                self._result["baseline_analysis"] = _json_value(analysis)
                self._stop(
                    f"ID{mid} 0x29/reference deviation={target_deviation} "
                    f"exceeds +/-{STATIC_POSITION_TOLERANCE_STEPS}"
                )
            if target_drift is not None and target_stats["span"] > 1:
                self._result["baseline_analysis"] = _json_value(analysis)
                self._stop(
                    f"ID{mid} 0x29 sustained drift {target_drift} "
                    f"span={target_stats['span']}"
                )
            if actual_drift is not None:
                self._result["baseline_analysis"] = _json_value(analysis)
                self._stop(f"ID{mid} static baseline drift actual={actual_drift}")
            references[mid] = reference

        self._result["baseline_analysis"] = _json_value(analysis)
        self._result["preflight_reference_positions"] = [
            int(pre_flight.positions_steps[mid - 1]) for mid in MOTOR_IDS
        ]
        return references, analysis

    def _partial_baseline_analysis(self, samples: dict[int, dict[str, list[int]]]) -> dict[str, Any]:
        result: dict[int, Any] = {}
        for mid, data in samples.items():
            if data["actual"]:
                result[mid] = {
                    "actual": self._stats(data["actual"]),
                    "target_0x29": self._stats(data["target"]),
                    "current_samples": list(data["current"]),
                    "fault_samples": list(data["fault"]),
                }
        return _json_value(result)

    def _validate_static_snapshot(self, snapshot: Any) -> None:
        for mid, status in zip(MOTOR_IDS, snapshot.states):
            if status.current_mA != 0:
                self._stop(f"ID{mid} pre-flight current={status.current_mA}mA != 0")
            if status.error_code != 0:
                self._stop(f"ID{mid} pre-flight fault=0x{status.error_code:02X}")

    def _read_configuration(self, references: dict[int, int]) -> dict[int, dict[str, int]]:
        readback: dict[int, dict[str, int]] = {}
        for mid in MOTOR_IDS:
            configuration = self.controller.read_single_axis_preflight(mid)
            reference = references[mid]
            if configuration["control_mode"] != SPEED_MODE:
                self._stop(f"ID{mid} config readback mode={configuration['control_mode']} != 2")
            if configuration["target_speed"] != DEFAULT_SPEED:
                self._stop(f"ID{mid} config readback speed={configuration['target_speed']} != 50")
            if abs(configuration["actual_position"] - reference) > STATIC_POSITION_TOLERANCE_STEPS:
                self._stop(f"ID{mid} config readback actual outside reference deadband")
            if abs(configuration["target_position"] - reference) > STATIC_POSITION_TOLERANCE_STEPS:
                self._stop(f"ID{mid} config readback 0x29 outside reference deadband")
            if configuration["current_mA"] != 0 or configuration["fault"] != 0:
                self._stop(f"ID{mid} config readback current/fault unsafe: {configuration}")
            readback[mid] = dict(configuration)
        return readback

    def _raw_write_audit(self) -> list[dict[str, Any]]:
        bus = self.controller.bus
        raw = getattr(bus, "write_audit", None)
        if raw is not None:
            return [
                {
                    "timestamp_ns": record.get("timestamp_ns"),
                    "timestamp_monotonic_ns": record.get("timestamp_monotonic_ns"),
                    "motor_id": int(record["motor_id"]),
                    "register_address": int(record["register_address"]),
                    "payload": [int(value) for value in record["payload"]],
                }
                for record in raw
            ]
        return [
            {
                "timestamp_ns": None,
                "timestamp_monotonic_ns": None,
                "motor_id": event[0],
                "register_address": event[2],
                "payload": list(event[3]),
            }
            for event in getattr(bus, "events", [])
            if event[1] == 0x32
        ]

    def _mark_controller_pause(self) -> None:
        pause_frames = [record for record in self._raw_write_audit() if record["register_address"] == PAUSE_ADDR]
        if pause_frames:
            self.pause_attempted = True
            self._result["pause"] = {
                "attempted": True,
                "source": "command_positions_with_metrics emergency pause_all",
                "results": None,
                "all_axes_attempted": {
                    str(mid): any(frame["motor_id"] == mid for frame in pause_frames)
                    for mid in MOTOR_IDS
                },
                "hard_stop_required": len({frame["motor_id"] for frame in pause_frames}) < len(MOTOR_IDS),
            }

    def _best_effort_pause(self, reason: str) -> None:
        if self.pause_attempted:
            return
        self.pause_attempted = True
        try:
            results = self.controller.pause_all()
            all_success = all(bool(results.get(mid, False)) for mid in MOTOR_IDS)
            self._result["pause"] = {
                "attempted": True,
                "reason": reason,
                "results": {str(mid): bool(results.get(mid, False)) for mid in MOTOR_IDS},
                "all_axes_attempted": {str(mid): mid in results for mid in MOTOR_IDS},
                "hard_stop_required": not all_success,
            }
        except Exception as exc:
            self._result["pause"] = {
                "attempted": True,
                "reason": reason,
                "results": None,
                "all_axes_attempted": {str(mid): False for mid in MOTOR_IDS},
                "hard_stop_required": True,
                "error": str(exc),
            }

    def _pause_confirmable(self) -> bool:
        pause = self._result.get("pause") or {}
        attempted = pause.get("all_axes_attempted") or {}
        return bool(
            pause
            and not pause.get("hard_stop_required", True)
            and all(bool(attempted.get(str(mid), False)) for mid in MOTOR_IDS)
        )

    def _ensure_stop_confirmation(self) -> None:
        if not self.connected or not self.pause_attempted:
            return
        if not self._pause_confirmable():
            self._stop("pause_all failed; HARD STOP REQUIRED")
        self._confirm_stop()

    def _confirm_stop(self) -> None:
        confirmations: list[dict[str, Any]] = []
        previous: list[int] | None = None
        stable = False
        for _ in range(2):
            snapshot = self.controller.read_state_snapshot()
            current_positions = list(snapshot.positions_steps)
            confirmations.append(
                {
                    "timestamp_end_ns": snapshot.timestamp_end_ns,
                    "positions_steps": current_positions,
                    "currents_mA": list(snapshot.currents_mA),
                    "fault_codes": list(snapshot.fault_codes),
                }
            )
            if previous is not None and previous == current_positions:
                stable = True
            previous = current_positions
            time.sleep(POLL_PERIOD_SEC)
        health_safe = all(
            current == 0 and fault == 0
            for item in confirmations
            for current, fault in zip(item["currents_mA"], item["fault_codes"])
        )
        self._result["stop_confirmation"] = {
            "confirmed": stable and health_safe,
            "samples": confirmations,
            "health_safe": health_safe,
            "stable": stable,
            "hard_stop_required": not (stable and health_safe),
        }
        if not stable or not health_safe:
            self._stop("pause/stop confirmation failed; HARD STOP REQUIRED")

    def _run_phase(
        self,
        *,
        cycle_idx: int,
        direction: str,
        target_vector: list[int],
        start_positions: list[int],
    ) -> dict[str, Any]:
        """Execute one directional motion phase with full telemetry and sync metrics."""
        phase_label = f"cycle_{cycle_idx}_{direction}"
        confirmed_streak = {mid: 0 for mid in MOTOR_IDS}
        first_raw_lat_ms: dict[str, float] = {}
        first_confirmed_lat_ms: dict[str, float] = {}
        first_error_lte_3_ms: dict[str, float] = {}
        first_error_lte_2_ms: dict[str, float] = {}
        peak_currents = {str(mid): 0 for mid in MOTOR_IDS}

        self.target_dispatch_started = True
        phase_dispatch_start_ns = time.perf_counter_ns()
        try:
            command_result = self.controller.command_positions_with_metrics(target_vector)
        except Exception:
            self._mark_controller_pause()
            raise

        dispatch_duration_ms = (command_result.timestamp_end - command_result.timestamp_start) * 1000.0
        phase_dispatch_info = {
            "phase": phase_label,
            "cycle_index": cycle_idx,
            "direction": direction,
            "commanded_targets": list(command_result.commanded_targets),
            "start_positions": list(start_positions),
            "timestamp_start": command_result.timestamp_start,
            "timestamp_end": command_result.timestamp_end,
            "dispatch_duration_ms": round(dispatch_duration_ms, 3),
            "timestamps": _json_value(command_result.per_axis_write_timestamps),
            "timestamps_ns": _json_value(command_result.per_axis_write_timestamps_ns),
            "completion_timestamps": _json_value(command_result.per_axis_write_completion_timestamps),
            "completion_timestamps_ns": _json_value(command_result.per_axis_write_completion_timestamps_ns),
        }

        deadline = time.monotonic() + TIMEOUT_PER_PHASE_SEC
        ready_samples = 0
        settled_early = False
        phase_feedback_samples: list[dict[str, Any]] = []

        while time.monotonic() < deadline:
            try:
                snapshot = self.controller.read_state_snapshot()
            except SafetyViolation as exc:
                self._stop(str(exc))

            elapsed_ms = (snapshot.timestamp_end_ns - phase_dispatch_start_ns) / 1e6
            errors = [status.current_position - tgt for status, tgt in zip(snapshot.states, target_vector)]
            sample = {
                "phase": phase_label,
                "cycle_index": cycle_idx,
                "direction": direction,
                "elapsed_since_phase_dispatch_ms": round(elapsed_ms, 3),
                "positions_steps": list(snapshot.positions_steps),
                "targets_steps": list(target_vector),
                "errors_steps": errors,
                "currents_mA": list(snapshot.currents_mA),
                "fault_codes": list(snapshot.fault_codes),
                "timestamp_end_ns": snapshot.timestamp_end_ns,
            }
            self._result["feedback_samples"].append(sample)
            phase_feedback_samples.append(sample)

            for index, mid in enumerate(MOTOR_IDS):
                mid_str = str(mid)
                cur_pos = snapshot.positions_steps[index]
                cur_start = start_positions[index]
                cur_current = snapshot.currents_mA[index]
                peak_currents[mid_str] = max(peak_currents[mid_str], cur_current)

                # Track motion relative to start position of this phase
                if direction == "forward":
                    moved_raw = cur_pos > cur_start
                    moved_confirmed = cur_pos > (cur_start + STATIC_POSITION_TOLERANCE_STEPS)
                else:
                    moved_raw = cur_pos < cur_start
                    moved_confirmed = cur_pos < (cur_start - STATIC_POSITION_TOLERANCE_STEPS)

                if mid_str not in first_raw_lat_ms and moved_raw:
                    first_raw_lat_ms[mid_str] = round(elapsed_ms, 3)

                if moved_confirmed:
                    confirmed_streak[mid] += 1
                else:
                    confirmed_streak[mid] = 0

                if confirmed_streak[mid] >= CONFIRMED_MOTION_CONSECUTIVE_SAMPLES and mid_str not in first_confirmed_lat_ms:
                    first_confirmed_lat_ms[mid_str] = round(elapsed_ms, 3)

                if abs(errors[index]) <= 3 and mid_str not in first_error_lte_3_ms:
                    first_error_lte_3_ms[mid_str] = round(elapsed_ms, 3)
                if abs(errors[index]) <= 2 and mid_str not in first_error_lte_2_ms:
                    first_error_lte_2_ms[mid_str] = round(elapsed_ms, 3)

            # Check Safety Redlines
            if any(current > 500 for current in snapshot.currents_mA):
                self._stop(f"runtime current exceeds 500mA in {phase_label}: {snapshot.currents_mA}")
            if any(fault != 0 for fault in snapshot.fault_codes):
                self._stop(f"runtime fault is non-zero in {phase_label}: {snapshot.fault_codes}")

            # Settling condition: all axes error <= 2
            if all(abs(error) <= 2 for error in errors):
                ready_samples += 1
                if ready_samples >= SETTLE_SAMPLES:
                    settled_early = True
                    break
            else:
                ready_samples = 0
            time.sleep(POLL_PERIOD_SEC)

        # Compute phase sync metrics
        final_sample = phase_feedback_samples[-1] if phase_feedback_samples else None
        final_positions = final_sample["positions_steps"] if final_sample else list(start_positions)
        final_errors = [pos - tgt for pos, tgt in zip(final_positions, target_vector)]
        displacements = [pos - start for pos, start in zip(final_positions, start_positions)]

        start_lats = dict(first_confirmed_lat_ms)
        for mid in MOTOR_IDS:
            mid_str = str(mid)
            if mid_str not in start_lats and mid_str in first_raw_lat_ms:
                start_lats[mid_str] = first_raw_lat_ms[mid_str]

        start_skew_ms = (max(start_lats.values()) - min(start_lats.values())) if len(start_lats) >= 2 else 0.0
        arrival_lats = dict(first_error_lte_2_ms)
        arrival_skew_ms = (max(arrival_lats.values()) - min(arrival_lats.values())) if len(arrival_lats) >= 2 else 0.0
        settled_axes = [mid for mid in MOTOR_IDS if str(mid) in arrival_lats]

        phase_summary = {
            "phase": phase_label,
            "cycle_index": cycle_idx,
            "direction": direction,
            "dispatch": phase_dispatch_info,
            "settled_early": settled_early,
            "start_latencies_ms": start_lats,
            "start_skew_ms": round(start_skew_ms, 3),
            "arrival_latencies_ms": arrival_lats,
            "arrival_skew_ms": round(arrival_skew_ms, 3),
            "first_raw_latencies_ms": first_raw_lat_ms,
            "first_confirmed_latencies_ms": first_confirmed_lat_ms,
            "settled_axes": settled_axes,
            "all_axes_settled": len(settled_axes) == len(MOTOR_IDS),
            "final_positions_steps": {str(mid): final_positions[i] for i, mid in enumerate(MOTOR_IDS)},
            "final_errors_steps": {str(mid): final_errors[i] for i, mid in enumerate(MOTOR_IDS)},
            "displacements_steps": {str(mid): displacements[i] for i, mid in enumerate(MOTOR_IDS)},
            "peak_currents_mA": peak_currents,
        }
        self._result["phases_summary"].append(phase_summary)
        return phase_summary

    def _compute_multicycle_statistics(self, references: dict[int, int]) -> None:
        """Aggregate forward/return statistics, cycle repeatability, and cumulative drift."""
        phases = self._result["phases_summary"]
        fwd_phases = [p for p in phases if p["direction"] == "forward"]
        ret_phases = [p for p in phases if p["direction"] == "return"]

        # 1. Forward Stats
        fwd_start_skews = [p["start_skew_ms"] for p in fwd_phases]
        fwd_arrival_skews = [p["arrival_skew_ms"] for p in fwd_phases]
        fwd_per_axis_start = {
            str(mid): [p["start_latencies_ms"][str(mid)] for p in fwd_phases if str(mid) in p["start_latencies_ms"]]
            for mid in MOTOR_IDS
        }
        fwd_per_axis_arrival = {
            str(mid): [p["arrival_latencies_ms"][str(mid)] for p in fwd_phases if str(mid) in p["arrival_latencies_ms"]]
            for mid in MOTOR_IDS
        }
        fwd_per_axis_disp = {
            str(mid): [p["displacements_steps"][str(mid)] for p in fwd_phases]
            for mid in MOTOR_IDS
        }
        self._result["forward_stats"] = {
            "start_skew_ms": _calc_stats(fwd_start_skews),
            "arrival_skew_ms": _calc_stats(fwd_arrival_skews),
            "per_axis_start_latency_ms": {k: _calc_stats(v) for k, v in fwd_per_axis_start.items()},
            "per_axis_arrival_latency_ms": {k: _calc_stats(v) for k, v in fwd_per_axis_arrival.items()},
            "per_axis_displacement_steps": {k: _calc_stats(v) for k, v in fwd_per_axis_disp.items()},
        }

        # 2. Return Stats
        if ret_phases:
            ret_start_skews = [p["start_skew_ms"] for p in ret_phases]
            ret_arrival_skews = [p["arrival_skew_ms"] for p in ret_phases]
            ret_per_axis_start = {
                str(mid): [p["start_latencies_ms"][str(mid)] for p in ret_phases if str(mid) in p["start_latencies_ms"]]
                for mid in MOTOR_IDS
            }
            ret_per_axis_arrival = {
                str(mid): [p["arrival_latencies_ms"][str(mid)] for p in ret_phases if str(mid) in p["arrival_latencies_ms"]]
                for mid in MOTOR_IDS
            }
            ret_per_axis_disp = {
                str(mid): [p["displacements_steps"][str(mid)] for p in ret_phases]
                for mid in MOTOR_IDS
            }
            self._result["return_stats"] = {
                "start_skew_ms": _calc_stats(ret_start_skews),
                "arrival_skew_ms": _calc_stats(ret_arrival_skews),
                "per_axis_start_latency_ms": {k: _calc_stats(v) for k, v in ret_per_axis_start.items()},
                "per_axis_arrival_latency_ms": {k: _calc_stats(v) for k, v in ret_per_axis_arrival.items()},
                "per_axis_displacement_steps": {k: _calc_stats(v) for k, v in ret_per_axis_disp.items()},
            }

        # 3. Cycle Data packaging & Repeatability
        cycles_data: list[dict[str, Any]] = []
        fwd_peak_positions = {str(mid): [] for mid in MOTOR_IDS}
        ret_end_positions = {str(mid): [] for mid in MOTOR_IDS}
        cumulative_drift_per_cycle = []

        for c_idx in range(1, self.cycles + 1):
            fwd_p = next((p for p in fwd_phases if p["cycle_index"] == c_idx), None)
            ret_p = next((p for p in ret_phases if p["cycle_index"] == c_idx), None)
            if fwd_p:
                for mid in MOTOR_IDS:
                    fwd_peak_positions[str(mid)].append(fwd_p["final_positions_steps"][str(mid)])
            if ret_p:
                for mid in MOTOR_IDS:
                    ret_end_positions[str(mid)].append(ret_p["final_positions_steps"][str(mid)])
                cycle_drift = {
                    str(mid): ret_p["final_positions_steps"][str(mid)] - references[mid]
                    for mid in MOTOR_IDS
                }
                cumulative_drift_per_cycle.append(
                    {
                        "cycle_index": c_idx,
                        "drift_steps": cycle_drift,
                        "drift_mm": {k: round(v * 0.025, 4) for k, v in cycle_drift.items()},
                    }
                )
            cycles_data.append({
                "cycle_index": c_idx,
                "forward": fwd_p,
                "return": ret_p,
            })
        self._result["cycles_data"] = cycles_data

        # 4. Repeatability (Standard deviations)
        self._result["repeatability"] = {
            "forward_peak_positions_steps": {k: _calc_stats(v) for k, v in fwd_peak_positions.items()},
            "return_positions_steps": {k: _calc_stats(v) for k, v in ret_end_positions.items()},
        }

        # 5. Cumulative drift
        last_ret = ret_phases[-1] if ret_phases else (fwd_phases[-1] if fwd_phases else None)
        final_pos = last_ret["final_positions_steps"] if last_ret else {str(mid): references[mid] for mid in MOTOR_IDS}
        net_drift = {str(mid): final_pos[str(mid)] - references[mid] for mid in MOTOR_IDS}
        self._result["cumulative_drift"] = {
            "per_cycle_return_drift": cumulative_drift_per_cycle,
            "final_net_drift_steps": net_drift,
            "final_net_drift_mm": {k: round(v * 0.025, 4) for k, v in net_drift.items()},
        }

        # 6. Target dispatch spread across all phases
        spreads = [p["dispatch"]["dispatch_duration_ms"] for p in phases]
        self._result["target_dispatch_spread"] = _calc_stats(spreads)

        # 7. Update top-level tracking sync_metrics with latest phase / forward summary
        tracking = self._result["tracking"]
        first_phase = phases[0] if phases else None
        last_phase = phases[-1] if phases else None
        if first_phase:
            tracking["first_raw_positive_sample_latency_ms"] = first_phase["first_raw_latencies_ms"]
            tracking["first_confirmed_motion_latency_ms"] = first_phase["first_confirmed_latencies_ms"]
        sync = tracking["sync_metrics"]
        if last_phase:
            sync["start_latencies_ms"] = last_phase["start_latencies_ms"]
            sync["start_skew_ms"] = last_phase["start_skew_ms"]
            sync["arrival_latencies_ms"] = last_phase["arrival_latencies_ms"]
            sync["arrival_skew_ms"] = last_phase["arrival_skew_ms"]
            sync["settled_axes"] = last_phase["settled_axes"]
            sync["all_axes_settled"] = last_phase["all_axes_settled"]
            sync["final_positions_steps"] = last_phase["final_positions_steps"]
            sync["final_errors_steps"] = last_phase["final_errors_steps"]
            sync["final_displacements_steps"] = {
                str(mid): last_phase["final_positions_steps"][str(mid)] - references[mid]
                for mid in MOTOR_IDS
            }
        for p in phases:
            for mid_str, cur in p["peak_currents_mA"].items():
                tracking["peak_current_mA"][mid_str] = max(tracking["peak_current_mA"][mid_str], cur)

    def _telemetry(self) -> dict[str, Any]:
        bus = self.controller.bus
        getter = getattr(bus, "get_telemetry", None)
        if callable(getter):
            return _json_value(getter())
        return {
            "transaction_count": getattr(bus, "transaction_count", 0),
            "attempt_count": getattr(bus, "attempt_count", 0),
            "retry_count": getattr(bus, "retry_count", 0),
            "timeout_count": getattr(bus, "timeout_count", 0),
            "checksum_error_count": getattr(bus, "checksum_error_count", 0),
            "protocol_error_count": getattr(bus, "protocol_error_count", 0),
            "failed_transaction_count": getattr(bus, "failed_transaction_count", 0),
        }

    def _write_audit(self) -> dict[str, Any]:
        frames = self._raw_write_audit()
        telemetry = self._result.get("uart_telemetry", {})
        expected: list[tuple[int, int, list[int]]] = []
        references = self._result.get("references") or {}
        fwd_targets = self._result.get("forward_targets") or {}
        ret_targets = self._result.get("return_targets") or {}

        # 1. Config frames (6 frames)
        for mid in MOTOR_IDS:
            ref = references.get(str(mid), references.get(mid))
            if ref is not None:
                expected.append((mid, 0x25, [2, 0, 0, 50, int(ref)]))

        # 2. Cycles target frames
        for _ in range(self.cycles):
            for mid in MOTOR_IDS:
                tgt = fwd_targets.get(str(mid), fwd_targets.get(mid))
                if tgt is not None:
                    expected.append((mid, 0x29, [int(tgt)]))
            if self.return_to_ref:
                for mid in MOTOR_IDS:
                    tgt = ret_targets.get(str(mid), ret_targets.get(mid))
                    if tgt is not None:
                        expected.append((mid, 0x29, [int(tgt)]))

        # 3. Pause frames (6 frames)
        expected.extend((mid, 0x1A, [1]) for mid in MOTOR_IDS)

        actual = [
            (frame["motor_id"], frame["register_address"], frame["payload"])
            for frame in frames
        ]
        per_axis = {str(mid): sum(frame["motor_id"] == mid for frame in frames) for mid in MOTOR_IDS}
        config_counts = {str(mid): sum(frame["motor_id"] == mid and frame["register_address"] == 0x25 for frame in frames) for mid in MOTOR_IDS}
        target_counts = {str(mid): sum(frame["motor_id"] == mid and frame["register_address"] == 0x29 for frame in frames) for mid in MOTOR_IDS}
        pause_counts = {str(mid): sum(frame["motor_id"] == mid and frame["register_address"] == 0x1A for frame in frames) for mid in MOTOR_IDS}
        expected_write_count = len(expected)

        return {
            "controller_write_count": self.controller.write_count,
            "expected_write_count": expected_write_count,
            "frames": frames,
            "expected_success_sequence": expected,
            "actual_sequence": actual,
            "sequence_is_expected": actual == expected,
            "per_axis_write_counts": per_axis,
            "config_write_counts": config_counts,
            "target_write_counts": target_counts,
            "pause_write_counts": pause_counts,
            "retry_count": telemetry.get("retry_count", 0),
            "no_retry": telemetry.get("retry_count", 0) == 0,
            "pass": self._result["outcome"] == "PASS" and actual == expected and self.controller.write_count == expected_write_count,
        }

    def run(self) -> dict[str, Any]:
        try:
            if self.controller.hardware_mode == HardwareMode.READ_ONLY:
                raise J2CStop("J2-C requires MOCK or explicitly approved ACTIVE mode")
            self.controller.connect()
            self.connected = True
            limits = self.controller.read_limits()
            pre_flight = self.controller.read_state_snapshot()
            self._validate_static_snapshot(pre_flight)
            self._result["pre_flight"] = {"limits": limits, "snapshot": _json_value(pre_flight)}

            # Baseline Admission
            references, _ = self._collect_baseline(pre_flight)
            self._result["references"] = {str(mid): references[mid] for mid in MOTOR_IDS}

            # Forward and Return Targets
            fwd_targets = [references[mid] + DELTA_STEPS[mid - 1] for mid in MOTOR_IDS]
            ret_targets = [references[mid] for mid in MOTOR_IDS]

            try:
                validated_fwd = validate_target_vector(fwd_targets, limits)
                validated_ret = validate_target_vector(ret_targets, limits)
            except Exception as exc:
                self._result["target_validation"] = {"valid": False, "error": str(exc)}
                self._stop(f"target vector validation failed: {exc}")

            self._result["target_validation"] = {
                "valid": True,
                "forward_targets": list(validated_fwd),
                "return_targets": list(validated_ret),
            }
            self._validated_forward_targets = list(validated_fwd)
            self._validated_return_targets = list(validated_ret)
            self._result["forward_targets"] = {str(mid): validated_fwd[mid - 1] for mid in MOTOR_IDS}
            self._result["return_targets"] = {str(mid): validated_ret[mid - 1] for mid in MOTOR_IDS}
            self._result["targets"] = self._result["forward_targets"]

            # Configuration Block Writes (0x25, 6 writes total)
            for mid in MOTOR_IDS:
                self.configuration_started = True
                dispatch_start_ns = time.perf_counter_ns()
                dispatch_start = time.time()
                response = self.controller.write_validated_config_block(mid, references[mid])
                dispatch_end_ns = time.perf_counter_ns()
                self._result["configuration_dispatch"].append(
                    {
                        "motor_id": mid,
                        "register_address": 0x25,
                        "payload": [2, 0, 0, 50, references[mid]],
                        "timestamp_start": dispatch_start,
                        "timestamp_start_ns": dispatch_start_ns,
                        "timestamp_end": time.time(),
                        "timestamp_end_ns": dispatch_end_ns,
                        "response": _json_value(response),
                    }
                )

            # Configuration Readback Check
            self._result["configuration_readback"] = _json_value(self._read_configuration(references))

            # Execute Cyclic Motion Loop
            current_positions = [references[mid] for mid in MOTOR_IDS]
            for cycle_idx in range(1, self.cycles + 1):
                # 1. Forward Phase
                fwd_summary = self._run_phase(
                    cycle_idx=cycle_idx,
                    direction="forward",
                    target_vector=validated_fwd,
                    start_positions=current_positions,
                )
                current_positions = [fwd_summary["final_positions_steps"][str(mid)] for mid in MOTOR_IDS]
                time.sleep(POLL_PERIOD_SEC)

                # 2. Return Phase (if enabled)
                if self.return_to_ref:
                    ret_summary = self._run_phase(
                        cycle_idx=cycle_idx,
                        direction="return",
                        target_vector=validated_ret,
                        start_positions=current_positions,
                    )
                    current_positions = [ret_summary["final_positions_steps"][str(mid)] for mid in MOTOR_IDS]
                    time.sleep(POLL_PERIOD_SEC)

            # Test completed successfully -> Single clean pause_all and stop confirmation
            self._best_effort_pause("all_cycles_complete")
            self._ensure_stop_confirmation()
            post = self.controller.read_state_snapshot()
            self._result["post_flight"] = _json_value(post)
            self._compute_multicycle_statistics(references)
            self._result["outcome"] = "PASS"

        except J2CStop as exc:
            self._result["outcome"] = "STOP"
            original_error = str(exc)
            self._result["error"] = original_error
            if (self.configuration_started or self.target_dispatch_started) and not self.pause_attempted:
                self._best_effort_pause(str(exc))
            if self.pause_attempted and self._result["stop_confirmation"] is None:
                try:
                    self._ensure_stop_confirmation()
                except J2CStop as confirm_exc:
                    self._result["error"] = f"{original_error}; {confirm_exc}"
        except Exception as exc:  # pragma: no cover - exercised by fault injection
            self._result["outcome"] = "FAIL"
            original_error = str(exc)
            self._result["error"] = original_error
            if (self.configuration_started or self.target_dispatch_started) and not self.pause_attempted:
                self._best_effort_pause(str(exc))
            if self.pause_attempted and self._result["stop_confirmation"] is None:
                try:
                    self._ensure_stop_confirmation()
                except J2CStop as confirm_exc:
                    self._result["error"] = f"{original_error}; {confirm_exc}"
        finally:
            if self.connected and self._result["post_flight"] is None:
                try:
                    self._result["post_flight"] = _json_value(self.controller.read_state_snapshot())
                except Exception as exc:
                    self._result["post_flight_error"] = str(exc)
            if self.connected:
                self._result["uart_telemetry"] = self._telemetry()
                self._result["write_audit"] = self._write_audit()
                self.controller.close()
            else:
                self._result["uart_telemetry"] = self._telemetry()
                self._result["write_audit"] = self._write_audit()
            if self.log_file is not None:
                self.log_file.parent.mkdir(parents=True, exist_ok=True)
                self.log_file.write_text(json.dumps(_json_value(self._result), indent=2), encoding="utf-8")
        return self._result


def _default_log_path(log_dir: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return log_dir / f"j2c_cyclic_{stamp}.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J2-C six-axis multi-cycle cyclic motion bring-up (fail-closed)")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES, help="Number of round-trip cycles (default: 5)")
    parser.add_argument("--single-direction", action="store_true", help="Do not return to ref (single forward move)")
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument("--confirm-vector", action="store_true")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument("--log-dir", type=Path, default=WORKSPACE_ROOT / "log")
    args = parser.parse_args(argv)

    if args.real_hardware and not (args.confirm_motion and args.validated_config and args.confirm_vector):
        parser.error("real J2-C requires --confirm-motion, --validated-config, and --confirm-vector")
    if not args.real_hardware and (args.confirm_motion or args.validated_config or args.confirm_vector):
        parser.error("J2-C authorization flags are valid only with --real-hardware")

    mode = HardwareMode.ACTIVE if args.real_hardware else HardwareMode.MOCK
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        hardware_mode=mode,
    )
    log_file = args.log_file or _default_log_path(args.log_dir)
    result = J2CVectorRunner(
        controller,
        cycles=args.cycles,
        return_to_ref=not args.single_direction,
        log_file=log_file,
        validated_config=args.validated_config,
    ).run()

    print(
        json.dumps(
            {
                "outcome": result["outcome"],
                "stage": result["stage"],
                "cycles": result["cycles"],
                "log_file": str(log_file),
                "references": result.get("references"),
                "forward_stats": result.get("forward_stats"),
                "return_stats": result.get("return_stats"),
                "repeatability": result.get("repeatability"),
                "cumulative_drift": result.get("cumulative_drift"),
                "target_dispatch_spread": result.get("target_dispatch_spread"),
                "write_audit": {
                    "write_count": (result.get("write_audit") or {}).get("controller_write_count"),
                    "expected_write_count": (result.get("write_audit") or {}).get("expected_write_count"),
                    "no_retry": (result.get("write_audit") or {}).get("no_retry"),
                    "pass": (result.get("write_audit") or {}).get("pass"),
                },
                "error": result.get("error"),
            },
            indent=2,
        )
    )
    return 0 if result["outcome"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
