#!/usr/bin/env python3
"""Preassembly J5-C2-HW acceptance toolchain (Mock/synthetic/replay only).

The runner consumes TimedJointFeedback-shaped frames, performs the frozen
Fixed-Lag -> q_sync -> numerical FK path, and writes one JSON plus one CSV
report. It never opens a UART, starts a ROS node, or sends a motor command.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
KIN_PKG_ROOT = SCRIPT_DIR.parent / "src" / "pus6_kinematics"
if str(KIN_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(KIN_PKG_ROOT))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.geometry_config import GeometryConfig, load_geometry_config
from pus6_kinematics.kinematics_interface import (
    KinematicsError,
    SingularityError,
    WorkspaceLimitError,
)
from pus6_kinematics.state_synchronizer import FixedLagJointSynchronizer, SyncStatus


@dataclass(frozen=True)
class TimedFeedbackFrame:
    """JSON/replay representation of ``pus6_msgs/TimedJointFeedback``."""

    uart_cycle_sequence: int
    per_axis_rx_done_monotonic_ns: tuple[int, ...]
    positions_meters: tuple[float, ...]
    raw_steps: tuple[float, ...]
    snapshot_start_monotonic_ns: int
    snapshot_end_monotonic_ns: int
    fault_codes: tuple[int, ...]

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "TimedFeedbackFrame":
        stamps = tuple(int(v) for v in raw["per_axis_rx_done_monotonic_ns"])
        positions = tuple(float(v) for v in raw["positions_meters"])
        steps = tuple(float(v) for v in raw["raw_steps"])
        if len(stamps) != 6 or len(positions) != 6 or len(steps) != 6:
            raise ValueError("TimedJointFeedback frame arrays must all have length 6")
        start = int(raw.get("snapshot_start_monotonic_ns", stamps[0]))
        end = int(raw.get("snapshot_end_monotonic_ns", stamps[-1]))
        faults = tuple(int(v) for v in raw.get("fault_codes", [0] * 6))
        if len(faults) != 6:
            raise ValueError("fault_codes must have length 6")
        return cls(
            uart_cycle_sequence=int(raw.get("uart_cycle_sequence", -1)),
            per_axis_rx_done_monotonic_ns=stamps,
            positions_meters=positions,
            raw_steps=steps,
            snapshot_start_monotonic_ns=start,
            snapshot_end_monotonic_ns=end,
            fault_codes=faults,
        )


def _field(msg: Any, name: str) -> Any:
    if isinstance(msg, Mapping):
        return msg[name]
    return getattr(msg, name)


def frame_from_message(msg: Any) -> TimedFeedbackFrame:
    """Adapt a real/generated TimedJointFeedback object without importing ROS."""
    return TimedFeedbackFrame.from_mapping(
        {
            "uart_cycle_sequence": _field(msg, "uart_cycle_sequence"),
            "per_axis_rx_done_monotonic_ns": _field(msg, "per_axis_rx_done_monotonic_ns"),
            "positions_meters": _field(msg, "positions_meters"),
            "raw_steps": _field(msg, "raw_steps"),
            "snapshot_start_monotonic_ns": _field(msg, "snapshot_start_monotonic_ns"),
            "snapshot_end_monotonic_ns": _field(msg, "snapshot_end_monotonic_ns"),
            "fault_codes": _field(msg, "fault_codes"),
        }
    )


def _percentiles(values: Iterable[float]) -> dict[str, float | None]:
    ordered = sorted(float(v) for v in values)
    if not ordered:
        return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None}

    def percentile(q: float) -> float:
        index = (len(ordered) - 1) * q
        low = math.floor(index)
        high = math.ceil(index)
        if low == high:
            return ordered[low]
        return ordered[low] + (ordered[high] - ordered[low]) * (index - low)

    return {
        "count": len(ordered),
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "max": ordered[-1],
    }


def so3_geodesic_distance_rad(R1: list[list[float]], R2: list[list[float]]) -> float:
    trace_product = sum(R1[row][col] * R2[row][col] for row in range(3) for col in range(3))
    return math.acos(max(-1.0, min(1.0, (trace_product - 1.0) * 0.5)))


class C2HwAcceptanceRunner:
    """Offline acceptance runner for TimedJointFeedback-shaped input."""

    def __init__(
        self,
        config: GeometryConfig | None = None,
        fixed_lag_delay_ms: float = 50.0,
        max_residual_m: float = 1e-4,
        max_pos_jump_m: float = 0.015,
        max_ang_jump_rad: float = 0.15,
    ) -> None:
        self.config = config or load_geometry_config()
        self.fixed_lag_delay_ns = int(fixed_lag_delay_ms * 1e6)
        self.max_residual_m = max_residual_m
        self.max_pos_jump_m = max_pos_jump_m
        self.max_ang_jump_rad = max_ang_jump_rad
        self.synchronizer = FixedLagJointSynchronizer(max_history_size=500)
        self.kinematics = Analytical6PusKinematics(config_path=self.config.path)
        self._last_successful_pose: list[float] | None = None
        self._last_successful_R: list[list[float]] | None = None
        self.records: list[dict[str, Any]] = []
        self._frame_count = 0
        self._fault_frame_count = 0
        self._sequence_values: list[int] = []
        self._sequence_duplicate_count = 0
        self._sequence_gap_count = 0
        self._sequence_out_of_order_count = 0
        self._sequence_last: int | None = None
        self._timestamp_violation_count = 0
        self._scan_spans_us: list[float] = []
        self._axis_spans_us: list[float] = []

    def ingest_timed_feedback(self, msg: Any) -> TimedFeedbackFrame:
        frame = msg if isinstance(msg, TimedFeedbackFrame) else frame_from_message(msg)
        self.synchronizer.push_raw_snapshot(
            frame.per_axis_rx_done_monotonic_ns,
            frame.positions_meters,
            frame.raw_steps,
        )
        self._frame_count += 1
        self._sequence_values.append(frame.uart_cycle_sequence)
        if self._sequence_last is not None:
            delta = frame.uart_cycle_sequence - self._sequence_last
            if delta == 0:
                self._sequence_duplicate_count += 1
            elif delta < 0:
                self._sequence_out_of_order_count += 1
            elif delta > 1:
                self._sequence_gap_count += delta - 1
        self._sequence_last = frame.uart_cycle_sequence
        if any(frame.fault_codes):
            self._fault_frame_count += 1
        if any(
            frame.per_axis_rx_done_monotonic_ns[idx]
            <= frame.per_axis_rx_done_monotonic_ns[idx - 1]
            for idx in range(1, 6)
        ):
            self._timestamp_violation_count += 1
        self._scan_spans_us.append(
            (frame.snapshot_end_monotonic_ns - frame.snapshot_start_monotonic_ns) / 1e3
        )
        self._axis_spans_us.append(
            (max(frame.per_axis_rx_done_monotonic_ns) - min(frame.per_axis_rx_done_monotonic_ns)) / 1e3
        )
        return frame

    def query(self, query_timestamp_ns: int) -> dict[str, Any]:
        bracketed_axes = self.synchronizer.bracketed_axes(query_timestamp_ns)
        sync_res = self.synchronizer.query_sync_configuration(query_timestamp_ns)
        record: dict[str, Any] = {
            "query_timestamp_ns": int(query_timestamp_ns),
            "bracketed_axes": bracketed_axes,
            "bracketed_all": all(bracketed_axes),
            "sync_status": sync_res.status.value,
            "result_status": "WAIT",
            "fk_converged": None,
            "fk_residual_m": None,
            "fk_iterations": None,
            "branch_jump": False,
            "branch_jump_pos_m": None,
            "branch_jump_angle_rad": None,
            "compute_time_us": None,
            "synced_positions_m": None,
            "synced_raw_steps": None,
            "uart_cycle_sequence": self._sequence_last,
        }
        if sync_res.status != SyncStatus.SYNCED:
            self.records.append(record)
            return record

        record["synced_positions_m"] = list(sync_res.positions_meters)
        record["synced_raw_steps"] = list(sync_res.raw_steps)
        solve_start_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        try:
            fk_res = self.kinematics.forward_kinematics_with_metrics(
                sync_res.positions_meters,
                initial_guess=self._last_successful_pose,
                max_iter=25,
                tol=1e-10,
            )
        except (WorkspaceLimitError, SingularityError, KinematicsError, ValueError, ArithmeticError) as exc:
            record["result_status"] = "DROP"
            record["drop_reason"] = f"fk_exception:{type(exc).__name__}"
            record["compute_time_us"] = (
                time.clock_gettime_ns(time.CLOCK_MONOTONIC) - solve_start_ns
            ) / 1e3
            self.records.append(record)
            return record

        record["compute_time_us"] = (time.clock_gettime_ns(time.CLOCK_MONOTONIC) - solve_start_ns) / 1e3
        record["fk_converged"] = bool(fk_res.converged)
        record["fk_residual_m"] = float(fk_res.residual_norm)
        record["fk_iterations"] = int(fk_res.iterations)
        if not fk_res.converged or fk_res.residual_norm > self.max_residual_m:
            record["result_status"] = "DROP"
            record["drop_reason"] = "fk_not_converged_or_residual_limit"
            self.records.append(record)
            return record

        if self._last_successful_pose is not None and self._last_successful_R is not None:
            dx = fk_res.pose[0] - self._last_successful_pose[0]
            dy = fk_res.pose[1] - self._last_successful_pose[1]
            dz = fk_res.pose[2] - self._last_successful_pose[2]
            pos_jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            angle_jump = so3_geodesic_distance_rad(self._last_successful_R, fk_res.rotation_matrix)
            record["branch_jump_pos_m"] = pos_jump
            record["branch_jump_angle_rad"] = angle_jump
            if pos_jump > self.max_pos_jump_m or angle_jump > self.max_ang_jump_rad:
                record["result_status"] = "DROP"
                record["branch_jump"] = True
                record["drop_reason"] = "branch_continuity_gate"
                self.records.append(record)
                return record

        self._last_successful_pose = list(fk_res.pose)
        self._last_successful_R = [list(row) for row in fk_res.rotation_matrix]
        record["result_status"] = "FK_OK"
        self.records.append(record)
        return record

    @property
    def summary(self) -> dict[str, Any]:
        result_counts = {"WAIT": 0, "DROP": 0, "FK_OK": 0}
        sync_counts: dict[str, int] = {}
        branch_jumps = 0
        for record in self.records:
            result_counts[record["result_status"]] = result_counts.get(record["result_status"], 0) + 1
            sync_status = record["sync_status"]
            sync_counts[sync_status] = sync_counts.get(sync_status, 0) + 1
            branch_jumps += int(record["branch_jump"])
        return {
            "frame_count": self._frame_count,
            "query_count": len(self.records),
            "result_counts": result_counts,
            "sync_status_counts": sync_counts,
            "branch_jump_count": branch_jumps,
            "fk_compute_time_us": _percentiles(
                record["compute_time_us"] for record in self.records if record["compute_time_us"] is not None
            ),
            "uart": {
                "first_sequence": self._sequence_values[0] if self._sequence_values else None,
                "last_sequence": self._sequence_values[-1] if self._sequence_values else None,
                "duplicate_count": self._sequence_duplicate_count,
                "gap_count": self._sequence_gap_count,
                "out_of_order_count": self._sequence_out_of_order_count,
                "timestamp_violation_count": self._timestamp_violation_count,
                "fault_frame_count": self._fault_frame_count,
                "scan_span_us": _percentiles(self._scan_spans_us),
                "axis_rx_span_us": _percentiles(self._axis_spans_us),
            },
        }

    def report(self, input_mode: str) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "tool": "j5_c2_hw_acceptance",
            "input_mode": input_mode,
            "hardware_motion_executed": False,
            "hardware_write_count": 0,
            "config": {
                "path": str(self.config.path),
                "model_id": self.config.model_id,
                "status": self.config.status,
                "confirmation": self.config.confirmation,
                "imu_status": self.config.imu.get("status"),
            },
            "fixed_lag_delay_ms": self.fixed_lag_delay_ns / 1e6,
            "acceptance_gates": {
                "software_toolchain_ready": True,
                "final_geometry_confirmed": False,
                "six_axis_zero_mapping_confirmed": False,
                "home_park_confirmed": False,
                "q_actual_fk_residual_stable": False,
                "small_xyz_rpy_consistency": False,
                "ready_for_imu_fk_fusion": False,
            },
            "summary": self.summary,
            "records": self.records,
        }


def _synthetic_frames(runner: C2HwAcceptanceRunner, duration_s: float, feedback_hz: float, inject_jump: bool) -> list[TimedFeedbackFrame]:
    frames: list[TimedFeedbackFrame] = []
    t0 = 1_000_000_000_000
    dt_ns = int(1e9 / feedback_hz)
    for index in range(max(2, int(duration_s * feedback_hz))):
        start = t0 + index * dt_ns
        t = index / feedback_hz
        pose = [
            0.001 * math.sin(2.0 * math.pi * 0.20 * t),
            0.0008 * math.sin(2.0 * math.pi * 0.17 * t),
            runner.kinematics.nominal_height + 0.001 * math.sin(2.0 * math.pi * 0.50 * t),
            0.002 * math.sin(2.0 * math.pi * 0.13 * t),
            0.003 * math.sin(2.0 * math.pi * 0.19 * t),
            0.002 * math.sin(2.0 * math.pi * 0.11 * t),
        ]
        positions = runner.kinematics.inverse_kinematics(pose)
        if inject_jump and index == max(2, int(duration_s * feedback_hz)) // 2:
            positions = [value + 0.030 for value in positions]
        stamps = tuple(start + axis * 4_000_000 for axis in range(6))
        frames.append(
            TimedFeedbackFrame(
                uart_cycle_sequence=index,
                per_axis_rx_done_monotonic_ns=stamps,
                positions_meters=tuple(positions),
                raw_steps=tuple(value * runner.config.steps_per_meter for value in positions),
                snapshot_start_monotonic_ns=start,
                snapshot_end_monotonic_ns=stamps[-1],
                fault_codes=(0, 0, 0, 0, 0, 0),
            )
        )
    return frames


def _load_replay(path: Path) -> list[TimedFeedbackFrame]:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return []
    if text.startswith("["):
        values = json.loads(text)
        return [TimedFeedbackFrame.from_mapping(item) for item in values]
    return [TimedFeedbackFrame.from_mapping(json.loads(line)) for line in text.splitlines() if line.strip()]


def _run_frames(runner: C2HwAcceptanceRunner, frames: list[TimedFeedbackFrame], query_rate_hz: float, fixed_lag_ms: float) -> None:
    if len(frames) < 2:
        raise ValueError("at least two feedback frames are required")
    frames = sorted(frames, key=lambda frame: frame.snapshot_end_monotonic_ns)
    query_step_ns = int(1e9 / query_rate_hz)
    query_ns = frames[0].snapshot_start_monotonic_ns
    query_end_ns = frames[-1].snapshot_end_monotonic_ns
    feed_index = 0
    lead_ns = int(fixed_lag_ms * 1e6)
    while query_ns <= query_end_ns:
        while feed_index < len(frames) and frames[feed_index].snapshot_end_monotonic_ns <= query_ns + lead_ns:
            runner.ingest_timed_feedback(frames[feed_index])
            feed_index += 1
        runner.query(query_ns)
        query_ns += query_step_ns


def _write_reports(report: dict[str, Any], output_prefix: Path) -> tuple[Path, Path]:
    json_path = output_prefix.with_suffix(".json")
    csv_path = output_prefix.with_suffix(".csv")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    records = report["records"]
    fieldnames = sorted({key for record in records for key in record}) if records else ["query_timestamp_ns"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            row = dict(record)
            row["bracketed_axes"] = json.dumps(row.get("bracketed_axes"), ensure_ascii=False)
            row["synced_positions_m"] = json.dumps(row.get("synced_positions_m"), ensure_ascii=False)
            row["synced_raw_steps"] = json.dumps(row.get("synced_raw_steps"), ensure_ascii=False)
            writer.writerow(row)
    return json_path, csv_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mock/replay-only J5-C2-HW Fixed-Lag -> FK acceptance report")
    parser.add_argument("--input", type=Path, help="JSON array or JSONL TimedJointFeedback replay; no UART is opened")
    parser.add_argument("--config", type=Path, help="Explicit geometry YAML path")
    parser.add_argument("--output-prefix", type=Path, default=Path("/tmp/j5_c2_hw_acceptance"))
    parser.add_argument("--duration-s", type=float, default=3.0, help="Synthetic trajectory duration")
    parser.add_argument("--feedback-hz", type=float, default=30.0, help="Synthetic feedback rate")
    parser.add_argument("--query-rate-hz", type=float, default=200.0)
    parser.add_argument("--fixed-lag-ms", type=float, default=50.0)
    parser.add_argument("--inject-branch-jump", action="store_true")
    args = parser.parse_args(argv)

    config = load_geometry_config(args.config)
    runner = C2HwAcceptanceRunner(config=config, fixed_lag_delay_ms=args.fixed_lag_ms)
    if args.input:
        frames = _load_replay(args.input)
        input_mode = "replay_json_or_jsonl"
    else:
        frames = _synthetic_frames(runner, args.duration_s, args.feedback_hz, args.inject_branch_jump)
        input_mode = "synthetic_mock"
    _run_frames(runner, frames, args.query_rate_hz, args.fixed_lag_ms)
    report = runner.report(input_mode)
    json_path, csv_path = _write_reports(report, args.output_prefix)
    print(json.dumps({"json": str(json_path), "csv": str(csv_path), "summary": report["summary"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
