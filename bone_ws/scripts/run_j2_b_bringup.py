#!/usr/bin/env python3
"""Fail-closed J2-B single-axis bring-up runner.

The default execution mode is Mock. Real hardware requires both
``--real-hardware`` and ``--confirm-motion``. The runner owns the complete
single-axis lifecycle: six-axis preflight, one target write, feedback polling,
one target-axis pause, stop confirmation, postflight, and an immutable JSON log.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, is_dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
if str(DRIVER_ROOT) not in sys.path:
    sys.path.insert(0, str(DRIVER_ROOT))

from pus6_motor_driver.safety import (  # noqa: E402
    HardwareMode,
    DEFAULT_SPEED,
    MOTOR_IDS,
    ReadOnlyModeError,
    SafetyViolation,
    SPEED_MODE,
    TargetValidationError,
    validate_status_feedback,
)
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


class J2BStop(SafetyViolation):
    """A controlled J2-B stop requiring the normal pause confirmation path."""


PAUSE_TRIGGER_TOLERANCE_STEPS = 2
FINAL_ERROR_TOLERANCE_STEPS = 3


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


class J2BRunner:
    """Synchronous safety harness around the formal SixMotorController API."""

    def __init__(
        self,
        controller: SixMotorController,
        motor_id: int,
        delta_steps: int,
        timeout_sec: float = 2.0,
        poll_period_sec: float = 0.05,
        settle_samples: int = 2,
        allow_mock_configuration: bool = False,
        validated_config: bool = False,
        log_file: str | Path | None = None,
        stage_label: str = "J2-B",
        max_delta_steps: int = 3,
        active_motion_parameters: tuple[int, int, float] = (4, 3, 3.0),
        force_mock_configuration: bool = False,
        tracking_metrics: bool = False,
    ) -> None:
        if motor_id not in MOTOR_IDS:
            raise TargetValidationError(f"motor_id must be one of {MOTOR_IDS}, got {motor_id}")
        if allow_mock_configuration and motor_id != 4:
            raise TargetValidationError(
                "J2-B CFG-AUDIT configuration is restricted to target axis ID4"
            )
        if validated_config and motor_id != 4:
            raise TargetValidationError(
                "J2-B validated-config authorization is restricted to target axis ID4"
            )
        if (
            not isinstance(delta_steps, int)
            or isinstance(delta_steps, bool)
            or not 0 < abs(delta_steps) <= max_delta_steps
        ):
            raise TargetValidationError(
                f"delta_steps must be an integer with absolute value <= {max_delta_steps}"
            )
        if timeout_sec <= 0 or poll_period_sec <= 0 or settle_samples < 1:
            raise ValueError("timeout_sec and poll_period_sec must be positive; settle_samples must be >= 1")
        if controller.hardware_mode == HardwareMode.ACTIVE and (
            motor_id != active_motion_parameters[0]
            or delta_steps != active_motion_parameters[1]
            or timeout_sec != active_motion_parameters[2]
        ):
            raise TargetValidationError(
                f"real {stage_label} is fixed to motor_id={active_motion_parameters[0]}, "
                f"delta_steps={active_motion_parameters[1]}, "
                f"timeout_sec={active_motion_parameters[2]}"
            )
        if controller.hardware_mode == HardwareMode.ACTIVE and not validated_config:
            raise TargetValidationError(
                "real J2-B requires explicit validated_config authorization"
            )
        self.controller = controller
        self.motor_id = motor_id
        self.delta_steps = delta_steps
        self.timeout_sec = timeout_sec
        self.poll_period_sec = poll_period_sec
        self.settle_samples = settle_samples
        self.allow_mock_configuration = allow_mock_configuration
        self.validated_config = validated_config
        self.stage_label = stage_label
        self.force_mock_configuration = force_mock_configuration
        self.tracking_metrics_enabled = tracking_metrics
        self.log_file = Path(log_file) if log_file else None
        self.pause_attempted = False
        self.pause_confirmed = False
        self.command_sent = False
        self.configuration_attempted = False
        self.configuration_verified = False
        self._baseline_position: int | None = None
        self._baseline_band: tuple[int, int] | None = None
        self._target_band: tuple[int, int] | None = None
        self._baseline_analysis: dict[str, Any] | None = None
        self._target_position: int | None = None
        self._tracking_target_write_ns: int | None = None
        self._result: dict[str, Any] = {
            "stage": stage_label,
            "timestamp": datetime.now().isoformat(),
            "hardware_mode": controller.hardware_mode.value,
            "motor_id": motor_id,
            "delta_steps": delta_steps,
            "timeout_sec": timeout_sec,
            "poll_period_sec": poll_period_sec,
            "settle_samples_required": settle_samples,
            "outcome": "NOT_STARTED",
            "pre_flight": None,
            "baseline_analysis": None,
            "target_configuration": None,
            "configuration_write": None,
            "configuration_attempted": False,
            "configuration_verified": False,
            "arm": None,
            "command": None,
            "feedback_samples": [],
            "arrival": None,
            "pause": None,
            "stop_confirmation": None,
            "post_flight": None,
            "post_flight_error": None,
            "uart_telemetry": {},
            "write_audit": None,
            "error": None,
        }
        if tracking_metrics:
            self._result["tracking"] = None

    def _prepare_baseline(self, pre: Any) -> tuple[int, dict[str, Any] | None]:
        """Return the static reference used by the validated setup block.

        B1 keeps its historical exact-sample behavior. B2 overrides this hook
        with a short read-only baseline window and the formal +/-2-step
        reference-centered static deadband.
        """
        return pre.positions_steps[self.motor_id - 1], None

    def _configuration_precondition_safe(
        self, configuration: dict[str, int], baseline: int
    ) -> bool:
        return (
            configuration["target_position"] == baseline
            and configuration["actual_position"] == baseline
            and configuration["current_mA"] == 0
            and configuration["fault"] == 0
        )

    def _configuration_readback_safe(
        self, configuration: dict[str, int], baseline: int
    ) -> bool:
        return (
            configuration["control_mode"] == SPEED_MODE
            and configuration["target_speed"] == DEFAULT_SPEED
            and configuration["target_position"] == baseline
            and configuration["actual_position"] == baseline
            and configuration["current_mA"] == 0
            and configuration["fault"] == 0
        )

    def _snapshot(self) -> Any:
        try:
            return self.controller.read_state_snapshot()
        except SafetyViolation as exc:
            raise J2BStop(str(exc)) from exc

    @staticmethod
    def _validate_static_snapshot(snapshot: Any) -> None:
        """Require zero current only while the mechanism is meant to be static."""
        for mid, status in zip(MOTOR_IDS, snapshot.states):
            if status.error_code != 0:
                raise J2BStop(f"ID{mid} fault is 0x{status.error_code:02X}")
            if status.current_mA != 0:
                raise J2BStop(f"ID{mid} current is {status.current_mA}mA, expected 0mA")
            try:
                validate_status_feedback(status, mid, None)
            except SafetyViolation as exc:
                raise J2BStop(str(exc)) from exc

    @staticmethod
    def _validate_runtime_snapshot(snapshot: Any) -> None:
        """Apply the M6 runtime fault/current limit without requiring zero current."""
        for mid, status in zip(MOTOR_IDS, snapshot.states):
            if status.error_code != 0:
                raise J2BStop(f"ID{mid} fault is 0x{status.error_code:02X}")
            try:
                validate_status_feedback(status, mid, None)
            except SafetyViolation as exc:
                raise J2BStop(str(exc)) from exc

    def _write_audit(self) -> dict[str, Any]:
        bus = self.controller.bus
        tx_counts: dict[str, int] = {}
        write_frames: list[dict[str, Any]] = []
        if hasattr(bus, "tx_cmd_counts"):
            tx_counts = {f"0x{key:02X}": value for key, value in bus.tx_cmd_counts.items()}
        raw_audit = getattr(bus, "write_audit", None)
        if raw_audit is not None:
            write_frames = [
                {
                    "timestamp_ns": record.get("timestamp_ns"),
                    "timestamp_monotonic_ns": record.get("timestamp_monotonic_ns"),
                    "motor_id": int(record["motor_id"]),
                    "register_address": int(record["register_address"]),
                    "payload": [int(value) for value in record["payload"]],
                }
                for record in raw_audit
            ]
        elif hasattr(bus, "events"):
            # Compatibility fallback for pre-audit Mock buses only.
            write_frames = [
                {
                    "timestamp_ns": None,
                    "timestamp_monotonic_ns": None,
                    "motor_id": event[0],
                    "register_address": event[2],
                    "payload": list(event[3]),
                }
                for event in bus.events
                if event[1] == 0x32
            ]

        baseline = self._baseline_position
        target = self._target_position
        expected_write_frames: list[dict[str, Any]] = []
        if self.configuration_attempted:
            expected_write_frames.append(
                {
                    "motor_id": self.motor_id,
                    "register_address": 0x25,
                    "payload": [SPEED_MODE, 0, 0, DEFAULT_SPEED, baseline],
                }
            )
            expected_write_frames.extend(
                [
                    {
                        "motor_id": self.motor_id,
                        "register_address": 0x29,
                        "payload": [target],
                    },
                    {
                        "motor_id": self.motor_id,
                        "register_address": 0x1A,
                        "payload": [1],
                    },
                ]
            )
        else:
            expected_write_frames = [
                {
                    "motor_id": self.motor_id,
                    "register_address": 0x29,
                    "payload": [target],
                },
                {
                    "motor_id": self.motor_id,
                    "register_address": 0x1A,
                    "payload": [1],
                },
            ]

        actual_sequence = [
            (frame["motor_id"], frame["register_address"], frame["payload"])
            for frame in write_frames
        ]
        expected_sequence = [
            (frame["motor_id"], frame["register_address"], frame["payload"])
            for frame in expected_write_frames
        ]
        allowed_prefix = actual_sequence == expected_sequence[: len(actual_sequence)]
        sequence_complete = actual_sequence == expected_sequence
        per_axis_write_counts = {
            str(mid): sum(frame["motor_id"] == mid for frame in write_frames)
            for mid in MOTOR_IDS
        }
        observer_zero_writes = all(
            per_axis_write_counts[str(mid)] == 0 for mid in MOTOR_IDS if mid != self.motor_id
        )
        tx_0x32_count = tx_counts.get("0x32", 0)
        tx_count_matches_audit = tx_0x32_count == len(write_frames)

        if self.controller.hardware_mode == HardwareMode.READ_ONLY:
            audit_pass = (
                not write_frames
                and self.controller.write_count == 0
                and tx_0x32_count == 0
            )
        else:
            audit_pass = allowed_prefix and observer_zero_writes and tx_count_matches_audit
            if self._result["outcome"] == "PASS":
                audit_pass = (
                    audit_pass
                    and sequence_complete
                    and self.controller.write_count == len(expected_write_frames)
                )
        return {
            "controller_write_count": self.controller.write_count,
            "cmd_0x32_tx_count": len(write_frames),
            "tx_cmd_counts": tx_counts,
            "write_sequence": [
                (frame["motor_id"], frame["register_address"]) for frame in write_frames
            ],
            "write_frames": write_frames,
            "expected_write_frames": expected_write_frames,
            "per_axis_write_counts": per_axis_write_counts,
            "observer_zero_writes": observer_zero_writes,
            "sequence_complete": sequence_complete,
            "sequence_is_allowed_prefix": allowed_prefix,
            "tx_0x32_matches_audit": tx_count_matches_audit,
            "configuration_attempted": self.configuration_attempted,
            "configuration_verified": self.configuration_verified,
            "pass": audit_pass,
        }

    def _pause_and_confirm(self, baseline_target_position: int) -> None:
        pause_error: str | None = None
        try:
            self.pause_attempted = True
            status = self.controller.pause_single_axis(self.motor_id)
            self._update_tracking_peak_current(status.current_mA)
            self._result["pause"] = {
                "attempted": True,
                "motor_id": self.motor_id,
                "status": _json_value(status),
            }
        except Exception as exc:
            pause_error = str(exc)
            self._result["pause"] = {
                "attempted": True,
                "motor_id": self.motor_id,
                "success": False,
                "error": pause_error,
            }

        confirmations: list[dict[str, Any]] = []
        stable = False
        previous_position: int | None = None
        for _ in range(2):
            snapshot = self._snapshot()
            target_status = snapshot.states[self.motor_id - 1]
            record = {
                "timestamp_ns": snapshot.timestamp_end_ns,
                "actual_position": target_status.current_position,
                "current_mA": target_status.current_mA,
                "fault": target_status.error_code,
                "all_positions_steps": list(snapshot.positions_steps),
            }
            confirmations.append(record)
            if previous_position is not None and target_status.current_position == previous_position:
                stable = True
            previous_position = target_status.current_position
            time.sleep(self.poll_period_sec)

        self.pause_confirmed = (
            pause_error is None
            and stable
            and all(item["current_mA"] == 0 and item["fault"] == 0 for item in confirmations)
        )
        self._result["stop_confirmation"] = {
            "confirmed": self.pause_confirmed,
            "baseline_target_position": baseline_target_position,
            "samples": confirmations,
            "hard_stop_required": not self.pause_confirmed,
        }
        if not self.pause_confirmed:
            raise J2BStop("Pause/stop confirmation failed; HARD STOP REQUIRED")

    def _capture_post_flight(self) -> Any:
        post = self._snapshot()
        self._result["post_flight"] = _json_value(post)
        self._update_tracking_peak_current(post.states[self.motor_id - 1].current_mA)
        self._validate_static_snapshot(post)
        return post

    def _update_tracking_peak_current(self, current_mA: int) -> None:
        if self.tracking_metrics_enabled and self._result.get("tracking") is not None:
            tracking = self._result["tracking"]
            tracking["peak_current_mA"] = max(tracking["peak_current_mA"], current_mA)

    def _target_write_timestamp_ns(self) -> int | None:
        for record in reversed(getattr(self.controller.bus, "write_audit", [])):
            if (
                record.get("motor_id") == self.motor_id
                and record.get("register_address") == 0x29
            ):
                return record.get("timestamp_monotonic_ns")
        return None

    def _start_tracking_metrics(self, command: Any) -> None:
        if not self.tracking_metrics_enabled:
            return
        self._tracking_target_write_ns = self._target_write_timestamp_ns()
        if self._tracking_target_write_ns is None:
            raise J2BStop("Missing target write timestamp for J2-B2 tracking audit")
        command_current = getattr(command.response, "current_mA", 0)
        self._result["tracking"] = {
            "target_write_timestamp_monotonic_ns": self._tracking_target_write_ns,
            "first_positive_motion_latency_ms": None,
            "time_to_error_lte_3_ms": None,
            "time_to_error_lte_2_ms": None,
            "peak_current_mA": command_current,
            "final_displacement_steps": None,
            "final_target_error_steps": None,
            "position_vs_time_samples": [],
        }

    def _record_tracking_sample(self, snapshot: Any, status: Any, target: int) -> None:
        if not self.tracking_metrics_enabled or self._result.get("tracking") is None:
            return
        tracking = self._result["tracking"]
        elapsed_ms = (
            snapshot.timestamp_end_ns - self._tracking_target_write_ns
        ) / 1e6
        error_steps = status.current_position - target
        sample = {
            "elapsed_since_target_write_ms": elapsed_ms,
            "actual_position": status.current_position,
            "target_position": target,
            "displacement_from_baseline_steps": status.current_position - self._baseline_position,
            "error_steps": error_steps,
            "current_mA": status.current_mA,
            "fault": status.error_code,
        }
        tracking["position_vs_time_samples"].append(sample)
        self._update_tracking_peak_current(status.current_mA)
        if (
            tracking["first_positive_motion_latency_ms"] is None
            and status.current_position > self._baseline_position
        ):
            tracking["first_positive_motion_latency_ms"] = elapsed_ms
        if tracking["time_to_error_lte_3_ms"] is None and abs(error_steps) <= 3:
            tracking["time_to_error_lte_3_ms"] = elapsed_ms
        if tracking["time_to_error_lte_2_ms"] is None and abs(error_steps) <= 2:
            tracking["time_to_error_lte_2_ms"] = elapsed_ms

    def run(self) -> dict[str, Any]:
        target: int | None = None
        try:
            if self.controller.hardware_mode == HardwareMode.READ_ONLY:
                raise ReadOnlyModeError("J2-B requires MOCK or explicitly approved ACTIVE mode")
            self.controller.connect()
            limits = self.controller.read_limits()
            pre = self._snapshot()
            self._result["pre_flight"] = {"limits": limits, "snapshot": _json_value(pre)}
            self._validate_static_snapshot(pre)

            baseline, baseline_analysis = self._prepare_baseline(pre)
            self._baseline_position = baseline
            self._baseline_analysis = baseline_analysis
            self._result["baseline_analysis"] = _json_value(baseline_analysis)
            target = baseline + self.delta_steps
            self._target_position = target
            configuration = self.controller.read_single_axis_preflight(self.motor_id)
            self._result["target_configuration"] = configuration
            configuration_ok = (
                configuration["control_mode"] == SPEED_MODE
                and configuration["target_speed"] == DEFAULT_SPEED
                and self._configuration_precondition_safe(configuration, baseline)
            )
            mock_authorized = (
                (self.allow_mock_configuration or self.force_mock_configuration)
                and self.controller.hardware_mode == HardwareMode.MOCK
            )
            real_authorized = (
                self.validated_config
                and self.controller.hardware_mode == HardwareMode.ACTIVE
            )
            configuration_requested = mock_authorized or real_authorized
            if not configuration_requested and not configuration_ok:
                raise J2BStop(
                    "Target-axis positioning configuration rejected before 0x29 write: "
                    f"{configuration}"
                )
            if configuration_requested:
                if not self._configuration_precondition_safe(configuration, baseline):
                    raise J2BStop(
                        "Target-axis baseline is unsafe for validated configuration before 0x29 write: "
                        f"{configuration}"
                    )
                self.configuration_attempted = True
                self._result["configuration_attempted"] = True
                try:
                    self._result["configuration_write"] = _json_value(
                        self.controller.configure_single_axis_for_bringup(
                            self.motor_id, baseline, baseline_band=self._baseline_band, target_band=self._target_band
                        )
                    )
                except Exception as exc:
                    raise J2BStop(
                        "Validated target-axis configuration failed before 0x29 write; "
                        f"no retry and no target write: {exc}"
                    ) from exc
                configuration = self.controller.read_single_axis_preflight(self.motor_id)
                self._result["target_configuration"] = configuration
                if not self._configuration_readback_safe(configuration, baseline):
                    raise J2BStop(
                        "Target-axis configuration readback rejected before 0x29 write: "
                        f"{configuration}"
                    )
                self.configuration_verified = True
                self._result["configuration_verified"] = True
            self._result["arm"] = {
                "motor_id": self.motor_id,
                "baseline_position": baseline,
                "target_step": target,
                "observer_ids": [mid for mid in MOTOR_IDS if mid != self.motor_id],
            }
            command = self.controller.command_single_axis_with_metrics(self.motor_id, target)
            self.command_sent = True
            self._result["command"] = _json_value(command)
            self._start_tracking_metrics(command)

            deadline = time.monotonic() + self.timeout_sec
            consecutive_arrivals = 0
            arrived = False
            while time.monotonic() < deadline:
                snapshot = self._snapshot()
                self._validate_runtime_snapshot(snapshot)
                status = snapshot.states[self.motor_id - 1]
                sample = {
                    "timestamp_ns": snapshot.timestamp_end_ns,
                    "positions_steps": list(snapshot.positions_steps),
                    "target_actual": status.current_position,
                    "target_current_mA": status.current_mA,
                    "target_fault": status.error_code,
                    "error_steps": status.current_position - target,
                }
                self._result["feedback_samples"].append(sample)
                self._record_tracking_sample(snapshot, status, target)
                if abs(sample["error_steps"]) <= PAUSE_TRIGGER_TOLERANCE_STEPS:
                    consecutive_arrivals += 1
                    if consecutive_arrivals >= self.settle_samples:
                        arrived = True
                        break
                else:
                    consecutive_arrivals = 0
                time.sleep(self.poll_period_sec)

            self._result["arrival"] = {
                "arrived": arrived,
                "target_step": target,
                "last_error_steps": self._result["feedback_samples"][-1]["error_steps"]
                if self._result["feedback_samples"]
                else None,
            }
            if not arrived:
                raise J2BStop(f"Arrival timeout after {self.timeout_sec:.3f}s")

            self._pause_and_confirm(target)
            post = self._capture_post_flight()
            final_error = post.positions_steps[self.motor_id - 1] - target
            self._result["final_error_steps"] = final_error
            if self.tracking_metrics_enabled and self._result.get("tracking") is not None:
                self._result["tracking"]["final_displacement_steps"] = (
                    post.positions_steps[self.motor_id - 1] - self._baseline_position
                )
                self._result["tracking"]["final_target_error_steps"] = final_error
            if abs(final_error) > FINAL_ERROR_TOLERANCE_STEPS:
                raise J2BStop(
                    f"Post-flight error {final_error} step exceeds "
                    f"{FINAL_ERROR_TOLERANCE_STEPS}-step limit"
                )
            self._result["outcome"] = "PASS"
        except Exception as exc:
            if self.command_sent and not self.pause_attempted and target is not None:
                try:
                    self._pause_and_confirm(target)
                except Exception as pause_exc:
                    self._result["error"] = f"{exc}; {pause_exc}"
            if (
                self.command_sent
                and self.pause_attempted
                and self._result["post_flight"] is None
            ):
                try:
                    self._capture_post_flight()
                except Exception as post_exc:
                    self._result["post_flight_error"] = str(post_exc)
            if self._result["error"] is None:
                self._result["error"] = str(exc)
            self._result["outcome"] = (
                "STOP"
                if isinstance(exc, SafetyViolation) and not isinstance(exc, ReadOnlyModeError)
                else "FAIL"
            )
        finally:
            try:
                self._result["uart_telemetry"] = self.controller.bus.get_telemetry()
            except Exception:
                self._result["uart_telemetry"] = {}
            try:
                self._result["write_audit"] = self._write_audit()
            except Exception as exc:
                self._result["write_audit"] = {"pass": False, "error": str(exc)}
            self.controller.close()
            self._persist()
        return self._result

    def _persist(self) -> None:
        if self.log_file is None:
            return
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        if self.log_file.exists():
            raise FileExistsError(f"Refusing to overwrite existing J2-B log: {self.log_file}")
        self.log_file.write_text(
            json.dumps(_json_value(self._result), indent=2, ensure_ascii=False), encoding="utf-8"
        )


def _default_log_path(log_dir: Path, motor_id: int) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return log_dir / f"j2b_motor{motor_id}_{stamp}.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J2-B single-axis bring-up (fail-closed)")
    parser.add_argument("--motor-id", type=int, default=4)
    parser.add_argument("--delta-steps", type=int, default=3)
    parser.add_argument("--real-hardware", action="store_true", help="Explicitly open the real UART")
    parser.add_argument("--confirm-motion", action="store_true", help="Second explicit gate for real motion")
    parser.add_argument(
        "--validated-config",
        action="store_true",
        help="Explicit authorization for the single ID4 validated setup block",
    )
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--poll-period-sec", type=float, default=0.05)
    parser.add_argument("--settle-samples", type=int, default=2)
    parser.add_argument(
        "--mock-configure-target",
        action="store_true",
        help="Apply the validated 0x25..0x29 setup block in Mock only",
    )
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument("--log-dir", type=Path, default=WORKSPACE_ROOT / "log")
    args = parser.parse_args(argv)

    if args.real_hardware and not args.confirm_motion:
        parser.error("--real-hardware requires --confirm-motion; no UART was opened")
    if args.confirm_motion and not args.real_hardware:
        parser.error("--confirm-motion is valid only with --real-hardware")
    if args.real_hardware and not args.validated_config:
        parser.error("real J2-B requires --validated-config; no UART was opened")
    if args.validated_config and not (args.real_hardware and args.confirm_motion):
        parser.error("--validated-config requires --real-hardware and --confirm-motion")
    if args.mock_configure_target and args.real_hardware:
        parser.error("--mock-configure-target cannot be used with --real-hardware")
    if args.mock_configure_target and args.motor_id != 4:
        parser.error("--mock-configure-target is restricted to --motor-id 4")
    if args.validated_config and args.motor_id != 4:
        parser.error("--validated-config is restricted to --motor-id 4")
    if args.real_hardware and (
        args.motor_id != 4 or args.delta_steps != 3 or args.timeout_sec != 3.0
    ):
        parser.error(
            "real J2-B parameters are fixed to motor_id=4, delta_steps=3, timeout-sec=3.0"
        )
    if args.mock_configure_target and args.validated_config:
        parser.error("--mock-configure-target and --validated-config are mutually exclusive")

    mode = HardwareMode.ACTIVE if args.real_hardware else HardwareMode.MOCK
    log_file = args.log_file or _default_log_path(args.log_dir, args.motor_id)
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        hardware_mode=mode,
    )
    result = J2BRunner(
        controller=controller,
        motor_id=args.motor_id,
        delta_steps=args.delta_steps,
        timeout_sec=args.timeout_sec,
        poll_period_sec=args.poll_period_sec,
        settle_samples=args.settle_samples,
        allow_mock_configuration=args.mock_configure_target,
        validated_config=args.validated_config,
        log_file=log_file,
    ).run()
    print(json.dumps({"outcome": result["outcome"], "log_file": str(log_file), "write_audit": result["write_audit"]}, indent=2))
    return 0 if result["outcome"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
