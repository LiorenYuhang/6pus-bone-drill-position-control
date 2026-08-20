#!/usr/bin/env python3
"""Fail-closed J2-B2 single-axis position-tracking runner.

J2-B2 is deliberately separate from the J2-B1 real-hardware profile.  It
keeps the validated ID4 configuration block and three-frame write audit, but
uses the fixed +10-step tracking target and records time-based position
metrics.  Mock is the default; real hardware requires a stage-specific
tracking confirmation in addition to the normal motion gates.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
DRIVER_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
for import_root in (SCRIPT_ROOT, DRIVER_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from run_j2_b_bringup import J2BRunner, J2BStop  # noqa: E402
from pus6_motor_driver.safety import (  # noqa: E402
    DEFAULT_SPEED,
    HardwareMode,
    SPEED_MODE,
    STATIC_POSITION_TOLERANCE_STEPS,
    TargetValidationError,
)
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


MOTOR_ID = 4
DELTA_STEPS = 10
TIMEOUT_SEC = 3.0
POLL_PERIOD_SEC = 0.05
SETTLE_SAMPLES = 2


class J2B2TrackingRunner(J2BRunner):
    """J2-B2 profile with fixed tracking parameters and B2 metrics."""

    def __init__(
        self,
        controller: SixMotorController,
        *,
        log_file: str | Path | None = None,
        validated_config: bool = False,
    ) -> None:
        if controller.hardware_mode == HardwareMode.ACTIVE and not validated_config:
            raise TargetValidationError(
                "real J2-B2 requires explicit validated_config authorization"
            )
        super().__init__(
            controller=controller,
            motor_id=MOTOR_ID,
            delta_steps=DELTA_STEPS,
            timeout_sec=TIMEOUT_SEC,
            poll_period_sec=POLL_PERIOD_SEC,
            settle_samples=SETTLE_SAMPLES,
            allow_mock_configuration=False,
            validated_config=validated_config,
            log_file=log_file,
            stage_label="J2-B2",
            max_delta_steps=DELTA_STEPS,
            active_motion_parameters=(MOTOR_ID, DELTA_STEPS, TIMEOUT_SEC),
            force_mock_configuration=True,
            tracking_metrics=True,
        )

    BASELINE_SAMPLE_COUNT = 10
    BASELINE_SAMPLE_PERIOD_SEC = 0.005

    @staticmethod
    def _drift_reason(samples: list[int]) -> str | None:
        if len(samples) < 4:
            return None
        if all(b >= a for a, b in zip(samples, samples[1:])) and samples[-1] > samples[0]:
            return "monotonic_increasing start={} end={}".format(samples[0], samples[-1])
        if all(b <= a for a, b in zip(samples, samples[1:])) and samples[-1] < samples[0]:
            return "monotonic_decreasing start={} end={}".format(samples[0], samples[-1])
        midpoint = len(samples) // 2
        first_half = samples[:midpoint]
        second_half = samples[midpoint:]
        if min(second_half) > max(first_half) or max(second_half) < min(first_half):
            return "disjoint_halves first=[{},{}] second=[{},{}]".format(
                min(first_half), max(first_half), min(second_half), max(second_half)
            )
        return None

    def _record_baseline_analysis_and_stop(
        self, analysis: dict[str, object], reason: str
    ) -> None:
        analysis["selection_reason"] = reason
        self._baseline_analysis = analysis
        self._result["baseline_analysis"] = analysis
        raise J2BStop("ID4 baseline admission failed: {}".format(reason))

    def _baseline_stats(self, samples: list[int]) -> dict[str, object]:
        histogram = Counter(samples)
        mode_count = max(histogram.values())
        mode_candidates = sorted(value for value, count in histogram.items() if count == mode_count)
        minimum = min(samples)
        maximum = max(samples)
        return {
            "histogram": dict(sorted(histogram.items())),
            "mode": mode_candidates[0],
            "mode_candidates": mode_candidates,
            "min": minimum,
            "max": maximum,
            "span": maximum - minimum,
            "static_band": [minimum, maximum],
        }

    def _prepare_baseline(self, pre: object) -> tuple[int, dict[str, object]]:
        actual_samples: list[int] = []
        target_samples: list[int] = []
        for index in range(self.BASELINE_SAMPLE_COUNT):
            configuration = self.controller.read_single_axis_preflight(self.motor_id)
            actual = int(configuration["actual_position"])
            target = int(configuration["target_position"])
            if configuration["current_mA"] != 0:
                self._record_baseline_analysis_and_stop(
                    {"actual_samples": actual_samples + [actual], "target_samples": target_samples + [target]},
                    "sample current={}mA != 0".format(configuration["current_mA"]),
                )
            if configuration["fault"] != 0:
                self._record_baseline_analysis_and_stop(
                    {"actual_samples": actual_samples + [actual], "target_samples": target_samples + [target]},
                    "sample fault=0x{:02X}".format(configuration["fault"]),
                )
            actual_samples.append(actual)
            target_samples.append(target)
            if index + 1 < self.BASELINE_SAMPLE_COUNT:
                time.sleep(self.BASELINE_SAMPLE_PERIOD_SEC)

        actual_stats = self._baseline_stats(actual_samples)
        target_stats = self._baseline_stats(target_samples)
        reference = int(actual_stats["mode"])
        actual_drift = self._drift_reason(actual_samples)
        target_drift = self._drift_reason(target_samples)
        reference_band = (
            reference - STATIC_POSITION_TOLERANCE_STEPS,
            reference + STATIC_POSITION_TOLERANCE_STEPS,
        )
        actual_deviations = [abs(sample - reference) for sample in actual_samples]
        target_deviations = [abs(sample - reference) for sample in target_samples]
        actual_max_deviation = max(actual_deviations)
        target_max_deviation = max(target_deviations)
        analysis: dict[str, object] = {
            "preflight_actual_position": int(pre.positions_steps[self.motor_id - 1]),
            "actual_samples": actual_samples,
            "actual_sample_count": len(actual_samples),
            "actual_histogram": actual_stats["histogram"],
            "actual_histogram_mode": actual_stats["mode"],
            "actual_mode_candidates": actual_stats["mode_candidates"],
            "actual_min": actual_stats["min"],
            "actual_max": actual_stats["max"],
            "actual_span": actual_stats["span"],
            "actual_static_band": actual_stats["static_band"],
            "actual_monotonic_drift": actual_drift,
            "target_samples": target_samples,
            "target_sample_count": len(target_samples),
            "target_histogram": target_stats["histogram"],
            "target_histogram_mode": target_stats["mode"],
            "target_mode_candidates": target_stats["mode_candidates"],
            "target_min": target_stats["min"],
            "target_max": target_stats["max"],
            "target_span": target_stats["span"],
            "target_static_band": target_stats["static_band"],
            "target_monotonic_drift": target_drift,
            "reference": reference,
            "reference_band": list(reference_band),
            "static_position_tolerance_steps": STATIC_POSITION_TOLERANCE_STEPS,
            "actual_max_reference_deviation": actual_max_deviation,
            "target_max_reference_deviation": target_max_deviation,
            "monotonic_drift": {"actual": actual_drift, "target": target_drift},
            "samples": actual_samples,
            "sample_count": len(actual_samples),
            "histogram": actual_stats["histogram"],
            "histogram_mode": actual_stats["mode"],
            "mode_candidates": actual_stats["mode_candidates"],
            "min": actual_stats["min"],
            "max": actual_stats["max"],
            "span": actual_stats["span"],
            "static_band": actual_stats["static_band"],
        }
        self._baseline_band = reference_band
        self._target_band = reference_band
        if actual_max_deviation > STATIC_POSITION_TOLERANCE_STEPS:
            self._record_baseline_analysis_and_stop(
                analysis,
                "actual/reference deviation={} exceeds +/-{} steps".format(
                    actual_max_deviation, STATIC_POSITION_TOLERANCE_STEPS
                ),
            )
        if target_max_deviation > STATIC_POSITION_TOLERANCE_STEPS:
            self._record_baseline_analysis_and_stop(
                analysis,
                "0x29/reference deviation={} exceeds +/-{} steps".format(
                    target_max_deviation, STATIC_POSITION_TOLERANCE_STEPS
                ),
            )
        if actual_drift is not None:
            self._record_baseline_analysis_and_stop(analysis, "actual sustained one-direction drift: {}".format(actual_drift))
        if target_drift is not None:
            self._record_baseline_analysis_and_stop(analysis, "0x29 sustained one-direction drift: {}".format(target_drift))
        analysis["selection_reason"] = "actual_histogram_mode"
        self._baseline_analysis = analysis
        return reference, analysis

    def _configuration_precondition_safe(
        self, configuration: dict[str, int], baseline: int
    ) -> bool:
        return (
            abs(configuration["actual_position"] - baseline) <= STATIC_POSITION_TOLERANCE_STEPS
            and abs(configuration["target_position"] - baseline) <= STATIC_POSITION_TOLERANCE_STEPS
            and configuration["current_mA"] == 0
            and configuration["fault"] == 0
        )

    def _configuration_readback_safe(
        self, configuration: dict[str, int], baseline: int
    ) -> bool:
        return (
            configuration["control_mode"] == SPEED_MODE
            and configuration["target_speed"] == DEFAULT_SPEED
            and abs(configuration["target_position"] - baseline) <= STATIC_POSITION_TOLERANCE_STEPS
            and abs(configuration["actual_position"] - baseline) <= STATIC_POSITION_TOLERANCE_STEPS
            and configuration["current_mA"] == 0
            and configuration["fault"] == 0
        )


def _default_log_path(log_dir: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return log_dir / f"j2b2_tracking_motor{MOTOR_ID}_{stamp}.json"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J2-B2 position tracking (fail-closed)")
    parser.add_argument("--motor-id", type=int, default=MOTOR_ID)
    parser.add_argument("--delta-steps", type=int, default=DELTA_STEPS)
    parser.add_argument("--timeout-sec", type=float, default=TIMEOUT_SEC)
    parser.add_argument("--real-hardware", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--validated-config", action="store_true")
    parser.add_argument(
        "--confirm-tracking",
        action="store_true",
        help="Explicit authorization for the J2-B2 +10 tracking stage",
    )
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument("--log-dir", type=Path, default=WORKSPACE_ROOT / "log")
    args = parser.parse_args(argv)

    if args.motor_id != MOTOR_ID or args.delta_steps != DELTA_STEPS or args.timeout_sec != TIMEOUT_SEC:
        parser.error("J2-B2 parameters are fixed to motor_id=4, delta_steps=10, timeout-sec=3.0")
    if args.real_hardware and not args.confirm_motion:
        parser.error("--real-hardware requires --confirm-motion; no UART was opened")
    if args.confirm_motion and not args.real_hardware:
        parser.error("--confirm-motion is valid only with --real-hardware")
    if args.real_hardware and not args.validated_config:
        parser.error("real J2-B2 requires --validated-config; no UART was opened")
    if args.validated_config and not args.real_hardware:
        parser.error("--validated-config is valid only with --real-hardware")
    if args.real_hardware and not args.confirm_tracking:
        parser.error("real J2-B2 requires --confirm-tracking; no UART was opened")
    if args.confirm_tracking and not args.real_hardware:
        parser.error("--confirm-tracking is valid only with --real-hardware")

    mode = HardwareMode.ACTIVE if args.real_hardware else HardwareMode.MOCK
    log_file = args.log_file or _default_log_path(args.log_dir)
    controller = SixMotorController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        hardware_mode=mode,
    )
    result = J2B2TrackingRunner(
        controller,
        log_file=log_file,
        validated_config=args.validated_config,
    ).run()
    print(
        json.dumps(
            {
                "outcome": result["outcome"],
                "log_file": str(log_file),
                "tracking": result.get("tracking"),
                "write_audit": result["write_audit"],
            },
            indent=2,
        )
    )
    return 0 if result["outcome"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
