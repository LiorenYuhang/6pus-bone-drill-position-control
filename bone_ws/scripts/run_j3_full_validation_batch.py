#!/usr/bin/env python3
"""Orchestrator for J3 PRE-ASSEMBLY FULL MOTION VALIDATION Batch (M1 ~ M16).

Executes all 16 predefined Cartesian IK motions, cyclic repeatability,
and dual-pose transitions in sequential order.

Halts immediately on any true Safety STOP without executing subsequent cases.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

# Ensure workspace source packages are on path
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)
SCRIPTS_PATH = str(WORKSPACE_ROOT / "scripts")
if SCRIPTS_PATH not in sys.path:
    sys.path.insert(0, SCRIPTS_PATH)

from run_j3_cartesian_motion import run_cartesian_test_session
from run_j3_startup_homing import run_homing_session

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("j3_full_batch")


TEST_SCHEDULE = [
    {"id": "M1", "type": "cartesian", "test_name": "heave_pos", "cycles": 1, "desc": "Heave +10mm (Z+)"},
    {"id": "M2", "type": "cartesian", "test_name": "heave_neg", "cycles": 1, "desc": "Heave -10mm (Z-)"},
    {"id": "M7", "type": "cartesian", "test_name": "pitch_pos", "cycles": 1, "desc": "Pitch +0.05rad (Beta+)"},
    {"id": "M8", "type": "cartesian", "test_name": "pitch_neg", "cycles": 1, "desc": "Pitch -0.05rad (Beta-)"},
    {"id": "M9", "type": "cartesian", "test_name": "roll_pos", "cycles": 1, "desc": "Roll +0.05rad (Alpha+)"},
    {"id": "M10", "type": "cartesian", "test_name": "roll_neg", "cycles": 1, "desc": "Roll -0.05rad (Alpha-)"},
    {"id": "M11", "type": "cartesian", "test_name": "yaw_pos", "cycles": 1, "desc": "Yaw +0.05rad (Gamma+)"},
    {"id": "M12", "type": "cartesian", "test_name": "yaw_neg", "cycles": 1, "desc": "Yaw -0.05rad (Gamma-)"},
    {"id": "M3", "type": "cartesian", "test_name": "surge_pos", "cycles": 1, "desc": "Surge +5mm (X+)"},
    {"id": "M4", "type": "cartesian", "test_name": "surge_neg", "cycles": 1, "desc": "Surge -5mm (X-)"},
    {"id": "M5", "type": "cartesian", "test_name": "sway_pos", "cycles": 1, "desc": "Sway +5mm (Y+)"},
    {"id": "M6", "type": "cartesian", "test_name": "sway_neg", "cycles": 1, "desc": "Sway -5mm (Y-)"},
    {"id": "M13", "type": "cartesian", "test_name": "combined_6dof", "cycles": 1, "desc": "6-DoF Combined Pose"},
    {"id": "M14", "type": "cartesian", "test_name": "heave_pos", "cycles": 3, "desc": "3-Cycle Repeatability (Heave +10mm)"},
    {"id": "M15", "type": "homing", "action": "park", "desc": "OPERATING_HOME -> STORAGE_PARK ([1000]->[0])"},
    {"id": "M16", "type": "homing", "action": "home", "desc": "STORAGE_PARK -> OPERATING_HOME ([0]->[1000])"},
]


def run_full_validation_batch(
    peak_speed: float = 200.0,
    control_rate_hz: float = 10.0,
    hold_time_s: float = 1.0,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    override_duration: float | None = None,
) -> dict[str, Any]:
    """Execute all tests in sequence, recording results and halting on Safety STOP."""
    logger.info("=" * 70)
    logger.info("STARTING J3 PRE-ASSEMBLY FULL MOTION VALIDATION BATCH (M1 ~ M16)")
    logger.info("=" * 70)

    batch_start_time = time.time()
    results: list[dict[str, Any]] = []
    overall_pass = True
    aborted_step = None
    abort_reason = None

    for idx, item in enumerate(TEST_SCHEDULE, 1):
        item_id = item["id"]
        item_desc = item["desc"]
        logger.info(f"\n[{idx}/{len(TEST_SCHEDULE)}] Executing {item_id}: {item_desc}")

        if item["type"] == "cartesian":
            report = run_cartesian_test_session(
                test_name=item["test_name"],
                peak_speed=peak_speed,
                control_rate_hz=control_rate_hz,
                hold_time_s=hold_time_s,
                cycles=item["cycles"],
                real_hardware=real_hardware,
                confirm_motion=confirm_motion,
                validated_config=validated_config,
                override_duration=override_duration,
            )
        elif item["type"] == "homing":
            report = run_homing_session(
                action=item["action"],
                speed=int(peak_speed),
                control_rate_hz=control_rate_hz,
                real_hardware=real_hardware,
                confirm_motion=confirm_motion,
                validated_config=validated_config,
                duration=override_duration,
            )
        else:
            raise ValueError(f"Unknown test type: {item['type']}")

        # Attach item metadata
        report["schedule_id"] = item_id
        report["schedule_desc"] = item_desc
        results.append(report)

        if report.get("safety_aborted", False):
            logger.error(f"🛑 SAFETY STOP triggered on {item_id}: {report.get('abort_reason')}")
            overall_pass = False
            aborted_step = item_id
            abort_reason = report.get("abort_reason")
            break

        # Brief pause between test items
        time.sleep(1.0)

    total_batch_duration = time.time() - batch_start_time
    logger.info("\n" + "=" * 70)
    logger.info("J3 PRE-ASSEMBLY FULL MOTION VALIDATION BATCH FINISHED")
    logger.info(f"Total Execution Time: {total_batch_duration:.2f} s")
    logger.info(f"Overall Result:       {'ALL PASS' if overall_pass else f'ABORTED AT {aborted_step}'}")
    logger.info("=" * 70)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary = {
        "timestamp": timestamp_str,
        "overall_pass": overall_pass,
        "aborted_step": aborted_step,
        "abort_reason": abort_reason,
        "total_batch_duration_s": total_batch_duration,
        "peak_speed_step_s": peak_speed,
        "items_count": len(results),
        "results": results,
    }

    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    summary_file = log_dir / f"j3_full_validation_summary_{timestamp_str}.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    logger.info(f"Batch summary saved to: {summary_file}")

    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="J3 Full Pre-Assembly Motion Batch Runner")
    parser.add_argument("--peak-speed", type=float, default=200.0, help="Trajectory peak speed in step/s (default: 200.0)")
    parser.add_argument("--control-rate", type=float, default=10.0, help="Control rate in Hz (default: 10.0)")
    parser.add_argument("--hold-time", type=float, default=1.0, help="Hold time at peak in seconds (default: 1.0)")
    parser.add_argument("--real-hardware", action="store_true", help="Enable real serial bus hardware connection")
    parser.add_argument("--confirm-motion", action="store_true", help="Explicit human confirmation for physical movement")
    parser.add_argument("--validated-config", action="store_true", help="Explicit human confirmation for validated parameters")

    args = parser.parse_args()
    summary = run_full_validation_batch(
        peak_speed=args.peak_speed,
        control_rate_hz=args.control_rate,
        hold_time_s=args.hold_time,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
    )

    if not summary.get("overall_pass", False):
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
