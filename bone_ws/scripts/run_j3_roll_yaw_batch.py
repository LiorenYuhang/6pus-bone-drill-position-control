#!/usr/bin/env python3
"""Batch runner for J3 Assembled Roll & Yaw Bidirectional Tests."""

from __future__ import annotations

import json
import logging
import sys
import time
from pathlib import Path

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

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("j3_roll_yaw_batch")

TEST_CASES = [
    ("roll_pos", "Roll +0.05rad (Alpha+)"),
    ("roll_neg", "Roll -0.05rad (Alpha-)"),
    ("yaw_pos", "Yaw +0.05rad (Gamma+)"),
    ("yaw_neg", "Yaw -0.05rad (Gamma-)"),
]


def main() -> None:
    results = []
    overall_pass = True

    for test_name, desc in TEST_CASES:
        logger.info("=" * 60)
        logger.info(f"EXECUTING: {desc}")
        logger.info("=" * 60)

        report = run_cartesian_test_session(
            test_name=test_name,
            peak_speed=200.0,
            control_rate_hz=10.0,
            hold_time_s=1.0,
            settle_time_s=2.0,
            cycles=1,
            real_hardware=True,
            confirm_motion=True,
            validated_config=True,
        )
        results.append(report)

        if report.get("safety_aborted", False):
            logger.error(f"🛑 Safety stop triggered on {test_name}: {report.get('abort_reason')}")
            overall_pass = False
            break

        time.sleep(1.0)

    log_file = WORKSPACE_ROOT / "log" / "j3_assembled_roll_yaw_summary.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump({"overall_pass": overall_pass, "results": results}, f, indent=2)
    logger.info(f"Batch completed with pass={overall_pass}. Summary saved to: {log_file}")

    if not overall_pass:
        sys.exit(1)


if __name__ == "__main__":
    main()
