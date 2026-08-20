"""Unit and mock tests for J3 6-motor large displacement test runner."""

from __future__ import annotations

import sys
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
from run_j3_large_displacement_test import run_large_displacement_test


def test_large_displacement_mock_execution():
    """Verify complete 1000 -> 1400 -> 1000 round trip execution in mock mode."""
    report = run_large_displacement_test(
        peak_speed=200.0,
        target_delta_steps=400,
        control_rate_hz=20.0,
        hold_time_s=0.01,
        settle_time_s=0.01,
        real_hardware=False,
        override_duration=0.1,
    )

    assert not report["safety_aborted"]
    assert report["q_start_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
    assert report["q_peak_actual"] == [1400, 1400, 1400, 1400, 1400, 1400]
    assert report["q_final_raw"] == [1000, 1000, 1000, 1000, 1000, 1000]
    assert report["q_final_relative"] == [0, 0, 0, 0, 0, 0]
    assert report["motor_reg_speed_0x28"] == 240
