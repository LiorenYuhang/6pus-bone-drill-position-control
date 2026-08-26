from __future__ import annotations

import sys
from pathlib import Path

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
PKG_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
for path in (SCRIPT_ROOT, PKG_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase3_single_axis_micro_jog import (  # noqa: E402
    run_micro_jog,
)
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import HardwareMode, TargetValidationError  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


@pytest.mark.parametrize("axis, displacement", [(1, 20), (3, -20), (6, 40)])
def test_micro_jog_mock_moves_one_axis_and_returns(axis, displacement):
    bus = FakeBus()
    bus.positions[axis] = 100
    bus.targets[axis] = 100
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK, retries=0)

    report = run_micro_jog(
        controller,
        axis=axis,
        displacement_steps=displacement,
        timeout_s=0.1,
        poll_period_s=0.001,
    )

    assert report["outcome"] == "PASS"
    assert report["software_pass"] is True
    assert report["baseline_positions_steps"][axis - 1] == 100
    assert report["observer_zero_writes"] is True
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] == 5
    assert report["per_axis_write_counts"][str(axis)] == 5
    assert all(
        record["motor_id"] == axis for record in report["write_audit"]
    )


def test_micro_jog_rejects_out_of_range_target_before_motion_writes():
    bus = FakeBus()
    bus.positions[2] = 0
    bus.targets[2] = 0
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)

    report = run_micro_jog(
        controller,
        axis=2,
        displacement_steps=-20,
        timeout_s=0.1,
        poll_period_s=0.001,
    )

    assert report["outcome"] == "FAIL"
    assert report["controller_write_count"] == 0
    assert report["write_audit"] == []


def test_micro_jog_rejects_invalid_displacement_without_bus_access():
    controller = SixMotorController(bus=FakeBus(), hardware_mode=HardwareMode.MOCK)

    with pytest.raises(TargetValidationError):
        run_micro_jog(controller, axis=1, displacement_steps=10)

    assert controller.bus.events == []


def test_active_micro_jog_requires_explicit_authorization():
    controller = SixMotorController(bus=FakeBus(), hardware_mode=HardwareMode.ACTIVE)

    with pytest.raises(TargetValidationError, match="requires real-hardware"):
        run_micro_jog(controller, axis=1, displacement_steps=20)

    assert controller.bus.events == []


def test_read_only_micro_jog_is_rejected_before_bus_access():
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)

    with pytest.raises(TargetValidationError, match="requires MOCK"):
        run_micro_jog(controller, axis=1, displacement_steps=20)

    assert bus.events == []
