from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
PKG_ROOT = WORKSPACE_ROOT / "src" / "pus6_motor_driver"
for path in (SCRIPT_ROOT, PKG_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from check_j6_hardware_connection import (  # noqa: E402
    _write_report,
    run_connection_check,
)
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import HardwareMode  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


def test_connection_checker_fake_bus_passes_with_zero_writes():
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)

    report = run_connection_check(controller, sample_count=2, sample_interval_s=0.0)

    assert report["pass"] is True
    assert report["pass_conditions"]["six_axis_feedback_complete"] is True
    assert report["hardware_write_count"] == 0
    assert report["write_audit"] == []
    assert len(report["snapshots"]) == 2


def test_connection_checker_fault_fails_without_writes():
    bus = FakeBus()
    bus.faults[4] = 0x04
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)

    report = run_connection_check(controller)

    assert report["pass"] is False
    assert report["hardware_write_count"] == 0
    assert any("connection/read failure" in reason for reason in report["failure_reasons"])


def test_connection_checker_rejects_non_read_only_controller():
    controller = SixMotorController(bus=FakeBus(), hardware_mode=HardwareMode.MOCK)

    report = run_connection_check(controller)

    assert report["pass"] is False
    assert report["hardware_write_count"] == 0


def test_connection_checker_report_refuses_overwrite(tmp_path):
    output = tmp_path / "connection.json"
    report = {"pass": True}
    _write_report(report, output)

    assert json.loads(output.read_text(encoding="utf-8")) == report
    with pytest.raises(FileExistsError):
        _write_report(report, output)
