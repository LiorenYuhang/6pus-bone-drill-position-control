from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_motor_driver"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j6_phase2_axis_mapping import _write_report, run_phase2_axis_mapping  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import HardwareMode  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


def _controller(positions=None):
    bus = FakeBus()
    if positions is not None:
        bus.positions.update(dict(zip(range(1, 7), positions)))
        bus.targets.update(dict(zip(range(1, 7), positions)))
    return bus, SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)


def test_phase2_fake_feedback_mapping_and_zero_passes():
    bus, controller = _controller([10, 11, 12, 13, 14, 15])
    report = run_phase2_axis_mapping(
        controller,
        observed_mapping=[1, 2, 3, 4, 5, 6],
        bottom_reference_steps=[10, 11, 12, 13, 14, 15],
    )
    assert report["pass"] is True
    assert report["phase"] == "J6_PHASE2"
    assert report["snapshots"][0]["positions_steps"] == [10, 11, 12, 13, 14, 15]
    assert report["hardware_write_count"] == 0
    assert report["mock_write_count"] == 0
    assert bus.write_audit == []


def test_phase2_mapping_mismatch_fails_without_writes():
    bus, controller = _controller([50] * 6)
    report = run_phase2_axis_mapping(
        controller,
        observed_mapping=[1, 2, 3, 4, 6, 5],
        bottom_reference_steps=[50] * 6,
    )
    assert report["pass"] is False
    assert report["pass_conditions"]["physical_mapping_matches_expected"] is False
    assert report["hardware_write_count"] == 0
    assert bus.write_audit == []


def test_phase2_requires_measured_bottom_reference():
    _, controller = _controller([50] * 6)
    report = run_phase2_axis_mapping(
        controller,
        observed_mapping=[1, 2, 3, 4, 5, 6],
    )
    assert report["pass"] is False
    assert "measured bottom_reference_steps were not supplied" in report["failure_reasons"]


def test_phase2_fault_is_reported_as_failure():
    bus, controller = _controller([50] * 6)
    bus.faults[3] = 0x04
    report = run_phase2_axis_mapping(
        controller,
        observed_mapping=[1, 2, 3, 4, 5, 6],
        bottom_reference_steps=[50] * 6,
    )
    assert report["pass"] is False
    assert any("static feedback read failed" in item for item in report["failure_reasons"])
    assert report["hardware_write_count"] == 0


def test_phase2_report_refuses_overwrite(tmp_path):
    path = tmp_path / "phase2.json"
    _write_report({"pass": True}, path)
    with pytest.raises(FileExistsError):
        _write_report({"pass": True}, path)
    assert json.loads(path.read_text(encoding="utf-8"))["pass"] is True
