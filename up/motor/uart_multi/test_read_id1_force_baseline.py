"""No-hardware tests for the read-only ID1 force baseline sampler."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import StatusFeedback  # noqa: E402
from tools import read_id1_force_baseline as tool  # noqa: E402


class FakeBus:
    instances = []

    def __init__(self, port, *, timeout, retries):
        self.port = port
        self.timeout = timeout
        self.retries = retries
        self.closed = False
        type(self).instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.closed = True
        return False


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _args(tmp_path, **overrides):
    values = {
        "--port": "/dev/serial/by-id/usb-test",
        "--duration": "0.04",
        "--rate": "50",
        "--timeout": "0.01",
        "--retries": "0",
        "--label": "horizontal",
        "--log": str(tmp_path / "force.log"),
    }
    values.update({key: str(value) for key, value in overrides.items()})
    argv = []
    for key, value in values.items():
        argv.extend((key, value))
    return tool.parse_args(argv)


def _status(position, current, force, temperature=25, fault=0):
    return StatusFeedback(
        target_position=position,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=123,
        temperature=temperature,
        error_code=fault,
    )


def test_sampler_is_read_only_and_reports_statistics(monkeypatch, tmp_path, capsys):
    statuses = iter((_status(100, 10, -411), _status(102, 20, -409)))
    calls = []

    class FakeMotor:
        def __init__(self, bus, motor_id):
            assert motor_id == 1

        def read_status(self):
            calls.append("read_status")
            return next(statuses)

    clock = FakeClock()
    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", FakeMotor)
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)

    assert tool.run(_args(tmp_path)) == 0
    output = capsys.readouterr().out
    assert "samples=2" in output
    assert "force_protocol_g mean=-410.000000 std=1.000000 min=-411 max=-409 peak_to_peak=2" in output
    assert "current_mA mean=15.000000 max=20" in output
    assert "position_steps mean=101.000000 min=100 max=102" in output
    assert calls == ["read_status", "read_status"]
    assert len(FakeBus.instances) == 1
    assert FakeBus.instances[0].retries == 0
    assert FakeBus.instances[0].closed is True


def test_default_sampling_is_10_seconds_at_50_hz():
    args = tool.parse_args(["--port", "/dev/ttyTEST"])
    assert args.duration == 10.0
    assert args.rate == 50.0
    assert round(args.duration * args.rate) == 500


def test_existing_log_is_never_overwritten(tmp_path, capsys):
    log_path = tmp_path / "existing.log"
    log_path.write_text("keep", encoding="utf-8")
    args = _args(tmp_path, **{"--log": str(log_path)})
    assert tool.run(args) == 1
    assert "cannot create independent log" in capsys.readouterr().err
    assert log_path.read_text(encoding="utf-8") == "keep"


def test_source_has_no_write_or_motion_calls():
    source = (ROOT / "tools" / "read_id1_force_baseline.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint({
        "write_register", "set_motor_id", "save_parameters", "read_register",
        "set_position", "set_speed", "move", "trajectory",
    })


@pytest.mark.parametrize("argv", [
    ["--port", "/dev/tty", "--duration", "0"],
    ["--port", "/dev/tty", "--rate", "0"],
    ["--port", "/dev/tty", "--label", "../unsafe"],
])
def test_invalid_sampling_arguments_are_rejected(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
