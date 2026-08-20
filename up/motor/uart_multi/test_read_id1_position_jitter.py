"""No-hardware tests for the ID1 position jitter diagnostic."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import StatusFeedback  # noqa: E402
from tools import read_id1_position_jitter as tool  # noqa: E402


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

    def __exit__(self, *args):
        self.closed = True
        return False


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _status(position, current, force, temperature=34, fault=0):
    return StatusFeedback(
        target_position=position,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=100,
        temperature=temperature,
        error_code=fault,
    )


def _args(tmp_path, **overrides):
    values = {
        "--port": "/dev/serial/by-id/usb-test",
        "--duration": "0.6",
        "--rate": "10",
        "--timeout": "0.01",
        "--retries": "0",
        "--log": str(tmp_path / "jitter.log"),
    }
    values.update({key: str(value) for key, value in overrides.items()})
    argv = []
    for key, value in values.items():
        argv.extend((key, value))
    return tool.parse_args(argv)


def test_jitter_summary_reports_histogram_runs_transitions_and_state_changes(
    monkeypatch, tmp_path, capsys
):
    statuses = iter(
        (
            _status(7, 0, -360),
            _status(7, 0, -360),
            _status(6, 1, -359),
            _status(6, 1, -359),
            _status(7, 0, -360),
            _status(8, 2, -358),
        )
    )

    class FakeMotor:
        def __init__(self, bus, motor_id):
            assert motor_id == 1

        def read_status(self):
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
    assert "samples=6" in output
    assert "position_histogram=6:2,7:3,8:1" in output
    assert "position_mode=7 mode_count=3 min=6 max=8 max_deviation_from_mode=1" in output
    assert "position_jump_count=3" in output
    assert "transitions=6->7:1,7->6:1,7->8:1" in output
    assert "p0_minus_1=6 max_consecutive=2" in output
    assert "p0_plus_1=8 max_consecutive=1" in output
    assert "run_lengths=6:n=1,max=2,mean=2.000;7:n=2,max=2,mean=1.500;8:n=1,max=1,mean=1.000" in output
    assert "position_jumps_with_other_state_change=3" in output
    assert "current_mA mean=0.666667 min=0 max=2" in output
    assert FakeBus.instances[0].closed is True


def test_defaults_are_thirty_seconds_at_fifty_hz():
    args = tool.parse_args(["--port", "/dev/ttyTEST"])
    assert args.duration == 30.0
    assert args.rate == 50.0


def test_existing_log_is_never_overwritten(tmp_path, capsys):
    log_path = tmp_path / "existing.log"
    log_path.write_text("keep", encoding="utf-8")
    args = _args(tmp_path, **{"--log": str(log_path)})
    assert tool.run(args) == 1
    assert "cannot create independent log" in capsys.readouterr().err
    assert log_path.read_text(encoding="utf-8") == "keep"


def test_source_has_no_write_or_motion_calls():
    source = (ROOT / "tools" / "read_id1_position_jitter.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    called = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called.isdisjoint({
        "write_register", "set_motor_id", "save_parameters", "set_position",
        "set_speed", "move", "pause",
    })


@pytest.mark.parametrize("argv", [
    ["--port", "/dev/tty", "--duration", "0"],
    ["--port", "/dev/tty", "--rate", "0"],
    ["--port", "/dev/tty", "--retries", "-1"],
])
def test_invalid_arguments_are_rejected(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
