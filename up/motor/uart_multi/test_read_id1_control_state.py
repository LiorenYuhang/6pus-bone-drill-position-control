"""No-hardware tests for the read-only ID1 control-state diagnostic."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import ReadRegisterResponse, StatusFeedback  # noqa: E402
from tools import read_id1_control_state as tool  # noqa: E402


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
        "--duration": "0.2",
        "--rate": "10",
        "--timeout": "0.01",
        "--retries": "0",
        "--log": str(tmp_path / "control.log"),
    }
    values.update({key: str(value) for key, value in overrides.items()})
    argv = []
    for key, value in values.items():
        argv.extend((key, value))
    return tool.parse_args(argv)


def test_read_only_diagnostic_reports_config_changes_and_statistics(
    monkeypatch, tmp_path, capsys
):
    statuses = iter((_status(7, 0, -377), _status(7, 0, -376)))
    register_reads = {0x25: 0, 0x28: 0, 0x29: 0}

    class FakeMotor:
        def __init__(self, bus, motor_id):
            assert motor_id == 1

        def read_register(self, address):
            register_reads[address] += 1
            # The target changes on the second sampled config read, simulating
            # a post-run state change without sending any write.
            value = {
                0x25: 0,
                0x28: 65535,
                0x29: 8 if register_reads[address] <= 2 else 9,
            }[address]
            return ReadRegisterResponse(register_addr=address, values=(value,))

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
    assert "CONFIG_INITIAL mode=0 target_speed=65535 target_position=8" in output
    assert "CONFIG_CHANGED mode=NO target_speed=NO target_position=YES" in output
    assert "actual_position_steps mean=7.000000 min=7 max=7" in output
    assert "current_mA mean=0.000000 min=0 max=0" in output
    assert "force_protocol_g mean=-376.500000 min=-377 max=-376" in output
    assert len(FakeBus.instances) == 1
    assert FakeBus.instances[0].closed is True


def test_defaults_are_five_seconds_at_ten_hz():
    args = tool.parse_args(["--port", "/dev/ttyTEST"])
    assert args.duration == 5.0
    assert args.rate == 10.0


def test_existing_log_is_never_overwritten(tmp_path, capsys):
    log_path = tmp_path / "existing.log"
    log_path.write_text("keep", encoding="utf-8")
    args = _args(tmp_path, **{"--log": str(log_path)})
    assert tool.run(args) == 1
    assert "cannot create independent log" in capsys.readouterr().err
    assert log_path.read_text(encoding="utf-8") == "keep"


def test_source_has_no_write_pause_or_motion_calls():
    source = (ROOT / "tools" / "read_id1_control_state.py").read_text(encoding="utf-8")
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
