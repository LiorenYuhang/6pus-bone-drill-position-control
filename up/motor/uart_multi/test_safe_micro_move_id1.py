"""No-hardware tests for the fixed ID1 safety motion sequence."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import StatusFeedback  # noqa: E402
from tools import safe_micro_move_id1 as tool  # noqa: E402
from uart_bus import RetryExhaustedError, NoResponseError  # noqa: E402


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


def _args(tmp_path, **overrides):
    values = {
        "--port": "/dev/serial/by-id/usb-test",
        "--max-current-ma": "500",
        "--max-force-g": "100",
        "--log": str(tmp_path / "micro.log"),
    }
    for key, value in overrides.items():
        values[key] = str(value)
    argv = []
    for key, value in values.items():
        argv.extend((key, value))
    return tool.parse_args(argv)


def _status(position, *, current=10, force=2, temperature=25, fault=0, target=None):
    return StatusFeedback(
        target_position=position if target is None else target,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=0,
        temperature=temperature,
        error_code=fault,
    )


def _register_value(register_addr, *, target=1000, mode=0, speed=100):
    values = {
        tool.LOWER_LIMIT_ADDR: 0,
        tool.UPPER_LIMIT_ADDR: 2000,
        tool.CONTROL_MODE_ADDR: mode,
        tool.TARGET_SPEED_ADDR: speed,
        tool.TARGET_POSITION_ADDR: target,
    }
    return values[register_addr]


def test_official_motion_and_limit_register_addresses_are_frozen():
    assert tool.UPPER_LIMIT_ADDR == 0x23
    assert tool.LOWER_LIMIT_ADDR == 0x24
    assert tool.CONTROL_MODE_ADDR == 0x25
    assert tool.TARGET_SPEED_ADDR == 0x28
    assert tool.TARGET_POSITION_ADDR == 0x29
    assert tool.SOFTWARE_LOWER_LIMIT == 200
    assert tool.SOFTWARE_UPPER_LIMIT == 1800


def test_fixed_sequence_uses_one_bus_and_official_registers(monkeypatch, tmp_path, capsys):
    writes = []

    class FakeMotor:
        def __init__(self, bus, motor_id):
            assert motor_id == 1
            self.target = 1000

        def read_register(self, register_addr):
            value = _register_value(
                register_addr,
                target=getattr(self, "target", 1000),
                mode=getattr(self, "mode", 0),
                speed=getattr(self, "speed", 100),
            )
            return type("Response", (), {"values": [value]})()

        def read_status(self):
            return _status(self.target, target=self.target)

        def write_register(self, register_addr, data):
            writes.append((register_addr, list(data)))
            if register_addr == tool.TARGET_POSITION_ADDR:
                self.target = data[0]
                return _status(self.target, target=self.target)
            if register_addr == tool.CONTROL_MODE_ADDR:
                assert data == [2, 0, 0, 50, 1000]
                self.mode = data[0]
                self.speed = data[3]
                self.target = data[-1]
                return _status(self.target, target=self.target)
            raise AssertionError(f"unexpected write 0x{register_addr:02X}")

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", FakeMotor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "MOVE ID1")

    assert tool.run(_args(tmp_path)) == 0
    output = capsys.readouterr().out
    assert "CONFIG BEFORE mode=0 speed=100 target=1000 device_limits=[0, 2000]" in output
    assert "CONFIG FINAL mode=2 speed=50 target=1000 restored=false" in output
    assert "PASS ID1 p0=1000" in output
    assert writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 1000]),
        (tool.TARGET_POSITION_ADDR, [1005]),
        (tool.TARGET_POSITION_ADDR, [1000]),
        (tool.TARGET_POSITION_ADDR, [995]),
        (tool.TARGET_POSITION_ADDR, [1000]),
    ]
    assert len(FakeBus.instances) == 1
    assert FakeBus.instances[0].retries == 2
    assert FakeBus.instances[0].closed is True


def test_force_limit_uses_absolute_value(tmp_path):
    args = _args(tmp_path)
    with pytest.raises(tool.SafetyViolation, match="force=-101g exceeds 100g"):
        tool._check_status(_status(100, force=-101), args)


def test_position_limit_violation_stops_before_motion_write(monkeypatch, tmp_path, capsys):
    writes = []

    class LimitMotor:
        def __init__(self, bus, motor_id):
            self.status = _status(200)

        def read_register(self, register_addr):
            value = _register_value(
                register_addr,
                target=getattr(self, "target", 1000),
                mode=getattr(self, "mode", 0),
                speed=getattr(self, "speed", 100),
            )
            return type("Response", (), {"values": [value]})()

        def read_status(self):
            return self.status

        def write_register(self, register_addr, data):
            writes.append((register_addr, data))
            return self.status

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", LimitMotor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "MOVE ID1")
    assert tool.run(_args(tmp_path)) == 1
    assert "exceed effective limits" in capsys.readouterr().err
    assert writes == [(tool.PAUSE_ADDR, [1])]


def test_device_limits_tighten_fixed_software_range(monkeypatch, tmp_path, capsys):
    writes = []

    class DeviceLimitMotor:
        def __init__(self, bus, motor_id):
            self.status = _status(500)

        def read_register(self, register_addr):
            values = {
                tool.LOWER_LIMIT_ADDR: 500,
                tool.UPPER_LIMIT_ADDR: 1500,
                tool.CONTROL_MODE_ADDR: 0,
                tool.TARGET_SPEED_ADDR: 100,
                tool.TARGET_POSITION_ADDR: 500,
            }
            return type("Response", (), {"values": [values[register_addr]]})()

        def read_status(self):
            return self.status

        def write_register(self, register_addr, data):
            writes.append((register_addr, list(data)))
            return self.status

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", DeviceLimitMotor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "MOVE ID1")
    assert tool.run(_args(tmp_path)) == 1
    assert "effective limits [500, 1500]" in capsys.readouterr().err
    assert writes == [(tool.PAUSE_ADDR, [1])]


def test_fault_stops_and_pause_is_sent_once(monkeypatch, tmp_path, capsys):
    writes = []

    class FaultMotor:
        def __init__(self, bus, motor_id):
            self.target = 1000

        def read_register(self, register_addr):
            value = _register_value(
                register_addr,
                target=getattr(self, "target", 1000),
                mode=getattr(self, "mode", 0),
                speed=getattr(self, "speed", 100),
            )
            return type("Response", (), {"values": [value]})()

        def read_status(self):
            return _status(self.target, fault=1, target=self.target)

        def write_register(self, register_addr, data):
            writes.append((register_addr, list(data)))
            return _status(self.target, fault=1, target=self.target)

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", FaultMotor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "MOVE ID1")
    assert tool.run(_args(tmp_path)) == 1
    assert "fault code=0x01" in capsys.readouterr().err
    assert writes == [(tool.PAUSE_ADDR, [1])]


def test_write_failure_is_not_automatically_retried(monkeypatch, tmp_path):
    writes = []

    class WriteFailureMotor:
        def __init__(self, bus, motor_id):
            pass

        def read_register(self, register_addr):
            value = _register_value(
                register_addr,
                target=getattr(self, "target", 1000),
                mode=getattr(self, "mode", 0),
                speed=getattr(self, "speed", 100),
            )
            return type("Response", (), {"values": [value]})()

        def read_status(self):
            return _status(1000)

        def write_register(self, register_addr, data):
            writes.append((register_addr, list(data)))
            raise RetryExhaustedError(1, NoResponseError("write ack lost"))

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", WriteFailureMotor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr("builtins.input", lambda prompt: "MOVE ID1")
    assert tool.run(_args(tmp_path)) == 1
    assert writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 1000]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_existing_log_is_refused_without_overwrite(tmp_path, capsys):
    log_path = tmp_path / "micro.log"
    log_path.write_text("existing-data", encoding="utf-8")
    assert tool.run(_args(tmp_path)) == 1
    assert "cannot create independent log" in capsys.readouterr().err
    assert log_path.read_text(encoding="utf-8") == "existing-data"


def test_motion_tool_has_no_general_motion_api_or_overwriteable_log():
    tree = ast.parse((ROOT / "tools" / "safe_micro_move_id1.py").read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint({"set_position", "set_speed", "move", "trajectory"})
    assert "mode=\"x\"" in (ROOT / "tools" / "safe_micro_move_id1.py").read_text(encoding="utf-8")
