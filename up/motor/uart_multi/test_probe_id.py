"""No-hardware tests for the read-only ID probe CLI."""

from __future__ import annotations

import ast
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import Register  # noqa: E402
from tools import probe_id  # noqa: E402
from uart_bus import NoResponseError, RetryExhaustedError  # noqa: E402


class FakeBus:
    instances = []

    def __init__(self, port, *, timeout, retries):
        self.port = port
        self.timeout = timeout
        self.retries = retries
        self.closed = False
        self.__class__.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.closed = True
        return False


def _args(tmp_path, *, motor_id=2):
    return probe_id.parse_args([
        "--port", "/dev/serial/by-id/usb-test",
        "--id", str(motor_id),
        "--timeout", "0.125",
        "--log", str(tmp_path / "probe.log"),
    ])


def test_probe_reports_response_and_reads_only_id_register(
    monkeypatch,
    tmp_path,
    capsys,
):
    calls = []

    class FakeMotor:
        def __init__(self, bus, motor_id):
            calls.append(("init", bus, motor_id))

        def read_register(self, register_addr):
            calls.append(("read", register_addr))
            return type("Response", (), {"values": [2]})()

    FakeBus.instances.clear()
    monkeypatch.setattr(probe_id, "UartBus", FakeBus)
    monkeypatch.setattr(probe_id, "Motor", FakeMotor)
    monkeypatch.setattr(probe_id, "_configure_logging", lambda path: None)

    assert probe_id.run(_args(tmp_path)) == 0

    output = capsys.readouterr()
    assert output.out.strip() == "RESPONSE requested_id=2 readback_id=2"
    assert output.err == ""
    assert calls[0][0] == "init"
    assert calls[0][2] == 2
    assert calls[1:] == [("read", Register.ID_ADDR)]
    assert len(FakeBus.instances) == 1
    bus = FakeBus.instances[0]
    assert (bus.port, bus.timeout, bus.retries) == (
        "/dev/serial/by-id/usb-test",
        0.125,
        0,
    )
    assert bus.closed is True


def test_probe_treats_timeout_as_normal_no_response(
    monkeypatch,
    tmp_path,
    capsys,
):
    class SilentMotor:
        def __init__(self, bus, motor_id):
            pass

        def read_register(self, register_addr):
            raise RetryExhaustedError(1, NoResponseError("timed out"))

    monkeypatch.setattr(probe_id, "UartBus", FakeBus)
    monkeypatch.setattr(probe_id, "Motor", SilentMotor)
    monkeypatch.setattr(probe_id, "_configure_logging", lambda path: None)

    assert probe_id.run(_args(tmp_path, motor_id=6)) == 0

    output = capsys.readouterr()
    assert output.out.strip() == "NO RESPONSE id=6"
    assert output.err == ""
    assert "Traceback" not in output.out + output.err


def test_probe_main_passes_its_four_parsed_arguments(monkeypatch, tmp_path):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 19

    log_path = tmp_path / "probe.log"
    monkeypatch.setattr(probe_id, "run", fake_run)
    result = probe_id.main([
        "--port", "/dev/ttyTEST0",
        "--id", "4",
        "--timeout", "0.2",
        "--log", str(log_path),
    ])

    args = captured["args"]
    assert result == 19
    assert vars(args) == {
        "port": "/dev/ttyTEST0",
        "id": 4,
        "timeout": 0.2,
        "log": str(log_path),
    }


def test_probe_source_has_no_mutating_or_motion_method_calls():
    source_path = ROOT / "tools" / "probe_id.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    forbidden = {
        "write_register",
        "set_motor_id",
        "save_parameters",
        "set_position",
        "set_speed",
        "move",
        "trajectory",
    }
    assert called_attributes.isdisjoint(forbidden)
