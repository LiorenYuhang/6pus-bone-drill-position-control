"""No-hardware tests for the two-motor read-only polling tool."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import Register  # noqa: E402
from tools import probe_two_ids  # noqa: E402
from uart_bus import NoResponseError, RetryExhaustedError  # noqa: E402


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


def _args(tmp_path, *, retries=1):
    return probe_two_ids.parse_args([
        "--port", "/dev/serial/by-id/usb-test",
        "--timeout", "0.125",
        "--retries", str(retries),
        "--log", str(tmp_path / "two.log"),
    ])


def test_two_id_run_shares_one_bus_reads_id3_and_polls_100_rounds(
    monkeypatch,
    tmp_path,
    capsys,
):
    calls = []
    attempts = {1: 0, 2: 0, 3: 0}

    class FakeMotor:
        def __init__(self, bus, motor_id):
            calls.append(("init", id(bus), motor_id))
            self.motor_id = motor_id

        def read_register(self, register_addr):
            assert register_addr == Register.ID_ADDR
            attempts[self.motor_id] += 1
            calls.append(("read", self.motor_id))
            if self.motor_id == 3:
                raise RetryExhaustedError(
                    1, NoResponseError("no ID3 response")
                )
            # Force one retry for ID1 on its first logical request.
            if self.motor_id == 1 and attempts[1] == 1:
                raise RetryExhaustedError(1, NoResponseError("transient"))
            return type("Response", (), {"values": [self.motor_id]})()

    FakeBus.instances.clear()
    monkeypatch.setattr(probe_two_ids, "UartBus", FakeBus)
    monkeypatch.setattr(probe_two_ids, "Motor", FakeMotor)
    monkeypatch.setattr(probe_two_ids, "_configure_logging", lambda path: None)
    monkeypatch.setattr(probe_two_ids, "ROUND_COUNT", 2)

    assert probe_two_ids.run(_args(tmp_path)) == 0
    output = capsys.readouterr()
    assert "ID1 RESPONSE readback_id=1" in output.out
    assert "ID2 RESPONSE readback_id=2" in output.out
    assert "ID3 EXPECTED NO RESPONSE" in output.out
    assert "requests=6" in output.out
    assert "first_success=5" in output.out
    assert "retry_success=1" in output.out
    assert "timeout=1" in output.out
    assert "wrong_id=0" in output.out
    assert "length_error=0" in output.out
    assert "checksum_error=0" in output.out
    assert len(FakeBus.instances) == 1
    assert FakeBus.instances[0].retries == 0
    assert FakeBus.instances[0].closed is True
    assert [item for item in calls if item[0] == "init"] == [
        ("init", id(FakeBus.instances[0]), 1),
        ("init", id(FakeBus.instances[0]), 2),
        ("init", id(FakeBus.instances[0]), 3),
    ]


def test_probe_counts_wrong_id_length_and_checksum_failures(monkeypatch, tmp_path, capsys):
    class FakeMotor:
        def __init__(self, bus, motor_id):
            self.motor_id = motor_id

        def read_register(self, register_addr):
            if self.motor_id == 1:
                return type("Response", (), {"values": [9]})()
            if self.motor_id == 2:
                from protocol import FrameChecksumError
                raise RetryExhaustedError(1, FrameChecksumError("bad checksum"))
            raise RetryExhaustedError(1, NoResponseError("missing"))

    monkeypatch.setattr(probe_two_ids, "UartBus", FakeBus)
    monkeypatch.setattr(probe_two_ids, "Motor", FakeMotor)
    monkeypatch.setattr(probe_two_ids, "_configure_logging", lambda path: None)
    monkeypatch.setattr(probe_two_ids, "ROUND_COUNT", 0)

    # ID1 is wrong, ID2 checksum-fails, and ID3 times out; all are expected
    # to be represented in the summary while the stage fails its pass gate.
    assert probe_two_ids.run(_args(tmp_path, retries=0)) == 1
    output = capsys.readouterr().out
    assert "wrong_id=1" in output
    assert "checksum_error=1" in output
    assert "timeout=0" in output


def test_probe_cli_arguments_and_read_only_source(monkeypatch, tmp_path):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 23

    monkeypatch.setattr(probe_two_ids, "run", fake_run)
    log_path = tmp_path / "two.log"
    assert probe_two_ids.main([
        "--port", "/dev/ttyTEST0",
        "--timeout", "0.2",
        "--retries", "3",
        "--log", str(log_path),
    ]) == 23
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST0",
        "ids": [1, 2],
        "probe_id": 3,
        "timeout": 0.2,
        "retries": 3,
        "log": str(log_path),
    }

    tree = ast.parse((ROOT / "tools" / "probe_two_ids.py").read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint({
        "write_register", "set_motor_id", "save_parameters",
        "set_position", "set_speed", "move", "trajectory",
    })


@pytest.mark.parametrize("argv", [
    ["--port", "/dev/tty", "--timeout", "0"],
    ["--port", "/dev/tty", "--retries", "-1"],
])
def test_probe_rejects_invalid_timing_arguments(argv):
    with pytest.raises(SystemExit):
        probe_two_ids.parse_args(argv)
