"""No-hardware tests for the parameterized four-motor read-only stage."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

from protocol import Register
from tools import probe_two_ids
from uart_bus import NoResponseError, RetryExhaustedError

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


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


def test_four_ids_use_one_bus_and_keep_absent_probe_out_of_online_timeout(
    monkeypatch, tmp_path, capsys
):
    calls = []

    class FakeMotor:
        def __init__(self, bus, motor_id):
            calls.append(("init", id(bus), motor_id))
            self.motor_id = motor_id

        def read_register(self, register_addr):
            assert register_addr == Register.ID_ADDR
            calls.append(("read", self.motor_id))
            if self.motor_id == 5:
                raise RetryExhaustedError(1, NoResponseError("ID5 absent"))
            return type("Response", (), {"values": [self.motor_id]})()

    FakeBus.instances.clear()
    monkeypatch.setattr(probe_two_ids, "UartBus", FakeBus)
    monkeypatch.setattr(probe_two_ids, "Motor", FakeMotor)
    monkeypatch.setattr(probe_two_ids, "_configure_logging", lambda path: None)

    args = probe_two_ids.parse_args([
        "--port", "/dev/serial/by-id/usb-test",
        "--ids", "1", "2", "3", "4",
        "--probe-id", "5",
        "--retries", "2",
        "--log", str(tmp_path / "four.log"),
    ])
    assert probe_two_ids.run(args) == 0
    output = capsys.readouterr().out

    assert "ID5 EXPECTED NO RESPONSE" in output
    for motor_id in range(1, 5):
        assert (
            f"ID{motor_id} STATS requests=101 first_success=101 "
            "retry_success=0 final_failure=0 timeout=0 wrong_id=0 "
            "length_error=0 checksum_error=0"
        ) in output
    assert "TOTAL requests=404 first_success=404 retry_success=0 final_failure=0" in output
    assert "timeout=0 wrong_id=0 length_error=0 checksum_error=0 expected_no_response=1" in output

    assert len(FakeBus.instances) == 1
    assert FakeBus.instances[0].retries == 0
    assert FakeBus.instances[0].closed is True
    assert {item[2] for item in calls if item[0] == "init"} == {1, 2, 3, 4, 5}
    assert sum(item[0] == "read" and item[1] == 5 for item in calls) == 1
    assert sum(item[0] == "read" and item[1] in {1, 2, 3, 4} for item in calls) == 404


def test_per_id_retry_and_failure_categories_are_separate(monkeypatch):
    class RetryMotor:
        def __init__(self):
            self.calls = 0

        def read_register(self, register_addr):
            self.calls += 1
            if self.calls == 1:
                raise RetryExhaustedError(1, NoResponseError("transient"))
            return type("Response", (), {"values": [1]})()

    stats = probe_two_ids.MotorStats()
    success, readback, error = probe_two_ids._read_online(
        RetryMotor(), 1, stats, retries=1
    )
    assert (success, readback, error) == (True, 1, None)
    assert stats.requests == 1
    assert stats.first_success == 0
    assert stats.retry_success == 1
    assert stats.final_failure == 0
    assert stats.timeout == 1


def test_probe_id_is_single_attempt_and_cli_rejects_overlap():
    class SilentMotor:
        def __init__(self):
            self.calls = 0

        def read_register(self, register_addr):
            self.calls += 1
            raise RetryExhaustedError(4, NoResponseError("absent"))

    motor = SilentMotor()
    overall = probe_two_ids.OverallStats()
    success, readback, error = probe_two_ids._probe_absent(motor, 5, overall)
    assert not success and readback is None
    assert isinstance(error, NoResponseError)
    assert motor.calls == 1
    assert overall.expected_no_response == 1

    try:
        probe_two_ids.parse_args([
            "--port", "/dev/tty", "--ids", "1", "2", "--probe-id", "2"
        ])
    except SystemExit:
        pass
    else:
        raise AssertionError("probe ID overlap must be rejected")


def test_parameterized_tool_has_no_write_or_motion_calls():
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
