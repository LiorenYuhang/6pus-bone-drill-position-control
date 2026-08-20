"""No-hardware tests for the fixed ID4 read-only diagnosis."""

from __future__ import annotations

import ast
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CMD_WR_REGISTER,
    ReadRegisterResponse,
    StatusFeedback,
    pack_read_register_cmd,
    pack_read_status_cmd,
    pack_write_register_cmd,
    parse_frame,
)
from tools import diagnose_id4_readonly as tool  # noqa: E402


def _status(target=-2, actual=-2, current=0, force=-76, fault=0):
    return StatusFeedback(
        target_position=target,
        current_position=actual,
        current_mA=current,
        force_g=force,
        force_adc=1926,
        temperature=32,
        error_code=fault,
    )


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeBus:
    instances = []

    def __init__(self, port, *, timeout, retries):
        self.port = port
        self.timeout = timeout
        self.retries = retries
        self.events = []
        self.closed = False
        type(self).instances.append(self)

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        self.events.append(
            ("request", parsed.motor_id, parsed.command, parsed.register_addr)
        )
        return b""

    def close(self):
        self.events.append(("close",))
        self.closed = True


class FakeMotor:
    def __init__(self, bus, motor_id, targets, statuses, fail_at=None):
        self.bus = bus
        self.motor_id = motor_id
        self.targets = targets
        self.statuses = statuses
        self.fail_at = fail_at
        self.index = 0

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        if self.fail_at is not None and self.index == self.fail_at:
            raise OSError("simulated read failure")
        index = min(self.index, len(self.targets) - 1)
        return ReadRegisterResponse(register_addr=address, values=(self.targets[index],))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        index = min(self.index, len(self.statuses) - 1)
        self.index += 1
        return self.statuses[index]


def _args(tmp_path):
    return tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--timeout",
            "0.05",
            "--retries",
            "2",
            "--log",
            str(tmp_path / "id4_readonly.log"),
        ]
    )


def _run(monkeypatch, tmp_path, targets, statuses, *, fail_at=None):
    messages = []

    def motor_factory(bus, motor_id):
        assert motor_id == 4
        return FakeMotor(bus, motor_id, targets, statuses, fail_at=fail_at)

    def capture(message, *args):
        messages.append(message % args if args else message)

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool.logger, "info", capture)
    monkeypatch.setattr(tool.logger, "error", capture)
    monkeypatch.setattr(tool, "SAMPLE_COUNT", 3)
    monkeypatch.setattr(tool, "SAMPLE_INTERVAL_S", 0.05)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    result = tool.run(_args(tmp_path))
    return result, FakeBus.instances[0], messages


def _write_events(bus):
    return [
        event
        for event in bus.events
        if event[0] == "request" and event[2] == CMD_WR_REGISTER
    ]


def test_persistent_minus_two_records_all_fields_and_zero_writes(
    monkeypatch, tmp_path, capsys
):
    result, bus, messages = _run(
        monkeypatch,
        tmp_path,
        [0xFFFE] * 3,
        [_status(current=value, force=-76 - value) for value in (0, 1, 0)],
    )
    assert result == 0
    assert _write_events(bus) == []
    assert bus.closed
    assert sum(event[3] == tool.TARGET_POSITION_ADDR for event in bus.events[:-1]) == 3
    assert any(
        "ID4_SAMPLE index=1" in message
        and "target_position=-2" in message
        and "target_raw_u16=65534" in message
        and "target_raw_hex=0xFFFE" in message
        and "actual_position=-2" in message
        for message in messages
    )
    assert any(
        "classification=PERSISTENT_MINUS2" in message
        and "consistent_minus2_count=3" in message
        and "write_count=0" in message
        for message in messages
    )
    assert "classification=PERSISTENT_MINUS2" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("targets", "statuses", "classification"),
    [
        (
            [0xFFFE, 0, 0xFFFE],
            [_status(), _status(target=0, actual=0), _status()],
            "INTERMITTENT_OR_MIXED_MINUS2",
        ),
        (
            [0, 0, 0],
            [_status(target=0, actual=0) for _ in range(3)],
            "MINUS2_NOT_OBSERVED",
        ),
        (
            [0xFFFE] * 3,
            [_status(), _status(target=-2, actual=0), _status()],
            "INTERMITTENT_OR_MIXED_MINUS2",
        ),
    ],
)
def test_diagnosis_distinguishes_intermittent_absent_and_mixed_values(
    monkeypatch, tmp_path, targets, statuses, classification
):
    result, bus, messages = _run(
        monkeypatch, tmp_path, targets, statuses
    )
    assert result == 0
    assert _write_events(bus) == []
    assert any(f"classification={classification}" in message for message in messages)


def test_fault_current_and_force_are_observations_not_writes(monkeypatch, tmp_path):
    statuses = [
        _status(current=17, force=-81, fault=2),
        _status(current=0, force=-79, fault=0),
        _status(current=5, force=-80, fault=2),
    ]
    result, bus, messages = _run(
        monkeypatch, tmp_path, [0xFFFE] * 3, statuses
    )
    assert result == 0
    assert _write_events(bus) == []
    assert any(
        "current_min_mA=0 current_max_mA=17" in message
        and "force_min_g=-81 force_max_g=-79" in message
        and "fault_histogram={0: 1, 2: 2}" in message
        for message in messages
    )


def test_read_failure_is_inconclusive_and_bus_closes(monkeypatch, tmp_path, capsys):
    result, bus, messages = _run(
        monkeypatch,
        tmp_path,
        [0xFFFE] * 3,
        [_status()] * 3,
        fail_at=1,
    )
    assert result == 1
    assert bus.closed
    assert _write_events(bus) == []
    assert any("classification=INCONCLUSIVE" in message for message in messages)
    assert "diagnosis incomplete" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (pack_write_register_cmd(4, 0x1A, [1]), "not read-only"),
        (pack_read_status_cmd(3), "outside fixed ID4 scope"),
    ],
)
def test_audit_blocks_writes_and_other_destinations_before_delegate(frame, message):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.Id4ReadOnlyAuditedBus(delegate)
    with pytest.raises(tool.ReadOnlyViolation, match=message):
        audited.request(frame)
    assert delegate.events == []
    assert "write_count=0" in audited.summary()


def test_signed_u16_conversion_is_local_and_exact():
    assert tool._signed_u16(0xFFFE) == -2
    assert tool._signed_u16(0) == 0
    assert tool._signed_u16(0x7FFF) == 32767
    with pytest.raises(ValueError):
        tool._signed_u16(0x10000)


def test_cli_is_fixed_id4_read_only_and_has_no_motion_or_write_calls(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 44

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(["--port", "/dev/ttyTEST", "--log", "diag.log"]) == 44
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "timeout": 0.05,
        "retries": 2,
        "log": "diag.log",
    }
    assert tool.MOTOR_ID == 4
    assert tool.SAMPLE_COUNT == 100
    assert tool.SAMPLE_INTERVAL_S == 0.05

    source = (ROOT / "tools" / "diagnose_id4_readonly.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint(
        {
            "write_register",
            "set_motor_id",
            "save_parameters",
            "set_position",
            "set_speed",
            "move",
        }
    )


@pytest.mark.parametrize(
    "argv",
    [
        ["--port", "/dev/tty", "--timeout", "0"],
        ["--port", "/dev/tty", "--retries", "-1"],
    ],
)
def test_cli_rejects_invalid_transport_arguments(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
