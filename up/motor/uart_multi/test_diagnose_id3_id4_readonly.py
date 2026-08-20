"""No-hardware tests for the fixed ID3/ID4 read-only comparison."""

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
from tools import diagnose_id3_id4_readonly as tool  # noqa: E402


def _status(target, actual, current=0, force=0, fault=0):
    return StatusFeedback(
        target_position=target,
        current_position=actual,
        current_mA=current,
        force_g=force,
        force_adc=2048,
        temperature=32,
        error_code=fault,
    )


def _state_data(target_raw, status, mode=0, speed=0xFFFF):
    return {
        "mode": mode,
        "speed": speed,
        "target_raw": target_raw,
        "status": status,
    }


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
    def __init__(self, bus, motor_id, rounds, fail_at=None):
        self.bus = bus
        self.motor_id = motor_id
        self.rounds = rounds
        self.fail_at = fail_at
        self.index = 0

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        if self.fail_at == (self.motor_id, self.index, address):
            raise OSError("simulated read failure")
        state = self.rounds[min(self.index, len(self.rounds) - 1)]
        values = {
            tool.MODE_ADDR: state["mode"],
            tool.TARGET_SPEED_ADDR: state["speed"],
            tool.TARGET_POSITION_ADDR: state["target_raw"],
        }
        return ReadRegisterResponse(register_addr=address, values=(values[address],))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        state = self.rounds[min(self.index, len(self.rounds) - 1)]
        self.index += 1
        return state["status"]


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
            str(tmp_path / "id3_id4_readonly.log"),
        ]
    )


def _run(monkeypatch, tmp_path, data, *, fail_at=None):
    messages = []

    def motor_factory(bus, motor_id):
        assert motor_id in tool.MOTOR_IDS
        return FakeMotor(bus, motor_id, data[motor_id], fail_at=fail_at)

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


def _persistent_data():
    return {
        3: [
            _state_data(0, _status(0, 0, current=0, force=353))
            for _ in range(3)
        ],
        4: [
            _state_data(0xFFFE, _status(-2, -2, current=0, force=-61))
            for _ in range(3)
        ],
    }


def _write_events(bus):
    return [
        event
        for event in bus.events
        if event[0] == "request" and event[2] == CMD_WR_REGISTER
    ]


def test_persistent_id3_zero_id4_minus2_records_every_requested_field(
    monkeypatch, tmp_path, capsys
):
    result, bus, messages = _run(monkeypatch, tmp_path, _persistent_data())
    assert result == 0
    assert bus.closed
    assert _write_events(bus) == []
    assert len([event for event in bus.events if event[0] == "request"]) == 24
    assert any(
        "ID3_SAMPLE index=1" in message
        and "mode=0" in message
        and "target_speed=65535" in message
        and "register_target_position=0" in message
        and "status_target_position=0" in message
        and "actual_position=0" in message
        and "current_mA=0" in message
        and "force_g=353" in message
        and "fault=0x00" in message
        for message in messages
    )
    assert any(
        "ID4_SAMPLE index=1" in message
        and "register_target_position=-2" in message
        and "target_raw_u16=65534" in message
        and "target_raw_hex=0xFFFE" in message
        and "actual_position=-2" in message
        for message in messages
    )
    assert any(
        "classification=PERSISTENT_ID3_ZERO_ID4_MINUS2" in message
        and "register_target_equal=0" in message
        and "register_target_different=3" in message
        and "fault_equal=3" in message
        and "write_count=0" in message
        for message in messages
    )
    assert "classification=PERSISTENT_ID3_ZERO_ID4_MINUS2" in capsys.readouterr().out


def test_per_id_summaries_include_histograms_for_all_fields(monkeypatch, tmp_path):
    data = _persistent_data()
    data[3][1] = _state_data(
        1, _status(1, 0, current=2, force=354, fault=1), mode=1, speed=50
    )
    result, bus, messages = _run(monkeypatch, tmp_path, data)
    assert result == 0
    assert _write_events(bus) == []
    id3_summary = next(message for message in messages if "ID3_SUMMARY" in message)
    for field in (
        "mode_histogram",
        "target_speed_histogram",
        "register_target_histogram",
        "status_target_histogram",
        "actual_histogram",
        "current_mA_histogram",
        "force_g_histogram",
        "fault_histogram",
    ):
        assert field in id3_summary


@pytest.mark.parametrize(
    ("id3_target", "id3_status", "id4_target", "id4_status", "classification"),
    [
        (
            5,
            _status(5, 5),
            0xFFFE,
            _status(-2, -2),
            "PERSISTENT_ID4_MINUS2_WITH_ID3_REFERENCE",
        ),
        (
            0,
            _status(0, 0),
            0,
            _status(0, 0),
            "ID4_MINUS2_NOT_OBSERVED",
        ),
    ],
)
def test_complete_comparison_classifications(
    monkeypatch,
    tmp_path,
    id3_target,
    id3_status,
    id4_target,
    id4_status,
    classification,
):
    data = {
        3: [_state_data(id3_target, id3_status) for _ in range(3)],
        4: [_state_data(id4_target, id4_status) for _ in range(3)],
    }
    result, bus, messages = _run(monkeypatch, tmp_path, data)
    assert result == 0
    assert _write_events(bus) == []
    assert any(f"classification={classification}" in message for message in messages)


def test_mixed_id4_minus2_is_not_reported_as_persistent(monkeypatch, tmp_path):
    data = _persistent_data()
    data[4][1] = _state_data(0, _status(0, 0))
    result, bus, messages = _run(monkeypatch, tmp_path, data)
    assert result == 0
    assert _write_events(bus) == []
    assert any(
        "classification=INTERMITTENT_OR_MIXED_ID4_MINUS2" in message
        for message in messages
    )


def test_partial_read_failure_is_inconclusive_and_closes_bus(
    monkeypatch, tmp_path, capsys
):
    fail_at = (4, 1, tool.TARGET_POSITION_ADDR)
    result, bus, messages = _run(
        monkeypatch, tmp_path, _persistent_data(), fail_at=fail_at
    )
    assert result == 1
    assert bus.closed
    assert _write_events(bus) == []
    assert any("classification=INCONCLUSIVE" in message for message in messages)
    assert "comparison incomplete" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("frame", "kwargs", "message"),
    [
        (pack_write_register_cmd(3, 0x1A, [1]), {}, "not read-only"),
        (pack_read_status_cmd(2), {}, "outside fixed ID3/ID4 scope"),
        (pack_read_register_cmd(3, 0x2A), {}, "outside read whitelist"),
        (pack_read_register_cmd(4, 0x25, 2), {}, "single-register"),
        (pack_read_status_cmd(3), {"expected_id": 4}, "does not match"),
    ],
)
def test_audit_blocks_every_out_of_scope_request_before_delegate(
    frame, kwargs, message
):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.Id3Id4ReadOnlyAuditedBus(delegate)
    with pytest.raises(tool.ReadOnlyViolation, match=message):
        audited.request(frame, **kwargs)
    assert delegate.events == []
    assert "write_count=0" in audited.summary()


def test_signed_target_conversion_is_local_exact_and_never_clamps():
    assert tool._signed_u16(0xFFFE) == -2
    assert tool._signed_u16(0) == 0
    assert tool._signed_u16(0x7FFF) == 32767
    with pytest.raises(ValueError):
        tool._signed_u16(0x10000)


def test_cli_scope_is_fixed_and_source_has_no_write_or_motion_calls(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 43

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(["--port", "/dev/ttyTEST", "--log", "diag.log"]) == 43
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "timeout": 0.05,
        "retries": 2,
        "log": "diag.log",
    }
    assert tool.MOTOR_IDS == (3, 4)
    assert tool.SAMPLE_COUNT == 100
    assert tool.SAMPLE_INTERVAL_S == 0.05

    source = (ROOT / "tools" / "diagnose_id3_id4_readonly.py").read_text(
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
