"""No-hardware tests for the fixed ID3/ID4 configuration audit."""

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
from tools import audit_id3_id4_config_readonly as tool  # noqa: E402


def _status(target, actual, current=0, force=0, fault=0):
    return StatusFeedback(
        target_position=target,
        current_position=actual,
        current_mA=current,
        force_g=force,
        force_adc=2000,
        temperature=30,
        error_code=fault,
    )


def _config(motor_id, target_raw):
    return {
        0x16: motor_id,
        0x17: 2,
        0x1E: 80,
        0x1F: 60,
        0x20: 500,
        0x21: 1000,
        0x22: 1000,
        0x23: 2000,
        0x24: 0,
        0x25: 0,
        0x26: 0,
        0x27: 0,
        0x28: 0xFFFF,
        0x29: target_raw,
    }


def _rounds(motor_id, target_raw, status):
    return [(_config(motor_id, target_raw), status) for _ in range(3)]


def _matching_data():
    return {
        3: _rounds(3, 0, _status(0, 0, force=360)),
        4: _rounds(4, 0xFFFE, _status(-2, -2, force=-60)),
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
        config, _ = self.rounds[min(self.index, len(self.rounds) - 1)]
        return ReadRegisterResponse(register_addr=address, values=(config[address],))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        _, status = self.rounds[min(self.index, len(self.rounds) - 1)]
        self.index += 1
        return status


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
            str(tmp_path / "config_audit.log"),
        ]
    )


def _run(monkeypatch, tmp_path, data, *, fail_at=None):
    messages = []

    def motor_factory(bus, motor_id):
        return FakeMotor(bus, motor_id, data[motor_id], fail_at=fail_at)

    def capture(message, *args):
        messages.append(message % args if args else message)

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool.logger, "info", capture)
    monkeypatch.setattr(tool.logger, "error", capture)
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


def test_matching_static_config_and_persistent_minus_two_are_reported(
    monkeypatch, tmp_path, capsys
):
    result, bus, messages = _run(monkeypatch, tmp_path, _matching_data())
    assert result == 0
    assert bus.closed
    assert _write_events(bus) == []
    assert len([event for event in bus.events if event[0] == "request"]) == 90
    assert any(
        "classification=CONFIG_MATCH_EXCEPT_ID_AND_TARGET_WITH_ID4_MINUS2"
        in message
        and "write_count=0" in message
        for message in messages
    )
    assert "CONFIG_MATCH_EXCEPT_ID_AND_TARGET_WITH_ID4_MINUS2" in capsys.readouterr().out


def test_every_documented_whitelist_register_is_read_three_times_per_id(
    monkeypatch, tmp_path
):
    result, bus, _ = _run(monkeypatch, tmp_path, _matching_data())
    assert result == 0
    for motor_id in tool.MOTOR_IDS:
        for address in tool.READ_REGISTER_ADDRS:
            assert bus.events.count(("request", motor_id, 0x31, address)) == 3
    assert bus.events.count(("request", 3, 0x30, 0)) == 3
    assert bus.events.count(("request", 4, 0x30, 0)) == 3


def test_register_and_status_output_preserve_raw_signed_and_all_fields(
    monkeypatch, tmp_path
):
    result, _, messages = _run(monkeypatch, tmp_path, _matching_data())
    assert result == 0
    assert any(
        "ID4_CONFIG_SAMPLE index=1" in message
        and "address=0x29" in message
        and "name=target_position" in message
        and "raw_u16=65534" in message
        and "raw_hex=0xFFFE" in message
        and "signed_i16=-2" in message
        for message in messages
    )
    assert any(
        "ID4_STATUS_SAMPLE index=1" in message
        and "status_target_position=-2" in message
        and "actual_position=-2" in message
        and "current_mA=0" in message
        and "force_g=-60" in message
        and "force_adc=2000" in message
        and "temperature_C=30" in message
        and "fault=0x00" in message
        for message in messages
    )


def test_static_configuration_difference_changes_classification(monkeypatch, tmp_path):
    data = _matching_data()
    for config, _ in data[4]:
        config[0x24] = 1
    result, _, messages = _run(monkeypatch, tmp_path, data)
    assert result == 0
    assert any(
        "classification=CONFIG_DIFFERENCES_OBSERVED_WITH_ID4_MINUS2" in message
        for message in messages
    )
    assert any(
        "address=0x24" in message
        and "id3_histogram={0: 3}" in message
        and "id4_histogram={1: 3}" in message
        and "different=3" in message
        for message in messages
    )


def test_device_version_model_and_position_offset_are_explicitly_unavailable(
    monkeypatch, tmp_path
):
    result, _, messages = _run(monkeypatch, tmp_path, _matching_data())
    assert result == 0
    availability = next(
        message for message in messages if message.startswith("DEVICE_INFO_AVAILABILITY")
    )
    assert "firmware_version=NOT_DOCUMENTED" in availability
    assert "protocol_version=NOT_DOCUMENTED" in availability
    assert "device_model=NOT_DOCUMENTED" in availability
    assert "position_zero_register=NOT_DOCUMENTED" in availability
    assert "position_offset_register=NOT_DOCUMENTED" in availability
    assert "protocol_family_observed=V2_STYLE_0x30_0x31" in availability


def test_protocol_family_is_not_claimed_without_a_complete_paired_sample():
    lines = tool._summary_lines([], expected_count=3)
    assert "protocol_family_observed=NOT_CONFIRMED" in lines[0]
    assert "classification=INCONCLUSIVE" in lines[-1]


def test_partial_failure_is_inconclusive_and_bus_closes(monkeypatch, tmp_path, capsys):
    result, bus, messages = _run(
        monkeypatch,
        tmp_path,
        _matching_data(),
        fail_at=(4, 1, 0x23),
    )
    assert result == 1
    assert bus.closed
    assert _write_events(bus) == []
    assert any("classification=INCONCLUSIVE" in message for message in messages)
    assert "config audit incomplete" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("frame", "kwargs", "message"),
    [
        (pack_write_register_cmd(3, 0x29, [10]), {}, "not read-only"),
        (pack_read_status_cmd(2), {}, "outside fixed ID3/ID4 scope"),
        (pack_read_register_cmd(3, 0x18), {}, "outside config whitelist"),
        (pack_read_register_cmd(3, 0x1A), {}, "outside config whitelist"),
        (pack_read_register_cmd(4, 0x1C), {}, "outside config whitelist"),
        (pack_read_register_cmd(4, 0x2A), {}, "outside config whitelist"),
        (pack_read_register_cmd(4, 0x23, 2), {}, "single-register"),
        (pack_read_status_cmd(3), {"expected_id": 4}, "does not match"),
    ],
)
def test_audit_blocks_writes_actions_unlisted_reads_and_other_ids_before_delegate(
    frame, kwargs, message
):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.ConfigReadOnlyAuditedBus(delegate)
    with pytest.raises(tool.ReadOnlyViolation, match=message):
        audited.request(frame, **kwargs)
    assert delegate.events == []
    assert "write_count=0" in audited.summary()


def test_signed_conversion_is_local_and_never_clamps():
    assert tool._signed_u16(0xFFFE) == -2
    assert tool._signed_u16(0) == 0
    assert tool._signed_u16(0xFFFF) == -1
    with pytest.raises(ValueError):
        tool._signed_u16(0x10000)


def test_cli_scope_is_fixed_and_source_has_no_write_or_motion_calls(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 34

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(["--port", "/dev/ttyTEST", "--log", "audit.log"]) == 34
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "timeout": 0.05,
        "retries": 2,
        "log": "audit.log",
    }
    assert tool.MOTOR_IDS == (3, 4)
    assert tool.SAMPLE_COUNT == 3
    assert tool.SAMPLE_INTERVAL_S == 0.05

    source = (ROOT / "tools" / "audit_id3_id4_config_readonly.py").read_text(
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
