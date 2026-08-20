"""No-hardware tests for one-shot ID4 power-on initialization."""

from __future__ import annotations

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
    _build_frame,
    pack_read_register_cmd,
    pack_read_status_cmd,
    pack_write_register_cmd,
    parse_frame,
)
from tools import safe_id4_poweron_initialize as tool  # noqa: E402


def _status(position, *, target=None, current=0, force=-15, fault=0):
    return StatusFeedback(
        target_position=position if target is None else target,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=100,
        temperature=25,
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
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


class FakeMotor:
    def __init__(
        self,
        bus,
        motor_id,
        *,
        entry_actual=-2,
        entry_target=-2,
        recovery_actual=9,
        recovery_current=0,
        entry_current=0,
        entry_fault=0,
        target_readback_override=None,
    ):
        assert motor_id == 4
        self.bus = bus
        self.motor_id = motor_id
        self.target = entry_target
        self.actual = entry_actual
        self.recovery_actual = recovery_actual
        self.recovery_current = recovery_current
        self.entry_current = entry_current
        self.entry_fault = entry_fault
        self.target_readback_override = target_readback_override
        self.recovery_started = False
        self.paused = False
        self.writes = []

    @staticmethod
    def _raw(value):
        return value & 0xFFFF

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        target = self.target
        if self.recovery_started and self.target_readback_override is not None:
            target = self.target_readback_override
        values = {
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.TARGET_POSITION_ADDR: self._raw(target),
            tool.ACTUAL_POSITION_ADDR: self._raw(self.actual),
        }
        return ReadRegisterResponse(register_addr=address, values=(values[address],))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        current = self.recovery_current if self.recovery_started and not self.paused else self.entry_current
        fault = 0 if self.recovery_started else self.entry_fault
        if self.paused:
            current = 0
        return _status(self.actual, target=self.target, current=current, fault=fault)

    def write_register(self, address, data):
        values = list(data)
        self.bus.request(
            pack_write_register_cmd(self.motor_id, address, values),
            expected_id=self.motor_id,
            retries=0,
        )
        self.writes.append((address, values))
        if address == tool.CONTROL_MODE_ADDR:
            self.recovery_started = True
            self.target = values[-1]
            self.actual = self.recovery_actual
        elif address == tool.PAUSE_ADDR:
            self.paused = True
            self.target = self.actual
        current = self.recovery_current if self.recovery_started and not self.paused else 0
        return _status(self.actual, target=self.target, current=current)


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
            str(tmp_path / "id4_init.log"),
        ]
    )


def _run(monkeypatch, tmp_path, **motor_options):
    created = []

    def motor_factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id, **motor_options)
        created.append(motor)
        return motor

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm_operator", lambda: None)
    monkeypatch.setattr(tool, "ENTRY_SAMPLE_COUNT", 3)
    monkeypatch.setattr(tool, "ENTRY_SAMPLE_INTERVAL_S", 0.01)
    monkeypatch.setattr(tool, "MOTION_TIMEOUT_S", 0.04)
    monkeypatch.setattr(tool, "POLL_INTERVAL_S", 0.02)
    monkeypatch.setattr(tool, "STOP_CONFIRM_DURATION_S", 0.10)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    result = tool.run(_args(tmp_path))
    return result, FakeBus.instances[0], created[0]


def _write_events(bus):
    return [
        event
        for event in bus.events
        if len(event) > 2 and event[2] == CMD_WR_REGISTER
    ]


def test_success_uses_one_atomic_recovery_write_and_one_pause(
    monkeypatch, tmp_path, capsys
):
    result, bus, motor = _run(monkeypatch, tmp_path)
    assert result == 0
    assert "ID4_POWERON_RECOVERY_COMPLETE" in capsys.readouterr().out
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]
    assert all(event[1] == 4 for event in _write_events(bus))
    assert bus.events.index(_write_events(bus)[-1]) < bus.events.index(("close",))


@pytest.mark.parametrize(
    "options",
    [
        {"entry_current": 1},
        {"entry_fault": 1},
        {"entry_actual": -1},
        {"entry_target": -1},
    ],
)
def test_entry_must_be_exact_minus2_and_other_states_are_zero_write(
    monkeypatch, tmp_path, options
):
    result, bus, motor = _run(monkeypatch, tmp_path, **options)
    assert result == 1
    assert motor.writes == []
    assert _write_events(bus) == []


def test_already_legal_state_passes_with_zero_writes(monkeypatch, tmp_path, capsys):
    result, bus, motor = _run(
        monkeypatch, tmp_path, entry_actual=39, entry_target=39
    )
    assert result == 0
    assert "ALREADY_IN_RANGE_NO_ACTION" in capsys.readouterr().out
    assert motor.writes == []
    assert _write_events(bus) == []


def test_target_readback_mismatch_pauses_and_fails(monkeypatch, tmp_path):
    result, _, motor = _run(
        monkeypatch, tmp_path, target_readback_override=8
    )
    assert result == 1
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_overshoot_pauses_and_fails(monkeypatch, tmp_path):
    result, _, motor = _run(monkeypatch, tmp_path, recovery_actual=11)
    assert result == 1
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_overcurrent_after_recovery_write_pauses_and_fails(monkeypatch, tmp_path):
    result, _, motor = _run(monkeypatch, tmp_path, recovery_current=501)
    assert result == 1
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_auditor_rejects_other_id_broadcast_and_wrong_write():
    audited = tool.Id4InitializeAuditedBus(FakeBus("test", timeout=0.05, retries=2))
    with pytest.raises(tool.SafetyViolation, match="ID4 only"):
        audited.request(pack_read_status_cmd(3), expected_id=3)
    with pytest.raises(tool.SafetyViolation, match="ID4 only"):
        audited.request(
            _build_frame(
                0xFF,
                CMD_WR_REGISTER,
                tool.PAUSE_ADDR,
                b"\x01\x00",
                allow_broadcast=True,
            )
        )
    with pytest.raises(tool.SafetyViolation, match="sequence mismatch"):
        audited.request(
            pack_write_register_cmd(4, tool.TARGET_POSITION_ADDR, [10]),
            expected_id=4,
        )


def test_parameters_are_fixed_and_not_cli_overridable():
    parser = tool.build_parser()
    options = {
        option
        for action in parser._actions
        for option in action.option_strings
    }
    assert "--target" not in options
    assert "--speed" not in options
    assert "--motion-timeout" not in options
    assert "--max-current" not in options
    assert tool.RECOVERY_TARGET == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500
    assert (tool.DEVICE_LOWER_LIMIT, tool.DEVICE_UPPER_LIMIT) == (0, 2000)


def test_operator_abort_is_zero_write(monkeypatch, tmp_path):
    FakeBus.instances.clear()
    created = []

    def motor_factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id)
        created.append(motor)
        return motor

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(
        tool,
        "_confirm_operator",
        lambda: (_ for _ in ()).throw(tool.SafetyViolation("not armed")),
    )
    monkeypatch.setattr(tool, "ENTRY_SAMPLE_COUNT", 1)
    assert tool.run(_args(tmp_path)) == 1
    assert created[0].writes == []
    assert _write_events(FakeBus.instances[0]) == []
