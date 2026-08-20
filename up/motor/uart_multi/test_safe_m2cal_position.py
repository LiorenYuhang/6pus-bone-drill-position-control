"""No-hardware tests for M2-CAL position characterization."""

from __future__ import annotations

from dataclasses import dataclass
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
    pack_write_register_cmd,
    parse_frame,
)
from tools import safe_m2cal_position as tool  # noqa: E402


def _status(position, current=0, force=-400, fault=0):
    return StatusFeedback(
        target_position=position,
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
        self.events.append(("request", parsed.motor_id, parsed.command, parsed.register_addr))
        return b""

    def close(self):
        self.events.append(("close",))
        self.closed = True


@dataclass
class Scenario:
    position: int
    writes: list[tuple[int, list[int]]]


class FakeMotor:
    def __init__(self, bus, motor_id, scenario):
        self.bus = bus
        self.motor_id = motor_id
        self.scenario = scenario

    def read_register(self, address):
        self.bus.request(pack_read_register_cmd(self.motor_id, address), expected_id=self.motor_id)
        values = {
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
        }
        return ReadRegisterResponse(register_addr=address, values=(values[address],))

    def read_status(self):
        return _status(self.scenario.position)

    def write_register(self, address, data):
        values = list(data)
        self.bus.request(
            pack_write_register_cmd(self.motor_id, address, values),
            expected_id=self.motor_id,
            retries=0,
        )
        self.scenario.writes.append((address, values))
        if address == tool.TARGET_POSITION_ADDR:
            self.scenario.position = values[0] - 2
        return _status(self.scenario.position)


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
            str(tmp_path / "m2cal.log"),
        ]
    )


def test_audit_rejects_broadcast_and_compensation_write():
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.CalibrationAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation):
        audited.request(pack_write_register_cmd(3, tool.PAUSE_ADDR, [1]), expected_id=3)
    with pytest.raises(tool.SafetyViolation):
        audited.request(pack_write_register_cmd(1, 0x1C, [1]), expected_id=1)
    assert delegate.events == []
    assert audited.write_frames == []


def test_calibration_run_uses_fixed_points_and_records_both_ids(monkeypatch, tmp_path):
    scenarios = {1: Scenario(20, []), 2: Scenario(30, [])}
    created = {}
    messages = []

    def bus_factory(port, *, timeout, retries):
        bus = FakeBus(port, timeout=timeout, retries=retries)
        FakeBus.instances.append(bus)
        return bus

    def motor_factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id, scenarios[motor_id])
        created[motor_id] = motor
        return motor

    def capture(message, *args):
        messages.append(message % args if args else message)

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", bus_factory)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm_operator", lambda: None)
    monkeypatch.setattr(tool.logger, "info", capture)
    monkeypatch.setattr(tool.logger, "warning", capture)
    monkeypatch.setattr(tool, "MOTION_TIMEOUT_S", 0.04)
    monkeypatch.setattr(tool, "STOP_CONFIRM_DURATION_S", 0.04)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    result = tool.run(_args(tmp_path))

    assert result == 0
    assert set(created) == {1, 2}
    bus = FakeBus.instances[-1]
    writes = [event for event in bus.events if event[0] == "request" and event[2] == CMD_WR_REGISTER]
    assert len(writes) == 24
    assert all(event[1] in (1, 2) for event in writes)
    assert [(event[1], event[3]) for event in writes[:3]] == [
        (1, tool.CONTROL_MODE_ADDR),
        (1, tool.TARGET_POSITION_ADDR),
        (1, tool.PAUSE_ADDR),
    ]
    assert [(event[1], event[3]) for event in writes[-3:]] == [
        (2, tool.CONTROL_MODE_ADDR),
        (2, tool.TARGET_POSITION_ADDR),
        (2, tool.PAUSE_ADDR),
    ]
    assert sum("CAL_POINT" in message for message in messages) == 8
    assert any("M2CAL_RESULT decision=PASS" in message for message in messages)
    assert any("compensation_write_count=0" in message for message in messages)


def test_operator_abort_happens_before_any_write(monkeypatch, tmp_path):
    scenarios = {1: Scenario(20, []), 2: Scenario(30, [])}
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: FakeMotor(bus, motor_id, scenarios[motor_id]))
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm_operator", lambda: (_ for _ in ()).throw(tool.SafetyViolation("not armed")))
    result = tool.run(_args(tmp_path))
    assert result == 1
    assert not [event for event in FakeBus.instances[-1].events if event[0] == "request" and event[2] == CMD_WR_REGISTER]
