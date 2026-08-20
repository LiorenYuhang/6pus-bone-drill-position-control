"""No-hardware tests for Stage M6-A six-motor read-only review."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import logging
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
from tools import safe_m6a_readonly_bringup as tool  # noqa: E402


def _status(position, current=0, force=-400, temperature=25, fault=0):
    return StatusFeedback(
        target_position=position,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=100,
        temperature=temperature,
        error_code=fault,
    )


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
        event = (
            "request",
            parsed.motor_id,
            parsed.command,
            parsed.register_addr,
        )
        self.events.append(event)
        return b""

    def close(self):
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


@dataclass
class MotorScenario:
    registers: dict[int, int]
    statuses: list[StatusFeedback]


class FakeMotor:
    def __init__(self, bus, motor_id, scenario):
        self.bus = bus
        self.motor_id = motor_id
        self.scenario = scenario
        self._status_index = 0

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        return ReadRegisterResponse(
            register_addr=address,
            values=(self.scenario.registers[address],),
        )

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        index = min(self._status_index, len(self.scenario.statuses) - 1)
        self._status_index += 1
        return self.scenario.statuses[index]


def _scenarios(*, override_id=None, override_scenario=None):
    positions = {1: 59, 2: 67, 3: 23, 4: 54, 5: 10, 6: 10}
    scenarios = {}
    for motor_id in tool.MOTOR_IDS:
        pos = positions.get(motor_id, 10)
        registers = {
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.CONTROL_MODE_ADDR: 2,
            tool.TARGET_SPEED_ADDR: 50,
            tool.TARGET_POSITION_ADDR: pos,
        }
        statuses = [_status(pos) for _ in range(50)]
        scenarios[motor_id] = MotorScenario(registers=registers, statuses=statuses)
    if override_id and override_scenario:
        scenarios[override_id] = override_scenario
    return scenarios


def _run(monkeypatch, tmp_path, scenarios, *, samples=5):
    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool.time, "sleep", lambda s: None)
    monkeypatch.setattr(tool, "_configure_logging", lambda p: None)

    created_motors = {}

    def fake_motor_factory(bus, motor_id):
        created = FakeMotor(bus, motor_id, scenarios[motor_id])
        created_motors[motor_id] = created
        return created

    monkeypatch.setattr(tool, "Motor", fake_motor_factory)

    messages = []
    monkeypatch.setattr(tool.logger, "info", lambda msg, *args: messages.append(msg % args if args else msg))
    monkeypatch.setattr(tool.logger, "error", lambda msg, *args: messages.append(msg % args if args else msg))

    log_file = tmp_path / "m6a_test.log"
    args = tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--samples",
            str(samples),
            "--log",
            str(log_file),
        ]
    )
    result = tool.run(args)
    bus = FakeBus.instances[0]
    return result, bus, created_motors, messages


def test_m6a_success_reads_all_six_motors_zero_writes(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios, samples=3)
    assert result == 0
    assert any("M6A_RESULT decision=PASS" in m for m in messages)
    assert len(created) == 6
    # Verify zero writes across all events
    write_events = [
        e for e in bus.events if e[0] == "request" and e[2] == CMD_WR_REGISTER
    ]
    assert len(write_events) == 0


@pytest.mark.parametrize(
    ("frame", "match"),
    [
        (
            pack_write_register_cmd(1, tool.CONTROL_MODE_ADDR, [2]),
            "strictly read-only",
        ),
        (
            pack_read_register_cmd(7, tool.LOWER_LIMIT_ADDR),
            "outside allowed scope",
        ),
        (
            pack_read_register_cmd(1, 0x1A),  # Pause is write-only, not allowed for read in M6A
            "is not allowed",
        ),
    ],
)
def test_audited_bus_strictly_blocks_writes_and_invalid_reads(frame, match):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.M6aReadOnlyAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation, match=match):
        audited.request(frame)
    assert delegate.events == []


def test_m6a_aborts_on_abnormal_limits(monkeypatch, tmp_path, capsys):
    bad_scenario = MotorScenario(
        registers={
            tool.LOWER_LIMIT_ADDR: -1,  # abnormal limit
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.CONTROL_MODE_ADDR: 0,
            tool.TARGET_SPEED_ADDR: 65535,
            tool.TARGET_POSITION_ADDR: 10,
        },
        statuses=[_status(10)],
    )
    scenarios = _scenarios(override_id=5, override_scenario=bad_scenario)
    result, bus, _created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert "must equal [0,2000]" in capsys.readouterr().err


def test_m6a_aborts_on_overcurrent(monkeypatch, tmp_path, capsys):
    bad_scenario = MotorScenario(
        registers={
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.CONTROL_MODE_ADDR: 0,
            tool.TARGET_SPEED_ADDR: 65535,
            tool.TARGET_POSITION_ADDR: 10,
        },
        statuses=[_status(10, current=600)],  # >500mA
    )
    scenarios = _scenarios(override_id=6, override_scenario=bad_scenario)
    result, bus, _created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert "exceeds 500mA" in capsys.readouterr().err


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers():
    assert tool.MOTOR_IDS == (1, 2, 3, 4, 5, 6)
    assert tool.DEFAULT_SAMPLE_COUNT == 100
    assert tool.MAX_CURRENT_MA == 500

    source = (ROOT / "tools" / "safe_m6a_readonly_bringup.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint(
        {"set_motor_id", "save_parameters", "set_position", "set_speed", "move"}
    )
