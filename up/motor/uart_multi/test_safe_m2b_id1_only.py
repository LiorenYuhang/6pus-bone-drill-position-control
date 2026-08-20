"""No-hardware tests for Stage M2-B ID1-only motion."""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
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
from tools import safe_m2b_id1_only as tool  # noqa: E402


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
        self.fail_register = None
        type(self).instances.append(self)

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        self.events.append(
            ("request", parsed.motor_id, parsed.command, parsed.register_addr)
        )
        if (
            parsed.command == CMD_WR_REGISTER
            and parsed.register_addr == self.fail_register
        ):
            raise OSError("simulated write failure")
        return b""

    def close(self):
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


@dataclass
class MotorScenario:
    registers: dict[int, int]
    statuses: list[StatusFeedback]
    setup_response: StatusFeedback
    target_response: StatusFeedback
    pause_response: StatusFeedback
    writes: list[tuple[int, list[int]]] = field(default_factory=list)


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

    def write_register(self, address, data):
        values = list(data)
        self.bus.request(
            pack_write_register_cmd(self.motor_id, address, values),
            expected_id=self.motor_id,
            retries=0,
        )
        self.scenario.writes.append((address, values))
        if address == tool.CONTROL_MODE_ADDR:
            return self.scenario.setup_response
        if address == tool.TARGET_POSITION_ADDR:
            return self.scenario.target_response
        return self.scenario.pause_response


def _scenarios(*, id1_poll=None, id2_poll=None):
    base_registers = {
        tool.LOWER_LIMIT_ADDR: 0,
        tool.UPPER_LIMIT_ADDR: 2000,
        tool.CONTROL_MODE_ADDR: 0,
        tool.TARGET_SPEED_ADDR: 65535,
        tool.TARGET_POSITION_ADDR: 15,
    }
    id1_poll = id1_poll or [_status(20, current=70), _status(23, current=40)]
    id2_poll = id2_poll or [_status(30), _status(30)]
    scenario1 = MotorScenario(
        registers=dict(base_registers),
        statuses=[_status(15), _status(15), _status(15), *id1_poll, _status(23)],
        setup_response=_status(15),
        target_response=_status(15, current=10),
        pause_response=_status(23),
    )
    registers2 = dict(base_registers)
    registers2[tool.TARGET_POSITION_ADDR] = 30
    scenario2 = MotorScenario(
        registers=registers2,
        statuses=[
            _status(30),
            _status(30),
            _status(30),
            _status(30),
            _status(30),
            *id2_poll,
            _status(30),
        ],
        setup_response=_status(30),
        target_response=_status(30),
        pause_response=_status(30),
    )
    return {1: scenario1, 2: scenario2}


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
            str(tmp_path / "m2b.log"),
        ]
    )


def _run(monkeypatch, tmp_path, scenarios, *, fail_register=None):
    created = {}
    messages = []

    def bus_factory(port, *, timeout, retries):
        bus = FakeBus(port, timeout=timeout, retries=retries)
        bus.fail_register = fail_register
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
    monkeypatch.setattr(tool, "BASELINE_DURATION_S", 0.04)
    monkeypatch.setattr(tool, "BASELINE_RATE_HZ", 50.0)
    monkeypatch.setattr(tool, "MOTION_TIMEOUT_S", 0.10)
    monkeypatch.setattr(tool, "STOP_CONFIRM_DURATION_S", 0.10)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    result = tool.run(_args(tmp_path))
    return result, FakeBus.instances[0], created, messages


def _write_events(bus):
    return [
        event
        for event in bus.events
        if event[0] == "request" and event[2] == CMD_WR_REGISTER
    ]


def test_m2b_success_uses_one_bus_writes_only_id1_and_logs_read_write_split(
    monkeypatch, tmp_path
):
    result, bus, created, messages = _run(monkeypatch, tmp_path, _scenarios())
    assert result == 0
    assert set(created) == {1, 2}
    assert all(motor.bus is created[1].bus for motor in created.values())
    writes = _write_events(bus)
    assert [(event[1], event[3]) for event in writes] == [
        (1, tool.CONTROL_MODE_ADDR),
        (1, tool.TARGET_POSITION_ADDR),
        (1, tool.PAUSE_ADDR),
    ]
    assert all(event[1] == 1 for event in writes)
    assert any(event[0] == "request" and event[1] == 2 for event in bus.events)
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))
    assert any(
        "ACCESS_SUMMARY" in message
        and "write_destinations=[1, 1, 1]" in message
        and "id2_write_count=0" in message
        for message in messages
    )
    assert any(
        "M2B_RESULT decision=PASS" in message
        and "ID2_config_unchanged=true" in message
        for message in messages
    )
    assert created[2].scenario.writes == []


def test_frame_audit_blocks_id2_write_before_underlying_bus_request():
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.Id1OnlyAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation, match="only ID1 is writable"):
        audited.request(
            pack_write_register_cmd(2, tool.TARGET_POSITION_ADDR, [40]),
            expected_id=2,
            retries=0,
        )
    assert delegate.events == []
    assert audited.write_frames == []


@pytest.mark.parametrize(
    ("bad_status", "message"),
    [
        (_status(31), "position left baseline band"),
        (_status(30, current=1), "current exceeded baseline maximum"),
        (_status(30, fault=1), "observer fault"),
    ],
)
def test_id2_position_current_or_fault_aborts_with_id1_pause_and_no_id2_write(
    monkeypatch, tmp_path, capsys, bad_status, message
):
    scenarios = _scenarios(id2_poll=[bad_status, _status(30)])
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert message in capsys.readouterr().err
    writes = _write_events(bus)
    assert all(event[1] == 1 for event in writes)
    assert [event[3] for event in writes][-1] == tool.PAUSE_ADDR
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))
    assert created[2].scenario.writes == []
    assert any("id2_write_count=0" in item for item in messages)


def test_id1_abnormal_current_pauses_before_close_without_id2_write(
    monkeypatch, tmp_path
):
    scenarios = _scenarios(id1_poll=[_status(15, current=501), _status(15)])
    result, bus, created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    writes = _write_events(bus)
    assert [event[3] for event in writes][-1] == tool.PAUSE_ADDR
    assert all(event[1] == 1 for event in writes)
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))
    assert created[2].scenario.writes == []


def test_pause_failure_prints_hard_stop_and_never_attempts_id2_pause(
    monkeypatch, tmp_path, capsys
):
    scenarios = _scenarios(id1_poll=[_status(15, current=501), _status(15)])
    result, bus, created, _messages = _run(
        monkeypatch, tmp_path, scenarios, fail_register=tool.PAUSE_ADDR
    )
    assert result == 1
    assert "HARD STOP REQUIRED" in capsys.readouterr().err
    writes = _write_events(bus)
    assert [event[3] for event in writes].count(tool.PAUSE_ADDR) == 1
    assert all(event[1] == 1 for event in writes)
    assert created[2].scenario.writes == []
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))


def test_cli_is_fixed_scope_and_source_has_no_id_or_flash_operations(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 19

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(["--port", "/dev/ttyTEST", "--log", "m2b.log"]) == 19
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "timeout": 0.05,
        "retries": 2,
        "log": "m2b.log",
    }
    assert tool.MOVE_STEPS == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500

    source_path = ROOT / "tools" / "safe_m2b_id1_only.py"
    source = source_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "motor2.write_register" not in source
    assert called_attributes.isdisjoint(
        {"set_motor_id", "save_parameters", "set_position", "set_speed", "move"}
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
