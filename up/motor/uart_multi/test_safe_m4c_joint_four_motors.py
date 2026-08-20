"""No-hardware tests for Stage M4-C joint four-motor motion."""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
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
from tools import safe_m4c_joint_four_motors as tool  # noqa: E402


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
        self.fail_pause_id = None
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
        if (
            parsed.command == CMD_WR_REGISTER
            and parsed.register_addr == tool.PAUSE_ADDR
            and parsed.motor_id == self.fail_pause_id
        ):
            raise OSError("simulated pause failure")
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
    target_readback_override: int | None = None
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
            self.scenario.registers[tool.CONTROL_MODE_ADDR] = values[0]
            self.scenario.registers[tool.TARGET_SPEED_ADDR] = values[3]
            self.scenario.registers[tool.TARGET_POSITION_ADDR] = values[4]
            return self.scenario.setup_response
        if address == tool.TARGET_POSITION_ADDR:
            self.scenario.registers[tool.TARGET_POSITION_ADDR] = values[0]
            if self.scenario.target_readback_override is not None:
                self.scenario.registers[tool.TARGET_POSITION_ADDR] = (
                    self.scenario.target_readback_override
                )
            return self.scenario.target_response
        if address == tool.PAUSE_ADDR:
            self.scenario.registers[tool.TARGET_POSITION_ADDR] = (
                self.scenario.pause_response.current_position
            )
        return self.scenario.pause_response


def _scenario(
    baseline,
    final,
    *,
    motion_statuses,
    post_current=0,
    target_readback_override=None,
    baseline_positions=None,
):
    registers = {
        tool.LOWER_LIMIT_ADDR: 0,
        tool.UPPER_LIMIT_ADDR: 2000,
        tool.CONTROL_MODE_ADDR: 0,
        tool.TARGET_SPEED_ADDR: 65535,
        tool.TARGET_POSITION_ADDR: baseline,
    }
    baseline_samples = baseline_positions or [baseline, baseline]
    statuses = [
        _status(baseline),
        *[_status(position) for position in baseline_samples],
        *motion_statuses,
        _status(final, current=post_current),
        _status(final, current=post_current),
        _status(final, current=post_current),
    ]
    return MotorScenario(
        registers=registers,
        statuses=statuses,
        setup_response=_status(baseline),
        target_response=_status(baseline, current=10),
        pause_response=_status(final),
        target_readback_override=target_readback_override,
    )


def _scenarios(
    *,
    override_motor_id=None,
    override_scenario=None,
):
    base = {
        1: _scenario(
            50,
            58,
            motion_statuses=[_status(54, current=60), _status(58, current=30)],
        ),
        2: _scenario(
            60,
            68,
            motion_statuses=[_status(64, current=55), _status(68, current=25)],
        ),
        3: _scenario(
            15,
            23,
            motion_statuses=[_status(19, current=45), _status(23, current=20)],
        ),
        4: _scenario(
            45,
            53,
            motion_statuses=[_status(49, current=50), _status(53, current=35)],
        ),
    }
    if override_motor_id and override_scenario:
        base[override_motor_id] = override_scenario
    return base


def _write_events(bus):
    return [
        event
        for event in bus.events
        if event[0] == "request" and event[2] == CMD_WR_REGISTER
    ]


def _run(
    monkeypatch,
    tmp_path,
    scenarios,
    *,
    fail_pause_id=None,
    operator_error=None,
):
    created = {}
    messages = []

    def bus_factory(port, *, timeout, retries):
        bus = FakeBus(port, timeout=timeout, retries=retries)
        bus.fail_pause_id = fail_pause_id
        return bus

    def motor_factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id, scenarios[motor_id])
        created[motor_id] = motor
        return motor

    def confirm_operator():
        if operator_error is not None:
            raise tool.SafetyViolation(operator_error)

    def capture(message, *args):
        messages.append(message % args if args else message)

    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", bus_factory)
    monkeypatch.setattr(tool, "Motor", motor_factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm_operator", confirm_operator)
    monkeypatch.setattr(tool.logger, "info", capture)
    monkeypatch.setattr(tool.logger, "warning", capture)
    monkeypatch.setattr(tool.logger, "error", capture)
    monkeypatch.setattr(tool.logger, "critical", capture)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    monkeypatch.setattr(tool, "BASELINE_DURATION_S", 0.04)
    monkeypatch.setattr(tool, "BASELINE_RATE_HZ", 50.0)
    monkeypatch.setattr(tool, "POLL_INTERVAL_S", 0.01)
    monkeypatch.setattr(tool, "STOP_CONFIRM_DURATION_S", 0.02)

    log_file = tmp_path / "m4c_test.log"
    args = tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--timeout",
            "0.05",
            "--retries",
            "2",
            "--log",
            str(log_file),
        ]
    )
    result = tool.run(args)
    bus = FakeBus.instances[0]
    return result, bus, created, messages


def test_m4c_success_uses_one_bus_and_pauses_all(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 0
    assert any("M4C_RESULT decision=PASS" in m for m in messages)
    assert len(FakeBus.instances) == 1
    writes = _write_events(bus)
    # 4 setups + 4 target writes + 4 pauses = 12 writes
    assert len(writes) == 12
    assert [w[1] for w in writes] == [1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]
    assert [w[3] for w in writes[:4]] == [tool.CONTROL_MODE_ADDR] * 4
    assert [w[3] for w in writes[4:8]] == [tool.TARGET_POSITION_ADDR] * 4
    assert [w[3] for w in writes[8:]] == [tool.PAUSE_ADDR] * 4


@pytest.mark.parametrize(
    ("frame", "match"),
    [
        (
            pack_write_register_cmd(5, tool.PAUSE_ADDR, [1]),
            "outside allowed scope",
        ),
        (
            pack_write_register_cmd(1, tool.LOWER_LIMIT_ADDR, [0]),
            "is not allowed",
        ),
    ],
)
def test_audited_bus_blocks_invalid_destination_and_register(frame, match):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.M4cAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation, match=match):
        audited.request(frame)
    assert delegate.events == []


def test_motor_runtime_fault_pauses_all_motors(monkeypatch, tmp_path, capsys):
    bad = _scenario(
        50,
        58,
        motion_statuses=[_status(54, current=600)],  # overcurrent >500mA
    )
    scenarios = _scenarios(override_motor_id=1, override_scenario=bad)
    result, bus, _created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert "exceeds 500mA" in capsys.readouterr().err
    pauses = [e for e in _write_events(bus) if e[3] == tool.PAUSE_ADDR]
    assert [p[1] for p in pauses] == [1, 2, 3, 4]


def test_target_readback_mismatch_fails_and_pauses_all(monkeypatch, tmp_path, capsys):
    bad = _scenario(
        60,
        68,
        motion_statuses=[_status(64, current=55)],
        target_readback_override=99,
    )
    scenarios = _scenarios(override_motor_id=2, override_scenario=bad)
    result, bus, _created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert "target write readback mismatch" in capsys.readouterr().err
    pauses = [e for e in _write_events(bus) if e[3] == tool.PAUSE_ADDR]
    assert [p[1] for p in pauses] == [1, 2, 3, 4]


def test_pause_failure_requires_hard_stop(monkeypatch, tmp_path, capsys):
    scenarios = _scenarios()
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, scenarios, fail_pause_id=1
    )
    assert result == 1
    err = capsys.readouterr().err
    assert "HARD STOP REQUIRED" in err
    assert "completion pause failed" in err
    # Verify best-effort: ID1 failed, but ID2, ID3, ID4 were still attempted
    pauses = [e for e in _write_events(bus) if e[3] == tool.PAUSE_ADDR]
    assert [p[1] for p in pauses] == [1, 2, 3, 4]


def test_setup_phase_exception_triggers_pause_all(monkeypatch, tmp_path, capsys):
    bad_setup = _scenario(
        60,
        68,
        motion_statuses=[_status(64, current=55)],
    )
    bad_setup.setup_response = _status(60, current=600)  # overcurrent in setup
    scenarios = _scenarios(override_motor_id=2, override_scenario=bad_setup)
    result, bus, _created, _messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert "exceeds 500mA" in capsys.readouterr().err
    # Armed was True before ID1 setup, so ID2 setup failure pauses all 4
    pauses = [e for e in _write_events(bus) if e[3] == tool.PAUSE_ADDR]
    assert [p[1] for p in pauses] == [1, 2, 3, 4]


def test_operator_abort_has_zero_writes(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, _created, messages = _run(
        monkeypatch,
        tmp_path,
        scenarios,
        operator_error="operator did not arm Stage M4-C",
    )
    assert result == 1
    assert _write_events(bus) == []
    assert any("write_count=0" in m for m in messages)


def test_final_error_three_passes_and_four_fails(monkeypatch, tmp_path, capsys):
    # Test error=3 passes (motion reaches 58 with error=2, settles at 57 with final_error=3)
    s3 = _scenario(
        50,
        57,  # target=60, final=57, error=3
        motion_statuses=[_status(54, current=60), _status(58, current=30)],
    )
    result3, _, _, _ = _run(
        monkeypatch, tmp_path, _scenarios(override_motor_id=1, override_scenario=s3)
    )
    assert result3 == 0

    # Test error=4 fails (motion reaches 58 with error=2, settles at 56 with final_error=4)
    s4 = _scenario(
        50,
        56,  # target=60, final=56, error=4
        motion_statuses=[_status(54, current=60), _status(58, current=30)],
    )
    result4, _, _, _ = _run(
        monkeypatch, tmp_path, _scenarios(override_motor_id=1, override_scenario=s4)
    )
    assert result4 == 1
    assert "post-pause final_error=4 exceeds 3 step" in capsys.readouterr().err


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers(monkeypatch):
    assert tool.MOTOR_IDS == (1, 2, 3, 4)
    assert tool.MOVE_STEPS == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500
    assert tool.PAUSE_TRIGGER_TOLERANCE_STEPS == 2
    assert tool.FINAL_ACCEPTANCE_TOLERANCE_STEPS == 3

    source = (ROOT / "tools" / "safe_m4c_joint_four_motors.py").read_text(
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
