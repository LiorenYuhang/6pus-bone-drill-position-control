"""No-hardware tests for Stage M4-B one-motor motion."""

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
from tools import safe_m4b_one_motor as tool  # noqa: E402


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
        self.events.append(
            ("request", parsed.motor_id, parsed.command, parsed.register_addr)
        )
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
    target_register_reads: list[int] | None = None
    mode_register_reads: list[int] | None = None
    speed_register_reads: list[int] | None = None
    writes: list[tuple[int, list[int]]] = field(default_factory=list)
    target_read_count: int = 0
    mode_read_count: int = 0
    speed_read_count: int = 0


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
        value = self.scenario.registers[address]
        if address == tool.TARGET_POSITION_ADDR:
            reads = self.scenario.target_register_reads
            if reads:
                index = min(self.scenario.target_read_count, len(reads) - 1)
                value = reads[index]
            self.scenario.target_read_count += 1
        elif address == tool.CONTROL_MODE_ADDR:
            reads = self.scenario.mode_register_reads
            if reads:
                index = min(self.scenario.mode_read_count, len(reads) - 1)
                value = reads[index]
            self.scenario.mode_read_count += 1
        elif address == tool.TARGET_SPEED_ADDR:
            reads = self.scenario.speed_register_reads
            if reads:
                index = min(self.scenario.speed_read_count, len(reads) - 1)
                value = reads[index]
            self.scenario.speed_read_count += 1
        return ReadRegisterResponse(register_addr=address, values=(value,))

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
            # The observed controller semantics synchronize target to stop actual.
            self.scenario.registers[tool.TARGET_POSITION_ADDR] = (
                self.scenario.pause_response.current_position
            )
        return self.scenario.pause_response


def _scenario(position, *, moving, bad_motion=None, post_current=0):
    registers = {
        tool.LOWER_LIMIT_ADDR: 0,
        tool.UPPER_LIMIT_ADDR: 2000,
        tool.CONTROL_MODE_ADDR: 0,
        tool.TARGET_SPEED_ADDR: 65535,
        tool.TARGET_POSITION_ADDR: position,
    }
    final = position + 8 if moving else position
    if moving:
        motion = bad_motion or [
            _status(position + 6, current=60),
            _status(final, current=30),
        ]
        statuses = [
            _status(position),
            _status(position),
            _status(position),
            *motion,
            _status(final, current=post_current),
            _status(final, current=post_current),
            _status(final, current=post_current),
        ]
    else:
        statuses = [_status(position) for _ in range(12)]
    return MotorScenario(
        registers=registers,
        statuses=statuses,
        setup_response=_status(position),
        target_response=_status(position, current=10),
        pause_response=_status(final),
    )


def _scenarios(moving_id, *, bad_motion=None, post_current=0):
    return {
        motor_id: _scenario(
            motor_id * 10,
            moving=motor_id == moving_id,
            bad_motion=bad_motion if motor_id == moving_id else None,
            post_current=post_current if motor_id == moving_id else 0,
        )
        for motor_id in tool.MOTOR_IDS
    }


def _args(tmp_path, moving_id):
    return tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--moving-id",
            str(moving_id),
            "--timeout",
            "0.05",
            "--retries",
            "2",
            "--log",
            str(tmp_path / f"m4b{moving_id}.log"),
        ]
    )


def _run(
    monkeypatch,
    tmp_path,
    moving_id,
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

    def confirm_operator(actual_moving_id):
        assert actual_moving_id == moving_id
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
    monkeypatch.setattr(tool, "BASELINE_DURATION_S", 0.04)
    monkeypatch.setattr(tool, "BASELINE_RATE_HZ", 50.0)
    monkeypatch.setattr(tool, "MOTION_TIMEOUT_S", 0.10)
    monkeypatch.setattr(tool, "STOP_CONFIRM_DURATION_S", 0.10)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    result = tool.run(_args(tmp_path, moving_id))
    return result, FakeBus.instances[0], created, messages


def _write_events(bus):
    return [
        event
        for event in bus.events
        if event[0] == "request" and event[2] == CMD_WR_REGISTER
    ]


@pytest.mark.parametrize("moving_id", tool.MOTOR_IDS)
def test_each_run_uses_one_bus_writes_and_pauses_only_moving_id(
    monkeypatch, tmp_path, moving_id
):
    scenarios = _scenarios(moving_id)
    result, bus, created, messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 0
    assert set(created) == set(tool.MOTOR_IDS)
    assert all(motor.bus is created[1].bus for motor in created.values())
    writes = _write_events(bus)
    assert [(event[1], event[3]) for event in writes] == [
        (moving_id, tool.CONTROL_MODE_ADDR),
        (moving_id, tool.TARGET_POSITION_ADDR),
        (moving_id, tool.PAUSE_ADDR),
    ]
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))
    for observer_id in set(tool.MOTOR_IDS) - {moving_id}:
        assert created[observer_id].scenario.writes == []
    assert (tool.TARGET_POSITION_ADDR, [moving_id * 10 + 10]) in (
        created[moving_id].scenario.writes
    )
    assert any(
        "ACCESS_SUMMARY" in message
        and f"write_destinations=[{moving_id}, {moving_id}, {moving_id}]" in message
        and "observer_write_count=0" in message
        for message in messages
    )
    assert any(
        "M4B_RESULT decision=PASS" in message
        and "observer_config_unchanged=true" in message
        for message in messages
    )


@pytest.mark.parametrize(
    ("frame", "moving_id", "message"),
    [
        (pack_write_register_cmd(2, tool.PAUSE_ADDR, [1]), 1, "only moving ID1"),
        (
            pack_write_register_cmd(1, tool.LOWER_LIMIT_ADDR, [0]),
            1,
            "write register",
        ),
        (pack_write_register_cmd(5, tool.PAUSE_ADDR, [1]), 1, "outside M4-B"),
    ],
)
def test_frame_audit_blocks_observer_forbidden_register_and_broadcast(
    frame, moving_id, message
):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.M4bAuditedBus(delegate, moving_id)
    with pytest.raises(tool.SafetyViolation, match=message):
        audited.request(frame, retries=0)
    assert delegate.events == []
    assert audited.write_frames == []


@pytest.mark.parametrize(
    ("bad_status", "message"),
    [
        (_status(32), "position left reference"),
        (_status(30, current=1), "current exceeded baseline maximum"),
        (_status(30, fault=1), "observer fault"),
    ],
)
def test_observer_violation_pauses_only_moving_id(
    monkeypatch, tmp_path, capsys, bad_status, message
):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    scenarios[3].statuses[5] = bad_status
    result, bus, created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert message in capsys.readouterr().err
    writes = _write_events(bus)
    assert all(event[1] == moving_id for event in writes)
    assert [event[3] for event in writes][-1] == tool.PAUSE_ADDR
    for observer_id in (2, 3, 4):
        assert created[observer_id].scenario.writes == []


def test_observer_target_jitter_within_baseline_band_passes(
    monkeypatch, tmp_path
):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    # Observer ID2 has preflight target 53, baseline status targets 52 and 53
    scenarios[2].registers[tool.TARGET_POSITION_ADDR] = 53
    scenarios[2].registers[tool.CONTROL_MODE_ADDR] = 2
    scenarios[2].registers[tool.TARGET_SPEED_ADDR] = 50
    scenarios[2].statuses = [
        _status(52),
        _status(52),
        _status(52),
        _status(53),
        _status(52),
        _status(53),
        _status(52),
        _status(53),
        _status(52),
        _status(53),
        _status(52),
        _status(53),
    ]
    # after_setup=52, after_target=53, final=52
    scenarios[2].target_register_reads = [53, 52, 53, 52]
    result, bus, _created, messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 0
    assert any("M4B_RESULT decision=PASS" in message for message in messages)


def test_observer_target_outside_baseline_band_fails(
    monkeypatch, tmp_path, capsys
):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    scenarios[2].target_register_reads = [20, 22]
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert "outside baseline target safety band" in capsys.readouterr().err
    assert [(event[1], event[3]) for event in _write_events(bus)] == [
        (moving_id, tool.CONTROL_MODE_ADDR),
        (moving_id, tool.PAUSE_ADDR),
    ]


def test_observer_mode_change_fails(monkeypatch, tmp_path, capsys):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    scenarios[2].mode_register_reads = [0, 2]
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert "observer mode changed" in capsys.readouterr().err
    assert [(event[1], event[3]) for event in _write_events(bus)] == [
        (moving_id, tool.CONTROL_MODE_ADDR),
        (moving_id, tool.PAUSE_ADDR),
    ]


def test_observer_speed_change_fails(monkeypatch, tmp_path, capsys):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    scenarios[2].speed_register_reads = [65535, 100]
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert "observer speed changed" in capsys.readouterr().err
    assert [(event[1], event[3]) for event in _write_events(bus)] == [
        (moving_id, tool.CONTROL_MODE_ADDR),
        (moving_id, tool.PAUSE_ADDR),
    ]


def test_observer_write_frame_blocked_by_audit():
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.M4bAuditedBus(delegate, moving_id=1)
    frame = pack_write_register_cmd(2, tool.TARGET_POSITION_ADDR, [50])
    with pytest.raises(tool.SafetyViolation, match="only moving ID1 is writable"):
        audited.request(frame, retries=0)
    assert delegate.events == []
    assert audited.write_frames == []


@pytest.mark.parametrize(
    ("bad_motion", "message"),
    [
        ([_status(9)], "negative-direction movement"),
        ([_status(10, current=501)], "exceeds 500mA"),
        ([_status(10, fault=1)], "fault code=0x01"),
    ],
)
def test_moving_motor_safety_violation_pauses_only_it(
    monkeypatch, tmp_path, capsys, bad_motion, message
):
    moving_id = 1
    result, bus, _created, _messages = _run(
        monkeypatch,
        tmp_path,
        moving_id,
        _scenarios(moving_id, bad_motion=bad_motion),
    )
    assert result == 1
    assert message in capsys.readouterr().err
    writes = _write_events(bus)
    assert [event[1] for event in writes] == [moving_id, moving_id, moving_id]
    assert writes[-1][3] == tool.PAUSE_ADDR


def test_target_readback_mismatch_fails_before_motion_and_pauses_moving_id(
    monkeypatch, tmp_path, capsys
):
    moving_id = 2
    scenarios = _scenarios(moving_id)
    scenarios[moving_id].target_readback_override = 29
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert "target write readback mismatch" in capsys.readouterr().err
    assert [(event[1], event[3]) for event in _write_events(bus)] == [
        (moving_id, tool.CONTROL_MODE_ADDR),
        (moving_id, tool.TARGET_POSITION_ADDR),
        (moving_id, tool.PAUSE_ADDR),
    ]


def test_pause_failure_requires_hard_stop_and_never_pauses_observers(
    monkeypatch, tmp_path, capsys
):
    moving_id = 3
    result, bus, created, _messages = _run(
        monkeypatch,
        tmp_path,
        moving_id,
        _scenarios(moving_id),
        fail_pause_id=moving_id,
    )
    assert result == 1
    assert "HARD STOP REQUIRED" in capsys.readouterr().err
    pauses = [event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR]
    assert [event[1] for event in pauses] == [moving_id]
    for observer_id in (1, 2, 4):
        assert created[observer_id].scenario.writes == []


def test_stop_confirmation_failure_requires_hard_stop(monkeypatch, tmp_path, capsys):
    moving_id = 4
    result, bus, _created, _messages = _run(
        monkeypatch,
        tmp_path,
        moving_id,
        _scenarios(moving_id, post_current=1),
    )
    assert result == 1
    assert "HARD STOP REQUIRED" in capsys.readouterr().err
    pauses = [event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR]
    assert [event[1] for event in pauses] == [moving_id]


def test_operator_abort_has_zero_writes(monkeypatch, tmp_path):
    moving_id = 1
    result, bus, _created, messages = _run(
        monkeypatch,
        tmp_path,
        moving_id,
        _scenarios(moving_id),
        operator_error="operator did not arm Stage M4-B",
    )
    assert result == 1
    assert _write_events(bus) == []
    assert any("write_count=0" in message for message in messages)


def test_pause_not_triggered_when_error_is_three(monkeypatch, tmp_path, capsys):
    moving_id = 1
    # baseline is 10, target is 20; motion only reaches 17 (error=3), does not reach error <= 2
    scenarios = _scenarios(
        moving_id,
        bad_motion=[_status(17, current=40) for _ in range(10)],
    )
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert (
        "did not confirm positive motion and reach +/-2 step"
        in capsys.readouterr().err
    )


def test_pause_triggered_when_error_is_two(monkeypatch, tmp_path):
    moving_id = 1
    # baseline is 10, target is 20; motion reaches 18 (error=2)
    scenarios = _scenarios(moving_id, bad_motion=[_status(18, current=30)])
    result, bus, _created, messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 0
    assert any("M4B_RESULT decision=PASS" in message for message in messages)


def test_post_pause_final_error_three_passes(monkeypatch, tmp_path):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    # reaches 18 (error=2) during motion, pause sent, settles at 17 (final_error=3)
    scenarios[1].statuses = [
        _status(10),
        _status(10),
        _status(10),
        _status(18, current=30),
        _status(17),
        _status(17),
        _status(17),
    ]
    scenarios[1].pause_response = _status(17)
    result, bus, _created, messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 0
    assert any(
        "M4B_RESULT decision=PASS" in message and "final_actual=17" in message
        for message in messages
    )


def test_post_pause_final_error_four_fails(monkeypatch, tmp_path, capsys):
    moving_id = 1
    scenarios = _scenarios(moving_id)
    # reaches 18 (error=2) during motion, pause sent, settles at 16 (final_error=4)
    scenarios[1].statuses = [
        _status(10),
        _status(10),
        _status(10),
        _status(18, current=30),
        _status(16),
        _status(16),
        _status(16),
    ]
    scenarios[1].pause_response = _status(16)
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, moving_id, scenarios
    )
    assert result == 1
    assert "post-pause final_error=4 exceeds 3 step" in capsys.readouterr().err


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 41

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(
        ["--port", "/dev/ttyTEST", "--moving-id", "1", "--log", "m4b.log"]
    ) == 41
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "moving_id": 1,
        "timeout": 0.05,
        "retries": 2,
        "log": "m4b.log",
    }
    assert tool.MOTOR_IDS == (1, 2, 3, 4)
    assert tool.MOVE_STEPS == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500

    source = (ROOT / "tools" / "safe_m4b_one_motor.py").read_text(
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


@pytest.mark.parametrize(
    "argv",
    [
        ["--port", "/dev/tty", "--moving-id", "0"],
        ["--port", "/dev/tty", "--moving-id", "5"],
        ["--port", "/dev/tty", "--moving-id", "1", "--timeout", "0"],
        ["--port", "/dev/tty", "--moving-id", "1", "--retries", "-1"],
    ],
)
def test_cli_rejects_out_of_scope_id_and_invalid_transport_arguments(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
