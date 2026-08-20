"""No-hardware tests for Stage M2-D joint ID1/ID2 motion."""

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
from tools import safe_m2d_joint_id1_id2 as tool  # noqa: E402


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
            # Model the controller's observed pause behavior: target follows
            # the actual stop position after the pause command.
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
    id1_motion=None,
    id2_motion=None,
    id1_post_current=0,
    id2_post_current=0,
    id1_target_readback_override=None,
    id2_target_readback_override=None,
):
    return {
        1: _scenario(
            10,
            18,
            motion_statuses=id1_motion
            or [_status(16, current=70), _status(18, current=40)],
            post_current=id1_post_current,
            target_readback_override=id1_target_readback_override,
        ),
        2: _scenario(
            20,
            28,
            motion_statuses=id2_motion
            or [_status(27, current=60), _status(28, current=35)],
            post_current=id2_post_current,
            target_readback_override=id2_target_readback_override,
        ),
    }


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
            str(tmp_path / "m2d.log"),
        ]
    )


def _run(
    monkeypatch,
    tmp_path,
    scenarios,
    *,
    fail_pause_id=None,
    operator_error=None,
    baseline_sample_count=2,
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
    monkeypatch.setattr(
        tool,
        "BASELINE_DURATION_S",
        baseline_sample_count / 50.0,
    )
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


def test_baseline_accepts_unique_mode_with_full_plus_minus_one_band():
    positions = [37] * 12 + [38] * 87 + [39]
    baseline = tool._build_m2d_position_baseline(positions)
    assert baseline.reference == 38
    assert baseline.safety_min == 37
    assert baseline.safety_max == 39
    assert baseline.histogram == {37: 12, 38: 87, 39: 1}


@pytest.mark.parametrize(
    ("positions", "expected_min", "expected_max"),
    [
        ([37, 38, 38], 37, 38),
        ([38, 38, 39], 38, 39),
    ],
)
def test_baseline_accepts_one_sided_quantization_band(
    positions, expected_min, expected_max
):
    baseline = tool._build_m2d_position_baseline(positions)
    assert baseline.reference == 38
    assert baseline.safety_min == expected_min
    assert baseline.safety_max == expected_max


def test_baseline_rejects_sample_two_steps_from_unique_mode():
    with pytest.raises(tool.SafetyViolation, match="at least 2 steps"):
        tool._build_m2d_position_baseline([36, 38, 38, 38, 39])


@pytest.mark.parametrize("positions", [[37, 38], [38, 39]])
def test_baseline_rejects_mode_tie(positions):
    with pytest.raises(tool.SafetyViolation, match="no unique mode"):
        tool._build_m2d_position_baseline(positions)


def test_direction_boundaries_use_observed_safety_min_and_max():
    baseline = tool._build_m2d_position_baseline(
        [37] * 12 + [38] * 87 + [39]
    )
    guard = tool.BaselineBandGuard(
        baseline_min=baseline.safety_min,
        baseline_max=baseline.safety_max,
    )
    guard.observe(37)
    guard.observe(39)
    with pytest.raises(tool.SafetyViolation, match="negative-direction movement"):
        guard.observe(36)

    trace = tool.MotionTrace(
        baseline_min=baseline.safety_min,
        baseline_max=baseline.safety_max,
        target=baseline.reference + tool.MOVE_STEPS,
        force_f0=0.0,
        t0_mono=0.0,
        t0_wall="test",
        samples=[],
    )
    trace.observe(_status(39), error=9)
    assert trace.first_motion_mono is None
    trace.observe(_status(40), error=8)
    assert trace.first_motion_mono is not None


def test_joint_run_targets_unique_mode_plus_ten_and_logs_safety_band(
    monkeypatch, tmp_path
):
    id1_positions = [37] * 12 + [38] * 87 + [39]
    scenarios = {
        1: _scenario(
            38,
            46,
            baseline_positions=id1_positions,
            motion_statuses=[
                _status(39, current=20),
                _status(40, current=50),
                _status(46, current=30),
            ],
        ),
        2: _scenario(
            20,
            28,
            baseline_positions=[20] * 100,
            motion_statuses=[
                _status(27, current=60),
                _status(28, current=35),
            ],
        ),
    }
    result, _bus, created, messages = _run(
        monkeypatch,
        tmp_path,
        scenarios,
        baseline_sample_count=100,
    )
    assert result == 0
    assert (tool.TARGET_POSITION_ADDR, [48]) in created[1].scenario.writes
    assert any(
        "M2D_PRECHECK ID1" in message
        and "baseline_reference=38" in message
        and "baseline_safety_min=37" in message
        and "baseline_safety_max=39" in message
        and "target=48" in message
        for message in messages
    )


def test_m2d_success_uses_one_bus_audits_both_ids_and_pauses_before_close(
    monkeypatch, tmp_path
):
    result, bus, created, messages = _run(
        monkeypatch, tmp_path, _scenarios()
    )
    assert result == 0
    assert set(created) == {1, 2}
    assert all(motor.bus is created[1].bus for motor in created.values())
    writes = _write_events(bus)
    assert [(event[1], event[3]) for event in writes] == [
        (1, tool.CONTROL_MODE_ADDR),
        (2, tool.CONTROL_MODE_ADDR),
        (1, tool.TARGET_POSITION_ADDR),
        (2, tool.TARGET_POSITION_ADDR),
        (1, tool.PAUSE_ADDR),
        (2, tool.PAUSE_ADDR),
    ]
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))
    assert any(
        "ACCESS_SUMMARY" in message
        and "write_destinations=[1, 2, 1, 2, 1, 2]" in message
        and "other_id_write_count=0" in message
        for message in messages
    )
    assert any("M2D_RESULT decision=PASS" in message for message in messages)
    assert any(
        "ID1_POST_PAUSE_STATE" in message
        and "readback_target_position=18" in message
        and "target_sync_observation_only=true" in message
        for message in messages
    )
    assert any(
        "ID2_POST_PAUSE_STATE" in message
        and "readback_target_position=28" in message
        and "target_sync_observation_only=true" in message
        for message in messages
    )


def test_target_readback_is_verified_before_motion_and_pause_is_still_attempted(
    monkeypatch, tmp_path, capsys
):
    result, bus, _created, _messages = _run(
        monkeypatch,
        tmp_path,
        _scenarios(id1_target_readback_override=31),
    )
    assert result == 1
    assert "target write readback mismatch" in capsys.readouterr().err
    pause_writes = [
        event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR
    ]
    assert [event[1] for event in pause_writes] == [1, 2]


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (
            pack_write_register_cmd(3, tool.PAUSE_ADDR, [1]),
            "outside M2-D ID1/ID2 scope",
        ),
        (
            pack_write_register_cmd(1, tool.LOWER_LIMIT_ADDR, [0]),
            "write register",
        ),
    ],
)
def test_frame_audit_blocks_out_of_scope_destination_or_register(frame, message):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.JointAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation, match=message):
        audited.request(frame, retries=0)
    assert delegate.events == []
    assert audited.write_frames == []


@pytest.mark.parametrize(
    ("scenario_kwargs", "message"),
    [
        ({"id1_motion": [_status(9)]}, "negative-direction movement"),
        ({"id2_motion": [_status(20, current=501)]}, "exceeds 500mA"),
        ({"id2_motion": [_status(20, fault=1)]}, "fault code=0x01"),
    ],
)
def test_either_motor_runtime_fault_pauses_both_once_before_close(
    monkeypatch, tmp_path, capsys, scenario_kwargs, message
):
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, _scenarios(**scenario_kwargs)
    )
    assert result == 1
    assert message in capsys.readouterr().err
    pause_writes = [
        event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR
    ]
    assert [(event[1], event[3]) for event in pause_writes] == [
        (1, tool.PAUSE_ADDR),
        (2, tool.PAUSE_ADDR),
    ]
    assert bus.events.index(pause_writes[-1]) < bus.events.index(("close",))


def test_one_pause_failure_still_attempts_other_pause_once_and_requires_hard_stop(
    monkeypatch, tmp_path, capsys
):
    result, bus, _created, _messages = _run(
        monkeypatch, tmp_path, _scenarios(), fail_pause_id=1
    )
    assert result == 1
    assert "HARD STOP REQUIRED" in capsys.readouterr().err
    pause_writes = [
        event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR
    ]
    assert [event[1] for event in pause_writes] == [1, 2]
    assert bus.events.index(pause_writes[-1]) < bus.events.index(("close",))


def test_stop_confirmation_failure_requires_hard_stop(monkeypatch, tmp_path, capsys):
    result, bus, _created, _messages = _run(
        monkeypatch,
        tmp_path,
        _scenarios(id1_post_current=1),
    )
    assert result == 1
    assert "HARD STOP REQUIRED" in capsys.readouterr().err
    pause_writes = [
        event for event in _write_events(bus) if event[3] == tool.PAUSE_ADDR
    ]
    assert [event[1] for event in pause_writes] == [1, 2]
    assert bus.events.index(pause_writes[-1]) < bus.events.index(("close",))


def test_operator_abort_has_no_writes(monkeypatch, tmp_path):
    result, bus, _created, messages = _run(
        monkeypatch,
        tmp_path,
        _scenarios(),
        operator_error="operator did not arm Stage M2-D",
    )
    assert result == 1
    assert _write_events(bus) == []
    assert any("write_count=0" in message for message in messages)


def test_cli_is_fixed_scope_and_source_has_no_id_flash_or_motion_helpers(
    monkeypatch,
):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 29

    monkeypatch.setattr(tool, "run", fake_run)
    assert tool.main(["--port", "/dev/ttyTEST", "--log", "m2d.log"]) == 29
    assert vars(captured["args"]) == {
        "port": "/dev/ttyTEST",
        "timeout": 0.05,
        "retries": 2,
        "log": "m2d.log",
    }
    assert tool.MOVE_STEPS == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500
    assert tool.MOTOR_IDS == (1, 2)

    source_path = ROOT / "tools" / "safe_m2d_joint_id1_id2.py"
    source = source_path.read_text(encoding="utf-8")
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
        ["--port", "/dev/tty", "--timeout", "0"],
        ["--port", "/dev/tty", "--retries", "-1"],
    ],
)
def test_cli_rejects_invalid_transport_arguments(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
