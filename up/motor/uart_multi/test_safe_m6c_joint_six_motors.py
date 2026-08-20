"""No-hardware tests for Stage M6-C joint six-motor motion."""

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
from tools import safe_m6c_joint_six_motors as tool  # noqa: E402


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

    def __init__(self, port, *, baudrate=921600, timeout, retries):
        self.port = port
        self.baudrate = baudrate
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
            59,
            67,
            motion_statuses=[_status(63, current=60), _status(67, current=30)],
        ),
        2: _scenario(
            67,
            75,
            motion_statuses=[_status(71, current=55), _status(75, current=25)],
        ),
        3: _scenario(
            22,
            30,
            motion_statuses=[_status(26, current=45), _status(30, current=20)],
        ),
        4: _scenario(
            54,
            62,
            motion_statuses=[_status(58, current=50), _status(62, current=35)],
        ),
        5: _scenario(
            15,
            23,
            motion_statuses=[_status(19, current=40), _status(23, current=25)],
        ),
        6: _scenario(
            8,
            16,
            motion_statuses=[_status(12, current=45), _status(16, current=30)],
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
    confirm_input="M6-C MOVE ALL SIX MOTORS",
):
    created = {}
    messages = []

    def bus_factory(port, *, baudrate=921600, timeout, retries):
        bus = FakeBus(port, baudrate=baudrate, timeout=timeout, retries=retries)
        bus.fail_pause_id = fail_pause_id
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
    monkeypatch.setattr("builtins.input", lambda prompt: confirm_input)
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

    log_file = tmp_path / "m6c_test.log"
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


def test_m6c_success_uses_one_bus_and_pauses_all(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 0
    assert any("M6C_RESULT decision=PASS" in m for m in messages)
    assert len(FakeBus.instances) == 1
    writes = _write_events(bus)
    # 6 setups + 6 target writes + 6 pauses = 18 writes
    assert len(writes) == 18
    assert [w[1] for w in writes] == [
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
        1, 2, 3, 4, 5, 6,
    ]
    assert [w[3] for w in writes[:6]] == [tool.CONTROL_MODE_ADDR] * 6
    assert [w[3] for w in writes[6:12]] == [tool.TARGET_POSITION_ADDR] * 6
    assert [w[3] for w in writes[12:]] == [tool.PAUSE_ADDR] * 6


@pytest.mark.parametrize(
    ("frame", "match"),
    [
        (
            pack_write_register_cmd(7, tool.PAUSE_ADDR, [1]),
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
    audited = tool.M6cAuditedBus(delegate)
    with pytest.raises(tool.SafetyViolation, match=match):
        audited.request(frame)


def test_motor_runtime_fault_pauses_all_motors(monkeypatch, tmp_path):
    scenarios = _scenarios(
        override_motor_id=5,
        override_scenario=_scenario(
            15,
            23,
            motion_statuses=[_status(19, fault=1)],
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert any("fault code=0x01" in m for m in messages)
    writes = _write_events(bus)
    # 6 setups + 6 targets + emergency 6 pauses
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    assert len(pause_writes) == 6
    assert [w[1] for w in pause_writes] == [1, 2, 3, 4, 5, 6]


def test_target_readback_mismatch_fails_and_pauses_all(monkeypatch, tmp_path):
    scenarios = _scenarios(
        override_motor_id=6,
        override_scenario=_scenario(
            8,
            16,
            motion_statuses=[_status(12), _status(16)],
            target_readback_override=0,
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert any("target write readback mismatch" in m for m in messages)
    writes = _write_events(bus)
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_pause_failure_requires_hard_stop(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, created, messages = _run(
        monkeypatch, tmp_path, scenarios, fail_pause_id=2
    )
    assert result == 1
    assert any("HARD STOP REQUIRED" in m for m in messages)
    writes = _write_events(bus)
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    # Even when ID2 fails, pause attempts were made
    assert len(pause_writes) >= 6


def test_setup_phase_exception_triggers_pause_all(monkeypatch, tmp_path):
    scenarios = _scenarios()
    original_write = FakeMotor.write_register

    def fail_setup(self, address, data):
        if address == tool.CONTROL_MODE_ADDR and self.motor_id == 3:
            raise tool.SafetyViolation("simulated setup failure on ID3")
        return original_write(self, address, data)

    monkeypatch.setattr(FakeMotor, "write_register", fail_setup)
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert any("simulated setup failure on ID3" in m for m in messages)
    writes = _write_events(bus)
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_operator_abort_has_zero_writes(monkeypatch, tmp_path):
    scenarios = _scenarios()
    result, bus, created, messages = _run(
        monkeypatch, tmp_path, scenarios, confirm_input="NO"
    )
    assert result == 1
    writes = _write_events(bus)
    assert len(writes) == 0


def test_final_error_three_passes_and_four_fails(monkeypatch, tmp_path):
    # Error = 3 passes (target=25, reached 23 to trigger pause, final settled at 22)
    scenarios_pass = _scenarios(
        override_motor_id=5,
        override_scenario=_scenario(
            15,
            22,  # target=25, final=22 -> final_err=3
            motion_statuses=[_status(19), _status(23)],
        ),
    )
    result_pass, _, _, _ = _run(monkeypatch, tmp_path, scenarios_pass)
    assert result_pass == 0

    # Error = 4 fails (target=25, reached 23 to trigger pause, final settled at 21)
    scenarios_fail = _scenarios(
        override_motor_id=5,
        override_scenario=_scenario(
            15,
            21,  # target=25, final=21 -> final_err=4
            motion_statuses=[_status(19), _status(23)],
        ),
    )
    result_fail, _, _, _ = _run(monkeypatch, tmp_path, scenarios_fail)
    assert result_fail == 1


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers():
    assert tool.MOTOR_IDS == (1, 2, 3, 4, 5, 6)
    assert tool.SPEED_MODE == 2
    assert tool.MOVE_STEPS == 10
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 3.0
    assert tool.MAX_CURRENT_MA == 500
    assert tool.PAUSE_TRIGGER_TOLERANCE_STEPS == 2
    assert tool.FINAL_ACCEPTANCE_TOLERANCE_STEPS == 3
    assert tool.SOFTWARE_UPPER_LIMIT == 1800

    source = (ROOT / "tools" / "safe_m6c_joint_six_motors.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint(
        {"set_motor_id", "save_parameters", "set_speed", "move"}
    )


def test_m6c_direction_guard_unit():
    # ID6 with baseline=[8, 8], reference=8
    guard = tool.M6cDirectionGuard(
        motor_id=6,
        reference=8,
        baseline_min=8,
        baseline_max=8,
        device_lower_limit=0,
    )
    assert not guard.positive_motion_confirmed

    # 1. 8 is in baseline -> pass
    guard.observe(8)
    assert not guard.positive_motion_confirmed

    # 2. 7 is reference-1 -> pass before positive motion
    guard.observe(7)
    assert not guard.positive_motion_confirmed

    # 3. 6 is below reference-1 -> fail before positive motion
    with pytest.raises(tool.SafetyViolation, match="pre-motion negative-direction"):
        guard.observe(6)

    # 4. 8 again -> pass
    guard.observe(8)
    assert not guard.positive_motion_confirmed

    # 5. 9 exceeds baseline_max(8) -> transitions to positive motion
    guard.observe(9)
    assert guard.positive_motion_confirmed

    # 6. 10..18 in motion phase -> pass
    guard.observe(14)
    guard.observe(18)

    # 7. Dropping below baseline_min(8) to 7 in motion phase -> fail
    with pytest.raises(tool.SafetyViolation, match="motion-phase negative-direction"):
        guard.observe(7)


def test_m6c_pre_motion_jitter_seven_passes_end_to_end(monkeypatch, tmp_path):
    # ID6 baseline=[8, 8], target_response=8, early poll=7, then moves to 10, 16 -> PASS
    scenarios = _scenarios(
        override_motor_id=6,
        override_scenario=_scenario(
            8,
            16,
            motion_statuses=[_status(7), _status(8), _status(12), _status(16)],
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 0
    writes = _write_events(bus)
    assert len(writes) == 18


def test_m6c_pre_motion_six_fails_and_pauses_all(monkeypatch, tmp_path):
    # ID6 baseline=[8, 8], early poll=6 -> fails and pauses all
    scenarios = _scenarios(
        override_motor_id=6,
        override_scenario=_scenario(
            8,
            16,
            motion_statuses=[_status(6)],
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert any("pre-motion negative-direction" in m for m in messages)
    writes = _write_events(bus)
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_m6c_motion_phase_drop_below_baseline_min_fails(monkeypatch, tmp_path):
    # ID6 baseline=[8, 8], reaches 10 (positive motion confirmed), then drops to 7 -> fails
    scenarios = _scenarios(
        override_motor_id=6,
        override_scenario=_scenario(
            8,
            16,
            motion_statuses=[_status(10), _status(7)],
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 1
    assert any("motion-phase negative-direction" in m for m in messages)
    writes = _write_events(bus)
    pause_writes = [w for w in writes if w[3] == tool.PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_other_motors_static_jitter_passes(monkeypatch, tmp_path):
    # ID2 ref=67 (samples 66), ID4 ref=54 (samples 53), ID5 ref=15 (samples 14) before motion
    scenarios = _scenarios(
        override_motor_id=5,
        override_scenario=_scenario(
            15,
            23,
            motion_statuses=[_status(14), _status(15), _status(20), _status(23)],
        ),
    )
    result, bus, created, messages = _run(monkeypatch, tmp_path, scenarios)
    assert result == 0
    writes = _write_events(bus)
    assert len(writes) == 18

