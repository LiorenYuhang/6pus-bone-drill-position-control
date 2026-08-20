"""No-hardware tests for Stage M6-B one-motor motion."""

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
from tools import safe_m6b_one_motor as tool  # noqa: E402


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

    def time(self):
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
            and self.fail_pause_id == parsed.motor_id
        ):
            raise RuntimeError(f"simulated pause failure on ID{parsed.motor_id}")
        return b""

    def close(self):
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


@dataclass
class MotorScenario:
    registers: dict[int, int]
    baseline_statuses: list[StatusFeedback]
    motion_statuses: list[StatusFeedback]
    stop_statuses: list[StatusFeedback]
    written_registers: dict[int, list[int]] = field(default_factory=dict)


class FakeMotor:
    def __init__(self, bus, motor_id, scenario):
        self.bus = bus
        self.motor_id = motor_id
        self.scenario = scenario
        self._baseline_index = 0
        self._motion_index = 0
        self._stop_index = 0
        self.phase = "baseline"

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        val = self.scenario.registers.get(address, 0)
        return ReadRegisterResponse(register_addr=address, values=(val,))

    def write_register(self, address, values):
        self.bus.request(
            pack_write_register_cmd(self.motor_id, address, values),
            expected_id=self.motor_id,
        )
        self.scenario.written_registers.setdefault(address, []).append(values)
        if address == tool.CONTROL_MODE_ADDR and len(values) >= 5:
            self.scenario.registers[tool.CONTROL_MODE_ADDR] = values[0]
            self.scenario.registers[tool.TARGET_SPEED_ADDR] = values[3]
            self.scenario.registers[tool.TARGET_POSITION_ADDR] = values[4]
        elif address == tool.TARGET_POSITION_ADDR:
            self.scenario.registers[address] = values[0]
        elif address == tool.PAUSE_ADDR:
            self.phase = "stop"
        return self.read_status()

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id),
            expected_id=self.motor_id,
        )
        if self.phase == "baseline":
            statuses = self.scenario.baseline_statuses
            index = min(self._baseline_index, len(statuses) - 1)
            self._baseline_index += 1
            return statuses[index]
        if self.phase == "motion":
            statuses = self.scenario.motion_statuses
            index = min(self._motion_index, len(statuses) - 1)
            self._motion_index += 1
            return statuses[index]
        statuses = self.scenario.stop_statuses
        index = min(self._stop_index, len(statuses) - 1)
        self._stop_index += 1
        return statuses[index]


def _default_scenarios(moving_id, *, final_error=2):
    positions = {1: 59, 2: 67, 3: 23, 4: 54, 5: 0, 6: 0}
    scenarios = {}
    for motor_id in tool.MOTOR_IDS:
        pos = positions[motor_id]
        registers = {
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.CONTROL_MODE_ADDR: 0,
            tool.TARGET_SPEED_ADDR: 65535,
            tool.TARGET_POSITION_ADDR: pos,
        }
        baseline = [_status(pos) for _ in range(20)]
        if motor_id == moving_id:
            target = pos + tool.MOVE_STEPS
            trigger_pos = target - 2
            final_pos = target - final_error
            motion = [
                _status(pos),
                _status(pos + 2, current=30),
                _status(pos + 5, current=40),
                _status(trigger_pos, current=35),
            ]
            stop = [_status(final_pos) for _ in range(30)]
        else:
            motion = [_status(pos) for _ in range(10)]
            stop = [_status(pos) for _ in range(30)]
        scenarios[motor_id] = MotorScenario(
            registers=registers,
            baseline_statuses=baseline,
            motion_statuses=motion,
            stop_statuses=stop,
        )
    return scenarios


def _run_m6b(monkeypatch, tmp_path, moving_id, scenarios, *, confirm_input=None):
    FakeBus.instances.clear()
    clock = FakeClock()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    monkeypatch.setattr(tool.time, "time", clock.time)
    monkeypatch.setattr(tool, "_configure_logging", lambda p: None)

    created_motors = {}

    def fake_motor_factory(bus, motor_id):
        m = FakeMotor(bus, motor_id, scenarios[motor_id])
        created_motors[motor_id] = m
        return m

    monkeypatch.setattr(tool, "Motor", fake_motor_factory)

    confirm_str = confirm_input if confirm_input is not None else f"M6-B MOVE ID{moving_id}"
    monkeypatch.setattr("builtins.input", lambda prompt: confirm_str)

    messages = []
    monkeypatch.setattr(tool.logger, "info", lambda msg, *args: messages.append(msg % args if args else msg))
    monkeypatch.setattr(tool.logger, "warning", lambda msg, *args: messages.append(msg % args if args else msg))
    monkeypatch.setattr(tool.logger, "error", lambda msg, *args: messages.append(msg % args if args else msg))

    log_file = tmp_path / f"m6b_{moving_id}_test.log"
    args = tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--moving-id",
            str(moving_id),
            "--log",
            str(log_file),
        ]
    )

    # Transition fake motors to motion phase after baseline
    original_collect_baseline = tool._collect_baseline

    def wrap_baseline(*args_b, **kwargs_b):
        res = original_collect_baseline(*args_b, **kwargs_b)
        for m in created_motors.values():
            m.phase = "motion"
        return res

    monkeypatch.setattr(tool, "_collect_baseline", wrap_baseline)

    result = tool.run(args)
    bus = FakeBus.instances[0]
    return result, bus, created_motors, messages


@pytest.mark.parametrize("moving_id", [5, 6])
def test_m6b_success_moves_only_target_motor(monkeypatch, tmp_path, moving_id):
    scenarios = _default_scenarios(moving_id, final_error=2)
    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 0
    assert any("M6B_RESULT decision=PASS" in m for m in messages)

    # Verify write audit: exactly 3 writes, all to moving_id
    write_events = [
        e for e in bus.events if e[0] == "request" and e[2] == CMD_WR_REGISTER
    ]
    assert len(write_events) == 3
    for we in write_events:
        assert we[1] == moving_id

    # Observers have 0 writes
    for oid in tool.MOTOR_IDS:
        if oid != moving_id:
            assert len(created[oid].scenario.written_registers) == 0


@pytest.mark.parametrize(
    ("frame", "moving_id", "match"),
    [
        (
            pack_write_register_cmd(1, tool.PAUSE_ADDR, [1]),
            5,
            "sent to observer ID1",
        ),
        (
            pack_write_register_cmd(5, 0x24, [0]),
            5,
            "forbidden",
        ),
        (
            pack_write_register_cmd(7, tool.PAUSE_ADDR, [1]),
            5,
            "outside M6-B scope",
        ),
    ],
)
def test_audited_bus_blocks_forbidden_writes(frame, moving_id, match):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.M6bAuditedBus(delegate, moving_id=moving_id)
    with pytest.raises(tool.SafetyViolation, match=match):
        audited.request(frame)


def test_operator_abort_has_zero_writes(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    result, bus, _created, _messages = _run_m6b(
        monkeypatch, tmp_path, moving_id, scenarios, confirm_input="NO"
    )
    assert result == 1
    write_events = [
        e for e in bus.events if e[0] == "request" and e[2] == CMD_WR_REGISTER
    ]
    assert len(write_events) == 0


def test_final_error_three_passes_and_four_fails(monkeypatch, tmp_path):
    moving_id = 5
    # error = 3 passes
    scenarios_pass = _default_scenarios(moving_id, final_error=3)
    result_pass, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios_pass)
    assert result_pass == 0

    # error = 4 fails
    scenarios_fail = _default_scenarios(moving_id, final_error=4)
    result_fail, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios_fail)
    assert result_fail == 1


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers():
    assert tool.MOTOR_IDS == (1, 2, 3, 4, 5, 6)
    assert tool.PAUSE_TRIGGER_TOLERANCE_STEPS == 2
    assert tool.FINAL_ACCEPTANCE_TOLERANCE_STEPS == 3
    assert tool.MAX_CURRENT_MA == 500

    source = (ROOT / "tools" / "safe_m6b_one_motor.py").read_text(
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


def test_observer_span_two_passes_and_span_three_fails(monkeypatch, tmp_path):
    moving_id = 5

    # 1. Observer ID2 has {65: 1, 66: 13, 67: 86}, span=2, non-monotonic -> PASS
    scenarios_pass = _default_scenarios(moving_id)
    # Interspersed samples to ensure no monotonic drift
    id2_baseline_pass = [_status(67)] * 40 + [_status(66)] * 10 + [_status(67)] * 30 + [_status(65)] * 1 + [_status(66)] * 3 + [_status(67)] * 16
    scenarios_pass[2].baseline_statuses = id2_baseline_pass
    scenarios_pass[2].registers[tool.TARGET_POSITION_ADDR] = 67
    scenarios_pass[2].motion_statuses = [_status(67)] * 10
    scenarios_pass[2].stop_statuses = [_status(67)] * 30
    result_pass, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios_pass)
    assert result_pass == 0

    # 2. Observer ID2 has span=3 ({65: 1, 66: 10, 67: 88, 68: 1}) -> FAIL
    scenarios_fail = _default_scenarios(moving_id)
    id2_baseline_fail = [_status(67)] * 40 + [_status(66)] * 10 + [_status(68)] * 1 + [_status(65)] * 1 + [_status(67)] * 48
    scenarios_fail[2].baseline_statuses = id2_baseline_fail
    result_fail, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios_fail)
    assert result_fail == 1


def test_observer_unidirectional_drift_fails(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Strictly monotonic non-decreasing sequence from 65 to 67 -> sustained drift
    drift_samples = [_status(65)] * 30 + [_status(66)] * 30 + [_status(67)] * 40
    scenarios[2].baseline_statuses = drift_samples
    result, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1


def test_moving_motor_reference_plus_minus_two_still_fails(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Moving motor ID5 has mode=0, but contains sample 2 (delta=2 from mode) -> must FAIL
    id5_bad_baseline = [_status(0)] * 80 + [_status(1)] * 19 + [_status(2)] * 1
    scenarios[5].baseline_statuses = id5_bad_baseline
    result, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1


def test_observer_exceeding_baseline_band_fails_runtime(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Baseline for ID2 established as [66, 67]
    scenarios[2].baseline_statuses = [_status(67)] * 80 + [_status(66)] * 20
    # During motion, ID2 drifts out of band to 68
    scenarios[2].motion_statuses = [_status(67), _status(68)]
    result, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1


def test_setup_writes_five_registers_and_sets_speed_fifty(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 0

    moving_writes = created[moving_id].scenario.written_registers
    # 1. Setup must write to 0x25 with 5 values [2, 0, 0, 50, pos]
    assert tool.CONTROL_MODE_ADDR in moving_writes
    setup_payload = moving_writes[tool.CONTROL_MODE_ADDR][0]
    assert len(setup_payload) == 5
    assert setup_payload[0] == 2   # 0x25: mode = SPEED_MODE
    assert setup_payload[1] == 0   # 0x26: voltage_target = 0
    assert setup_payload[2] == 0   # 0x27: force_target = 0
    assert setup_payload[3] == 50  # 0x28: target_speed = 50 step/s
    assert setup_payload[4] == 0   # 0x29: target_position = baseline static pos

    # 2. Target write must write to 0x29 with [10]
    assert tool.TARGET_POSITION_ADDR in moving_writes
    target_payload = moving_writes[tool.TARGET_POSITION_ADDR][0]
    assert target_payload == [10]

    # 3. Pause write must write to 0x1A with [1]
    assert tool.PAUSE_ADDR in moving_writes
    pause_payload = moving_writes[tool.PAUSE_ADDR][0]
    assert pause_payload == [1]


def test_emergency_pause_on_exception_and_hard_stop_escalation(monkeypatch, tmp_path):
    moving_id = 5
    # Simulate motion timeout by never reaching trigger tolerance
    scenarios = _default_scenarios(moving_id)
    scenarios[moving_id].motion_statuses = [_status(0)] * 200

    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1
    # Emergency pause must be executed
    moving_writes = created[moving_id].scenario.written_registers
    assert tool.PAUSE_ADDR in moving_writes
    assert moving_writes[tool.PAUSE_ADDR] == [[1]]

    # Now simulate pause write failure on bus
    FakeBus.instances.clear()
    bus_delegate = FakeBus("test", timeout=0.05, retries=0)
    bus_delegate.fail_pause_id = moving_id
    scenarios_fail = _default_scenarios(moving_id)
    scenarios_fail[moving_id].motion_statuses = [_status(0)] * 200
    result_fail, _, _, _ = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios_fail)
    assert result_fail == 1


def test_motion_reaching_error_two_between_two_and_three_seconds_passes(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Stays at actual=0 for 2.2s (e.g. 110 poll ticks of 0.02s), then reaches 8 (err=2) at 2.4s
    slow_start_motion = [_status(0, current=25)] * 110 + [_status(8, current=30)] * 10
    scenarios[moving_id].motion_statuses = slow_start_motion
    scenarios[moving_id].stop_statuses = [_status(8)] * 30
    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 0
    assert any("M6B_RESULT decision=PASS" in m for m in messages)


def test_motion_timeout_at_three_seconds_aborts_and_pauses(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Never reaches error <= 2 within 3.0s (160 poll ticks of 0.02s)
    scenarios[moving_id].motion_statuses = [_status(7, current=30)] * 200

    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1
    assert any("failed to reach error <= 2 within 3.0s" in m for m in messages)
    moving_writes = created[moving_id].scenario.written_registers
    assert tool.PAUSE_ADDR in moving_writes
    assert moving_writes[tool.PAUSE_ADDR] == [[1]]


def test_runtime_fault_or_overcurrent_aborts_immediately_without_waiting_for_timeout(monkeypatch, tmp_path):
    moving_id = 5
    scenarios = _default_scenarios(moving_id)
    # Error occurs at tick 10 (t ~ 0.2s), far before 3.0s timeout
    scenarios[moving_id].motion_statuses = [_status(2, current=30)] * 10 + [_status(2, current=600)] * 50

    result, bus, created, messages = _run_m6b(monkeypatch, tmp_path, moving_id, scenarios)
    assert result == 1
    assert any("current=600mA exceeds 500mA" in m for m in messages)
    # Pause is sent immediately
    moving_writes = created[moving_id].scenario.written_registers
    assert tool.PAUSE_ADDR in moving_writes
    assert moving_writes[tool.PAUSE_ADDR] == [[1]]



