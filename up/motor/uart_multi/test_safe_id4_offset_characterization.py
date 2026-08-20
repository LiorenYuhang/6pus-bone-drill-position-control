"""No-hardware tests for the restricted ID4-OFFSET characterization."""

from __future__ import annotations

from dataclasses import replace
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
    _build_frame,
)
from tools import safe_id4_offset_characterization as tool  # noqa: E402


def _status(position: int, *, target=None, current=0, force=-60, fault=0):
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
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


class FakeMotor:
    def __init__(self, bus, motor_id, *, entry_actual=-2, errors=None):
        assert motor_id == tool.MOTOR_ID
        self.bus = bus
        self.motor_id = motor_id
        self.target = -2
        self.actual = entry_actual
        self.errors = errors or {10: -2, 20: -2, 30: -2, 40: -2}
        self.writes = []

    @staticmethod
    def _raw(value):
        return value & 0xFFFF

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        values = {
            tool.LOWER_LIMIT_ADDR: 0,
            tool.UPPER_LIMIT_ADDR: 2000,
            tool.TARGET_POSITION_ADDR: self._raw(self.target),
            tool.ACTUAL_POSITION_ADDR: self._raw(self.actual),
        }
        return ReadRegisterResponse(register_addr=address, values=(values[address],))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id), expected_id=self.motor_id
        )
        return _status(self.actual, target=self.target)

    def write_register(self, address, data):
        values = list(data)
        self.bus.request(
            pack_write_register_cmd(self.motor_id, address, values),
            expected_id=self.motor_id,
            retries=0,
        )
        self.writes.append((address, values))
        if address == tool.CONTROL_MODE_ADDR:
            self.target = values[-1]
            self.actual = self.target + self.errors[self.target]
        elif address == tool.TARGET_POSITION_ADDR:
            self.target = values[0]
            self.actual = self.target + self.errors[self.target]
        elif address == tool.PAUSE_ADDR:
            self.target = self.actual
        return _status(self.actual, target=self.target)


def _point(target, error):
    status = _status(target + error, target=target)
    return tool.PointResult(
        commanded_target=target,
        readback_target=target,
        actual_position=target + error,
        error=error,
        current_mA=0,
        force_g=-60,
        fault=0,
        pause_response=status,
        pause_after_target=target + error,
        pause_after_actual=target + error,
    )


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
            str(tmp_path / "id4_offset.log"),
        ]
    )


def _run(monkeypatch, tmp_path, *, entry_actual=-2, errors=None):
    created = []

    def motor_factory(bus, motor_id):
        motor = FakeMotor(
            bus, motor_id, entry_actual=entry_actual, errors=errors
        )
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


def test_exact_fixed_minus2_run_uses_only_expected_id4_writes(
    monkeypatch, tmp_path, capsys
):
    result, bus, motor = _run(monkeypatch, tmp_path)
    assert result == 0
    assert "FIXED_MINUS2_ZERO_REFERENCE_OFFSET_SUPPORTED" in capsys.readouterr().out
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
        (tool.TARGET_POSITION_ADDR, [20]),
        (tool.PAUSE_ADDR, [1]),
        (tool.TARGET_POSITION_ADDR, [30]),
        (tool.PAUSE_ADDR, [1]),
        (tool.TARGET_POSITION_ADDR, [40]),
        (tool.PAUSE_ADDR, [1]),
    ]
    writes = [
        event
        for event in bus.events
        if len(event) > 2 and event[2] == CMD_WR_REGISTER
    ]
    assert all(event[1] == 4 for event in writes)
    assert bus.events.index(writes[-1]) < bus.events.index(("close",))


def test_persistent_entry_gate_rejects_any_non_minus2_before_writes(
    monkeypatch, tmp_path
):
    result, bus, motor = _run(monkeypatch, tmp_path, entry_actual=-1)
    assert result == 1
    assert motor.writes == []
    assert not [
        event
        for event in bus.events
        if len(event) > 2 and event[2] == CMD_WR_REGISTER
    ]


def test_auditor_rejects_other_id_broadcast_and_unlisted_write():
    audited = tool.Id4OffsetAuditedBus(FakeBus("test", timeout=0.05, retries=2))
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
    with pytest.raises(tool.SafetyViolation, match="write sequence mismatch"):
        audited.request(
            pack_write_register_cmd(4, tool.TARGET_POSITION_ADDR, [10]),
            expected_id=4,
        )


@pytest.mark.parametrize(
    ("errors", "decision", "classification"),
    [
        (
            [-2, -2, -2, -2],
            "PASS",
            "FIXED_MINUS2_ZERO_REFERENCE_OFFSET_SUPPORTED",
        ),
        ([0, 0, 0, 0], "PASS", "ZERO_ENDPOINT_ONLY_SUPPORTED"),
        (
            [0, -3, -1, -2],
            "PASS",
            "ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED",
        ),
        ([-1, -1, -1, -1], "INCONCLUSIVE", "CONSTANT_NON_MINUS2_ERROR"),
        ([-4, -2, -2, -2], "FAIL", "POSITION_ERROR_OUTSIDE_0_TO_3_STEP_BAND"),
    ],
)
def test_classification_is_explicit(errors, decision, classification):
    points = [_point(target, error) for target, error in zip(tool.TARGETS, errors)]
    assert tool._classify_points(points) == (decision, classification)


def test_incomplete_and_target_readback_mismatch_are_not_passed():
    points = [_point(target, -2) for target in tool.TARGETS]
    assert tool._classify_points(points[:3]) == (
        "INCONCLUSIVE",
        "INCOMPLETE_POINTS",
    )
    mismatch = [*points]
    mismatch[2] = replace(mismatch[2], readback_target=29)
    assert tool._classify_points(mismatch) == (
        "FAIL",
        "PRE_PAUSE_TARGET_READBACK_MISMATCH",
    )


def test_varying_known_residual_run_is_pass_not_fixed(monkeypatch, tmp_path, capsys):
    errors = {10: 0, 20: -3, 30: -1, 40: -2}
    result, _, _ = _run(monkeypatch, tmp_path, errors=errors)
    assert result == 0
    assert "ORDINARY_0_TO_3_STEP_RESIDUAL_SUPPORTED" in capsys.readouterr().out


def test_out_of_band_first_point_pauses_and_blocks_later_targets(
    monkeypatch, tmp_path
):
    errors = {10: -4, 20: -2, 30: -2, 40: -2}
    result, _, motor = _run(monkeypatch, tmp_path, errors=errors)
    assert result == 1
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_first_target_overshoot_pauses_and_blocks_later_targets(
    monkeypatch, tmp_path
):
    errors = {10: 1, 20: -2, 30: -2, 40: -2}
    result, _, motor = _run(monkeypatch, tmp_path, errors=errors)
    assert result == 1
    assert motor.writes == [
        (tool.CONTROL_MODE_ADDR, [2, 0, 0, 50, 10]),
        (tool.PAUSE_ADDR, [1]),
    ]


def test_fixed_parameters_and_raw_minus2_are_not_cli_overridable():
    parser = tool.build_parser()
    option_strings = {
        option
        for action in parser._actions
        for option in action.option_strings
    }
    assert "--targets" not in option_strings
    assert "--speed" not in option_strings
    assert "--motion-timeout" not in option_strings
    assert "--max-current" not in option_strings
    assert tool.TARGETS == (10, 20, 30, 40)
    assert tool.ENTRY_POSITION == -2
    assert tool.ENTRY_RAW_U16 == 0xFFFE
    assert (tool.DEVICE_LOWER_LIMIT, tool.DEVICE_UPPER_LIMIT) == (0, 2000)
    assert tool.SPEED_STEPS_PER_SECOND == 50
    assert tool.MOTION_TIMEOUT_S == 2.0
    assert tool.MAX_CURRENT_MA == 500
