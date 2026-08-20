"""No-hardware tests for the single positive endpoint escape tool."""

from __future__ import annotations

import ast
import itertools
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import ReadRegisterResponse, StatusFeedback  # noqa: E402
from tools import safe_endpoint_escape_id1 as tool  # noqa: E402


def _status(position, current=0, force=-420, temperature=25, fault=0):
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
        self.closed = False
        self.events = []
        type(self).instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
        return False

    def close(self):
        if not self.closed:
            self.events.append("close")
            self.closed = True


class FakeMotor:
    instances = []

    def __init__(self, bus, motor_id):
        assert motor_id == 1
        self.bus = bus
        self.writes = []
        self.statuses = iter(())
        self.pause_failure = False
        self.setup_response = None
        self.target_response = None
        type(self).instances.append(self)

    def read_register(self, address):
        values = {0x24: 0, 0x23: 2000, 0x25: 0, 0x28: 65535, 0x29: 3}
        return ReadRegisterResponse(register_addr=address, values=(values[address],))

    def read_status(self):
        if self.bus is not None:
            self.bus.events.append("read")
        return next(self.statuses)

    def write_register(self, address, data):
        if self.bus is not None:
            self.bus.events.append(f"write:{address:02X}")
        if self.pause_failure and address == tool.PAUSE_ADDR:
            raise OSError("pause unavailable")
        self.writes.append((address, list(data)))
        if address == tool.CONTROL_MODE_ADDR and self.setup_response is not None:
            return self.setup_response
        if address == tool.TARGET_POSITION_ADDR and self.target_response is not None:
            return self.target_response
        return _status(data[-1], force=-418)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def _args(tmp_path, **overrides):
    values = {
        "--port": "/dev/serial/by-id/test",
        "--max-current-ma": "1000",
        "--baseline-duration": "0.1",
        "--baseline-rate": "20",
        "--retries": "0",
        "--log": str(tmp_path / "escape.log"),
    }
    values.update({key: str(value) for key, value in overrides.items()})
    argv = []
    for key, value in values.items():
        argv.extend((key, value))
    return tool.parse_args(argv)


def test_single_positive_five_step_and_force_delta_is_logged(monkeypatch, tmp_path, capsys):
    FakeBus.instances.clear()
    FakeMotor.instances.clear()
    motor = FakeMotor(None, 1)
    motor.statuses = iter(
        [
            _status(4, force=-428),
            _status(3, force=-427),
            _status(3, force=-426),
            _status(8, force=-425),
        ]
    )

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path)) == 0
    output = capsys.readouterr().out
    assert "precheck_single_position=4" in output
    assert "baseline=[3,3]" in output
    assert "final=8" in output
    assert len(motor.writes) == 2
    assert motor.writes[0][0] == tool.CONTROL_MODE_ADDR
    assert motor.writes[1] == (tool.TARGET_POSITION_ADDR, [8])
    assert "delta_force" in output


def test_run04_uses_baseline_max_plus_fixed_ten_step(monkeypatch, tmp_path, capsys):
    motor = FakeMotor(None, 1)
    motor.statuses = iter(
        [_status(4), _status(3), _status(3), _status(13)]
    )
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path, **{"--escape-step": "10"})) == 0
    output = capsys.readouterr().out
    assert "precheck_single_position=4" in output
    assert "baseline=[3,3]" in output
    assert "target=13" in output
    assert motor.writes[1] == (tool.TARGET_POSITION_ADDR, [13])


def test_setup_response_inside_future_baseline_band_does_not_abort(monkeypatch, tmp_path):
    motor = FakeMotor(None, 1)
    motor.setup_response = _status(6)
    motor.statuses = iter([_status(7), _status(7), _status(7), _status(17)])
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path, **{"--escape-step": "10"})) == 0
    assert (tool.TARGET_POSITION_ADDR, [17]) in motor.writes
    assert all(address != tool.PAUSE_ADDR for address, _ in motor.writes)


def test_setup_success_arms_pause_before_close_for_later_setup_validation_error(
    monkeypatch, tmp_path
):
    created = []

    def factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id)
        motor.setup_response = _status(6, fault=1)
        motor.statuses = itertools.chain(
            [_status(7), _status(7), _status(7)],
            itertools.repeat(_status(7)),
        )
        created.append(motor)
        return motor

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path, **{"--escape-step": "10"})) == 1
    motor = created[0]
    addresses = [address for address, _ in motor.writes]
    assert addresses == [tool.CONTROL_MODE_ADDR, tool.PAUSE_ADDR]
    assert motor.bus.events.index("write:1A") < motor.bus.events.index("close")


def test_setup_position_does_not_set_first_motion_before_motion_t0(
    monkeypatch, tmp_path
):
    messages = []

    def factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id)
        motor.setup_response = _status(8)
        motor.target_response = _status(7)
        motor.statuses = itertools.chain(
            [_status(7), _status(7), _status(7)],
            itertools.repeat(_status(7)),
        )
        return motor

    def capture(message, *args):
        messages.append(message % args if args else message)

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    monkeypatch.setattr(tool.logger, "info", capture)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(
        _args(
            tmp_path,
            **{"--escape-step": "10", "--settle-timeout": "0.05"},
        )
    ) == 1
    result = next(message for message in messages if message.startswith("RUN_RESULT"))
    assert "first_motion_time=-" in result
    assert "pause_success=True" in result


def test_force_bias_does_not_trigger_an_absolute_force_stop(monkeypatch, tmp_path):
    motor = FakeMotor(None, 1)
    motor.statuses = iter(
        [_status(4, force=-500), _status(4, force=-499), _status(4, force=-498), _status(9, force=-497)]
    )
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    assert tool.run(_args(tmp_path)) == 0


def test_position_above_low_endpoint_gate_is_rejected(monkeypatch, tmp_path, capsys):
    motor = FakeMotor(None, 1)
    motor.statuses = iter([_status(200), _status(200), _status(200)])
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    assert tool.run(_args(tmp_path)) == 1
    assert "low-endpoint gate" in capsys.readouterr().err
    assert motor.writes == []


def test_baseline_band_accepts_arbitrary_six_seven_repetition(monkeypatch, tmp_path):
    motor = FakeMotor(None, 1)
    motor.statuses = iter([_status(7), _status(6), _status(7), _status(12)])
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path)) == 0
    assert motor.writes[0] == (
        tool.CONTROL_MODE_ADDR,
        [tool.SPEED_MODE, 0, 0, tool.ESCAPE_SPEED_STEPS_PER_SECOND, 7],
    )
    assert motor.writes[1] == (tool.TARGET_POSITION_ADDR, [12])


def test_run04_r1_pattern_stays_in_band_until_actual_exceeds_baseline_max(
    monkeypatch, tmp_path
):
    motor = FakeMotor(None, 1)
    motor.target_response = _status(7)
    motor.statuses = iter(
        [
            _status(7),
            _status(6),
            _status(7),
            _status(6),
            _status(6),
            _status(6),
            _status(7),
            _status(8),
            _status(17),
        ]
    )
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)

    assert tool.run(_args(tmp_path, **{"--escape-step": "10"})) == 0
    assert motor.writes[1] == (tool.TARGET_POSITION_ADDR, [17])
    assert all(address != tool.PAUSE_ADDR for address, _ in motor.writes)


def test_baseline_band_rejects_span_above_one_before_any_write(
    monkeypatch, tmp_path, capsys
):
    motor = FakeMotor(None, 1)
    motor.statuses = iter([_status(3), _status(3), _status(3), _status(5)])
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)

    assert tool.run(_args(tmp_path, **{"--baseline-duration": "0.15"})) == 1
    assert "span exceeds one step" in capsys.readouterr().err
    assert motor.writes == []


def test_build_position_baseline_accepts_one_step_band_and_rejects_invalid_sets():
    baseline = tool._build_position_baseline([7, 6, 7, 6])
    assert baseline.minimum == 6
    assert baseline.maximum == 7
    assert baseline.span == 1
    assert baseline.histogram == {6: 2, 7: 2}
    with pytest.raises(tool.SafetyViolation, match="empty"):
        tool._build_position_baseline([])
    with pytest.raises(tool.SafetyViolation, match="span exceeds one step"):
        tool._build_position_baseline([6, 8])


def test_fault_or_current_stops_before_any_write(monkeypatch, tmp_path):
    motor = FakeMotor(None, 1)
    motor.statuses = iter([_status(4, current=1001, fault=0)])
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", lambda bus, motor_id: motor)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    assert tool.run(_args(tmp_path)) == 1
    assert motor.writes == []


def _patch_timeout_run(monkeypatch, tmp_path, *, pause_failure=False):
    created = []

    def factory(bus, motor_id):
        motor = FakeMotor(bus, motor_id)
        motor.pause_failure = pause_failure
        # Initial preflight + two baseline samples, then a non-settling stream.
        motor.statuses = itertools.chain(
            [_status(3, force=-420), _status(3, force=-419), _status(3, force=-418)],
            itertools.repeat(_status(3, force=-417)),
        )
        created.append(motor)
        return motor

    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool, "Motor", factory)
    monkeypatch.setattr(tool, "_configure_logging", lambda path: None)
    monkeypatch.setattr(tool, "_confirm", lambda: None)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    args = _args(tmp_path, **{"--settle-timeout": "0.05"})
    return tool.run(args), created[0]


def test_timeout_pauses_before_uart_close_and_confirms_read_only(monkeypatch, tmp_path):
    result, motor = _patch_timeout_run(monkeypatch, tmp_path)
    assert result == 1
    events = motor.bus.events
    pause_index = events.index("write:1A")
    close_index = events.index("close")
    assert pause_index < close_index
    assert events.count("write:1A") == 1
    assert "read" in events[pause_index + 1 : close_index]
    assert motor.bus.closed is True


def test_pause_failure_requires_hard_stop_and_then_closes(monkeypatch, tmp_path, capsys):
    result, motor = _patch_timeout_run(monkeypatch, tmp_path, pause_failure=True)
    assert result == 1
    error = capsys.readouterr().err
    assert "HARD STOP REQUIRED" in error
    pause_index = motor.bus.events.index("write:1A")
    close_index = motor.bus.events.index("close")
    assert pause_index < close_index
    assert motor.bus.closed is True


def test_motion_trace_records_first_motion_first_tolerance_and_extrema(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    trace = tool.MotionTrace(
        baseline_min=6,
        baseline_max=7,
        target=12,
        force_f0=-377.0,
        t0_mono=clock.monotonic(),
        t0_wall="2026-08-16T00:00:00+00:00",
        samples=[],
    )
    trace.observe(_status(7, current=0, force=-377), error=5)
    clock.now = 0.1
    trace.observe(_status(8, current=12, force=-370), error=4)
    clock.now = 0.2
    trace.observe(_status(12, current=18, force=-368), error=0)
    clock.now = 0.3
    trace.observe(_status(12, current=16, force=-369), error=0)
    line = trace.result_line(
        decision="PASS",
        pause_triggered=False,
        pause_success=False,
        stop_confirmation=True,
    )
    assert trace.first_motion_mono == 0.1
    assert trace.first_motion_current_mA == 12
    assert trace.first_in_tolerance_mono == 0.2
    assert "first_motion_elapsed_s=0.100000" in line
    assert "first_motion_current_mA=12" in line
    assert "first_in_tolerance_elapsed_s=0.200000" in line
    assert "actual_max=12" in line
    assert "final_platform_actual=12 final_platform_elapsed_s=0.200000" in line
    assert "final_actual=12 final_error=0" in line
    assert "max_current_mA=18" in line
    assert "force_min_g=-377 force_max_g=-368" in line
    assert "delta_force_min_g=0.0 delta_force_max_g=9.0" in line


def test_motion_trace_final_platform_uses_start_of_trailing_position_run(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    trace = tool.MotionTrace(
        baseline_min=6,
        baseline_max=7,
        target=17,
        force_f0=-377.0,
        t0_mono=0.0,
        t0_wall="2026-08-16T00:00:00+00:00",
        samples=[],
    )
    for elapsed, position in [(0.1, 8), (0.2, 15), (0.3, 14), (0.4, 15), (0.5, 15)]:
        clock.now = elapsed
        trace.observe(_status(position, current=20), error=abs(17 - position))
    line = trace.result_line(
        decision="FAIL",
        pause_triggered=True,
        pause_success=True,
        stop_confirmation=True,
    )
    assert "actual_max=15" in line
    assert "final_platform_actual=15 final_platform_elapsed_s=0.400000" in line


def test_motion_trace_separates_poll_and_pause_response_current(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    trace = tool.MotionTrace(
        baseline_min=6,
        baseline_max=7,
        target=12,
        force_f0=-377.0,
        t0_mono=0.0,
        t0_wall="2026-08-16T00:00:00+00:00",
        samples=[],
    )
    trace.observe(_status(7, current=56), error=5)
    trace.observe(_status(7, current=43), error=5, source="command")
    trace.set_pause_status(_status(7, current=71))
    line = trace.result_line(
        decision="FAIL",
        pause_triggered=True,
        pause_success=True,
        stop_confirmation=False,
    )
    assert "max_poll_current_mA=56" in line
    assert "max_observed_current_mA=71" in line
    assert "max_current_mA=71" in line


def _confirm_statuses(monkeypatch, statuses):
    motor = FakeMotor(None, 1)
    motor.statuses = iter(statuses)
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(tool.time, "sleep", clock.sleep)
    return tool._confirm_stopped(motor, duration=0.2)


def test_stop_confirmation_accepts_known_one_step_bidirectional_jitter(monkeypatch):
    stable, final = _confirm_statuses(
        monkeypatch, [_status(7), _status(6), _status(7), _status(6), _status(7)]
    )
    assert stable is True
    assert final.current_position == 7


def test_stop_confirmation_accepts_one_step_final_landing_with_stable_tail(monkeypatch):
    stable, final = _confirm_statuses(
        monkeypatch, [_status(7), _status(7), _status(6), _status(6), _status(6)]
    )
    assert stable is True
    assert final.current_position == 6


def test_stop_confirmation_rejects_multi_step_unidirectional_drift(monkeypatch):
    stable, _ = _confirm_statuses(
        monkeypatch, [_status(7), _status(6), _status(5), _status(4), _status(4)]
    )
    assert stable is False


def test_stop_confirmation_rejects_nonzero_current_or_fault(monkeypatch):
    stable_current, _ = _confirm_statuses(
        monkeypatch,
        [_status(7), _status(7, current=1), _status(7), _status(7), _status(7)],
    )
    assert stable_current is False
    stable_fault, _ = _confirm_statuses(
        monkeypatch,
        [_status(7), _status(7, fault=1), _status(7), _status(7), _status(7)],
    )
    assert stable_fault is False


def test_negative_quantization_is_not_first_positive_motion_and_post_pause_is_final(
    monkeypatch,
):
    clock = FakeClock()
    monkeypatch.setattr(tool.time, "monotonic", clock.monotonic)
    trace = tool.MotionTrace(
        baseline_min=6,
        baseline_max=7,
        target=17,
        force_f0=-377.0,
        t0_mono=0.0,
        t0_wall="2026-08-16T00:00:00+00:00",
        samples=[],
    )
    trace.observe(_status(6, current=1, force=-377), error=11)
    trace.observe(_status(7, current=1, force=-377), error=10)
    assert trace.first_motion_mono is None
    trace.set_post_pause_status(_status(7, current=0, force=-377))
    line = trace.result_line(
        decision="FAIL",
        pause_triggered=True,
        pause_success=True,
        stop_confirmation=True,
    )
    assert "first_motion_time=-" in line
    assert "first_motion_current_mA=-" in line
    assert "actual_max=7" in line
    assert "final_actual=7 final_error=10" in line
    assert "trigger_actual=7 trigger_error=10" in line
    assert "post_stop_final_actual=7 post_stop_final_error=10" in line
    assert "pause_triggered=True pause_success=True stop_confirmation=True" in line


def test_baseline_band_guard_accepts_any_repetition_inside_band():
    guard = tool.BaselineBandGuard(baseline_min=6, baseline_max=7)
    for position in [6, 6, 7, 6, 7, 7, 6]:
        guard.observe(position)


def test_baseline_band_guard_accepts_positive_motion_above_band():
    guard = tool.BaselineBandGuard(baseline_min=6, baseline_max=7)
    guard.observe(8)
    guard.observe(17)


def test_baseline_band_guard_rejects_any_position_below_baseline_min():
    guard = tool.BaselineBandGuard(baseline_min=6, baseline_max=7)
    with pytest.raises(tool.SafetyViolation, match="below baseline band"):
        guard.observe(5)


def test_default_position_baseline_is_two_seconds_at_fifty_hz():
    args = tool.parse_args(["--port", "/dev/tty", "--max-current-ma", "500"])
    assert args.baseline_duration == 2.0
    assert args.baseline_rate == 50.0


def test_removed_reverse_guard_model_is_not_exposed():
    assert not hasattr(tool, "ReverseDirectionGuard")


def test_source_has_no_general_motion_or_force_write_interface():
    source = (ROOT / "tools" / "safe_endpoint_escape_id1.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    called = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called.isdisjoint({"set_motor_id", "save_parameters", "set_position", "set_speed", "move"})


@pytest.mark.parametrize("argv", [
    ["--port", "/dev/tty", "--max-current-ma", "0"],
    ["--port", "/dev/tty", "--max-current-ma", "1", "--baseline-rate", "0"],
    ["--port", "/dev/tty", "--max-current-ma", "1", "--escape-step", "6"],
])
def test_invalid_arguments_are_rejected(argv):
    with pytest.raises(SystemExit):
        tool.parse_args(argv)
