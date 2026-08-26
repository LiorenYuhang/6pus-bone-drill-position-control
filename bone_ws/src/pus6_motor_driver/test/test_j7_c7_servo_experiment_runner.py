"""Mock-only tests for the isolated J7 Servo experiment runner."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.j7_servo_experiment_runner import (
    HOME_TARGET,
    J7ServoExperimentRunner,
    PARK_TARGET,
    RunnerState,
    ServoRunnerError,
)
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.protocol import (
    pack_read_status_cmd,
    parse_frame,
)
from pus6_motor_driver.safety import MOTOR_IDS


class Clock:
    def __init__(self) -> None:
        self.now = 0

    def now_ns(self) -> int:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += int(seconds * 1e9)

    def advance_ns(self, delta_ns: int) -> None:
        self.now += delta_ns


class RunnerMockBus(FakeBus):
    def __init__(self) -> None:
        super().__init__()
        self.rx = deque[bytes]()
        self.write_calls: list[bytes] = []
        self.request_log: list[tuple[int, int, int]] = []
        self.hardware_write_count = 0
        self.fail_config_id: int | None = None
        self.timeout_next = False
        self.mismatch_next = False

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        self.request_log.append((parsed.motor_id, parsed.command, parsed.register_addr))
        if (
            self.fail_config_id == parsed.motor_id
            and parsed.command == 0x32
            and parsed.register_addr == 0x25
        ):
            raise OSError(f"mock config failure on ID{parsed.motor_id}")
        if self.timeout_next:
            self.timeout_next = False
            raise TimeoutError("mock feedback timeout")
        if self.mismatch_next:
            self.mismatch_next = False
            return super().request(pack_read_status_cmd(2))
        return super().request(frame, **kwargs)

    def write(self, frame: bytes) -> int:
        parsed = parse_frame(frame)
        if (
            self.fail_config_id == parsed.motor_id
            and parsed.command == 0x32
            and parsed.register_addr == 0x25
        ):
            raise OSError(f"mock config failure on ID{parsed.motor_id}")
        self.write_calls.append(frame)
        self.rx.append(super().request(frame))
        return len(frame)

    def read_available(self) -> bytes:
        data = b"".join(self.rx)
        self.rx.clear()
        return data


def make_runner(
    *,
    clock: Clock | None = None,
    pause_events: list[str] | None = None,
) -> tuple[Clock, RunnerMockBus, J7ServoExperimentRunner]:
    clock = clock or Clock()
    bus = RunnerMockBus()
    writer = J7SixAxisServoTransport(
        bus,
        clock_ns=clock.now_ns,
        sleep_fn=clock.sleep,
        pause_callback=(pause_events.append if pause_events is not None else None),
    )
    feedback = J7ServoFeedbackScheduler(writer, clock_ns=clock.now_ns)
    runner = J7ServoExperimentRunner(
        writer,
        feedback,
        speed=50,
        watchdog_timeout_s=0.100,
        clock_ns=clock.now_ns,
    )
    return clock, bus, runner


def prepare_armed(runner: J7ServoExperimentRunner) -> None:
    runner.check()
    runner.configure_servo()
    runner.verify()
    runner.arm()
    assert runner.state == RunnerState.ARMED


def test_j7_c7_configuration_failure_never_arms() -> None:
    pauses: list[str] = []
    _clock, bus, runner = make_runner(pause_events=pauses)
    runner.check()
    bus.fail_config_id = 3

    with pytest.raises(ServoRunnerError, match="config failure"):
        runner.configure_servo()

    assert runner.state == RunnerState.STOP
    assert not runner.armed
    assert runner.transport.configured is None
    assert runner.audit_snapshot().stop_count == 1
    assert pauses == ["servo_startup_write_failure"]
    assert bus.hardware_write_count == 0


def test_j7_c7_park_home_servo_stream_starts_after_all_gates() -> None:
    clock, bus, runner = make_runner()
    prepare_armed(runner)

    park_audit, _ = runner.run_cycle(PARK_TARGET, cycle_id=1, target_timestamp_ns=clock.now_ns())
    home_audit, _ = runner.run_cycle(HOME_TARGET, cycle_id=2, target_timestamp_ns=clock.now_ns())

    assert runner.state == RunnerState.SERVO_RUN
    assert park_audit.tx_records[0].target_position == 0
    assert home_audit.tx_records[0].target_position == 1000
    motion_frames = [
        parse_frame(frame) for frame in bus.write_calls
        if parse_frame(frame).register_addr == 0x29
    ]
    assert [frame.motor_id for frame in motion_frames] == list(MOTOR_IDS) * 2
    assert all(frame.register_addr == 0x29 for frame in motion_frames)
    assert bus.hardware_write_count == 0


def test_j7_c7_write_first_start_stream_holds_check_positions() -> None:
    clock, bus, runner = make_runner()
    runner.check()
    runner.configure_servo()

    first_audit, _ = runner.start_servo_stream(
        cycle_id=10,
        target_timestamp_ns=clock.now_ns(),
    )

    assert runner.state == RunnerState.SERVO_RUN
    assert [record.target_position for record in first_audit.tx_records] == [50] * 6
    motion_frames = [
        parse_frame(frame) for frame in bus.write_calls
        if parse_frame(frame).register_addr == 0x29
    ]
    assert [frame.motor_id for frame in motion_frames] == list(MOTOR_IDS)
    assert [parse_frame(frame).register_addr for frame in bus.write_calls] == [0x29] * 6
    verify = runner.verify()
    assert all(sample.mode is None for sample in verify.samples)
    assert bus.hardware_write_count == 0


def test_j7_c7_runtime_gates_arm_without_mode_reads_or_mode_readback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    pauses: list[str] = []
    _clock, bus, runner = make_runner(pause_events=pauses)
    bus.modes.update({motor_id: 0 for motor_id in MOTOR_IDS})
    original_request = bus.request

    def reject_runtime_mode_read(frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        if parsed.command == 0x31:
            raise AssertionError("runtime Servo gate must not issue mode read 0x31")
        return original_request(frame, **kwargs)

    check_audit = runner.check()
    runner.configure_servo()
    runtime_request_start = len(bus.request_log)
    monkeypatch.setattr(bus, "request", reject_runtime_mode_read)

    verify_audit = runner.verify()
    runner.arm()

    assert runner.state == RunnerState.ARMED
    assert all(sample.fault == 0 for sample in check_audit.samples)
    assert all(not sample.timeout for sample in check_audit.samples)
    assert all(sample.mode is None for sample in check_audit.samples)
    assert all(sample.fault == 0 for sample in verify_audit.samples)
    assert all(not sample.timeout for sample in verify_audit.samples)
    assert all(sample.mode is None for sample in verify_audit.samples)
    assert [command for _, command, _ in bus.request_log[runtime_request_start:]] == [0x30] * 6
    assert runner.audit_snapshot().stop_count == 0
    assert runner.audit_snapshot().stop_reason is None
    assert pauses == []
    assert bus.hardware_write_count == 0


def test_j7_c7_runtime_status_fault_stops_servo_once() -> None:
    pauses: list[str] = []
    clock, bus, runner = make_runner(pause_events=pauses)
    prepare_armed(runner)
    runtime_request_start = len(bus.request_log)
    bus.faults.update({motor_id: 7 for motor_id in MOTOR_IDS})

    with pytest.raises(ServoRunnerError, match=r"fault=7"):
        runner.run_cycle(
            HOME_TARGET,
            cycle_id=30,
            target_timestamp_ns=clock.now_ns(),
        )

    runtime_requests = bus.request_log[runtime_request_start:]
    assert runner.state == RunnerState.STOP
    assert runner.audit_snapshot().stop_count == 1
    assert runner.audit_snapshot().stop_reason == "feedback_fault"
    assert runner.transport.stopped
    assert pauses == ["feedback_fault"]
    assert [command for _, command, _ in runtime_requests] == [0x30]
    assert bus.hardware_write_count == 0


def test_j7_c7_start_stream_status_fault_stops_servo_once() -> None:
    pauses: list[str] = []
    clock, bus, runner = make_runner(pause_events=pauses)
    runner.check()
    runner.configure_servo()
    bus.faults.update({motor_id: 7 for motor_id in MOTOR_IDS})

    with pytest.raises(ServoRunnerError, match=r"fault=7"):
        runner.start_servo_stream(
            cycle_id=31,
            target_timestamp_ns=clock.now_ns(),
        )

    assert runner.state == RunnerState.STOP
    assert runner.audit_snapshot().stop_count == 1
    assert runner.audit_snapshot().stop_reason == "feedback_fault"
    assert pauses == ["feedback_fault"]
    assert bus.hardware_write_count == 0


def test_j7_c7_feedback_exception_stops_runner_once() -> None:
    pauses: list[str] = []
    clock, bus, runner = make_runner(pause_events=pauses)
    prepare_armed(runner)
    bus.mismatch_next = True

    with pytest.raises(ServoRunnerError, match="response ID"):
        runner.run_cycle(HOME_TARGET, cycle_id=3, target_timestamp_ns=clock.now_ns())

    assert runner.state == RunnerState.STOP
    assert runner.audit_snapshot().stop_count == 1
    assert pauses == ["feedback_mismatch"]
    assert bus.hardware_write_count == 0


def test_j7_c7_watchdog_stops_stale_target_stream() -> None:
    pauses: list[str] = []
    clock, bus, runner = make_runner(clock=Clock(), pause_events=pauses)
    prepare_armed(runner)
    runner.run_cycle(PARK_TARGET, cycle_id=4, target_timestamp_ns=clock.now_ns())

    clock.advance_ns(101_000_000)
    assert runner.watchdog_tick()
    assert runner.state == RunnerState.STOP
    assert runner.audit_snapshot().watchdog_count == 1
    assert pauses == ["watchdog_timeout"]
    assert bus.hardware_write_count == 0


def test_j7_c7_stop_is_idempotent_and_only_calls_transport_once() -> None:
    pauses: list[str] = []
    _clock, bus, runner = make_runner(pause_events=pauses)
    prepare_armed(runner)

    runner.stop("operator_request")
    runner.stop("second_request")

    assert runner.state == RunnerState.STOP
    assert runner.audit_snapshot().stop_count == 1
    assert runner.audit_snapshot().stop_reason == "operator_request"
    assert pauses == ["operator_request"]
    assert bus.hardware_write_count == 0
