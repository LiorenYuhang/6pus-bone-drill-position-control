import json
import sys
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from run_j2_b_bringup import J2BRunner  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import (  # noqa: E402
    HardwareMode,
    ReadOnlyModeError,
    TargetValidationError,
)
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


def test_j2b_mock_complete_lifecycle_and_json(tmp_path):
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    log_file = tmp_path / "j2b_mock.json"
    result = J2BRunner(
        controller,
        motor_id=3,
        delta_steps=2,
        timeout_sec=0.2,
        poll_period_sec=0.001,
        log_file=log_file,
    ).run()

    assert result["outcome"] == "PASS"
    assert result["arrival"]["arrived"] is True
    assert result["stop_confirmation"]["confirmed"] is True
    assert result["target_configuration"] == {
        "motor_id": 3,
        "control_mode": 2,
        "target_speed": 50,
        "target_position": 50,
        "actual_position": 50,
        "current_mA": 0,
        "fault": 0,
    }
    assert abs(result["final_error_steps"]) <= 3
    assert result["write_audit"]["controller_write_count"] == 2
    assert result["write_audit"]["pass"] is True
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2], event[3]) for event in writes] == [
        (3, 0x29, [52]),
        (3, 0x1A, [1]),
    ]
    assert all(event[0] == 3 for event in writes)

    saved = json.loads(log_file.read_text(encoding="utf-8"))
    assert saved["pre_flight"]["snapshot"]["fault_codes"] == [0, 0, 0, 0, 0, 0]
    assert saved["post_flight"]["fault_codes"] == [0, 0, 0, 0, 0, 0]
    assert saved["write_audit"]["pass"] is True


def test_j2b_mock_timeout_pauses_target_once(tmp_path):
    class StalledBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                original = self.positions[parsed.motor_id]
                response = super().request(frame, **kwargs)
                self.positions[parsed.motor_id] = original
                return response
            return super().request(frame, **kwargs)

    bus = StalledBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=1,
        delta_steps=1,
        timeout_sec=0.01,
        poll_period_sec=0.001,
        settle_samples=100,
        log_file=tmp_path / "timeout.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert result["arrival"]["arrived"] is False
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2]) for event in writes] == [(1, 0x29), (1, 0x1A)]
    assert result["stop_confirmation"]["confirmed"] is True


def test_j2b_read_only_is_rejected_without_writes(tmp_path):
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)
    result = J2BRunner(
        controller,
        motor_id=2,
        delta_steps=1,
        log_file=tmp_path / "readonly.json",
    ).run()

    assert result["outcome"] == "FAIL"
    assert result["write_audit"]["controller_write_count"] == 0
    assert result["write_audit"]["cmd_0x32_tx_count"] == 0
    assert not [event for event in bus.events if event[1] == 0x32]


def test_j2b_cli_requires_second_real_motion_gate():
    from run_j2_b_bringup import main

    with pytest.raises(SystemExit):
        main(["--motor-id", "1", "--delta-steps", "1", "--real-hardware"])


def test_j2b_pause_single_axis_is_read_only_blocked():
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)
    controller.connect()
    with pytest.raises(ReadOnlyModeError):
        controller.pause_single_axis(1)
    assert controller.write_count == 0
    assert not [event for event in bus.events if event[1] == 0x32]
    controller.close()


def test_j2b_configuration_mismatch_stops_before_motion_write(tmp_path):
    bus = FakeBus()
    bus.modes[4] = 0
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        log_file=tmp_path / "config_mismatch.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert result["target_configuration"]["control_mode"] == 0
    assert "configuration rejected" in result["error"]
    assert not [event for event in bus.events if event[1] == 0x32]
    assert result["write_audit"]["controller_write_count"] == 0
    assert result["write_audit"]["pass"] is True


def test_j2b_mock_configuration_uses_one_validated_block_then_motion(tmp_path):
    bus = FakeBus()
    bus.modes[4] = 0
    bus.speeds[4] = 0xFFFF
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=0.2,
        poll_period_sec=0.001,
        allow_mock_configuration=True,
        log_file=tmp_path / "config_then_motion.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["configuration_write"] == {
        "motor_id": 4,
        "control_mode": 2,
        "target_speed": 50,
        "target_position": 50,
        "actual_position": 50,
        "current_mA": 0,
        "fault": 0,
    }
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2], event[3]) for event in writes] == [
        (4, 0x25, [2, 0, 0, 50, 50]),
        (4, 0x29, [53]),
        (4, 0x1A, [1]),
    ]
    assert result["write_audit"]["controller_write_count"] == 3
    assert result["write_audit"]["pass"] is True
    assert all(event[0] == 4 for event in writes)
    assert [
        (frame["motor_id"], frame["register_address"], frame["payload"])
        for frame in result["write_audit"]["write_frames"]
    ] == [
        (4, 0x25, [2, 0, 0, 50, 50]),
        (4, 0x29, [53]),
        (4, 0x1A, [1]),
    ]
    assert all(frame["timestamp_ns"] > 0 for frame in result["write_audit"]["write_frames"])
    assert result["write_audit"]["per_axis_write_counts"] == {
        "1": 0,
        "2": 0,
        "3": 0,
        "4": 3,
        "5": 0,
        "6": 0,
    }
    assert result["write_audit"]["observer_zero_writes"] is True
    assert result["write_audit"]["sequence_complete"] is True


def test_j2b_active_validated_config_has_exactly_three_id4_writes(tmp_path):
    bus = FakeBus()
    bus.modes[4] = 0
    bus.speeds[4] = 0xFFFF
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        validated_config=True,
        log_file=tmp_path / "active_gate_fake.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["configuration_verified"] is True
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2], event[3]) for event in writes] == [
        (4, 0x25, [2, 0, 0, 50, 50]),
        (4, 0x29, [53]),
        (4, 0x1A, [1]),
    ]
    assert result["write_audit"]["controller_write_count"] == 3
    assert all(event[0] == 4 for event in writes)


def test_j2b_active_runner_requires_validated_config_before_bus_access():
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)

    with pytest.raises(TargetValidationError, match="validated_config"):
        J2BRunner(controller, motor_id=4, delta_steps=3, timeout_sec=3.0)
    assert bus.events == []


def test_j2b_active_validated_config_runs_setup_when_preflight_already_matches(tmp_path):
    bus = FakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        validated_config=True,
        log_file=tmp_path / "active_matching_config_fake.json",
    ).run()

    assert result["outcome"] == "PASS"
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2], event[3]) for event in writes] == [
        (4, 0x25, [2, 0, 0, 50, 50]),
        (4, 0x29, [53]),
        (4, 0x1A, [1]),
    ]


def test_j2b_config_readback_failure_stops_before_target(tmp_path):
    class ReadbackFailureBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x25:
                self.modes[parsed.motor_id] = 1
            return response

    bus = ReadbackFailureBus()
    bus.modes[4] = 0
    bus.speeds[4] = 0xFFFF
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        validated_config=True,
        log_file=tmp_path / "config_readback_failure.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert result["configuration_attempted"] is True
    assert result["configuration_verified"] is False
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2]) for event in writes] == [(4, 0x25)]
    assert result["write_audit"]["pass"] is True


def test_j2b_config_write_exception_is_not_retried(tmp_path):
    bus = FakeBus()
    bus.modes[4] = 0
    bus.speeds[4] = 0xFFFF
    bus.fail_on_id = 4
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        validated_config=True,
        log_file=tmp_path / "config_write_exception.json",
    ).run()

    assert result["outcome"] == "STOP"
    writes = [event for event in bus.events if event[1] == 0x32]
    assert [(event[0], event[2]) for event in writes] == [(4, 0x25)]
    assert result["write_audit"]["cmd_0x32_tx_count"] == 1
    assert result["write_audit"]["controller_write_count"] == 0
    assert result["write_audit"]["pass"] is True


def test_j2b_pause_trigger_at_two_and_final_error_within_three(tmp_path):
    class TwoStepErrorBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                self.positions[parsed.motor_id] = self.targets[parsed.motor_id] - 2
            return response

    bus = TwoStepErrorBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        log_file=tmp_path / "two_step_arrival.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["arrival"]["arrived"] is True
    assert result["feedback_samples"][-1]["error_steps"] == -2
    assert result["final_error_steps"] == -2
    assert result["pause"]["attempted"] is True


@pytest.mark.parametrize("running_current_ma", [9, 47])
def test_j2b_runtime_normal_motion_current_does_not_stop(tmp_path, running_current_ma):
    class RuntimeCurrentBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x1A:
                self.currents[parsed.motor_id] = 0
                self.faults[parsed.motor_id] = 0
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                self.currents[parsed.motor_id] = running_current_ma
            return response

    bus = RuntimeCurrentBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        log_file=tmp_path / f"runtime_current_{running_current_ma}.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["feedback_samples"]
    assert result["feedback_samples"][0]["target_current_mA"] == running_current_ma
    assert result["stop_confirmation"]["confirmed"] is True
    assert result["post_flight"]["currents_mA"][3] == 0


def test_j2b_runtime_current_above_500ma_stops_and_recovers_to_zero(tmp_path):
    class OverCurrentBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x1A:
                self.currents[parsed.motor_id] = 0
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                self.currents[parsed.motor_id] = 501
            return response

    bus = OverCurrentBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        log_file=tmp_path / "over_current.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert "exceeds threshold 500mA" in result["error"]
    assert result["stop_confirmation"]["confirmed"] is True
    assert result["post_flight"]["currents_mA"][3] == 0


def test_j2b_runtime_fault_stops_and_confirms_clear_fault(tmp_path):
    class FaultBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x1A:
                self.currents[parsed.motor_id] = 0
                self.faults[parsed.motor_id] = 0
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                self.faults[parsed.motor_id] = 0x04
            return response

    bus = FaultBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        log_file=tmp_path / "runtime_fault.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert "error code 0x04" in result["error"]
    assert result["stop_confirmation"]["confirmed"] is True
    assert result["post_flight"]["fault_codes"][3] == 0


def test_j2b_stop_confirmation_requires_current_to_return_to_zero(tmp_path):
    class PersistentCurrentBus(FakeBus):
        def request(self, frame: bytes, **kwargs) -> bytes:
            from pus6_motor_driver.protocol import CMD_WR_REGISTER, parse_frame

            parsed = parse_frame(frame)
            response = super().request(frame, **kwargs)
            if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
                self.currents[parsed.motor_id] = 9
            return response

    bus = PersistentCurrentBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2BRunner(
        controller,
        motor_id=4,
        delta_steps=3,
        timeout_sec=3.0,
        poll_period_sec=0.001,
        log_file=tmp_path / "persistent_current.json",
    ).run()

    assert result["outcome"] == "STOP"
    assert result["stop_confirmation"]["confirmed"] is False
    assert result["stop_confirmation"]["hard_stop_required"] is True
    assert result["post_flight"]["currents_mA"][3] == 9
    assert "HARD STOP REQUIRED" in result["error"]


@pytest.mark.parametrize(
    "argv",
    [
        ["--real-hardware"],
        ["--real-hardware", "--confirm-motion"],
        ["--confirm-motion", "--validated-config"],
    ],
)
def test_j2b_real_cli_missing_authorization_has_no_controller_access(monkeypatch, argv):
    import run_j2_b_bringup as runner_module

    def fail_controller(*args, **kwargs):
        raise AssertionError("controller must not be constructed without validated-config")

    monkeypatch.setattr(runner_module, "SixMotorController", fail_controller)
    with pytest.raises(SystemExit):
        runner_module.main(argv)
