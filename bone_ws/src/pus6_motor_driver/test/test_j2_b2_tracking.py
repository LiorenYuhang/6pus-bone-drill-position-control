import sys
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from run_j2_b2_tracking import J2B2TrackingRunner  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.safety import HardwareMode  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


class TrackingFakeBus(FakeBus):
    """Deterministic +10-step trajectory with nonzero runtime current."""

    def __init__(self) -> None:
        super().__init__()
        self._tracking = False
        self._tracking_read_index = 0
        self._positions = [50, 51, 54, 57, 58, 58]
        self._currents = [5, 10, 15, 20, 18, 16]

    def request(self, frame: bytes, **kwargs) -> bytes:
        from pus6_motor_driver.protocol import CMD_RD_REGISTER, CMD_RD_STATUS, CMD_WR_REGISTER, parse_frame

        parsed = parse_frame(frame)
        if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
            response = super().request(frame, **kwargs)
            self.positions[parsed.motor_id] = 50
            self._tracking = True
            self._tracking_read_index = 0
            return response
        if parsed.command == CMD_RD_STATUS and parsed.motor_id == 4 and self._tracking:
            index = min(self._tracking_read_index, len(self._positions) - 1)
            self.positions[4] = self._positions[index]
            self.currents[4] = self._currents[index]
            self._tracking_read_index += 1
            return super().request(frame, **kwargs)
        if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x1A:
            self._tracking = False
            self.positions[4] = 58
            self.currents[4] = 0
        return super().request(frame, **kwargs)


def test_j2b2_mock_tracking_metrics_and_three_frame_audit(tmp_path):
    bus = TrackingFakeBus()
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2B2TrackingRunner(
        controller,
        log_file=tmp_path / "j2b2_tracking.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["stage"] == "J2-B2"
    assert result["motor_id"] == 4
    assert result["delta_steps"] == 10
    assert result["arm"]["baseline_position"] == 50
    assert result["arm"]["target_step"] == 60

    tracking = result["tracking"]
    assert tracking["first_positive_motion_latency_ms"] > 0
    assert tracking["time_to_error_lte_3_ms"] > 0
    assert tracking["time_to_error_lte_2_ms"] > tracking["time_to_error_lte_3_ms"]
    assert tracking["peak_current_mA"] == 20
    assert tracking["final_displacement_steps"] == 8
    assert tracking["final_target_error_steps"] == -2
    assert len(tracking["position_vs_time_samples"]) >= 6
    assert tracking["position_vs_time_samples"][0]["error_steps"] == -10

    audit = result["write_audit"]
    assert audit["pass"] is True
    assert [
        (frame["motor_id"], frame["register_address"], frame["payload"])
        for frame in audit["write_frames"]
    ] == [
        (4, 0x25, [2, 0, 0, 50, 50]),
        (4, 0x29, [60]),
        (4, 0x1A, [1]),
    ]
    assert audit["per_axis_write_counts"] == {
        "1": 0,
        "2": 0,
        "3": 0,
        "4": 3,
        "5": 0,
        "6": 0,
    }
    assert audit["observer_zero_writes"] is True


def test_j2b2_mock_always_uses_validated_configuration_block(tmp_path):
    bus = FakeBus()
    bus.modes[4] = 0
    bus.speeds[4] = 0xFFFF
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2B2TrackingRunner(
        controller,
        log_file=tmp_path / "j2b2_config.json",
    ).run()

    assert result["outcome"] == "PASS"
    assert result["configuration_attempted"] is True
    assert result["configuration_verified"] is True
    assert [event[2] for event in bus.events if event[1] == 0x32] == [0x25, 0x29, 0x1A]


@pytest.mark.parametrize(
    "argv",
    [
        ["--real-hardware"],
        ["--real-hardware", "--confirm-motion"],
        ["--real-hardware", "--confirm-motion", "--validated-config"],
    ],
)
def test_j2b2_real_cli_requires_all_gates_without_controller_access(monkeypatch, argv):
    import run_j2_b2_tracking as runner_module

    def fail_controller(*args, **kwargs):
        raise AssertionError("controller must not be constructed without all B2 gates")

    monkeypatch.setattr(runner_module, "SixMotorController", fail_controller)
    with pytest.raises(SystemExit):
        runner_module.main(argv)


def test_j2b2_cli_rejects_non_fixed_parameters():
    from run_j2_b2_tracking import main

    with pytest.raises(SystemExit):
        main(["--delta-steps", "3"])


class BaselineSequenceBus(FakeBus):
    def __init__(self, positions: list[int], target: int | list[int]) -> None:
        super().__init__()
        self._baseline_positions = list(positions)
        self._status_reads = 0
        self._target_reads = 0
        self._configured = False
        self.positions[4] = self._baseline_positions[0]
        self._baseline_targets = [target] if isinstance(target, int) else list(target)
        self.targets[4] = self._baseline_targets[0]

    def request(self, frame: bytes, **kwargs):
        from pus6_motor_driver.protocol import CMD_RD_REGISTER, CMD_RD_STATUS, CMD_WR_REGISTER, parse_frame

        parsed = parse_frame(frame)
        if (
            parsed.command == CMD_RD_REGISTER
            and parsed.motor_id == 4
            and parsed.register_addr == 0x29
            and not self._configured
        ):
            index = min(self._target_reads, len(self._baseline_targets) - 1)
            self.targets[4] = self._baseline_targets[index]
            self._target_reads += 1
        if parsed.command == CMD_RD_STATUS and parsed.motor_id == 4 and not self._configured:
            index = min(self._status_reads, len(self._baseline_positions) - 1)
            self.positions[4] = self._baseline_positions[index]
            self._status_reads += 1
        if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x25:
            self._configured = True
        return super().request(frame, **kwargs)

def _run_baseline_sequence(tmp_path, positions: list[int], target: int | list[int]):
    bus = BaselineSequenceBus(positions, target)
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    result = J2B2TrackingRunner(
        controller, log_file=tmp_path / "baseline_sequence.json"
    ).run()
    writes = [event for event in bus.events if event[1] == 0x32]
    return result, writes

def test_j2b2_baseline_one_step_jitter_preserves_target_reference(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [64, 64, 63, 64, 63, 64, 63, 64, 63, 64, 63], 63
    )
    assert result["outcome"] == "PASS"
    analysis = result["baseline_analysis"]
    assert analysis["static_band"] == [63, 64]
    assert analysis["span"] == 1
    assert analysis["reference"] == 63
    assert analysis["selection_reason"] == "actual_histogram_mode"
    assert writes[0][0:4] == (4, 0x32, 0x25, [2, 0, 0, 50, 63])
    assert result["write_audit"]["per_axis_write_counts"] == {
        "1": 0, "2": 0, "3": 0, "4": 3, "5": 0, "6": 0
    }

def test_j2b2_baseline_outside_reference_deadband_stops_before_write(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63, 64, 67, 64, 63, 64, 67, 64, 63, 64, 67], 63
    )
    assert result["outcome"] == "STOP"
    assert "actual/reference deviation=3" in result["error"]
    assert writes == []
    assert result["write_audit"]["controller_write_count"] == 0

def test_j2b2_baseline_monotonic_drift_stops_before_write(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64], 63
    )
    assert result["outcome"] == "STOP"
    assert "one-direction drift" in result["error"]
    assert writes == []
    assert result["write_audit"]["controller_write_count"] == 0

def test_j2b2_target_outside_static_reference_band_stops_before_write(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63] * 11, 66
    )
    assert result["outcome"] == "STOP"
    assert "0x29/reference deviation=3" in result["error"]
    assert writes == []
    assert result["write_audit"]["controller_write_count"] == 0


def test_j2b2_actual_static_target_jitter_passes(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63] * 11, [63, 64, 63, 64, 63, 64, 63, 64, 63, 64]
    )
    assert result["outcome"] == "PASS"
    analysis = result["baseline_analysis"]
    assert analysis["actual_static_band"] == [63, 63]
    assert analysis["target_static_band"] == [63, 64]
    assert analysis["actual_histogram_mode"] == 63
    assert analysis["reference"] == 63
    assert analysis["reference_band"] == [61, 65]
    assert writes[0][3] == [2, 0, 0, 50, 63]

def test_j2b2_actual_and_target_one_step_jitter_passes(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path,
        [63, 63, 64, 63, 64, 63, 64, 63, 64, 63, 64],
        [63, 64, 63, 64, 63, 64, 63, 64, 63, 64],
    )
    assert result["outcome"] == "PASS"
    analysis = result["baseline_analysis"]
    assert analysis["actual_static_band"] == [63, 64]
    assert analysis["target_static_band"] == [63, 64]
    assert analysis["reference"] == 63
    assert writes[0][3] == [2, 0, 0, 50, 63]

def test_j2b2_target_monotonic_drift_stops_before_write(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63] * 11, [63, 63, 63, 63, 64, 64, 64, 64, 64, 64]
    )
    assert result["outcome"] == "STOP"
    assert "0x29 sustained one-direction drift" in result["error"]
    assert writes == []
    assert result["write_audit"]["controller_write_count"] == 0


def test_j2b2_target_raw_state_two_steps_from_reference_passes(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63] * 11, [61, 65, 61, 65, 61, 65, 61, 65, 61, 65]
    )
    assert result["outcome"] == "PASS"
    analysis = result["baseline_analysis"]
    assert analysis["reference"] == 63
    assert analysis["target_max_reference_deviation"] == 2
    assert writes[0][3] == [2, 0, 0, 50, 63]


def test_j2b2_actual_raw_state_two_steps_from_reference_passes(tmp_path):
    result, writes = _run_baseline_sequence(
        tmp_path, [63, 65, 63, 61, 63, 65, 63, 61, 63, 65, 63], 63
    )
    assert result["outcome"] == "PASS"
    analysis = result["baseline_analysis"]
    assert analysis["reference"] == 63
    assert analysis["actual_max_reference_deviation"] == 2
    assert writes[0][3] == [2, 0, 0, 50, 63]
