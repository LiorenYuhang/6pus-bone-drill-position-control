import sys
from pathlib import Path

import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

import run_j2_c_vector as runner_module  # noqa: E402
from run_j2_c_vector import DELTA_STEPS, J2CVectorRunner  # noqa: E402
from pus6_motor_driver.fake_bus import FakeBus  # noqa: E402
from pus6_motor_driver.protocol import (  # noqa: E402
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    CMD_WR_REGISTER,
    parse_frame,
)
from pus6_motor_driver.safety import HardwareMode  # noqa: E402
from pus6_motor_driver.six_motor_controller import SixMotorController  # noqa: E402


class VectorMotionBus(FakeBus):
    """Fake six-axis bus with deterministic sequential multi-phase motion feedback."""

    def __init__(
        self,
        *,
        runtime_current: int = 20,
        runtime_fault: int = 0,
        fail_target_id: int | None = None,
        readback_mode_id: int | None = None,
        stalled: bool = False,
        static_one_step_ids: set[int] | None = None,
        read_only: bool = False,
    ) -> None:
        super().__init__(read_only=read_only)
        self.runtime_current = runtime_current
        self.runtime_fault = runtime_fault
        self.fail_target_id = fail_target_id
        self.readback_mode_id = readback_mode_id
        self.stalled = stalled
        self.static_one_step_ids = static_one_step_ids or set()
        self.configured_ids: set[int] = set()
        self.target_ids: set[int] = set()
        self.motion_started = False
        self.paused = False
        self.poll_cycle = 0
        self.baselines = {mid: 50 for mid in range(1, 7)}
        self.commanded_targets = {mid: 50 for mid in range(1, 7)}
        self.phase_start_positions = {mid: 50 for mid in range(1, 7)}

    def _trajectory_position(self, mid: int, cycle: int) -> int:
        start_pos = self.phase_start_positions[mid]
        target = self.commanded_targets[mid]
        if self.stalled:
            return start_pos
        if mid in self.static_one_step_ids:
            return start_pos + 1 if cycle == 1 else start_pos
        diff = target - start_pos
        if diff == 0:
            return target
        sign = 1 if diff > 0 else -1
        mag = abs(diff)
        increments = [0, sign * 1, sign * 2, sign * max(0, mag - 2)]
        return start_pos + increments[min(cycle, len(increments) - 1)]

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)

        if (
            parsed.command == CMD_RD_REGISTER
            and parsed.motor_id == self.readback_mode_id
            and parsed.register_addr == 0x25
            and parsed.motor_id in self.configured_ids
        ):
            self.modes[parsed.motor_id] = 1

        if parsed.command == CMD_RD_STATUS and self.motion_started and not self.paused:
            cycle = self.poll_cycle
            self.positions[parsed.motor_id] = self._trajectory_position(parsed.motor_id, cycle)
            self.currents[parsed.motor_id] = self.runtime_current
            self.faults[parsed.motor_id] = self.runtime_fault
            if parsed.motor_id == 6:
                self.poll_cycle += 1

        if parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x25:
            self.configured_ids.add(parsed.motor_id)
        elif parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x29:
            val = int.from_bytes(parsed.data[:2], "little") if len(parsed.data) >= 2 else 50
            self.commanded_targets[parsed.motor_id] = val
            if parsed.motor_id == 1:
                self.target_ids = {1}
                self.phase_start_positions = dict(self.positions)
            else:
                self.target_ids.add(parsed.motor_id)

            if self.target_ids == set(range(1, 7)):
                self.motion_started = True
                self.paused = False
                self.poll_cycle = 0
                for mid in range(1, 7):
                    self.positions[mid] = self.phase_start_positions[mid]
                    self.currents[mid] = 0

        elif parsed.command == CMD_WR_REGISTER and parsed.register_addr == 0x1A:
            self.paused = True
            for mid in range(1, 7):
                self.positions[mid] = self._trajectory_position(mid, max(self.poll_cycle, 3))
                self.currents[mid] = 0
                self.faults[mid] = 0

        response = super().request(frame, **kwargs)

        if (
            parsed.command == CMD_WR_REGISTER
            and parsed.register_addr == 0x29
            and parsed.motor_id == self.fail_target_id
        ):
            raise OSError(f"Simulated target response failure on ID{parsed.motor_id}")

        return response


class BaselinePatternBus(VectorMotionBus):
    """Inject deterministic raw baseline sequences without changing writes."""

    def __init__(
        self,
        *,
        actual_patterns: dict[int, list[int]] | None = None,
        target_patterns: dict[int, list[int]] | None = None,
    ) -> None:
        super().__init__()
        self.actual_patterns = actual_patterns or {}
        self.target_patterns = target_patterns or {}
        self.actual_reads = {mid: 0 for mid in range(1, 7)}
        self.target_reads = {mid: 0 for mid in range(1, 7)}

    def request(self, frame: bytes, **kwargs) -> bytes:
        parsed = parse_frame(frame)
        if parsed.command == CMD_RD_STATUS and not self.motion_started:
            pattern = self.actual_patterns.get(parsed.motor_id)
            if pattern:
                index = min(self.actual_reads[parsed.motor_id], len(pattern) - 1)
                self.positions[parsed.motor_id] = pattern[index]
                self.actual_reads[parsed.motor_id] += 1
        elif parsed.command == CMD_RD_REGISTER and parsed.register_addr == 0x29:
            pattern = self.target_patterns.get(parsed.motor_id)
            if pattern:
                index = min(self.target_reads[parsed.motor_id], len(pattern) - 1)
                self.targets[parsed.motor_id] = pattern[index]
                self.target_reads[parsed.motor_id] += 1
        return super().request(frame, **kwargs)


def _run(tmp_path, bus: FakeBus, cycles: int = 1, return_to_ref: bool = False):
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    runner = J2CVectorRunner(
        controller,
        cycles=cycles,
        return_to_ref=return_to_ref,
        log_file=tmp_path / "j2c.json",
    )
    return runner.run(), bus


def _write_frames(bus: FakeBus):
    return [event for event in bus.events if event[1] == CMD_WR_REGISTER]


def test_j2c_success_has_exact_18_frame_order_and_timestamps(tmp_path):
    result, bus = _run(tmp_path, VectorMotionBus(), cycles=1, return_to_ref=False)

    assert result["outcome"] == "PASS"
    assert result["references"] == {str(mid): 50 for mid in range(1, 7)}
    assert result["targets"] == {
        str(mid): 50 + DELTA_STEPS[mid - 1] for mid in range(1, 7)
    }
    frames = _write_frames(bus)
    assert len(frames) == 18
    assert [(event[0], event[2]) for event in frames] == (
        [(mid, 0x25) for mid in range(1, 7)]
        + [(mid, 0x29) for mid in range(1, 7)]
        + [(mid, 0x1A) for mid in range(1, 7)]
    )
    assert result["write_audit"]["sequence_is_expected"] is True
    assert result["write_audit"]["controller_write_count"] == 18
    assert result["write_audit"]["no_retry"] is True
    assert len(result["configuration_dispatch"]) == 6
    assert len(result["phases_summary"]) == 1
    phase0 = result["phases_summary"][0]
    assert len(phase0["dispatch"]["timestamps"]) == 6
    assert len(phase0["dispatch"]["completion_timestamps_ns"]) == 6
    assert all(
        phase0["dispatch"]["completion_timestamps_ns"][str(mid)]
        >= phase0["dispatch"]["timestamps_ns"][str(mid)]
        for mid in range(1, 7)
    )
    assert result["pause"]["hard_stop_required"] is False
    assert result["stop_confirmation"]["confirmed"] is True
    assert set(result["tracking"]["first_raw_positive_sample_latency_ms"]) == set(
        str(mid) for mid in range(1, 7)
    )
    assert set(result["tracking"]["first_confirmed_motion_latency_ms"]) == set(
        str(mid) for mid in range(1, 7)
    )
    assert all(
        result["tracking"]["first_confirmed_motion_latency_ms"][str(mid)]
        > result["tracking"]["first_raw_positive_sample_latency_ms"][str(mid)]
        for mid in range(1, 7)
    )
    sync = result["tracking"]["sync_metrics"]
    assert sync["all_axes_settled"] is True
    assert sync["settled_axes"] == list(range(1, 7))
    assert sync["start_skew_ms"] is not None
    assert sync["arrival_skew_ms"] is not None
    assert len(sync["final_positions_steps"]) == 6
    assert len(sync["final_errors_steps"]) == 6


def test_j2c_5_cycle_roundtrip_motion_audit_and_statistics(tmp_path):
    result, bus = _run(tmp_path, VectorMotionBus(), cycles=5, return_to_ref=True)

    assert result["outcome"] == "PASS"
    assert result["stage"] == "J2-C-CYCLIC"
    assert result["cycles"] == 5
    assert result["return_to_ref"] is True

    # 1. Write Audit: 6 config + 5 cycles * (6 fwd + 6 ret) + 6 pause = 6 + 60 + 6 = 72 writes
    frames = _write_frames(bus)
    assert len(frames) == 72
    assert result["write_audit"]["controller_write_count"] == 72
    assert result["write_audit"]["expected_write_count"] == 72
    assert result["write_audit"]["sequence_is_expected"] is True
    assert result["write_audit"]["no_retry"] is True

    # 2. Phases & Cycles
    assert len(result["phases_summary"]) == 10  # 5 forward + 5 return
    assert len(result["cycles_data"]) == 5
    for c_idx, c_data in enumerate(result["cycles_data"], start=1):
        assert c_data["cycle_index"] == c_idx
        assert c_data["forward"]["direction"] == "forward"
        assert c_data["return"]["direction"] == "return"
        assert c_data["forward"]["all_axes_settled"] is True
        assert c_data["return"]["all_axes_settled"] is True

    # 3. Forward vs Return Statistics
    fwd_stats = result["forward_stats"]
    assert "start_skew_ms" in fwd_stats
    assert fwd_stats["start_skew_ms"]["mean"] >= 0.0
    assert "arrival_skew_ms" in fwd_stats
    assert len(fwd_stats["per_axis_displacement_steps"]) == 6

    ret_stats = result["return_stats"]
    assert "start_skew_ms" in ret_stats
    assert ret_stats["start_skew_ms"]["mean"] >= 0.0
    assert "arrival_skew_ms" in ret_stats
    assert len(ret_stats["per_axis_displacement_steps"]) == 6

    # 4. Repeatability & Cumulative Drift
    rep = result["repeatability"]
    assert len(rep["forward_peak_positions_steps"]) == 6
    assert len(rep["return_positions_steps"]) == 6
    for mid in range(1, 7):
        assert len(rep["forward_peak_positions_steps"][str(mid)]["samples"]) == 5
        assert len(rep["return_positions_steps"][str(mid)]["samples"]) == 5

    drift = result["cumulative_drift"]
    assert len(drift["per_cycle_return_drift"]) == 5
    assert len(drift["final_net_drift_steps"]) == 6
    assert len(drift["final_net_drift_mm"]) == 6

    # 5. Target dispatch spread stats across all 10 dispatches
    dispatch_spread = result["target_dispatch_spread"]
    assert len(dispatch_spread["samples"]) == 10
    assert dispatch_spread["min"] >= 0.0
    assert dispatch_spread["max"] >= dispatch_spread["min"]


def test_j2c_one_step_baseline_change_is_raw_only_not_confirmed_motion(tmp_path):
    result, _ = _run(tmp_path, VectorMotionBus(static_one_step_ids={6}), cycles=1, return_to_ref=False)

    assert result["outcome"] == "PASS"
    raw = result["tracking"]["first_raw_positive_sample_latency_ms"]
    confirmed = result["tracking"]["first_confirmed_motion_latency_ms"]
    assert "6" in raw
    assert "6" not in confirmed
    assert result["stop_confirmation"] is not None
    sync = result["tracking"]["sync_metrics"]
    assert sync["all_axes_settled"] is False
    assert 6 not in sync["settled_axes"]


def test_j2c_invalid_target_vector_is_zero_write(tmp_path):
    bus = VectorMotionBus()
    bus.limits[1] = (0, 52)
    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "target vector validation failed" in result["error"]
    assert _write_frames(bus) == []
    assert result["write_audit"]["controller_write_count"] == 0


def test_j2c_baseline_failure_is_zero_write(tmp_path):
    bus = VectorMotionBus()
    bus.currents[3] = 5
    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "pre-flight current" in result["error"]
    assert _write_frames(bus) == []


def test_j2c_raw_target_outside_deadband_is_zero_write(tmp_path):
    bus = VectorMotionBus()
    bus.targets[4] = 53
    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "0x29/reference deviation=3" in result["error"]
    assert _write_frames(bus) == []


def test_j2c_raw_target_one_step_transition_is_admitted(tmp_path):
    bus = BaselinePatternBus(
        target_patterns={2: [75, 75, 75, 75, 75, 74, 74, 74, 74, 74]},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "PASS"
    assert result["references"]["2"] == 75
    assert result["baseline_analysis"]["2"]["target_drift"] == (
        "monotonic_decreasing start=75 end=74"
    )
    assert _write_frames(bus)


def test_j2c_raw_target_switches_within_reference_deadband_are_admitted(tmp_path):
    bus = BaselinePatternBus(
        target_patterns={2: [73, 74, 75, 76, 77, 73, 74, 75, 76, 77]},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "PASS"
    assert result["baseline_analysis"]["2"]["target_max_reference_deviation"] == 2


def test_j2c_raw_target_out_of_band_stops_before_any_write(tmp_path):
    bus = BaselinePatternBus(
        target_patterns={2: [75, 75, 75, 75, 75, 75, 75, 75, 75, 78]},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "0x29/reference deviation=3" in result["error"]
    assert _write_frames(bus) == []


def test_j2c_actual_sustained_drift_stops_even_when_target_is_in_band(tmp_path):
    bus = BaselinePatternBus(
        actual_patterns={2: [75, 75, 75, 76, 76, 77, 77, 77, 77, 77, 77]},
        target_patterns={2: [75] * 10},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "static baseline drift actual=monotonic_increasing" in result["error"]
    assert _write_frames(bus) == []


def test_j2c_stable_actual_with_one_step_target_change_does_not_false_stop(tmp_path):
    bus = BaselinePatternBus(
        actual_patterns={2: [75] * 11},
        target_patterns={2: [75] * 5 + [74] + [75] * 4},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "PASS"
    assert result["baseline_analysis"]["2"]["actual_drift"] is None


def test_j2c_raw_target_multistep_sustained_drift_stops_in_band(tmp_path):
    bus = BaselinePatternBus(
        target_patterns={2: [75, 74, 73, 73, 73, 73, 73, 73, 73, 73]},
    )
    bus.baselines[2] = 75
    bus.positions[2] = 75
    bus.targets[2] = 75

    result, bus = _run(tmp_path, bus)

    assert result["outcome"] == "STOP"
    assert "0x29 sustained drift" in result["error"]
    assert _write_frames(bus) == []


def test_j2c_config_readback_failure_sends_no_targets_and_pauses_all(tmp_path):
    result, bus = _run(tmp_path, VectorMotionBus(readback_mode_id=2))

    assert result["outcome"] == "STOP"
    assert "config readback mode" in result["error"]
    frames = _write_frames(bus)
    assert [event[2] for event in frames] == [0x25] * 6 + [0x1A] * 6
    assert result["write_audit"]["target_write_counts"] == {
        str(mid): 0 for mid in range(1, 7)
    }
    assert result["pause"]["all_axes_attempted"] == {
        str(mid): True for mid in range(1, 7)
    }
    assert result["stop_confirmation"] is not None
    assert result["stop_confirmation"]["confirmed"] is True


def test_j2c_target_mid_dispatch_failure_pauses_all_without_retry(tmp_path):
    result, bus = _run(tmp_path, VectorMotionBus(fail_target_id=3))

    assert result["outcome"] == "FAIL"
    frames = _write_frames(bus)
    assert [event[2] for event in frames[:6]] == [0x25] * 6
    assert [event[0] for event in frames[6:9]] == [1, 2, 3]
    assert [event[2] for event in frames[9:]] == [0x1A] * 6
    assert result["write_audit"]["target_write_counts"]["3"] == 1
    assert result["write_audit"]["no_retry"] is True
    assert result["pause"]["all_axes_attempted"] == {
        str(mid): True for mid in range(1, 7)
    }
    assert result["stop_confirmation"] is not None


@pytest.mark.parametrize(
    "kwargs, expected",
    [
        ({"runtime_current": 501}, "runtime current exceeds 500mA"),
        ({"runtime_fault": 4}, "fault error code 0x04"),
    ],
)
def test_j2c_runtime_current_or_fault_stops_and_pauses_all(tmp_path, kwargs, expected):
    result, bus = _run(tmp_path, VectorMotionBus(**kwargs))

    assert result["outcome"] == "STOP"
    assert expected in result["error"]
    assert result["pause"]["all_axes_attempted"] == {
        str(mid): True for mid in range(1, 7)
    }
    assert result["stop_confirmation"] is not None


def test_j2c_observation_timeout_pauses_all_and_records_unsettled_performance(tmp_path, monkeypatch):
    monkeypatch.setattr(runner_module, "TIMEOUT_PER_PHASE_SEC", 0.01)
    monkeypatch.setattr(runner_module, "POLL_PERIOD_SEC", 0.001)
    result, bus = _run(tmp_path, VectorMotionBus(stalled=True), cycles=1, return_to_ref=False)

    assert result["outcome"] == "PASS"
    assert result["pause"]["all_axes_attempted"] == {
        str(mid): True for mid in range(1, 7)
    }
    assert result["stop_confirmation"] is not None
    assert result["stop_confirmation"]["confirmed"] is True
    assert result["write_audit"]["controller_write_count"] == 18
    sync = result["tracking"]["sync_metrics"]
    assert sync["all_axes_settled"] is False
    assert sync["settled_axes"] == []


def test_j2c_candidate_vector_homogeneous_10_step(tmp_path):
    assert DELTA_STEPS == (10, 10, 10, 10, 10, 10)
    result, bus = _run(tmp_path, VectorMotionBus(), cycles=1, return_to_ref=False)
    assert result["outcome"] == "PASS"
    assert result["delta_steps"] == [10, 10, 10, 10, 10, 10]
    assert result["targets"] == {str(mid): 50 + 10 for mid in range(1, 7)}


@pytest.mark.parametrize(
    "argv",
    [
        ["--real-hardware"],
        ["--real-hardware", "--confirm-motion"],
        ["--real-hardware", "--confirm-motion", "--validated-config"],
    ],
)
def test_j2c_real_cli_requires_all_gates_before_controller(monkeypatch, argv):
    def fail_controller(*args, **kwargs):
        raise AssertionError("controller must not be constructed without all J2-C gates")

    monkeypatch.setattr(runner_module, "SixMotorController", fail_controller)
    with pytest.raises(SystemExit):
        runner_module.main(argv)


def test_j2c_read_only_runner_performs_zero_writes(tmp_path):
    bus = VectorMotionBus(read_only=True)
    controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)
    result = J2CVectorRunner(controller, log_file=tmp_path / "readonly.json").run()

    assert result["outcome"] == "STOP"
    assert _write_frames(bus) == []
    assert result["write_audit"]["controller_write_count"] == 0
