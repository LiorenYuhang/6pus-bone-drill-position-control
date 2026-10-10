"""Offline clock/slot accounting, no ROS or hardware execution."""
from j14_a2_q1f_analyze import deadline_resets, stats
from j14_a2_q1f_capture import command


def test_reset_phase_estimate_is_distinct_from_executed_cycle_deficit():
    expected = [0, 20_000_000, 79_425_255, 173_118_363, 295_840_254]
    result = deadline_resets([dict(cycle_seq=i, expected_ns=n) for i, n in enumerate(expected)])
    assert len(result) == 3
    assert [r['skipped_slots_estimate'] for r in result] == [2, 4, 5]
    assert sum(r['phase_shift_ns'] for r in result) == 215_840_254
    assert deadline_resets([dict(cycle_seq=i, expected_ns=i*20_000_000) for i in range(10)]) == []


def test_full_distribution_and_missing_diagnostics_are_not_zero():
    assert stats([]) is None
    result = stats([0, 1, 2, 3, 100])
    assert result == dict(n=5, P50=2, P95=80.6, P99=96.12, P99_9=99.612, Max=100)


def test_capture_uses_normal_default_architecture_with_qualification(tmp_path):
    args = command(tmp_path, 30)
    assert args[:4] == ['ros2', 'launch', 'pus6_bringup', 'pus6_system.launch.py']
    assert 'driver_telemetry_mode:=qualification' in args
    assert 'servo_run_duration_s:=30.0' in args
    assert not any('PUS6_J14_' in arg or 'driver_architecture' in arg for arg in args)
