import sys
from pathlib import Path
import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_kinematics.state_synchronizer import (
    FixedLagJointSynchronizer,
    SyncConfigurationResult,
    SyncStatus,
)

def test_state_synchronizer_empty_and_insufficient():
    sync = FixedLagJointSynchronizer()
    res = sync.query_sync_configuration(1000000)
    assert res.status == SyncStatus.EMPTY_BUFFER
    assert res.positions_meters == []

    # Push only 1 frame
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[100, 110, 120, 130, 140, 150],
        positions_meters=[0.05]*6,
        raw_steps=[1000]*6,
    )
    res2 = sync.query_sync_configuration(125)
    assert res2.status == SyncStatus.EMPTY_BUFFER

def test_state_synchronizer_bracket_and_interpolation():
    sync = FixedLagJointSynchronizer()

    # Frame 0: timestamps [1000, 1010, 1020, 1030, 1040, 1050], pos [0.050, 0.050, 0.050, 0.050, 0.050, 0.050], steps 1000
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[1000, 1010, 1020, 1030, 1040, 1050],
        positions_meters=[0.050, 0.050, 0.050, 0.050, 0.050, 0.050],
        raw_steps=[1000, 1000, 1000, 1000, 1000, 1000],
    )

    # Frame 1: timestamps [2000, 2010, 2020, 2030, 2040, 2050], pos [0.060, 0.060, 0.060, 0.060, 0.060, 0.060], steps 1400
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[2000, 2010, 2020, 2030, 2040, 2050],
        positions_meters=[0.060, 0.060, 0.060, 0.060, 0.060, 0.060],
        raw_steps=[1400, 1400, 1400, 1400, 1400, 1400],
    )

    # Query at t = 1500 (interior: bracketed by [1000..2000] for M1, [1050..2050] for M6)
    res = sync.query_sync_configuration(1500)
    assert res.status == SyncStatus.SYNCED
    assert len(res.positions_meters) == 6
    assert len(res.raw_steps) == 6

    # For Motor 1: t_left=1000, t_right=2000 -> alpha = (1500-1000)/1000 = 0.5
    assert abs(res.per_axis_interp_weights[0] - 0.5) < 1e-6
    assert abs(res.positions_meters[0] - 0.055) < 1e-6
    assert abs(res.raw_steps[0] - 1200.0) < 1e-6

    # For Motor 6: t_left=1050, t_right=2050 -> alpha = (1500-1050)/1000 = 0.45
    assert abs(res.per_axis_interp_weights[5] - 0.45) < 1e-6
    assert abs(res.positions_meters[5] - 0.0545) < 1e-6
    assert abs(res.raw_steps[5] - 1180.0) < 1e-6

def test_state_synchronizer_strict_zero_extrapolation():
    sync = FixedLagJointSynchronizer()
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[1000, 1010, 1020, 1030, 1040, 1050],
        positions_meters=[0.050]*6,
        raw_steps=[1000]*6,
    )
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[2000, 2010, 2020, 2030, 2040, 2050],
        positions_meters=[0.060]*6,
        raw_steps=[1400]*6,
    )

    # Query before oldest available sample: t = 990 (< M1 oldest 1000)
    res_early = sync.query_sync_configuration(990)
    assert res_early.status == SyncStatus.WAIT_TOO_EARLY
    assert res_early.positions_meters == []

    # Query after newest available sample: t = 2060 (> M6 newest 2050)
    res_future = sync.query_sync_configuration(2060)
    assert res_future.status == SyncStatus.WAIT_FUTURE_SAMPLE
    assert res_future.positions_meters == []

    # Query at partial boundary: t = 2005 (M1 is 2000 so t > M1 newest!) -> Must WAIT_FUTURE_SAMPLE
    res_partial_future = sync.query_sync_configuration(2005)
    assert res_partial_future.status == SyncStatus.WAIT_FUTURE_SAMPLE
    assert res_partial_future.positions_meters == []

def test_state_synchronizer_monotonicity_enforcement():
    sync = FixedLagJointSynchronizer()
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[1000]*6,
        positions_meters=[0.050]*6,
        raw_steps=[1000]*6,
    )
    # Duplicate push
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[1000]*6,
        positions_meters=[0.050]*6,
        raw_steps=[1000]*6,
    )
    # Length of buffer should still be 1
    assert len(sync._buffers[0]) == 1

    # Out of order push
    sync.push_raw_snapshot(
        per_axis_timestamps_ns=[900]*6,
        positions_meters=[0.050]*6,
        raw_steps=[1000]*6,
    )
    assert len(sync._buffers[0]) == 1
