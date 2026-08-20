import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[3] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_j5_c2_hw_acceptance import (  # noqa: E402
    C2HwAcceptanceRunner,
    _run_frames,
    _synthetic_frames,
    _write_reports,
)


def test_synthetic_acceptance_records_wait_and_fk(tmp_path):
    runner = C2HwAcceptanceRunner()
    frames = _synthetic_frames(runner, duration_s=1.0, feedback_hz=30.0, inject_jump=False)
    _run_frames(runner, frames, query_rate_hz=200.0, fixed_lag_ms=50.0)
    report = runner.report("synthetic_mock")
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["summary"]["result_counts"]["WAIT"] > 0
    assert report["summary"]["result_counts"]["FK_OK"] > 0
    assert report["summary"]["uart"]["gap_count"] == 0
    json_path, csv_path = _write_reports(report, tmp_path / "c2_hw")
    assert json_path.is_file()
    assert csv_path.is_file()


def test_synthetic_acceptance_records_branch_jump_drop():
    runner = C2HwAcceptanceRunner()
    frames = _synthetic_frames(runner, duration_s=1.0, feedback_hz=30.0, inject_jump=True)
    _run_frames(runner, frames, query_rate_hz=200.0, fixed_lag_ms=50.0)
    summary = runner.report("synthetic_mock")["summary"]
    assert summary["branch_jump_count"] > 0
    assert summary["result_counts"]["DROP"] > 0
