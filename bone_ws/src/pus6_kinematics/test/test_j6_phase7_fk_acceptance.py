from __future__ import annotations

from dataclasses import asdict
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_kinematics"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from run_j5_c2_hw_acceptance import C2HwAcceptanceRunner, _synthetic_frames  # noqa: E402
from run_j6_phase7_fk_acceptance import (  # noqa: E402
    run_phase7_fk_acceptance,
    write_phase7_reports,
)


def test_synthetic_phase7_pass_is_numerical_only():
    report = run_phase7_fk_acceptance(duration_s=1.0, feedback_hz=30.0)

    assert report["phase"] == "J6_PHASE7"
    assert report["hardware_mode"] == "mock"
    assert report["pass"] is True, json.dumps(report, indent=2)
    assert report["numerical_fk_validation_passed"] is True
    assert report["hardware_acceptance_passed"] is False
    assert report["physical_accuracy_claimed"] is False
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["pass_conditions"]["fixed_lag_sync_observed"] is True


def test_replay_jsonl_is_read_only_and_provenance_is_preserved(tmp_path):
    runner = C2HwAcceptanceRunner()
    frames = _synthetic_frames(runner, duration_s=1.0, feedback_hz=30.0, inject_jump=False)
    replay = tmp_path / "feedback.jsonl"
    replay.write_text(
        "\n".join(json.dumps(asdict(frame)) for frame in frames) + "\n",
        encoding="utf-8",
    )
    before = replay.read_bytes()

    report = run_phase7_fk_acceptance(input_path=replay)

    assert report["hardware_mode"] == "offline"
    assert report["pass"] is True, json.dumps(report, indent=2)
    assert report["source_report_path"] == str(replay)
    assert report["source_reports"] == [str(replay)]
    assert report["evidence"]["source_sha256"]
    assert replay.read_bytes() == before
    assert report["hardware_acceptance_passed"] is False


def test_branch_jump_stops_numerical_acceptance_without_physical_claim():
    report = run_phase7_fk_acceptance(
        duration_s=1.0,
        feedback_hz=30.0,
        inject_branch_jump=True,
    )

    assert report["pass"] is False
    assert report["outcome"] == "STOP"
    assert report["pass_conditions"]["branch_continuity"] is False
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["hardware_acceptance_passed"] is False


def test_invalid_or_short_replay_returns_failure_report(tmp_path):
    invalid = tmp_path / "invalid.json"
    invalid.write_text("{\"positions_meters\": [0.0]}\n", encoding="utf-8")
    report = run_phase7_fk_acceptance(input_path=invalid)

    assert report["pass"] is False
    assert report["outcome"] == "FAIL"
    assert report["hardware_write_count"] == 0
    assert any("unable to load replay" in reason for reason in report["failure_reasons"])


def test_external_ground_truth_does_not_promote_mock_to_hardware_pass():
    report = run_phase7_fk_acceptance(
        duration_s=1.0,
        feedback_hz=30.0,
        external_ground_truth={"instrument": "fixture", "record": "future"},
    )

    assert report["pass"] is True
    assert report["evidence"]["external_ground_truth"]["instrument"] == "fixture"
    assert report["hardware_acceptance_passed"] is False
    assert report["hardware_acceptance_status"] == "NOT_EVALUATED"


def test_report_writer_creates_json_csv_and_refuses_overwrite(tmp_path):
    report = run_phase7_fk_acceptance(duration_s=0.5, feedback_hz=30.0)
    output = tmp_path / "phase7.json"
    json_path, csv_path = write_phase7_reports(report, output)

    assert json_path == output
    assert json_path.is_file()
    assert csv_path.is_file()
    assert json.loads(json_path.read_text(encoding="utf-8"))["phase"] == "J6_PHASE7"
    with pytest.raises(FileExistsError):
        write_phase7_reports(report, output)
