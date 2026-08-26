from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from generate_j6_commissioning_summary import collect_reports, generate_markdown_summary  # noqa: E402


def _report(*, phase: str = "J6_PHASE1", passed: bool = True, tool: str = "fixture") -> dict:
    return {
        "schema_version": 1,
        "tool": tool,
        "phase": phase,
        "timestamp": "2026-08-22T00:00:00+00:00",
        "hardware_mode": "offline",
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "pass": passed,
        "outcome": "PASS" if passed else "FAIL",
        "pass_conditions": {"fixture": passed},
        "failure_reasons": [] if passed else ["fixture failure"],
    }


def test_collect_and_generate_summary_without_mutating_source(tmp_path):
    source = tmp_path / "j6_phase1_fixture.json"
    source.write_text(json.dumps(_report()), encoding="utf-8")
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    summary = generate_markdown_summary(collect_reports(input_dir=tmp_path), source_description="fixtures")
    assert "# J6 Commissioning Summary" in summary
    assert "J6_PHASE1" in summary and "PASS" in summary
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before


def test_legacy_connection_checker_phase_is_explicitly_inferred(tmp_path):
    source = tmp_path / "j6_connection_fixture.json"
    payload = _report(tool="j6_hardware_connection_checker")
    payload.pop("phase")
    source.write_text(json.dumps(payload), encoding="utf-8")
    report = collect_reports(input_dir=tmp_path)[0]
    assert report["_status"] == "PASS"
    assert report["phase"] == "J6_PHASE1"
    assert any("phase inferred" in warning for warning in report["_warnings"])


def test_invalid_and_missing_phase_are_not_pass(tmp_path):
    (tmp_path / "j6_invalid.json").write_text(json.dumps({"tool": "bad", "pass": True}), encoding="utf-8")
    (tmp_path / "j6_phase3.json").write_text(json.dumps(_report(phase="J6_PHASE3", passed=False)), encoding="utf-8")
    reports = collect_reports(input_dir=tmp_path)
    summary = generate_markdown_summary(reports, source_description="fixtures")
    assert any(report["_status"] == "INVALID" for report in reports)
    assert any(report["_status"] == "FAIL" for report in reports)
    assert "J6_PHASE0" in summary and "MISSING" in summary


def test_unreadable_json_is_invalid(tmp_path):
    (tmp_path / "j6_broken.json").write_text("not-json", encoding="utf-8")
    report = collect_reports(input_dir=tmp_path)[0]
    assert report["_status"] == "INVALID"
    assert "unable to read JSON" in report["_validation_errors"][0]
