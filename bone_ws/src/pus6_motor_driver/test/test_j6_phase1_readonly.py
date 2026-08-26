from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_ROOT = WORKSPACE_ROOT / "scripts"
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from run_j6_phase1_readonly import run_phase1  # noqa: E402


def _source_report(*, total_writes: int = 0) -> dict:
    return {
        "verdict": "J2-A COMPLETE / PASS" if total_writes == 0 else "J2-A STOP / NOT PASSED",
        "runtime_duration_s": 30.0,
        "target_duration_s": 30.0,
        "hardware_mode": "read_only",
        "write_audit": {
            "startup_writes": 0,
            "runtime_writes": total_writes,
            "shutdown_writes": 0,
            "total_hardware_writes": total_writes,
        },
        "pass_conditions": {"duration_met": total_writes == 0, "zero_total_writes": total_writes == 0},
        "per_motor_stats": {
            f"motor_{mid}": {"fault_set": ["0x00"]} for mid in range(1, 7)
        },
    }


def _delegate_factory(log_dir: Path, *, total_writes: int = 0):
    def delegate(*, duration_sec: float, port: str, baudrate: int) -> int:
        assert duration_sec == 30.0
        assert port == "mock-port"
        assert baudrate == 921600
        log_dir.mkdir(parents=True, exist_ok=True)
        source = log_dir / "j2a_readonly_test.json"
        source.write_text(json.dumps(_source_report(total_writes=total_writes)), encoding="utf-8")
        return 0 if total_writes == 0 else 1

    return delegate


def test_phase1_wrapper_normalizes_successful_source_report(tmp_path):
    source_dir = tmp_path / "source"
    output_prefix = tmp_path / "reports" / "j6_phase1"

    report = run_phase1(
        duration_s=30.0,
        port="mock-port",
        source_log_dir=source_dir,
        output_prefix=output_prefix,
        delegate=_delegate_factory(source_dir),
    )

    assert report["pass"] is True
    assert report["hardware_write_count"] == 0
    assert report["hardware_motion_executed"] is False
    assert output_prefix.with_suffix(".json").exists()
    assert output_prefix.with_suffix(".log").exists()


def test_phase1_wrapper_fails_on_nonzero_source_writes(tmp_path):
    source_dir = tmp_path / "source"

    report = run_phase1(
        duration_s=30.0,
        port="mock-port",
        source_log_dir=source_dir,
        output_prefix=tmp_path / "j6_phase1_fail",
        delegate=_delegate_factory(source_dir, total_writes=1),
    )

    assert report["pass"] is False
    assert report["hardware_write_count"] == 1
    assert "total_writes_zero" in report["failure_reasons"]


def test_phase1_wrapper_requires_at_least_thirty_seconds(tmp_path):
    with pytest.raises(ValueError):
        run_phase1(
            duration_s=29.9,
            source_log_dir=tmp_path / "source",
            output_prefix=tmp_path / "report",
            delegate=lambda **_: 0,
        )
