from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
for path in (ROOT / "scripts", ROOT / "src" / "pus6_kinematics"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from j6_geometry_common import default_config_path  # noqa: E402
from run_j6_phase6_geometry_measurement import (  # noqa: E402
    _write_report as write_measurement_report,
    record_measurements,
)
from run_j6_phase6_geometry_validation import (  # noqa: E402
    _write_report as write_validation_report,
    validate_geometry_candidate,
)
from run_j6_phase6_ik_fk_regression import (  # noqa: E402
    _write_report as write_regression_report,
    run_ik_fk_regression,
)
from pus6_kinematics.geometry_config import load_geometry_config  # noqa: E402


def _measurements():
    return {
        "base_radius_m": [0.0400, 0.040001],
        "platform_radius_m": [0.0275, 0.027501],
        "strut_lengths_m": [0.084000, 0.084001, 0.083999, 0.084000, 0.084001, 0.084000],
        "lambda_rad": [0.5235987756, 0.5235987757],
        "mu_rad": [1.0471975512, 1.0471975513],
        "nominal_height_m": [0.13261226796, 0.13261226797],
    }


def test_measurement_record_normalizes_samples_and_passes():
    report = record_measurements(_measurements(), operator="tester", instrument="CMM")
    assert report["pass"] is True
    assert report["phase"] == "J6_PHASE6"
    assert report["measurements"]["strut_lengths_m"]["count"] == 6
    assert report["hardware_write_count"] == 0
    assert report["hardware_motion_executed"] is False


def test_measurement_record_rejects_large_strut_spread():
    measurements = _measurements()
    measurements["strut_lengths_m"][-1] = 0.085
    report = record_measurements(measurements)
    assert report["pass"] is False
    assert report["pass_conditions"]["strut_repeatability_within_limit"] is False


def test_validation_creates_candidate_and_preserves_production(tmp_path):
    base = default_config_path()
    before = Path(base).read_bytes()
    measurement = record_measurements(_measurements())
    candidate = tmp_path / "candidate_geometry.yaml"
    report = validate_geometry_candidate(
        measurement,
        base_config_path=base,
        candidate_yaml_path=candidate,
        measurement_source="fixture.json",
    )
    assert report["pass"] is True, json.dumps(report, indent=2)
    assert candidate.is_file()
    assert Path(base).read_bytes() == before
    loaded = load_geometry_config(candidate)
    assert loaded.status == "candidate_phase6_validation"
    assert report["candidate_config_path"] == str(candidate)


def test_validation_rejects_production_overwrite(tmp_path):
    measurement = record_measurements(_measurements())
    base = default_config_path()
    report = validate_geometry_candidate(
        measurement,
        base_config_path=base,
        candidate_yaml_path=base,
    )
    assert report["pass"] is False
    assert any("production geometry YAML" in item for item in report["failure_reasons"])
    assert not (tmp_path / "unused.yaml").exists()


def test_validation_rejects_failed_measurement(tmp_path):
    measurement = record_measurements(_measurements(), max_strut_spread_m=0.0)
    report = validate_geometry_candidate(
        measurement,
        base_config_path=default_config_path(),
        candidate_yaml_path=tmp_path / "candidate.yaml",
    )
    assert report["pass"] is False
    assert "measurement report is not PASS" in report["failure_reasons"]


def test_candidate_passes_ik_fk_regression(tmp_path):
    measurement = record_measurements(_measurements())
    candidate = tmp_path / "candidate_geometry.yaml"
    validation = validate_geometry_candidate(
        measurement,
        base_config_path=default_config_path(),
        candidate_yaml_path=candidate,
    )
    assert validation["pass"] is True
    regression = run_ik_fk_regression(config_path=candidate, sample_count=10, seed=11)
    assert regression["pass"] is True, regression
    assert regression["sample_count_converged"] == 10


def test_ik_fk_regression_runs_deterministically_on_production_config():
    report = run_ik_fk_regression(config_path=default_config_path(), sample_count=1000, seed=7)
    assert report["pass"] is True, report
    assert report["sample_count_converged"] == 1000
    assert report["hardware_write_count"] == 0


def test_ik_fk_regression_rejects_invalid_sample_count():
    with pytest.raises(ValueError):
        run_ik_fk_regression(config_path=default_config_path(), sample_count=0)


def test_reports_refuse_overwrite(tmp_path):
    for writer in (write_measurement_report, write_validation_report, write_regression_report):
        path = tmp_path / f"{writer.__module__}.json"
        writer({"pass": True}, path)
        with pytest.raises(FileExistsError):
            writer({"pass": True}, path)
        assert json.loads(path.read_text(encoding="utf-8"))["pass"] is True
