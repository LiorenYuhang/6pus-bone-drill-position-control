from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from run_j6_phase8_imu_preparation import (  # noqa: E402
    build_phase8_imu_report,
    record_imu_measurements,
    validate_imu_metadata,
    validate_imu_stream,
    write_phase8_report,
)


BASE_FRAME = "pus6_base_imu_link"
PLATFORM_FRAME = "pus6_platform_imu_link"


def _sample(frame_id: str, index: int, *, topic: str | None = None) -> dict:
    sample = {
        "header": {
            "frame_id": frame_id,
            "stamp": {"sec": 100 + index, "nanosec": 0},
        },
        "tid": index,
        "has_sample_timestamp": True,
        "sample_timestamp_us": 500000 + index * 5000,
        "has_data_ready_timestamp": True,
        "data_ready_timestamp_us": 500100 + index * 5000,
        "host_monotonic_arrival_time_ns": 1_000_000_000_000 + index * 5_000_000,
    }
    if topic is not None:
        sample["topic"] = topic
    return sample


def _measurement(*, same_board: bool = False, final: bool = False) -> dict:
    return {
        "session_id": "mock-phase8",
        "installation": {
            "final_installation_confirmed": final,
            "same_board_fixture": same_board,
            "base_frame_id": BASE_FRAME,
            "platform_frame_id": PLATFORM_FRAME,
        },
        "lever_arms": {
            "base": {
                "reference_frame": "base_link",
                "vector_m": [0.010, -0.002, 0.015],
                "uncertainty_m": [0.0005, 0.0005, 0.0008],
                "instrument": "mock-cmm",
                "method": "fixture measurement record",
            },
            "platform": {
                "reference_frame": "platform_link",
                "vector_m": [0.004, 0.003, 0.020],
                "uncertainty_m": 0.0008,
                "instrument": "mock-cmm",
                "method": "fixture measurement record",
            },
        },
        "extrinsics": {
            "base": {
                "parent_frame": "base_link",
                "child_frame": BASE_FRAME,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "platform": {
                "parent_frame": "platform_link",
                "child_frame": PLATFORM_FRAME,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        },
    }


def test_measurement_record_validates_lever_arms_without_calibrating():
    report = record_imu_measurements(_measurement())

    assert report["pass"] is True, json.dumps(report, indent=2)
    assert report["hardware_mode"] == "offline"
    assert report["hardware_motion_executed"] is False
    assert report["hardware_write_count"] == 0
    assert report["phase8_acceptance_passed"] is False
    assert report["calibration_performed"] is False
    assert report["metrics"]["lever_arms"]["platform"]["uncertainty_m"] == [0.0008] * 3


def test_same_board_fixture_is_rejected_as_final_evidence():
    report = record_imu_measurements(_measurement(same_board=True))

    assert report["pass"] is False
    assert report["pass_conditions"]["same_board_fixture_excluded"] is False
    assert any("same-board fixture" in reason for reason in report["failure_reasons"])


def test_timed_imu_metadata_validates_frame_and_timestamp_contract():
    base = [_sample(BASE_FRAME, index, topic="/pus6/imu_base/timed") for index in range(3)]
    platform = [_sample(PLATFORM_FRAME, index, topic="/pus6/imu_platform/timed") for index in range(3)]
    report = validate_imu_metadata(base, platform)

    assert report["pass"] is True, json.dumps(report, indent=2)
    assert report["streams"]["base"]["pass_conditions"]["sample_timestamp_metadata_present"] is True
    assert report["streams"]["platform"]["metrics"]["tid_gap_count"] == 0


def test_bad_frame_and_nonmonotonic_timestamp_fail_closed():
    samples = [
        _sample(BASE_FRAME, 0),
        _sample("wrong_frame", 2),
        _sample(BASE_FRAME, 1),
    ]
    samples[2]["sample_timestamp_us"] = samples[0]["sample_timestamp_us"] - 1
    report = validate_imu_stream(samples, stream="base")

    assert report["pass"] is False
    assert report["pass_conditions"]["expected_frame_id_matches"] is False
    assert report["pass_conditions"]["timestamps_strictly_ordered"] is False


def test_phase8_report_is_hold_until_future_signed_phase7_and_installation():
    base = [_sample(BASE_FRAME, index) for index in range(3)]
    platform = [_sample(PLATFORM_FRAME, index) for index in range(3)]
    report = build_phase8_imu_report(
        measurements=_measurement(),
        base_samples=base,
        platform_samples=platform,
    )

    assert report["pass"] is True, json.dumps(report, indent=2)
    assert report["outcome"] == "HOLD"
    assert report["phase8_acceptance_passed"] is False
    assert report["commissioning_gates"]["phase7_signed_hardware_acceptance"] is False
    assert report["eskf_enabled"] is False
    assert report["fusion_enabled"] is False
    assert report["control_loop_enabled"] is False


def test_report_writer_is_read_only_to_inputs_and_refuses_overwrite(tmp_path):
    report = record_imu_measurements(_measurement())
    input_record = tmp_path / "measurement.json"
    input_record.write_text(json.dumps(_measurement(), indent=2), encoding="utf-8")
    before = input_record.read_bytes()
    output = tmp_path / "phase8.json"

    written = write_phase8_report(report, output)

    assert written == output
    assert json.loads(output.read_text(encoding="utf-8"))["tool"] == "j6_phase8_imu_measurement_record"
    assert input_record.read_bytes() == before
    with pytest.raises(FileExistsError):
        write_phase8_report(report, output)
