#!/usr/bin/env python3
"""J6 Phase 8 IMU installation and extrinsic-preparation framework.

The tool is intentionally report-only.  It validates future ``TimedImu``
evidence, records measured lever arms/extrinsics, and describes the remaining
commissioning gates.  It never opens an IMU port, invokes calibration, starts
fusion, or changes a production configuration file.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


TOOL_NAME = "j6_phase8_imu_preparation"
MEASUREMENT_TOOL = "j6_phase8_imu_measurement_record"
VALIDATION_TOOL = "j6_phase8_imu_validation"
PHASE_NAME = "J6_PHASE8"
TID_MODULUS = 1 << 16
TIMESTAMP_MODULUS = 1 << 32

EXPECTED_STREAMS = {
    "base": {
        "frame_id": "pus6_base_imu_link",
        "topic": "/pus6/imu_base/timed",
    },
    "platform": {
        "frame_id": "pus6_platform_imu_link",
        "topic": "/pus6/imu_platform/timed",
    },
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_vector(value: Any, *, length: int, name: str) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{name} must be a sequence of {length} numbers")
    if len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def _uncertainty_vector(value: Any, *, name: str) -> list[float]:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        result = [float(value)] * 3
    else:
        result = _finite_vector(value, length=3, name=name)
    if any(item < 0.0 for item in result):
        raise ValueError(f"{name} must be non-negative")
    return result


def _mapping(value: Any, *, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _header_frame_id(sample: Mapping[str, Any]) -> str | None:
    direct = sample.get("frame_id")
    if direct is not None:
        return str(direct)
    header = sample.get("header")
    if isinstance(header, Mapping) and header.get("frame_id") is not None:
        return str(header["frame_id"])
    return None


def _header_stamp_valid(sample: Mapping[str, Any]) -> bool:
    header = sample.get("header")
    if not isinstance(header, Mapping) or "stamp" not in header:
        return True
    stamp = header["stamp"]
    if not isinstance(stamp, Mapping):
        return False
    try:
        sec = int(stamp["sec"])
        nanosec = int(stamp["nanosec"])
    except (KeyError, TypeError, ValueError):
        return False
    return sec >= 0 and 0 <= nanosec < 1_000_000_000


def _int_field(sample: Mapping[str, Any], name: str) -> int:
    value = sample[name]
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _mod_delta(current: int, previous: int, modulus: int) -> tuple[str, int]:
    delta = (current - previous) % modulus
    if delta == 0:
        return "duplicate", 0
    if delta > modulus // 2:
        return "backward", delta
    return "forward", delta


def validate_imu_stream(
    samples: Sequence[Mapping[str, Any]],
    *,
    stream: str,
    expected_frame_id: str | None = None,
    expected_topic: str | None = None,
    min_samples: int = 2,
) -> dict[str, Any]:
    """Validate offline ``h30_imu/msg/TimedImu``-shaped records.

    This checks provenance and ordering only.  It does not estimate bias,
    solve rotations, or infer physical installation accuracy.
    """
    if stream not in EXPECTED_STREAMS:
        raise ValueError(f"stream must be one of {sorted(EXPECTED_STREAMS)}")
    expected_frame_id = expected_frame_id or EXPECTED_STREAMS[stream]["frame_id"]
    expected_topic = expected_topic or EXPECTED_STREAMS[stream]["topic"]
    values = list(samples)
    failures: list[str] = []
    frame_ids: set[str] = set()
    tids: list[int] = []
    sample_timestamps: list[int] = []
    data_ready_timestamps: list[int] = []
    host_timestamps: list[int] = []
    header_stamp_valid = True
    sample_flags_valid = True
    data_ready_flags_valid = True
    host_timestamp_valid = True
    frame_valid = True
    topic_valid = True
    tid_gaps = 0
    tid_backwards = 0
    sample_duplicates = 0
    sample_backwards = 0
    data_ready_duplicates = 0
    data_ready_backwards = 0

    if len(values) < min_samples:
        failures.append(f"at least {min_samples} samples are required")

    for index, raw in enumerate(values):
        if not isinstance(raw, Mapping):
            failures.append(f"sample {index} is not an object")
            continue
        frame_id = _header_frame_id(raw)
        if frame_id is None:
            frame_valid = False
        else:
            frame_ids.add(frame_id)
            if frame_id != expected_frame_id:
                frame_valid = False
        if raw.get("topic") is not None and str(raw["topic"]) != expected_topic:
            topic_valid = False
        if not _header_stamp_valid(raw):
            header_stamp_valid = False
        try:
            if raw.get("has_sample_timestamp") is not True:
                sample_flags_valid = False
            sample_timestamps.append(_int_field(raw, "sample_timestamp_us"))
            if not 0 <= sample_timestamps[-1] < TIMESTAMP_MODULUS:
                sample_flags_valid = False

            if raw.get("has_data_ready_timestamp") is not True:
                data_ready_flags_valid = False
            data_ready_timestamps.append(_int_field(raw, "data_ready_timestamp_us"))
            if not 0 <= data_ready_timestamps[-1] < TIMESTAMP_MODULUS:
                data_ready_flags_valid = False

            host_timestamps.append(_int_field(raw, "host_monotonic_arrival_time_ns"))
            if host_timestamps[-1] <= 0:
                host_timestamp_valid = False
            tids.append(_int_field(raw, "tid"))
            if not 0 <= tids[-1] < TID_MODULUS:
                host_timestamp_valid = False
        except (KeyError, TypeError, ValueError, OverflowError):
            sample_flags_valid = False
            data_ready_flags_valid = False
            host_timestamp_valid = False

    for previous, current in zip(tids, tids[1:]):
        state, delta = _mod_delta(current, previous, TID_MODULUS)
        if state == "duplicate":
            tid_gaps += 1
        elif state == "backward":
            tid_backwards += 1
        elif delta > 1:
            tid_gaps += delta - 1

    for previous, current in zip(sample_timestamps, sample_timestamps[1:]):
        state, _ = _mod_delta(current, previous, TIMESTAMP_MODULUS)
        if state == "duplicate":
            sample_duplicates += 1
        elif state == "backward":
            sample_backwards += 1
    for previous, current in zip(data_ready_timestamps, data_ready_timestamps[1:]):
        state, _ = _mod_delta(current, previous, TIMESTAMP_MODULUS)
        if state == "duplicate":
            data_ready_duplicates += 1
        elif state == "backward":
            data_ready_backwards += 1
    host_strict = all(current > previous for previous, current in zip(host_timestamps, host_timestamps[1:]))
    timestamps_ordered = (
        sample_backwards == 0
        and sample_duplicates == 0
        and data_ready_backwards == 0
        and data_ready_duplicates == 0
        and host_strict
    )
    conditions = {
        "samples_present": len(values) >= min_samples,
        "frame_metadata_valid": frame_valid and len(frame_ids) == 1,
        "expected_frame_id_matches": frame_valid and frame_ids == {expected_frame_id},
        "topic_metadata_valid": topic_valid,
        "sample_timestamp_metadata_present": sample_flags_valid,
        "data_ready_timestamp_metadata_present": data_ready_flags_valid,
        "host_monotonic_metadata_present": host_timestamp_valid,
        "timestamps_strictly_ordered": timestamps_ordered,
        "tid_sequence_continuous": tid_gaps == 0 and tid_backwards == 0,
        "ros_header_timestamps_valid": header_stamp_valid,
    }
    failures.extend(name for name, passed in conditions.items() if not passed)
    return {
        "stream": stream,
        "expected_frame_id": expected_frame_id,
        "expected_topic": expected_topic,
        "pass": not failures,
        "pass_conditions": conditions,
        "failure_reasons": sorted(set(failures)),
        "metrics": {
            "sample_count": len(values),
            "frame_ids": sorted(frame_ids),
            "first_tid": tids[0] if tids else None,
            "last_tid": tids[-1] if tids else None,
            "tid_gap_count": tid_gaps,
            "tid_backward_count": tid_backwards,
            "sample_timestamp_duplicate_count": sample_duplicates,
            "sample_timestamp_backward_count": sample_backwards,
            "data_ready_timestamp_duplicate_count": data_ready_duplicates,
            "data_ready_timestamp_backward_count": data_ready_backwards,
            "first_sample_timestamp_us": sample_timestamps[0] if sample_timestamps else None,
            "last_sample_timestamp_us": sample_timestamps[-1] if sample_timestamps else None,
            "first_host_monotonic_ns": host_timestamps[0] if host_timestamps else None,
            "last_host_monotonic_ns": host_timestamps[-1] if host_timestamps else None,
        },
    }


def _validate_extrinsic(entry: Any, *, role: str) -> tuple[dict[str, Any] | None, str | None]:
    if entry is None:
        return None, None
    try:
        value = _mapping(entry, name=f"{role} extrinsic")
        parent = str(value["parent_frame"])
        child = str(value["child_frame"])
        if not parent or not child or parent == child:
            raise ValueError("parent_frame and child_frame must be distinct")
        if "quaternion_xyzw" in value:
            quaternion = _finite_vector(value["quaternion_xyzw"], length=4, name=f"{role} quaternion_xyzw")
            norm = math.sqrt(sum(item * item for item in quaternion))
            if abs(norm - 1.0) > 1e-3 or norm < 1e-12:
                raise ValueError(f"{role} quaternion_xyzw must be unit length")
            normalized = [item / norm for item in quaternion]
            return {
                "parent_frame": parent,
                "child_frame": child,
                "quaternion_xyzw": normalized,
            }, None
        if "rotation_matrix" in value:
            matrix = value["rotation_matrix"]
            if not isinstance(matrix, Sequence) or len(matrix) != 3:
                raise ValueError(f"{role} rotation_matrix must be 3x3")
            rows = [_finite_vector(row, length=3, name=f"{role} rotation_matrix row") for row in matrix]
            for row in rows:
                if abs(math.sqrt(sum(item * item for item in row)) - 1.0) > 1e-3:
                    raise ValueError(f"{role} rotation_matrix rows must be unit length")
            dot_products = [sum(rows[i][k] * rows[j][k] for k in range(3)) for i, j in ((0, 1), (0, 2), (1, 2))]
            if any(abs(value) > 1e-3 for value in dot_products):
                raise ValueError(f"{role} rotation_matrix rows must be orthogonal")
            determinant = (
                rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
                - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
                + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
            )
            if abs(determinant - 1.0) > 1e-3:
                raise ValueError(f"{role} rotation_matrix determinant must be +1")
            return {"parent_frame": parent, "child_frame": child, "rotation_matrix": rows}, None
        raise ValueError(f"{role} requires quaternion_xyzw or rotation_matrix")
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return None, f"{role} extrinsic invalid: {exc}"


def record_imu_measurements(measurements: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a future final-installation lever-arm/extrinsic record."""
    failures: list[str] = []
    installation: Mapping[str, Any]
    try:
        installation = _mapping(measurements.get("installation"), name="installation")
    except ValueError as exc:
        installation = {}
        failures.append(str(exc))

    base_frame = str(installation.get("base_frame_id", EXPECTED_STREAMS["base"]["frame_id"]))
    platform_frame = str(installation.get("platform_frame_id", EXPECTED_STREAMS["platform"]["frame_id"]))
    same_board = bool(installation.get("same_board_fixture", False))
    final_installation_confirmed = bool(installation.get("final_installation_confirmed", False))
    if not base_frame or not platform_frame or base_frame == platform_frame:
        failures.append("base and platform IMU frame IDs must be distinct")
    if same_board:
        failures.append("same-board fixture evidence cannot be used as final installation evidence")

    arms = measurements.get("lever_arms", {})
    try:
        arms = _mapping(arms, name="lever_arms")
    except ValueError as exc:
        arms = {}
        failures.append(str(exc))
    normalized_arms: dict[str, Any] = {}
    arm_conditions: dict[str, bool] = {}
    for role, default_frame in (("base", "base_link"), ("platform", "platform_link")):
        try:
            raw_arm = _mapping(arms.get(role), name=f"{role} lever arm")
            reference_frame = str(raw_arm.get("reference_frame", default_frame))
            vector = _finite_vector(raw_arm.get("vector_m", raw_arm.get("lever_arm_m")), length=3, name=f"{role} vector_m")
            uncertainty = _uncertainty_vector(raw_arm.get("uncertainty_m"), name=f"{role} uncertainty_m")
            instrument = raw_arm.get("instrument")
            method = raw_arm.get("method")
            normalized_arms[role] = {
                "reference_frame": reference_frame,
                "vector_m": vector,
                "uncertainty_m": uncertainty,
                "instrument": instrument,
                "method": method,
            }
            arm_conditions[f"{role}_lever_arm_record_valid"] = bool(reference_frame)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            arm_conditions[f"{role}_lever_arm_record_valid"] = False
            failures.append(f"{role} lever arm invalid: {exc}")

    extrinsics = measurements.get("extrinsics", {})
    try:
        extrinsics = _mapping(extrinsics, name="extrinsics")
    except ValueError as exc:
        extrinsics = {}
        failures.append(str(exc))
    normalized_extrinsics: dict[str, Any] = {}
    extrinsic_errors: list[str] = []
    for role in ("base", "platform"):
        normalized, error = _validate_extrinsic(extrinsics.get(role), role=role)
        if normalized is not None:
            normalized_extrinsics[role] = normalized
        if error:
            extrinsic_errors.append(error)
    failures.extend(extrinsic_errors)
    conditions = {
        "installation_metadata_valid": bool(base_frame and platform_frame and base_frame != platform_frame),
        "same_board_fixture_excluded": not same_board,
        **arm_conditions,
        "no_production_geometry_write": True,
    }
    failures.extend(name for name, passed in conditions.items() if not passed)
    measurement_pass = not failures
    open_gates = []
    if not final_installation_confirmed:
        open_gates.append("final installation confirmation is pending")
    if len(normalized_extrinsics) != 2:
        open_gates.append("final extrinsic rotation records are pending")
    return {
        "schema_version": 1,
        "tool": MEASUREMENT_TOOL,
        "phase": PHASE_NAME,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "session_id": measurements.get("session_id"),
        "hardware_mode": "offline",
        "port": None,
        "baudrate": None,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "pass": measurement_pass,
        "outcome": "PASS" if measurement_pass else "FAIL",
        "pass_conditions": conditions,
        "failure_reasons": sorted(set(failures)),
        "evidence": {
            "record_type": "operator_supplied_final_installation_measurement",
            "production_geometry_modified": False,
        },
        "observations": {
            "final_installation_confirmed": final_installation_confirmed,
            "extrinsics_recorded": len(normalized_extrinsics) == 2,
            "open_gates": open_gates,
        },
        "metrics": {
            "base_frame_id": base_frame,
            "platform_frame_id": platform_frame,
            "lever_arms": normalized_arms,
            "extrinsics": normalized_extrinsics,
        },
        "source_report_path": None,
        "source_reports": [],
        "phase8_acceptance_passed": False,
        "calibration_performed": False,
        "fusion_enabled": False,
        "control_loop_enabled": False,
    }


def validate_imu_metadata(
    base_samples: Sequence[Mapping[str, Any]],
    platform_samples: Sequence[Mapping[str, Any]],
    *,
    base_frame_id: str | None = None,
    platform_frame_id: str | None = None,
    base_topic: str | None = None,
    platform_topic: str | None = None,
    min_samples: int = 2,
) -> dict[str, Any]:
    """Validate both future final-installation TimedImu streams."""
    base = validate_imu_stream(
        base_samples,
        stream="base",
        expected_frame_id=base_frame_id,
        expected_topic=base_topic,
        min_samples=min_samples,
    )
    platform = validate_imu_stream(
        platform_samples,
        stream="platform",
        expected_frame_id=platform_frame_id,
        expected_topic=platform_topic,
        min_samples=min_samples,
    )
    frames = {base["expected_frame_id"], platform["expected_frame_id"]}
    conditions = {
        "base_stream_valid": bool(base["pass"]),
        "platform_stream_valid": bool(platform["pass"]),
        "distinct_stream_frames": len(frames) == 2,
        "distinct_stream_topics": base["expected_topic"] != platform["expected_topic"],
    }
    failures = list(base["failure_reasons"]) + list(platform["failure_reasons"])
    failures.extend(name for name, passed in conditions.items() if not passed)
    return {
        "tool": VALIDATION_TOOL,
        "phase": PHASE_NAME,
        "pass": not failures,
        "pass_conditions": conditions,
        "failure_reasons": sorted(set(failures)),
        "streams": {"base": base, "platform": platform},
        "metrics": {
            "base_sample_count": base["metrics"]["sample_count"],
            "platform_sample_count": platform["metrics"]["sample_count"],
            "base_tid_gap_count": base["metrics"]["tid_gap_count"],
            "platform_tid_gap_count": platform["metrics"]["tid_gap_count"],
        },
    }


def _as_measurement_report(value: Mapping[str, Any] | None) -> dict[str, Any]:
    if value is None:
        return record_imu_measurements({})
    if value.get("tool") == MEASUREMENT_TOOL and "pass_conditions" in value:
        return dict(value)
    return record_imu_measurements(value)


def build_phase8_imu_report(
    *,
    measurements: Mapping[str, Any] | None,
    base_samples: Sequence[Mapping[str, Any]],
    platform_samples: Sequence[Mapping[str, Any]],
    phase7_report: Mapping[str, Any] | None = None,
    source_reports: Sequence[str] = (),
    min_samples: int = 2,
) -> dict[str, Any]:
    """Build a report-only Phase 8 preparation/validation report."""
    measurement = _as_measurement_report(measurements)
    install = measurements.get("installation", {}) if isinstance(measurements, Mapping) else {}
    install = install if isinstance(install, Mapping) else {}
    metadata = validate_imu_metadata(
        base_samples,
        platform_samples,
        base_frame_id=install.get("base_frame_id"),
        platform_frame_id=install.get("platform_frame_id"),
        min_samples=min_samples,
    )
    structural_conditions = {
        "measurement_record_valid": bool(measurement.get("pass")),
        "imu_timestamp_and_frame_metadata_valid": bool(metadata["pass"]),
        "no_hardware_execution": True,
        "calibration_not_performed": True,
        "fusion_not_enabled": True,
        "control_loop_not_enabled": True,
        "production_geometry_not_modified": True,
    }
    failures = list(measurement.get("failure_reasons", [])) + list(metadata.get("failure_reasons", []))
    failures.extend(name for name, passed in structural_conditions.items() if not passed)
    validation_pass = not failures

    phase7_signed = bool(
        isinstance(phase7_report, Mapping)
        and phase7_report.get("pass") is True
        and phase7_report.get("hardware_acceptance_passed") is True
    )
    final_installation = bool(install.get("final_installation_confirmed", False))
    same_board = bool(install.get("same_board_fixture", False))
    extrinsics_recorded = len(measurement.get("metrics", {}).get("extrinsics", {})) == 2
    commissioning_gates = {
        "phase7_signed_hardware_acceptance": phase7_signed,
        "final_installation_confirmed": final_installation,
        "same_board_fixture_excluded": not same_board,
        "lever_arm_measurements_recorded": all(
            measurement.get("pass_conditions", {}).get(f"{role}_lever_arm_record_valid", False)
            for role in ("base", "platform")
        ),
        "extrinsic_rotation_records_present": extrinsics_recorded,
        "timed_imu_metadata_valid": bool(metadata["pass"]),
        "future_calibration_authorized_only_after_review": True,
    }
    open_gates = [name for name, passed in commissioning_gates.items() if not passed]
    if validation_pass:
        failure_reasons = []
        outcome = "PASS" if not open_gates else "HOLD"
    else:
        failure_reasons = sorted(set(failures))
        outcome = "FAIL"

    return {
        "schema_version": 1,
        "tool": TOOL_NAME,
        "phase": PHASE_NAME,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "session_id": measurements.get("session_id") if isinstance(measurements, Mapping) else None,
        "hardware_mode": "offline",
        "port": None,
        "baudrate": None,
        "hardware_motion_executed": False,
        "hardware_write_count": 0,
        "pass": validation_pass,
        "outcome": outcome,
        "pass_conditions": structural_conditions,
        "failure_reasons": failure_reasons,
        "evidence": {
            "source_reports": list(source_reports),
            "phase7_report_supplied": phase7_report is not None,
            "phase7_hardware_acceptance_claim_used": phase7_signed,
            "measurement_tool": MEASUREMENT_TOOL,
            "validation_tool": VALIDATION_TOOL,
            "production_geometry_modified": False,
        },
        "observations": {
            "acceptance_scope": "phase8_imu_installation_and_extrinsic_preparation_only",
            "physical_calibration_status": "not performed",
            "open_commissioning_gates": open_gates,
            "future_workflow": [
                "confirm final rigid Base/Platform IMU installation",
                "record signed lever-arm measurements and uncertainty",
                "collect final TimedImu streams with valid frame/timestamp metadata",
                "run reviewed B2 extrinsic calibration and external validation",
            ],
        },
        "metrics": {
            "measurement": measurement.get("metrics", {}),
            "imu_metadata": metadata.get("metrics", {}),
            "streams": metadata.get("streams", {}),
        },
        "source_report_path": source_reports[0] if source_reports else None,
        "source_reports": list(source_reports),
        "commissioning_gates": commissioning_gates,
        "phase8_acceptance_passed": False,
        "calibration_performed": False,
        "eskf_enabled": False,
        "fusion_enabled": False,
        "control_loop_enabled": False,
    }


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_sample_file(path: Path) -> list[Mapping[str, Any]]:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return []
    if text.startswith("["):
        value = json.loads(text)
        if not isinstance(value, list):
            raise ValueError("sample replay array is required")
        return value
    values = [json.loads(line) for line in text.splitlines() if line.strip()]
    if not all(isinstance(value, Mapping) for value in values):
        raise ValueError("JSONL samples must be objects")
    return values


def write_phase8_report(report: Mapping[str, Any], output: str | Path) -> Path:
    path = Path(output).expanduser()
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing report: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(report), indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Report-only J6 Phase 8 IMU preparation validator")
    parser.add_argument("--input", type=Path, required=True, help="JSON preparation mapping")
    parser.add_argument("--base-replay", type=Path, help="TimedImu JSON array or JSONL for Base IMU")
    parser.add_argument("--platform-replay", type=Path, help="TimedImu JSON array or JSONL for Platform IMU")
    parser.add_argument("--phase7-report", type=Path, help="Existing signed Phase 7 report; read only")
    parser.add_argument("--output", type=Path, default=Path("/tmp/j6_phase8_imu_preparation.json"))
    args = parser.parse_args(argv)
    payload = _load_json(args.input)
    if not isinstance(payload, Mapping):
        raise SystemExit("input must be a JSON object")
    base_samples = payload.get("base_samples", [])
    platform_samples = payload.get("platform_samples", [])
    source_reports = [str(args.input)]
    if args.base_replay:
        base_samples = _load_sample_file(args.base_replay)
        source_reports.append(str(args.base_replay))
    if args.platform_replay:
        platform_samples = _load_sample_file(args.platform_replay)
        source_reports.append(str(args.platform_replay))
    phase7 = _load_json(args.phase7_report) if args.phase7_report else payload.get("phase7_report")
    if args.phase7_report:
        source_reports.append(str(args.phase7_report))
    measurements = payload.get("measurements", payload.get("measurement_record", payload))
    report = build_phase8_imu_report(
        measurements=measurements if isinstance(measurements, Mapping) else None,
        base_samples=base_samples if isinstance(base_samples, Sequence) else [],
        platform_samples=platform_samples if isinstance(platform_samples, Sequence) else [],
        phase7_report=phase7 if isinstance(phase7, Mapping) else None,
        source_reports=source_reports,
    )
    output = write_phase8_report(report, args.output)
    print(json.dumps({"phase": report["phase"], "pass": report["pass"], "outcome": report["outcome"], "report": str(output)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
