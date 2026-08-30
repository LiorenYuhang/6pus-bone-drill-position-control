#!/usr/bin/env python3
"""Offline J9-E1A platform IMU orientation extrinsic calibrator.

Estimates only R_P_IP, the rotation from pus6_platform_imu_link coordinates
into platform_link coordinates. Lever arm, base IMU extrinsics, fusion, and
control are intentionally out of scope.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.spatial.transform import Rotation


UINT32_US = 4294967296.0


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def unwrap_u32_us(raw_values: Iterable[int]) -> tuple[np.ndarray, dict[str, int]]:
    out: list[float] = []
    wraps = 0
    resets = 0
    nonmonotonic = 0
    have = False
    last = 0
    for value in raw_values:
        raw = int(value) & 0xFFFFFFFF
        if have and raw < last:
            if (last - raw) > 0x80000000:
                wraps += 1
            else:
                resets += 1
                wraps = 0
        unwrapped = raw + wraps * UINT32_US
        if out and unwrapped <= out[-1]:
            nonmonotonic += 1
        out.append(float(unwrapped))
        last = raw
        have = True
    return np.asarray(out, dtype=float), {"wraps": wraps, "resets": resets, "nonmonotonic": nonmonotonic}


def fit_affine_device_to_host(device_us: np.ndarray, host_ns: np.ndarray) -> dict[str, float]:
    if len(device_us) != len(host_ns) or len(device_us) < 3:
        raise ValueError("at least three matched device/host timestamps are required")
    x = np.asarray(device_us, dtype=float)
    y = np.asarray(host_ns, dtype=float) * 1e-3
    good = np.isfinite(x) & np.isfinite(y)
    x = x[good]
    y = y[good]
    if len(x) < 3:
        raise ValueError("at least three finite matched timestamps are required")
    x0 = float(np.mean(x))
    y0 = float(np.mean(y))
    denom = float(np.sum((x - x0) ** 2))
    if denom <= 0.0:
        raise ValueError("device timestamps do not span time")
    a = float(np.sum((x - x0) * (y - y0)) / denom)
    b_us = float(y0 - a * x0)
    residual = y - (a * x + b_us)
    return {
        "a": a,
        "b_us": b_us,
        "ppm_equivalent": (a - 1.0) * 1e6,
        "residual_rms_us": float(np.sqrt(np.mean(residual ** 2))),
        "residual_p95_abs_us": float(np.percentile(np.abs(residual), 95.0)),
        "residual_max_abs_us": float(np.max(np.abs(residual))),
        "count": int(len(x)),
    }


def map_sample_timestamp_to_monotonic_ns(sample_timestamp_us: np.ndarray, affine_a: float, affine_b_us: float) -> np.ndarray:
    return np.rint(1000.0 * (affine_a * np.asarray(sample_timestamp_us, dtype=float) + affine_b_us)).astype(np.int64)


def local_body_angular_velocity_from_rotations(
    times_ns: np.ndarray,
    rotations_b_p: Rotation,
    window_s: float = 0.12,
    poly_degree: int = 2,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Estimate platform-frame angular velocity from FK rotations.

    For each sample i, fit a local polynomial to log(R_i^-1 R_j) over a
    symmetric time window. The linear coefficient is body angular velocity in
    platform_link coordinates. This avoids direct adjacent finite differences.
    """
    times_s = (np.asarray(times_ns, dtype=float) - float(times_ns[0])) * 1e-9
    n = len(times_s)
    if n != len(rotations_b_p):
        raise ValueError("time and rotation lengths differ")
    if n < poly_degree + 2:
        raise ValueError("not enough FK samples for local polynomial smoothing")

    half = 0.5 * float(window_s)
    degree = int(poly_degree)
    omega = np.full((n, 3), np.nan, dtype=float)
    valid = np.zeros(n, dtype=bool)
    min_points = degree + 2
    used_counts: list[int] = []

    for i in range(n):
        dt = times_s - times_s[i]
        idx = np.flatnonzero(np.abs(dt) <= half)
        if len(idx) < min_points:
            continue
        dt_i = dt[idx]
        # Smooth compact Gaussian weighting inside the chosen local window.
        sigma = max(half / 2.0, 1e-6)
        w = np.exp(-0.5 * (dt_i / sigma) ** 2)
        xcols = [np.ones_like(dt_i)]
        for p in range(1, degree + 1):
            xcols.append(dt_i ** p)
        xmat = np.vstack(xcols).T
        rel = rotations_b_p[i].inv() * rotations_b_p[idx]
        ymat = rel.as_rotvec()
        aw = xmat * w[:, None]
        bw = ymat * w[:, None]
        try:
            coeff, *_ = np.linalg.lstsq(aw, bw, rcond=None)
        except np.linalg.LinAlgError:
            continue
        omega[i, :] = coeff[1, :]
        valid[i] = True
        used_counts.append(int(len(idx)))

    return omega, valid, {
        "method": "local_so3_log_polynomial",
        "window_s": float(window_s),
        "poly_degree": degree,
        "valid_count": int(np.sum(valid)),
        "invalid_count": int(n - np.sum(valid)),
        "local_points_min": int(min(used_counts) if used_counts else 0),
        "local_points_max": int(max(used_counts) if used_counts else 0),
    }


def interpolate_gyro(times_imu_ns: np.ndarray, gyro_imu: np.ndarray, query_times_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(times_imu_ns)
    t = np.asarray(times_imu_ns, dtype=float)[order]
    g = np.asarray(gyro_imu, dtype=float)[order]
    unique_t, unique_idx = np.unique(t, return_index=True)
    g = g[unique_idx]
    valid = (query_times_ns >= unique_t[0]) & (query_times_ns <= unique_t[-1])
    out = np.full((len(query_times_ns), 3), np.nan, dtype=float)
    for axis in range(3):
        out[valid, axis] = np.interp(np.asarray(query_times_ns, dtype=float)[valid], unique_t, g[:, axis])
    return out, valid


@dataclass
class DataQualityGateConfig:
    require_publisher_sequence: bool = True
    max_missing_publisher_sequences: int = 0
    max_duplicate_publisher_sequences: int = 0
    max_out_of_order_publisher_sequences: int = 0
    max_missing_sample_timestamps: int = 0
    max_sample_timestamp_resets: int = 0
    min_platform_rows: int = 12


def _sequence_stats(values: Iterable[int], expected_step: int = 1) -> dict[str, Any]:
    vals = np.asarray([int(v) for v in values], dtype=np.int64)
    if len(vals) < 2:
        return {
            "count": int(len(vals)),
            "missing": 0,
            "duplicates": 0,
            "out_of_order": 0,
            "positive_gap_count": 0,
            "min_delta": None,
            "max_delta": None,
        }
    diff = np.diff(vals)
    missing = int(np.sum(np.maximum(0, diff - expected_step) // max(1, expected_step)))
    return {
        "count": int(len(vals)),
        "missing": missing,
        "duplicates": int(np.sum(diff == 0)),
        "out_of_order": int(np.sum(diff < 0)),
        "positive_gap_count": int(np.sum(diff > expected_step)),
        "min_delta": int(np.min(diff)),
        "max_delta": int(np.max(diff)),
    }


def evaluate_platform_data_quality(
    rows: list[dict[str, Any]],
    config: DataQualityGateConfig | None = None,
) -> dict[str, Any]:
    cfg = config or DataQualityGateConfig()
    failures: list[str] = []
    valid_rows = [r for r in rows if r.get("has_sample_timestamp", False)]
    if len(valid_rows) < cfg.min_platform_rows:
        failures.append(f"platform rows with 0x51 timestamps {len(valid_rows)} < {cfg.min_platform_rows}")

    publisher_sequence_present = all("publisher_sequence" in r for r in valid_rows) if valid_rows else False
    if cfg.require_publisher_sequence and not publisher_sequence_present:
        failures.append("publisher_sequence missing from one or more platform TimedImu rows")
    pub_stats = _sequence_stats([r["publisher_sequence"] for r in valid_rows], 1) if publisher_sequence_present else {"count": 0}
    if int(pub_stats.get("missing", 0)) > cfg.max_missing_publisher_sequences:
        failures.append(f"publisher_sequence missing count {pub_stats['missing']} > {cfg.max_missing_publisher_sequences}")
    if int(pub_stats.get("duplicates", 0)) > cfg.max_duplicate_publisher_sequences:
        failures.append(f"publisher_sequence duplicates {pub_stats['duplicates']} > {cfg.max_duplicate_publisher_sequences}")
    if int(pub_stats.get("out_of_order", 0)) > cfg.max_out_of_order_publisher_sequences:
        failures.append(f"publisher_sequence out-of-order {pub_stats['out_of_order']} > {cfg.max_out_of_order_publisher_sequences}")

    sample_unwrapped, unwrap_diag = unwrap_u32_us([int(r["sample_timestamp_us"]) for r in valid_rows]) if valid_rows else (np.asarray([], dtype=float), {"wraps": 0, "resets": 0, "nonmonotonic": 0})
    if len(sample_unwrapped) >= 2:
        dt = np.diff(sample_unwrapped)
        sample_missing = int(np.sum(np.maximum(0.0, dt - 5001.0) // 5000.0))
        sample_positive_gap_count = int(np.sum(dt > 5001.0))
        sample_stats = {
            "count": int(len(sample_unwrapped)),
            "missing": sample_missing,
            "positive_gap_count": sample_positive_gap_count,
            "duplicates": int(np.sum(dt == 0.0)),
            "out_of_order": int(np.sum(dt < 0.0)),
            "min_delta_us": float(np.min(dt)),
            "p50_delta_us": float(np.percentile(dt, 50.0)),
            "p95_delta_us": float(np.percentile(dt, 95.0)),
            "max_delta_us": float(np.max(dt)),
        }
    else:
        sample_stats = {"count": int(len(sample_unwrapped)), "missing": 0, "positive_gap_count": 0, "duplicates": 0, "out_of_order": 0}
    if int(sample_stats.get("missing", 0)) > cfg.max_missing_sample_timestamps:
        failures.append(f"0x51 missing sample count {sample_stats['missing']} > {cfg.max_missing_sample_timestamps}")
    if int(unwrap_diag.get("resets", 0)) > cfg.max_sample_timestamp_resets or int(unwrap_diag.get("nonmonotonic", 0)) > 0:
        failures.append(f"0x51 reset/nonmonotonic detected: {unwrap_diag}")

    return {
        "status": "PASS" if not failures else "DO_NOT_FIT",
        "failures": failures,
        "config": cfg.__dict__,
        "publisher_sequence": pub_stats,
        "sample_timestamp_0x51": sample_stats,
        "sample_timestamp_unwrap": unwrap_diag,
        "valid_sample_timestamp_rows": int(len(valid_rows)),
        "input_rows": int(len(rows)),
    }


@dataclass
class OrientationCalibrationResult:
    rotation_p_ip: np.ndarray
    gyro_bias_ip_rad_s: np.ndarray
    residuals_rad_s: np.ndarray
    singular_values: np.ndarray
    rank: int
    condition_number: float
    covariance: np.ndarray | None
    iterations: int

    def to_jsonable(self) -> dict[str, Any]:
        residual_norm = np.linalg.norm(self.residuals_rad_s, axis=1)
        return {
            "extrinsic": {
                "estimated_parameter": "R_P_IP",
                "frame_from": "pus6_platform_imu_link",
                "frame_to": "platform_link",
                "rotation_matrix_R_P_IP": self.rotation_p_ip.tolist(),
                "quaternion_xyzw_R_P_IP": Rotation.from_matrix(self.rotation_p_ip).as_quat().tolist(),
                "gyro_bias_ip_rad_s": self.gyro_bias_ip_rad_s.tolist(),
                "lever_arm_status": "NOT_ESTIMATED_J9_E1A",
                "robot_extrinsics_status": "UNCALIBRATED_UNSET",
            },
            "residual_rad_s": {
                "count": int(len(residual_norm)),
                "rms": float(np.sqrt(np.mean(residual_norm ** 2))),
                "p95": float(np.percentile(residual_norm, 95.0)),
                "max": float(np.max(residual_norm)),
            },
            "observability": {
                "rank": int(self.rank),
                "singular_values": self.singular_values.tolist(),
                "condition_number": float(self.condition_number),
                "covariance_orientation_bias": None if self.covariance is None else self.covariance.tolist(),
            },
            "iterations": int(self.iterations),
        }


def solve_rotation_and_bias(
    omega_platform_p: np.ndarray,
    gyro_imu_ip: np.ndarray,
    max_iterations: int = 40,
    tolerance: float = 1e-11,
    min_rotation_vector_norm: float = 1e-5,
) -> OrientationCalibrationResult:
    """Solve min || R_P_IP (gyro_IP - b_IP) - omega_P || on SO(3)."""
    w = np.asarray(omega_platform_p, dtype=float)
    g = np.asarray(gyro_imu_ip, dtype=float)
    good = np.all(np.isfinite(w), axis=1) & np.all(np.isfinite(g), axis=1)
    w = w[good]
    g = g[good]
    if len(w) < 12:
        raise ValueError("at least 12 matched gyro/FK angular velocity samples are required")

    bias = np.mean(g, axis=0) - np.mean(w, axis=0)
    rot = Rotation.identity()
    last_cost = math.inf
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        unbiased = g - bias
        informative = (np.linalg.norm(unbiased, axis=1) > min_rotation_vector_norm) & (
            np.linalg.norm(w, axis=1) > min_rotation_vector_norm
        )
        if int(np.sum(informative)) < 6:
            raise ValueError("angular velocity excitation is insufficient for orientation alignment")
        rot, _ = Rotation.align_vectors(w[informative], unbiased[informative])
        bias_new = np.mean(g - rot.inv().apply(w), axis=0)
        pred = rot.apply(g - bias_new)
        residuals = pred - w
        cost = float(np.mean(np.sum(residuals ** 2, axis=1)))
        if abs(last_cost - cost) < tolerance and float(np.linalg.norm(bias_new - bias)) < tolerance:
            bias = bias_new
            break
        bias = bias_new
        last_cost = cost

    rmat = rot.as_matrix()
    pred = rot.apply(g - bias)
    residuals = pred - w
    jac_blocks = []
    for row in pred:
        jac_blocks.append(np.hstack([-skew(row), -rmat]))
    jac = np.vstack(jac_blocks)
    singular_values = np.linalg.svd(jac, compute_uv=False)
    smax = float(singular_values[0]) if len(singular_values) else 0.0
    rank = int(np.sum(singular_values > max(smax * 1e-9, 1e-10)))
    condition = math.inf if len(singular_values) == 0 or singular_values[-1] <= 0.0 else float(singular_values[0] / singular_values[-1])
    covariance = None
    dof = max(1, 3 * len(w) - 6)
    if rank == 6:
        sigma2 = float(np.sum(residuals ** 2) / dof)
        covariance = sigma2 * np.linalg.pinv(jac.T @ jac)
    return OrientationCalibrationResult(rmat, bias, residuals, singular_values, rank, condition, covariance, iterations)


def parse_platform_timed_rows(rows: list[dict[str, Any]], affine_a: float | None, affine_b_us: float | None) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    valid_rows = [r for r in rows if r.get("has_sample_timestamp", False)]
    if not valid_rows:
        raise ValueError("no platform TimedImu rows with 0x51 sample timestamps found")
    sample_unwrapped_us, unwrap_diag = unwrap_u32_us([int(r["sample_timestamp_us"]) for r in valid_rows])
    host_ns = np.asarray([int(r["host_monotonic_arrival_time_ns"]) for r in valid_rows], dtype=np.int64)
    if affine_a is None or affine_b_us is None:
        fit = fit_affine_device_to_host(sample_unwrapped_us, host_ns)
        affine_a = fit["a"]
        affine_b_us = fit["b_us"]
    else:
        fit = {"a": float(affine_a), "b_us": float(affine_b_us), "source": "cli"}
    t_ns = map_sample_timestamp_to_monotonic_ns(sample_unwrapped_us, float(affine_a), float(affine_b_us))
    gyro = np.asarray([r["angular_velocity_rad_s"] for r in valid_rows], dtype=float)
    return t_ns, gyro, {"timestamp_unwrap": unwrap_diag, "affine": fit, "count": len(valid_rows)}


def parse_fk_rows(rows: list[dict[str, Any]]) -> tuple[np.ndarray, Rotation]:
    if not rows:
        raise ValueError("no FK rows found")
    times: list[int] = []
    quats: list[list[float]] = []
    for row in rows:
        if "query_monotonic_timestamp_ns" in row:
            times.append(int(row["query_monotonic_timestamp_ns"]))
        elif "query_time_ns" in row:
            times.append(int(row["query_time_ns"]))
        else:
            raise ValueError("FK row is missing query_monotonic_timestamp_ns")
        if "pose_quaternion_xyzw" in row:
            quats.append([float(v) for v in row["pose_quaternion_xyzw"]])
        elif "pose" in row and "orientation" in row["pose"]:
            q = row["pose"]["orientation"]
            quats.append([float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])])
        else:
            raise ValueError("FK row is missing pose_quaternion_xyzw or pose.orientation")
    order = np.argsort(np.asarray(times, dtype=np.int64))
    return np.asarray(times, dtype=np.int64)[order], Rotation.from_quat(np.asarray(quats, dtype=float)[order])


def load_json_capture(path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    platform_rows = raw.get("platform") or raw.get("platform_timed") or []
    fk_rows = raw.get("fk") or raw.get("fk_state") or []
    diagnostics = raw.get("diagnostics", {})
    return platform_rows, fk_rows, diagnostics


def load_platform_affine_from_j9d_analysis(path: Path, run_label: str | None = None) -> dict[str, float]:
    """Load platform affine timing from a J9-D analysis JSON artifact."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    candidates: list[dict[str, Any]] = []
    if "runs" in raw:
        for run in raw["runs"]:
            if run_label is not None and run.get("label") != run_label:
                continue
            fit = run.get("platform", {}).get("affine_fit_sample_to_host_arrival")
            if fit and "a" in fit and "b_us" in fit:
                candidates.append(fit)
    elif "platform" in raw:
        fit = raw["platform"].get("affine_fit_sample_to_host_arrival") or raw["platform"].get("affine")
        if fit and "a" in fit and "b_us" in fit:
            candidates.append(fit)
    if not candidates:
        raise ValueError("no platform affine fit found in J9-D analysis JSON")
    if len(candidates) > 1 and run_label is None:
        raise ValueError("J9-D analysis contains multiple runs; pass --j9d-run-label")
    return {"a": float(candidates[0]["a"]), "b_us": float(candidates[0]["b_us"])}


def load_rosbag_capture(path: Path, platform_topic: str, fk_topic: str) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    import rosbag2_py
    from h30_imu.msg import TimedImu
    from pus6_msgs.msg import FkStateStamped
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    platform_rows: list[dict[str, Any]] = []
    fk_rows: list[dict[str, Any]] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == platform_topic:
            msg = deserialize_message(data, TimedImu)
            platform_rows.append({
                "has_sample_timestamp": bool(msg.has_sample_timestamp),
                "sample_timestamp_us": int(msg.sample_timestamp_us),
                "publisher_sequence": int(msg.publisher_sequence),
                "host_monotonic_arrival_time_ns": int(msg.host_monotonic_arrival_time_ns),
                "angular_velocity_rad_s": [
                    float(msg.imu.angular_velocity.x),
                    float(msg.imu.angular_velocity.y),
                    float(msg.imu.angular_velocity.z),
                ],
            })
        elif topic == fk_topic:
            msg = deserialize_message(data, FkStateStamped)
            fk_rows.append({
                "query_monotonic_timestamp_ns": int(msg.query_monotonic_timestamp_ns),
                "pose_quaternion_xyzw": [
                    float(msg.pose.orientation.x),
                    float(msg.pose.orientation.y),
                    float(msg.pose.orientation.z),
                    float(msg.pose.orientation.w),
                ],
            })
    return platform_rows, fk_rows, {}


def calibrate_capture(
    platform_rows: list[dict[str, Any]],
    fk_rows: list[dict[str, Any]],
    affine_a: float | None = None,
    affine_b_us: float | None = None,
    smoothing_window_s: float = 0.12,
    poly_degree: int = 2,
    data_quality_config: DataQualityGateConfig | None = None,
    enforce_data_quality: bool = True,
) -> dict[str, Any]:
    data_quality = evaluate_platform_data_quality(platform_rows, data_quality_config)
    if enforce_data_quality and data_quality["status"] != "PASS":
        return {
            "status": "DO_NOT_FIT",
            "data_quality": data_quality,
            "extrinsic": {
                "estimated_parameter": "R_P_IP",
                "frame_from": "pus6_platform_imu_link",
                "frame_to": "platform_link",
                "robot_extrinsics_status": "UNCALIBRATED_UNSET",
            },
        }
    imu_t_ns, imu_gyro, timing_diag = parse_platform_timed_rows(platform_rows, affine_a, affine_b_us)
    fk_t_ns, fk_rot = parse_fk_rows(fk_rows)
    omega_fk, omega_valid, smoothing_diag = local_body_angular_velocity_from_rotations(
        fk_t_ns, fk_rot, window_s=smoothing_window_s, poly_degree=poly_degree
    )
    gyro_at_fk, interp_valid = interpolate_gyro(imu_t_ns, imu_gyro, fk_t_ns)
    matched = omega_valid & interp_valid & np.all(np.isfinite(omega_fk), axis=1) & np.all(np.isfinite(gyro_at_fk), axis=1)
    if int(np.sum(matched)) < 12:
        raise ValueError("not enough matched FK/IMU samples after timing and smoothing")
    result = solve_rotation_and_bias(omega_fk[matched], gyro_at_fk[matched])
    out = result.to_jsonable()
    out["status"] = "FIT_OK"
    out["data_quality"] = data_quality
    out["inputs"] = {
        "platform_timed_count": int(len(platform_rows)),
        "fk_count": int(len(fk_rows)),
        "matched_count": int(np.sum(matched)),
    }
    out["timing"] = timing_diag
    out["fk_angular_velocity"] = smoothing_diag
    out["equations"] = {
        "gyro_residual": "r_i = R_P_IP * (omega_IP_meas_i - b_g_IP) - omega_P_FK_i",
        "time_mapping": "t_host_ns = round(1000 * (a * unwrapped_0x51_sample_timestamp_us + b_us))",
        "fk_time": "FkStateStamped.query_monotonic_timestamp_ns",
    }
    out["trajectory_for_j9_e1b"] = hardware_trajectory_template()
    return out


def hardware_trajectory_template() -> list[dict[str, Any]]:
    return [
        {"segment": "home_stationary", "purpose": "gyro bias, timing sanity, no orientation excitation"},
        {"segment": "roll_excitation", "purpose": "dominant platform_link X angular velocity"},
        {"segment": "pitch_excitation", "purpose": "dominant platform_link Y angular velocity"},
        {"segment": "yaw_excitation", "purpose": "dominant platform_link Z angular velocity"},
        {"segment": "combined_multiaxis_excitation", "purpose": "rank/conditioning improvement for SO(3) alignment"},
        {"segment": "independent_validation", "purpose": "withheld residual and drift check, not used in fit"},
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--input-json", type=Path, help="JSON topic capture containing platform TimedImu rows and FK rows")
    src.add_argument("--rosbag", type=Path, help="rosbag2 directory containing /pus6/imu_platform/timed and /pus6/fk/state")
    parser.add_argument("--platform-topic", default="/pus6/imu_platform/timed")
    parser.add_argument("--fk-topic", default="/pus6/fk/state")
    parser.add_argument("--affine-a", type=float)
    parser.add_argument("--affine-b-us", type=float)
    parser.add_argument("--j9d-analysis-json", type=Path, help="J9-D analysis JSON with platform affine timing fit")
    parser.add_argument("--j9d-run-label", help="Run label to select when the J9-D analysis JSON contains multiple runs")
    parser.add_argument("--smoothing-window-s", type=float, default=0.12)
    parser.add_argument("--poly-degree", type=int, default=2)
    parser.add_argument("--allow-missing-publisher-sequence", action="store_true", help="Disable the new J9-E1B-R2 publisher_sequence gate for legacy captures")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    if args.input_json:
        platform_rows, fk_rows, _ = load_json_capture(args.input_json)
    else:
        platform_rows, fk_rows, _ = load_rosbag_capture(args.rosbag, args.platform_topic, args.fk_topic)

    affine_a = args.affine_a
    affine_b_us = args.affine_b_us
    if args.j9d_analysis_json is not None:
        if affine_a is not None or affine_b_us is not None:
            raise ValueError("use either --j9d-analysis-json or explicit --affine-a/--affine-b-us, not both")
        fit = load_platform_affine_from_j9d_analysis(args.j9d_analysis_json, args.j9d_run_label)
        affine_a = fit["a"]
        affine_b_us = fit["b_us"]

    result = calibrate_capture(
        platform_rows,
        fk_rows,
        affine_a=affine_a,
        affine_b_us=affine_b_us,
        smoothing_window_s=args.smoothing_window_s,
        poly_degree=args.poly_degree,
        data_quality_config=DataQualityGateConfig(require_publisher_sequence=not args.allow_missing_publisher_sequence),
    )
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if result.get("status") == "DO_NOT_FIT":
        print(json.dumps({
            "status": "DO_NOT_FIT",
            "data_quality": result["data_quality"],
            "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        }, indent=2))
    else:
        print(json.dumps({
            "status": result.get("status", "FIT_OK"),
            "R_P_IP_quaternion_xyzw": result["extrinsic"]["quaternion_xyzw_R_P_IP"],
            "gyro_bias_ip_rad_s": result["extrinsic"]["gyro_bias_ip_rad_s"],
            "residual_rad_s": result["residual_rad_s"],
            "observability": result["observability"],
            "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
