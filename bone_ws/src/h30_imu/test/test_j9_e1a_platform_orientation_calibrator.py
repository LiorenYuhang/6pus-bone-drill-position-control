from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


PKG = Path(__file__).resolve().parents[1]
SCRIPT = PKG / "scripts" / "j9_e1a_platform_orientation_calibrator.py"
spec = importlib.util.spec_from_file_location("j9_e1a_platform_orientation_calibrator", SCRIPT)
cal = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = cal
spec.loader.exec_module(cal)


def _integrate_body_rotations(times_ns: np.ndarray, omega_body: np.ndarray) -> Rotation:
    rotations = [Rotation.identity()]
    for i in range(1, len(times_ns)):
        dt = (times_ns[i] - times_ns[i - 1]) * 1e-9
        # R_B_P,k+1 = R_B_P,k * Exp(omega_P * dt), body-rate convention.
        rotations.append(rotations[-1] * Rotation.from_rotvec(omega_body[i - 1] * dt))
    return Rotation.concatenate(rotations)


def _synthetic_dataset(*, multiaxis: bool, noise_std: float = 0.0):
    rng = np.random.default_rng(42)
    dt = 0.005
    t_s = np.arange(0.0, 30.0, dt)
    times_ns = np.rint(t_s * 1e9).astype(np.int64) + 10_000_000_000
    if multiaxis:
        omega_body = np.column_stack([
            0.35 * np.sin(2.0 * math.pi * 0.37 * t_s),
            0.28 * np.sin(2.0 * math.pi * 0.53 * t_s + 0.4),
            0.22 * np.sin(2.0 * math.pi * 0.71 * t_s + 1.0),
        ])
    else:
        omega_body = np.column_stack([
            0.35 * np.sin(2.0 * math.pi * 0.37 * t_s),
            np.zeros_like(t_s),
            np.zeros_like(t_s),
        ])

    r_p_ip = Rotation.from_euler("xyz", [0.31, -0.18, 0.24]).as_matrix()
    bias = np.array([0.004, -0.0025, 0.0015])
    gyro = Rotation.from_matrix(r_p_ip).inv().apply(omega_body) + bias
    if noise_std > 0.0:
        gyro = gyro + rng.normal(0.0, noise_std, size=gyro.shape)
    fk_rot = _integrate_body_rotations(times_ns, omega_body)
    return times_ns, fk_rot, gyro, r_p_ip, bias


def test_affine_mapping_uses_sample_timestamp_not_ready_timestamp():
    raw = [4_294_967_000, 704, 5704]
    sample_us, diag = cal.unwrap_u32_us(raw)
    mapped = cal.map_sample_timestamp_to_monotonic_ns(sample_us, 1.00001, 123.0)

    assert diag["wraps"] == 1
    assert diag["resets"] == 0
    assert np.all(np.diff(mapped) > 0)
    # A bogus ready timestamp sequence is intentionally absent from this API.
    assert mapped[0] == round(1000.0 * (1.00001 * sample_us[0] + 123.0))


def test_loads_platform_affine_from_j9d_analysis_json(tmp_path):
    path = tmp_path / "j9d_analysis.json"
    path.write_text(json.dumps({
        "runs": [
            {"label": "run_a", "platform": {"affine_fit_sample_to_host_arrival": {"a": 0.99, "b_us": 1.0}}},
            {"label": "run_b", "platform": {"affine_fit_sample_to_host_arrival": {"a": 1.01, "b_us": 2.0}}},
        ]
    }), encoding="utf-8")

    fit = cal.load_platform_affine_from_j9d_analysis(path, "run_b")

    assert fit == {"a": 1.01, "b_us": 2.0}


def test_multiaxis_synthetic_orientation_and_bias_recovery_with_noise():
    times_ns, fk_rot, gyro, r_true, bias_true = _synthetic_dataset(multiaxis=True, noise_std=0.001)
    omega_fk, valid, smoothing = cal.local_body_angular_velocity_from_rotations(
        times_ns, fk_rot, window_s=0.16, poly_degree=2
    )
    result = cal.solve_rotation_and_bias(omega_fk[valid], gyro[valid])

    r_err = Rotation.from_matrix(result.rotation_p_ip @ r_true.T).magnitude()
    b_err = np.linalg.norm(result.gyro_bias_ip_rad_s - bias_true)

    assert smoothing["method"] == "local_so3_log_polynomial"
    assert smoothing["valid_count"] > 0.95 * len(times_ns)
    assert r_err < 0.015
    assert b_err < 0.0025
    assert result.rank == 6
    assert result.condition_number < 500.0
    residual_norm = np.linalg.norm(result.residuals_rad_s, axis=1)
    assert float(np.sqrt(np.mean(residual_norm**2))) < 0.008


def test_single_axis_excitation_reports_weak_observability():
    times_ns, fk_rot, gyro, _, _ = _synthetic_dataset(multiaxis=False, noise_std=0.0)
    omega_fk, valid, _ = cal.local_body_angular_velocity_from_rotations(
        times_ns, fk_rot, window_s=0.16, poly_degree=2
    )
    result = cal.solve_rotation_and_bias(omega_fk[valid], gyro[valid])

    assert result.rank < 6 or result.condition_number > 1.0e6


def test_capture_calibration_reports_uncalibrated_status_and_no_lever_arm():
    times_ns, fk_rot, gyro, r_true, _ = _synthetic_dataset(multiaxis=True, noise_std=0.0)
    device_us = np.arange(len(times_ns), dtype=np.uint32) * np.uint32(5000)
    affine_a = 1.000002
    affine_b_us = 12_000_000.0
    host_ns = cal.map_sample_timestamp_to_monotonic_ns(device_us.astype(float), affine_a, affine_b_us)
    platform_rows = [
        {
            "has_sample_timestamp": True,
            "sample_timestamp_us": int(device_us[i]),
            "publisher_sequence": int(i),
            "host_monotonic_arrival_time_ns": int(host_ns[i]),
            "angular_velocity_rad_s": gyro[i].tolist(),
        }
        for i in range(len(times_ns))
    ]
    fk_rows = [
        {
            "query_monotonic_timestamp_ns": int(host_ns[i]),
            "pose_quaternion_xyzw": fk_rot[i].as_quat().tolist(),
        }
        for i in range(len(times_ns))
    ]

    out = cal.calibrate_capture(platform_rows, fk_rows, smoothing_window_s=0.16, poly_degree=2)
    r_est = np.asarray(out["extrinsic"]["rotation_matrix_R_P_IP"])
    r_err = Rotation.from_matrix(r_est @ r_true.T).magnitude()

    assert out["status"] == "FIT_OK"
    assert out["data_quality"]["status"] == "PASS"
    assert r_err < 0.012
    assert out["extrinsic"]["lever_arm_status"] == "NOT_ESTIMATED_J9_E1A"
    assert out["extrinsic"]["robot_extrinsics_status"] == "UNCALIBRATED_UNSET"
    assert out["equations"]["fk_time"] == "FkStateStamped.query_monotonic_timestamp_ns"



def test_capture_calibration_returns_do_not_fit_on_sequence_gap():
    times_ns, fk_rot, gyro, _, _ = _synthetic_dataset(multiaxis=True, noise_std=0.0)
    times_ns = times_ns[:100]
    gyro = gyro[:100]
    device_us = np.arange(len(times_ns), dtype=np.uint32) * np.uint32(5000)
    host_ns = cal.map_sample_timestamp_to_monotonic_ns(device_us.astype(float), 1.0, 12_000_000.0)
    platform_rows = []
    for i in range(len(times_ns)):
        seq = i if i < 50 else i + 3
        sample_us = int(device_us[i] if i < 50 else device_us[i] + np.uint32(15000))
        platform_rows.append({
            "has_sample_timestamp": True,
            "sample_timestamp_us": sample_us,
            "publisher_sequence": int(seq),
            "host_monotonic_arrival_time_ns": int(host_ns[i]),
            "angular_velocity_rad_s": gyro[i].tolist(),
        })
    fk_rows = [
        {
            "query_monotonic_timestamp_ns": int(host_ns[i]),
            "pose_quaternion_xyzw": fk_rot[i].as_quat().tolist(),
        }
        for i in range(len(times_ns))
    ]

    out = cal.calibrate_capture(platform_rows, fk_rows, smoothing_window_s=0.16, poly_degree=2)

    assert out["status"] == "DO_NOT_FIT"
    failures = "\n".join(out["data_quality"]["failures"])
    assert "publisher_sequence missing count" in failures
    assert "0x51 missing sample count" in failures
    assert out["extrinsic"]["robot_extrinsics_status"] == "UNCALIBRATED_UNSET"


def test_data_quality_rejects_missing_publisher_sequence_by_default():
    rows = [
        {"has_sample_timestamp": True, "sample_timestamp_us": i * 5000, "host_monotonic_arrival_time_ns": i * 5_000_000}
        for i in range(20)
    ]

    dq = cal.evaluate_platform_data_quality(rows)

    assert dq["status"] == "DO_NOT_FIT"
    assert any("publisher_sequence missing" in failure for failure in dq["failures"])
