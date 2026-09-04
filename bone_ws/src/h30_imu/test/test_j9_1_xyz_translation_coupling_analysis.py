import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_xyz_translation_coupling_analysis.py"
spec = importlib.util.spec_from_file_location("j9_1_xyz_translation_coupling_analysis", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def _write_run(root: Path, stem: str, axis: str, sign: str, fk, rpy, accel_scale: float, gyro_scale: float):
    meta = {
        "status": "PASS",
        "metrics": {
            "status": "PASS",
            "fk_local_displacement": {
                "dx_mm": fk[0], "dy_mm": fk[1], "dz_mm": fk[2],
                "droll_deg": rpy[0], "dpitch_deg": rpy[1], "dyaw_deg": rpy[2],
            },
        },
    }
    (root / f"{stem}_metadata.json").write_text(json.dumps(meta), encoding="utf-8")
    stage = f"action_{axis}_{'pos' if sign == '+' else 'neg'}"
    fields = [
        "host_log_time_ns", "host_monotonic_time_ns", "stage", "p_stamp_ns", "b_stamp_ns",
        "p_aw_x", "p_aw_y", "p_aw_z", "p_ax_raw", "p_ay_raw", "p_az_raw",
        "p_gx_rad_s", "p_gy_rad_s", "p_gz_rad_s",
        "b_aw_x", "b_aw_y", "b_aw_z", "b_ax_raw", "b_ay_raw", "b_az_raw",
    ]
    rows = []
    for i in range(20):
        t = 1_000_000_000 + i * 5_000_000
        rows.append({
            "host_log_time_ns": str(t), "host_monotonic_time_ns": str(t), "stage": "pre_home",
            "p_stamp_ns": str(t), "b_stamp_ns": str(t + 100_000),
            "p_aw_x": "0", "p_aw_y": "0", "p_aw_z": "0", "p_ax_raw": "0", "p_ay_raw": "0", "p_az_raw": "9.8",
            "p_gx_rad_s": "0", "p_gy_rad_s": "0", "p_gz_rad_s": "0",
            "b_aw_x": "0", "b_aw_y": "0", "b_aw_z": "0", "b_ax_raw": "0", "b_ay_raw": "0", "b_az_raw": "9.7",
        })
    for i in range(220):
        t = 2_000_000_000 + i * 5_000_000
        s = 1.0 if i < 80 else 0.25
        direction = 1.0 if sign == '+' else -1.0
        rows.append({
            "host_log_time_ns": str(t), "host_monotonic_time_ns": str(t), "stage": stage,
            "p_stamp_ns": str(t), "b_stamp_ns": str(t + 100_000),
            "p_aw_x": str(direction * accel_scale * s),
            "p_aw_y": str(direction * accel_scale * 0.5 * s),
            "p_aw_z": str(direction * accel_scale * 0.25 * s),
            "p_ax_raw": "0", "p_ay_raw": "0", "p_az_raw": str(9.8 + direction * accel_scale * 0.25 * s),
            "p_gx_rad_s": str(direction * gyro_scale * s),
            "p_gy_rad_s": "0", "p_gz_rad_s": "0",
            "b_aw_x": "0", "b_aw_y": "0", "b_aw_z": "0", "b_ax_raw": "0", "b_ay_raw": "0", "b_az_raw": "9.7",
        })
    for i in range(20):
        t = 4_000_000_000 + i * 5_000_000
        rows.append({
            "host_log_time_ns": str(t), "host_monotonic_time_ns": str(t), "stage": "post_home",
            "p_stamp_ns": str(t), "b_stamp_ns": str(t + 100_000),
            "p_aw_x": "0", "p_aw_y": "0", "p_aw_z": "0", "p_ax_raw": "0", "p_ay_raw": "0", "p_az_raw": "9.8",
            "p_gx_rad_s": "0", "p_gy_rad_s": "0", "p_gz_rad_s": "0",
            "b_aw_x": "0", "b_aw_y": "0", "b_aw_z": "0", "b_ax_raw": "0", "b_ay_raw": "0", "b_az_raw": "9.7",
        })
    with (root / f"{stem}.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_fk_odd_matrix_and_output_files(tmp_path: Path, monkeypatch):
    stems = {}
    for axis in mod.AXES:
        for sign in ("+", "-"):
            stem = f"run_{axis}_{'p' if sign == '+' else 'n'}"
            stems[(axis, sign)] = stem
    monkeypatch.setattr(mod, "FORMAL_RUNS", stems)

    fk_by_axis = {
        "x": [0.4, 0.02, 0.01],
        "y": [0.03, 0.5, 0.02],
        "z": [0.01, 0.04, 0.48],
    }
    rpy_by_axis = {"x": [0.1, 0.0, 0.0], "y": [0.0, 0.2, 0.0], "z": [0.0, 0.0, 0.3]}
    for axis in mod.AXES:
        _write_run(tmp_path, stems[(axis, "+")], axis, "+", fk_by_axis[axis], rpy_by_axis[axis], 0.1, 0.02)
        _write_run(tmp_path, stems[(axis, "-")], axis, "-", [-v for v in fk_by_axis[axis]], [-v for v in rpy_by_axis[axis]], 0.1, 0.02)

    out = tmp_path / "out"
    report = mod.analyze(tmp_path, out)

    assert report["hardware_write_count"] == 0
    assert math.isclose(report["M_FK_mm_per_mm"]["x"]["x"], 0.8, abs_tol=1e-9)
    assert math.isclose(report["M_FK_mm_per_mm"]["y"]["y"], 1.0, abs_tol=1e-9)
    assert report["architecture_decision"]["selected_option"] == "C_measurement_refinement_before_controller_selection"
    assert (out / "j9_1_4e_xyz_translation_coupling_analysis.json").exists()
    assert (out / "j9_1_4e_M_accel_odd_rms_m_s2.csv").exists()


def test_formal_run_stems_exclude_invalid_artifacts():
    stems = set(mod.FORMAL_RUNS.values())
    assert "j9_1_x_translation_pos05mm_20260903_203101" not in stems
    assert "j9_1_y_translation_pos05mm_20260904_092502" not in stems
    assert "j9_1_z_translation_pos05mm_20260904_123707" in stems
    assert "j9_1_z_translation_neg05mm_20260904_131517" in stems
