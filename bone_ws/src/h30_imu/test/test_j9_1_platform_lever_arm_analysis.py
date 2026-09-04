import csv
import importlib.util
import json
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_platform_lever_arm_analysis.py"
spec = importlib.util.spec_from_file_location("j9_1_platform_lever_arm_analysis", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def _write_attitude_run(root: Path, stem: str, include_accel: bool = False) -> None:
    metadata = {
        "status": "PASS",
        "hardware_mode": "active",
        "stages_executed": ["roll_pos_closed_loop", "roll_ret_home"],
        "total_recorded_samples": 40,
        "final_write_count": 12,
    }
    (root / f"{stem}_metadata.json").write_text(json.dumps(metadata), encoding="utf-8")

    fields = [
        "host_monotonic_time_ns",
        "stage",
        "roll_ref_deg",
        "pitch_ref_deg",
        "yaw_ref_deg",
        "qx",
        "qy",
        "qz",
        "qw",
        "gx_rad_s",
        "gy_rad_s",
        "gz_rad_s",
        "joint_cmd_1",
        "joint_cmd_2",
        "joint_cmd_3",
        "joint_cmd_4",
        "joint_cmd_5",
        "joint_cmd_6",
        "joint_fb_1",
        "joint_fb_2",
        "joint_fb_3",
        "joint_fb_4",
        "joint_fb_5",
        "joint_fb_6",
    ]
    if include_accel:
        fields += ["p_aw_x", "p_aw_y", "p_aw_z"]

    rows = []
    for i in range(40):
        stage = "roll_pos_closed_loop" if i < 30 else "roll_ret_home"
        row = {
            "host_monotonic_time_ns": str(1_000_000_000 + i * 5_000_000),
            "stage": stage,
            "roll_ref_deg": "0.5" if stage == "roll_pos_closed_loop" else "0.0",
            "pitch_ref_deg": "0.0",
            "yaw_ref_deg": "0.0",
            "qx": "0.0",
            "qy": "0.0",
            "qz": "0.0",
            "qw": "1.0",
            "gx_rad_s": str(0.01 * i),
            "gy_rad_s": str(0.005 * i),
            "gz_rad_s": "0.0",
        }
        for axis in range(1, 7):
            row[f"joint_cmd_{axis}"] = "1000"
            row[f"joint_fb_{axis}"] = "1000"
        if include_accel:
            row["p_aw_x"] = "0.0"
            row["p_aw_y"] = "0.0"
            row["p_aw_z"] = "0.0"
        rows.append(row)

    with (root / f"{stem}.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_missing_platform_accel_forces_not_reliably_identifiable(tmp_path: Path):
    _write_attitude_run(tmp_path, "attitude_no_accel", include_accel=False)
    out = tmp_path / "out"

    report = mod.analyze((tmp_path,), out)

    assert report["hardware_write_count"] == 0
    assert report["availability_summary"]["candidate_count"] == 1
    assert report["availability_summary"]["usable_for_lever_arm_fit_count"] == 0
    assert report["datasets"][0]["has_gyro_xyz"] is True
    assert report["datasets"][0]["has_platform_accel_xyz"] is False
    assert report["decision"]["verdict"] == "NOT_RELIABLY_IDENTIFIABLE"
    assert report["lever_arm_estimates_mm"]["combined_mm"] is None
    assert (out / "j9_1_4f_platform_lever_arm_identifiability.json").exists()
    assert (out / "j9_1_4f_attitude_dataset_availability.csv").exists()


def test_accel_field_contract_marks_dataset_fit_eligible(tmp_path: Path):
    _write_attitude_run(tmp_path, "attitude_with_accel", include_accel=True)

    dataset = mod.classify_dataset(mod.DatasetSpec(
        "attitude_with_accel",
        tmp_path / "attitude_with_accel.csv",
        tmp_path / "attitude_with_accel_metadata.json",
        "tmp",
    ))

    assert dataset["has_platform_accel_xyz"] is True
    assert dataset["usable_for_lever_arm_fit"] is True


def test_gyro_only_excitation_audit_reports_rank(tmp_path: Path):
    _write_attitude_run(tmp_path, "attitude_no_accel", include_accel=False)
    audit = mod.excitation_audit(
        mod.DatasetSpec(
            "attitude_no_accel",
            tmp_path / "attitude_no_accel.csv",
            tmp_path / "attitude_no_accel_metadata.json",
            "tmp",
        ),
        filter_window=5,
    )

    assert audit["sample_count"] == 30
    assert audit["rank"] >= 1
    assert audit["dt_ms"]["mean"] > 0.0
