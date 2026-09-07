import csv
import importlib.util
import math
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_rotation_lever_arm_characterization.py"
spec = importlib.util.spec_from_file_location("j9_1_rotation_lever_arm_characterization", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_roll_positive_reference_and_ik_budget_is_finite():
    kin = mod.Analytical6PusKinematics()
    continuous, quantized = mod.steps_for_attitude(kin, 0.5, 0.0, 0.0)

    assert len(continuous) == 6
    assert len(quantized) == 6
    assert all(math.isfinite(step) for step in continuous)
    assert max(abs(step - 1000.0) for step in continuous) > 1.0
    assert max(abs(step - 1000.0) for step in continuous) < 80.0


def test_translation_reference_remains_zero_and_pitch_yaw_zero():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "msg.pose.position.x = 0.0" in source
    assert "msg.pose.position.y = 0.0" in source
    assert "msg.pose.position.z = float(self.z0)" in source
    assert "harness.publish_rotation_target(angle_deg, 0.0, 0.0)" in source
    assert "J9.1-4G rotation measurement requires a nonzero roll angle" in source
    assert "action_roll_neg" in source


def test_output_schema_preserves_acceleration_and_timestamps():
    for field in (
        "host_log_time_ns",
        "host_monotonic_time_ns",
        "p_stamp_ns",
        "p_qx",
        "p_qy",
        "p_qz",
        "p_qw",
        "p_gx_rad_s",
        "p_gy_rad_s",
        "p_gz_rad_s",
        "p_ax_raw",
        "p_ay_raw",
        "p_az_raw",
        "p_aw_x",
        "p_aw_y",
        "p_aw_z",
        "fk_x_mm",
        "fk_roll_deg",
    ):
        assert field in mod.CSV_FIELDS


def test_stage_labels_and_artifact_naming_contract():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "pre_home" in source
    assert "action_roll_pos" in source
    assert "return_home" in source
    assert "post_home" in source
    assert "j9_1_rotation_lever_arm_{axis}_{dir_tag}_{stamp_str}.csv" in source
    assert "j9_1_rotation_lever_arm_{axis}_{dir_tag}_{stamp_str}_metadata.json" in source
    assert "pos{int(round(abs(angle_deg) * 10)):02d}deg" in source
    assert "neg{int(round(abs(angle_deg) * 10)):02d}deg" in source


def test_hardware_mode_guard_and_no_negative_first_run():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "hardware_mode=\"active\"" in source
    assert "use_j7_servo=True" in source
    assert "if angle_deg == 0.0" in source
    assert "nonzero roll angle" in source


def test_analysis_synthetic_rotation_csv(tmp_path: Path):
    csv_file = tmp_path / "synthetic_rotation.csv"
    fields = list(mod.CSV_FIELDS)
    rows = []
    base_t = 1_000_000_000

    def row(stage: str, idx: int, fk_roll: float, gx: float, raw_ax: float, aw_x: float):
        qx, qy, qz, qw = mod.euler_deg_to_quat_xyzw(fk_roll, 0.0, 0.0)
        data = {field: "0" for field in fields}
        data.update({
            "host_log_time_ns": str(base_t + idx * 5_000_000),
            "host_monotonic_time_ns": str(base_t + idx * 5_000_000),
            "stage": stage,
            "target_x_mm": "0.0",
            "target_y_mm": "0.0",
            "target_z_mm": "132.6123",
            "target_roll_deg": "0.5" if stage == "action_roll_pos" else "0.0",
            "target_pitch_deg": "0.0",
            "target_yaw_deg": "0.0",
            "fk_x_mm": "0.020" if stage == "action_roll_pos" else "0.000",
            "fk_y_mm": "0.010" if stage == "action_roll_pos" else "0.000",
            "fk_z_mm": "132.612",
            "fk_roll_deg": f"{fk_roll:.4f}",
            "fk_pitch_deg": "0.0000",
            "fk_yaw_deg": "0.0000",
            "p_stamp_ns": str(base_t + idx * 5_000_000),
            "b_stamp_ns": str(base_t + idx * 5_000_000 + 200_000),
            "p_qx": f"{qx:.12f}",
            "p_qy": f"{qy:.12f}",
            "p_qz": f"{qz:.12f}",
            "p_qw": f"{qw:.12f}",
            "b_qw": "1.0",
            "p_gx_rad_s": f"{gx:.8f}",
            "p_gy_rad_s": "0.001",
            "p_gz_rad_s": "0.002",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_ax_raw": f"{raw_ax:.6f}",
            "p_ay_raw": "0.020000",
            "p_az_raw": "9.806650",
            "p_aw_x": f"{aw_x:.6f}",
            "p_aw_y": "0.010000",
            "p_aw_z": "0.002000",
            "b_ax_raw": "0.001000",
            "b_ay_raw": "0.002000",
            "b_az_raw": "9.806650",
            "b_aw_x": "0.001000",
            "b_aw_y": "0.002000",
            "b_aw_z": "0.003000",
            "fault_codes": "[0, 0, 0, 0, 0, 0]",
        })
        for axis in range(1, 7):
            data[f"joint_cmd_{axis}"] = "1001" if stage == "action_roll_pos" else "1000"
            data[f"joint_fb_{axis}"] = "1001" if stage == "action_roll_pos" else "1000"
        return data

    for i in range(110):
        rows.append(row("pre_home", i, 0.0, 0.0, 0.010, 0.001))
    for i in range(120):
        rows.append(row("action_roll_pos", 110 + i, 0.48, 0.08 if i == 5 else 0.01, 0.140 if i == 5 else 0.020, 0.120 if i == 5 else 0.004))
    for i in range(50):
        rows.append(row("return_home", 230 + i, 0.05, 0.02, 0.030, 0.010))
    for i in range(110):
        rows.append(row("post_home", 280 + i, 0.0, 0.0, 0.011, 0.001))

    with csv_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    metrics = mod.analyze_rotation_lever_arm_measurement(csv_file, axis="roll", angle_deg=0.5)

    assert metrics["status"] == "PASS"
    assert metrics["telemetry_completeness"]["platform_raw_accel_complete"] is True
    assert metrics["telemetry_completeness"]["platform_gravity_removed_accel_complete"] is True
    assert metrics["telemetry_completeness"]["host_monotonic_strict"] is True
    assert metrics["sample_counts"]["action"] == 120
    assert math.isclose(metrics["fk_local_displacement"]["droll_deg"], 0.48, abs_tol=1e-6)
    assert metrics["fk_local_displacement"]["dx_mm"] > 0.0
    assert metrics["platform_raw_accel"]["action"]["x"]["max"] > 0.10
    assert metrics["qualified_for_lever_arm_identification"] is True
