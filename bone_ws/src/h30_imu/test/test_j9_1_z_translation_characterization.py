import csv
import importlib.util
import math
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_z_translation_characterization.py"
spec = importlib.util.spec_from_file_location("j9_1_z_translation_characterization", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def _steps_for_z(kin, z_delta_mm):
    pose = [0.0, 0.0, kin.nominal_height + z_delta_mm * 1e-3, 0.0, 0.0, 0.0]
    q = kin.inverse_kinematics(pose)
    continuous = [1000.0 + (value - 0.05) * 40000.0 for value in q]
    return continuous, [round(value) for value in continuous]


def test_z_motion_budget_calculation():
    """Verify frozen IK budget for +0.5 mm Z translation."""
    kin = mod.Analytical6PusKinematics()
    continuous, quantized = _steps_for_z(kin, 0.5)
    assert [step - 1000 for step in quantized] == [20, 20, 20, 20, 20, 20]
    assert max(abs(step - 1000.0) for step in continuous) < 21.0
    assert max(quantized) - min(quantized) == 0


def test_signed_z_reference_and_axis_invariants():
    kin = mod.Analytical6PusKinematics()
    for z_delta_mm, expected_delta in ((0.25, 10), (0.5, 20), (1.0, 40), (-0.25, -10), (-0.5, -20), (-1.0, -40)):
        continuous, quantized = _steps_for_z(kin, z_delta_mm)
        assert all(math.isfinite(step) for step in continuous)
        assert [step - 1000 for step in quantized] == [expected_delta] * 6
        pose = [0.0, 0.0, kin.nominal_height + z_delta_mm * 1e-3, 0.0, 0.0, 0.0]
        assert pose[0] == 0.0
        assert pose[1] == 0.0
        assert math.isclose(pose[2], kin.nominal_height + z_delta_mm * 1e-3, abs_tol=1e-15)
        assert pose[3:] == [0.0, 0.0, 0.0]


def test_positive_negative_artifact_naming_and_stages():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "j9_1_z_translation_{dir_tag}_{stamp_str}.csv" in source
    assert "j9_1_z_translation_{dir_tag}_{stamp_str}_metadata.json" in source
    assert "action_z_pos" in source
    assert "action_z_neg" in source
    assert "target_dz_mm >= 0" in source


def test_hardware_mode_guard_and_j8_topic_contract():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "hardware_mode=\"active\"" in source
    assert "use_j7_servo=True" in source
    for topic in (
        "\"/pus6/target_cartesian_pose\"",
        "\"/pus6/command_joint_steps\"",
        "\"/pus6/joint_feedback/timed\"",
        "\"/pus6/fk_state\"",
        "\"/pus6/imu_base/data\"",
        "\"/pus6/imu_platform/data\"",
    ):
        assert topic in source
    assert "\"hardware_write_count\"" in source
    assert "\"fault_codes\"" in source


def test_z_action_target_keeps_xy_zero_and_offsets_nominal_z():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "harness.set_stage(action_stage, 0.0, 0.0, z0 + target_dz_m)" in source
    assert "harness.publish_target_pose(x_m=0.0, y_m=0.0, z_m=z0 + target_dz_m)" in source


def test_analysis_synthetic_csv(tmp_path: Path):
    """Test analysis function on a synthetic Z-axis CSV dataset."""
    csv_file = tmp_path / "test_telemetry.csv"
    rows = []
    base_t = 1_000_000_000

    def row(stage, idx, fk_z, p_aw_z, p_raw_z, cmd, fb):
        return {
            "stage": stage,
            "fk_x_mm": "0.001",
            "fk_y_mm": "0.000",
            "fk_z_mm": f"{fk_z:.3f}",
            "fk_roll_deg": "0.000",
            "fk_pitch_deg": "0.000",
            "fk_yaw_deg": "0.000",
            "p_aw_x": "0.001",
            "p_aw_y": "0.002",
            "p_aw_z": f"{p_aw_z:.3f}",
            "b_aw_x": "-0.001",
            "b_aw_y": "-0.002",
            "b_aw_z": "-0.020",
            "p_ax_raw": "0.010",
            "p_ay_raw": "0.020",
            "p_az_raw": f"{p_raw_z:.3f}",
            "b_ax_raw": "-0.010",
            "b_ay_raw": "-0.020",
            "b_az_raw": "9.780",
            "p_gx_rad_s": "0.005" if idx == 5 else "0.0001",
            "p_gy_rad_s": "0.0001",
            "p_gz_rad_s": "0.0001",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_stamp_ns": str(base_t + idx * 5_000_000),
            "b_stamp_ns": str(base_t + idx * 5_000_000 + 150_000),
            **{f"joint_cmd_{i}": str(cmd[i - 1]) for i in range(1, 7)},
            **{f"joint_fb_{i}": str(fb[i - 1]) for i in range(1, 7)},
        }

    for i in range(100):
        rows.append(row("pre_home", i, 132.612, 0.003, 9.806, [1000] * 6, [1000] * 6))
    for i in range(100):
        rows.append(row("action_z_pos", 100 + i, 133.114, 0.180 if i == 5 else 0.003, 9.990 if i == 5 else 9.806, [1020] * 6, [1020] * 6))
    for i in range(100):
        rows.append(row("post_home", 200 + i, 132.612, 0.003, 9.806, [1000] * 6, [1000] * 6))

    with csv_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    metrics = mod.analyze_z_translation_experiment(csv_file, target_dz_mm=0.5)

    assert metrics["status"] == "PASS"
    assert math.isclose(metrics["fk_local_displacement"]["dz_mm"], 0.502, abs_tol=1e-4)
    assert math.isclose(metrics["fk_local_displacement"]["dy_mm"], 0.0, abs_tol=1e-4)
    assert metrics["joint_state"]["delta_steps"] == [20, 20, 20, 20, 20, 20]
    assert math.isclose(metrics["platform_accel_peaks"]["z_pos_peak"], 0.180, abs_tol=1e-4)
    assert math.isclose(metrics["timestamp_pairing_stats"]["mean_offset_ms"], 0.150, abs_tol=1e-4)


def test_csv_fields():
    assert "target_x_mm" in mod.CSV_FIELDS
    assert "target_y_mm" in mod.CSV_FIELDS
    assert "target_z_mm" in mod.CSV_FIELDS
    assert "p_az_raw" in mod.CSV_FIELDS
    assert "p_aw_z" in mod.CSV_FIELDS
    assert "b_az_raw" in mod.CSV_FIELDS
    assert "b_aw_z" in mod.CSV_FIELDS
    assert "joint_fb_1" in mod.CSV_FIELDS
