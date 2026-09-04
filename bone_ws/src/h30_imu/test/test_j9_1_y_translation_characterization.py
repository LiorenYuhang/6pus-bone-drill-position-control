import csv
import importlib.util
import math
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_y_translation_characterization.py"
spec = importlib.util.spec_from_file_location("j9_1_y_translation_characterization", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def _steps_for_y(kin, y_mm):
    pose = [0.0, y_mm * 1e-3, kin.nominal_height, 0.0, 0.0, 0.0]
    q = kin.inverse_kinematics(pose)
    continuous = [1000.0 + (value - 0.05) * 40000.0 for value in q]
    return continuous, [round(value) for value in continuous]


def test_y_motion_budget_calculation():
    """Verify frozen IK budget for +0.5 mm Y translation."""
    kin = mod.Analytical6PusKinematics()
    continuous, quantized = _steps_for_y(kin, 0.5)
    assert [step - 1000 for step in quantized] == [-4, 3, 1, 1, 3, -4]
    assert max(abs(step - 1000.0) for step in continuous) < 4.0
    assert max(quantized) - min(quantized) == 7


def test_signed_y_reference_and_axis_invariants():
    kin = mod.Analytical6PusKinematics()
    for y_mm, expected_sign in ((0.25, 1), (0.5, 1), (1.0, 1), (-0.25, -1), (-0.5, -1), (-1.0, -1)):
        continuous, quantized = _steps_for_y(kin, y_mm)
        assert all(math.isfinite(step) for step in continuous)
        assert expected_sign * (quantized[0] - 1000) < 0
        pose = [0.0, y_mm * 1e-3, kin.nominal_height, 0.0, 0.0, 0.0]
        assert pose[0] == 0.0
        assert math.isclose(pose[2], kin.nominal_height, abs_tol=1e-15)
        assert pose[3:] == [0.0, 0.0, 0.0]


def test_positive_negative_artifact_naming_and_stages():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "j9_1_y_translation_{dir_tag}_{stamp_str}.csv" in source
    assert "j9_1_y_translation_{dir_tag}_{stamp_str}_metadata.json" in source
    assert "action_y_pos" in source
    assert "action_y_neg" in source
    assert "target_dy_mm >= 0" in source


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


def test_analysis_synthetic_csv(tmp_path: Path):
    """Test analysis function on a synthetic CSV dataset."""
    csv_file = tmp_path / "test_telemetry.csv"
    rows = []
    base_t = 1_000_000_000

    # 100 pre_home rows
    for i in range(100):
        rows.append({
            "stage": "pre_home",
            "fk_x_mm": "0.001",
            "fk_y_mm": "0.000",
            "fk_z_mm": "132.612",
            "fk_roll_deg": "0.000",
            "fk_pitch_deg": "0.000",
            "fk_yaw_deg": "0.000",
            "p_aw_x": "0.001",
            "b_aw_x": "-0.001",
            "p_aw_y": "0.002",
            "p_aw_z": "0.003",
            "b_aw_y": "-0.002",
            "b_aw_z": "-0.020",
            "p_gx_rad_s": "0.0001",
            "p_gy_rad_s": "0.0001",
            "p_gz_rad_s": "0.0001",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_stamp_ns": str(base_t + i * 5_000_000),
            "b_stamp_ns": str(base_t + i * 5_000_000 + 150_000), # 0.15ms offset
            "joint_cmd_1": "1000", "joint_cmd_2": "1000", "joint_cmd_3": "1000",
            "joint_cmd_4": "1000", "joint_cmd_5": "1000", "joint_cmd_6": "1000",
            "joint_fb_1": "1000", "joint_fb_2": "1000", "joint_fb_3": "1000",
            "joint_fb_4": "1000", "joint_fb_5": "1000", "joint_fb_6": "1000",
        })

    # 100 action_y_pos rows
    for i in range(100):
        rows.append({
            "stage": "action_y_pos",
            "fk_x_mm": "0.000",
            "fk_y_mm": "0.502",
            "fk_z_mm": "132.612",
            "fk_roll_deg": "0.001",
            "fk_pitch_deg": "0.000",
            "fk_yaw_deg": "0.000",
            "p_aw_x": "0.001",
            "b_aw_x": "-0.001",
            "p_aw_y": "0.150" if i == 5 else "0.002", # transient peak
            "p_aw_z": "0.003",
            "b_aw_y": "0.005" if i == 5 else "-0.002",
            "b_aw_z": "-0.020",
            "p_gx_rad_s": "0.005" if i == 5 else "0.0001",
            "p_gy_rad_s": "0.0001",
            "p_gz_rad_s": "0.0001",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_stamp_ns": str(base_t + (100 + i) * 5_000_000),
            "b_stamp_ns": str(base_t + (100 + i) * 5_000_000 + 160_000),
            "joint_cmd_1": "996", "joint_cmd_2": "1003", "joint_cmd_3": "1001",
            "joint_cmd_4": "1001", "joint_cmd_5": "1003", "joint_cmd_6": "996",
            "joint_fb_1": "996", "joint_fb_2": "1003", "joint_fb_3": "1001",
            "joint_fb_4": "1001", "joint_fb_5": "1003", "joint_fb_6": "996",
        })

    # 100 post_home rows
    for i in range(100):
        rows.append({
            "stage": "post_home",
            "fk_x_mm": "0.001",
            "fk_y_mm": "0.000",
            "fk_z_mm": "132.612",
            "fk_roll_deg": "0.000",
            "fk_pitch_deg": "0.000",
            "fk_yaw_deg": "0.000",
            "p_aw_x": "0.001",
            "b_aw_x": "-0.001",
            "p_aw_y": "0.002",
            "p_aw_z": "0.003",
            "b_aw_y": "-0.002",
            "b_aw_z": "-0.020",
            "p_gx_rad_s": "0.0001",
            "p_gy_rad_s": "0.0001",
            "p_gz_rad_s": "0.0001",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_stamp_ns": str(base_t + (200 + i) * 5_000_000),
            "b_stamp_ns": str(base_t + (200 + i) * 5_000_000 + 150_000),
            "joint_cmd_1": "1000", "joint_cmd_2": "1000", "joint_cmd_3": "1000",
            "joint_cmd_4": "1000", "joint_cmd_5": "1000", "joint_cmd_6": "1000",
            "joint_fb_1": "1000", "joint_fb_2": "1000", "joint_fb_3": "1000",
            "joint_fb_4": "1000", "joint_fb_5": "1000", "joint_fb_6": "1000",
        })

    with csv_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    metrics = mod.analyze_y_translation_experiment(csv_file, target_dy_mm=0.5)

    assert metrics["status"] == "PASS"
    assert math.isclose(metrics["fk_local_displacement"]["dy_mm"], 0.502, abs_tol=1e-4)
    assert metrics["joint_state"]["delta_steps"] == [-4, 3, 1, 1, 3, -4]
    assert math.isclose(metrics["platform_accel_peaks"]["y_pos_peak"], 0.150, abs_tol=1e-4)
    assert math.isclose(metrics["timestamp_pairing_stats"]["mean_offset_ms"], 0.1533, abs_tol=1e-2)


def test_csv_fields():
    assert "target_y_mm" in mod.CSV_FIELDS
    assert "p_aw_y" in mod.CSV_FIELDS
    assert "b_aw_y" in mod.CSV_FIELDS
    assert "joint_fb_1" in mod.CSV_FIELDS
