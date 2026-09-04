import csv
import importlib.util
import math
import sys
from pathlib import Path
import numpy as np

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_x_translation_characterization.py"
spec = importlib.util.spec_from_file_location("j9_1_x_translation_characterization", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_motion_budget_calculation():
    """Verify kinematics calculation for +0.5 mm X translation."""
    kin = mod.Analytical6PusKinematics()
    z0 = kin.nominal_height

    pose_home = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    q_home = kin.inverse_kinematics(pose_home)
    steps_home = [1000.0 + (q - 0.05) * 40000.0 for q in q_home]

    pose_x = [0.0005, 0.0, z0, 0.0, 0.0, 0.0]
    q_x = kin.inverse_kinematics(pose_x)
    steps_x_cont = [1000.0 + (q - 0.05) * 40000.0 for q in q_x]
    steps_x_int = [round(s) for s in steps_x_cont]

    delta_int = [steps_x_int[i] - 1000 for i in range(6)]
    assert delta_int == [-1, 3, 4, -3, -3, 1]
    assert max(abs(s - 1000) for s in steps_x_cont) < 4.0
    assert max(steps_x_int) - min(steps_x_int) == 7


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
            "p_aw_y": "0.002",
            "p_aw_z": "0.003",
            "b_aw_x": "-0.001",
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

    # 100 action_x_pos rows
    for i in range(100):
        rows.append({
            "stage": "action_x_pos",
            "fk_x_mm": "0.502",
            "fk_y_mm": "0.001",
            "fk_z_mm": "132.612",
            "fk_roll_deg": "0.001",
            "fk_pitch_deg": "0.000",
            "fk_yaw_deg": "0.000",
            "p_aw_x": "0.150" if i == 5 else "0.002", # transient peak
            "p_aw_y": "0.002",
            "p_aw_z": "0.003",
            "b_aw_x": "0.005" if i == 5 else "-0.001",
            "b_aw_y": "-0.002",
            "b_aw_z": "-0.020",
            "p_gx_rad_s": "0.005" if i == 5 else "0.0001",
            "p_gy_rad_s": "0.0001",
            "p_gz_rad_s": "0.0001",
            "b_gx_rad_s": "0.0001",
            "b_gy_rad_s": "0.0001",
            "b_gz_rad_s": "0.0001",
            "p_stamp_ns": str(base_t + (100 + i) * 5_000_000),
            "b_stamp_ns": str(base_t + (100 + i) * 5_000_000 + 160_000),
            "joint_cmd_1": "999", "joint_cmd_2": "1003", "joint_cmd_3": "1004",
            "joint_cmd_4": "997", "joint_cmd_5": "997", "joint_cmd_6": "1001",
            "joint_fb_1": "999", "joint_fb_2": "1003", "joint_fb_3": "1004",
            "joint_fb_4": "997", "joint_fb_5": "997", "joint_fb_6": "1001",
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
            "p_aw_y": "0.002",
            "p_aw_z": "0.003",
            "b_aw_x": "-0.001",
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

    metrics = mod.analyze_x_translation_experiment(csv_file, target_dx_mm=0.5)

    assert metrics["status"] == "PASS"
    assert math.isclose(metrics["fk_local_displacement"]["dx_mm"], 0.501, abs_tol=1e-4)
    assert metrics["joint_state"]["delta_steps"] == [-1, 3, 4, -3, -3, 1]
    assert math.isclose(metrics["platform_accel_peaks"]["x_pos_peak"], 0.150, abs_tol=1e-4)
    assert math.isclose(metrics["timestamp_pairing_stats"]["mean_offset_ms"], 0.1533, abs_tol=1e-2)


def test_csv_fields():
    assert "target_x_mm" in mod.CSV_FIELDS
    assert "p_aw_x" in mod.CSV_FIELDS
    assert "b_aw_x" in mod.CSV_FIELDS
    assert "joint_fb_1" in mod.CSV_FIELDS
