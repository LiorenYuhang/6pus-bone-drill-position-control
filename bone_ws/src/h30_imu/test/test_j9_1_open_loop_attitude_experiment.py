import importlib.util
import math
import sys
from pathlib import Path

# Load script module directly
SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_open_loop_attitude_experiment.py"
spec = importlib.util.spec_from_file_location("j9_1_open_loop_attitude_experiment", SCRIPT)
exp_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = exp_module
spec.loader.exec_module(exp_module)


def test_settled_metrics_computation():
    samples = []
    # Create 100 samples with transition in first 50 and steady in last 50
    for i in range(100):
        samples.append({
            "relative_roll_deg": 0.1 * i / 50.0 if i < 50 else 0.50 + 0.01 * (i % 3 - 1),
            "relative_pitch_deg": 0.02,
            "relative_yaw_deg": -0.01,
        })

    metrics = exp_module.compute_stage_settled_metrics(samples, settle_skip_ratio=0.5)
    assert metrics["count"] == 50
    assert math.isclose(metrics["mean_r"], 0.50, abs_tol=0.02)
    assert math.isclose(metrics["mean_p"], 0.02, abs_tol=1e-5)
    assert math.isclose(metrics["mean_y"], -0.01, abs_tol=1e-5)


def test_csv_field_consistency():
    expected_fields = [
        "host_log_time_ns",
        "host_monotonic_time_ns",
        "ros_stamp_sec",
        "ros_stamp_nanosec",
        "stage",
        "command_roll_deg",
        "command_pitch_deg",
        "command_yaw_deg",
        "qx",
        "qy",
        "qz",
        "qw",
        "relative_roll_deg",
        "relative_pitch_deg",
        "relative_yaw_deg",
        "platform_roll_deg",
        "platform_pitch_deg",
        "platform_yaw_deg",
        "gx_rad_s",
        "gy_rad_s",
        "gz_rad_s",
        "joint_cmd_1",
        "joint_cmd_2",
        "joint_cmd_3",
        "joint_cmd_4",
        "joint_cmd_5",
        "joint_cmd_6",
        "joint_cmd_stamp_ns",
        "joint_fb_1",
        "joint_fb_2",
        "joint_fb_3",
        "joint_fb_4",
        "joint_fb_5",
        "joint_fb_6",
        "joint_fb_stamp_ns",
        "hardware_write_count",
        "fault_codes",
    ]
    assert exp_module.CSV_FIELDS == expected_fields
