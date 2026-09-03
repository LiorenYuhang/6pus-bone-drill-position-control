import importlib.util
import math
import sys
from pathlib import Path

# Load script module directly
SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_roll_attitude_closed_loop.py"
spec = importlib.util.spec_from_file_location("j9_1_roll_attitude_closed_loop", SCRIPT)
exp_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = exp_module
spec.loader.exec_module(exp_module)


def test_control_law_and_clamp():
    # Simulation harness instance without ROS initialization
    harness = exp_module.J91RollClosedLoopHarness.__new__(exp_module.J91RollClosedLoopHarness)
    import threading
    harness.lock = threading.Lock()
    harness.kp = 0.5
    harness.residual_limit_deg = 0.25
    harness.roll_ref_deg = 0.5
    harness.latest_plat_rpy = (0.4, 0.0, 0.0) # meas = 0.4 deg

    # Test nominal: err = 0.5 - 0.4 = 0.1, residual = 0.5 * 0.1 = 0.05, cmd = 0.5 + 0.05 = 0.55
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, 0.1, abs_tol=1e-6)
    assert math.isclose(residual, 0.05, abs_tol=1e-6)
    assert math.isclose(cmd, 0.55, abs_tol=1e-6)

    # Test clamp on large positive error: meas = 0.0 -> err = 0.5 -> raw residual = 0.25 (at limit)
    harness.latest_plat_rpy = (0.0, 0.0, 0.0)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, 0.5, abs_tol=1e-6)
    assert math.isclose(residual, 0.25, abs_tol=1e-6)
    assert math.isclose(cmd, 0.75, abs_tol=1e-6)

    # Test clamp on overshoot / negative error: meas = 1.2 -> err = -0.7 -> raw residual = -0.35 -> clamped to -0.25
    harness.latest_plat_rpy = (1.2, 0.0, 0.0)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, -0.7, abs_tol=1e-6)
    assert math.isclose(residual, -0.25, abs_tol=1e-6)
    assert math.isclose(cmd, 0.25, abs_tol=1e-6)


def test_settled_metrics_closed_loop():
    # Test positive reference metrics
    samples_pos = []
    for i in range(100):
        samples_pos.append({
            "plat_roll_deg": 0.45 if i >= 50 else (0.60 if i == 20 else 0.2),
            "plat_pitch_deg": 0.01,
            "plat_yaw_deg": -0.02,
            "roll_error_deg": 0.05 if i >= 50 else 0.3,
            "roll_residual_deg": 0.025 if i >= 50 else 0.15,
            "roll_cartesian_cmd_deg": 0.525 if i >= 50 else 0.65,
        })
    metrics_pos = exp_module.compute_stage_settled_metrics(samples_pos, ref_angle_deg=0.5, settle_skip_ratio=0.5)
    assert metrics_pos["count"] == 50
    assert math.isclose(metrics_pos["mean_plat_r"], 0.45, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_error"], 0.05, abs_tol=1e-5)
    assert math.isclose(metrics_pos["peak_plat_r"], 0.60, abs_tol=1e-5)
    assert math.isclose(metrics_pos["abs_overshoot_deg"], 0.10, abs_tol=1e-5)
    assert math.isclose(metrics_pos["overshoot_pct"], 20.0, abs_tol=1e-5)

    # Test negative reference metrics
    samples_neg = []
    for i in range(100):
        samples_neg.append({
            "plat_roll_deg": -0.45 if i >= 50 else (-0.62 if i == 20 else -0.2),
            "plat_pitch_deg": 0.01,
            "plat_yaw_deg": -0.02,
            "roll_error_deg": -0.05 if i >= 50 else -0.3,
            "roll_residual_deg": -0.025 if i >= 50 else -0.15,
            "roll_cartesian_cmd_deg": -0.525 if i >= 50 else -0.65,
        })
    metrics_neg = exp_module.compute_stage_settled_metrics(samples_neg, ref_angle_deg=-0.5, settle_skip_ratio=0.5)
    assert metrics_neg["count"] == 50
    assert math.isclose(metrics_neg["mean_plat_r"], -0.45, abs_tol=1e-5)
    assert math.isclose(metrics_neg["mean_error"], -0.05, abs_tol=1e-5)
    assert math.isclose(metrics_neg["peak_plat_r"], -0.62, abs_tol=1e-5)
    assert math.isclose(metrics_neg["abs_overshoot_deg"], 0.12, abs_tol=1e-5)
    assert math.isclose(metrics_neg["overshoot_pct"], 24.0, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_residual"], 0.025, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_cmd"], 0.525, abs_tol=1e-5)


def test_csv_field_consistency():
    expected_fields = [
        "host_log_time_ns",
        "host_monotonic_time_ns",
        "ros_stamp_sec",
        "ros_stamp_nanosec",
        "stage",
        "roll_ref_deg",
        "roll_meas_deg",
        "roll_error_deg",
        "roll_residual_deg",
        "roll_cartesian_cmd_deg",
        "qx",
        "qy",
        "qz",
        "qw",
        "raw_rel_roll_deg",
        "raw_rel_pitch_deg",
        "raw_rel_yaw_deg",
        "plat_roll_deg",
        "plat_pitch_deg",
        "plat_yaw_deg",
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
