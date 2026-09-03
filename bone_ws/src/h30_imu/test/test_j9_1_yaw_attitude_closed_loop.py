import importlib.util
import math
import sys
from pathlib import Path

# Load script module directly
SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_yaw_attitude_closed_loop.py"
spec = importlib.util.spec_from_file_location("j9_1_yaw_attitude_closed_loop", SCRIPT)
exp_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = exp_module
spec.loader.exec_module(exp_module)


def test_yaw_control_law_and_clamp():
    harness = exp_module.J91YawClosedLoopHarness.__new__(exp_module.J91YawClosedLoopHarness)
    import threading
    harness.lock = threading.Lock()
    harness.kp = 0.5
    harness.residual_limit_deg = 0.5
    harness.yaw_ref_deg = 2.0
    harness.latest_plat_rpy = (0.0, 0.0, 1.6)  # meas yaw = 1.6 deg

    # Test nominal: err = 2.0 - 1.6 = 0.4, residual = 0.5 * 0.4 = 0.2, cmd = 2.0 + 0.2 = 2.2
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, 0.4, abs_tol=1e-6)
    assert math.isclose(residual, 0.2, abs_tol=1e-6)
    assert math.isclose(cmd, 2.2, abs_tol=1e-6)

    # Test clamp on large positive error: meas = 0.0 -> err = 2.0 -> raw residual = 1.0 -> clamped to 0.5
    harness.latest_plat_rpy = (0.0, 0.0, 0.0)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, 2.0, abs_tol=1e-6)
    assert math.isclose(residual, 0.5, abs_tol=1e-6)
    assert math.isclose(cmd, 2.5, abs_tol=1e-6)

    # Test clamp on overshoot / negative error: meas = 3.5 -> err = -1.5 -> raw residual = -0.75 -> clamped to -0.5
    harness.latest_plat_rpy = (0.0, 0.0, 3.5)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, -1.5, abs_tol=1e-6)
    assert math.isclose(residual, -0.5, abs_tol=1e-6)
    assert math.isclose(cmd, 1.5, abs_tol=1e-6)


def test_yaw_settled_metrics_closed_loop():
    samples_pos = []
    for i in range(100):
        samples_pos.append({
            "plat_roll_deg": 0.01,
            "plat_pitch_deg": -0.02,
            "plat_yaw_deg": 1.80 if i >= 50 else (2.30 if i == 20 else 0.5),
            "yaw_error_deg": 0.20 if i >= 50 else 1.5,
            "yaw_residual_deg": 0.10 if i >= 50 else 0.5,
            "yaw_cartesian_cmd_deg": 2.10 if i >= 50 else 2.5,
        })
    metrics_pos = exp_module.compute_stage_settled_metrics(samples_pos, ref_angle_deg=2.0, settle_skip_ratio=0.5)
    assert metrics_pos["count"] == 50
    assert math.isclose(metrics_pos["mean_plat_y"], 1.80, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_error"], 0.20, abs_tol=1e-5)
    assert math.isclose(metrics_pos["peak_plat_y"], 2.30, abs_tol=1e-5)
    assert math.isclose(metrics_pos["abs_overshoot_deg"], 0.30, abs_tol=1e-5)
    assert math.isclose(metrics_pos["overshoot_pct"], 15.0, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_residual"], 0.10, abs_tol=1e-5)
    assert math.isclose(metrics_pos["mean_cmd"], 2.10, abs_tol=1e-5)


def test_yaw_negative_control_law_and_metrics():
    harness = exp_module.J91YawClosedLoopHarness.__new__(exp_module.J91YawClosedLoopHarness)
    import threading
    harness.lock = threading.Lock()
    harness.kp = 0.5
    harness.residual_limit_deg = 0.5
    harness.yaw_ref_deg = -2.0
    harness.latest_plat_rpy = (0.0, 0.0, -1.6)  # meas yaw = -1.6 deg

    # Test nominal: err = -2.0 - (-1.6) = -0.4, residual = 0.5 * -0.4 = -0.2, cmd = -2.0 + (-0.2) = -2.2
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, -0.4, abs_tol=1e-6)
    assert math.isclose(residual, -0.2, abs_tol=1e-6)
    assert math.isclose(cmd, -2.2, abs_tol=1e-6)

    # Test clamp on large error toward negative: meas = 0.0 -> err = -2.0 -> clamped to -0.5 -> cmd = -2.5
    harness.latest_plat_rpy = (0.0, 0.0, 0.0)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, -2.0, abs_tol=1e-6)
    assert math.isclose(residual, -0.5, abs_tol=1e-6)
    assert math.isclose(cmd, -2.5, abs_tol=1e-6)

    # Test clamp on overshoot in negative direction: meas = -3.5 -> err = +1.5 -> clamped to +0.5 -> cmd = -1.5
    harness.latest_plat_rpy = (0.0, 0.0, -3.5)
    err, residual, cmd = harness.compute_control_step()
    assert math.isclose(err, 1.5, abs_tol=1e-6)
    assert math.isclose(residual, 0.5, abs_tol=1e-6)
    assert math.isclose(cmd, -1.5, abs_tol=1e-6)

    # Test negative settled metrics
    samples_neg = []
    for i in range(100):
        samples_neg.append({
            "plat_roll_deg": 0.01,
            "plat_pitch_deg": -0.02,
            "plat_yaw_deg": -1.80 if i >= 50 else (-2.30 if i == 20 else -0.5),
            "yaw_error_deg": -0.20 if i >= 50 else -1.5,
            "yaw_residual_deg": -0.10 if i >= 50 else -0.5,
            "yaw_cartesian_cmd_deg": -2.10 if i >= 50 else -2.5,
        })
    metrics_neg = exp_module.compute_stage_settled_metrics(samples_neg, ref_angle_deg=-2.0, settle_skip_ratio=0.5)
    assert metrics_neg["count"] == 50
    assert math.isclose(metrics_neg["mean_plat_y"], -1.80, abs_tol=1e-5)
    assert math.isclose(metrics_neg["mean_error"], -0.20, abs_tol=1e-5)
    assert math.isclose(metrics_neg["peak_plat_y"], -2.30, abs_tol=1e-5)
    assert math.isclose(metrics_neg["abs_overshoot_deg"], 0.30, abs_tol=1e-5)
    assert math.isclose(metrics_neg["overshoot_pct"], 15.0, abs_tol=1e-5)



def test_csv_field_consistency():
    expected_fields = [
        "host_log_time_ns",
        "host_monotonic_time_ns",
        "ros_stamp_sec",
        "ros_stamp_nanosec",
        "stage",
        "yaw_ref_deg",
        "yaw_meas_deg",
        "yaw_error_deg",
        "yaw_residual_deg",
        "yaw_cartesian_cmd_deg",
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
