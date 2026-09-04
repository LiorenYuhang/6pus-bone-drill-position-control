import importlib.util
import math
import sys
from pathlib import Path
import numpy as np
from sensor_msgs.msg import Imu

SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "j9_1_dual_imu_translation_recorder.py"
spec = importlib.util.spec_from_file_location("j9_1_dual_imu_translation_recorder", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_process_sample_gravity_removal():
    """Verify that specific force [0, 0, g] in sensor frame with identity rotation produces a_W = [0, 0, 0]."""
    rec = mod.DualImuTranslationRecorder.__new__(mod.DualImuTranslationRecorder)

    msg = Imu()
    msg.header.stamp.sec = 100
    msg.header.stamp.nanosec = 500_000_000
    msg.header.frame_id = "test_imu"
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = mod.G_STANDARD
    msg.angular_velocity.x = 0.01
    msg.angular_velocity.y = -0.02
    msg.angular_velocity.z = 0.03
    msg.orientation.x = 0.0
    msg.orientation.y = 0.0
    msg.orientation.z = 0.0
    msg.orientation.w = 1.0

    row = rec._process_sample("base", msg, seq=42)

    assert row["stream"] == "base"
    assert row["sequence_id"] == 42
    assert math.isclose(row["f_world_z"], mod.G_STANDARD, abs_tol=1e-6)
    assert math.isclose(row["a_world_x"], 0.0, abs_tol=1e-6)
    assert math.isclose(row["a_world_y"], 0.0, abs_tol=1e-6)
    assert math.isclose(row["a_world_z"], 0.0, abs_tol=1e-6)


def test_compute_stream_statistics():
    """Verify statistical computation on synthetic records."""
    records = []
    base_t = 1_000_000_000
    for i in range(100):
        records.append({
            "ros_stamp_ns": base_t + i * 5_000_000, # 200 Hz
            "ax_m_s2": 0.1 * math.sin(i * 0.1),
            "ay_m_s2": 0.2,
            "az_m_s2": 9.8,
            "accel_norm_m_s2": 9.805,
            "gx_rad_s": 0.001,
            "gy_rad_s": 0.002,
            "gz_rad_s": 0.003,
            "a_world_x": 0.05,
            "a_world_y": -0.05,
            "a_world_z": 0.01,
        })

    stats = mod.compute_stream_statistics(records)
    assert stats["sample_count"] == 100
    assert math.isclose(stats["effective_rate_hz"], 200.0, rel_tol=0.01)
    assert stats["timestamp_monotonic"] is True
    assert math.isclose(stats["linear_accel_world_mean_m_s2"][0], 0.05, abs_tol=1e-6)


def test_compute_timestamp_pairing_metrics():
    """Verify nearest-timestamp pairing metrics."""
    base_records = [{"ros_stamp_ns": 1000 + i * 5000} for i in range(10)]
    plat_records = [{"ros_stamp_ns": 1002 + i * 5000} for i in range(10)] # constant 2ns offset

    res = mod.compute_timestamp_pairing_metrics(base_records, plat_records)
    assert res["paired_sample_count"] == 10
    assert math.isclose(res["mean_pairing_offset_ms"], 0.000002, abs_tol=1e-8)


def test_csv_fields():
    assert len(mod.CSV_FIELDS) == 29
    assert "stream" in mod.CSV_FIELDS
    assert "a_world_z" in mod.CSV_FIELDS
    assert "accel_norm_m_s2" in mod.CSV_FIELDS
