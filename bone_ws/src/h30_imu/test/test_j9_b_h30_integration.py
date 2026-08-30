from __future__ import annotations

import ast
from pathlib import Path


PKG = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (PKG / relative).read_text(encoding="utf-8")


def test_timed_message_contract_contains_all_timestamp_domains():
    msg = _read("msg/TimedImu.msg")

    assert "std_msgs/Header header" in msg
    assert "uint32 sample_timestamp_us" in msg
    assert "uint64 publisher_sequence" in msg
    assert "uint32 data_ready_timestamp_us" in msg
    assert "int64 host_monotonic_arrival_time_ns" in msg
    assert "sensor_msgs/Imu imu" in msg


def test_dual_launch_keeps_base_and_platform_namespaces_separate():
    source = _read("launch/h30_dual.launch.py")
    ast.parse(source)

    assert "namespace='pus6/imu_base'" in source
    assert "namespace='pus6/imu_platform'" in source
    assert "'frame_id': 'pus6_base_imu_link'" in source
    assert "'frame_id': 'pus6_platform_imu_link'" in source
    assert "'timed_topic': '/pus6/imu_base/timed'" in source
    assert "'timed_topic': '/pus6/imu_platform/timed'" in source
    assert "'diagnostics_topic': '/pus6/imu_base/diagnostics'" in source
    assert "'diagnostics_topic': '/pus6/imu_platform/diagnostics'" in source
    assert "'publish_tf': False" in source


def test_relative_node_declares_io_and_rejects_missing_timestamps():
    source = _read("src/h30_relative_attitude_node.cpp")

    assert "create_subscription<h30_imu::msg::TimedImu>" in source
    assert "create_publisher<h30_imu::msg::RelativeAttitude>" in source
    assert "create_publisher<diagnostic_msgs::msg::DiagnosticArray>" in source
    assert "!msg->has_sample_timestamp" in source
    assert "++invalid_base_" in source
    assert "++invalid_platform_" in source
    assert "Exp(-omega_B*dt)*R*Exp(omega_P*dt)" in source


def test_h30_node_publishes_ros_visible_parser_diagnostics():
    source = _read("src/h30_imu_node.cpp")

    assert "diagnostics_topic" in source
    assert "parser_.stats()" in source
    assert "valid_frames" in source
    assert "checksum_errors" in source
    assert "malformed_tlv_errors" in source
    assert "tid_dropped_frames" in source
    assert "tid_duplicates" in source
    assert "tid_out_of_order" in source
    assert "published_timed_imu_count" in source
    assert "true_tid_missing_frames" in source
    assert "tid_resets" in source
    assert "sample_timestamp_missing_samples" in source
    assert "sample_timestamp_resets" in source
    assert "runtime_true_tid_missing_frames" in source
    assert "runtime_sample_timestamp_missing_samples" in source


def test_timed_imu_qos_is_default_safe_and_calibration_opt_in():
    source = _read("src/h30_imu_node.cpp")
    config = _read("config/h30.yaml")
    launch = _read("launch/h30.launch.py") + _read("launch/h30_dual.launch.py")

    assert 'declare_parameter<std::string>("timed_qos_profile", "sensor_data")' in source
    assert "return rclcpp::SensorDataQoS()" in source
    assert 'profile == "calibration"' in source
    assert "timed_qos_profile: sensor_data" in config
    assert "timed_qos_profile" in launch
    assert "timed_qos_depth" in launch
    assert "timed_qos_reliability" in launch


def test_j9_d_affine_clock_contract_replaces_ambiguous_ppm_config():
    source = _read("src/h30_relative_attitude_node.cpp")
    config = _read("config/h30_relative_attitude.yaml")

    assert "host_monotonic_us = affine_a * unwrapped_0x51_sample_timestamp_us + affine_b_us" in source
    assert "base_clock_affine_a" in source
    assert "platform_clock_affine_a" in source
    assert "base_clock_affine_a:" in config
    assert "platform_clock_affine_a:" in config
    assert "ppm equivalent = (affine_a - 1.0) * 1e6" in config
    assert "0x52 data_ready_timestamp_us is kept as diagnostic latency" in config

    combined = "\n".join([
        source,
        config,
        _read("launch/h30_relative.launch.py"),
        _read("launch/h30_dual.launch.py"),
    ])
    assert "base_clock_scale_ppm" not in combined
    assert "platform_clock_scale_ppm" not in combined


def test_frame_metadata_is_explicit_and_uncalibrated():
    config = _read("config/h30_relative_attitude.yaml")

    assert "base_link_frame: base_link" in config
    assert "platform_link_frame: platform_link" in config
    assert "base_imu_frame: pus6_base_imu_link" in config
    assert "platform_imu_frame: pus6_platform_imu_link" in config
    assert "robot_extrinsics_status: UNCALIBRATED_UNSET" in config


def test_h30_package_has_no_motor_control_dependency():
    forbidden = ("pus6_motor_driver", "pus6_control", "command_joint_steps", "target_joint_steps")
    for subdir in ("src", "include", "launch", "config", "msg"):
        for path in (PKG / subdir).rglob("*"):
            if not path.is_file():
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            assert not any(token in text for token in forbidden), path

    for path in [PKG / "package.xml", PKG / "CMakeLists.txt"]:
        if not path.is_file() or path.parts[-2] == "__pycache__":
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        assert not any(token in text for token in forbidden), path
