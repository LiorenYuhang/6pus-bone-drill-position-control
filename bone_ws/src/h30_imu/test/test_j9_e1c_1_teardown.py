from __future__ import annotations

from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "j9_e1c_1_platform_orientation_calibration.py"


def test_shutdown_helper_orders_executor_before_observer_close() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    start = source.index("def shutdown_calibration_pipeline(")
    end = source.index("def start_h30_imu(", start)
    body = source[start:end]

    ordered = [
        "target_pub_node.destroy_node()",
        "executor.shutdown()",
        "spin_thread.join(timeout=5.0)",
        "observer_node.close()",
        "kinematics_node.destroy_node()",
        "control_node.destroy_node()",
        "motor_driver_node.destroy_node()",
        "fk_node.destroy_node()",
        "observer_node.destroy_node()",
    ]

    positions = [body.index(token) for token in ordered]
    assert positions == sorted(positions)
