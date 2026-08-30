#!/usr/bin/env python3
"""J9-E1D-RX2B Targeted UART Handoff Hardware Qualification Runner.

Executes the exact 15-step sequence:
1. READ_ONLY startup audit
2. Canonical HOME
3. Start active J8 ROS motion pipeline at HOME
4. Run active pipeline for ~100s at safe HOME target (50Hz J7 traffic)
5. Release ACTIVE_ROS using exclusive handoff (timer cancel -> executor shutdown -> thread join -> serial close -> RELEASED)
6. Run standalone HOME once
7. Release standalone UART
8. Restart ACTIVE_ROS cleanly at HOME (audit stale targets, fresh feedback/FK)
9. Run restarted pipeline for ~10s
10. Release ACTIVE_ROS completely
11. Run canonical PARK once
12. Final READ_ONLY audit
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[4]
WS_ROOT = REPO_ROOT / "bone_ws"
for pkg in ("pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"):
    path = WS_ROOT / "src" / pkg
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))
scripts_dir = WS_ROOT / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))
imu_scripts_dir = WS_ROOT / "src" / "h30_imu" / "scripts"
if imu_scripts_dir.exists() and str(imu_scripts_dir) not in sys.path:
    sys.path.insert(0, str(imu_scripts_dir))

import rclpy
from geometry_msgs.msg import PoseStamped
from pus6_kinematics.geometry_config import load_geometry_config
from run_j8_c_hardware_readonly_audit import run_j8_c_audit

import j9_e1d_1_static_gravity_calibration as e1d

MOTOR_PORT = e1d.MOTOR_PORT
BAUD_MOTOR = e1d.BAUD_MOTOR


def publish_home_target(node: Any, publisher: Any, z0: float) -> None:
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "pus6_base"
    message.pose.position.x = 0.0
    message.pose.position.y = 0.0
    message.pose.position.z = float(z0)
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 1.0
    publisher.publish(message)


def run_qualification(args: argparse.Namespace) -> int:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.out_dir / "rx2b_raw.json"
    ownership = e1d.UartOwnershipAudit()

    report: dict[str, Any] = {
        "stage": "J9-E1D-RX2B",
        "description": "Targeted UART Ownership Handoff Hardware Qualification",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "hardware_write_count": 0,
        "preflight_audit": None,
        "home_1_initial": None,
        "active_ros_1_startup": None,
        "active_ros_1_stream": None,
        "active_ros_1_release": None,
        "home_2_between": None,
        "active_ros_2_startup": None,
        "active_ros_2_stream": None,
        "active_ros_2_release": None,
        "park_final": None,
        "final_readonly_audit": None,
        "uart_ownership_audit": ownership.as_dict(),
        "status": "INITIALIZING",
        "error": None,
    }

    active_pipeline_1: e1d.ActiveMotionPipeline | None = None
    active_pipeline_2: e1d.ActiveMotionPipeline | None = None

    try:
        # Step 2: READ_ONLY startup audit
        print("=== [Step 2] READ_ONLY Startup Audit (4.0s) ===")
        preflight = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        report["preflight_audit"] = preflight
        report["uart_ownership_audit"] = ownership.as_dict()
        if preflight.get("status") != "PASS":
            raise RuntimeError(f"Preflight audit failed: {preflight}")
        print(f"Preflight Status: {preflight.get('status')} | Classification: {preflight.get('startup_classification')}")

        # Step 3: Canonical Initial HOME
        print("=== [Step 3] Canonical Initial HOME ===")
        home1 = e1d.run_exclusive_standalone_action("home", ownership, "initial_home")
        report["home_1_initial"] = home1.__dict__
        report["uart_ownership_audit"] = ownership.as_dict()
        if not home1.passed:
            raise RuntimeError(f"Initial standalone HOME failed: {home1.summary}")
        print(f"HOME 1 Passed: {home1.passed} | Final: {home1.q_final}")

        # Initialize ROS 2
        rclpy.init()
        geometry = load_geometry_config()
        z0 = float(geometry.nominal_height_m)

        # Step 4: Start Active ROS Pipeline 1 at HOME
        print("=== [Step 4] Starting Active ROS Pipeline 1 (Traversals: 100s nominal) ===")
        active_pipeline_1 = e1d.start_active_motion_pipeline(z0, ownership, "active_ros_phase_1")
        stale_audit_1 = getattr(active_pipeline_1.motor_node, "_e1d_startup_stale_target_audit", None)
        report["active_ros_1_startup"] = {
            "stale_target_audit": stale_audit_1,
            "uart_ownership_audit": ownership.as_dict(),
        }
        print(f"Active Pipeline 1 Started. Stale target audit: {stale_audit_1}")

        # Step 5: Run active pipeline for ~100s at safe HOME target (50Hz J7 traffic)
        print("=== [Step 5] Streaming HOME Target at 50Hz for 100.0s ===")
        t_start_1 = time.monotonic()
        target_count_1 = 0
        while time.monotonic() - t_start_1 < 100.0:
            publish_home_target(active_pipeline_1.target_node, active_pipeline_1.publisher, z0)
            target_count_1 += 1
            time.sleep(0.020)
        t_end_1 = time.monotonic()
        duration_1 = t_end_1 - t_start_1
        report["active_ros_1_stream"] = {
            "duration_s": duration_1,
            "target_published_count": target_count_1,
            "motor_cycle_seq": int(getattr(active_pipeline_1.motor_node, "_uart_cycle_seq", 0)),
        }
        print(f"Active Pipeline 1 Stream complete: {duration_1:.2f}s, {target_count_1} targets sent, {report['active_ros_1_stream']['motor_cycle_seq']} cycles.")

        # Step 6: Release ACTIVE_ROS 1 using exclusive handoff
        print("=== [Step 6] Releasing Active ROS Pipeline 1 ===")
        release_audit_1 = e1d.release_active_motion_pipeline(
            active_pipeline_1,
            ownership,
            "active_ros_phase_1_complete",
        )
        report["active_ros_1_release"] = release_audit_1
        report["uart_ownership_audit"] = ownership.as_dict()
        active_pipeline_1 = None
        print(f"Active Pipeline 1 Released: {release_audit_1}")
        if not release_audit_1.get("released"):
            raise RuntimeError(f"Active Pipeline 1 release failed: {release_audit_1}")

        # Step 7: Run standalone HOME once
        print("=== [Step 7] Running Standalone HOME Between Traversals ===")
        home2 = e1d.run_exclusive_standalone_action("home", ownership, "home_between_traversals")
        report["home_2_between"] = home2.__dict__
        report["uart_ownership_audit"] = ownership.as_dict()
        if not home2.passed:
            raise RuntimeError(f"Standalone HOME between traversals failed: {home2.summary}")
        print(f"HOME 2 Passed: {home2.passed} | Final: {home2.q_final}")

        # Step 8: Release standalone UART (handled automatically in finally block of run_exclusive_standalone_action)
        print(f"Ownership State after Standalone HOME: {ownership.state.value}")
        if ownership.state != e1d.UartOwnershipState.RELEASED:
            raise RuntimeError(f"Ownership not RELEASED after Standalone HOME: {ownership.state}")

        # Step 9 & 10: Restart ACTIVE_ROS cleanly at HOME
        print("=== [Step 9 & 10] Restarting Active ROS Pipeline 2 (Validation Phase) ===")
        active_pipeline_2 = e1d.start_active_motion_pipeline(z0, ownership, "active_ros_phase_2")
        stale_audit_2 = getattr(active_pipeline_2.motor_node, "_e1d_startup_stale_target_audit", None)
        report["active_ros_2_startup"] = {
            "stale_target_audit": stale_audit_2,
            "uart_ownership_audit": ownership.as_dict(),
        }
        print(f"Active Pipeline 2 Started. Stale target audit: {stale_audit_2}")

        # Step 11: Run restarted active pipeline briefly (~10s)
        print("=== [Step 11] Streaming HOME Target at 50Hz for 10.0s ===")
        t_start_2 = time.monotonic()
        target_count_2 = 0
        while time.monotonic() - t_start_2 < 10.0:
            publish_home_target(active_pipeline_2.target_node, active_pipeline_2.publisher, z0)
            target_count_2 += 1
            time.sleep(0.020)
        t_end_2 = time.monotonic()
        duration_2 = t_end_2 - t_start_2
        report["active_ros_2_stream"] = {
            "duration_s": duration_2,
            "target_published_count": target_count_2,
            "motor_cycle_seq": int(getattr(active_pipeline_2.motor_node, "_uart_cycle_seq", 0)),
        }
        print(f"Active Pipeline 2 Stream complete: {duration_2:.2f}s, {target_count_2} targets sent, {report['active_ros_2_stream']['motor_cycle_seq']} cycles.")

        # Step 12: Release ACTIVE_ROS 2 completely
        print("=== [Step 12] Releasing Active ROS Pipeline 2 ===")
        release_audit_2 = e1d.release_active_motion_pipeline(
            active_pipeline_2,
            ownership,
            "active_ros_phase_2_complete",
        )
        report["active_ros_2_release"] = release_audit_2
        report["uart_ownership_audit"] = ownership.as_dict()
        active_pipeline_2 = None
        print(f"Active Pipeline 2 Released: {release_audit_2}")
        if not release_audit_2.get("released"):
            raise RuntimeError(f"Active Pipeline 2 release failed: {release_audit_2}")

        # Step 13: Run canonical PARK once
        print("=== [Step 13] Running Canonical Final PARK ===")
        park = e1d.run_exclusive_standalone_action("park", ownership, "final_park")
        report["park_final"] = park.__dict__
        report["uart_ownership_audit"] = ownership.as_dict()
        print(f"PARK Passed: {park.passed} | Final: {park.q_final}")
        if not park.passed:
            raise RuntimeError(f"Final PARK failed: {park.summary}")

        # Step 14: Final READ_ONLY audit
        print("=== [Step 14] Final READ_ONLY Hardware Audit (4.0s) ===")
        final_audit = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        report["final_readonly_audit"] = final_audit
        report["uart_ownership_audit"] = ownership.as_dict()
        print(f"Final Audit Status: {final_audit.get('status')} | Classification: {final_audit.get('startup_classification')}")

        if final_audit.get("status") == "PASS" and final_audit.get("startup_classification") == "VALID_PARK":
            report["status"] = "RX_HANDOFF_QUALIFIED"
        else:
            report["status"] = "RX_HANDOFF_FAILED"

    except Exception as exc:
        report["status"] = "RX_HANDOFF_FAILED"
        report["error"] = f"{type(exc).__name__}: {exc}"
        print(f"ERROR encountered during qualification: {exc}")
    finally:
        # Cleanup any remaining active pipeline
        if active_pipeline_1 is not None:
            try:
                e1d.release_active_motion_pipeline(active_pipeline_1, ownership, "error_cleanup_pipeline_1")
            except Exception:
                pass
        if active_pipeline_2 is not None:
            try:
                e1d.release_active_motion_pipeline(active_pipeline_2, ownership, "error_cleanup_pipeline_2")
            except Exception:
                pass

        # If not parked, attempt emergency PARK
        if report.get("park_final") is None and ownership.state == e1d.UartOwnershipState.RELEASED:
            try:
                print("Emergency cleanup: attempting standalone PARK...")
                emergency_park = e1d.run_exclusive_standalone_action("park", ownership, "emergency_cleanup_park")
                report["emergency_park"] = emergency_park.__dict__
            except Exception as e_park:
                report["emergency_park_error"] = str(e_park)

        if report.get("final_readonly_audit") is None:
            try:
                report["final_readonly_audit"] = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        report["uart_ownership_audit"] = ownership.as_dict()
        raw_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Qualification complete. Status: {report['status']}. Report saved to {raw_path}")

    return 0 if report["status"] == "RX_HANDOFF_QUALIFIED" else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "reports" / "j9_e1d_rx2b_handoff_qualification_20260829",
    )
    parser.add_argument("--real-hardware", action="store_true", required=True)
    parser.add_argument("--confirm-motion", action="store_true", required=True)
    args = parser.parse_args()
    return run_qualification(args)


if __name__ == "__main__":
    raise SystemExit(main())
