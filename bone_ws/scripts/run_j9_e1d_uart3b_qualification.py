#!/usr/bin/env python3
"""J9-E1D-UART3B Targeted Standalone Design-B Hardware Qualification Runner.

Executes:
1. READ_ONLY startup audit (4.0s)
2. Canonical standalone HOME with Design-B 0x32 feedback ingestion + 100.0s hold (~5938 cycles @ 50Hz)
3. Canonical standalone PARK (18.75s transition + 2.0s hold = ~1038 cycles @ 50Hz)
4. Final READ_ONLY audit (4.0s)
5. Captures full passive TX provenance on any write failure with deadline accounting
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

REPO_ROOT = Path(__file__).resolve().parents[4] if "src" in str(Path(__file__)) else Path("/home/lyapunov/bone_drill")
WS_ROOT = REPO_ROOT / "bone_ws"
for pkg in ("pus6_motor_driver", "pus6_kinematics", "pus6_control", "pus6_msgs", "h30_imu"):
    path = WS_ROOT / "src" / pkg
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))
scripts_dir = WS_ROOT / "scripts"
if scripts_dir.exists() and str(scripts_dir) not in sys.path:
    sys.path.insert(0, str(scripts_dir))

from run_j7_servo_home_park import run_servo_home_park
from run_j8_c_hardware_readonly_audit import run_j8_c_audit

MOTOR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_MOTOR = 921600


def run_qualification(args: argparse.Namespace) -> int:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.out_dir / "uart3b_raw.json"

    report: dict[str, Any] = {
        "stage": "J9-E1D-UART3B",
        "description": "Targeted Standalone Design-B Hardware Qualification (write_timeout=0.015s)",
        "robot_extrinsics_status": "UNCALIBRATED_UNSET",
        "hardware_write_count": 0,
        "write_timeout_config_s": 0.015,
        "design_architecture": "Design-B (0x32 StatusFeedback direct ingestion, zero motion 0x30 poll)",
        "preflight_audit": None,
        "home_stream_100s": None,
        "park_final": None,
        "emergency_park": None,
        "final_readonly_audit": None,
        "tx_failure_provenance": None,
        "status": "INITIALIZING",
        "error": None,
    }

    try:
        # Step 1: Preflight READ_ONLY audit
        print("=== [Step 1] Preflight READ_ONLY Audit (4.0s) ===")
        preflight = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        report["preflight_audit"] = preflight
        if preflight.get("status") != "PASS":
            raise RuntimeError(f"Preflight audit failed: {preflight}")
        print(f"Preflight Status: {preflight.get('status')}")

        # Step 2: Canonical Standalone HOME with 100s hold
        print("=== [Step 2] Standalone HOME + 100s Continuous Hold (~5938 cycles @ 50Hz, Design-B) ===")
        home_res = run_servo_home_park(
            action="home",
            port=MOTOR_PORT,
            baudrate=BAUD_MOTOR,
            speed=100.0,
            rate_hz=50.0,
            hold_duration_s=100.0,
            real_hardware=True,
            confirm_motion=True,
        )
        report["home_stream_100s"] = home_res.__dict__
        if home_res.last_tx_failure_provenance is not None:
            report["tx_failure_provenance"] = home_res.last_tx_failure_provenance
        if not home_res.passed:
            raise RuntimeError(f"Standalone HOME 100s stream failed: {home_res.summary}")
        print(f"HOME 100s Stream Passed: {home_res.passed} | Total Cycles: {home_res.total_cycles}")

        # Step 3: Canonical Standalone PARK
        print("=== [Step 3] Canonical Standalone PARK (Design-B) ===")
        park_res = run_servo_home_park(
            action="park",
            port=MOTOR_PORT,
            baudrate=BAUD_MOTOR,
            speed=100.0,
            rate_hz=50.0,
            hold_duration_s=2.0,
            real_hardware=True,
            confirm_motion=True,
        )
        report["park_final"] = park_res.__dict__
        if park_res.last_tx_failure_provenance is not None and report["tx_failure_provenance"] is None:
            report["tx_failure_provenance"] = park_res.last_tx_failure_provenance
        if not park_res.passed:
            raise RuntimeError(f"Standalone PARK failed: {park_res.summary}")
        print(f"PARK Passed: {park_res.passed} | Total Cycles: {park_res.total_cycles}")

        # Step 4: Final READ_ONLY audit
        print("=== [Step 4] Final READ_ONLY Audit (4.0s) ===")
        final_audit = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
        report["final_readonly_audit"] = final_audit
        print(f"Final READ_ONLY Status: {final_audit.get('status')}")

        if final_audit.get("classification") != "VALID_PARK":
            raise RuntimeError(f"Final position not VALID_PARK: {final_audit.get('classification')}")

        report["status"] = "PASS"
        report["hardware_write_count"] = 1
        print("\n>>> J9-E1D-UART3B Targeted Qualification PASSED <<<")
        return 0

    except Exception as exc:
        print(f"\n[FAILURE] Exception during UART3B qualification: {exc}")
        report["status"] = "FAIL"
        report["error"] = str(exc)

        # Attempt Emergency PARK if HOME failed or interrupted
        try:
            print("\n=== [Recovery] Executing Emergency Standalone PARK ===")
            rec_park = run_servo_home_park(
                action="park",
                port=MOTOR_PORT,
                baudrate=BAUD_MOTOR,
                speed=50.0,
                rate_hz=50.0,
                hold_duration_s=1.0,
                real_hardware=True,
                confirm_motion=True,
            )
            report["emergency_park"] = rec_park.__dict__
            print(f"Emergency PARK result: {rec_park.passed}")
        except Exception as p_exc:
            print(f"Emergency PARK error: {p_exc}")

        # Final audit attempt
        try:
            print("\n=== [Recovery] Final Post-Failure READ_ONLY Audit ===")
            rec_audit = run_j8_c_audit(port=MOTOR_PORT, baudrate=BAUD_MOTOR, duration_s=4.0)
            report["final_readonly_audit"] = rec_audit
            print(f"Post-Failure Status: {rec_audit.get('status')}")
        except Exception as a_exc:
            print(f"Post-Failure audit error: {a_exc}")

        return 1

    finally:
        with open(raw_path, "w") as f:
            json.dump(report, f, indent=2)
        print(f"\nSaved raw qualification report to {raw_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="J9-E1D-UART3B Targeted Standalone Hardware Qualification")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "reports" / f"j9_e1d_uart3b_qualification_{time.strftime('%Y%m%d')}")
    parser.add_argument("--real-hardware", action="store_true", help="Authorize single real-hardware invocation")
    parser.add_argument("--confirm-motion", action="store_true", help="Confirm real motor motion")
    args = parser.parse_args()

    if not args.real_hardware or not args.confirm_motion:
        print("ERROR: --real-hardware and --confirm-motion required.")
        return 2

    return run_qualification(args)


if __name__ == "__main__":
    sys.exit(main())
