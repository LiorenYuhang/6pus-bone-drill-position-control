#!/usr/bin/env python3
"""Stage J11: Graceful Stopper for Base-IMU Feedforward Attitude Hold.

Performs:
1. Touch /tmp/j11_stop_requested to signal graceful stop.
2. Wait for runner process to exit and save telemetry files.
3. Clean active marker.
4. Does NOT automatically trigger STORAGE_PARK.
5. Reads and displays final summary and six-axis motor feedback.
6. Provides re-homing prompt if user desires another take.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


def stop_j11_session(output_dir: Optional[str] = None, stage: str = "j11_video_manual"):
    print("=" * 60)
    print("[J11 STOPPER] Initiating graceful session termination...")
    print("=" * 60, flush=True)

    # 0. Check for latest run directory pointer
    if output_dir is None or output_dir == "results/j11/j11_video_manual":
        if Path("/tmp/j11_latest_run_dir").exists():
            try:
                latest_p = Path("/tmp/j11_latest_run_dir").read_text().strip()
                if latest_p and Path(latest_p).exists():
                    output_dir = latest_p
            except OSError:
                pass
    if output_dir is None:
        output_dir = "results/j11/j11_video_manual"

    # 1. Request stop via marker
    stop_marker = Path("/tmp/j11_stop_requested")
    stop_marker.touch()
    print("[J11 STOPPER] Touched /tmp/j11_stop_requested.", flush=True)

    # 2. Wait up to 6s for runner to exit
    t0 = time.monotonic()
    runner_exited = False
    while time.monotonic() - t0 < 6.0:
        # Check if j11_base_feedforward_runner process is gone
        res = subprocess.run(["pgrep", "-f", "j11_base_feedforward_runner.py"], stdout=subprocess.PIPE)
        if not res.stdout.strip():
            runner_exited = True
            break
        time.sleep(0.2)

    if runner_exited:
        print("[J11 STOPPER] Runner process has stopped gracefully.", flush=True)
    else:
        print("[J11 STOPPER] Warning: Runner process still present after 6s. Requesting SIGINT...", flush=True)
        subprocess.run(["pkill", "-INT", "-f", "j11_base_feedforward_runner.py"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    # 3. Clean active marker
    for m in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_stop_requested"]:
        p = Path(m)
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass

    # 4. Read telemetry summary
    summary_path = Path(output_dir) / f"{stage}_feedforward_summary.json"
    if not summary_path.exists():
        # Fallback to general summary in output_dir
        json_files = list(Path(output_dir).glob("*summary.json"))
        if json_files:
            summary_path = sorted(json_files, key=os.path.getmtime)[-1]

    if summary_path.exists():
        try:
            with open(summary_path, "r") as f:
                data = json.load(f)
            print("\n" + "=" * 60)
            print("[J11 SESSION REPORT]")
            print(f"Stage:                {data.get('stage')}")
            print(f"Verdict:              {data.get('verdict')}")
            print(f"Total Cycles:         {data.get('total_cycles')}")
            print(f"Active Cycles:        {data.get('active_cycles')} ({data.get('active_cycles', 0) * 0.02:.2f}s)")
            print(f"Hardware Writes:      {data.get('hardware_write_count')}")
            print(f"Final Fault Reason:   {data.get('final_fault_reason') or 'None (Clean Exit)'}")
            
            fb_ranges = data.get("motor_feedback_ranges", {})
            print(f"Motor Feedback Min:   {fb_ranges.get('min_steps_axis_1_to_6')}")
            print(f"Motor Feedback Max:   {fb_ranges.get('max_steps_axis_1_to_6')}")

            curr_metrics = data.get("joint_currents_metrics_mA", {})
            if curr_metrics:
                print(f"Peak Current (All):   {curr_metrics.get('overall_max_current_mA')} mA")
                print(f"Peak Current Axes:    {curr_metrics.get('max_current_per_axis_mA')} mA")
                print(f"Mean Current Axes:    {curr_metrics.get('mean_current_per_axis_mA')} mA")

            slew_info = data.get("slew_recovery_summary", {})
            if slew_info:
                print(f"Slew Recovery Events: {slew_info.get('total_slew_events')}")

            fault_audit = data.get("fault_window_audit_last_2s")
            if fault_audit:
                print("\n[FAULT AUDIT - LAST 2 SECONDS]")
                print(f"Max Feedback Age:     {fault_audit.get('feedback_age_max_ms')} ms (Stall: {fault_audit.get('feedback_age_stall_confirmed')})")
                print(f"Peak Current:         {fault_audit.get('peak_current_mA')} mA (Axis {fault_audit.get('peak_current_axis')})")
                print(f"Max Cmd-FB Errors:    {fault_audit.get('max_command_feedback_error_steps')} steps")
                print(f"Slew Events in Win:   {fault_audit.get('window_slew_event_count')}")
                eval_map = fault_audit.get("causal_chain_evaluation", {})
                for k, v in eval_map.items():
                    print(f"  {k}: {v}")
            
            plat_err = data.get("platform_world_attitude_errors", {})
            if plat_err and data.get("active_cycles", 0) > 0:
                print(f"\nPlatform SO(3) Max:   {plat_err.get('so3_error_max_deg', 0.0):.2f}°")
                print(f"Platform Tilt RMS:    {plat_err.get('tilt_error_rms_deg', 0.0):.2f}°")
            print("=" * 60)
            print(f"Summary JSON: {summary_path}")
        except Exception as exc:
            print(f"[J11 STOPPER] Could not parse summary JSON: {exc}")
    else:
        print(f"[J11 STOPPER] No summary JSON found at {summary_path}.")

    print("\n[ROBOT STATE] Robot is resting safely at current position (NO auto STORAGE_PARK).")
    print("If you wish to return to OPERATING_HOME [1000]*6 before the next take, run:")
    print("  python3 bone_ws/scripts/run_j7_servo_home_park.py --action home --real-hardware\n")


def main():
    parser = argparse.ArgumentParser(description="J11 Graceful Session Stopper")
    parser.add_argument("--stage", default="j11_video_manual")
    parser.add_argument("--output-dir", default=None, help="Directory of results (defaults to /tmp/j11_latest_run_dir)")
    args = parser.parse_args()

    stop_j11_session(output_dir=args.output_dir, stage=args.stage)


if __name__ == "__main__":
    main()
