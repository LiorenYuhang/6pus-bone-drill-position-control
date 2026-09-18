#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J11-M2: Graceful Stopper for Dual-IMU Feedforward + PID Residual Runner."""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


def stop_m2_session(output_dir: Optional[str] = None, stage: str = "j11_m2_1_roll_p"):
    print("=" * 60)
    print("[J11-M2 STOPPER] Initiating graceful session termination...")
    print("=" * 60, flush=True)

    if output_dir is None or output_dir == "results/j11/m2_pid":
        if Path("/tmp/j11_m2_latest_run_dir").exists():
            try:
                latest_p = Path("/tmp/j11_m2_latest_run_dir").read_text().strip()
                if latest_p and Path(latest_p).exists():
                    output_dir = latest_p
            except OSError:
                pass
        elif Path("/tmp/j11_latest_run_dir").exists():
            try:
                latest_p = Path("/tmp/j11_latest_run_dir").read_text().strip()
                if latest_p and Path(latest_p).exists():
                    output_dir = latest_p
            except OSError:
                pass

    if output_dir is None:
        output_dir = "results/j11/m2_pid"

    stop_marker = Path("/tmp/j11_stop_requested")
    stop_marker.touch()
    print("[J11-M2 STOPPER] Touched /tmp/j11_stop_requested.", flush=True)

    t0 = time.monotonic()
    runner_exited = False
    while time.monotonic() - t0 < 6.0:
        res = subprocess.run(["pgrep", "-f", "j11_m2_pid_residual_runner.py"], stdout=subprocess.PIPE)
        if not res.stdout.strip():
            runner_exited = True
            break
        time.sleep(0.2)

    if runner_exited:
        print("[J11-M2 STOPPER] M2 runner process has stopped gracefully.", flush=True)
    else:
        print("[J11-M2 STOPPER] Warning: M2 runner still present after 6s. Requesting SIGINT...", flush=True)
        subprocess.run(["pkill", "-INT", "-f", "j11_m2_pid_residual_runner.py"], stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    for m in ["/tmp/j11_active_signaled", "/tmp/j11_ready_for_start", "/tmp/j11_stop_requested"]:
        p = Path(m)
        if p.exists():
            try:
                p.unlink()
            except OSError:
                pass

    summary_path = Path(output_dir) / f"{stage}_summary_powered.json"
    if not summary_path.exists():
        json_files = list(Path(output_dir).glob("*summary*.json"))
        if json_files:
            summary_path = sorted(json_files, key=os.path.getmtime)[-1]

    if summary_path.exists():
        try:
            with open(summary_path, "r") as f:
                data = json.load(f)
            print("\n" + "=" * 60)
            print("[J11-M2 SESSION REPORT]")
            print(f"Stage:                {data.get('stage')}")
            print(f"Mode:                 {data.get('mode')}")
            print(f"Total Cycles:         {data.get('total_cycles')} ({data.get('total_cycles', 0) * 0.02:.2f}s)")
            print(f"Hardware Writes:      {data.get('hardware_write_count')}")
            print(f"Final State:          {data.get('final_state')}")
            print(f"Fault Reason:         {data.get('fault_reason') or 'None (Clean Exit)'}")
            print("=" * 60 + "\n", flush=True)
        except Exception as exc:
            print(f"[J11-M2 STOPPER] Could not parse summary: {exc}")
    else:
        print(f"[J11-M2 STOPPER] Summary file not found in {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="J11-M2 Graceful Stopper")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--stage", default="j11_m2_1_roll_p")
    args = parser.parse_args()

    stop_m2_session(output_dir=args.output_dir, stage=args.stage)


if __name__ == "__main__":
    main()
