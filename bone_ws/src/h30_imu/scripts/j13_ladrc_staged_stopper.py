#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage J13-2A2: Graceful Stopper for LADRC Shadow Runner."""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


def stop_j13_session(output_dir: Optional[str] = None, stage: str = "j13_2a2_static_shadow") -> None:
    print("=" * 60)
    print("[J13 STOPPER] Initiating graceful session termination...")
    print("=" * 60, flush=True)

    if output_dir is None:
        if Path("/tmp/j13_latest_run_dir").exists():
            try:
                latest_p = Path("/tmp/j13_latest_run_dir").read_text().strip()
                if latest_p and Path(latest_p).exists():
                    output_dir = latest_p
            except OSError:
                pass

    if output_dir is None:
        output_dir = "results/j13/shadow_mode"

    stop_marker = Path("/tmp/j13_stop_requested")
    stop_marker.touch()
    print("[J13 STOPPER] Touched /tmp/j13_stop_requested.", flush=True)

    t0 = time.monotonic()
    runner_exited = False
    while time.monotonic() - t0 < 6.0:
        res = subprocess.run(
            ["pgrep", "-f", "j13_ladrc_shadow_runner.py"], stdout=subprocess.PIPE
        )
        if not res.stdout.strip():
            runner_exited = True
            break
        time.sleep(0.2)

    if runner_exited:
        print("[J13 STOPPER] J13 runner process has stopped gracefully.", flush=True)
    else:
        print(
            "[J13 STOPPER] Warning: J13 runner still present after 6s. Requesting SIGINT...",
            flush=True,
        )
        subprocess.run(
            ["pkill", "-INT", "-f", "j13_ladrc_shadow_runner.py"], stderr=subprocess.DEVNULL
        )
        time.sleep(1.0)

    for m in ["/tmp/j13_active_signaled", "/tmp/j13_ready_for_start", "/tmp/j13_stop_requested"]:
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
            print("[J13 SESSION REPORT]")
            print(f"Stage:                {data.get('stage')}")
            print(f"Mode:                 {data.get('mode')}")
            print(
                f"Total Cycles:         {data.get('total_cycles')} ({data.get('total_cycles', 0) * 0.02:.2f}s)"
            )
            print(f"Hardware Writes:      {data.get('hardware_write_count')}")
            print(f"Final State:          {data.get('final_state')}")
            print(f"Fault Reason:         {data.get('fault_reason') or 'None (Clean Exit)'}")
            print("=" * 60 + "\n", flush=True)
        except Exception as exc:
            print(f"[J13 STOPPER] Could not parse summary: {exc}")
    else:
        print(f"[J13 STOPPER] Summary file not found in {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="J13 Graceful Stopper")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--stage", default="j13_2a2_static_shadow")
    args = parser.parse_args()

    stop_j13_session(output_dir=args.output_dir, stage=args.stage)


if __name__ == "__main__":
    main()
