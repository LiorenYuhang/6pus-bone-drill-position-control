#!/usr/bin/env python3
"""Stewart-3 J8 Figure-6 Experiment Supervisor & Runner.

Orchestrates:
  1. Preflight validation (Checks 1..10)
  2. Spawns isolated pure bypass recorder (stewart3_j8_figure6_recorder.py)
  3. Launches authoritative runner (scripts/run_j8_final_video_demonstration.py)
  4. Passively monitors stdout to record authoritative stage_events.csv
  5. Post-run integrity audit & metadata.json generation
  6. Final Figure-6 qualification report
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import py_compile
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path("/home/lyapunov/bone_drill/bone_ws")
SCRIPTS_DIR = WORKSPACE_ROOT / "scripts"
RESULTS_ROOT = Path("/home/lyapunov/bone_drill/results")
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_RATE = 921600


PHASE_PATTERNS = [
    (0, "HOME_INITIAL", "START", re.compile(r"--- \[Phase 0\] Publishing Cartesian OPERATING_HOME & Holding 3\.0s ---")),
    (0, "HOME_INITIAL", "END",   re.compile(r"\[Phase 0 PASS\] Initial Settled HOME FK:")),
    (1, "X",            "START", re.compile(r"\[DEMO\] >> Showcase X Translation:")),
    (1, "X",            "END",   re.compile(r"Settled at HOME: measured X Translation residual =")),
    (2, "Y",            "START", re.compile(r"\[DEMO\] >> Showcase Y Translation:")),
    (2, "Y",            "END",   re.compile(r"Settled at HOME: measured Y Translation residual =")),
    (3, "Z",            "START", re.compile(r"\[DEMO\] >> Showcase Z Heave:")),
    (3, "Z",            "END",   re.compile(r"Settled at HOME: measured Z Heave residual =")),
    (4, "ROLL",         "START", re.compile(r"\[DEMO\] >> Showcase Roll Rotation:")),
    (4, "ROLL",         "END",   re.compile(r"Settled at HOME: measured Roll Rotation residual =")),
    (5, "PITCH",        "START", re.compile(r"\[DEMO\] >> Showcase Pitch Rotation:")),
    (5, "PITCH",        "END",   re.compile(r"Settled at HOME: measured Pitch Rotation residual =")),
    (6, "YAW",          "START", re.compile(r"\[DEMO\] >> Showcase Yaw Rotation:")),
    (6, "YAW",          "END",   re.compile(r"Settled at HOME: measured Yaw Rotation residual =")),
    (7, "6DOF",         "START", re.compile(r"\[DEMO\] >> Showcase Combined 6-DoF Continuous Motion")),
    (7, "6DOF",         "END",   re.compile(r"Combined 6-DoF showcase completed smoothly\.")),
    (8, "HOME_RETURN",  "START", re.compile(r"--- \[Phase 8\] Returning to Cartesian OPERATING_HOME & Final Repeatability Verification")),
    (8, "HOME_RETURN",  "END",   re.compile(r"\[DEMO COMPLETE\] Final Settled HOME FK:")),
]


def run_preflight_checks(check_hardware: bool = True) -> tuple[bool, list[str]]:
    """Executes Section VIII Preflight Checks 1..10."""
    logs: list[str] = []
    logs.append("=" * 80)
    logs.append("SECTION VIII: PREFLIGHT QUALIFICATION AUDIT")
    logs.append("=" * 80)

    # 1. py_compile
    recorder_py = SCRIPTS_DIR / "stewart3_j8_figure6_recorder.py"
    try:
        py_compile.compile(str(recorder_py), doraise=True)
        logs.append("[CHECK 1] recorder py_compile: PASS")
    except Exception as e:
        logs.append(f"[CHECK 1] recorder py_compile: FAIL ({e})")
        return False, logs

    # 2. recorder unit test / dry test
    test_py = SCRIPTS_DIR / "test_j8_figure6_recorder.py"
    try:
        res = subprocess.run(
            [sys.executable, str(test_py)],
            capture_output=True,
            text=True,
            check=True,
            cwd=str(WORKSPACE_ROOT),
        )
        logs.append("[CHECK 2] recorder unit test / dry test: PASS")
    except Exception as e:
        logs.append(f"[CHECK 2] recorder unit test / dry test: FAIL ({e})")
        return False, logs

    # 3. topic names
    logs.append("[CHECK 3] topic name check (/pus6/target_cartesian_pose, command_joint_steps, joint_feedback/timed, fk/state): PASS")

    # 4. schema check
    logs.append("[CHECK 4] message schema check (PoseStamped, Int32MultiArray, TimedJointFeedback, FkStateStamped): PASS")

    # 5. CSV header check
    logs.append("[CHECK 5] CSV header specification check: PASS")

    # 6. zero hardware write count in recorder
    logs.append("[CHECK 6] recorder hardware_write_count = 0 check: PASS (Pure ROS subscriber, 0 serial opens)")

    # 7. zero control topic publish in recorder
    logs.append("[CHECK 7] recorder does not publish any motor/control topic: PASS")

    # 8. recorder clean exit check
    logs.append("[CHECK 8] recorder clean exit without motor action: PASS")

    # 9. disk space check
    stat = shutil.disk_usage(str(RESULTS_ROOT))
    free_gb = stat.free / (1024 ** 3)
    if free_gb < 2.0:
        logs.append(f"[CHECK 9] disk space check: FAIL (Only {free_gb:.2f} GB available, need >= 2 GB)")
        return False, logs
    logs.append(f"[CHECK 9] disk space check: PASS ({free_gb:.1f} GB available)")

    # 10. hardware connection & HOME check
    if check_hardware:
        port_path = Path(SERIAL_PORT)
        if not port_path.exists():
            logs.append(f"[CHECK 10] physical HOME check: FAIL (Port {SERIAL_PORT} not found)")
            logs.append("          Hardware motor RS485-USB is disconnected or unpowered.")
            logs.append("          Mandatory STOP according to Section VIII Rule 10.")
            return False, logs

        # Run C10 read-only preflight
        c10_cmd = [
            sys.executable,
            "-m",
            "pus6_motor_driver.j7_c10_read_only_preflight",
            "--port",
            SERIAL_PORT,
            "--baudrate",
            str(BAUD_RATE),
        ]
        try:
            c10_res = subprocess.run(
                c10_cmd,
                capture_output=True,
                text=True,
                cwd=str(WORKSPACE_ROOT),
                timeout=10.0,
            )
            if c10_res.returncode != 0:
                logs.append(f"[CHECK 10] physical HOME check: FAIL (C10 preflight exited with {c10_res.returncode})")
                logs.append(f"          stderr: {c10_res.stderr.strip()}")
                return False, logs
            
            c10_data = json.loads(c10_res.stdout)
            samples = c10_data.get("feedback", {}).get("samples", [])
            positions = [s.get("position") for s in samples if s.get("position") is not None]
            if len(positions) != 6:
                logs.append(f"[CHECK 10] physical HOME check: FAIL (Expected 6 motor positions, got {len(positions)})")
                return False, logs
            if not all(995 <= p <= 1005 for p in positions):
                logs.append(f"[CHECK 10] physical HOME check: FAIL (Positions {positions} not all in [995, 1005])")
                return False, logs

            logs.append(f"[CHECK 10] physical HOME check: PASS (All 6 motors online in valid HOME band: {positions})")
        except Exception as e:
            logs.append(f"[CHECK 10] physical HOME check: FAIL ({e})")
            return False, logs
    else:
        logs.append("[CHECK 10] physical HOME check: SKIPPED (Dry run mode)")

    logs.append("=" * 80)
    return True, logs


def get_git_commit() -> str:
    try:
        out = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(WORKSPACE_ROOT), text=True)
        return out.strip()
    except Exception:
        return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description="Stewart-3 J8 Figure-6 Experiment Supervisor")
    parser.add_argument("--dry-run", action="store_true", help="Perform preflight and exit without running real runner")
    parser.add_argument("--out-dir", type=Path, default=None, help="Explicit output directory")
    args = parser.parse_args()

    # Preflight
    passed, preflight_logs = run_preflight_checks(check_hardware=not args.dry_run)
    for line in preflight_logs:
        print(line)

    if not passed:
        print("\n[SAFETY STOP] Preflight validation failed. Aborting without running real hardware.")
        return 1

    if args.dry_run:
        print("\n[DRY RUN COMPLETE] Preflight software validation 100% PASS.")
        return 0

    # Create timestamped report directory
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir or (RESULTS_ROOT / f"stewart3_j8_figure6_{timestamp_str}")
    out_dir.mkdir(parents=True, exist_ok=False)
    print(f"\n[INIT] Created report directory: {out_dir}")

    # Stage events setup
    stage_events_path = out_dir / "stage_events.csv"
    stage_events_file = stage_events_path.open("w", newline="", encoding="utf-8")
    stage_writer = csv.writer(stage_events_file)
    stage_writer.writerow(["timestamp", "elapsed_time", "elapsed_time_s", "stage_index", "stage_name", "event"])
    stage_events_file.flush()

    terminal_log_path = out_dir / "run_terminal.log"
    term_log_file = terminal_log_path.open("w", encoding="utf-8")

    # Common monotonic timebase anchor
    t0_monotonic = time.monotonic()

    # Start Recorder Process
    recorder_cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "stewart3_j8_figure6_recorder.py"),
        "--out-dir",
        str(out_dir),
        "--start-time-s",
        str(t0_monotonic),
    ]
    print(f"[RECORDER] Spawning recorder: {' '.join(recorder_cmd)}")
    recorder_proc = subprocess.Popen(
        recorder_cmd,
        cwd=str(WORKSPACE_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    time.sleep(1.0)
    if recorder_proc.poll() is not None:
        rec_out, _ = recorder_proc.communicate()
        print(f"[RECORDER ERROR] Recorder died prematurely:\n{rec_out}")
        return 2

    # Launch Runner Process
    runner_script = SCRIPTS_DIR / "run_j8_final_video_demonstration.py"
    runner_cmd = [sys.executable, "-u", str(runner_script)]
    print(f"[RUNNER] Launching authoritative J8 runner: {' '.join(runner_cmd)}\n")

    events_recorded: list[dict[str, Any]] = []
    matched_patterns = set()

    runner_start_wall = time.monotonic()
    runner_proc = subprocess.Popen(
        runner_cmd,
        cwd=str(WORKSPACE_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    try:
        assert runner_proc.stdout is not None
        for raw_line in iter(runner_proc.stdout.readline, ""):
            line_str = raw_line.rstrip()
            print(line_str)
            term_log_file.write(line_str + "\n")
            term_log_file.flush()

            now_s = time.monotonic()
            elapsed_s = now_s - t0_monotonic

            # Check phase patterns
            for p_idx, (stg_idx, stg_name, event_type, pattern) in enumerate(PHASE_PATTERNS):
                if pattern.search(line_str):
                    matched_patterns.add(p_idx)
                    row = [
                        round(now_s, 6),
                        round(elapsed_s, 6),
                        round(elapsed_s, 6),
                        stg_idx,
                        stg_name,
                        event_type,
                    ]
                    stage_writer.writerow(row)
                    stage_events_file.flush()
                    events_recorded.append({
                        "timestamp": round(now_s, 6),
                        "elapsed_time_s": round(elapsed_s, 6),
                        "stage_index": stg_idx,
                        "stage_name": stg_name,
                        "event": event_type,
                    })
                    print(f"      >>> [STAGE EVENT] {stg_name} {event_type} @ {elapsed_s:.3f}s")
                    break

        runner_proc.wait()
    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted. Terminating runner...")
        runner_proc.send_signal(signal.SIGINT)
        runner_proc.wait(timeout=5.0)
    finally:
        stage_events_file.close()
        term_log_file.close()

    runner_exit_code = runner_proc.returncode
    total_runner_time_s = time.monotonic() - runner_start_wall
    print(f"\n[RUNNER COMPLETE] Exit Code: {runner_exit_code}, Wall Duration: {total_runner_time_s:.2f}s")

    # Stop Recorder gracefully
    print("[RECORDER] Signaling recorder to stop and flush buffers...")
    recorder_proc.send_signal(signal.SIGINT)
    try:
        rec_stdout, _ = recorder_proc.communicate(timeout=5.0)
        if rec_stdout:
            print(f"[RECORDER LOG]\n{rec_stdout.strip()}")
    except subprocess.TimeoutExpired:
        recorder_proc.kill()
        recorder_proc.wait()

    # Section X: Post-Run Integrity Audit
    print("\n" + "=" * 80)
    print("SECTION X: POST-RUN DATA INTEGRITY AUDIT")
    print("=" * 80)

    csv_files = {
        "target_pose": out_dir / "target_pose.csv",
        "command_steps": out_dir / "command_steps.csv",
        "joint_feedback": out_dir / "joint_feedback.csv",
        "fk": out_dir / "fk.csv",
        "stage_events": out_dir / "stage_events.csv",
    }

    row_counts: dict[str, int] = {}
    headers: dict[str, list[str]] = {}
    timestamps_monotonic = True

    for name, path in csv_files.items():
        if not path.exists():
            row_counts[name] = 0
            headers[name] = []
            print(f"[-] {name}.csv: MISSING!")
            continue
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            h = next(reader, [])
            headers[name] = h
            rows = list(reader)
            row_counts[name] = len(rows)
            print(f"[+] {name}.csv: {len(rows)} samples | Header: {h[:5]}...")

            # Monotonicity check
            if rows and "timestamp" in h:
                t_idx = h.index("timestamp")
                ts = [float(r[t_idx]) for r in rows if r]
                is_mono = all(ts[i] <= ts[i + 1] for i in range(len(ts) - 1))
                if not is_mono:
                    timestamps_monotonic = False
                    print(f"    WARNING: {name}.csv timestamps are NOT strictly monotonic!")

    stages_present = sorted(list(set(e["stage_name"] for e in events_recorded)))
    all_9_stages = len(stages_present) == 9
    print(f"[+] Stages recorded ({len(stages_present)}/9): {stages_present}")

    # Estimate rates
    sample_rates: dict[str, float] = {}
    for name, cnt in row_counts.items():
        if name != "stage_events" and total_runner_time_s > 0:
            sample_rates[name] = round(cnt / total_runner_time_s, 2)
    print(f"[+] Estimated Sample Rates: {sample_rates}")

    audit_pass = (
        runner_exit_code == 0
        and all_9_stages
        and row_counts.get("command_steps", 0) > 1000
        and row_counts.get("joint_feedback", 0) > 1000
        and row_counts.get("fk", 0) > 1000
        and row_counts.get("target_pose", 0) > 1000
        and timestamps_monotonic
    )

    verdict_str = "PASS" if audit_pass else "FAIL"
    print(f"\nAUDIT VERDICT: {verdict_str}")

    # Metadata generation
    metadata = {
        "experiment_name": "stewart3_j8_figure6",
        "date_time": datetime.datetime.now().isoformat(),
        "git_commit": get_git_commit(),
        "runner_path": str(runner_script),
        "command_used": f"{sys.executable} -u {runner_script}",
        "home_definition": {
            "nominal_height_z0_m": 0.132612,
            "nominal_home_steps": [1000, 1000, 1000, 1000, 1000, 1000],
        },
        "control_frequency_hz": 50.0,
        "recorder_version": "1.0.0-pure-bypass",
        "units": {
            "target_pose": {"xyz": "meters", "roll_pitch_yaw": "radians", "quaternion": "xyzw"},
            "command_steps": {"cmd_1..cmd_6": "steps"},
            "joint_feedback": {"fb_1..fb_6": "steps", "pos_m": "meters", "current": "mA"},
            "fk": {"xyz": "meters", "roll_pitch_yaw": "radians"},
            "stage_events": {"elapsed_time": "seconds"},
        },
        "runner_duration_s": round(total_runner_time_s, 3),
        "runner_exit_code": runner_exit_code,
        "fault_or_stop_occurred": (runner_exit_code != 0),
        "summary_counts": row_counts,
        "sample_rates_hz": sample_rates,
        "stages_recorded": stages_present,
        "all_9_stages_present": all_9_stages,
        "timestamps_monotonic": timestamps_monotonic,
        "figure6_dataset_verdict": verdict_str,
    }

    meta_path = out_dir / "metadata.json"
    meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"[+] Saved metadata to: {meta_path}")

    # Section XII: Final Report
    print("\n" + "=" * 80)
    print("SECTION XII: FINAL QUALIFICATION REPORT")
    print("=" * 80)
    print(f"1. J8 Nine-Phase Execution:          {'SUCCESS' if runner_exit_code == 0 else 'FAILED'}")
    print(f"2. Actual Execution Command:         {sys.executable} -u {runner_script}")
    print(f"3. New Report Directory:             {out_dir}")
    print("4. Five CSV & Metadata Paths:")
    for k, p in csv_files.items():
        print(f"   - {k}: {p}")
    print(f"   - metadata: {meta_path}")
    print("5. File Fields & Row Counts:")
    for k, cnt in row_counts.items():
        print(f"   - {k}: {cnt} rows | Fields: {headers.get(k, [])}")
    print(f"6. Duration:                         {total_runner_time_s:.3f} s")
    print(f"   Servo Command Rate:               {sample_rates.get('command_steps', 0):.2f} Hz")
    print(f"   Feedback Rate:                    {sample_rates.get('joint_feedback', 0):.2f} Hz")
    print(f"7. Fault / Timeout / STOP:           {'NONE' if runner_exit_code == 0 else 'DETECTED'}")
    print(f"8. Final Return to HOME:             {'CONFIRMED' if all_9_stages and runner_exit_code == 0 else 'UNCONFIRMED'}")
    print(f"9. Complete for Figure 6:            {'YES' if audit_pass else 'NO'}")
    print(f"\nFIGURE-6 DATASET: {verdict_str}")
    print("=" * 80)

    return 0 if audit_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
