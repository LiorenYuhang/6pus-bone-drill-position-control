#!/usr/bin/env python3
"""J14-A1.12 Mock A/B Benchmark: Parent-consumer (A1.11) vs Dedicated Feedback Bridge (A1.12).

Tests software architecture behavior under simulated Parent CPU load:
A: A1.11 Parent-consumer architecture
B: A1.12 Dedicated Feedback Bridge Process
"""
import json
import os
from pathlib import Path
import select
import statistics
import subprocess
import sys
import threading
import time

ROOT = Path("/home/lyapunov/bone_drill")
OUT = ROOT / "results/j14/j14_a1_12_software"
OUT.mkdir(parents=True, exist_ok=True)


def stats(v):
    if not v:
        return None
    v = sorted(v)

    def q(p):
        i = (len(v) - 1) * p
        lo = int(i)
        return v[lo] + (v[min(lo + 1, len(v) - 1)] - v[lo]) * (i - lo)

    return dict(n=len(v), P50=q(0.5), P95=q(0.95), P99=q(0.99), Max=v[-1])


def cpu_burn_worker(stop_event):
    """Simulates Parent CPU load (e.g. ROS executors, heavy calculations, DDS)."""
    while not stop_event.is_set():
        until = time.perf_counter() + 0.005  # burn for 5ms
        val = 0
        while time.perf_counter() < until:
            val = (val * 17 + 1) % 1000003
        time.sleep(0.002)  # sleep 2ms


def run_benchmark_trial(arch: str, rep: int, duration_s: float = 3.0) -> dict:
    trial_dir = OUT / f"mock_ab_{arch}_rep{rep}"
    if trial_dir.exists():
        import shutil
        shutil.rmtree(trial_dir)
    trial_dir.mkdir(parents=True, exist_ok=True)

    config = dict(
        epoch=time.monotonic_ns(),
        output_dir=str(trial_dir),
        hardware_mode="mock",
        port="/dev/DO_NOT_OPEN",
        baudrate=921600,
        servo_speed=50,
        maintenance_mode=True,
        reconnect_max_jump_steps=80,
        target_timeout_sec=0.2,
        feedback_bridge=(arch == "B_feedback_bridge"),
    )

    from pus6_motor_driver.j14_process_uart import ProcessClient

    # Start simulated CPU burn in Parent process (2 threads)
    stop_event = threading.Event()
    burners = [threading.Thread(target=cpu_burn_worker, args=(stop_event,), daemon=True) for _ in range(2)]
    for b in burners:
        b.start()

    client = None
    t0_wall = time.monotonic()
    t0_pcpu = time.process_time_ns()

    try:
        client = ProcessClient(config)
        end_time = time.monotonic() + duration_s
        next_lease = 0

        while time.monotonic() < end_time:
            now_ns = time.monotonic_ns()
            if now_ns >= next_lease:
                client.send_state()
                next_lease = now_ns + 50_000_000  # 50ms lease refresh

            # Parent drains whatever is on its socket
            client.receive()
            time.sleep(0.005)

        parent_wall_s = time.monotonic() - t0_wall
        parent_pcpu_ns = time.process_time_ns() - t0_pcpu
    finally:
        stop_event.set()
        for b in burners:
            b.join(timeout=1.0)
        if client:
            client.close()

    # Collect metrics
    child_cycles_path = trial_dir / "child_cycles.jsonl"
    child_cycles = [json.loads(l) for l in child_cycles_path.read_text().splitlines()] if child_cycles_path.exists() else []

    if arch == "A_parent_consumer":
        # Metrics from parent_ipc.jsonl
        parent_ipc_path = trial_dir / "parent_ipc.jsonl"
        parent_ipc = [json.loads(l) for l in parent_ipc_path.read_text().splitlines()] if parent_ipc_path.exists() else []
        recv_events = [e for e in parent_ipc if e.get("event") == "parent_receive" and e.get("type") == "feedback"]

        receive_ages_ms = []
        gaps_ms = []
        last_ns = None
        last_seq = -1
        drop_count = 0

        for e in recv_events:
            seq = e.get("cycle_seq", 0)
            if last_seq >= 0 and seq > last_seq + 1:
                drop_count += (seq - last_seq - 1)
            last_seq = seq

            t_recv = e["ns"]
            t_child = e.get("child_ready_ns")
            if t_child:
                receive_ages_ms.append((t_recv - t_child) / 1e6)
            if last_ns is not None:
                gaps_ms.append((t_recv - last_ns) / 1e6)
            last_ns = t_recv

        return dict(
            arch=arch,
            rep=rep,
            child_cycles=len(child_cycles),
            received_cycles=len(recv_events),
            drop_count=drop_count,
            parent_cpu_pct=(parent_pcpu_ns / (parent_wall_s * 1e9)) * 100,
            receive_age_ms=stats(receive_ages_ms),
            receive_gap_ms=stats(gaps_ms),
            publish_wall_ms=None,
        )

    else:
        # Architecture B: Metrics from bridge_feedback_pipeline.jsonl
        pipeline_path = trial_dir / "bridge_feedback_pipeline.jsonl"
        pipeline = [json.loads(l) for l in pipeline_path.read_text().splitlines()] if pipeline_path.exists() else []

        receive_ages_ms = [p["ipc_age_ms"] for p in pipeline if "ipc_age_ms" in p]
        gaps_ms = [p["feedback_gap_ms"] for p in pipeline if "feedback_gap_ms" in p and p["feedback_gap_ms"] > 0]
        pub_walls_ms = [
            (p["t_pub_tc_exit_ns"] - p["t_pub_js_enter_ns"]) / 1e6
            for p in pipeline
            if "t_pub_tc_exit_ns" in p and "t_pub_js_enter_ns" in p
        ]

        lifecycle_path = trial_dir / "bridge_lifecycle.jsonl"
        drop_count = 0
        if lifecycle_path.exists():
            for l in lifecycle_path.read_text().splitlines():
                ev = json.loads(l)
                if ev.get("event") == "bridge_stopped":
                    drop_count = ev.get("drop_count", 0)

        return dict(
            arch=arch,
            rep=rep,
            child_cycles=len(child_cycles),
            received_cycles=len(pipeline),
            drop_count=drop_count,
            parent_cpu_pct=(parent_pcpu_ns / (parent_wall_s * 1e9)) * 100,
            receive_age_ms=stats(receive_ages_ms),
            receive_gap_ms=stats(gaps_ms),
            publish_wall_ms=stats(pub_walls_ms),
        )


def main():
    print("Running Mock A/B Benchmark...")
    results = {"A_parent_consumer": [], "B_feedback_bridge": []}

    # Run 3 alternating trials for each architecture
    for rep in range(3):
        order = ["A_parent_consumer", "B_feedback_bridge"] if rep % 2 == 0 else ["B_feedback_bridge", "A_parent_consumer"]
        for arch in order:
            print(f"  Executing {arch} (trial {rep + 1}/3)...")
            res = run_benchmark_trial(arch, rep, duration_s=3.0)
            results[arch].append(res)
            time.sleep(0.5)

    summary = {
        "timestamp": time.time(),
        "method": "3 alternating pairs x 3 seconds @ 50 Hz mock; Parent process under 2-thread synthetic CPU load (~140% CPU)",
        "hardware_write_count": 0,
        "results": results,
    }

    out_file = OUT / "mock_benchmark.json"
    out_file.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Mock A/B Benchmark results written to {out_file}")

    # Print summary table
    print("\n" + "=" * 80)
    print("MOCK A/B BENCHMARK SUMMARY")
    print("=" * 80)
    for arch in ("A_parent_consumer", "B_feedback_bridge"):
        trials = results[arch]
        p50_ages = [t["receive_age_ms"]["P50"] for t in trials if t["receive_age_ms"]]
        max_ages = [t["receive_age_ms"]["Max"] for t in trials if t["receive_age_ms"]]
        max_gaps = [t["receive_gap_ms"]["Max"] for t in trials if t["receive_gap_ms"]]
        drops = [t["drop_count"] for t in trials]
        print(f"Architecture: {arch}")
        print(f"  Receive Age P50: {statistics.mean(p50_ages):.3f} ms | Max: {max(max_ages):.3f} ms")
        print(f"  Receive Gap Max: {max(max_gaps):.3f} ms")
        print(f"  Total Drops across reps: {sum(drops)}")
    print("=" * 80)


if __name__ == "__main__":
    main()
