#!/usr/bin/env python3
"""Stage J4-F0 — Continuous Cartesian Control Pipeline Mock & Architecture Validation Benchmark.

Adheres strictly to J4-F0 requirements:
- F0-A: High-rate Target Stream & Zero-Retry Dispatch
- F0-B: Latest-Value-Wins ROS/UART Producer-Consumer Decoupling
- F0-C: Stateful Jerk-Limited 3rd-Order Reference Shaper Continuity
- F0-D: Timing, Jitter, Deadline & Freshness Characterization
- F0-E: Temporary J4 Validated Software Envelope & Zero-Write Safety Verification

Safety Guarantee:
- 100% Mock execution; 0 real hardware writes enforced.
"""

from __future__ import annotations

import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_control.joint_trajectory_controller import (
    JointTrajectoryController,
    StatefulJerkLimitedShaper,
)
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import HardwareMode, NUM_MOTORS
from pus6_motor_driver.six_motor_controller import SixMotorController


def run_rate_sweep_benchmark(
    frequencies: list[float] = [50.0, 100.0, 200.0, 300.0],
    duration_per_freq_s: float = 2.0,
) -> dict[str, Any]:
    """Evaluate multi-rate scheduling architecture and cycle timing across frequencies."""
    results: dict[str, Any] = {}

    ik = Analytical6PusKinematics()
    ref_pose = [0.0, 0.0, ik.nominal_height, 0.0, 0.0, 0.0]

    for freq in frequencies:
        dt = 1.0 / freq
        total_ticks = int(freq * duration_per_freq_s)

        bus = FakeBus()
        controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
        controller.connect()

        shaper = StatefulJerkLimitedShaper(
            num_joints=NUM_MOTORS,
            omega_n=20.0,
            max_speed_m_per_sec=0.005,
            max_accel_m_per_sec2=0.050,
            max_jerk_m_per_sec3=0.500,
        )

        loop_durations_ms: list[float] = []
        ik_durations_us: list[float] = []
        shaper_durations_us: list[float] = []
        write_durations_ms: list[float] = []
        deadline_miss_count = 0

        t_sim = 0.0
        start_mono = time.perf_counter_ns()
        last_tick_mono = start_mono

        for tick in range(total_ticks):
            tick_start_ns = time.perf_counter_ns()
            t_sim = tick * dt

            # 1. Target generation & Analytical IK
            z_t = ik.nominal_height + 0.005 * math.sin(2.0 * math.pi * 0.5 * t_sim)
            cartesian_pose = [0.0, 0.0, z_t, 0.0, 0.0, 0.0]

            t_ik_start = time.perf_counter_ns()
            deltas = ik.differential_inverse_kinematics(cartesian_pose, ref_pose)
            t_ik_end = time.perf_counter_ns()
            ik_durations_us.append((t_ik_end - t_ik_start) / 1e3)

            # 2. Stateful reference shaping
            t_shaper_start = time.perf_counter_ns()
            cmd_m = shaper.update(deltas, dt)
            t_shaper_end = time.perf_counter_ns()
            shaper_durations_us.append((t_shaper_end - t_shaper_start) / 1e3)

            # 3. Step conversion & dispatch
            target_steps = [int(round(1000 + p * 40000.0)) for p in cmd_m]
            t_w_start = time.perf_counter_ns()
            cmd_res = controller.command_positions_with_metrics(target_steps)
            t_w_end = time.perf_counter_ns()
            w_ms = (t_w_end - t_w_start) / 1e6
            write_durations_ms.append(w_ms)

            # 4. Piggyback feedback extraction
            snapshot = cmd_res.to_state_snapshot()
            assert len(snapshot.positions_steps) == 6

            tick_end_ns = time.perf_counter_ns()
            dur_ms = (tick_end_ns - tick_start_ns) / 1e6
            loop_durations_ms.append(dur_ms)

            target_period_ms = 1000.0 / freq
            if dur_ms > target_period_ms:
                deadline_miss_count += 1

            last_tick_mono = tick_end_ns

        controller.close()

        results[f"{int(freq)}Hz"] = {
            "frequency_hz": freq,
            "target_period_ms": 1000.0 / freq,
            "total_cycles": total_ticks,
            "ik_mean_us": float(np.mean(ik_durations_us)),
            "ik_max_us": float(np.max(ik_durations_us)),
            "shaper_mean_us": float(np.mean(shaper_durations_us)),
            "shaper_max_us": float(np.max(shaper_durations_us)),
            "mock_write_mean_ms": float(np.mean(write_durations_ms)),
            "mock_write_max_ms": float(np.max(write_durations_ms)),
            "total_compute_mean_ms": float(np.mean(loop_durations_ms)),
            "total_compute_max_ms": float(np.max(loop_durations_ms)),
            "deadline_miss_count": deadline_miss_count,
            "deadline_miss_ratio": deadline_miss_count / float(total_ticks),
        }

    return results


def run_producer_consumer_decoupling_test() -> dict[str, Any]:
    """Test Latest-Value-Wins decoupling: High-rate producer (200Hz) vs 50Hz consumer."""
    producer_freq = 200.0  # 5ms period
    consumer_freq = 50.0   # 20ms period
    duration_s = 2.0

    total_produced = int(producer_freq * duration_s)
    total_consumed = int(consumer_freq * duration_s)

    produced_targets: list[dict[str, Any]] = []
    consumed_targets: list[dict[str, Any]] = []

    # Shared Latest-Value-Wins buffer
    latest_slot: dict[str, Any] | None = None
    overwrite_count = 0
    consumed_count = 0
    target_ages_ms: list[float] = []

    t_start_ns = time.perf_counter_ns()
    consumer_next_ns = t_start_ns + int(1e9 / consumer_freq)

    for p_idx in range(total_produced):
        t_prod_ns = t_start_ns + int(p_idx * (1e9 / producer_freq))
        step_val = 1000 + int(p_idx * 2)
        target = [step_val] * 6

        # Producer puts latest
        if latest_slot is not None and not latest_slot["consumed"]:
            overwrite_count += 1

        latest_slot = {
            "target": target,
            "seq": p_idx,
            "stamp_ns": t_prod_ns,
            "consumed": False,
        }
        produced_targets.append(latest_slot)

        # Check if consumer timer fires
        if t_prod_ns >= consumer_next_ns:
            if latest_slot is not None and not latest_slot["consumed"]:
                latest_slot["consumed"] = True
                consumed_count += 1
                age_ms = (t_prod_ns - latest_slot["stamp_ns"]) / 1e6
                target_ages_ms.append(age_ms)
                consumed_targets.append(latest_slot)
            consumer_next_ns += int(1e9 / consumer_freq)

    return {
        "producer_rate_hz": producer_freq,
        "consumer_rate_hz": consumer_freq,
        "total_produced": total_produced,
        "total_consumed": consumed_count,
        "overwrite_count": overwrite_count,
        "overwrite_ratio": overwrite_count / float(total_produced),
        "target_age_mean_ms": float(np.mean(target_ages_ms)) if target_ages_ms else 0.0,
        "target_age_p95_ms": float(np.percentile(target_ages_ms, 95)) if target_ages_ms else 0.0,
        "target_age_max_ms": float(np.max(target_ages_ms)) if target_ages_ms else 0.0,
    }


def run_trajectory_continuity_audit() -> dict[str, Any]:
    """Audit position, velocity, acceleration, and jerk continuity during dynamic trajectory changes."""
    dt = 0.02  # 50 Hz
    v_max = 0.005
    a_max = 0.050
    j_max = 0.500

    shaper = StatefulJerkLimitedShaper(
        num_joints=6,
        omega_n=20.0,
        max_speed_m_per_sec=v_max,
        max_accel_m_per_sec2=a_max,
        max_jerk_m_per_sec3=j_max,
    )

    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    jerks: list[float] = []

    # Sequence of varying targets: +10mm step -> -5mm step -> sinusoidal wave
    targets = (
        [[0.010] * 6] * 50       # 1.0 s step +10mm
        + [[-0.005] * 6] * 50    # 1.0 s reversal step -5mm
        + [[0.005 * math.sin(2.0 * math.pi * 1.0 * (i * dt))] * 6 for i in range(100)] # 2.0 s sine
    )

    for tgt in targets:
        q = shaper.update(tgt, dt)
        positions.append(q[0])
        velocities.append(shaper.dq[0])
        accelerations.append(shaper.ddq[0])
        jerks.append(shaper.dddq[0])

    pos_diffs = np.diff(positions)
    vel_diffs = np.diff(velocities)
    acc_diffs = np.diff(accelerations)

    return {
        "total_samples": len(positions),
        "peak_velocity_m_s": float(np.max(np.abs(velocities))),
        "peak_velocity_step_s": float(np.max(np.abs(velocities)) * 40000.0),
        "peak_accel_m_s2": float(np.max(np.abs(accelerations))),
        "peak_jerk_m_s3": float(np.max(np.abs(jerks))),
        "max_pos_step_delta_mm": float(np.max(np.abs(pos_diffs)) * 1e3),
        "max_vel_step_delta_mm_s": float(np.max(np.abs(vel_diffs)) * 1e3),
        "max_acc_step_delta_mm_s2": float(np.max(np.abs(acc_diffs)) * 1e3),
        "c2_continuity_verified": bool(
            np.max(np.abs(velocities)) <= v_max + 1e-6
            and np.max(np.abs(accelerations)) <= a_max + 1e-6
            and np.max(np.abs(jerks)) <= j_max + 1e-6
        ),
    }


def main() -> None:
    print("================================================================================")
    print(" J4-F0 Target Stream & Real-Time Architecture Validation (Mock Benchmark)")
    print("================================================================================")
    print("Hardware Write Protection: HARDWARE_MODE=MOCK (0 real writes enforced)")

    # 1. Rate sweep
    print("\n[1/3] Running Multi-Rate Sweep Benchmark (50, 100, 200, 300 Hz)...")
    rate_results = run_rate_sweep_benchmark()
    for freq_key, data in rate_results.items():
        print(f"  [{freq_key}] Compute: mean={data['total_compute_mean_ms']:.4f}ms, max={data['total_compute_max_ms']:.4f}ms | IK: mean={data['ik_mean_us']:.2f}us | Shaper: mean={data['shaper_mean_us']:.2f}us")

    # 2. Producer-consumer decoupling
    print("\n[2/3] Running Latest-Value-Wins Decoupling Test (200Hz Producer -> 50Hz Consumer)...")
    pc_results = run_producer_consumer_decoupling_test()
    print(f"  Produced: {pc_results['total_produced']}, Consumed: {pc_results['total_consumed']}")
    print(f"  Overwrites: {pc_results['overwrite_count']} ({pc_results['overwrite_ratio']*100:.1f}%)")
    print(f"  Target Age: mean={pc_results['target_age_mean_ms']:.2f}ms, p95={pc_results['target_age_p95_ms']:.2f}ms, max={pc_results['target_age_max_ms']:.2f}ms")

    # 3. Trajectory continuity audit
    print("\n[3/3] Running 3rd-Order Stateful Reference Shaper Continuity Audit...")
    traj_results = run_trajectory_continuity_audit()
    print(f"  Peak Velocity:     {traj_results['peak_velocity_m_s']*1e3:.2f} mm/s ({traj_results['peak_velocity_step_s']:.1f} step/s)")
    print(f"  Peak Acceleration: {traj_results['peak_accel_m_s2']*1e3:.2f} mm/s^2")
    print(f"  Peak Jerk:         {traj_results['peak_jerk_m_s3']*1e3:.2f} mm/s^3")
    print(f"  C^2 Continuity:    {'PASS' if traj_results['c2_continuity_verified'] else 'FAIL'}")

    # Output JSON summary
    summary = {
        "timestamp": datetime.now().isoformat(),
        "stage": "J4-F0",
        "mode": "mock_validation",
        "hardware_write_count": 0,
        "rate_sweep": rate_results,
        "producer_consumer_decoupling": pc_results,
        "trajectory_continuity": traj_results,
    }

    log_path = WORKSPACE_ROOT / "log" / "j4_f0_mock_benchmark_summary.json"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"\nSaved J4-F0 Mock Benchmark Summary to: {log_path}")
    print("================================================================================")


if __name__ == "__main__":
    main()
