#!/usr/bin/env python3
"""Runner for J3 Six-Motor Large Displacement & High-Speed Coordinated Test.

Executes synchronized 6-axis S-curve round-trip motion:
OPERATING_HOME ([1000]*6) -> Target ([1400]*6, +10mm) -> OPERATING_HOME ([1000]*6)
at trajectory peak velocity = 200 step/s (5.0 mm/s) with speed register = 240 step/s.

Strictly follows Safety STOP / Performance Result separation.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

# Ensure workspace source packages are on path
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

from pus6_control.joint_trajectory_controller import compute_synchronized_step_point, quintic_progress
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    MAX_CURRENT_MA,
    DEFAULT_SERIAL_PORT,
    MOTOR_IDS,
    NUM_MOTORS,
    OPERATING_HOME_RAW_STEPS,
    HardwareMode,
    raw_to_relative_steps,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("j3_large_displacement")

QUINTIC_PEAK_DERIVATIVE = 1.875


def execute_s_curve_segment(
    controller: SixMotorController,
    start_steps: list[int],
    target_steps: list[int],
    duration: float,
    control_rate_hz: float,
    phase_name: str,
    trajectory_log: list[dict[str, Any]],
    global_start_time: float,
) -> tuple[bool, str, int]:
    """Execute a single S-curve motion segment."""
    dt = 1.0 / control_rate_hz
    last_cmd: list[int] | None = None
    seg_start_time = time.time()
    current_t = 0.0
    write_count = 0

    logger.info(f"Starting {phase_name}: {start_steps} -> {target_steps} (duration={duration:.2f}s)")

    while current_t <= duration + 0.02:
        loop_start = time.time()
        tau = min(1.0, current_t / max(0.001, duration))
        s_prog = quintic_progress(tau)

        step_cmd = [
            int(round(s_i + s_prog * (t_i - s_i)))
            for s_i, t_i in zip(start_steps, target_steps)
        ]

        if step_cmd != last_cmd:
            controller.command_positions(step_cmd)
            write_count += NUM_MOTORS
            last_cmd = list(step_cmd)

        snapshot = controller.read_state_snapshot()
        t_global = time.time() - global_start_time

        record = {
            "t_global": round(t_global, 3),
            "phase": phase_name,
            "tau": round(tau, 4),
            "s": round(s_prog, 4),
            "cmd_steps": step_cmd,
            "actual_steps": snapshot.positions_steps,
            "currents_ma": snapshot.currents_mA,
            "temperatures_c": snapshot.temperatures_C,
            "forces_g": snapshot.forces_g_raw,
            "faults": snapshot.fault_codes,
        }
        trajectory_log.append(record)

        # Safety Invariant Checks
        if any(f != 0 for f in snapshot.fault_codes):
            reason = f"Motor hardware fault detected in {phase_name}: {snapshot.fault_codes}"
            logger.error(f"🛑 SAFETY STOP: {reason}")
            return False, reason, write_count

        if any(c > MAX_CURRENT_MA for c in snapshot.currents_mA):
            reason = f"Overcurrent detected in {phase_name}: {snapshot.currents_mA} mA (> {MAX_CURRENT_MA} mA)"
            logger.error(f"🛑 SAFETY STOP: {reason}")
            return False, reason, write_count

        current_t = time.time() - seg_start_time
        sleep_rem = dt - (time.time() - loop_start)
        if sleep_rem > 0:
            time.sleep(sleep_rem)

    return True, "", write_count


def run_large_displacement_test(
    peak_speed: float = 200.0,
    target_delta_steps: int = 400,
    control_rate_hz: float = 10.0,
    hold_time_s: float = 1.0,
    settle_time_s: float = 2.0,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    custom_start: list[int] | None = None,
    override_duration: float | None = None,
    port: str = DEFAULT_SERIAL_PORT,
) -> dict[str, Any]:
    """Execute complete 1000 -> 1400 -> 1000 synchronized round-trip test."""
    if real_hardware and confirm_motion and validated_config:
        hw_mode = HardwareMode.ACTIVE
        logger.info("⚡ GATED ACTIVE MODE: REAL HARDWARE WRITE AUTHORIZED")
    else:
        hw_mode = HardwareMode.MOCK
        logger.info("🛡️  MOCK MODE: Simulating 6-axis controller")

    # Connect controller
    if hw_mode == HardwareMode.MOCK:
        bus = FakeBus()
        if custom_start is not None:
            bus.positions = {mid: custom_start[idx] for idx, mid in enumerate(MOTOR_IDS)}
            bus.targets = dict(bus.positions)
        else:
            bus.positions = {mid: 1000 for mid in MOTOR_IDS}
            bus.targets = dict(bus.positions)
        controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    else:
        controller = SixMotorController(hardware_mode=HardwareMode.ACTIVE, port=port)

    controller.connect()

    # Pre-motion check
    logger.info("Phase 1: Reading pre-motion baseline positions...")
    initial_snapshot = controller.read_positions_snapshot()
    q_start = list(initial_snapshot.positions_steps)
    logger.info(f"Initial positions: {q_start}")

    # Forward and return target vectors
    # Operating home is 1000 steps; target is 1000 + target_delta_steps = 1400 steps
    forward_targets = validate_target_vector([1000 + target_delta_steps] * NUM_MOTORS)
    return_targets = validate_target_vector([1000] * NUM_MOTORS)

    # Duration calculation
    max_disp = float(target_delta_steps)
    if override_duration is not None:
        duration_segment = max(0.01, override_duration)
    else:
        duration_segment = max(1.0, (QUINTIC_PEAK_DERIVATIVE * max_disp) / peak_speed)

    planned_peak_speed = (QUINTIC_PEAK_DERIVATIVE * max_disp) / duration_segment
    # Motor speed register setting (20% higher than trajectory peak, at least 60)
    motor_reg_speed = max(60, int(round(peak_speed * 1.2)))

    logger.info(f"Trajectory Specification:")
    logger.info(f"  Displacement:         {target_delta_steps} steps ({target_delta_steps * 0.025:.2f} mm)")
    logger.info(f"  Target Peak Speed:    {planned_peak_speed:.2f} step/s ({planned_peak_speed * 0.025:.2f} mm/s)")
    logger.info(f"  0x28 Speed Register:  {motor_reg_speed} step/s ({motor_reg_speed * 0.025:.2f} mm/s)")
    logger.info(f"  Segment Duration:     {duration_segment:.2f} s")
    logger.info(f"  Control Update Rate:  {control_rate_hz:.1f} Hz")

    # Configure motor parameters
    logger.info(f"Phase 2: Configuring motor parameters (mode=2, speed_reg={motor_reg_speed})...")
    controller.configure_all(speed=motor_reg_speed)

    trajectory_log: list[dict[str, Any]] = []
    global_start_time = time.time()
    total_dispatched_writes = 0

    # Phase 3: Forward Motion (1000 -> 1400)
    ok, reason, writes_fwd = execute_s_curve_segment(
        controller=controller,
        start_steps=q_start,
        target_steps=forward_targets,
        duration=duration_segment,
        control_rate_hz=control_rate_hz,
        phase_name="FORWARD (1000->1400)",
        trajectory_log=trajectory_log,
        global_start_time=global_start_time,
    )
    total_dispatched_writes += writes_fwd

    if not ok:
        controller.pause_all()
        return {"safety_aborted": True, "abort_reason": reason}

    # Phase 4: Hold at Forward Peak
    logger.info(f"Phase 4: Holding at peak position for {hold_time_s:.2f}s...")
    time.sleep(hold_time_s)
    peak_snapshot = controller.read_positions_snapshot()
    q_peak_actual = list(peak_snapshot.positions_steps)
    logger.info(f"Peak reached positions: {q_peak_actual}")

    # Phase 5: Return Motion (1400 -> 1000)
    ok, reason, writes_ret = execute_s_curve_segment(
        controller=controller,
        start_steps=q_peak_actual,
        target_steps=return_targets,
        duration=duration_segment,
        control_rate_hz=control_rate_hz,
        phase_name="RETURN (1400->1000)",
        trajectory_log=trajectory_log,
        global_start_time=global_start_time,
    )
    total_dispatched_writes += writes_ret

    if not ok:
        controller.pause_all()
        return {"safety_aborted": True, "abort_reason": reason}

    # Phase 6: Settle Observation and Stop Confirmation
    logger.info(f"Phase 6: Settling observation and stop confirmation ({settle_time_s:.2f}s)...")
    time.sleep(settle_time_s)
    final_snapshot = controller.read_positions_snapshot()
    controller.pause_all()

    q_final = list(final_snapshot.positions_steps)
    q_final_rel = raw_to_relative_steps(q_final)
    final_errors = [abs(f - 1000) for f in q_final]
    max_final_error = max(final_errors)

    # Post-process velocity & tracking performance
    actual_max_speeds = [0.0] * NUM_MOTORS
    max_tracking_lag = [0] * NUM_MOTORS
    peak_currents = [0] * NUM_MOTORS

    for idx in range(len(trajectory_log) - 1):
        pt1 = trajectory_log[idx]
        pt2 = trajectory_log[idx + 1]
        dt_sample = pt2["t_global"] - pt1["t_global"]
        if dt_sample > 0.001:
            for i in range(NUM_MOTORS):
                spd = abs(pt2["actual_steps"][i] - pt1["actual_steps"][i]) / dt_sample
                if spd > actual_max_speeds[i]:
                    actual_max_speeds[i] = round(spd, 1)

    for pt in trajectory_log:
        for i in range(NUM_MOTORS):
            lag = abs(pt["actual_steps"][i] - pt["cmd_steps"][i])
            if lag > max_tracking_lag[i]:
                max_tracking_lag[i] = lag
            if pt["currents_ma"][i] > peak_currents[i]:
                peak_currents[i] = pt["currents_ma"][i]

    logger.info("=" * 60)
    logger.info("LARGE DISPLACEMENT ROUND-TRIP TEST SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Start Positions:         {q_start}")
    logger.info(f"Forward Peak Target:     {forward_targets}")
    logger.info(f"Peak Reached:            {q_peak_actual}")
    logger.info(f"Final Return Positions:  {q_final}")
    logger.info(f"Final Relative Steps:    {q_final_rel}")
    logger.info(f"Final Return Errors:     {final_errors} (max: {max_final_error} step)")
    logger.info(f"Planned Peak Speed:      {planned_peak_speed:.1f} step/s ({planned_peak_speed * 0.025:.2f} mm/s)")
    logger.info(f"0x28 Speed Register:     {motor_reg_speed} step/s ({motor_reg_speed * 0.025:.2f} mm/s)")
    logger.info(f"Measured Max Speeds:     {actual_max_speeds} step/s")
    logger.info(f"Dynamic Max Lag:         {max_tracking_lag} step")
    logger.info(f"Peak Currents:           {peak_currents} mA")
    logger.info(f"Return Tolerable (<=2):  {'PASS' if max_final_error <= 2 else 'RECORDED'}")
    logger.info(f"Safety Invariants Kept:  PASS")
    logger.info("=" * 60)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    report = {
        "timestamp": timestamp_str,
        "test_name": "J3_large_displacement_200step_s",
        "hardware_mode": hw_mode.value,
        "target_delta_steps": target_delta_steps,
        "target_delta_mm": target_delta_steps * 0.025,
        "planned_peak_speed_step_s": planned_peak_speed,
        "planned_peak_speed_mm_s": planned_peak_speed * 0.025,
        "motor_reg_speed_0x28": motor_reg_speed,
        "segment_duration_s": duration_segment,
        "total_elapsed_s": time.time() - global_start_time,
        "q_start_raw": q_start,
        "q_peak_target": forward_targets,
        "q_peak_actual": q_peak_actual,
        "q_final_raw": q_final,
        "q_final_relative": q_final_rel,
        "final_return_errors_steps": final_errors,
        "max_return_error_steps": max_final_error,
        "measured_max_speeds_step_s": actual_max_speeds,
        "dynamic_max_lag_steps": max_tracking_lag,
        "peak_currents_ma": peak_currents,
        "safety_aborted": False,
        "abort_reason": "",
        "dispatched_write_frames": total_dispatched_writes,
        "write_audit": {
            "total_writes": getattr(controller, "write_count", 0),
            "total_reads": getattr(controller, "read_count", 0),
        },
        "trajectory_sample_count": len(trajectory_log),
    }

    # Save log
    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"j3_large_displacement_{timestamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump({"report": report, "trajectory": trajectory_log}, f, indent=2)
    logger.info(f"Detailed log saved to: {log_file}")

    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="J3 Six-Motor Large Displacement Test")
    parser.add_argument("--peak-speed", type=float, default=200.0, help="Planned trajectory peak speed in step/s (default: 200.0)")
    parser.add_argument("--delta-steps", type=int, default=400, help="Displacement in steps from home (default: 400 = 10mm)")
    parser.add_argument("--control-rate", type=float, default=10.0, help="Control rate in Hz (default: 10.0)")
    parser.add_argument("--hold-time", type=float, default=1.0, help="Hold time at peak in seconds (default: 1.0)")
    parser.add_argument("--real-hardware", action="store_true", help="Enable real serial bus hardware connection")
    parser.add_argument("--confirm-motion", action="store_true", help="Explicit human confirmation for physical movement")
    parser.add_argument("--validated-config", action="store_true", help="Explicit human confirmation for validated parameters")
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=f"UART device (default: validated by-id path {DEFAULT_SERIAL_PORT})",
    )

    args = parser.parse_args()
    report = run_large_displacement_test(
        peak_speed=args.peak_speed,
        target_delta_steps=args.delta_steps,
        control_rate_hz=args.control_rate,
        hold_time_s=args.hold_time,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
        port=args.port,
    )

    if report.get("safety_aborted", False):
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
