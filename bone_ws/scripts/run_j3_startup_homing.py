#!/usr/bin/env python3
"""Runner for J3 6-Axis Coordinated Startup Homing and Controlled Parking.

Executes synchronized 6-axis S-curve motion between arbitrary startup positions
and OPERATING_HOME ([1000]*6) or STORAGE_PARK ([0]*6).

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
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    DEFAULT_SPEED,
    DEFAULT_SERIAL_PORT,
    MAX_CURRENT_MA,
    MOTOR_IDS,
    NUM_MOTORS,
    OPERATING_HOME_RAW_STEPS,
    STORAGE_PARK_RAW_STEPS,
    HardwareMode,
    raw_to_relative_steps,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController
from pus6_motor_driver.uart_bus import UartBus

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("j3_startup_homing")


def run_homing_session(
    action: str = "home",
    speed: int = DEFAULT_SPEED,
    control_rate_hz: float = 10.0,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
    custom_start: list[int] | None = None,
    duration: float | None = None,
    settle_time: float = 2.0,
    port: str = DEFAULT_SERIAL_PORT,
) -> dict[str, Any]:
    """Execute synchronized 6-axis homing or parking motion."""
    # Safety mode gating
    if real_hardware and confirm_motion and validated_config:
        hw_mode = HardwareMode.ACTIVE
        logger.info("⚡ GATED ACTIVE MODE: REAL HARDWARE WRITE AUTHORIZED")
    else:
        hw_mode = HardwareMode.MOCK
        logger.info("🛡️  MOCK MODE: Simulating 6-axis controller")

    # Target selection
    if action == "home":
        target_steps = list(OPERATING_HOME_RAW_STEPS)
        target_name = "OPERATING_HOME ([1000]*6)"
    elif action == "park":
        target_steps = list(STORAGE_PARK_RAW_STEPS)
        target_name = "STORAGE_PARK ([0]*6)"
    else:
        raise ValueError(f"Unknown action '{action}'. Must be 'home' or 'park'.")

    # Hardware connection
    if hw_mode == HardwareMode.MOCK:
        bus = FakeBus()
        if custom_start is not None:
            assert len(custom_start) == NUM_MOTORS
            bus.positions = {mid: custom_start[idx] for idx, mid in enumerate(MOTOR_IDS)}
            bus.targets = dict(bus.positions)
        controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    else:
        controller = SixMotorController(hardware_mode=HardwareMode.ACTIVE, port=port)

    controller.connect()

    # Pre-motion read-only baseline
    logger.info("Phase 1: Reading initial 6-axis starting positions...")
    initial_snapshot = controller.read_positions_snapshot()
    start_steps = list(initial_snapshot.positions_steps)
    logger.info(f"Initial positions (q_start): {start_steps}")
    logger.info(f"Target positions ({action}): {target_steps} -> {target_name}")

    # Validate target vector against safety boundaries
    validated_target = validate_target_vector(target_steps)

    # Compute duration based on maximum axis displacement
    displacements = [abs(t - s) for t, s in zip(validated_target, start_steps)]
    max_disp = max(displacements)

    # For quintic polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5:
    # ds/dtau = 30*tau^2 - 60*tau^3 + 30*tau^4, with exact peak derivative = 1.875 at tau = 0.5.
    # Therefore, to constrain maximum planned trajectory velocity <= speed (e.g. 50 step/s):
    # v_peak = D_max * 1.875 / T <= speed ==> T >= 1.875 * D_max / speed
    QUINTIC_PEAK_DERIVATIVE = 1.875

    if duration is not None:
        total_duration = max(0.01, duration)
    elif max_disp == 0:
        logger.info("All axes already at target. Motion skipped.")
        total_duration = 0.5
    else:
        peak_speed = max(1.0, float(speed))
        total_duration = max(2.0, (QUINTIC_PEAK_DERIVATIVE * float(max_disp)) / peak_speed)

    planned_peak_speed = (QUINTIC_PEAK_DERIVATIVE * float(max_disp)) / total_duration if total_duration > 0 else 0.0

    logger.info(
        f"Trajectory plan: max_disp={max_disp} steps ({max_disp * 0.025:.3f} mm), "
        f"target_peak_speed={planned_peak_speed:.2f} step/s ({planned_peak_speed * 0.025:.3f} mm/s), "
        f"duration T={total_duration:.2f} s (1.875 factor applied)"
    )

    # Configure motor speeds and operating mode
    # Set motor hardware velocity limit register (0x28) to at least the peak speed
    motor_reg_speed = max(50, int(round(planned_peak_speed * 1.2)))
    logger.info(f"Phase 2: Configuring motor parameters (mode=2, speed_reg={motor_reg_speed})...")
    controller.configure_all(speed=motor_reg_speed)

    # Coordinated S-curve trajectory execution loop
    logger.info(f"Phase 3: Dispatching coordinated S-curve trajectory @ {control_rate_hz:.1f} Hz...")
    start_time = time.time()
    dt = 1.0 / control_rate_hz
    trajectory_log: list[dict[str, Any]] = []
    last_dispatched_cmd: list[int] | None = None
    safety_aborted = False
    abort_reason = ""
    write_frame_count = 0

    current_t = 0.0
    while current_t <= total_duration + 0.05:
        loop_start = time.time()
        tau = min(1.0, current_t / max(0.001, total_duration))
        s_prog = quintic_progress(tau)

        # Interpolated step coordinate: q_i(t) = q_start_i + s(t) * (q_target_i - q_start_i)
        step_cmd = [
            int(round(s_i + s_prog * (t_i - s_i)))
            for s_i, t_i in zip(start_steps, validated_target)
        ]

        # Dispatch if command changed or at start
        if step_cmd != last_dispatched_cmd:
            controller.command_positions(step_cmd)
            write_frame_count += NUM_MOTORS
            last_dispatched_cmd = list(step_cmd)

        # Telemetry sampling
        current_snapshot = controller.read_state_snapshot()
        record = {
            "t": round(current_t, 3),
            "tau": round(tau, 4),
            "s": round(s_prog, 4),
            "cmd_steps": step_cmd,
            "actual_steps": current_snapshot.positions_steps,
            "currents_ma": current_snapshot.currents_mA,
            "temperatures_c": current_snapshot.temperatures_C,
            "forces_g": current_snapshot.forces_g_raw,
            "faults": current_snapshot.fault_codes,
        }
        trajectory_log.append(record)

        # Safety Check: strict Safety STOP conditions only
        if any(f != 0 for f in current_snapshot.fault_codes):
            safety_aborted = True
            abort_reason = f"Motor hardware fault detected: {current_snapshot.fault_codes}"
            logger.error(f"🛑 SAFETY STOP: {abort_reason}")
            break

        if any(c > MAX_CURRENT_MA for c in current_snapshot.currents_mA):
            safety_aborted = True
            abort_reason = f"Overcurrent detected: {current_snapshot.currents_mA} mA (> {MAX_CURRENT_MA} mA)"
            logger.error(f"🛑 SAFETY STOP: {abort_reason}")
            break

        current_t = time.time() - start_time
        sleep_rem = dt - (time.time() - loop_start)
        if sleep_rem > 0:
            time.sleep(sleep_rem)

    # Phase 4: Settle window and Stop Confirmation
    logger.info(f"Phase 4: Settling observation and stop confirmation ({settle_time:.2f}s)...")
    time.sleep(settle_time)
    final_snapshot = controller.read_positions_snapshot()
    controller.pause_all()

    # Performance Evaluation (Recorded as performance metrics, not abort conditions)
    final_errors = [
        abs(f - t) for f, t in zip(final_snapshot.positions_steps, validated_target)
    ]
    max_error = max(final_errors) if final_errors else 0
    tracking_pass = max_error <= 2  # +/- 2 step normal quantization/settle band

    relative_steps = raw_to_relative_steps(final_snapshot.positions_steps)

    logger.info("=" * 60)
    logger.info("HOMING / PARKING EXECUTION SUMMARY")
    logger.info("=" * 60)
    logger.info(f"Action:                  {action.upper()}")
    logger.info(f"Hardware Mode:           {hw_mode.value}")
    logger.info(f"Start Raw Steps:         {start_steps}")
    logger.info(f"Target Raw Steps:        {validated_target}")
    logger.info(f"Final Raw Steps:         {final_snapshot.positions_steps}")
    logger.info(f"Final Relative Steps:    {relative_steps}")
    logger.info(f"Final Position Errors:   {final_errors} (max: {max_error} step)")
    logger.info(f"Tracking Tolerable (<=2):{'PASS' if tracking_pass else 'RECORDED (deviation > 2)'}")
    logger.info(f"Safety Invariants Kept:  {'PASS' if not safety_aborted else 'FAILED (' + abort_reason + ')'}")
    logger.info("=" * 60)

    # Build report object
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    report = {
        "timestamp": timestamp_str,
        "action": action,
        "hardware_mode": hw_mode.value,
        "planned_duration_s": total_duration,
        "planned_peak_speed_step_s": planned_peak_speed,
        "actual_elapsed_s": time.time() - start_time,
        "speed_step_s": speed,
        "port": port if hw_mode == HardwareMode.ACTIVE else None,
        "motor_reg_speed_0x28": motor_reg_speed,
        "control_rate_hz": control_rate_hz,
        "dispatched_write_frames": write_frame_count,
        "q_start_raw": start_steps,
        "q_target_raw": validated_target,
        "q_final_raw": final_snapshot.positions_steps,
        "q_final_relative": relative_steps,
        "final_errors_steps": final_errors,
        "max_error_steps": max_error,
        "tracking_tolerable": tracking_pass,
        "safety_aborted": safety_aborted,
        "abort_reason": abort_reason,
        "write_audit": {
            "total_writes": getattr(controller, "write_count", 0),
            "total_reads": getattr(controller, "read_count", 0),
        },
        "trajectory_sample_count": len(trajectory_log),
    }

    # Save log
    log_dir = WORKSPACE_ROOT / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"j3_homing_{action}_{timestamp_str}.json"
    with open(log_file, "w", encoding="utf-8") as f:
        json.dump({"report": report, "trajectory": trajectory_log}, f, indent=2)
    logger.info(f"Detailed log saved to: {log_file}")

    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="J3 6-Axis Startup Homing / Controlled Parking")
    parser.add_argument("--action", choices=["home", "park"], default="home", help="Target action: home ([1000]*6) or park ([0]*6)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Cruise speed in step/s (default: 50)")
    parser.add_argument("--control-rate", type=float, default=10.0, help="Control dispatch rate in Hz (default: 10.0)")
    parser.add_argument("--real-hardware", action="store_true", help="Enable real serial bus hardware connection")
    parser.add_argument("--confirm-motion", action="store_true", help="Explicit human confirmation for physical movement")
    parser.add_argument("--validated-config", action="store_true", help="Explicit human confirmation for validated parameters")
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=f"UART device (default: validated by-id path {DEFAULT_SERIAL_PORT})",
    )

    args = parser.parse_args()
    report = run_homing_session(
        action=args.action,
        speed=args.speed,
        control_rate_hz=args.control_rate,
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
        port=args.port,
    )

    if report["safety_aborted"]:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
