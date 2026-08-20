#!/usr/bin/env python3
"""Stage M6-C: move ID1..ID6 synchronously once on one audited UART bus."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import logging
from pathlib import Path
import statistics
import sys
import time
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import (  # noqa: E402
    CMD_FRAME_HEADER,
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    CMD_WR_REGISTER,
    Frame,
    StatusFeedback,
    parse_frame,
)
from tools.safe_endpoint_escape_id1 import (  # noqa: E402
    MotionTrace,
    SafetyViolation,
    _check_status,
    _stop_confirmation_metrics,
)
from tools.safe_m2d_joint_id1_id2 import (  # noqa: E402
    M2dPositionBaseline,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (1, 2, 3, 4, 5, 6)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
MOVE_STEPS = 10
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 3.0
MAX_CURRENT_MA = 500
BASELINE_DURATION_S = 2.0
BASELINE_RATE_HZ = 50.0
POLL_INTERVAL_S = 0.02
STOP_CONFIRM_DURATION_S = 0.5
PAUSE_TRIGGER_TOLERANCE_STEPS = 2
FINAL_ACCEPTANCE_TOLERANCE_STEPS = 3
STRICT_TOLERANCE_STEPS = 1
SOFTWARE_UPPER_LIMIT = 1800
ALLOWED_WRITE_REGISTERS = {
    CONTROL_MODE_ADDR,
    TARGET_POSITION_ADDR,
    PAUSE_ADDR,
}

logger = logging.getLogger("safe_m6c_joint_six_motors")


@dataclass
class M6cDirectionGuard:
    """Direction guard for Stage M6-C six-motor joint motion.

    - Before positive motion is confirmed (positive_motion_confirmed=False):
      Allows actual position down to min(baseline_min, max(device_lower_limit, reference - 1)).
      Samples strictly below this lower bound trigger pre-motion negative-direction SafetyViolation.
      When actual_position > baseline_max, positive motion is established and
      positive_motion_confirmed becomes True.
    - Once positive motion is confirmed (positive_motion_confirmed=True):
      Immediately reverts to the standard motion-phase direction guard (actual >= baseline_min),
      removing the pre-motion -1 step tolerance.
    """

    motor_id: int
    reference: int
    baseline_min: int
    baseline_max: int
    device_lower_limit: int = 0
    positive_motion_confirmed: bool = False

    def observe(self, actual_position: int) -> None:
        if not self.positive_motion_confirmed:
            if actual_position > self.baseline_max:
                self.positive_motion_confirmed = True
            else:
                allowed_min = min(
                    self.baseline_min,
                    max(self.device_lower_limit, self.reference - 1),
                )
                if actual_position < allowed_min:
                    raise SafetyViolation(
                        f"ID{self.motor_id} pre-motion negative-direction movement detected below allowed lower bound: "
                        f"reference={self.reference} allowed_min={allowed_min} "
                        f"baseline=[{self.baseline_min},{self.baseline_max}] "
                        f"actual={actual_position}"
                    )
                return

        if actual_position < self.baseline_min:
            raise SafetyViolation(
                f"ID{self.motor_id} motion-phase negative-direction movement detected below baseline band: "
                f"baseline=[{self.baseline_min},{self.baseline_max}] "
                f"actual={actual_position}"
            )


@dataclass(frozen=True)
class ConfigSnapshot:
    mode: int
    speed: int
    target: int


class M6cAuditedBus:
    """Audited transport wrapper for Stage M6-C six-motor joint motion."""

    def __init__(self, bus: UartBus):
        self._bus = bus
        self.write_frames: list[bytes] = []
        self.write_destinations: list[int] = []
        self.write_registers: list[str] = []
        self.read_counts: dict[int, int] = {motor_id: 0 for motor_id in MOTOR_IDS}

    @property
    def write_count(self) -> int:
        return len(self.write_frames)

    def request(self, frame: bytes, **kwargs) -> Frame:
        parsed = parse_frame(frame)
        if parsed.motor_id not in MOTOR_IDS:
            raise SafetyViolation(
                f"M6-C destination ID{parsed.motor_id} is outside allowed scope {MOTOR_IDS}"
            )

        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                raise SafetyViolation(
                    f"M6-C write to register 0x{parsed.register_addr:04X} on ID{parsed.motor_id} is not allowed"
                )
            self.write_frames.append(frame)
            self.write_destinations.append(parsed.motor_id)
            self.write_registers.append(f"0x{parsed.register_addr:04X}")
            data_hex = " ".join(f"{b:02X}" for b in parsed.data)
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=%d register=0x%04X data=%s verified=true",
                parsed.motor_id,
                parsed.register_addr,
                data_hex,
            )
        elif parsed.command in (CMD_RD_REGISTER, CMD_RD_STATUS):
            self.read_counts[parsed.motor_id] = (
                self.read_counts.get(parsed.motor_id, 0) + 1
            )
            reg_info = (
                f" register=0x{parsed.register_addr:04X}"
                if parsed.command == CMD_RD_REGISTER
                else " register=0x0000"
            )
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X%s",
                parsed.motor_id,
                parsed.command,
                reg_info,
            )

        return self._bus.request(frame, **kwargs)

    def close(self) -> None:
        self._bus.close()


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(log_path, mode="w", encoding="utf-8")
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.INFO)

    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.handlers.clear()
    root.addHandler(file_handler)

    bus_logger = logging.getLogger("uart_bus")
    bus_logger.setLevel(logging.INFO)
    bus_logger.handlers.clear()
    bus_logger.addHandler(file_handler)


def _read_value(motor: Motor, address: int) -> int:
    return motor.read_register(address).values[0]


def _read_limits(motor: Motor) -> tuple[int, int]:
    lower = _read_value(motor, LOWER_LIMIT_ADDR)
    upper = _read_value(motor, UPPER_LIMIT_ADDR)
    return lower, upper


def _read_config(motor: Motor) -> ConfigSnapshot:
    mode = _read_value(motor, CONTROL_MODE_ADDR)
    speed = _read_value(motor, TARGET_SPEED_ADDR)
    target = _read_value(motor, TARGET_POSITION_ADDR)
    return ConfigSnapshot(mode=mode, speed=speed, target=target)


def _check_no_unidirectional_drift(
    samples: list[int], motor_id: int, name: str = "position"
) -> None:
    if len(samples) < 10:
        return
    diffs = [samples[i + 1] - samples[i] for i in range(len(samples) - 1)]
    non_zero = [d for d in diffs if d != 0]
    if non_zero and (all(d > 0 for d in non_zero) or all(d < 0 for d in non_zero)):
        if abs(samples[-1] - samples[0]) >= 2:
            raise SafetyViolation(
                f"ID{motor_id} {name} baseline shows monotonic drift: "
                f"start={samples[0]} end={samples[-1]}"
            )
    half = len(samples) // 2
    first_half = samples[:half]
    second_half = samples[half:]
    if set(first_half).isdisjoint(set(second_half)):
        raise SafetyViolation(
            f"ID{motor_id} {name} baseline split into disjoint halves: "
            f"first_half={set(first_half)} second_half={set(second_half)}"
        )


def _build_m6c_position_baseline(
    samples: list[int], motor_id: int
) -> M2dPositionBaseline:
    if not samples:
        raise SafetyViolation(f"ID{motor_id} collected 0 baseline position samples")

    for sample in samples:
        if not (0 <= sample <= 2000):
            raise SafetyViolation(
                f"ID{motor_id} baseline sample out of range: {sample}"
            )

    histogram = Counter(samples)
    most_common = histogram.most_common(2)
    if len(most_common) > 1 and most_common[0][1] == most_common[1][1]:
        raise SafetyViolation(
            f"ID{motor_id} baseline position has non-unique mode: {dict(histogram)}"
        )

    unique_mode = most_common[0][0]
    s_min = min(samples)
    s_max = max(samples)
    if s_max - s_min > 2:
        raise SafetyViolation(
            f"ID{motor_id} baseline span {s_max - s_min} > 2: min={s_min} max={s_max}"
        )

    _check_no_unidirectional_drift(samples, motor_id, name="position")

    return M2dPositionBaseline(
        reference=unique_mode,
        safety_min=s_min,
        safety_max=s_max,
        histogram=dict(histogram),
    )


def _collect_all_baselines(
    motors: dict[int, Motor],
    limits: dict[int, tuple[int, int]],
    duration_s: Optional[float] = None,
    rate_hz: Optional[float] = None,
) -> tuple[
    dict[int, M2dPositionBaseline],
    dict[int, float],
    dict[int, int],
    dict[int, int],
]:
    dur = duration_s if duration_s is not None else BASELINE_DURATION_S
    rate = rate_hz if rate_hz is not None else BASELINE_RATE_HZ
    interval = 1.0 / rate
    target_samples = max(1, int(dur * rate))
    positions: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}
    forces: dict[int, list[float]] = {motor_id: [] for motor_id in MOTOR_IDS}
    currents_max: dict[int, int] = {motor_id: 0 for motor_id in MOTOR_IDS}
    target_positions: dict[int, list[int]] = {motor_id: [] for motor_id in MOTOR_IDS}

    for sample_idx in range(1, target_samples + 1):
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            lower, upper = limits[motor_id]
            _check_status(
                status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            positions[motor_id].append(status.current_position)
            forces[motor_id].append(float(status.force_g))
            currents_max[motor_id] = max(currents_max[motor_id], status.current_mA)
            target_positions[motor_id].append(status.target_position)
            logger.info(
                "M6C_BASELINE_SAMPLE sample=%d/%d motor_id=%d actual=%d target=%d "
                "current_mA=%d force_g=%d fault=0x%02X",
                sample_idx,
                target_samples,
                motor_id,
                status.current_position,
                status.target_position,
                status.current_mA,
                status.force_g,
                status.error_code,
            )
        time.sleep(interval)

    baselines: dict[int, M2dPositionBaseline] = {}
    forces_f0: dict[int, float] = {}
    initial_targets: dict[int, int] = {}

    for motor_id in MOTOR_IDS:
        if currents_max[motor_id] != 0:
            raise SafetyViolation(
                f"ID{motor_id} baseline current max {currents_max[motor_id]}mA != 0mA"
            )
        baseline = _build_m6c_position_baseline(positions[motor_id], motor_id)
        baselines[motor_id] = baseline
        forces_f0[motor_id] = round(statistics.mean(forces[motor_id]), 1)
        initial_targets[motor_id] = target_positions[motor_id][-1]

        logger.info(
            "BASELINE_ESTABLISHED motor_id=%d reference=%d safety_min=%d "
            "safety_max=%d target_ref=%d f0=%.1f current_max_mA=%d histogram=%s",
            motor_id,
            baseline.reference,
            baseline.safety_min,
            baseline.safety_max,
            initial_targets[motor_id],
            forces_f0[motor_id],
            currents_max[motor_id],
            baseline.histogram,
        )

    return baselines, forces_f0, currents_max, initial_targets


def _log_status(
    motor_id: int,
    source: str,
    status: StatusFeedback,
    force_f0: Optional[float] = None,
    target: Optional[int] = None,
    elapsed_s: Optional[float] = None,
) -> None:
    delta_force = (
        f" delta_force_g={status.force_g - force_f0:.1f}"
        if force_f0 is not None
        else ""
    )
    tgt_info = f" target={target}" if target is not None else ""
    el_info = f" elapsed_s={elapsed_s:.3f}" if elapsed_s is not None else ""
    logger.info(
        "ID%d_STATUS source=%s actual=%d%s%s current_mA=%d force_g=%d%s "
        "temperature_C=%d fault=0x%02X",
        motor_id,
        source,
        status.current_position,
        tgt_info,
        el_info,
        status.current_mA,
        status.force_g,
        delta_force,
        status.temperature,
        status.error_code,
    )


def _pause_all_once(
    motors: dict[int, Motor],
) -> tuple[dict[int, bool], dict[int, Optional[StatusFeedback]]]:
    results: dict[int, bool] = {}
    responses: dict[int, Optional[StatusFeedback]] = {}
    for motor_id in MOTOR_IDS:
        motor = motors.get(motor_id)
        if motor is None:
            results[motor_id] = False
            responses[motor_id] = None
            continue
        try:
            response = motor.write_register(PAUSE_ADDR, [1])
            results[motor_id] = True
            responses[motor_id] = response
            logger.warning(
                "ID%d_PAUSE sent_once=true destination_id=%d actual=%d "
                "current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
                motor_id,
                motor_id,
                response.current_position,
                response.current_mA,
                response.force_g,
                response.temperature,
                response.error_code,
            )
        except Exception as exc:
            results[motor_id] = False
            responses[motor_id] = None
            logger.critical("ID%d_PAUSE_FAILED error=%s", motor_id, exc)
    if not all(results.get(motor_id, False) for motor_id in MOTOR_IDS):
        message = (
            "HARD STOP REQUIRED: M6-C one or more pause commands failed; use the "
            "hardware emergency stop or remove motor power immediately"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return results, responses


def _confirm_all_stopped(
    motors: dict[int, Motor],
    limits: dict[int, tuple[int, int]],
) -> tuple[bool, dict[int, StatusFeedback]]:
    deadline = time.monotonic() + STOP_CONFIRM_DURATION_S
    statuses: dict[int, list[StatusFeedback]] = {
        motor_id: [] for motor_id in MOTOR_IDS
    }
    while True:
        for motor_id in MOTOR_IDS:
            status = motors[motor_id].read_status()
            statuses[motor_id].append(status)
            lower, upper = limits[motor_id]
            _check_status(
                status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            _log_status(motor_id, "post_pause_read_only", status)
        if time.monotonic() >= deadline:
            break
        time.sleep(POLL_INTERVAL_S)

    stables: dict[int, bool] = {}
    reasons: dict[int, dict] = {}
    for motor_id in MOTOR_IDS:
        stable, reason = _stop_confirmation_metrics(statuses[motor_id])
        stables[motor_id] = stable
        reasons[motor_id] = reason

    overall_stable = all(stables.values())
    logger.info(
        "M6C_STOP_CONFIRMATION stable=%s "
        + " ".join(f"id{m}_stable=%s id{m}_metrics=%s" for m in MOTOR_IDS),
        overall_stable,
        *[val for m in MOTOR_IDS for val in (stables[m], reasons[m])],
    )
    if not overall_stable:
        message = (
            "HARD STOP REQUIRED: M6-C stop confirmation failed; use the "
            "hardware emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return overall_stable, {
        motor_id: statuses[motor_id][-1] for motor_id in MOTOR_IDS
    }


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe small positive motion of ID1..ID6 synchronously on one audited UART bus"
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="UART device path",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=921600,
        help="Fixed baudrate (default: 921600)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="Per-request UART timeout seconds",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="Allowed frame retries",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="Path to dedicated log file",
    )
    args = parser.parse_args(argv)
    if args.baudrate != 921600:
        parser.error("baudrate must be 921600")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if args.retries < 0:
        parser.error("retries must be non-negative")
    return args


def run(args: argparse.Namespace) -> int:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = (
        args.log
        if args.log is not None
        else ROOT
        / "logs"
        / "multi_motor"
        / "m6"
        / f"{timestamp}_m6c_joint_six_motors.log"
    )
    _configure_logging(log_path)
    logger.info("M6C_START command_line=%s", " ".join(sys.argv))

    audited_bus: Optional[M6cAuditedBus] = None
    motors: dict[int, Motor] = {}
    armed = False

    try:
        raw_bus = UartBus(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            retries=args.retries,
        )
        audited_bus = M6cAuditedBus(raw_bus)
        motors = {
            motor_id: Motor(audited_bus, motor_id=motor_id) for motor_id in MOTOR_IDS
        }

        # Step 1: Limits check
        limits: dict[int, tuple[int, int]] = {}
        for motor_id in MOTOR_IDS:
            lower, upper = _read_limits(motors[motor_id])
            if (lower, upper) != (0, 2000):
                raise SafetyViolation(
                    f"ID{motor_id} hardware limits {(lower, upper)} != (0, 2000)"
                )
            limits[motor_id] = (lower, upper)

        # Step 2: Read configs before setup
        configs_before: dict[int, ConfigSnapshot] = {
            motor_id: _read_config(motors[motor_id]) for motor_id in MOTOR_IDS
        }

        # Step 3: Collect baselines
        baselines, forces_f0, currents_max, initial_targets = _collect_all_baselines(
            motors, limits
        )

        # Step 4: Calculate targets
        targets: dict[int, int] = {}
        direction_guards: dict[int, M6cDirectionGuard] = {}
        for motor_id in MOTOR_IDS:
            target = baselines[motor_id].reference + MOVE_STEPS
            lower, upper = limits[motor_id]
            effective_upper = min(upper, SOFTWARE_UPPER_LIMIT)
            if not (lower <= target <= effective_upper):
                raise SafetyViolation(
                    f"ID{motor_id} target {target} outside safe range [{lower}, {effective_upper}]"
                )
            targets[motor_id] = target
            direction_guards[motor_id] = M6cDirectionGuard(
                motor_id=motor_id,
                reference=baselines[motor_id].reference,
                baseline_min=baselines[motor_id].safety_min,
                baseline_max=baselines[motor_id].safety_max,
                device_lower_limit=lower,
            )

        for motor_id in MOTOR_IDS:
            print(
                f"M6-C PRECHECK ID{motor_id} baseline_ref={baselines[motor_id].reference} "
                f"safety_band=[{baselines[motor_id].safety_min},{baselines[motor_id].safety_max}] "
                f"target={targets[motor_id]}"
            )

        # Step 5: Human confirmation
        expected_word = "M6-C MOVE ALL SIX MOTORS"
        prompt = (
            f"Ready to move ALL SIX MOTORS ID1..ID6 +{MOVE_STEPS} steps synchronously.\n"
            f"Targets: {targets}\n"
            f"Type '{expected_word}' to proceed: "
        )
        if input(prompt).strip() != expected_word:
            logger.warning("OPERATOR_ABORT confirmation mismatch, zero writes")
            print("ABORTED: confirmation mismatch, 0 writes executed")
            return 1

        # Armed for motion
        armed = True
        logger.info(
            "M6C_ARMING targets=%s allowed_write_registers=[0x0025,0x0029,0x001A]",
            targets,
        )

        # Step 6: Setup all 6 motors
        for motor_id in MOTOR_IDS:
            setup = motors[motor_id].write_register(
                CONTROL_MODE_ADDR,
                [
                    SPEED_MODE,
                    0,
                    0,
                    SPEED_STEPS_PER_SECOND,
                    baselines[motor_id].safety_max,
                ],
            )
            lower, upper = limits[motor_id]
            _check_status(
                setup,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
                target=baselines[motor_id].safety_max,
            )
            _log_status(
                motor_id,
                "setup_write_response",
                setup,
                force_f0=forces_f0[motor_id],
                target=baselines[motor_id].safety_max,
            )
            readback_mode = _read_value(motors[motor_id], CONTROL_MODE_ADDR)
            readback_speed = _read_value(motors[motor_id], TARGET_SPEED_ADDR)
            if readback_mode != SPEED_MODE:
                raise SafetyViolation(
                    f"ID{motor_id} mode readback {readback_mode} != {SPEED_MODE}"
                )
            if readback_speed != SPEED_STEPS_PER_SECOND:
                raise SafetyViolation(
                    f"ID{motor_id} speed readback {readback_speed} != {SPEED_STEPS_PER_SECOND}"
                )

        # Step 7: Write target position to all 6 motors
        t0_wall = datetime.now(timezone.utc).isoformat()
        t0_mono = time.monotonic()
        traces: dict[int, MotionTrace] = {
            motor_id: MotionTrace(
                baseline_min=baselines[motor_id].safety_min,
                baseline_max=baselines[motor_id].safety_max,
                target=targets[motor_id],
                force_f0=forces_f0[motor_id],
                t0_mono=t0_mono,
                t0_wall=t0_wall,
                samples=[],
            )
            for motor_id in MOTOR_IDS
        }

        for motor_id in MOTOR_IDS:
            response = motors[motor_id].write_register(
                TARGET_POSITION_ADDR, [targets[motor_id]]
            )
            error = abs(response.current_position - targets[motor_id])
            elapsed = traces[motor_id].observe(response, error, source="command")
            lower, upper = limits[motor_id]
            _check_status(
                response,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
                target=targets[motor_id],
                direction_guard=direction_guards[motor_id],
            )
            _log_status(
                motor_id,
                "target_write_response",
                response,
                force_f0=forces_f0[motor_id],
                target=targets[motor_id],
                elapsed_s=elapsed,
            )

        for motor_id in MOTOR_IDS:
            readback_target = _read_value(motors[motor_id], TARGET_POSITION_ADDR)
            if readback_target != targets[motor_id]:
                raise SafetyViolation(
                    f"ID{motor_id} target write readback mismatch: "
                    f"commanded_target={targets[motor_id]} "
                    f"readback_target_position={readback_target}"
                )

        # Step 8: Motion polling
        deadline = t0_mono + MOTION_TIMEOUT_S
        while True:
            all_ready = True
            for motor_id in MOTOR_IDS:
                status = motors[motor_id].read_status()
                error = abs(status.current_position - targets[motor_id])
                elapsed = traces[motor_id].observe(status, error)
                lower, upper = limits[motor_id]
                _check_status(
                    status,
                    lower=lower,
                    upper=upper,
                    max_current_ma=MAX_CURRENT_MA,
                    target=targets[motor_id],
                    direction_guard=direction_guards[motor_id],
                )
                _log_status(
                    motor_id,
                    "joint_motion_poll",
                    status,
                    force_f0=forces_f0[motor_id],
                    target=targets[motor_id],
                    elapsed_s=elapsed,
                )
                if error > PAUSE_TRIGGER_TOLERANCE_STEPS:
                    all_ready = False

            if all_ready:
                break

            if time.monotonic() >= deadline:
                raise SafetyViolation(
                    f"M6-C joint motion timed out after {MOTION_TIMEOUT_S}s before all six motors reached tolerance"
                )

            time.sleep(POLL_INTERVAL_S)

        # Step 9: Pause all motors
        pause_results, pause_responses = _pause_all_once(motors)
        if not all(pause_results.values()):
            raise SafetyViolation(
                "M6-C one or more motors failed pause; HARD STOP REQUIRED"
            )

        # Step 10: Stop confirmation
        all_stopped, final_statuses = _confirm_all_stopped(motors, limits)
        if not all_stopped:
            raise SafetyViolation("M6-C stop confirmation failed on one or more motors")

        # Step 11: Final acceptance verification
        final_errors: dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            final_pos = final_statuses[motor_id].current_position
            final_err = abs(final_pos - targets[motor_id])
            final_errors[motor_id] = final_err
            if final_err > FINAL_ACCEPTANCE_TOLERANCE_STEPS:
                raise SafetyViolation(
                    f"ID{motor_id} final_error={final_err} > {FINAL_ACCEPTANCE_TOLERANCE_STEPS}"
                )

        # Final write audit: 6 setup + 6 target + 6 pause = 18 writes
        if audited_bus.write_count != 18:
            raise SafetyViolation(
                f"M6-C write audit failed: write_count={audited_bus.write_count} != 18"
            )

        logger.info(
            "M6C_RESULT decision=PASS targets=%s final_positions=%s final_errors=%s "
            "pause_all=true stop_confirmation=true",
            targets,
            {m: final_statuses[m].current_position for m in MOTOR_IDS},
            final_errors,
        )
        print(
            f"PASS M6-C targets={targets} "
            f"finals={{m: final_statuses[m].current_position for m in MOTOR_IDS}} "
            f"errors={final_errors} pause_all=true stop_confirmation=true"
        )
        return 0

    except Exception as exc:
        logger.error("M6C_ABORT error=%s", exc)
        print(f"STOP M6-C: {exc}", file=sys.stderr)
        if armed and audited_bus is not None and motors:
            try:
                _pause_all_once(motors)
            except Exception as pause_exc:
                logger.critical("EMERGENCY_PAUSE_ALL_FAILED error=%s", pause_exc)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(
                "ACCESS_SUMMARY write_count=%d write_destinations=%s "
                "write_registers=%s read_counts=%s",
                audited_bus.write_count,
                audited_bus.write_destinations,
                audited_bus.write_registers,
                audited_bus.read_counts,
            )
            try:
                audited_bus.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(run(parse_args()))
