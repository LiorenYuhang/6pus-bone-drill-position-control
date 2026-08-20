#!/usr/bin/env python3
"""Stage M4-C: move ID1, ID2, ID3, and ID4 synchronously once on one audited UART bus."""

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
    BaselineBandGuard,
    MotionTrace,
    SafetyViolation,
    _check_status,
    _stop_confirmation_metrics,
)
from tools.safe_m2d_joint_id1_id2 import (  # noqa: E402
    M2dPositionBaseline,
    _build_m2d_position_baseline,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (1, 2, 3, 4)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
SPEED_MODE = 2
MOVE_STEPS = 10
SPEED_STEPS_PER_SECOND = 50
MOTION_TIMEOUT_S = 2.0
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

logger = logging.getLogger("safe_m4c_joint_four_motors")


@dataclass(frozen=True)
class ConfigSnapshot:
    mode: int
    speed: int
    target: int


class M4cAuditedBus:
    """Audited transport wrapper for Stage M4-C four-motor joint motion."""

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
                f"M4-C destination ID{parsed.motor_id} is outside allowed scope {MOTOR_IDS}"
            )
        if parsed.command == CMD_WR_REGISTER:
            if parsed.register_addr not in ALLOWED_WRITE_REGISTERS:
                raise SafetyViolation(
                    f"M4-C write register 0x{parsed.register_addr:04X} is not allowed"
                )
            self.write_frames.append(frame)
            self.write_destinations.append(parsed.motor_id)
            self.write_registers.append(f"0x{parsed.register_addr:04X}")
            logger.warning(
                "ACCESS_AUDIT operation=WRITE destination_id=%d register=0x%04X "
                "data=%s verified=true",
                parsed.motor_id,
                parsed.register_addr,
                " ".join(f"{b:02X}" for b in parsed.data),
            )
            return self._bus.request(frame, **kwargs)
        if parsed.command in (CMD_RD_REGISTER, CMD_RD_STATUS):
            self.read_counts[parsed.motor_id] = (
                self.read_counts.get(parsed.motor_id, 0) + 1
            )
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X "
                "register=0x%04X",
                parsed.motor_id,
                parsed.command,
                parsed.register_addr or 0,
            )
            return self._bus.request(frame, **kwargs)
        raise SafetyViolation(f"unsupported command 0x{parsed.command:02X}")

    def close(self) -> None:
        self._bus.close()


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists():
        raise RuntimeError(f"log file already exists: {log_path}")

    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    bus_logger = logging.getLogger("uart_bus")
    bus_logger.setLevel(logging.INFO)
    bus_logger.handlers.clear()
    bus_logger.addHandler(file_handler)


def _read_value(motor: Motor, address: int) -> int:
    return motor.read_register(address).values[0]


def _read_config(motor: Motor) -> ConfigSnapshot:
    mode = _read_value(motor, CONTROL_MODE_ADDR)
    speed = _read_value(motor, TARGET_SPEED_ADDR)
    target = _read_value(motor, TARGET_POSITION_ADDR)
    return ConfigSnapshot(mode=mode, speed=speed, target=target)


def _validate_limits(motor_id: int, lower: int, upper: int) -> None:
    if lower != 0 or upper != 2000:
        raise SafetyViolation(
            f"ID{motor_id} limits [{lower},{upper}] must equal [0,2000]"
        )


def _log_status(
    motor_id: int,
    source: str,
    status: StatusFeedback,
    *,
    force_f0: Optional[float] = None,
    target: Optional[int] = None,
    elapsed_s: Optional[float] = None,
) -> None:
    delta_force = (
        f"{status.force_g - force_f0:.3f}" if force_f0 is not None else "-"
    )
    logger.info(
        "ID%d_STATUS source=%s elapsed_s=%s target=%s actual=%d error=%s "
        "current_mA=%d force_g=%d delta_force_g=%s temperature_C=%d "
        "fault=0x%02X",
        motor_id,
        source,
        f"{elapsed_s:.6f}" if elapsed_s is not None else "-",
        str(target) if target is not None else "-",
        status.current_position,
        str(abs(status.current_position - target))
        if target is not None
        else "-",
        status.current_mA,
        status.force_g,
        delta_force,
        status.temperature,
        status.error_code,
    )


def _collect_baseline(
    motor: Motor,
    lower: int,
    upper: int,
) -> tuple[M2dPositionBaseline, float, int]:
    samples_count = int(BASELINE_DURATION_S * BASELINE_RATE_HZ)
    interval_s = 1.0 / BASELINE_RATE_HZ
    positions: list[int] = []
    forces: list[int] = []
    currents: list[int] = []
    for _ in range(samples_count):
        status = motor.read_status()
        _check_status(
            status,
            lower=lower,
            upper=upper,
            max_current_ma=MAX_CURRENT_MA,
        )
        positions.append(status.current_position)
        forces.append(status.force_g)
        currents.append(status.current_mA)
        time.sleep(interval_s)
    return (
        _build_m2d_position_baseline(positions),
        statistics.fmean(forces),
        max(currents),
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
            "HARD STOP REQUIRED: M4-C one or more pause commands failed; use the "
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
    reasons: dict[int, str] = {}
    for motor_id in MOTOR_IDS:
        stable, reason = _stop_confirmation_metrics(statuses[motor_id])
        stables[motor_id] = stable
        reasons[motor_id] = reason

    overall_stable = all(stables.values())
    logger.info(
        "M4C_STOP_CONFIRMATION stable=%s "
        + " ".join(f"id{m}_stable=%s id{m}_reason=%s" for m in MOTOR_IDS),
        overall_stable,
        *[val for m in MOTOR_IDS for val in (stables[m], reasons[m])],
    )
    if not overall_stable:
        message = (
            "HARD STOP REQUIRED: M4-C stop confirmation failed; use the "
            "hardware emergency stop or remove motor power"
        )
        logger.critical(message)
        print(message, file=sys.stderr)
    return overall_stable, {
        motor_id: statuses[motor_id][-1] for motor_id in MOTOR_IDS
    }


def _confirm_operator() -> None:
    answer = input(
        "确认仅连接并固定 ID1~ID4、全四台电机正方向区域安全、"
        "急停可用；输入 M4-C MOVE ALL FOUR MOTORS："
    ).strip()
    if answer != "M4-C MOVE ALL FOUR MOTORS":
        raise SafetyViolation("operator did not arm Stage M4-C")


def _hard_stop_for_confirmation_exception(exc: Exception) -> None:
    message = (
        "HARD STOP REQUIRED: M4-C post-pause read-only confirmation failed; "
        "use the hardware emergency stop or remove motor power"
    )
    logger.critical("%s: %s", message, exc)
    print(message, file=sys.stderr)


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m4/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m4c_joint_four_motors.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(
            f"ERROR cannot create independent log {log_path}: {exc}",
            file=sys.stderr,
        )
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[M4cAuditedBus] = None
    motors: dict[int, Motor] = {}
    limits: dict[int, tuple[int, int]] = {}
    configs_before: dict[int, ConfigSnapshot] = {}
    traces: dict[int, MotionTrace] = {}
    targets: dict[int, int] = {}
    armed = False
    pause_attempted = False
    pause_results: dict[int, bool] = {}
    confirmation_attempted = False
    stop_confirmation: Optional[bool] = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = M4cAuditedBus(bus)
        motors = {
            motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS
        }

        for motor_id in MOTOR_IDS:
            lower = _read_value(motors[motor_id], LOWER_LIMIT_ADDR)
            upper = _read_value(motors[motor_id], UPPER_LIMIT_ADDR)
            _validate_limits(motor_id, lower, upper)
            limits[motor_id] = (lower, upper)
            configs_before[motor_id] = _read_config(motors[motor_id])
            initial = motors[motor_id].read_status()
            _check_status(
                initial,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            _log_status(motor_id, "preflight", initial)

        baselines: dict[int, M2dPositionBaseline] = {}
        forces_f0: dict[int, float] = {}
        currents_max_mA: dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            lower, upper = limits[motor_id]
            base_pos, force_f0, curr_max = _collect_baseline(
                motors[motor_id], lower, upper
            )
            baselines[motor_id] = base_pos
            forces_f0[motor_id] = force_f0
            currents_max_mA[motor_id] = curr_max
            target = base_pos.reference + MOVE_STEPS
            effective_upper = min(upper, SOFTWARE_UPPER_LIMIT)
            if not lower <= target <= effective_upper:
                raise SafetyViolation(
                    f"ID{motor_id} target={target} outside allowed "
                    f"[{lower},{effective_upper}]"
                )
            targets[motor_id] = target

        for motor_id in MOTOR_IDS:
            baseline = baselines[motor_id]
            logger.info(
                "M4C_PRECHECK ID%d config=%s limits=[%d,%d] "
                "baseline_reference=%d baseline_safety_min=%d "
                "baseline_safety_max=%d baseline_histogram=%s "
                "force_f0=%.3f baseline_current_max_mA=%d target=%d "
                "speed=%d timeout_s=%.3f max_current_mA=%d",
                motor_id,
                configs_before[motor_id],
                limits[motor_id][0],
                limits[motor_id][1],
                baseline.reference,
                baseline.safety_min,
                baseline.safety_max,
                baseline.histogram,
                forces_f0[motor_id],
                currents_max_mA[motor_id],
                targets[motor_id],
                SPEED_STEPS_PER_SECOND,
                MOTION_TIMEOUT_S,
                MAX_CURRENT_MA,
            )
        print(
            "M4-C PRECHECK "
            + " ".join(
                f"ID{m}_ref={baselines[m].reference} ID{m}_target={targets[m]}"
                for m in MOTOR_IDS
            )
        )
        _confirm_operator()

        logger.warning(
            "M4C_ARMING moving_ids=%s "
            "allowed_write_registers=[0x0025,0x0029,0x001A]",
            list(MOTOR_IDS),
        )
        armed = True

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

        t0_mono = time.monotonic()
        t0_wall = datetime.now(timezone.utc).isoformat()
        traces = {
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
        direction_guards = {
            motor_id: BaselineBandGuard(
                baseline_min=baselines[motor_id].safety_min,
                baseline_max=baselines[motor_id].safety_max,
            )
            for motor_id in MOTOR_IDS
        }
        target_write_times: dict[int, float] = {}
        target_write_responses: dict[int, StatusFeedback] = {}
        for motor_id in MOTOR_IDS:
            target_write_times[motor_id] = time.monotonic()
            response = motors[motor_id].write_register(
                TARGET_POSITION_ADDR, [targets[motor_id]]
            )
            target_write_responses[motor_id] = response
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
            logger.info(
                "ID%d_TARGET_WRITE_READBACK commanded_target=%d "
                "readback_target_position=%d actual_position=%d current_mA=%d "
                "force_g=%d fault=0x%02X",
                motor_id,
                targets[motor_id],
                readback_target,
                target_write_responses[motor_id].current_position,
                target_write_responses[motor_id].current_mA,
                target_write_responses[motor_id].force_g,
                target_write_responses[motor_id].error_code,
            )
        dispatch_skew = target_write_times[4] - target_write_times[1]
        logger.info(
            "M4C_MOTION_T0 t0=%s targets=%s target_dispatch_skew_s=%.6f",
            t0_wall,
            targets,
            dispatch_skew,
        )

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
                positive_motion = (
                    traces[motor_id].first_motion_mono is not None
                )
                all_ready = (
                    all_ready
                    and positive_motion
                    and error <= PAUSE_TRIGGER_TOLERANCE_STEPS
                )
            if all_ready:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SafetyViolation(
                    "ID1~ID4 did not all confirm positive motion and reach "
                    f"+/-{PAUSE_TRIGGER_TOLERANCE_STEPS} step within "
                    f"{MOTION_TIMEOUT_S:.2f}s"
                )
            time.sleep(min(POLL_INTERVAL_S, remaining))

        pause_attempted = True
        pause_results, pause_responses = _pause_all_once(motors)
        for motor_id, response in pause_responses.items():
            if response is not None:
                traces[motor_id].set_pause_status(response)
        if not all(pause_results.get(m, False) for m in MOTOR_IDS):
            raise SafetyViolation("M4-C completion pause failed")

        confirmation_attempted = True
        try:
            stop_confirmation, final_statuses = _confirm_all_stopped(
                motors, limits
            )
        except Exception as exc:
            _hard_stop_for_confirmation_exception(exc)
            stop_confirmation = False
            raise
        for motor_id, status in final_statuses.items():
            traces[motor_id].set_post_pause_status(status)
        if not stop_confirmation:
            raise SafetyViolation("M4-C stop confirmation failed")

        final_errors: dict[int, int] = {}
        for motor_id in MOTOR_IDS:
            final_status = final_statuses[motor_id]
            lower, upper = limits[motor_id]
            _check_status(
                final_status,
                lower=lower,
                upper=upper,
                max_current_ma=MAX_CURRENT_MA,
            )
            final_errors[motor_id] = abs(
                final_status.current_position - targets[motor_id]
            )
            if final_errors[motor_id] > FINAL_ACCEPTANCE_TOLERANCE_STEPS:
                raise SafetyViolation(
                    f"ID{motor_id} post-pause final_error={final_errors[motor_id]} "
                    f"exceeds {FINAL_ACCEPTANCE_TOLERANCE_STEPS} step"
                )
            post_pause_target = _read_value(
                motors[motor_id], TARGET_POSITION_ADDR
            )
            logger.info(
                "ID%d_POST_PAUSE_STATE commanded_target=%d "
                "readback_target_position=%d actual_position=%d error=%d "
                "current_mA=%d force_g=%d fault=0x%02X "
                "target_sync_observation_only=true",
                motor_id,
                targets[motor_id],
                post_pause_target,
                final_status.current_position,
                final_errors[motor_id],
                final_status.current_mA,
                final_status.force_g,
                final_status.error_code,
            )

        logger.info(
            "M4C_RESULT decision=PASS "
            + " ".join(
                f"ID{m}_target={targets[m]} ID{m}_final={final_statuses[m].current_position} ID{m}_error={final_errors[m]}"
                for m in MOTOR_IDS
            )
            + " final_tolerance_pass=true pause_all=true stop_confirmation=true",
        )
        for motor_id in MOTOR_IDS:
            logger.info(
                "ID%d_%s",
                motor_id,
                traces[motor_id].result_line(
                    decision="PASS",
                    pause_triggered=True,
                    pause_success=pause_results[motor_id],
                    stop_confirmation=stop_confirmation,
                ),
            )
        print(
            "PASS M4-C "
            + " ".join(
                f"ID{m}_final={final_statuses[m].current_position}(err={final_errors[m]})"
                for m in MOTOR_IDS
            )
            + " pause_all=true stop_confirmation=true"
        )
        return 0
    except Exception as exc:
        if armed and not pause_attempted and motors:
            pause_attempted = True
            pause_results, pause_responses = _pause_all_once(motors)
            for motor_id, response in pause_responses.items():
                if response is not None and motor_id in traces:
                    traces[motor_id].set_pause_status(response)
        logger.error("M4C_ABORT error=%s", exc)
        print(f"STOP M4-C: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(
                "ACCESS_SUMMARY write_count=%d write_destinations=%s "
                "write_registers=%s reads=%s",
                audited_bus.write_count,
                audited_bus.write_destinations,
                audited_bus.write_registers,
                audited_bus.read_counts,
            )
            try:
                audited_bus.close()
            except Exception as close_exc:
                logger.error("CLOSE_ERROR %s", close_exc)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage M4-C: safe four-motor joint motion"
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="serial device path",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="serial read timeout in seconds",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="serial read retries",
    )
    parser.add_argument(
        "--log",
        default="",
        help="path to log file",
    )
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if args.retries < 0:
        parser.error("retries must be non-negative")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
