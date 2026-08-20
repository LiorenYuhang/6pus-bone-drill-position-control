#!/usr/bin/env python3
"""Conservative single-motor ID1 micro-motion acceptance test.

Motion values are intentionally fixed to the first safety envelope: 50
step/s and five steps around the measured starting position. The tool uses the
official registers 0x25, 0x28, 0x29 and read-only status/limit registers; it
does not expose a general motion or trajectory interface.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import Register, StatusFeedback  # noqa: E402
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 1
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
PAUSE_ADDR = 0x1A
POSITION_MODE = 0
SPEED_MODE = 2
SAFE_SPEED_STEPS_PER_SECOND = 50
MICRO_STEP = 5
SETTLE_TOLERANCE_STEPS = 1
SOFTWARE_LOWER_LIMIT = 200
SOFTWARE_UPPER_LIMIT = 1800

logger = logging.getLogger("safe_micro_move_id1")


class SafetyViolation(RuntimeError):
    """A safety precondition or runtime limit was violated."""


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    # Exclusive creation prevents an operator mistake from overwriting a run.
    handler = logging.FileHandler(log_path, mode="x", encoding="utf-8")
    handler.setFormatter(formatter)
    for target in (logger, logging.getLogger("uart_bus")):
        target.setLevel(logging.INFO)
        target.handlers.clear()
        target.addHandler(handler)
        target.propagate = False


def _read_value(motor: Motor, register_addr: int) -> int:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise SafetyViolation(
            f"register 0x{register_addr:02X} returned {len(response.values)} values"
        )
    return response.values[0]


def _check_status(
    status: StatusFeedback,
    args: argparse.Namespace,
    *,
    target: Optional[int] = None,
) -> int:
    if status.error_code != 0:
        raise SafetyViolation(f"fault code=0x{status.error_code:02X}")
    if status.current_mA > args.max_current_ma:
        raise SafetyViolation(
            f"current={status.current_mA}mA exceeds {args.max_current_ma}mA"
        )
    if abs(status.force_g) > args.max_force_g:
        raise SafetyViolation(
            f"force={status.force_g}g exceeds {args.max_force_g}g"
        )
    error_steps = abs(status.current_position - target) if target is not None else 0
    if target is not None and error_steps > args.max_error_steps:
        raise SafetyViolation(
            f"tracking error={error_steps} steps exceeds {args.max_error_steps}"
        )
    return error_steps


def _log_status(label: str, status: StatusFeedback, target: Optional[int]) -> None:
    error_steps = (
        abs(status.current_position - target) if target is not None else "-"
    )
    logger.info(
        "%s target=%s actual=%d error=%s current_mA=%d force_g=%d temperature_C=%d fault=0x%02X",
        label,
        target if target is not None else "-",
        status.current_position,
        error_steps,
        status.current_mA,
        status.force_g,
        status.temperature,
        status.error_code,
    )


def _pause_once(motor: Optional[Motor]) -> None:
    if motor is None:
        return
    try:
        # Official pause command, one write only and never retried.
        motor.write_register(PAUSE_ADDR, [1])
        logger.warning("safety pause command sent once")
    except Exception as exc:
        logger.error("safety pause command failed: %s", exc)


def _wait_for_target(
    motor: Motor,
    target: int,
    args: argparse.Namespace,
) -> None:
    deadline = time.monotonic() + args.settle_timeout
    while True:
        status = motor.read_status()
        error_steps = _check_status(status, args, target=target)
        _log_status("poll", status, target)
        if error_steps <= SETTLE_TOLERANCE_STEPS:
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise SafetyViolation(
                f"target={target} did not settle within {args.settle_timeout:.2f}s"
            )
        time.sleep(min(args.poll_interval, remaining))


def _confirm_motion(
    p0: int,
    targets: list[int],
    effective_lower: int,
    effective_upper: int,
    initial: StatusFeedback,
) -> None:
    print(
        f"PRECHECK ID1 p0={p0} targets={targets} "
        f"limits=[{effective_lower}, {effective_upper}] "
        f"current={initial.current_mA}mA force={initial.force_g}g "
        f"temperature={initial.temperature}C fault=0x{initial.error_code:02X}"
    )
    answer = input("确认只连接并固定 ID1、急停可用、运动区域安全；输入 MOVE ID1：").strip()
    if answer != "MOVE ID1":
        raise SafetyViolation("operator did not arm ID1 micro-motion")


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        f"logs/id1_motion/id1_micro_move_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    motor: Optional[Motor] = None
    try:
        logger.info(
            "ID1 micro-motion start speed=%d step=%d min=%d max=%d max_current=%d max_force=%d max_error=%d",
            SAFE_SPEED_STEPS_PER_SECOND,
            MICRO_STEP,
            SOFTWARE_LOWER_LIMIT,
            SOFTWARE_UPPER_LIMIT,
            args.max_current_ma,
            args.max_force_g,
            args.max_error_steps,
        )
        with UartBus(args.port, timeout=args.timeout, retries=args.retries) as bus:
            motor = Motor(bus, MOTOR_ID)
            hardware_lower = _read_value(motor, LOWER_LIMIT_ADDR)
            hardware_upper = _read_value(motor, UPPER_LIMIT_ADDR)
            initial_mode = _read_value(motor, CONTROL_MODE_ADDR)
            initial_speed = _read_value(motor, TARGET_SPEED_ADDR)
            initial_target = _read_value(motor, TARGET_POSITION_ADDR)
            logger.info(
                "preflight config mode=%d speed=%d target_position=%d hardware_lower=%d hardware_upper=%d",
                initial_mode, initial_speed, initial_target, hardware_lower, hardware_upper,
            )
            print(
                f"CONFIG BEFORE mode={initial_mode} speed={initial_speed} "
                f"target={initial_target} device_limits=[{hardware_lower}, {hardware_upper}]"
            )
            if not 0 <= hardware_lower <= hardware_upper <= 2000:
                raise SafetyViolation(
                    f"invalid hardware limits lower={hardware_lower} upper={hardware_upper}"
                )
            effective_lower = max(SOFTWARE_LOWER_LIMIT, hardware_lower)
            effective_upper = min(SOFTWARE_UPPER_LIMIT, hardware_upper)
            if effective_lower > effective_upper:
                raise SafetyViolation("software and hardware position limits do not overlap")

            initial = motor.read_status()
            p0 = initial.current_position
            _check_status(initial, args)
            _log_status("initial", initial, None)
            targets = [p0, p0 + MICRO_STEP, p0, p0 - MICRO_STEP, p0]
            if any(target < effective_lower or target > effective_upper for target in targets):
                raise SafetyViolation(
                    f"micro-motion targets {targets} exceed effective limits "
                    f"[{effective_lower}, {effective_upper}]"
                )
            _confirm_motion(
                p0, targets, effective_lower, effective_upper, initial
            )

            # Official manual method 2: mode, unused fields, speed and target
            # are written contiguously so entering speed mode cannot reuse an
            # unrelated old target.
            setup = motor.write_register(
                CONTROL_MODE_ADDR,
                [SPEED_MODE, 0, 0, SAFE_SPEED_STEPS_PER_SECOND, p0],
            )
            _check_status(setup, args, target=p0)
            _log_status("setup", setup, p0)

            for index, target in enumerate(targets[1:], start=1):
                response = motor.write_register(TARGET_POSITION_ADDR, [target])
                _check_status(response, args, target=target)
                _log_status(f"command{index}", response, target)
                _wait_for_target(motor, target, args)
            final_mode = _read_value(motor, CONTROL_MODE_ADDR)
            final_speed = _read_value(motor, TARGET_SPEED_ADDR)
            final_target = _read_value(motor, TARGET_POSITION_ADDR)
            final_status = motor.read_status()
            _check_status(final_status, args, target=p0)
            _log_status("final", final_status, p0)
            if (
                final_mode != SPEED_MODE
                or final_speed != SAFE_SPEED_STEPS_PER_SECOND
                or final_target != p0
            ):
                raise SafetyViolation(
                    f"unexpected final config mode={final_mode} speed={final_speed} "
                    f"target={final_target}; expected mode={SPEED_MODE} "
                    f"speed={SAFE_SPEED_STEPS_PER_SECOND} target={p0}"
                )
            logger.info(
                "final config mode=%d speed=%d target_position=%d restored=false",
                final_mode, final_speed, final_target,
            )
            print(
                f"CONFIG FINAL mode={final_mode} speed={final_speed} "
                f"target={final_target} restored=false"
            )

        logger.info("ID1 micro-motion PASS targets=%s", targets)
        print(f"PASS ID1 p0={p0} targets={targets} speed={SAFE_SPEED_STEPS_PER_SECOND} final_mode={final_mode} final_speed={final_speed}")
        return 0
    except Exception as exc:
        _pause_once(motor)
        logger.error("ID1 micro-motion STOP: %s", exc)
        print(f"STOP ID1: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ID1 固定 ±5 步安全极小位移测试")
    parser.add_argument("--port", required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--max-current-ma", type=int, required=True)
    parser.add_argument("--max-force-g", type=int, required=True)
    parser.add_argument("--max-error-steps", type=int, default=10)
    parser.add_argument("--settle-timeout", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.02)
    parser.add_argument("--log", help="独立日志路径；已有文件不会覆盖")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.settle_timeout <= 0 or args.poll_interval <= 0:
        parser.error("timeout、settle-timeout、poll-interval 必须大于 0")
    if args.retries < 0:
        parser.error("--retries 不能小于 0")
    if args.max_current_ma <= 0 or args.max_force_g <= 0:
        parser.error("电流和受力安全阈值必须大于 0")
    if args.max_error_steps < SETTLE_TOLERANCE_STEPS:
        parser.error("--max-error-steps 不能小于 1")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
