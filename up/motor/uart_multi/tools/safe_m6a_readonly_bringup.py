#!/usr/bin/env python3
"""Stage M6-A: read-only six-motor (ID1~ID6) state review on one audited UART bus."""

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
    SafetyViolation,
    _check_status,
)
from uart_bus import UartBus  # noqa: E402


MOTOR_IDS = (1, 2, 3, 4, 5, 6)
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
LOWER_LIMIT_ADDR = 0x24
UPPER_LIMIT_ADDR = 0x23
ALLOWED_READ_REGISTERS = frozenset(
    (
        CONTROL_MODE_ADDR,
        TARGET_SPEED_ADDR,
        TARGET_POSITION_ADDR,
        LOWER_LIMIT_ADDR,
        UPPER_LIMIT_ADDR,
    )
)
DEFAULT_SAMPLE_COUNT = 100
SAMPLE_INTERVAL_S = 0.05
MAX_CURRENT_MA = 500

logger = logging.getLogger("safe_m6a_readonly_bringup")


class M6aReadOnlyAuditedBus:
    """Audited transport wrapper ensuring strictly ZERO writes for Stage M6-A."""

    def __init__(self, bus: UartBus):
        self._bus = bus
        self.write_count = 0
        self.read_counts: dict[int, int] = {motor_id: 0 for motor_id in MOTOR_IDS}

    def request(self, frame: bytes, **kwargs) -> Frame:
        parsed = parse_frame(frame)
        if parsed.motor_id not in MOTOR_IDS:
            raise SafetyViolation(
                f"M6-A destination ID{parsed.motor_id} is outside allowed scope {MOTOR_IDS}"
            )
        if parsed.command == CMD_WR_REGISTER:
            self.write_count += 1
            raise SafetyViolation(
                f"M6-A is strictly read-only; write command to ID{parsed.motor_id} "
                f"register 0x{parsed.register_addr:04X} is forbidden"
            )
        if parsed.command in (CMD_RD_REGISTER, CMD_RD_STATUS):
            if (
                parsed.command == CMD_RD_REGISTER
                and parsed.register_addr not in ALLOWED_READ_REGISTERS
            ):
                raise SafetyViolation(
                    f"M6-A read register 0x{parsed.register_addr:04X} is not allowed"
                )
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


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m6/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_m6a_readonly_review.log"
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
    audited_bus: Optional[M6aReadOnlyAuditedBus] = None
    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = M6aReadOnlyAuditedBus(bus)
        motors = {
            motor_id: Motor(audited_bus, motor_id) for motor_id in MOTOR_IDS
        }

        # Step 1: Preflight configuration audit (Limits, Mode, Speed, Target)
        configs: dict[int, dict[str, int]] = {}
        for motor_id in MOTOR_IDS:
            lower = _read_value(motors[motor_id], LOWER_LIMIT_ADDR)
            upper = _read_value(motors[motor_id], UPPER_LIMIT_ADDR)
            if lower != 0 or upper != 2000:
                raise SafetyViolation(
                    f"ID{motor_id} limits [{lower},{upper}] must equal [0,2000]"
                )
            mode = _read_value(motors[motor_id], CONTROL_MODE_ADDR)
            speed = _read_value(motors[motor_id], TARGET_SPEED_ADDR)
            target = _read_value(motors[motor_id], TARGET_POSITION_ADDR)
            configs[motor_id] = {
                "lower": lower,
                "upper": upper,
                "mode": mode,
                "speed": speed,
                "target": target,
            }
            logger.info(
                "M6A_PREFLIGHT_CONFIG motor_id=%d lower=%d upper=%d mode=%d speed=%d target=%d",
                motor_id,
                lower,
                upper,
                mode,
                speed,
                target,
            )

        # Step 2: Sample collection (status and target over N rounds)
        samples_count = args.samples
        position_samples: dict[int, list[int]] = {m: [] for m in MOTOR_IDS}
        current_samples: dict[int, list[int]] = {m: [] for m in MOTOR_IDS}
        force_samples: dict[int, list[int]] = {m: [] for m in MOTOR_IDS}
        fault_samples: dict[int, list[int]] = {m: [] for m in MOTOR_IDS}

        for round_idx in range(1, samples_count + 1):
            for motor_id in MOTOR_IDS:
                status = motors[motor_id].read_status()
                _check_status(
                    status,
                    lower=0,
                    upper=2000,
                    max_current_ma=MAX_CURRENT_MA,
                )
                position_samples[motor_id].append(status.current_position)
                current_samples[motor_id].append(status.current_mA)
                force_samples[motor_id].append(status.force_g)
                fault_samples[motor_id].append(status.error_code)
                logger.info(
                    "M6A_SAMPLE round=%d/%d motor_id=%d actual=%d target=%d "
                    "current_mA=%d force_g=%d temp_C=%d fault=0x%02X",
                    round_idx,
                    samples_count,
                    motor_id,
                    status.current_position,
                    status.target_position,
                    status.current_mA,
                    status.force_g,
                    status.temperature,
                    status.error_code,
                )
            if round_idx < samples_count:
                time.sleep(SAMPLE_INTERVAL_S)

        # Step 3: Statistical summary & classification
        stats_summary: list[str] = []
        for motor_id in MOTOR_IDS:
            pos_hist = dict(sorted(Counter(position_samples[motor_id]).items()))
            pos_min = min(position_samples[motor_id])
            pos_max = max(position_samples[motor_id])
            curr_max = max(current_samples[motor_id])
            faults = set(fault_samples[motor_id])
            logger.info(
                "M6A_SUMMARY motor_id=%d position_min=%d position_max=%d "
                "position_hist=%s current_max_mA=%d faults=%s",
                motor_id,
                pos_min,
                pos_max,
                pos_hist,
                curr_max,
                [f"0x{f:02X}" for f in faults],
            )
            stats_summary.append(
                f"ID{motor_id}(pos={pos_min}~{pos_max},curr_max={curr_max}mA,fault={faults})"
            )

        if audited_bus.write_count != 0:
            raise SafetyViolation(
                f"M6-A write audit failed: write_count={audited_bus.write_count} must be 0"
            )

        logger.info(
            "M6A_RESULT decision=PASS write_count=0 samples=%d motors=6",
            samples_count,
        )
        print(
            f"PASS M6-A samples={samples_count} write_count=0 "
            + " ".join(stats_summary)
        )
        return 0
    except Exception as exc:
        logger.error("M6A_ABORT error=%s", exc)
        print(f"STOP M6-A: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(
                "ACCESS_SUMMARY write_count=%d read_counts=%s",
                audited_bus.write_count,
                audited_bus.read_counts,
            )
            try:
                audited_bus.close()
            except Exception as close_exc:
                logger.error("CLOSE_ERROR %s", close_exc)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage M6-A: safe read-only six-motor state review"
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
        "--samples",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help="number of read-only sample rounds",
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
    if args.samples <= 0:
        parser.error("samples must be positive")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
