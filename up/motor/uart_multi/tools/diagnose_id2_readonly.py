#!/usr/bin/env python3
"""Stage M6: Bounded pure read-only extended diagnosis for motor ID2."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import logging
from pathlib import Path
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
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 2
TARGET_POSITION_ADDR = 0x29
DEFAULT_SAMPLE_COUNT = 500
SAMPLE_INTERVAL_S = 0.02

logger = logging.getLogger("diagnose_id2_readonly")


class ReadOnlyViolation(RuntimeError):
    """An outgoing request is outside the fixed ID2 read-only scope."""


@dataclass(frozen=True)
class Id2Sample:
    index: int
    elapsed_s: float
    target_register_raw: int
    target_register_signed: int
    status: StatusFeedback


class Id2ReadOnlyAuditedBus:
    """Audited transport wrapper ensuring strictly ZERO writes for ID2 diagnosis."""

    def __init__(self, bus: UartBus):
        self._bus = bus
        self.write_count = 0
        self.read_register_count = 0
        self.read_status_count = 0

    def request(self, frame: bytes, **kwargs) -> Frame:
        parsed = parse_frame(frame)
        if parsed.motor_id != MOTOR_ID:
            raise ReadOnlyViolation(
                f"destination ID{parsed.motor_id} is outside fixed ID2 scope"
            )
        if parsed.command == CMD_WR_REGISTER:
            self.write_count += 1
            raise ReadOnlyViolation(
                f"ID2 diagnosis is strictly read-only; write command to "
                f"register 0x{parsed.register_addr:04X} is forbidden"
            )
        if parsed.command == CMD_RD_REGISTER:
            if parsed.register_addr != TARGET_POSITION_ADDR:
                raise ReadOnlyViolation(
                    f"read register 0x{parsed.register_addr:04X} is not allowed"
                )
            self.read_register_count += 1
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X "
                "register=0x%04X",
                parsed.motor_id,
                parsed.command,
                parsed.register_addr,
            )
            return self._bus.request(frame, **kwargs)
        if parsed.command == CMD_RD_STATUS:
            self.read_status_count += 1
            logger.info(
                "ACCESS_AUDIT operation=READ destination_id=%d command=0x%02X "
                "register=0x0000",
                parsed.motor_id,
                parsed.command,
            )
            return self._bus.request(frame, **kwargs)
        raise ReadOnlyViolation(f"unsupported command 0x{parsed.command:02X}")

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


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/multi_motor/m6/"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_id2_readonly_diagnosis.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create log {log_path}: {exc}", file=sys.stderr)
        return 1

    bus: Optional[UartBus] = None
    audited_bus: Optional[Id2ReadOnlyAuditedBus] = None

    try:
        bus = UartBus(args.port, timeout=args.timeout, retries=args.retries)
        audited_bus = Id2ReadOnlyAuditedBus(bus)
        motor = Motor(audited_bus, MOTOR_ID)

        samples_count = args.samples
        samples: list[Id2Sample] = []
        t0 = time.monotonic()

        for idx in range(1, samples_count + 1):
            now_mono = time.monotonic()
            elapsed_s = now_mono - t0

            # 1. Read register 0x29 (target position)
            reg_resp = motor.read_register(TARGET_POSITION_ADDR)
            raw_u16 = reg_resp.values[0]
            # Convert unsigned 16-bit to signed if needed
            signed_target = raw_u16 if raw_u16 <= 32767 else raw_u16 - 65536

            # 2. Read status 0x30
            st = motor.read_status()

            sample = Id2Sample(
                index=idx,
                elapsed_s=elapsed_s,
                target_register_raw=raw_u16,
                target_register_signed=signed_target,
                status=st,
            )
            samples.append(sample)

            logger.info(
                "ID2_SAMPLE idx=%d/%d elapsed_s=%.3f reg_target=%d "
                "status_target=%d actual=%d current_mA=%d force_g=%d "
                "temp_C=%d fault=0x%02X",
                idx,
                samples_count,
                elapsed_s,
                signed_target,
                st.target_position,
                st.current_position,
                st.current_mA,
                st.force_g,
                st.temperature,
                st.error_code,
            )

            if idx < samples_count:
                time.sleep(SAMPLE_INTERVAL_S)

        # Statistical analysis
        actual_positions = [s.status.current_position for s in samples]
        status_targets = [s.status.target_position for s in samples]
        reg_targets = [s.target_register_signed for s in samples]
        currents = [s.status.current_mA for s in samples]
        faults = {s.status.error_code for s in samples}

        actual_hist = dict(sorted(Counter(actual_positions).items()))
        status_target_hist = dict(sorted(Counter(status_targets).items()))
        reg_target_hist = dict(sorted(Counter(reg_targets).items()))

        act_min = min(actual_positions)
        act_max = max(actual_positions)
        curr_max = max(currents)

        count_le_64 = sum(1 for p in actual_positions if p <= 64)
        count_65 = sum(1 for p in actual_positions if p == 65)
        count_66 = sum(1 for p in actual_positions if p == 66)
        count_67 = sum(1 for p in actual_positions if p == 67)
        count_ge_68 = sum(1 for p in actual_positions if p >= 68)

        # Classification
        if count_le_64 > 0 or count_ge_68 > 0 or len(status_target_hist) > 1:
            classification = "C_DEEPER_DRIFT_OR_TARGET_SHIFT"
        elif count_65 > 0:
            classification = "B_STABLE_LOW_PROB_2STEP_STATIC_JITTER"
        else:
            classification = "A_SINGLE_OUTLIER_NO_65_IN_SAMPLE"

        logger.info(
            "DIAGNOSIS_SUMMARY classification=%s samples=%d actual_min=%d "
            "actual_max=%d actual_hist=%s reg_target_hist=%s status_target_hist=%s "
            "count_le_64=%d count_65=%d count_66=%d count_67=%d count_ge_68=%d "
            "current_max_mA=%d faults=%s",
            classification,
            samples_count,
            act_min,
            act_max,
            actual_hist,
            reg_target_hist,
            status_target_hist,
            count_le_64,
            count_65,
            count_66,
            count_67,
            count_ge_68,
            curr_max,
            [f"0x{f:02X}" for f in faults],
        )

        print(
            f"DIAGNOSIS_RESULT classification={classification} samples={samples_count} "
            f"actual_hist={actual_hist} reg_targets={reg_target_hist} "
            f"status_targets={status_target_hist} current_max={curr_max}mA "
            f"faults={faults}"
        )
        return 0

    except Exception as exc:
        logger.error("DIAGNOSIS_ABORT error=%s", exc)
        print(f"STOP DIAGNOSIS: {exc}", file=sys.stderr)
        return 1
    finally:
        if audited_bus is not None:
            logger.info(
                "ACCESS_SUMMARY write_count=%d read_reg=%d read_status=%d",
                audited_bus.write_count,
                audited_bus.read_register_count,
                audited_bus.read_status_count,
            )
            try:
                audited_bus.close()
            except Exception as close_exc:
                logger.error("CLOSE_ERROR %s", close_exc)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stage M6: pure read-only extended diagnosis for ID2"
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="serial device path",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help="number of read-only samples (default 500)",
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
    if args.samples <= 0:
        parser.error("samples must be positive")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
