#!/usr/bin/env python3
"""Parameterized read-only polling test for one shared UART bus."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import (  # noqa: E402
    FrameChecksumError,
    FrameFieldError,
    FrameLengthError,
    Register,
)
from uart_bus import (  # noqa: E402
    BusError,
    NoResponseError,
    RetryExhaustedError,
    UartBus,
)


ROUND_COUNT = 100
logger = logging.getLogger("probe_two_ids")


class WrongIdReadback(BusError):
    """The ID register value did not match the addressed motor ID."""


@dataclass
class MotorStats:
    requests: int = 0
    first_success: int = 0
    retry_success: int = 0
    final_failure: int = 0
    timeout: int = 0
    wrong_id: int = 0
    length_error: int = 0
    checksum_error: int = 0
    response_times: list[float] = field(default_factory=list)

    def record_failure(self, error: Exception) -> None:
        if isinstance(error, RetryExhaustedError):
            error = error.last_error
        if isinstance(error, NoResponseError):
            self.timeout += 1
        elif isinstance(error, WrongIdReadback):
            self.wrong_id += 1
        elif isinstance(error, FrameLengthError):
            self.length_error += 1
        elif isinstance(error, FrameChecksumError):
            self.checksum_error += 1
        elif isinstance(error, FrameFieldError) and "response ID" in str(error):
            self.wrong_id += 1


@dataclass
class OverallStats:
    expected_no_response: int = 0


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(log_path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    for target in (logger, logging.getLogger("uart_bus")):
        target.setLevel(logging.INFO)
        target.handlers.clear()
        target.addHandler(handler)
        target.propagate = False


def _root_error(error: Exception) -> Exception:
    return error.last_error if isinstance(error, RetryExhaustedError) else error


def _read_online(
    motor: Motor,
    motor_id: int,
    stats: MotorStats,
    retries: int,
) -> tuple[bool, Optional[int], Optional[Exception]]:
    stats.requests += 1
    last_error: Optional[Exception] = None
    for attempt in range(1, retries + 2):
        if attempt > 1:
            logger.info("ID%d retry attempt=%d", motor_id, attempt)
        started = time.monotonic()
        try:
            response = motor.read_register(Register.ID_ADDR)
            if len(response.values) != 1:
                raise FrameLengthError(
                    f"ID read returned {len(response.values)} values"
                )
            readback_id = response.values[0]
            if readback_id != motor_id:
                raise WrongIdReadback(
                    f"ID register returned {readback_id}, expected {motor_id}"
                )
        except Exception as error:
            root = _root_error(error)
            stats.record_failure(root)
            last_error = root
            logger.warning("ID%d attempt=%d failed: %s", motor_id, attempt, root)
            continue
        elapsed = time.monotonic() - started
        stats.response_times.append(elapsed)
        if attempt == 1:
            stats.first_success += 1
        else:
            stats.retry_success += 1
        return True, readback_id, None
    stats.final_failure += 1
    return False, None, last_error


def _probe_absent(
    motor: Motor,
    motor_id: int,
    overall: OverallStats,
) -> tuple[bool, Optional[int], Optional[Exception]]:
    """Probe an absent ID exactly once; its timeout is not online-device data."""

    try:
        response = motor.read_register(Register.ID_ADDR)
        if len(response.values) != 1:
            error = FrameLengthError(
                f"ID read returned {len(response.values)} values"
            )
        else:
            readback_id = response.values[0]
            if readback_id != motor_id:
                error = WrongIdReadback(
                    f"ID register returned {readback_id}, expected {motor_id}"
                )
            else:
                return True, readback_id, None
    except Exception as caught:
        error = _root_error(caught)
    if isinstance(error, NoResponseError):
        overall.expected_no_response += 1
    return False, None, error


def _result_line(
    motor_id: int,
    success: bool,
    readback_id: Optional[int],
    error: Optional[Exception],
    *,
    absent: bool = False,
) -> str:
    if success:
        return f"ID{motor_id} RESPONSE readback_id={readback_id}"
    if absent and isinstance(error, NoResponseError):
        return f"ID{motor_id} EXPECTED NO RESPONSE"
    return f"ID{motor_id} ERROR {error}"


def _stats_line(motor_id: int, stats: MotorStats) -> str:
    average_ms = (
        sum(stats.response_times) / len(stats.response_times) * 1000
        if stats.response_times
        else 0.0
    )
    maximum_ms = max(stats.response_times, default=0.0) * 1000
    return (
        f"ID{motor_id} STATS requests={stats.requests} "
        f"first_success={stats.first_success} "
        f"retry_success={stats.retry_success} "
        f"final_failure={stats.final_failure} "
        f"timeout={stats.timeout} wrong_id={stats.wrong_id} "
        f"length_error={stats.length_error} checksum_error={stats.checksum_error} "
        f"avg_response_ms={average_ms:.3f} max_response_ms={maximum_ms:.3f}"
    )


def _overall_line(ids: Iterable[int], stats_by_id: dict[int, MotorStats], overall: OverallStats) -> str:
    stats = [stats_by_id[motor_id] for motor_id in ids]
    total_times = [value for item in stats for value in item.response_times]
    total = {
        name: sum(getattr(item, name) for item in stats)
        for name in (
            "requests", "first_success", "retry_success", "final_failure",
            "timeout", "wrong_id", "length_error", "checksum_error",
        )
    }
    average_ms = sum(total_times) / len(total_times) * 1000 if total_times else 0.0
    maximum_ms = max(total_times, default=0.0) * 1000
    return (
        "TOTAL "
        + " ".join(f"{name}={value}" for name, value in total.items())
        + f" expected_no_response={overall.expected_no_response}"
        + f" avg_response_ms={average_ms:.3f} max_response_ms={maximum_ms:.3f}"
    )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        f"probe_two_ids_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    _configure_logging(log_path)
    stats_by_id = {motor_id: MotorStats() for motor_id in args.ids}
    overall = OverallStats()
    initial_ok = True
    rounds_ok = True
    try:
        with UartBus(args.port, timeout=args.timeout, retries=0) as bus:
            motors = {motor_id: Motor(bus, motor_id) for motor_id in args.ids}
            for motor_id in args.ids:
                success, readback, error = _read_online(
                    motors[motor_id], motor_id, stats_by_id[motor_id], args.retries
                )
                line = _result_line(motor_id, success, readback, error)
                print(line)
                logger.info(line)
                initial_ok = initial_ok and success

            absent_motor = Motor(bus, args.probe_id)
            success, readback, error = _probe_absent(
                absent_motor, args.probe_id, overall
            )
            line = _result_line(
                args.probe_id, success, readback, error, absent=True
            )
            print(line)
            logger.info(line)

            for round_index in range(1, ROUND_COUNT + 1):
                for motor_id in args.ids:
                    success, readback, error = _read_online(
                        motors[motor_id], motor_id, stats_by_id[motor_id], args.retries
                    )
                    rounds_ok = rounds_ok and success
                    logger.info(
                        "round=%d ID%d success=%s readback_id=%s error=%s",
                        round_index, motor_id, success, readback, error or "",
                    )
        for motor_id in args.ids:
            line = _stats_line(motor_id, stats_by_id[motor_id])
            print(line)
            logger.info(line)
        line = _overall_line(args.ids, stats_by_id, overall)
        print(line)
        logger.info(line)
        logger.info("汇总日志和原始 TX/RX 已保存: %s", log_path)
        return 0 if initial_ok and rounds_ok and overall.expected_no_response == 1 else 1
    except Exception as exc:
        logger.error("多电机只读通信测试失败: %s", exc)
        print(f"ERROR {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="多电机只读串行轮询测试")
    parser.add_argument("--port", required=True)
    parser.add_argument("--ids", nargs="+", type=int, default=[1, 2])
    parser.add_argument("--probe-id", type=int, default=None)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log", help="日志文件路径；默认当前目录按时间生成")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.ids or len(set(args.ids)) != len(args.ids):
        parser.error("--ids 必须至少包含一个且不能重复")
    if any(not 1 <= motor_id <= 0xFE for motor_id in args.ids):
        parser.error("--ids 中每个 ID 必须在 1..254")
    if args.probe_id is None:
        absent = next((candidate for candidate in range(1, 0xFF) if candidate not in args.ids), None)
        if absent is None:
            parser.error("无法自动选择不存在 ID，请显式指定 --probe-id")
        args.probe_id = absent
    if not 1 <= args.probe_id <= 0xFE:
        parser.error("--probe-id 必须在 1..254")
    if args.probe_id in args.ids:
        parser.error("--probe-id 必须不在 --ids 在线 ID 列表中")
    if args.timeout <= 0:
        parser.error("--timeout 必须大于 0")
    if args.retries < 0:
        parser.error("--retries 不能小于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
