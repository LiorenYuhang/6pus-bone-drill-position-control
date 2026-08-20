#!/usr/bin/env python3
"""Probe one LAF motor ID using a single read of register 0x16."""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import Register  # noqa: E402
from uart_bus import (  # noqa: E402
    NoResponseError,
    RetryExhaustedError,
    UartBus,
)


logger = logging.getLogger("probe_id")


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)

    for target in (logger, logging.getLogger("uart_bus")):
        target.setLevel(logging.INFO)
        target.handlers.clear()
        target.addHandler(file_handler)
        target.propagate = False


def _is_no_response(exc: Exception) -> bool:
    if isinstance(exc, NoResponseError):
        return True
    return isinstance(exc, RetryExhaustedError) and isinstance(
        exc.last_error,
        NoResponseError,
    )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        f"probe_id_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    _configure_logging(log_path)
    logger.info(
        "开始只读 ID 探测: port=%s requested_id=%d timeout=%.3fs",
        args.port,
        args.id,
        args.timeout,
    )

    try:
        with UartBus(args.port, timeout=args.timeout, retries=0) as bus:
            response = Motor(bus, args.id).read_register(Register.ID_ADDR)
        if len(response.values) != 1:
            raise RuntimeError(f"ID read returned {len(response.values)} values")
        readback_id = response.values[0]
        result = f"RESPONSE requested_id={args.id} readback_id={readback_id}"
        print(result)
        logger.info(result)
        return 0
    except Exception as exc:
        if _is_no_response(exc):
            result = f"NO RESPONSE id={args.id}"
            print(result)
            logger.info("%s (%s)", result, exc)
            return 0
        result = f"ERROR id={args.id}: {exc}"
        print(result, file=sys.stderr)
        logger.error(result)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="只读探测指定因时电缸 ID 是否在线")
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--log", help="日志文件路径；默认当前目录按时间生成")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.id <= 0xFE:
        parser.error("--id 必须在 1..254")
    if args.timeout <= 0:
        parser.error("--timeout 必须大于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
