#!/usr/bin/env python3
"""Interactively assign and verify one LAF motor ID.

The operator must physically connect exactly one motor. Power cycling is
deliberately manual: the script pauses after the optional ID change and Flash
save, then continues only after power was removed and restored.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import Register  # noqa: E402
from uart_bus import BusError, NoResponseError, RetryExhaustedError, UartBus  # noqa: E402


logger = logging.getLogger("assign_id")


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    # Include UartBus raw TX/RX records in the same archive.
    bus_logger = logging.getLogger("uart_bus")
    bus_logger.setLevel(logging.INFO)
    bus_logger.handlers.clear()
    bus_logger.addHandler(file_handler)
    bus_logger.addHandler(console_handler)


def _read_id(motor: Motor) -> int:
    response = motor.read_register(Register.ID_ADDR)
    if len(response.values) != 1:
        raise RuntimeError(f"ID read returned {len(response.values)} values")
    return response.values[0]


def _old_id_is_silent(bus: UartBus, old_id: int) -> bool:
    old_motor = Motor(bus, old_id)
    try:
        old_motor.read_register(Register.ID_ADDR)
    except RetryExhaustedError as exc:
        if isinstance(exc.last_error, NoResponseError):
            logger.info("旧 ID=%d 无有效应答，反向验证通过", old_id)
            return True
        raise
    raise RuntimeError(f"旧 ID={old_id} 仍返回有效应答，反向验证失败")


def _port_exists(port: str) -> bool:
    """Check the original path without resolving its ttyUSB target."""

    return Path(port).exists()


def _open_reconnected_bus(
    port: str,
    *,
    timeout: float,
    retries: int,
    reconnect_timeout: float,
) -> UartBus:
    deadline = time.monotonic() + reconnect_timeout
    last_error = None
    logger.info(
        "等待串口重新出现: path=%s timeout=%.1fs",
        port,
        reconnect_timeout,
    )
    while True:
        if _port_exists(port):
            try:
                bus = UartBus(port, timeout=timeout, retries=retries)
                logger.info("已通过原始路径重新打开串口: %s", port)
                return bus
            except (BusError, OSError) as exc:
                last_error = exc
                logger.warning("串口路径已出现但暂时无法打开: %s", exc)

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            detail = f"；最后一次打开错误: {last_error}" if last_error else ""
            raise TimeoutError(
                f"等待串口 {port} 重新出现并可用超时（{reconnect_timeout:.1f}s）{detail}"
            )
        time.sleep(min(0.2, remaining))


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        f"assign_id_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    _configure_logging(log_path)
    logger.info(
        "开始单电机编号: port=%s old_id=%d new_id=%d log=%s",
        args.port,
        args.old_id,
        args.new_id,
        log_path,
    )
    if not args.port.startswith("/dev/serial/by-id/"):
        logger.warning(
            "建议使用稳定路径 /dev/serial/by-id/...；程序会按原始 --port 路径重开，不缓存 ttyUSB 编号"
        )

    confirmation = input(
        "确认当前只有一台电缸物理连接到 AES-LA-31-06？确认请输入 YES："
    ).strip()
    if confirmation != "YES":
        logger.error("操作者未确认单台连接，退出")
        return 2

    try:
        with UartBus(
            args.port,
            timeout=args.timeout,
            retries=args.retries,
        ) as bus:
            motor = Motor(bus, args.old_id)
            actual_old_id = _read_id(motor)
            logger.info("读取旧 ID=0x%02X", actual_old_id)
            if actual_old_id != args.old_id:
                raise RuntimeError(
                    f"设备读回 ID={actual_old_id}，不是预期旧 ID={args.old_id}"
                )

            if args.new_id != args.old_id:
                logger.info("写入新 ID=%d", args.new_id)
                motor.set_motor_id(args.new_id)
                logger.info("改号应答已按旧 ID=%d 接收，本地目标切换为 ID=%d", args.old_id, args.new_id)

                readback_id = _read_id(motor)
                if readback_id != args.new_id:
                    raise RuntimeError(f"改号后立即读回 ID={readback_id}，预期 {args.new_id}")
                logger.info("改号后立即读回新 ID=%d", readback_id)

                saved = motor.save_parameters()
                logger.info("Flash 保存两帧已解析: success=%s", saved.success)
                if not saved.success:
                    raise RuntimeError("Flash 保存第二帧未返回 0x40 成功标志")
            else:
                logger.info("旧 ID=新 ID=%d，跳过 ID 写入和 Flash 保存", args.old_id)

        logger.info("断电前 UART 已关闭: %s", args.port)
        power_confirmation = input(
            "串口已关闭。请现在手动断电，等待放电后重新上电；完成后输入 YES："
        ).strip()
        if power_confirmation != "YES":
            raise RuntimeError("未确认人工断电/上电")

        with _open_reconnected_bus(
            args.port,
            timeout=args.timeout,
            retries=args.retries,
            reconnect_timeout=args.reconnect_timeout,
        ) as reconnected_bus:
            reconnected_motor = Motor(reconnected_bus, args.new_id)
            forward_id = _read_id(reconnected_motor)
            if forward_id != args.new_id:
                raise RuntimeError(f"掉电正向验证读回 ID={forward_id}，预期 {args.new_id}")
            logger.info("掉电正向验证通过，新 ID=%d", forward_id)

            if args.new_id != args.old_id:
                _old_id_is_silent(reconnected_bus, args.old_id)
            else:
                logger.info("旧 ID=新 ID=%d，旧 ID 反向验证不适用", args.old_id)

        logger.info("单电机编号成功，日志已归档到 %s", log_path)
        return 0
    except Exception:
        logger.exception("单电机编号失败")
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="单台因时电缸 ID 修改与人工掉电验证")
    parser.add_argument(
        "--port",
        required=True,
        help="优先使用 /dev/serial/by-id/... 稳定路径",
    )
    parser.add_argument("--old-id", type=int, default=1)
    parser.add_argument("--new-id", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument(
        "--reconnect-timeout",
        type=float,
        default=30.0,
        help="人工上电后等待原始串口路径重新可用的秒数，默认 30",
    )
    parser.add_argument("--log", help="日志文件路径；默认当前目录按时间生成")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.old_id <= 0xFE:
        parser.error("--old-id 必须在 1..254")
    if not 1 <= args.new_id <= 0xFE:
        parser.error("--new-id 必须在 1..254")
    if args.reconnect_timeout <= 0:
        parser.error("--reconnect-timeout 必须大于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
