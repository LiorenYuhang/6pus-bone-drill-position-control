#!/usr/bin/env python3
"""Read-only ID1 control/configuration state diagnostic."""

from __future__ import annotations

import argparse
import logging
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 1
CONTROL_MODE_ADDR = 0x25
TARGET_SPEED_ADDR = 0x28
TARGET_POSITION_ADDR = 0x29
DEFAULT_DURATION_SECONDS = 5.0
DEFAULT_RATE_HZ = 10.0

logger = logging.getLogger("read_id1_control_state")


@dataclass(frozen=True)
class Config:
    mode: int
    target_speed: int
    target_position: int


@dataclass(frozen=True)
class Sample:
    index: int
    elapsed_s: float
    wall_time: str
    config: Config
    position_steps: int
    current_mA: int
    force_protocol_g: int
    temperature_C: int
    fault_code: int


def _configure_logging(log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(log_path, mode="x", encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    for target in (logger, logging.getLogger("uart_bus")):
        target.setLevel(logging.INFO)
        target.handlers.clear()
        target.addHandler(handler)
        target.propagate = False


def _read_one(motor: Motor, register_addr: int) -> int:
    response = motor.read_register(register_addr)
    if len(response.values) != 1:
        raise RuntimeError(
            f"register 0x{register_addr:02X} returned {len(response.values)} values"
        )
    return response.values[0]


def _read_config(motor: Motor) -> Config:
    return Config(
        mode=_read_one(motor, CONTROL_MODE_ADDR),
        target_speed=_read_one(motor, TARGET_SPEED_ADDR),
        target_position=_read_one(motor, TARGET_POSITION_ADDR),
    )


def _sample_line(sample: Sample) -> str:
    return (
        f"SAMPLE index={sample.index} elapsed_s={sample.elapsed_s:.6f} "
        f"wall_time={sample.wall_time} mode={sample.config.mode} "
        f"target_speed={sample.config.target_speed} "
        f"target_position={sample.config.target_position} "
        f"actual_position={sample.position_steps} current_mA={sample.current_mA} "
        f"force_protocol_g={sample.force_protocol_g} temperature_C={sample.temperature_C} "
        f"fault=0x{sample.fault_code:02X}"
    )


def _summary(
    samples: list[Sample],
    initial: Config,
    *,
    duration: float,
    rate: float,
) -> str:
    positions = [sample.position_steps for sample in samples]
    currents = [sample.current_mA for sample in samples]
    forces = [sample.force_protocol_g for sample in samples]
    configs = [sample.config for sample in samples]
    final = configs[-1]
    return "\n".join(
        (
            f"CONTROL_STATE ID1 samples={len(samples)} duration_s={duration:.3f} rate_hz={rate:.3f}",
            f"CONFIG_INITIAL mode={initial.mode} target_speed={initial.target_speed} "
            f"target_position={initial.target_position}",
            f"CONFIG_FINAL mode={final.mode} target_speed={final.target_speed} "
            f"target_position={final.target_position}",
            "CONFIG_CHANGED "
            f"mode={'YES' if any(c.mode != initial.mode for c in configs) else 'NO'} "
            f"target_speed={'YES' if any(c.target_speed != initial.target_speed for c in configs) else 'NO'} "
            f"target_position={'YES' if any(c.target_position != initial.target_position for c in configs) else 'NO'}",
            "actual_position_steps "
            f"mean={statistics.fmean(positions):.6f} min={min(positions)} max={max(positions)}",
            "current_mA "
            f"mean={statistics.fmean(currents):.6f} min={min(currents)} max={max(currents)}",
            "force_protocol_g "
            f"mean={statistics.fmean(forces):.6f} min={min(forces)} max={max(forces)}",
        )
    )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/id1_motion/"
        f"control_state_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    sample_count = max(1, int(round(args.duration * args.rate)))
    samples: list[Sample] = []
    try:
        with UartBus(args.port, timeout=args.timeout, retries=args.retries) as bus:
            motor = Motor(bus, MOTOR_ID)
            initial = _read_config(motor)
            logger.info(
                "CONFIG_INITIAL mode=%d target_speed=%d target_position=%d",
                initial.mode,
                initial.target_speed,
                initial.target_position,
            )
            start = time.monotonic()
            period = 1.0 / args.rate
            for index in range(sample_count):
                scheduled = start + index * period
                delay = scheduled - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                config = _read_config(motor)
                status = motor.read_status()
                sample = Sample(
                    index=index,
                    elapsed_s=time.monotonic() - start,
                    wall_time=datetime.now(timezone.utc).isoformat(),
                    config=config,
                    position_steps=status.current_position,
                    current_mA=status.current_mA,
                    force_protocol_g=status.force_g,
                    temperature_C=status.temperature,
                    fault_code=status.error_code,
                )
                samples.append(sample)
                logger.info(_sample_line(sample))

        summary = _summary(samples, initial, duration=args.duration, rate=args.rate)
        print(summary)
        logger.info("%s", summary.replace("\n", " | "))
        logger.info("read-only raw TX/RX and control-state samples saved: %s", log_path)
        return 0
    except Exception as exc:
        logger.error("ID1 control-state diagnostic stopped: %s", exc)
        print(f"ERROR ID1 control-state diagnostic: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ID1 只读控制状态诊断")
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--log", help="独立日志路径；已有文件不会覆盖")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.rate <= 0 or args.timeout <= 0:
        parser.error("duration、rate、timeout 必须大于 0")
    if args.retries < 0:
        parser.error("retries 不能小于 0")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
