#!/usr/bin/env python3
"""Read-only static force baseline sampler for the standalone ID1 motor."""

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
DEFAULT_DURATION_SECONDS = 10.0
DEFAULT_RATE_HZ = 50.0

logger = logging.getLogger("read_id1_force_baseline")


@dataclass(frozen=True)
class Sample:
    index: int
    elapsed_s: float
    wall_time: str
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


def _sample_line(sample: Sample) -> str:
    return (
        f"SAMPLE index={sample.index} elapsed_s={sample.elapsed_s:.6f} "
        f"wall_time={sample.wall_time} position_steps={sample.position_steps} "
        f"current_mA={sample.current_mA} force_protocol_g={sample.force_protocol_g} "
        f"temperature_C={sample.temperature_C} fault=0x{sample.fault_code:02X}"
    )


def _summary(samples: list[Sample], *, label: str, duration: float, rate: float) -> str:
    forces = [sample.force_protocol_g for sample in samples]
    currents = [sample.current_mA for sample in samples]
    positions = [sample.position_steps for sample in samples]
    faults = sum(sample.fault_code != 0 for sample in samples)
    return "\n".join(
        (
            f"FORCE_BASELINE ID1 label={label} samples={len(samples)} "
            f"duration_s={duration:.3f} rate_hz={rate:.3f}",
            "force_protocol_g "
            f"mean={statistics.fmean(forces):.6f} "
            f"std={statistics.pstdev(forces):.6f} "
            f"min={min(forces)} max={max(forces)} "
            f"peak_to_peak={max(forces) - min(forces)}",
            "current_mA "
            f"mean={statistics.fmean(currents):.6f} max={max(currents)}",
            "position_steps "
            f"mean={statistics.fmean(positions):.6f} "
            f"min={min(positions)} max={max(positions)}",
            f"fault_samples={faults}",
        )
    )


def run(args: argparse.Namespace) -> int:
    label = args.label.replace(" ", "_")
    log_path = Path(args.log) if args.log else Path(
        "logs/id1_motion/"
        f"force_baseline_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{label}.log"
    )
    try:
        _configure_logging(log_path)
    except Exception as exc:
        print(f"ERROR cannot create independent log {log_path}: {exc}", file=sys.stderr)
        return 1

    sample_count = int(round(args.duration * args.rate))
    samples: list[Sample] = []
    try:
        with UartBus(args.port, timeout=args.timeout, retries=args.retries) as bus:
            motor = Motor(bus, MOTOR_ID)
            start = time.monotonic()
            period = 1.0 / args.rate
            for index in range(sample_count):
                scheduled = start + index * period
                delay = scheduled - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                status = motor.read_status()
                elapsed = time.monotonic() - start
                sample = Sample(
                    index=index,
                    elapsed_s=elapsed,
                    wall_time=datetime.now(timezone.utc).isoformat(),
                    position_steps=status.current_position,
                    current_mA=status.current_mA,
                    force_protocol_g=status.force_g,
                    temperature_C=status.temperature,
                    fault_code=status.error_code,
                )
                samples.append(sample)
                logger.info(_sample_line(sample))

        summary = _summary(
            samples,
            label=label,
            duration=args.duration,
            rate=args.rate,
        )
        print(summary)
        logger.info("%s", summary.replace("\n", " | "))
        logger.info("raw TX/RX and baseline samples saved: %s", log_path)
        return 0
    except Exception as exc:
        logger.error("ID1 force baseline stopped: %s", exc)
        print(f"ERROR ID1 force baseline: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ID1 只读静态力基线采样")
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--label", default="static")
    parser.add_argument("--log", help="独立日志路径；已有文件不会覆盖")
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.rate <= 0 or args.timeout <= 0:
        parser.error("duration、rate、timeout 必须大于 0")
    if args.retries < 0:
        parser.error("--retries 不能小于 0")
    if not args.label or any(character in args.label for character in "/\\"):
        parser.error("--label 不能为空且不能包含路径分隔符")
    return args


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
