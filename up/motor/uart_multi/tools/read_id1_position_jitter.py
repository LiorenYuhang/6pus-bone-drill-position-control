#!/usr/bin/env python3
"""Read-only ID1 position quantization/jitter diagnostic."""

from __future__ import annotations

import argparse
import logging
import statistics
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from uart_bus import UartBus  # noqa: E402


MOTOR_ID = 1
DEFAULT_DURATION_SECONDS = 30.0
DEFAULT_RATE_HZ = 50.0

logger = logging.getLogger("read_id1_position_jitter")


@dataclass(frozen=True)
class Sample:
    index: int
    elapsed_s: float
    wall_time: str
    position: int
    current_mA: int
    force_g: int
    temperature_C: int
    fault: int


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
        f"wall_time={sample.wall_time} position={sample.position} "
        f"current_mA={sample.current_mA} force_g={sample.force_g} "
        f"temperature_C={sample.temperature_C} fault=0x{sample.fault:02X}"
    )


def _mode_count(values: list[int]) -> int:
    counts = Counter(values)
    return min(
        (value for value, count in counts.items() if count == max(counts.values())),
        default=0,
    )


def _run_lengths(positions: list[int]) -> dict[int, list[int]]:
    result: dict[int, list[int]] = defaultdict(list)
    if not positions:
        return result
    current = positions[0]
    length = 1
    for position in positions[1:]:
        if position == current:
            length += 1
        else:
            result[current].append(length)
            current = position
            length = 1
    result[current].append(length)
    return result


def _max_consecutive(positions: list[int], value: int) -> int:
    longest = current = 0
    for position in positions:
        current = current + 1 if position == value else 0
        longest = max(longest, current)
    return longest


def _summary(samples: list[Sample], *, duration: float, rate: float) -> str:
    positions = [sample.position for sample in samples]
    currents = [sample.current_mA for sample in samples]
    forces = [sample.force_g for sample in samples]
    temperatures = [sample.temperature_C for sample in samples]
    faults = [sample.fault for sample in samples]
    counts = Counter(positions)
    mode_position = _mode_count(positions)
    run_lengths = _run_lengths(positions)
    transitions = Counter(
        (before.position, after.position)
        for before, after in zip(samples, samples[1:])
        if before.position != after.position
    )
    jump_state_changes = 0
    for before, after in zip(samples, samples[1:]):
        if before.position == after.position:
            continue
        if (
            before.current_mA != after.current_mA
            or before.force_g != after.force_g
            or before.temperature_C != after.temperature_C
            or before.fault != after.fault
        ):
            jump_state_changes += 1
    histogram = ",".join(
        f"{position}:{counts[position]}" for position in sorted(counts)
    )
    transition_text = ",".join(
        f"{before}->{after}:{count}"
        for (before, after), count in sorted(transitions.items())
    ) or "-"
    run_text = ";".join(
        f"{position}:n={len(lengths)},max={max(lengths)},mean={statistics.fmean(lengths):.3f}"
        for position, lengths in sorted(run_lengths.items())
    ) or "-"
    max_deviation = max((abs(position - mode_position) for position in positions), default=0)
    p0 = positions[0] if positions else 0
    return "\n".join(
        (
            f"POSITION_JITTER ID1 samples={len(samples)} duration_s={duration:.3f} rate_hz={rate:.3f} p0={p0}",
            f"position_histogram={histogram}",
            f"position_mode={mode_position} mode_count={counts.get(mode_position, 0)} "
            f"min={min(positions) if positions else '-'} max={max(positions) if positions else '-'} "
            f"max_deviation_from_mode={max_deviation}",
            f"position_jump_count={sum(transitions.values())} transitions={transition_text}",
            f"p0_minus_1={p0 - 1} max_consecutive={_max_consecutive(positions, p0 - 1)}",
            f"p0_plus_1={p0 + 1} max_consecutive={_max_consecutive(positions, p0 + 1)}",
            f"run_lengths={run_text}",
            f"current_mA mean={statistics.fmean(currents):.6f} min={min(currents)} max={max(currents)}",
            f"force_g mean={statistics.fmean(forces):.6f} min={min(forces)} max={max(forces)}",
            f"temperature_C mean={statistics.fmean(temperatures):.6f} min={min(temperatures)} max={max(temperatures)}",
            f"fault_histogram={dict(sorted(Counter(faults).items()))}",
            f"position_jumps_with_other_state_change={jump_state_changes}",
        )
    )


def run(args: argparse.Namespace) -> int:
    log_path = Path(args.log) if args.log else Path(
        "logs/id1_motion/"
        f"position_jitter_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
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
            start = time.monotonic()
            period = 1.0 / args.rate
            for index in range(sample_count):
                scheduled = start + index * period
                delay = scheduled - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                status = motor.read_status()
                sample = Sample(
                    index=index,
                    elapsed_s=time.monotonic() - start,
                    wall_time=datetime.now(timezone.utc).isoformat(),
                    position=status.current_position,
                    current_mA=status.current_mA,
                    force_g=status.force_g,
                    temperature_C=status.temperature,
                    fault=status.error_code,
                )
                samples.append(sample)
                logger.info(_sample_line(sample))

        summary = _summary(samples, duration=args.duration, rate=args.rate)
        print(summary)
        logger.info("%s", summary.replace("\n", " | "))
        logger.info("read-only raw TX/RX and position jitter samples saved: %s", log_path)
        return 0
    except Exception as exc:
        logger.error("ID1 position jitter diagnostic stopped: %s", exc)
        print(f"ERROR ID1 position jitter diagnostic: {exc}", file=sys.stderr)
        return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ID1 只读位置量化/抖动诊断")
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
