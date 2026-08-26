"""J7-C10 real-UART injection entry for read-only preflight.

The CLI opens a serial object only when explicitly invoked.  Its first-phase
operation performs status and mode reads only; it never calls Servo
configuration or the ``0x29`` motion stream.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import logging
from pathlib import Path
import time
from typing import Callable, Sequence

from .j7_real_uart_adapter import J7UartAdapter
from .j7_servo_feedback_scheduler import FeedbackCycleAudit, J7ServoFeedbackScheduler
from .j7_six_axis_servo_transport import J7SixAxisServoTransport
from .safety import MOTOR_IDS

logger = logging.getLogger("pus6_motor_driver.j7_c10_preflight")


@dataclass(frozen=True)
class ReadOnlyPreflightResult:
    port: str
    baudrate: int
    motor_ids: tuple[int, ...]
    feedback: FeedbackCycleAudit
    rx_owner: dict
    requests: tuple[dict, ...]


def parse_motor_ids(value: str) -> tuple[int, ...]:
    """Parse and validate the fixed six-axis J7 ID order."""
    try:
        ids = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as exc:
        raise ValueError("motor IDs must be comma-separated integers") from exc
    if ids != MOTOR_IDS:
        raise ValueError(f"J7 C10 requires motor IDs in fixed order {MOTOR_IDS}, got {ids}")
    return ids


def run_read_only_preflight(
    adapter: J7UartAdapter,
    *,
    port: str,
    baudrate: int,
    motor_ids: Sequence[int] = MOTOR_IDS,
    clock_ns: Callable[[], int] | None = None,
    audit_file: str | Path | None = None,
) -> ReadOnlyPreflightResult:
    """Run one six-axis status/mode scan without configuration or motion writes."""
    ids = tuple(motor_ids)
    if ids != MOTOR_IDS:
        raise ValueError(f"J7 C10 requires motor IDs in fixed order {MOTOR_IDS}, got {ids}")
    clock = clock_ns or time.monotonic_ns

    # C10 is strictly read-only. Do not wire C7/C9's stop callback here:
    # any read timeout or malformed response must close/report, never emit
    # a 0x1A stop frame during this preflight.
    writer = J7SixAxisServoTransport(
        adapter,
        pause_callback=None,
        clock_ns=clock,
    )
    feedback = J7ServoFeedbackScheduler(writer, clock_ns=clock)
    feedback_audit = feedback.full_scan(
        servo_cycle_id=0,
        servo_cycle_timestamp_ns=clock(),
    )
    if feedback_audit.deferred or len(feedback_audit.samples) != len(MOTOR_IDS):
        raise RuntimeError("read-only feedback scan incomplete")
    for sample in feedback_audit.samples:
        if sample.timeout or sample.fault not in (0,):
            raise RuntimeError(
                f"ID{sample.motor_id} read-only health failure: "
                f"fault={sample.fault}, timeout={sample.timeout}"
            )

    result = ReadOnlyPreflightResult(
        port=port,
        baudrate=baudrate,
        motor_ids=ids,
        feedback=feedback_audit,
        rx_owner=asdict(adapter.rx_owner.audit_snapshot()),
        requests=tuple(asdict(item) for item in adapter.audit_snapshot()),
    )
    if audit_file is not None:
        Path(audit_file).write_text(
            json.dumps(asdict(result), indent=2, sort_keys=True),
            encoding="utf-8",
        )
    return result


def _open_serial(port: str, baudrate: int, timeout_s: float):
    """Open pyserial only from the explicitly invoked C10 CLI."""
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - depends on host install.
        raise RuntimeError("pyserial is required for the C10 real-UART entry") from exc
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout_s,
        write_timeout=timeout_s,
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="J7-C10 read-only UART preflight")
    parser.add_argument("--port", required=True, help="approved serial port path")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--motor-ids", default="1,2,3,4,5,6")
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--audit-file", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        motor_ids = parse_motor_ids(args.motor_ids)
    except ValueError as exc:
        parser.error(str(exc))
        return 2

    serial_instance = None
    try:
        serial_instance = _open_serial(args.port, args.baudrate, args.timeout)
        adapter = J7UartAdapter(
            serial_instance,
            baudrate=args.baudrate,
            timeout_s=args.timeout,
        )
        result = run_read_only_preflight(
            adapter,
            port=args.port,
            baudrate=args.baudrate,
            motor_ids=motor_ids,
            audit_file=args.audit_file,
        )
        print(json.dumps(asdict(result), indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        logger.error("J7-C10 read-only preflight failed: %s", exc)
        return 1
    finally:
        if serial_instance is not None:
            serial_instance.close()


if __name__ == "__main__":  # pragma: no cover - real CLI invocation only.
    raise SystemExit(main())

