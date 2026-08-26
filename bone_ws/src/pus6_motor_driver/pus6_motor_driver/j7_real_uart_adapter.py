"""Isolated J7 UART adapter and six-axis stop path.

The adapter accepts an already-created serial-like object.  It never opens a
port itself, which keeps C9 Mock validation physically safe.  One RX owner
parses complete response frames before exposing canonical bytes to the C5/C6
transport consumers; all physical access is serialized by ``uart_lock``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Protocol
import threading
import time

try:
    from .protocol import (
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        pack_write_register_cmd,
        validate_response,
    )
    from .safety import MOTOR_IDS, PAUSE_ADDR
except ImportError:  # pragma: no cover - direct module execution fallback.
    from protocol import (  # type: ignore
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        pack_write_register_cmd,
        validate_response,
    )
    from safety import MOTOR_IDS, PAUSE_ADDR  # type: ignore


class J7SerialLike(Protocol):
    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int | None: ...


class J7AdapterError(RuntimeError):
    """Base adapter error."""


class J7AdapterTimeout(J7AdapterError):
    """A bounded request or stop response timed out."""


class J7AdapterRxError(J7AdapterError):
    """RX bytes could not be safely parsed or bounded."""


@dataclass(frozen=True)
class RxOwnerAudit:
    residual_frames: int
    residual_bytes: int
    stale_response_count: int
    feedback_response_count: int
    malformed_count: int
    overflow_count: int


class J7RxOwner:
    """Single parser/queue owner for all response bytes from one UART."""

    def __init__(self, *, max_frames: int = 256, max_bytes: int = 32_768) -> None:
        if max_frames < 1 or max_bytes < 1:
            raise ValueError("RX limits must be positive")
        self.max_frames = max_frames
        self.max_bytes = max_bytes
        self._parser = FrameStreamParser(expected_header=RSP_FRAME_HEADER)
        self._frames: Deque[Frame] = deque()
        self._residual_bytes = 0
        self._stale_response_count = 0
        self._feedback_response_count = 0
        self._malformed_count = 0
        self._overflow_count = 0

    @property
    def residual_frame_count(self) -> int:
        return len(self._frames)

    @property
    def residual_bytes(self) -> int:
        return self._residual_bytes

    def ingest(self, data: bytes) -> tuple[Frame, ...]:
        if not isinstance(data, (bytes, bytearray)):
            raise J7AdapterRxError("RX data must be bytes-like")
        if not data:
            return tuple()
        try:
            frames = self._parser.feed(bytes(data))
        except Exception as exc:
            self._malformed_count += 1
            self._parser.clear()
            raise J7AdapterRxError(f"malformed RX frame: {exc}") from exc
        for frame in frames:
            self._frames.append(frame)
            self._residual_bytes += len(frame.raw)
        if len(self._frames) > self.max_frames or self._residual_bytes > self.max_bytes:
            self._overflow_count += 1
            raise J7AdapterRxError(
                f"RX residual limit exceeded: frames={len(self._frames)}/{self.max_frames}, "
                f"bytes={self._residual_bytes}/{self.max_bytes}"
            )
        return tuple(frames)

    def take_matching(
        self,
        *,
        expected_id: int,
        expected_command: int,
        expected_register: int | None = None,
    ) -> Frame | None:
        """Return one matching frame; classify every non-match as stale."""
        while self._frames:
            frame = self._frames.popleft()
            self._residual_bytes -= len(frame.raw)
            if (
                frame.motor_id == expected_id
                and frame.command == expected_command
                and (expected_register is None or frame.register_addr == expected_register)
            ):
                self._feedback_response_count += 1
                return frame
            self._stale_response_count += 1
        return None

    def drain(self) -> tuple[Frame, ...]:
        """Remove all pending frames as residual/stale without matching."""
        frames = tuple(self._frames)
        self._stale_response_count += len(frames)
        self._frames.clear()
        self._residual_bytes = 0
        return frames

    def take_for_consumer(self) -> tuple[Frame, ...]:
        frames = tuple(self._frames)
        self._frames.clear()
        self._residual_bytes = 0
        return frames

    def audit_snapshot(self) -> RxOwnerAudit:
        return RxOwnerAudit(
            residual_frames=len(self._frames),
            residual_bytes=self._residual_bytes,
            stale_response_count=self._stale_response_count,
            feedback_response_count=self._feedback_response_count,
            malformed_count=self._malformed_count,
            overflow_count=self._overflow_count,
        )


@dataclass(frozen=True)
class J7RequestAudit:
    motor_id: int
    command: int
    register_address: int
    tx_start_ns: int
    tx_end_ns: int
    response_received: bool
    timeout: bool
    retry_count: int
    error: str | None = None


class J7UartAdapter:
    """C5/C6 bus adapter around an injected serial-like object.

    The object is intentionally not a serial-port factory.  A real caller
    must explicitly construct and authorize the serial instance outside this
    module; Mock tests inject ``MockSerial`` instead.
    """

    def __init__(
        self,
        serial_instance: J7SerialLike,
        *,
        baudrate: int = 921600,
        timeout_s: float = 0.05,
        min_frame_interval_s: float = 0.001,
        rx_owner: J7RxOwner | None = None,
        clock_ns: Callable[[], int] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
    ) -> None:
        if serial_instance is None:
            raise ValueError("serial_instance is required; this adapter never opens a port")
        if baudrate <= 0 or timeout_s <= 0.0 or min_frame_interval_s < 0.0:
            raise ValueError("invalid UART timing configuration")
        self.serial = serial_instance
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.min_frame_interval_s = min_frame_interval_s
        self.rx_owner = rx_owner or J7RxOwner()
        self.uart_lock = threading.RLock()
        self._clock_ns = clock_ns or time.monotonic_ns
        self._sleep_fn = sleep_fn or time.sleep
        self._last_tx_end_ns: int | None = None
        self._request_audit: list[J7RequestAudit] = []

    def _available(self) -> int:
        value = getattr(self.serial, "in_waiting", 0)
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return 0

    def _read_chunk(self) -> bytes:
        size = self._available()
        if size <= 0:
            size = 1
        data = self.serial.read(size)
        if data is None:
            return b""
        if not isinstance(data, (bytes, bytearray)):
            raise J7AdapterRxError("serial.read() must return bytes-like data")
        return bytes(data)

    def _ingest_available_locked(self) -> tuple[Frame, ...]:
        frames: list[Frame] = []
        while self._available() > 0:
            data = self._read_chunk()
            if not data:
                break
            frames.extend(self.rx_owner.ingest(data))
        return tuple(frames)

    def _wait_frame_interval_locked(self) -> None:
        if self._last_tx_end_ns is None:
            return
        remaining_ns = self._last_tx_end_ns + int(self.min_frame_interval_s * 1e9) - self._clock_ns()
        if remaining_ns > 0:
            self._sleep_fn(remaining_ns / 1e9)

    def write(self, frame: bytes) -> int | None:
        """Write one frame without reading or waiting for a response."""
        with self.uart_lock:
            self._wait_frame_interval_locked()
            start_ns = self._clock_ns()
            try:
                written = self.serial.write(frame)
                if written is not None and written != len(frame):
                    raise J7AdapterError(f"short UART write: {written}/{len(frame)} bytes")
            except Exception as exc:
                raise J7AdapterError(f"UART write failed: {exc}") from exc
            self._last_tx_end_ns = self._clock_ns()
            return written

    def read_available(self) -> bytes:
        """Drain currently available bytes through the single RX owner.

        Complete canonical frames are returned for C5/C6 consumers; physical
        bytes never bypass ``J7RxOwner``.
        """
        with self.uart_lock:
            self._ingest_available_locked()
            frames = self.rx_owner.take_for_consumer()
            return b"".join(frame.raw for frame in frames)

    def _request_once_locked(
        self,
        frame: bytes,
        *,
        expected_id: int | None,
        expected_command: int | None,
        expected_register: int | None,
    ) -> tuple[bytes, J7RequestAudit]:
        # Classify any response left by a previous write-only transaction.
        self._ingest_available_locked()
        self.rx_owner.drain()
        parsed_command = frame[4] if len(frame) > 4 else -1
        parsed_register = int.from_bytes(frame[5:7], "little") if len(frame) >= 7 else -1
        parsed_id = frame[3] if len(frame) > 3 else -1
        self._wait_frame_interval_locked()
        start_ns = self._clock_ns()
        self.write(frame)
        deadline_ns = self._clock_ns() + int(self.timeout_s * 1e9)
        while self._clock_ns() <= deadline_ns:
            self._ingest_available_locked()
            matched = self.rx_owner.take_matching(
                expected_id=expected_id if expected_id is not None else parsed_id,
                expected_command=(
                    expected_command if expected_command is not None else parsed_command
                ),
                expected_register=expected_register,
            )
            if matched is not None:
                raw = matched.raw
                validate_response(
                    raw,
                    expected_id=expected_id,
                    expected_command=expected_command,
                )
                end_ns = self._clock_ns()
                audit = J7RequestAudit(
                    motor_id=parsed_id,
                    command=parsed_command,
                    register_address=parsed_register,
                    tx_start_ns=start_ns,
                    tx_end_ns=end_ns,
                    response_received=True,
                    timeout=False,
                    retry_count=0,
                )
                return raw, audit
            self._sleep_fn(0.0001)
        audit = J7RequestAudit(
            motor_id=parsed_id,
            command=parsed_command,
            register_address=parsed_register,
            tx_start_ns=start_ns,
            tx_end_ns=self._clock_ns(),
            response_received=False,
            timeout=True,
            retry_count=0,
            error="UART response timeout",
        )
        raise J7AdapterTimeout("UART response timeout")

    def request(
        self,
        frame: bytes,
        *,
        expected_id: int | None = None,
        expected_command: int | None = None,
        retries: int = 0,
    ) -> bytes:
        """Request one validated response; C5/C6 use ``retries=0``."""
        if retries < 0:
            raise ValueError("retries must be non-negative")
        with self.uart_lock:
            last_error: Exception | None = None
            for attempt in range(retries + 1):
                try:
                    raw, audit = self._request_once_locked(
                        frame,
                        expected_id=expected_id,
                        expected_command=expected_command,
                        expected_register=None,
                    )
                    self._request_audit.append(audit)
                    return raw
                except Exception as exc:
                    last_error = exc
                    if attempt >= retries:
                        raise
            raise J7AdapterError(str(last_error))

    def audit_snapshot(self) -> tuple[J7RequestAudit, ...]:
        return tuple(self._request_audit)


@dataclass(frozen=True)
class StopAxisRecord:
    motor_id: int
    tx_start_ns: int
    tx_end_ns: int
    response_received: bool
    timeout: bool
    error: str | None = None


@dataclass(frozen=True)
class StopAudit:
    reason: str
    records: tuple[StopAxisRecord, ...]
    success: bool
    callback_count: int


class J7StopAdapter:
    """One-shot six-axis ``0x1A`` stop callback for C7/C8 runner wiring."""

    def __init__(self, uart: J7UartAdapter) -> None:
        self.uart = uart
        self._called = False
        self._callback_count = 0
        self._last_audit: StopAudit | None = None

    @property
    def called(self) -> bool:
        return self._called

    def __call__(self, reason: str) -> StopAudit:
        return self.stop_all(reason)

    def stop_all(self, reason: str) -> StopAudit:
        if self._called and self._last_audit is not None:
            return self._last_audit
        self._called = True
        self._callback_count += 1
        records: list[StopAxisRecord] = []
        with self.uart.uart_lock:
            for motor_id in MOTOR_IDS:
                start_ns = self.uart._clock_ns()
                try:
                    self.uart.request(
                        pack_write_register_cmd(motor_id, PAUSE_ADDR, [1]),
                        expected_id=motor_id,
                        expected_command=CMD_WR_REGISTER,
                        retries=0,
                    )
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=True,
                            timeout=False,
                        )
                    )
                except J7AdapterTimeout as exc:
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=False,
                            timeout=True,
                            error=str(exc),
                        )
                    )
                except Exception as exc:
                    records.append(
                        StopAxisRecord(
                            motor_id=motor_id,
                            tx_start_ns=start_ns,
                            tx_end_ns=self.uart._clock_ns(),
                            response_received=False,
                            timeout=False,
                            error=str(exc),
                        )
                    )
        self._last_audit = StopAudit(
            reason=str(reason),
            records=tuple(records),
            success=all(record.response_received for record in records),
            callback_count=self._callback_count,
        )
        return self._last_audit

    def audit_snapshot(self) -> StopAudit | None:
        return self._last_audit


__all__ = [
    "J7AdapterError",
    "J7AdapterRxError",
    "J7AdapterTimeout",
    "J7RxOwner",
    "J7SerialLike",
    "J7StopAdapter",
    "J7UartAdapter",
    "J7RequestAudit",
    "RxOwnerAudit",
    "StopAudit",
    "StopAxisRecord",
]
