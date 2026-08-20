"""Single-serial-port transport for one or more logical Motor objects."""

from __future__ import annotations

import logging
import threading
import time
from typing import List, Optional, Protocol

from protocol import (
    CMD_FRAME_HEADER,
    RSP_FRAME_HEADER,
    Frame,
    ProtocolError,
    FrameLengthError,
    bytes_to_hex,
    parse_frame,
    validate_response,
)

try:  # pyserial is only required when opening a real port.
    import serial as _serial
except ImportError:  # pragma: no cover - exercised only on minimal hosts.
    _serial = None


class SerialLike(Protocol):
    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes): ...


class BusError(Exception):
    """Base class for transport and response matching errors."""


class NoResponseError(BusError):
    """No complete response arrived before the transaction deadline."""


class TransportError(BusError):
    """The serial device could not send or receive a frame."""


class RetryExhaustedError(BusError):
    """All attempts for one transaction failed."""

    def __init__(self, attempts: int, last_error: Exception):
        super().__init__(f"UART transaction failed after {attempts} attempts: {last_error}")
        self.attempts = attempts
        self.last_error = last_error


logger = logging.getLogger(__name__)


class UartBus:
    """Own exactly one physical UART and serialize all transactions.

    ``serial_instance`` is intended for deterministic unit tests. Production
    callers should pass a port and let this class open the sole pyserial
    object. Motor objects receive this bus and never open a serial port.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        *,
        baudrate: int = 921600,
        timeout: float = 0.05,
        retries: int = 2,
        min_frame_interval: float = 0.001,
        serial_instance: Optional[SerialLike] = None,
    ) -> None:
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        if retries < 0:
            raise ValueError("retries must be non-negative")
        if min_frame_interval < 0:
            raise ValueError("min_frame_interval must be non-negative")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retries = retries
        self.min_frame_interval = min_frame_interval
        self._lock = threading.RLock()
        self._last_tx_time: Optional[float] = None
        self._owns_serial = serial_instance is None

        if serial_instance is not None:
            self.serial = serial_instance
        else:
            if _serial is None:
                raise BusError("pyserial is required to open a real UART port")
            if not port:
                raise ValueError("port is required when serial_instance is not supplied")
            try:
                self.serial = _serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=_serial.EIGHTBITS,
                    parity=_serial.PARITY_NONE,
                    stopbits=_serial.STOPBITS_ONE,
                    timeout=timeout,
                    write_timeout=timeout,
                )
            except Exception as exc:  # pyserial exception types vary by backend.
                raise TransportError(f"failed to open UART {port}: {exc}") from exc

        self._reset_input_buffer(reason="initialization")
        reset_output = getattr(self.serial, "reset_output_buffer", None)
        if callable(reset_output):
            reset_output()

    def close(self) -> None:
        close = getattr(self.serial, "close", None)
        if callable(close):
            close()

    def __enter__(self) -> "UartBus":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False

    @property
    def lock(self) -> threading.RLock:
        """Expose the shared lock for higher-level multi-step operations."""

        return self._lock

    def _reset_input_buffer(self, *, reason: str) -> None:
        reset = getattr(self.serial, "reset_input_buffer", None)
        if callable(reset):
            reset()
            logger.debug("UART RX buffer cleared (%s)", reason)
            return
        # A test double or unusual backend may not implement the helper.
        logger.debug("UART RX buffer clear unavailable (%s)", reason)

    def _wait_for_frame_interval(self) -> None:
        if self._last_tx_time is None:
            return
        remaining = self.min_frame_interval - (time.monotonic() - self._last_tx_time)
        if remaining > 0:
            time.sleep(remaining)

    def _send(self, frame: bytes) -> None:
        self._wait_for_frame_interval()
        self._reset_input_buffer(reason="before TX")
        logger.info("UART TX [%d] %s", len(frame), bytes_to_hex(frame))
        try:
            written = self.serial.write(frame)
            flush = getattr(self.serial, "flush", None)
            if callable(flush):
                flush()
        except Exception as exc:
            raise TransportError(f"UART write failed: {exc}") from exc
        if written is not None and written != len(frame):
            raise TransportError(f"short UART write: {written}/{len(frame)} bytes")
        self._last_tx_time = time.monotonic()

    def _read_exact(self, size: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise NoResponseError(f"UART read timed out after {self.timeout:.3f}s")
            old_timeout = getattr(self.serial, "timeout", None)
            try:
                if old_timeout is not None:
                    self.serial.timeout = min(float(old_timeout), remaining)
                chunk = self.serial.read(size - len(result))
            except Exception as exc:
                raise TransportError(f"UART read failed: {exc}") from exc
            finally:
                if old_timeout is not None:
                    self.serial.timeout = old_timeout
            if not chunk:
                raise NoResponseError(f"UART read timed out after {self.timeout:.3f}s")
            result.extend(chunk)
        return bytes(result)

    def _read_response_frame(self) -> bytes:
        deadline = time.monotonic() + self.timeout
        # Scan for AA 55 so a stale/noisy byte cannot shift the frame.
        while True:
            first = self._read_exact(1, deadline)
            if first != RSP_FRAME_HEADER[:1]:
                continue
            second = self._read_exact(1, deadline)
            if second == RSP_FRAME_HEADER[1:2]:
                break
        length = self._read_exact(1, deadline)[0]
        if length < 3:
            raise FrameLengthError(f"invalid response data length {length}")
        remainder = self._read_exact(length + 2, deadline)
        raw = RSP_FRAME_HEADER + bytes((length,)) + remainder
        parse_frame(raw, expected_header=RSP_FRAME_HEADER)
        logger.info("UART RX [%d] %s", len(raw), bytes_to_hex(raw))
        return raw

    def request_frames(
        self,
        frame: bytes,
        *,
        response_count: int = 1,
        expected_id: Optional[int] = None,
        expected_command: Optional[int] = None,
        retries: Optional[int] = None,
    ) -> List[bytes]:
        """Send once and read one or more complete length-delimited responses.

        A retry repeats the complete transaction after clearing residual input.
        During a multi-response transaction (SAVE_PARAMS), no buffer clear is
        performed between response frames.
        """

        if response_count < 1:
            raise ValueError("response_count must be positive")
        retry_count = self.retries if retries is None else retries
        if retry_count < 0:
            raise ValueError("retries must be non-negative")
        last_error: Exception = BusError("transaction did not run")
        with self._lock:
            for attempt in range(1, retry_count + 2):
                try:
                    self._send(frame)
                    responses = [self._read_response_frame() for _ in range(response_count)]
                    for raw in responses:
                        validate_response(
                            raw,
                            expected_id=expected_id,
                            expected_command=expected_command,
                        )
                    return responses
                except (BusError, ProtocolError) as exc:
                    last_error = exc
                    logger.warning(
                        "UART transaction attempt %d/%d failed: %s",
                        attempt,
                        retry_count + 1,
                        exc,
                    )
                    self._reset_input_buffer(reason="retry")
            raise RetryExhaustedError(retry_count + 1, last_error) from last_error

    def request(
        self,
        frame: bytes,
        *,
        expected_id: Optional[int] = None,
        expected_command: Optional[int] = None,
        retries: Optional[int] = None,
    ) -> bytes:
        """Send one command and return one validated raw response frame."""

        return self.request_frames(
            frame,
            response_count=1,
            expected_id=expected_id,
            expected_command=expected_command,
            retries=retries,
        )[0]
