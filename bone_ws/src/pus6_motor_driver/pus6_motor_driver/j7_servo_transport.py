"""J7 isolated single-axis Servo transport.

This module intentionally does not use :class:`UartBus` or open a serial
device.  A caller injects a bus object implementing the small protocol below.
Configuration uses request/response; motion writes use the injected ``write``
method and never read or retry a ``0x29`` frame.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import time
import threading
from typing import Any, Callable, Deque, Protocol

try:
    from .protocol import (
        CMD_RD_REGISTER,
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        RSP_FRAME_HEADER,
        pack_read_register_cmd,
        pack_write_register_cmd,
        unpack_read_register_response,
        unpack_write_response,
    )
    from .safety import (
        CONTROL_MODE_ADDR,
        MOTOR_IDS,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
    )
except ImportError:  # pragma: no cover - direct module execution fallback.
    from protocol import (  # type: ignore
        CMD_RD_REGISTER,
        CMD_WR_REGISTER,
        Frame,
        FrameStreamParser,
        RSP_FRAME_HEADER,
        pack_read_register_cmd,
        pack_write_register_cmd,
        unpack_read_register_response,
        unpack_write_response,
    )
    from safety import (  # type: ignore
        CONTROL_MODE_ADDR,
        MOTOR_IDS,
        TARGET_POSITION_ADDR,
        TARGET_SPEED_ADDR,
    )


class ServoTransportError(RuntimeError):
    """Base error for the isolated J7 transport."""


class RxSafetyError(ServoTransportError):
    """Raised when residual RX state cannot be safely bounded or matched."""


class ServoRxPolicy(str, Enum):
    """How residual response frames are handled before a motion TX."""

    DISCARD_BEFORE_TX = "discard_before_tx"
    DEFER_TO_FEEDBACK = "defer_to_feedback"
    REJECT_RESIDUAL = "reject_residual"


class ServoBus(Protocol):
    """Dependency-injection surface; no serial implementation is supplied here."""

    def request(
        self,
        frame: bytes,
        *,
        expected_id: int | None = None,
        expected_command: int | None = None,
        retries: int = 0,
    ) -> bytes: ...

    def write(self, frame: bytes) -> int | None: ...

    def read_available(self) -> bytes: ...


@dataclass(frozen=True)
class ServoConfig:
    motor_id: int
    mode: int
    speed: int
    mode_readback: int | None
    speed_readback: int | None


@dataclass(frozen=True)
class ServoTxRecord:
    motor_id: int
    cycle_id: int
    target_position: int
    register_address: int
    tx_start_ns: int
    tx_end_ns: int
    response_wait: bool
    retry_count: int
    residual_frames_before: int
    residual_frames_after: int


@dataclass(frozen=True)
class RxResidualRecord:
    timestamp_ns: int
    policy: ServoRxPolicy
    residual_frames_before: int
    discarded_frames: int
    discarded_bytes: int
    malformed_frames: int
    residual_frames_after: int


@dataclass(frozen=True)
class FeedbackRequestPlaceholder:
    """Descriptor reserved for a later feedback scheduler; it performs no I/O."""

    motor_id: int
    kind: str
    created_ns: int
    implemented: bool = False


@dataclass(frozen=True)
class ServoTransportAudit:
    tx_records: tuple[ServoTxRecord, ...]
    rx_records: tuple[RxResidualRecord, ...]
    response_wait_count: int
    motion_retry_count: int
    delivery_uncertain: bool
    stopped: bool
    stop_reason: str | None


@dataclass(frozen=True)
class _ResidualFrame:
    frame: Frame
    received_ns: int


class J7SingleAxisServoTransport:
    """Minimal, lock-serialized Servo transport for one motor ID.

    The class is deliberately single-axis.  It is suitable for Mock testing
    and for a future explicitly authorized single-motor hardware adapter, but
    it does not create a serial port or implement a six-axis scheduler.
    """

    def __init__(
        self,
        bus: ServoBus,
        motor_id: int,
        *,
        rx_policy: ServoRxPolicy = ServoRxPolicy.DEFER_TO_FEEDBACK,
        max_residual_frames: int = 32,
        max_residual_bytes: int = 4096,
        min_frame_interval_s: float = 0.001,
        clock_ns: Callable[[], int] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        pause_callback: Callable[[str], None] | None = None,
    ) -> None:
        if motor_id not in MOTOR_IDS:
            raise ValueError(f"motor_id must be one of {MOTOR_IDS}, got {motor_id}")
        if max_residual_frames < 1 or max_residual_bytes < 1:
            raise ValueError("RX residual limits must be positive")
        if min_frame_interval_s < 0.0:
            raise ValueError("min_frame_interval_s must be non-negative")
        self.bus = bus
        self.motor_id = motor_id
        self.rx_policy = ServoRxPolicy(rx_policy)
        self.max_residual_frames = max_residual_frames
        self.max_residual_bytes = max_residual_bytes
        self.min_frame_interval_ns = int(min_frame_interval_s * 1e9)
        self._clock_ns = clock_ns or time.monotonic_ns
        self._sleep_fn = sleep_fn or time.sleep
        self._pause_callback = pause_callback
        self._lock = threading.RLock()
        self._parser = FrameStreamParser(expected_header=RSP_FRAME_HEADER)
        self._residual: Deque[_ResidualFrame] = deque()
        self._tx_records: list[ServoTxRecord] = []
        self._rx_records: list[RxResidualRecord] = []
        self._last_tx_end_ns: int | None = None
        self._configured: ServoConfig | None = None
        self._stopped = False
        self._stop_reason: str | None = None
        self._delivery_uncertain = False
        self._pause_sent = False
        self._response_wait_count = 0
        self._motion_retry_count = 0
        self._malformed_count = 0

    @property
    def configured(self) -> ServoConfig | None:
        return self._configured

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def residual_frame_count(self) -> int:
        return len(self._residual)

    @property
    def residual_bytes(self) -> int:
        return sum(len(item.frame.raw) for item in self._residual)

    def configure_servo(self, *, speed: int) -> ServoConfig:
        """Enter Servo mode after an acknowledged, fault-free mode command.

        Servo entry does not depend on diagnostic ``0x31`` readbacks.  The
        requested speed is included in the mode payload but is not read back;
        runtime Servo safety is provided by status feedback and watchdogs.
        """
        if not isinstance(speed, int) or speed < 0 or speed > 0xFFFF:
            raise ValueError("speed must be an unsigned 16-bit integer")
        with self._lock:
            if self._stopped:
                raise ServoTransportError("transport is stopped")
            try:
                self._ingest_rx_locked()
                self._drain_locked(ServoRxPolicy.DISCARD_BEFORE_TX)
                frame = pack_write_register_cmd(
                    self.motor_id,
                    CONTROL_MODE_ADDR,
                    [1, 0, 0, speed],
                )
                response = self.bus.request(
                    frame,
                    expected_id=self.motor_id,
                    expected_command=CMD_WR_REGISTER,
                    retries=0,
                )
                status = unpack_write_response(response)
                if status is None:
                    raise ServoTransportError("Servo mode command response is invalid")
                if status.error_code != 0:
                    raise ServoTransportError(
                        f"Servo mode command returned fault={status.error_code}"
                    )
                self._configured = ServoConfig(
                    motor_id=self.motor_id,
                    mode=1,
                    speed=speed,
                    mode_readback=None,
                    speed_readback=None,
                )
                return self._configured
            except RxSafetyError:
                self._configured = None
                self._delivery_uncertain = True
                self.stop("rx_safety_failure")
                raise
            except Exception:
                self._configured = None
                self._delivery_uncertain = True
                self.stop("servo_startup_write_failure")
                raise

    def _read_register_locked(self, register_address: int) -> int:
        frame = pack_read_register_cmd(self.motor_id, register_address)
        raw = self.bus.request(
            frame,
            expected_id=self.motor_id,
            expected_command=CMD_RD_REGISTER,
            retries=0,
        )
        result = unpack_read_register_response(raw, register_address, 1)
        return result.values[0]

    def write_only_position(self, target_position: int, *, cycle_id: int) -> ServoTxRecord:
        """Write one 0x29 target with no response read and no retry."""
        if not isinstance(target_position, int) or not 0 <= target_position <= 2000:
            raise ValueError("target_position must be an integer in [0, 2000]")
        if not isinstance(cycle_id, int) or cycle_id < 0:
            raise ValueError("cycle_id must be a non-negative integer")
        with self._lock:
            if self._configured is None:
                raise ServoTransportError("configure_servo() must succeed before motion TX")
            if self._stopped:
                raise ServoTransportError("transport is stopped")
            try:
                residual_before = self._prepare_motion_rx_locked()
            except RxSafetyError:
                self._delivery_uncertain = True
                self.stop("rx_safety_failure")
                raise
            self._wait_for_frame_interval_locked()
            frame = pack_write_register_cmd(
                self.motor_id,
                TARGET_POSITION_ADDR,
                [target_position],
            )
            tx_start_ns = self._clock_ns()
            try:
                written = self.bus.write(frame)
                if written is not None and written != len(frame):
                    raise ServoTransportError(
                        f"short Servo write: {written}/{len(frame)} bytes"
                    )
            except Exception:
                self._delivery_uncertain = True
                self.stop("motion_write_failure")
                raise
            tx_end_ns = self._clock_ns()
            self._last_tx_end_ns = tx_end_ns
            record = ServoTxRecord(
                motor_id=self.motor_id,
                cycle_id=cycle_id,
                target_position=target_position,
                register_address=TARGET_POSITION_ADDR,
                tx_start_ns=tx_start_ns,
                tx_end_ns=tx_end_ns,
                response_wait=False,
                retry_count=0,
                residual_frames_before=residual_before,
                residual_frames_after=len(self._residual),
            )
            self._tx_records.append(record)
            return record

    def _wait_for_frame_interval_locked(self) -> None:
        if self._last_tx_end_ns is None or self.min_frame_interval_ns == 0:
            return
        remaining_ns = self.min_frame_interval_ns - (self._clock_ns() - self._last_tx_end_ns)
        if remaining_ns > 0:
            self._sleep_fn(remaining_ns / 1e9)

    def _read_available_locked(self) -> bytes:
        reader = getattr(self.bus, "read_available", None)
        if reader is None:
            return b""
        data = reader()
        if data is None:
            return b""
        if not isinstance(data, (bytes, bytearray)):
            raise ServoTransportError("read_available() must return bytes-like data")
        return bytes(data)

    def _ingest_rx_locked(self) -> int:
        data = self._read_available_locked()
        if not data:
            return 0
        try:
            frames = self._parser.feed(data)
        except Exception as exc:
            self._malformed_count += 1
            self._parser.clear()
            raise RxSafetyError(f"malformed RX data: {exc}") from exc
        now_ns = self._clock_ns()
        for frame in frames:
            self._residual.append(_ResidualFrame(frame=frame, received_ns=now_ns))
        if self.residual_bytes > self.max_residual_bytes or len(self._residual) > self.max_residual_frames:
            raise RxSafetyError(
                "RX residual limit exceeded: "
                f"frames={len(self._residual)}/{self.max_residual_frames}, "
                f"bytes={self.residual_bytes}/{self.max_residual_bytes}"
            )
        return len(frames)

    def _prepare_motion_rx_locked(self) -> int:
        before = len(self._residual)
        self._ingest_rx_locked()
        if self.rx_policy == ServoRxPolicy.DISCARD_BEFORE_TX:
            self._drain_locked(self.rx_policy)
        elif self.rx_policy == ServoRxPolicy.REJECT_RESIDUAL and self._residual:
            raise RxSafetyError(
                f"unexpected RX residual before Servo TX: {len(self._residual)} frame(s)"
            )
        return before

    def _drain_locked(self, policy: ServoRxPolicy) -> RxResidualRecord:
        before = len(self._residual)
        discarded = 0
        discarded_bytes = 0
        if policy == ServoRxPolicy.DISCARD_BEFORE_TX:
            discarded = len(self._residual)
            discarded_bytes = self.residual_bytes
            self._residual.clear()
        record = RxResidualRecord(
            timestamp_ns=self._clock_ns(),
            policy=policy,
            residual_frames_before=before,
            discarded_frames=discarded,
            discarded_bytes=discarded_bytes,
            malformed_frames=self._malformed_count,
            residual_frames_after=len(self._residual),
        )
        self._rx_records.append(record)
        return record

    def drain_residual(self, *, policy: ServoRxPolicy | None = None) -> RxResidualRecord:
        """Ingest and classify pending RX data without issuing motion TX."""
        with self._lock:
            selected = ServoRxPolicy(policy or self.rx_policy)
            self._ingest_rx_locked()
            return self._drain_locked(selected)

    def feedback_request(self, *, kind: str = "status") -> FeedbackRequestPlaceholder:
        """Return a non-I/O placeholder for the future feedback scheduler."""
        if kind not in {"status", "position"}:
            raise ValueError("feedback kind must be 'status' or 'position'")
        return FeedbackRequestPlaceholder(
            motor_id=self.motor_id,
            kind=kind,
            created_ns=self._clock_ns(),
        )

    def stop(self, reason: str) -> None:
        """Enter a one-way stopped state and invoke an optional pause hook once."""
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop_reason = str(reason)
            if self._pause_callback is not None and not self._pause_sent:
                self._pause_sent = True
                self._pause_callback(self._stop_reason)

    def audit_snapshot(self) -> ServoTransportAudit:
        with self._lock:
            return ServoTransportAudit(
                tx_records=tuple(self._tx_records),
                rx_records=tuple(self._rx_records),
                response_wait_count=self._response_wait_count,
                motion_retry_count=self._motion_retry_count,
                delivery_uncertain=self._delivery_uncertain,
                stopped=self._stopped,
                stop_reason=self._stop_reason,
            )


__all__ = [
    "FeedbackRequestPlaceholder",
    "J7SingleAxisServoTransport",
    "RxSafetyError",
    "ServoBus",
    "ServoConfig",
    "ServoRxPolicy",
    "ServoTransportAudit",
    "ServoTransportError",
    "ServoTxRecord",
]
