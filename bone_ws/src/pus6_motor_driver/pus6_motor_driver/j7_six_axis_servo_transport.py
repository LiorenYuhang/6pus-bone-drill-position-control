"""Isolated six-axis Servo transport for the J7 experiment path.

The transport owns one injected bus and one lock.  Configuration is
request/response and verified per axis; motion cycles are ordered ID1..ID6
write-only ``0x29`` transmissions.  No serial device is opened here and the
module is intentionally independent of the ROS2 node and the frozen UART
modules.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Protocol
import threading
import time

try:
    from .j7_servo_transport import (
        RxResidualRecord,
        RxSafetyError,
        ServoConfig,
        ServoRxPolicy,
        ServoTransportError,
    )
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
        validate_target_vector,
    )
except ImportError:  # pragma: no cover - direct module execution fallback.
    from j7_servo_transport import (  # type: ignore
        RxResidualRecord,
        RxSafetyError,
        ServoConfig,
        ServoRxPolicy,
        ServoTransportError,
    )
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
        validate_target_vector,
    )


class SixAxisServoBus(Protocol):
    """Injected bus surface used by configuration and the write-only stream."""

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
class SixAxisTxRecord:
    motor_id: int
    cycle_id: int
    target_position: int
    target_timestamp_ns: int
    tx_start_ns: int
    tx_end_ns: int
    response_wait: bool
    retry_count: int
    residual_frames_before: int
    residual_frames_after: int


@dataclass(frozen=True)
class SixAxisCycleAudit:
    cycle_id: int
    target_timestamp_ns: int
    cycle_start_ns: int
    cycle_end_ns: int
    cycle_jitter_ns: int
    target_age_ns: int
    tx_records: tuple[SixAxisTxRecord, ...]
    tx_skew_ns: int
    batch_complete_ns: int
    deadline_miss: bool


@dataclass(frozen=True)
class SixAxisServoTransportAudit:
    cycles: tuple[SixAxisCycleAudit, ...]
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


class J7SixAxisServoTransport:
    """Single-writer six-axis Servo transport.

    ``cycle_rate_hz`` is fixed to 50 Hz for J7-C5.  The class does not
    implement feedback scheduling; a future scheduler must use this same bus
    and lock boundary when it is added.
    """

    def __init__(
        self,
        bus: SixAxisServoBus,
        *,
        cycle_rate_hz: float = 50.0,
        rx_policy: ServoRxPolicy = ServoRxPolicy.DEFER_TO_FEEDBACK,
        max_residual_frames: int = 192,
        max_residual_bytes: int = 24_576,
        min_frame_interval_s: float = 0.001,
        clock_ns: Callable[[], int] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        pause_callback: Callable[[str], None] | None = None,
    ) -> None:
        if cycle_rate_hz != 50.0:
            raise ValueError("J7-C5 six-axis Servo transport requires a fixed 50 Hz cycle")
        if max_residual_frames < 1 or max_residual_bytes < 1:
            raise ValueError("RX residual limits must be positive")
        if min_frame_interval_s < 0.0:
            raise ValueError("min_frame_interval_s must be non-negative")

        self.bus = bus
        self.cycle_rate_hz = cycle_rate_hz
        self.period_ns = int(1e9 / cycle_rate_hz)
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
        self._rx_records: list[RxResidualRecord] = []
        self._cycles: list[SixAxisCycleAudit] = []
        self._configured: tuple[ServoConfig, ...] | None = None
        self._last_tx_end_ns: int | None = None
        self._last_cycle_start_ns: int | None = None
        self._stopped = False
        self._stop_reason: str | None = None
        self._pause_sent = False
        self._delivery_uncertain = False
        self._malformed_count = 0
        self._response_wait_count = 0
        self._motion_retry_count = 0

    @property
    def configured(self) -> tuple[ServoConfig, ...] | None:
        return self._configured

    @property
    def uart_lock(self) -> threading.RLock:
        """Shared serialization boundary for the J7 feedback scheduler."""
        return self._lock

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def residual_frame_count(self) -> int:
        return len(self._residual)

    @property
    def residual_bytes(self) -> int:
        return sum(len(item.frame.raw) for item in self._residual)

    def configure_servo(self, *, speed: int) -> tuple[ServoConfig, ...]:
        """Enter Servo mode with acknowledged writes, without runtime readback.

        The mode/speed payload is sent as a normal request so every axis must
        acknowledge the command.  The returned readback fields stay ``None``:
        runtime Servo entry does not depend on ``0x31`` reads of ``0x25`` or
        ``0x28``.
        """
        if not isinstance(speed, int) or speed < 0 or speed > 0xFFFF:
            raise ValueError("speed must be an unsigned 16-bit integer")
        with self._lock:
            self._ensure_running_locked()
            try:
                self._ingest_rx_locked()
                self._drain_locked(ServoRxPolicy.DISCARD_BEFORE_TX)
                configs: list[ServoConfig] = []
                for motor_id in MOTOR_IDS:
                    response = self.bus.request(
                        pack_write_register_cmd(
                            motor_id, CONTROL_MODE_ADDR, [1, 0, 0, speed]
                        ),
                        expected_id=motor_id,
                        expected_command=CMD_WR_REGISTER,
                        retries=0,
                    )
                    status = unpack_write_response(response)
                    if status is None:
                        raise ServoTransportError(
                            f"ID{motor_id} Servo mode command response is invalid"
                        )
                    if status.error_code != 0:
                        raise ServoTransportError(
                            f"ID{motor_id} Servo mode command fault={status.error_code}"
                        )
                    configs.append(ServoConfig(motor_id, 1, speed, None, None))
            except Exception:
                self._delivery_uncertain = True
                self.stop("servo_startup_write_failure")
                raise
            self._configured = tuple(configs)
            self._last_cycle_start_ns = None
            return self._configured

    def configure_servo_for_stream(self, *, speed: int = 50) -> tuple[ServoConfig, ...]:
        """Compatibility alias for the acknowledged Servo configuration path."""
        return self.configure_servo(speed=speed)

    def _read_register_locked(self, motor_id: int, register_address: int) -> int:
        raw = self.bus.request(
            pack_read_register_cmd(motor_id, register_address),
            expected_id=motor_id,
            expected_command=CMD_RD_REGISTER,
            retries=0,
        )
        response = unpack_read_register_response(raw, register_address, 1)
        return response.values[0]

    def send_cycle(
        self,
        targets: list[int] | tuple[int, ...],
        *,
        cycle_id: int,
        target_timestamp_ns: int | None = None,
    ) -> SixAxisCycleAudit:
        """Send one ordered six-axis write-only ``0x29`` batch."""
        if not isinstance(cycle_id, int) or cycle_id < 0:
            raise ValueError("cycle_id must be a non-negative integer")
        validate_target_vector(targets)
        with self._lock:
            self._ensure_running_locked()
            if self._configured is None:
                raise ServoTransportError("configure_servo() must succeed before motion TX")

            timestamp_ns = self._clock_ns() if target_timestamp_ns is None else target_timestamp_ns
            if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
                raise ValueError("target_timestamp_ns must be a non-negative integer")

            try:
                residual_before = self._prepare_motion_rx_locked()
            except RxSafetyError:
                self._delivery_uncertain = True
                self.stop("rx_safety_failure")
                raise

            cycle_start_ns = self._clock_ns()
            if self._last_cycle_start_ns is None:
                cycle_jitter_ns = 0
            else:
                cycle_jitter_ns = (
                    cycle_start_ns - self._last_cycle_start_ns - self.period_ns
                )

            records: list[SixAxisTxRecord] = []
            try:
                for motor_id, target in zip(MOTOR_IDS, targets):
                    self._wait_for_frame_interval_locked()
                    frame = pack_write_register_cmd(
                        motor_id, TARGET_POSITION_ADDR, [target]
                    )
                    tx_start_ns = self._clock_ns()
                    written = self.bus.write(frame)
                    if written is not None and written != len(frame):
                        raise ServoTransportError(
                            f"short Servo write on ID{motor_id}: {written}/{len(frame)} bytes"
                        )
                    tx_end_ns = self._clock_ns()
                    self._last_tx_end_ns = tx_end_ns
                    records.append(
                        SixAxisTxRecord(
                            motor_id=motor_id,
                            cycle_id=cycle_id,
                            target_position=target,
                            target_timestamp_ns=timestamp_ns,
                            tx_start_ns=tx_start_ns,
                            tx_end_ns=tx_end_ns,
                            response_wait=False,
                            retry_count=0,
                            residual_frames_before=(
                                residual_before if not records else len(self._residual)
                            ),
                            residual_frames_after=len(self._residual),
                        )
                    )
            except Exception:
                self._delivery_uncertain = True
                self.stop("motion_write_failure")
                raise

            batch_complete_ns = self._clock_ns()
            cycle_end_ns = batch_complete_ns
            tx_starts = [record.tx_start_ns for record in records]
            tx_skew_ns = max(tx_starts) - min(tx_starts)
            audit = SixAxisCycleAudit(
                cycle_id=cycle_id,
                target_timestamp_ns=timestamp_ns,
                cycle_start_ns=cycle_start_ns,
                cycle_end_ns=cycle_end_ns,
                cycle_jitter_ns=cycle_jitter_ns,
                target_age_ns=cycle_start_ns - timestamp_ns,
                tx_records=tuple(records),
                tx_skew_ns=tx_skew_ns,
                batch_complete_ns=batch_complete_ns,
                deadline_miss=(batch_complete_ns - cycle_start_ns) > self.period_ns,
            )
            self._cycles.append(audit)
            self._last_cycle_start_ns = cycle_start_ns
            return audit

    def _wait_for_frame_interval_locked(self) -> None:
        if self._last_tx_end_ns is None or self.min_frame_interval_ns == 0:
            return
        remaining_ns = self.min_frame_interval_ns - (
            self._clock_ns() - self._last_tx_end_ns
        )
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
        received_ns = self._clock_ns()
        for frame in frames:
            self._residual.append(_ResidualFrame(frame, received_ns))
        if (
            len(self._residual) > self.max_residual_frames
            or self.residual_bytes > self.max_residual_bytes
        ):
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
                f"unexpected RX residual before Servo batch: {len(self._residual)} frame(s)"
            )
        return before

    def _drain_locked(self, policy: ServoRxPolicy) -> RxResidualRecord:
        before = len(self._residual)
        discarded = 0
        discarded_bytes = 0
        if policy == ServoRxPolicy.DISCARD_BEFORE_TX:
            discarded = before
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
        """Ingest and classify pending RX without issuing a motion frame."""
        with self._lock:
            selected = ServoRxPolicy(policy or self.rx_policy)
            self._ingest_rx_locked()
            return self._drain_locked(selected)

    def stop(self, reason: str) -> None:
        """Enter a one-way stopped state and invoke the pause hook once."""
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop_reason = str(reason)
            if self._pause_callback is not None and not self._pause_sent:
                self._pause_sent = True
                self._pause_callback(self._stop_reason)

    def audit_snapshot(self) -> SixAxisServoTransportAudit:
        with self._lock:
            return SixAxisServoTransportAudit(
                cycles=tuple(self._cycles),
                rx_records=tuple(self._rx_records),
                response_wait_count=self._response_wait_count,
                motion_retry_count=self._motion_retry_count,
                delivery_uncertain=self._delivery_uncertain,
                stopped=self._stopped,
                stop_reason=self._stop_reason,
            )

    def _ensure_running_locked(self) -> None:
        if self._stopped:
            raise ServoTransportError("transport is stopped")


__all__ = [
    "J7SixAxisServoTransport",
    "SixAxisCycleAudit",
    "SixAxisServoBus",
    "SixAxisServoTransportAudit",
    "SixAxisTxRecord",
]
