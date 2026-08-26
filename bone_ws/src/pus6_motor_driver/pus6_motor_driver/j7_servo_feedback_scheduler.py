"""Isolated J7 Servo feedback scheduler.

Feedback is deliberately subordinate to the six-axis Servo writer: it uses
the writer's shared UART lock, acquires it without blocking, and defers when a
Servo deadline is imminent.  Motion response frames are never used as
feedback; runtime feedback uses status ``0x30`` only and validates the
requested ID and command.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import time

try:
    from .j7_six_axis_servo_transport import J7SixAxisServoTransport
    from .protocol import (
        CMD_RD_STATUS,
        FrameFieldError,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        pack_read_status_cmd,
        unpack_status_response,
        validate_response,
    )
    from .safety import MOTOR_IDS
except ImportError:  # pragma: no cover - direct module execution fallback.
    from j7_six_axis_servo_transport import J7SixAxisServoTransport  # type: ignore
    from protocol import (  # type: ignore
        CMD_RD_STATUS,
        FrameFieldError,
        FrameStreamParser,
        ProtocolError,
        RSP_FRAME_HEADER,
        pack_read_status_cmd,
        unpack_status_response,
        validate_response,
    )
    from safety import MOTOR_IDS  # type: ignore


class FeedbackSchedulerError(RuntimeError):
    """Base error for the isolated feedback scheduler."""


class FeedbackRxError(FeedbackSchedulerError):
    """Raised when RX bytes cannot be parsed or safely classified."""


@dataclass(frozen=True)
class FeedbackSample:
    motor_id: int
    timestamp_ns: int
    position: int | None
    current_mA: int | None
    fault: int | None
    mode: int | None
    feedback_age_ns: int
    timeout: bool
    error: str | None = None


@dataclass(frozen=True)
class FeedbackCycleAudit:
    servo_cycle_id: int
    feedback_cycle_id: int
    timestamp_ns: int
    motor_ids: tuple[int, ...]
    samples: tuple[FeedbackSample, ...]
    full_scan: bool
    deferred: bool
    defer_count: int
    timeout_count: int
    rx_residual_count: int
    stale_response_count: int
    mismatch_count: int
    malformed_count: int


@dataclass(frozen=True)
class FeedbackSchedulerAudit:
    cycles: tuple[FeedbackCycleAudit, ...]
    defer_count: int
    timeout_count: int
    rx_residual_count: int
    stale_response_count: int
    mismatch_count: int
    malformed_count: int
    stopped: bool
    stop_reason: str | None


class J7ServoFeedbackScheduler:
    """Round-robin feedback with an explicit low-frequency full scan."""

    def __init__(
        self,
        writer: J7SixAxisServoTransport,
        *,
        full_scan_period_s: float = 0.100,
        deadline_guard_s: float = 0.001,
        feedback_budget_s: float = 0.010,
        clock_ns: Callable[[], int] | None = None,
    ) -> None:
        if full_scan_period_s <= 0.0:
            raise ValueError("full_scan_period_s must be positive")
        if deadline_guard_s < 0.0 or feedback_budget_s <= 0.0:
            raise ValueError("deadline_guard_s must be non-negative and feedback_budget_s positive")
        self.writer = writer
        self.bus = writer.bus
        self.full_scan_period_ns = int(full_scan_period_s * 1e9)
        self.deadline_guard_ns = int(deadline_guard_s * 1e9)
        self.feedback_budget_ns = int(feedback_budget_s * 1e9)
        self._clock_ns = clock_ns or time.monotonic_ns
        self._uart_lock = writer.uart_lock
        self._parser = FrameStreamParser(expected_header=RSP_FRAME_HEADER)
        self._cycles: list[FeedbackCycleAudit] = []
        self._next_axis_index = 0
        self._feedback_cycle_id = 0
        self._last_full_scan_ns = self._clock_ns()
        self._defer_count = 0
        self._timeout_count = 0
        self._rx_residual_count = 0
        self._stale_response_count = 0
        self._mismatch_count = 0
        self._malformed_count = 0
        self._stopped = False
        self._stop_reason: str | None = None

    @property
    def stopped(self) -> bool:
        return self._stopped or self.writer.stopped

    def notify_servo_cycle(self, servo_cycle_id: int, timestamp_ns: int) -> None:
        """Validate the cycle identity supplied by the Servo writer."""
        if not isinstance(servo_cycle_id, int) or servo_cycle_id < 0:
            raise ValueError("servo_cycle_id must be a non-negative integer")
        if not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")

    def poll_once(
        self,
        *,
        servo_cycle_id: int,
        servo_cycle_timestamp_ns: int,
        servo_cycle_due_ns: int | None = None,
        force_full_scan: bool = False,
    ) -> FeedbackCycleAudit:
        """Poll one round-robin axis, or all axes when a full scan is due.

        The UART lock is acquired non-blocking.  A busy writer or an imminent
        Servo deadline produces an audit-only deferred cycle and performs no
        I/O.
        """
        self.notify_servo_cycle(servo_cycle_id, servo_cycle_timestamp_ns)
        if self.stopped:
            raise FeedbackSchedulerError("feedback scheduler is stopped")

        now_ns = self._clock_ns()
        if servo_cycle_due_ns is not None and now_ns + self.deadline_guard_ns >= servo_cycle_due_ns:
            return self._deferred_audit(
                servo_cycle_id, now_ns, full_scan=False, reason="servo_deadline"
            )
        if not self._uart_lock.acquire(blocking=False):
            return self._deferred_audit(
                servo_cycle_id, now_ns, full_scan=False, reason="uart_busy"
            )

        try:
            cycle_timestamp_ns = self._clock_ns()
            full_scan = force_full_scan or (
                cycle_timestamp_ns - self._last_full_scan_ns >= self.full_scan_period_ns
            )
            axis_ids = tuple(MOTOR_IDS) if full_scan else (MOTOR_IDS[self._next_axis_index],)
            if full_scan:
                self._last_full_scan_ns = cycle_timestamp_ns
                self._next_axis_index = 0
            else:
                self._next_axis_index = (self._next_axis_index + 1) % len(MOTOR_IDS)

            residual_count = self._clear_writer_and_bus_residual_locked()
            samples: list[FeedbackSample] = []
            timeout_count = 0
            for motor_id in axis_ids:
                sample = self._read_axis_locked(
                    motor_id,
                    servo_cycle_timestamp_ns,
                    residual_count,
                )
                samples.append(sample)
                if sample.timeout:
                    timeout_count += 1
                    break

            self._feedback_cycle_id += 1
            audit = FeedbackCycleAudit(
                servo_cycle_id=servo_cycle_id,
                feedback_cycle_id=self._feedback_cycle_id,
                timestamp_ns=cycle_timestamp_ns,
                motor_ids=axis_ids,
                samples=tuple(samples),
                full_scan=full_scan,
                deferred=False,
                defer_count=self._defer_count,
                timeout_count=timeout_count,
                rx_residual_count=residual_count,
                stale_response_count=self._stale_response_count,
                mismatch_count=self._mismatch_count,
                malformed_count=self._malformed_count,
            )
            self._cycles.append(audit)
            return audit
        finally:
            self._uart_lock.release()

    def full_scan(
        self,
        *,
        servo_cycle_id: int,
        servo_cycle_timestamp_ns: int,
        servo_cycle_due_ns: int | None = None,
    ) -> FeedbackCycleAudit:
        """Request a low-frequency full ID1-ID6 safety scan."""
        return self.poll_once(
            servo_cycle_id=servo_cycle_id,
            servo_cycle_timestamp_ns=servo_cycle_timestamp_ns,
            servo_cycle_due_ns=servo_cycle_due_ns,
            force_full_scan=True,
        )

    def _deferred_audit(
        self,
        servo_cycle_id: int,
        timestamp_ns: int,
        *,
        full_scan: bool,
        reason: str,
    ) -> FeedbackCycleAudit:
        self._defer_count += 1
        self._feedback_cycle_id += 1
        audit = FeedbackCycleAudit(
            servo_cycle_id=servo_cycle_id,
            feedback_cycle_id=self._feedback_cycle_id,
            timestamp_ns=timestamp_ns,
            motor_ids=tuple(),
            samples=tuple(),
            full_scan=full_scan,
            deferred=True,
            defer_count=self._defer_count,
            timeout_count=0,
            rx_residual_count=0,
            stale_response_count=self._stale_response_count,
            mismatch_count=self._mismatch_count,
            malformed_count=self._malformed_count,
        )
        self._cycles.append(audit)
        return audit

    def _clear_writer_and_bus_residual_locked(self) -> int:
        """Clear residuals before a feedback transaction, never as a response."""
        residual_count = 0
        try:
            record = self.writer.drain_residual(policy="discard_before_tx")
            residual_count += record.discarded_frames
        except Exception as exc:
            self._malformed_count += 1
            self._stop("rx_residual_failure")
            raise FeedbackRxError(str(exc)) from exc

        data = self.bus.read_available()
        if not data:
            self._rx_residual_count += residual_count
            self._stale_response_count += residual_count
            return residual_count
        try:
            frames = self._parser.feed(data)
        except Exception as exc:
            self._malformed_count += 1
            self._parser.clear()
            self._stop("malformed_feedback_rx")
            raise FeedbackRxError(str(exc)) from exc
        residual_count += len(frames)
        self._rx_residual_count += residual_count
        self._stale_response_count += len(frames)
        return residual_count

    def _read_axis_locked(
        self,
        motor_id: int,
        servo_cycle_timestamp_ns: int,
        residual_count: int,
    ) -> FeedbackSample:
        start_ns = self._clock_ns()
        try:
            status_raw = self.bus.request(
                pack_read_status_cmd(motor_id),
                expected_id=motor_id,
                expected_command=CMD_RD_STATUS,
                retries=0,
            )
            status_frame = validate_response(
                status_raw,
                expected_id=motor_id,
                expected_command=CMD_RD_STATUS,
            )
            status = unpack_status_response(status_frame.raw)
            end_ns = self._clock_ns()
            if end_ns - start_ns > self.feedback_budget_ns:
                self._timeout_count += 1
                self._stop("feedback_budget_timeout")
                return FeedbackSample(
                    motor_id=motor_id,
                    timestamp_ns=end_ns,
                    position=status.current_position,
                    current_mA=status.current_mA,
                    fault=status.error_code,
                    mode=None,
                    feedback_age_ns=max(0, end_ns - servo_cycle_timestamp_ns),
                    timeout=True,
                    error="feedback budget exceeded",
                )
            return FeedbackSample(
                motor_id=motor_id,
                timestamp_ns=end_ns,
                position=status.current_position,
                current_mA=status.current_mA,
                fault=status.error_code,
                mode=None,
                feedback_age_ns=max(0, end_ns - servo_cycle_timestamp_ns),
                timeout=False,
            )
        except TimeoutError as exc:
            self._timeout_count += 1
            self._stop("feedback_timeout")
            return FeedbackSample(
                motor_id=motor_id,
                timestamp_ns=self._clock_ns(),
                position=None,
                current_mA=None,
                fault=None,
                mode=None,
                feedback_age_ns=max(0, self._clock_ns() - servo_cycle_timestamp_ns),
                timeout=True,
                error=str(exc),
            )
        except FrameFieldError as exc:
            self._mismatch_count += 1
            self._stop("feedback_mismatch")
            raise FeedbackRxError(str(exc)) from exc
        except ProtocolError as exc:
            self._malformed_count += 1
            self._stop("malformed_feedback_response")
            raise FeedbackRxError(str(exc)) from exc
        except Exception as exc:
            self._timeout_count += 1
            self._stop("feedback_timeout")
            return FeedbackSample(
                motor_id=motor_id,
                timestamp_ns=self._clock_ns(),
                position=None,
                current_mA=None,
                fault=None,
                mode=None,
                feedback_age_ns=max(0, self._clock_ns() - servo_cycle_timestamp_ns),
                timeout=True,
                error=str(exc),
            )

    def _stop(self, reason: str) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._stop_reason = reason
        self.writer.stop(reason)

    def audit_snapshot(self) -> FeedbackSchedulerAudit:
        return FeedbackSchedulerAudit(
            cycles=tuple(self._cycles),
            defer_count=self._defer_count,
            timeout_count=self._timeout_count,
            rx_residual_count=self._rx_residual_count,
            stale_response_count=self._stale_response_count,
            mismatch_count=self._mismatch_count,
            malformed_count=self._malformed_count,
            stopped=self.stopped,
            stop_reason=self._stop_reason,
        )


__all__ = [
    "FeedbackCycleAudit",
    "FeedbackSample",
    "FeedbackSchedulerAudit",
    "FeedbackSchedulerError",
    "FeedbackRxError",
    "J7ServoFeedbackScheduler",
]
