"""Isolated J7 six-axis Servo experiment runner.

This runner composes the J7-C5 write-only transport and J7-C6 feedback
scheduler behind a small explicit state machine.  It has no ROS2 or serial
opening logic; callers inject a transport backed by Mock/FakeBus for software
validation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable
import time

try:
    from .j7_servo_feedback_scheduler import (
        FeedbackCycleAudit,
        FeedbackRxError,
        J7ServoFeedbackScheduler,
    )
    from .j7_six_axis_servo_transport import J7SixAxisServoTransport, SixAxisCycleAudit
    from .safety import MOTOR_IDS, validate_target_vector
except ImportError:  # pragma: no cover - direct module execution fallback.
    from j7_servo_feedback_scheduler import (  # type: ignore
        FeedbackCycleAudit,
        FeedbackRxError,
        J7ServoFeedbackScheduler,
    )
    from j7_six_axis_servo_transport import (  # type: ignore
        J7SixAxisServoTransport,
        SixAxisCycleAudit,
    )
    from safety import MOTOR_IDS, validate_target_vector  # type: ignore


class RunnerState(str, Enum):
    INIT = "INIT"
    CHECK = "CHECK"
    CONFIG_SERVO = "CONFIG_SERVO"
    START_SERVO_STREAM = "START_SERVO_STREAM"
    VERIFY = "VERIFY"
    ARMED = "ARMED"
    SERVO_RUN = "SERVO_RUN"
    STOP = "STOP"


PARK_TARGET: tuple[int, ...] = (0, 0, 0, 0, 0, 0)
HOME_TARGET: tuple[int, ...] = (1000, 1000, 1000, 1000, 1000, 1000)


class ServoRunnerError(RuntimeError):
    """Base error for invalid state or failed J7 experiment gates."""


@dataclass(frozen=True)
class RunnerEvent:
    timestamp_ns: int
    state: RunnerState
    event: str
    reason: str | None = None


@dataclass(frozen=True)
class RunnerAudit:
    state: RunnerState
    events: tuple[RunnerEvent, ...]
    servo_cycles: tuple[SixAxisCycleAudit, ...]
    feedback_cycles: tuple[FeedbackCycleAudit, ...]
    target_overwrite_count: int
    watchdog_count: int
    stop_count: int
    stop_reason: str | None


class J7ServoExperimentRunner:
    """Software-only state machine for the first six-axis Servo experiment."""

    def __init__(
        self,
        transport: J7SixAxisServoTransport,
        feedback: J7ServoFeedbackScheduler,
        *,
        speed: int = 50,
        watchdog_timeout_s: float = 0.100,
        clock_ns: Callable[[], int] | None = None,
    ) -> None:
        if not isinstance(speed, int) or speed < 0 or speed > 0xFFFF:
            raise ValueError("speed must be an unsigned 16-bit integer")
        if watchdog_timeout_s <= 0.0:
            raise ValueError("watchdog_timeout_s must be positive")
        if feedback.writer is not transport:
            raise ValueError("feedback scheduler must use the same J7 Servo transport")
        self.transport = transport
        self.feedback = feedback
        self.speed = speed
        self.watchdog_timeout_ns = int(watchdog_timeout_s * 1e9)
        self._clock_ns = clock_ns or time.monotonic_ns
        self._state = RunnerState.INIT
        self._events: list[RunnerEvent] = []
        self._servo_cycles: list[SixAxisCycleAudit] = []
        self._feedback_cycles: list[FeedbackCycleAudit] = []
        self._last_target_timestamp_ns: int | None = None
        self._target_overwrite_count = 0
        self._watchdog_count = 0
        self._stop_count = 0
        self._stop_reason: str | None = None
        self._last_target: tuple[int, ...] | None = None
        self._current_positions: dict[int, int] = {}
        self._record("init")

    @property
    def state(self) -> RunnerState:
        return self._state

    @property
    def armed(self) -> bool:
        return self._state in (RunnerState.ARMED, RunnerState.SERVO_RUN)

    def check(self) -> FeedbackCycleAudit:
        """Run the pre-configuration six-axis health scan."""
        self._require(RunnerState.INIT)
        self._transition(RunnerState.CHECK, "check")
        try:
            audit = self.feedback.full_scan(
                servo_cycle_id=0,
                servo_cycle_timestamp_ns=self._clock_ns(),
            )
            self._feedback_cycles.append(audit)
            self._require_healthy_feedback(audit)
            self._current_positions = {
                sample.motor_id: sample.position
                for sample in audit.samples
                if sample.position is not None
            }
            return audit
        except Exception as exc:
            self.stop("check_failure")
            raise ServoRunnerError(str(exc)) from exc

    def configure_servo(self) -> None:
        """Configure all axes mode=1/speed and wait for transport readback."""
        self._require(RunnerState.CHECK)
        self._transition(RunnerState.CONFIG_SERVO, "configure_servo")
        try:
            configs = self.transport.configure_servo(speed=self.speed)
            if tuple(config.motor_id for config in configs) != MOTOR_IDS:
                raise ServoRunnerError("Servo configuration did not cover ID1..ID6")
            if any(config.mode != 1 for config in configs):
                raise ServoRunnerError("Servo mode write gate failed")
            self._record("config_servo_write_pass")
        except Exception as exc:
            self.stop("config_servo_failure")
            raise ServoRunnerError(str(exc)) from exc

    def verify(self) -> FeedbackCycleAudit:
        """Verify fault-free status feedback after configuration."""
        if self._state == RunnerState.CONFIG_SERVO:
            self._transition(RunnerState.VERIFY, "verify")
        elif self._state != RunnerState.SERVO_RUN:
            raise ServoRunnerError(f"runner state {self._state.value} does not permit verify")
        try:
            audit = self.feedback.full_scan(
                servo_cycle_id=0,
                servo_cycle_timestamp_ns=self._clock_ns(),
            )
            self._feedback_cycles.append(audit)
            self._require_healthy_feedback(audit)
            self._record("verify_pass")
            return audit
        except Exception as exc:
            self.stop("verify_failure")
            raise ServoRunnerError(str(exc)) from exc

    def start_servo_stream(
        self,
        targets: list[int] | tuple[int, ...] | None = None,
        *,
        cycle_id: int = 0,
        target_timestamp_ns: int | None = None,
    ) -> tuple[SixAxisCycleAudit, FeedbackCycleAudit]:
        """Send the first hold/current target immediately after mode writes."""
        self._require(RunnerState.CONFIG_SERVO)
        if targets is None:
            if tuple(sorted(self._current_positions)) != MOTOR_IDS:
                raise ServoRunnerError("q_current is unavailable for Servo startup")
            targets = tuple(self._current_positions[mid] for mid in MOTOR_IDS)
        validate_target_vector(targets)
        timestamp_ns = self._clock_ns() if target_timestamp_ns is None else target_timestamp_ns
        self._transition(RunnerState.START_SERVO_STREAM, "start_servo_stream")
        self._set_latest_target(tuple(targets), timestamp_ns)
        try:
            servo_audit = self.transport.send_cycle(
                list(targets), cycle_id=cycle_id, target_timestamp_ns=timestamp_ns
            )
            feedback_audit = self.feedback.poll_once(
                servo_cycle_id=cycle_id,
                servo_cycle_timestamp_ns=timestamp_ns,
            )
            self._servo_cycles.append(servo_audit)
            self._feedback_cycles.append(feedback_audit)
            self._require_runtime_feedback(feedback_audit, phase="startup")
            self._transition(RunnerState.SERVO_RUN, "servo_run")
            return servo_audit, feedback_audit
        except Exception as exc:
            if self._state != RunnerState.STOP:
                self.stop("servo_start_failure")
            raise ServoRunnerError(str(exc)) from exc

    def arm(self) -> None:
        """Enter ARMED only after CHECK, CONFIG_SERVO and VERIFY pass."""
        self._require(RunnerState.VERIFY)
        if self.transport.configured is None:
            raise ServoRunnerError("cannot arm before Servo configuration")
        self._transition(RunnerState.ARMED, "armed")

    def run_cycle(
        self,
        targets: list[int] | tuple[int, ...],
        *,
        cycle_id: int,
        target_timestamp_ns: int | None = None,
    ) -> tuple[SixAxisCycleAudit, FeedbackCycleAudit]:
        """Send one Servo batch and schedule its non-blocking feedback slot."""
        if self._state not in (RunnerState.ARMED, RunnerState.SERVO_RUN):
            raise ServoRunnerError(f"run_cycle invalid in state {self._state.value}")
        validate_target_vector(targets)
        timestamp_ns = self._clock_ns() if target_timestamp_ns is None else target_timestamp_ns
        self._set_latest_target(tuple(targets), timestamp_ns)
        if self._state == RunnerState.ARMED:
            self._transition(RunnerState.SERVO_RUN, "servo_run")
        try:
            servo_audit = self.transport.send_cycle(
                list(targets),
                cycle_id=cycle_id,
                target_timestamp_ns=timestamp_ns,
            )
            feedback_audit = self.feedback.poll_once(
                servo_cycle_id=cycle_id,
                servo_cycle_timestamp_ns=timestamp_ns,
            )
            self._servo_cycles.append(servo_audit)
            self._feedback_cycles.append(feedback_audit)
            if feedback_audit.deferred:
                self._record("feedback_deferred")
            self._require_runtime_feedback(feedback_audit, phase="runtime")
            return servo_audit, feedback_audit
        except Exception as exc:
            if self._state != RunnerState.STOP:
                feedback_reason = self.feedback.audit_snapshot().stop_reason
                self.stop(feedback_reason or "servo_cycle_failure")
            raise ServoRunnerError(str(exc)) from exc

    def watchdog_tick(self, *, now_ns: int | None = None) -> bool:
        """Stop when the latest target age exceeds the configured watchdog."""
        if self._state != RunnerState.SERVO_RUN or self._last_target_timestamp_ns is None:
            return False
        now = self._clock_ns() if now_ns is None else now_ns
        if now - self._last_target_timestamp_ns <= self.watchdog_timeout_ns:
            return False
        self._watchdog_count += 1
        self.stop("watchdog_timeout")
        return True

    def stop(self, reason: str = "requested") -> None:
        """Enter STOP once and invoke the transport stop hook once."""
        if self._state == RunnerState.STOP:
            return
        self._stop_count += 1
        self._stop_reason = str(reason)
        self._transition(RunnerState.STOP, "stop", reason=self._stop_reason)
        self.transport.stop(self._stop_reason)

    def audit_snapshot(self) -> RunnerAudit:
        return RunnerAudit(
            state=self._state,
            events=tuple(self._events),
            servo_cycles=tuple(self._servo_cycles),
            feedback_cycles=tuple(self._feedback_cycles),
            target_overwrite_count=self._target_overwrite_count,
            watchdog_count=self._watchdog_count,
            stop_count=self._stop_count,
            stop_reason=self._stop_reason,
        )

    def _require_healthy_feedback(self, audit: FeedbackCycleAudit) -> None:
        if audit.deferred or len(audit.samples) != len(MOTOR_IDS):
            raise ServoRunnerError("six-axis health scan was incomplete")
        for sample in audit.samples:
            if sample.timeout or sample.fault not in (0,):
                raise ServoRunnerError(
                    f"ID{sample.motor_id} health failure: fault={sample.fault}, timeout={sample.timeout}"
                )

    def _require_runtime_feedback(
        self,
        audit: FeedbackCycleAudit,
        *,
        phase: str,
    ) -> None:
        if audit.deferred:
            if phase == "startup":
                raise ServoRunnerError("startup feedback did not complete safely")
            return
        for sample in audit.samples:
            if sample.timeout:
                self.stop("feedback_timeout")
                raise ServoRunnerError(
                    f"ID{sample.motor_id} feedback timeout stopped Servo run"
                )
            if sample.fault not in (0,):
                self.stop("feedback_fault")
                raise ServoRunnerError(
                    f"ID{sample.motor_id} status fault={sample.fault} stopped Servo run"
                )

    def _set_latest_target(self, targets: tuple[int, ...], timestamp_ns: int) -> None:
        if self._last_target_timestamp_ns is not None:
            self._target_overwrite_count += 1
        self._last_target_timestamp_ns = timestamp_ns
        self._last_target = targets

    def _require(self, expected: RunnerState) -> None:
        if self._state != expected:
            raise ServoRunnerError(
                f"runner state {self._state.value} does not permit this operation; expected {expected.value}"
            )

    def _transition(self, state: RunnerState, event: str, reason: str | None = None) -> None:
        self._state = state
        self._record(event, reason=reason)

    def _record(self, event: str, reason: str | None = None) -> None:
        self._events.append(
            RunnerEvent(
                timestamp_ns=self._clock_ns(),
                state=self._state,
                event=event,
                reason=reason,
            )
        )


__all__ = [
    "HOME_TARGET",
    "J7ServoExperimentRunner",
    "PARK_TARGET",
    "RunnerAudit",
    "RunnerEvent",
    "RunnerState",
    "ServoRunnerError",
]
