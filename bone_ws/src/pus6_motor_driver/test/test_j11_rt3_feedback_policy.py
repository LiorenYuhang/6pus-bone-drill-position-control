"""Tests for J11 two-tier feedback-age safety policy and J7/J8 baseline compatibility."""

from __future__ import annotations

from pathlib import Path
import sys
import pytest
import rclpy
from std_msgs.msg import Int32MultiArray

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.fake_bus import FakeBus, MockServoBus
from pus6_motor_driver.j7_servo_feedback_scheduler import (
    FeedbackSchedulerError,
    J7ServoFeedbackScheduler,
)
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_transport import ServoRxPolicy
from pus6_motor_driver.protocol import StatusFeedback
from pus6_motor_driver.safety import MOTOR_IDS, MAX_CURRENT_MA
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class SyntheticClock:
    def __init__(self, start_ns: int = 1_000_000_000) -> None:
        self.now = start_ns

    def now_ns(self) -> int:
        return self.now

    def advance_ms(self, ms: float) -> None:
        self.now += int(ms * 1e6)


def make_test_scheduler(
    *,
    policy: str = "default",
    max_feedback_age_s: float | None = None,
    warning_feedback_age_s: float = 0.2833,
    hard_stop_feedback_age_s: float = 1.0,
    timeout_escalation_misses: int = 4,
    clock: SyntheticClock | None = None,
    warn_events: list[str] | None = None,
    pause_events: list[str] | None = None,
) -> tuple[SyntheticClock, J7ServoFeedbackScheduler, J7SixAxisServoTransport]:
    clk = clock or SyntheticClock()
    bus = FakeBus()
    pauses = pause_events if pause_events is not None else []
    writer = J7SixAxisServoTransport(
        bus,
        cycle_rate_hz=50.0,
        rx_policy=ServoRxPolicy.DISCARD_BEFORE_TX,
        clock_ns=clk.now_ns,
        pause_callback=pauses.append,
    )
    warns = warn_events if warn_events is not None else []
    scheduler = J7ServoFeedbackScheduler(
        writer,
        full_scan_period_s=1e9,
        feedback_budget_s=0.015,
        servo_cycle_period_s=0.020,
        timeout_escalation_misses=timeout_escalation_misses,
        max_feedback_age_s=max_feedback_age_s,
        warning_feedback_age_s=warning_feedback_age_s,
        hard_stop_feedback_age_s=hard_stop_feedback_age_s,
        clock_ns=clk.now_ns,
        feedback_loss_policy=policy,
        warn_callback=warns.append,
    )
    return clk, scheduler, writer


def healthy_feedbacks(pos: int = 1000, current: int = 0, fault: int = 0) -> dict[int, StatusFeedback]:
    return {
        mid: StatusFeedback(
            target_position=pos,
            current_position=pos,
            current_mA=current,
            force_g=0,
            force_adc=0,
            temperature=25,
            error_code=fault,
        )
        for mid in MOTOR_IDS
    }


def test_1_stale_progression_200ms_300ms_600ms_recovery_and_hard_stop():
    """Verify:
    - 200ms stale -> NORMAL, not STOP
    - 300ms stale -> WARNING, not STOP
    - 600ms stale -> WARNING, not STOP
    - feedback recovers after warning -> NORMAL continues
    - >1.0s stale -> HARD STOP
    """
    warn_events: list[str] = []
    pauses: list[str] = []
    clk, scheduler, writer = make_test_scheduler(
        policy="age_based",
        warning_feedback_age_s=0.2833,
        hard_stop_feedback_age_s=1.0,
        warn_events=warn_events,
        pause_events=pauses,
    )

    # Initial cycle at t=0: all 6 axes healthy
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.feedback_age_watchdog_armed
    assert not scheduler.stopped
    assert not writer.stopped
    assert scheduler.is_feedback_normal()
    assert not scheduler.is_feedback_stale_warning()

    feedbacks_no_id5 = {mid: fb for mid, fb in healthy_feedbacks().items() if mid != 5}

    # 1. 200ms stale on ID5 (10 cycles of 20ms)
    for c in range(1, 11):
        clk.advance_ms(20.0)
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=c, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.feedback_age_ms(5) == pytest.approx(200.0, abs=0.1)
    assert not scheduler.stopped
    assert scheduler.is_feedback_normal()  # <= 283.3ms is NORMAL
    assert not scheduler.is_feedback_stale_warning()

    # 2. 300ms stale on ID5 (5 more cycles of 20ms = 300ms)
    for c in range(11, 16):
        clk.advance_ms(20.0)
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=c, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.feedback_age_ms(5) == pytest.approx(300.0, abs=0.1)
    assert not scheduler.stopped
    assert not scheduler.is_feedback_normal()
    assert scheduler.is_feedback_stale_warning()  # 283.3ms < age <= 1.0s is WARNING
    assert any("ID5" in w and "warning" in w for w in warn_events)

    # 3. 600ms stale on ID5 (15 more cycles of 20ms = 600ms)
    for c in range(16, 31):
        clk.advance_ms(20.0)
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=c, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.feedback_age_ms(5) == pytest.approx(600.0, abs=0.1)
    assert not scheduler.stopped
    assert scheduler.is_feedback_stale_warning()

    # 4. Feedback recovers after warning -> normal continues
    clk.advance_ms(20.0)
    scheduler.ingest_cycle_feedback(healthy_feedbacks(pos=998), servo_cycle_id=31, servo_cycle_timestamp_ns=clk.now_ns())
    assert not scheduler.stopped
    assert scheduler.consecutive_timeouts(5) == 0
    assert scheduler.feedback_age_ms(5) == 0.0
    assert scheduler.is_feedback_normal()
    assert not scheduler.is_feedback_stale_warning()

    # 5. Missing again -> progresses beyond 1.0s (51 cycles of 20ms = 1020ms)
    for c in range(32, 82):  # 50 cycles = 1000ms
        clk.advance_ms(20.0)
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=c, servo_cycle_timestamp_ns=clk.now_ns())
        assert not scheduler.stopped
    assert scheduler.feedback_age_ms(5) == pytest.approx(1000.0, abs=0.1)

    # 51st cycle: 1020ms > 1000ms -> HARD STOP!
    clk.advance_ms(20.0)
    with pytest.raises(FeedbackSchedulerError, match="feedback age exceeded"):
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=82, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "feedback_age_exceeded_ID5"
    assert "feedback_age_exceeded_ID5" in pauses


def test_2_scheduler_gap_watchdog_warning_and_hard_stop():
    """Verify Tier 2 execution gap watchdog:
    - gap <= 283.3ms -> NORMAL
    - 283.3ms < gap <= 1.0s -> WARNING, no STOP
    - gap > 1.0s -> HARD STOP (scheduler_deadline_exceeded)
    """
    warn_events: list[str] = []
    pauses: list[str] = []
    clk, scheduler, writer = make_test_scheduler(
        policy="age_based",
        warning_feedback_age_s=0.2833,
        hard_stop_feedback_age_s=1.0,
        warn_events=warn_events,
        pause_events=pauses,
    )

    # Initial cycle at t=0: all 6 axes healthy
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())

    # Cycle 1: gap = 300ms (UART stall in previous cycle)
    clk.advance_ms(300.0)
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=1, servo_cycle_timestamp_ns=clk.now_ns())
    assert not scheduler.stopped
    assert any("Servo scheduler gap" in w for w in warn_events)

    # Cycle 2: gap = 600ms
    clk.advance_ms(600.0)
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=2, servo_cycle_timestamp_ns=clk.now_ns())
    assert not scheduler.stopped

    # Cycle 3: gap = 1050ms > 1000ms -> HARD STOP!
    clk.advance_ms(1050.0)
    with pytest.raises(FeedbackSchedulerError, match="deadline exceeded"):
        scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=3, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "scheduler_deadline_exceeded"
    assert "scheduler_deadline_exceeded" in pauses


def test_3_safety_stops_remain_immediate():
    """Verify hardware safety stops are NEVER relaxed:
    - overcurrent >= 500mA -> immediate STOP
    - motor hardware fault != 0 -> immediate STOP
    - actuator limit violation -> immediate STOP
    """
    # 3a. Overcurrent on ID4
    clk, scheduler, writer = make_test_scheduler(policy="age_based")
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    clk.advance_ms(20.0)
    overcurrent_fbs = healthy_feedbacks()
    overcurrent_fbs[4] = StatusFeedback(
        target_position=1000, current_position=1000, current_mA=501,
        force_g=0, force_adc=0, temperature=25, error_code=0
    )
    scheduler.ingest_cycle_feedback(overcurrent_fbs, servo_cycle_id=1, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "overcurrent_ID4"

    # 3b. Motor hardware fault on ID1
    clk, scheduler, writer = make_test_scheduler(policy="age_based")
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    clk.advance_ms(20.0)
    fault_fbs = healthy_feedbacks()
    fault_fbs[1] = StatusFeedback(
        target_position=1000, current_position=1000, current_mA=0,
        force_g=0, force_adc=0, temperature=25, error_code=0x0002
    )
    scheduler.ingest_cycle_feedback(fault_fbs, servo_cycle_id=1, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "motor_fault_ID1"

    # 3c. Actuator limit violation on ID6
    clk, scheduler, writer = make_test_scheduler(policy="age_based")
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    clk.advance_ms(20.0)
    limit_fbs = healthy_feedbacks()
    limit_fbs[6] = StatusFeedback(
        target_position=50000, current_position=50000, current_mA=0,
        force_g=0, force_adc=0, temperature=25, error_code=0
    )
    scheduler.ingest_cycle_feedback(limit_fbs, servo_cycle_id=1, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason == "feedback_position_out_of_range_ID6"


def test_4_default_j7_j8_frozen_baseline():
    """Verify default policy preserves strict J7/J8 frozen baseline:
    - 4 consecutive misses -> persistent_feedback_loss STOP
    - execution gap > 283.3ms -> scheduler_deadline_exceeded STOP
    """
    # 4a. 4 consecutive misses
    pauses: list[str] = []
    clk, scheduler, writer = make_test_scheduler(policy="default", pause_events=pauses)
    scheduler.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk.now_ns())
    feedbacks_no_id5 = {mid: fb for mid, fb in healthy_feedbacks().items() if mid != 5}
    for c in range(1, 4):
        clk.advance_ms(20.0)
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=c, servo_cycle_timestamp_ns=clk.now_ns())
        assert not scheduler.stopped
    clk.advance_ms(20.0)
    with pytest.raises(FeedbackSchedulerError):
        scheduler.ingest_cycle_feedback(feedbacks_no_id5, servo_cycle_id=4, servo_cycle_timestamp_ns=clk.now_ns())
    assert scheduler.stopped
    assert writer.stopped
    assert scheduler.audit_snapshot().stop_reason.startswith("persistent_")

    # 4b. execution gap > 283.3ms in default policy
    pauses2: list[str] = []
    clk2, scheduler2, writer2 = make_test_scheduler(policy="default", pause_events=pauses2)
    scheduler2.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=0, servo_cycle_timestamp_ns=clk2.now_ns())
    clk2.advance_ms(300.0)  # > 283.3ms
    with pytest.raises(FeedbackSchedulerError, match="deadline exceeded"):
        scheduler2.ingest_cycle_feedback(healthy_feedbacks(), servo_cycle_id=1, servo_cycle_timestamp_ns=clk2.now_ns())
    assert scheduler2.stopped
    assert writer2.stopped
    assert scheduler2.audit_snapshot().stop_reason == "scheduler_deadline_exceeded"


def test_5_motor_driver_node_holds_safe_target_during_warning(rclpy_init):
    """Verify Pus6MotorDriverNode holds recent safe target when feedback is in warning state."""
    node = Pus6MotorDriverNode(
        hardware_mode="mock",
        bus=MockServoBus(),
        feedback_loss_policy="age_based",
    )
    assert node.feedback_loss_policy == "age_based"
    assert node.j7_feedback.feedback_loss_policy == "age_based"
    assert node.j7_feedback.warning_feedback_age_ns == int(0.2833 * 1e9)
    assert node.j7_feedback.hard_stop_feedback_age_ns == int(1.0 * 1e9)

    # Initial state: positions initialized to 1000
    node._last_commanded_steps = [1000] * 6

    # When feedback is normal, a new incoming target is consumed
    msg = Int32MultiArray()
    msg.data = [1002] * 6
    node._handle_command_joint_steps(msg)
    assert node._latest_target is not None
    assert not node._latest_target.consumed

    # Execute cycle when normal: target consumed
    node._execute_servo_cycle()
    assert node._latest_target.consumed
    assert node._last_commanded_steps == [1002] * 6

    # Now simulate feedback entering warning state (age = 400ms)
    # Stale ID5 feedback
    now_ns = node.j7_feedback._clock_ns()
    node.j7_feedback._last_feedback_ns[5] = now_ns - int(0.400 * 1e9)
    assert node.j7_feedback.is_feedback_stale_warning(now_ns)

    # New target arrives while in warning
    msg2 = Int32MultiArray()
    msg2.data = [1005] * 6
    node._handle_command_joint_steps(msg2)
    assert not node._latest_target.consumed

    # Execute servo cycle: node must HOLD last commanded steps and NOT consume new target
    node._execute_servo_cycle()
    assert not node._latest_target.consumed  # Target not consumed while warning
    assert node._last_commanded_steps == [1002] * 6  # Held safe reference

    # Now feedback recovers
    node.j7_feedback._last_feedback_ns[5] = node.j7_feedback._clock_ns()
    assert not node.j7_feedback.is_feedback_stale_warning()

    # Execute servo cycle: node now consumes the new target
    node._execute_servo_cycle()
    assert node._latest_target.consumed
    assert node._last_commanded_steps == [1005] * 6

    node.destroy_node()
