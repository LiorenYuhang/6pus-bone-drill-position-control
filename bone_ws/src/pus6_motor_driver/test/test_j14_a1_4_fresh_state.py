"""A1.4 uses fake serial only; hardware_write_count is always zero."""
import json
import time

import pytest

from test_j14_a1_3_communication_recovery import make_recovery, fail, motion, RecoverySerial
from pus6_motor_driver.communication_recovery import CommunicationRecoveryError
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter, J7AdapterTimeout
from pus6_motor_driver.j7_servo_transport import ServoTransportError
from pus6_motor_driver.protocol import CMD_RD_STATUS, parse_frame, pack_read_status_cmd
from pus6_motor_driver.safety import PAUSE_ADDR


def fresh_recovery(tmp_path):
    objects = make_recovery(tmp_path)
    objects[5].fresh_state_only = True
    return objects


@pytest.mark.parametrize('axis', [1, 3, 6])
def test_six_fresh_samples_hold_without_motion_or_reanchor(axis, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer, axis)
    result = recovery.recover(exc, [1010]*6)
    event = result.event
    assert event['success'] and event['state'] == 'HOLD'
    assert event['motion_inhibited'] and not event['resumed']
    assert not anchors and result.verification_audit is None
    assert len(result.feedback_audit.samples) == 6
    assert all(s.timestamp_ns > event['refresh_epoch_ns'] for s in result.feedback_audit.samples)
    assert all(not s.timeout and s.fault == 0 and s.current_mA == 0 for s in result.feedback_audit.samples)
    assert len(motion(port)) == axis
    assert sum(parse_frame(f).command == CMD_RD_STATUS for f in port.frames) == 6
    assert sum(parse_frame(f).register_addr == PAUSE_ADDR for f in port.frames) == 6
    count = len(port.frames)
    for action in [lambda: writer.send_cycle([1000]*6, cycle_id=41),
                   lambda: writer.verify_recovery_hold([1000]*6, cycle_id=41),
                   lambda: writer.complete_communication_recovery(None)]:
        with pytest.raises(ServoTransportError):
            action()
    assert len(port.frames) == count and port.hardware_write_count == 0
    assert writer.communication_transient and writer.recovery_feedback_only and not writer.stopped


@pytest.mark.parametrize('kind', ['baseline_missing', 'fault_axis', 'overcurrent_axis', 'bad_position_axis'])
def test_incomplete_timeout_fault_or_current_fails_and_never_resumes(kind, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer)
    setattr(port, kind, 6)
    with pytest.raises(CommunicationRecoveryError):
        recovery.recover(exc, [1010]*6)
    final = json.loads((tmp_path/'recovery.jsonl').read_text().splitlines()[-1])
    assert not final['success'] and final['state'] == 'HARD_STOP_LATCH'
    assert len(final['fresh_status']) == (5 if kind == 'baseline_missing' else 6)
    assert writer.stopped and not anchors and len(motion(port)) == 3
    if kind == 'baseline_missing':
        query = json.loads((tmp_path/'fresh_state_acquisition.jsonl').read_text().splitlines()[-1])
        assert query['stage'] == 'QUERY_FAILED' and query['motor_id'] == 6
        assert query['tx_done_ns'] and not query['response_received'] and not query['reads']
    assert port.hardware_write_count == 0


@pytest.mark.parametrize('stale', ['rx_timestamp', 'tx_timestamp', 'unresolved_timeout'])
def test_old_acquisition_or_unresolved_transaction_is_rejected(stale, tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer)
    original = writer.request_recovery_status
    def old(mid, **kwargs):
        status, timestamp, record = original(mid, **kwargs)
        if stale == 'rx_timestamp':
            timestamp = kwargs['refresh_epoch_ns']-1
        elif stale == 'tx_timestamp':
            record['tx_start_ns'] = kwargs['refresh_epoch_ns']-1
        else:
            record['error'] = 'unresolved timeout'
        return status, timestamp, record
    writer.request_recovery_status = old
    with pytest.raises(CommunicationRecoveryError):
        recovery.recover(exc, [1010]*6)
    assert writer.stopped and not anchors and len(motion(port)) == 3


def test_six_stale_acquisitions_fail_even_when_statuses_are_complete(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer)
    statuses = [port.device.request(pack_read_status_cmd(m)) for m in range(1,7)]
    from pus6_motor_driver.protocol import unpack_status_response
    writer.request_recovery_status = lambda mid, **kw: (
        unpack_status_response(statuses[mid-1]), kw['refresh_epoch_ns']-1,
        dict(response_received=True, tx_start_ns=kw['refresh_epoch_ns']+1))
    with pytest.raises(CommunicationRecoveryError, match='not fresh'):
        recovery.recover(exc, [1010]*6)
    assert writer.stopped and not anchors


def test_stale_cache_and_partial_frame_are_discarded_then_only_new_query_used(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer)
    original = writer.request_recovery_status
    def stale_then_fresh(mid, **kwargs):
        # Completed cached response and a partial pre-query response must not win.
        port.device.positions[mid] = 1234
        stale = port.device.request(pack_read_status_cmd(mid))
        uart.rx_owner.ingest(stale)
        uart.rx_owner.ingest(stale[:8])
        port.device.positions[mid] = 1000
        return original(mid, **kwargs)
    writer.request_recovery_status = stale_then_fresh
    result = recovery.recover(exc, [1010]*6)
    assert result.positions == [1000]*6
    assert all(a['discarded_pre_query_frames'] for a in result.event['fresh_acquisition'].values())
    assert not anchors and port.hardware_write_count == 0


def test_existing_1000ms_epoch_is_not_reset(tmp_path):
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    exc = fail(port, writer)
    feedback._last_poll_completion_ns = time.monotonic_ns()-1_000_000_001
    with pytest.raises(CommunicationRecoveryError, match='1000ms'):
        recovery.recover(exc, [1010]*6)
    assert writer.stopped and not anchors


class DelayedSerial(RecoverySerial):
    def __init__(self):
        super().__init__()
        self.ready = False
    @property
    def in_waiting(self):
        return len(self.rx) if self.ready else 0


@pytest.mark.parametrize('new_path', [False, True])
def test_response_buffered_across_deadline_is_collected_before_timeout(new_path):
    port = DelayedSerial()
    now = [1_000_000_000]
    def sleep(_):
        now[0] += 2_000_000
        port.ready = True
    uart = J7UartAdapter(port, timeout_s=.001, min_frame_interval_s=.0001,
                         clock_ns=lambda: now[0], sleep_fn=sleep)
    if not new_path:
        with pytest.raises(J7AdapterTimeout):
            uart.request(pack_read_status_cmd(1), expected_id=1, expected_command=CMD_RD_STATUS, expected_register=0)
        assert port.in_waiting == 20  # Existing loop exits without collecting ready response.
    else:
        saved = []
        status, timestamp, record = uart.request_recovery_status(1, refresh_epoch_ns=now[0]-1,
            hard_deadline_ns=now[0]+1_000_000_000, observer=lambda r: saved.append(json.loads(json.dumps(r))))
        assert status.current_position == 1000 and timestamp > record['response_wait_deadline_ns']
        assert record['final_collection'] and record['completed_after_wait_deadline']
        assert len(port.frames) == 1 and record['retry_count'] == 0 and port.hardware_write_count == 0


def test_actual_driver_stays_in_hold_without_resetting_target_or_shaper(tmp_path):
    import rclpy
    from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
    from pus6_motor_driver.fake_bus import MockServoBus
    if not rclpy.ok():
        rclpy.init()
    node = Pus6MotorDriverNode(hardware_mode='mock', bus=MockServoBus(initial_positions=[1000]*6))
    for timer in [node.write_timer, node.poll_timer]:
        if timer is not None:
            timer.cancel()
    port, uart, pause, writer, feedback, recovery, anchors = fresh_recovery(tmp_path)
    node.j7_adapter, node.j7_stop_adapter, node.j7_writer, node.j7_feedback = uart, pause, writer, feedback
    node._communication_audit_dir = lambda: tmp_path
    node._fresh_state_only = True
    node._reanchor_communication_state = lambda *args: pytest.fail('must not re-anchor')
    initial_command = node._last_commanded_steps
    try:
        port.timeout_axis = 3
        node._execute_servo_cycle()
        assert node._fresh_state_hold and not node._j7_fault_latched
        assert node._communication_recovery.state == 'HOLD'
        count = len(port.frames)
        node._enqueue_latest_target([1005]*6)
        for _ in range(3):
            node._execute_servo_cycle()
        assert len(port.frames) == count and node._last_commanded_steps == initial_command
        assert node._reanchor_expected is None and port.hardware_write_count == 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
