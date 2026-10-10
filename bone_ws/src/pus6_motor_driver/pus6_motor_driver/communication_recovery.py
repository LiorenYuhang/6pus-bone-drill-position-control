"""J14-A1.3 transaction recovery; normal UART and controller algorithms are reused."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

from .protocol import CMD_RD_STATUS, pack_read_status_cmd, unpack_status_response
from .safety import MOTOR_IDS, validate_status_feedback


class CommunicationRecoveryError(RuntimeError):
    pass


def append_evidence(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        stream.write(json.dumps(record, separators=(',', ':')) + '\n')


@dataclass
class RecoveryResult:
    feedback_audit: object
    verification_audit: object
    positions: list[int]
    event: dict


class CommunicationRecovery:
    def __init__(self, writer, pause, feedback, *, reanchor, audit_path, recovery_jump_limit, clock_ns=time.monotonic_ns, fresh_state_only=False, static_resume=False, first_resume_target=None):
        self.writer, self.pause, self.feedback = writer, pause, feedback
        self.reanchor = reanchor
        self.audit_path = Path(audit_path)
        self.recovery_jump_limit = recovery_jump_limit
        self.clock_ns = clock_ns
        self.recovery_count = 0
        self.state = 'NORMAL_SERVO'
        self.fresh_state_only = fresh_state_only
        self.static_resume = static_resume
        self.first_resume_target = first_resume_target
        self.static_event = None

    def _fresh_state_hold(self, event, barrier):
        """A1.4 ends at a validated read-only state; no 0x29 or resets."""
        from .j7_servo_feedback_scheduler import FeedbackSample
        from types import SimpleNamespace

        self.writer.begin_recovery_feedback_only()
        event['motion_inhibited'] = True
        self._checkpoint(event, 'RECOVERY_ONLY_COMMUNICATION')
        self._checkpoint(event, 'REFRESH_STATE')
        event['refresh_epoch_ns'] = barrier
        event['fresh_status'] = {}
        event['fresh_feedback_timestamps_ns'] = {}
        event['fresh_acquisition'] = {}
        samples = []
        statuses = {}
        # Retain the existing 1000ms boundary; do not reset old feedback ages.
        last_valid = self.feedback._last_feedback_ns
        if set(last_valid) != set(MOTOR_IDS) or any(t <= 0 for t in last_valid.values()):
            raise CommunicationRecoveryError('missing pre-failure feedback watchdog epoch')
        deadline = min(*last_valid.values(), self.feedback._last_poll_completion_ns) + self.feedback.hard_stop_feedback_age_ns
        event['existing_freshness_deadline_ns'] = deadline
        for mid in MOTOR_IDS:
            event['refresh_motor_id'] = mid
            status, rx_done, acquisition = self.writer.request_recovery_status(mid,
                refresh_epoch_ns=barrier, hard_deadline_ns=deadline,
                observer=lambda r: append_evidence(self.audit_path.parent/'fresh_state_acquisition.jsonl', r))
            event['fresh_acquisition'][mid] = acquisition
            event['fresh_status'][mid] = asdict(status)
            event['fresh_feedback_timestamps_ns'][mid] = rx_done
            if acquisition.get('response_received') is not True or acquisition.get('error'):
                raise CommunicationRecoveryError(f'ID{mid} unresolved feedback transaction')
            if rx_done <= barrier or acquisition['tx_start_ns'] <= barrier:
                raise CommunicationRecoveryError(f'ID{mid} feedback is not fresh after pause')
            validate_status_feedback(status, mid)
            statuses[mid] = status
            samples.append(FeedbackSample(mid, rx_done, status.current_position,
                status.current_mA, status.error_code, None, 0, False))
            self._checkpoint(event, 'REFRESH_STATE')
        self._validate(statuses)
        if self.clock_ns() >= deadline:
            raise CommunicationRecoveryError('existing 1000ms freshness boundary expired')
        positions = [s.position for s in samples]
        if self.static_resume:
            event['fresh_state_complete_ns'] = self.clock_ns()
            return self._prepare_static_resume(event, positions, samples, deadline)
        event.update(fresh_positions=positions, success=True, reanchor_result=False,
                     resumed=False, recovery_duration_ns=self.clock_ns()-event['start_ns'])
        self._checkpoint(event, 'REFRESH_STATE_COMPLETE')
        self._checkpoint(event, 'HOLD')
        print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)
        return RecoveryResult(SimpleNamespace(samples=tuple(samples)), None, positions, event)

    def _prepare_static_resume(self, event, positions, samples, deadline):
        from types import SimpleNamespace
        event.update(fresh_positions=list(positions), reanchor_success=False,
                     resume_success=False, shaper_sync_result=False, success=False)
        self._checkpoint(event, 'REANCHOR_STATE')
        if not self.reanchor(positions, event['start_ns'], deadline):
            raise CommunicationRecoveryError('driver/shaper re-anchor confirmation missing')
        event.update(reanchor_success=True, reanchor_result=True, shaper_sync_result=True,
                     reanchor_ns=self.clock_ns(), reanchor_target=list(positions))
        self._checkpoint(event, '80_STEP_VALIDATION')
        if self.first_resume_target is None:
            raise CommunicationRecoveryError('no production Control/Shaper command source')
        first = self.first_resume_target()
        if first is None or len(first) != len(MOTOR_IDS):
            raise CommunicationRecoveryError('missing first synchronized Control/Shaper command')
        event['first_resume_target'] = list(first)
        self._check_jump(first, positions, event)
        event['first_resume_delta_per_axis'] = list(event['recovery_jump_per_axis'])
        if self.clock_ns() >= deadline:
            raise CommunicationRecoveryError('existing 1000ms freshness boundary expired during re-anchor')
        self.writer.authorize_static_resume(first, fresh_positions=positions,
            first_cycle_id=event['cycle_seq']+1, jump_limit=self.recovery_jump_limit,
            first_deadline_ns=deadline)
        # A new motion epoch invalidates the earlier pause as a stop guarantee.
        # Re-arm before the first write so a failed first/next transaction pauses anew.
        self.pause.arm_after_confirmed_recovery()
        event['first_resume_cycle_seq'] = event['cycle_seq']+1
        self.static_event = event
        self._checkpoint(event, 'STATIC_HOLD_RESUME')
        print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)
        return RecoveryResult(SimpleNamespace(samples=tuple(samples)), None, positions, event)

    def observe_static_cycle(self, audit, feedback_audit):
        """Observe normal production transactions; never send a special batch."""
        event = self.static_event
        if event is None:
            raise CommunicationRecoveryError('static resume was not authorized')
        record = dict(recovery_epoch_ns=event['start_ns'], cycle_seq=audit.cycle_id,
            tx_start_ns=audit.cycle_start_ns, tx_done_ns=audit.cycle_end_ns,
            targets=[r.target_position for r in audit.tx_records],
            tx_per_axis=[dict(motor_id=r.motor_id, tx_start_ns=r.tx_start_ns, tx_end_ns=r.tx_end_ns,
                wall_ns=r.duration_ns, serial_write_metric=asdict(r.serial_write_metric) if r.serial_write_metric else None) for r in audit.tx_records],
            feedback={m:asdict(s) for m,s in audit.feedbacks.items()},
            feedback_samples=[asdict(s) for s in feedback_audit.samples],
            complete_feedback=set(audit.feedbacks)==set(MOTOR_IDS))
        append_evidence(self.audit_path.parent/'communication_static_cycles.jsonl', record)
        first = 'resume_ns' not in event
        if first:
            event['first_resume_transaction'] = asdict(audit)
            self._validate(audit.feedbacks)
            self.writer.confirm_static_resume(audit)
            event.update(resume_success=True, resume_ns=self.clock_ns(),
                resumed=True, active_control_enabled=False,
                first_resumed_feedback={m:asdict(s) for m,s in audit.feedbacks.items()},
                static_cycle_count=0, static_incomplete_feedback_cycles=0)
            self._checkpoint(event, 'RECOVERY_STATIC_HOLD')
            print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)
        event['static_cycle_count'] += 1
        event['static_incomplete_feedback_cycles'] += int(not record['complete_feedback'])
        event['static_hold_duration_ns'] = self.clock_ns()-event['resume_ns']
        event['last_static_cycle_seq'] = audit.cycle_id
        if event['static_hold_duration_ns'] >= 5_000_000_000 and self.state != 'RECOVERY_STABLE':
            event['success'] = event['static_incomplete_feedback_cycles']==0
            self._checkpoint(event, 'RECOVERY_STABLE')
            print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)

    def fail_static_resume(self, failure, phase, transaction=None):
        event = self.static_event
        if event is None:
            return
        failed_audit = getattr(failure, 'cycle_audit', None)
        event.update(success=False, failure_phase=phase,
            failure_reason=f'{type(failure).__name__}: {failure}')
        if failed_audit is not None:
            event['failed_static_transaction'] = asdict(failed_audit)
        elif transaction is not None:
            event['failed_static_transaction'] = asdict(transaction)
        if 'resume_ns' in event:
            event['static_hold_duration_ns'] = self.clock_ns()-event['resume_ns']
        self._checkpoint(event, 'HARD_STOP_LATCH')
        print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)

    def _checkpoint(self, event, state):
        self.state = state
        event['state'] = state
        event['stage_history'].append(dict(state=state, monotonic_ns=self.clock_ns()))
        append_evidence(self.audit_path, event)

    def _validate(self, statuses):
        if set(statuses) != set(MOTOR_IDS):
            raise CommunicationRecoveryError('fresh feedback incomplete: requires 6/6 axes')
        for mid, status in statuses.items():
            validate_status_feedback(status, mid)

    def _check_jump(self, candidate, measured, event):
        jumps = [int(t)-int(p) for t,p in zip(candidate,measured)]
        event['recovery_jump_per_axis'] = jumps
        if len(candidate) != len(MOTOR_IDS) or max(abs(j) for j in jumps) > self.recovery_jump_limit:
            raise CommunicationRecoveryError(
                f'Timeout recovery target rejected: max jump {max((abs(j) for j in jumps), default=0)} steps '
                f'> limit {self.recovery_jump_limit} steps')

    def recover(self, exc, candidate, *, old_command=None):
        audit = exc.cycle_audit
        self.recovery_count += 1
        start = self.clock_ns()
        event = dict(recovery_count=self.recovery_count, cycle_seq=audit.cycle_id,
                     failed_motor_id=audit.error_motor_id, failed_frame_type='0x32 / target 0x29',
                     configured_timeout_s=exc.timeout_s, observed_serial_write_wall_ns=exc.write_metric.wall_duration_ns,
                     observed_wrapper_write_wall_ns=audit.tx_records[-1].duration_ns,
                     delivery_uncertain=True,
                     sent_motor_ids=[r.motor_id for r in audit.tx_records if r.error is None],
                     attempted_motor_ids=[r.motor_id for r in audit.tx_records],
                     unsent_motor_ids=[m for m in MOTOR_IDS if m not in [r.motor_id for r in audit.tx_records]],
                     candidate_at_failure=list(candidate), start_ns=start, success=False,
                     reanchor_result=False, stage_history=[])
        if self.static_resume:
            event.update(old_command=list(old_command) if old_command is not None else None,
                         reanchor_success=False, resume_success=False, resumed=False)
        try:
            self._checkpoint(event, 'COMM_TRANSIENT')
            if not self.writer.communication_transient or self.writer.stopped or self.feedback.stopped:
                raise CommunicationRecoveryError('transport or feedback already hard stopped')
            self._checkpoint(event, 'PAUSE_ALL')
            stop = self.pause.stop_all('communication_transient')
            event['pause'] = asdict(stop)
            if not stop.success or len(stop.records) != 6 or {r.motor_id for r in stop.records} != set(MOTOR_IDS):
                raise CommunicationRecoveryError('pause incomplete: requires 6/6 acknowledged axes')
            self._validate({r.motor_id:r.status for r in stop.records})
            barrier = max(r.tx_end_ns for r in stop.records)
            event['pause_barrier_ns'] = barrier
            if self.fresh_state_only or self.static_resume:
                return self._fresh_state_hold(event, barrier)
            # A read-only status bootstrap is necessary: 0x29 status is emitted
            # by a target write. Never use the old failed target to obtain it.
            self._checkpoint(event, 'REFRESH_STATE')
            baseline = {}; acquisition = {}
            event['bootstrap_acquisition'] = acquisition
            event['bootstrap_status'] = {}
            for mid in MOTOR_IDS:
                request_ns = self.clock_ns()
                raw = self.writer.bus.request(pack_read_status_cmd(mid), expected_id=mid,
                                              expected_command=CMD_RD_STATUS, expected_register=0, retries=0)
                response_ns = self.clock_ns()
                baseline[mid] = unpack_status_response(raw)
                event['bootstrap_status'][mid] = asdict(baseline[mid])
                acquisition[mid] = dict(request_ns=request_ns, response_ns=response_ns, response_hex=raw.hex())
                if request_ns <= barrier:
                    raise CommunicationRecoveryError('state acquisition is not after complete pause')
            event['bootstrap_acquisition'] = acquisition
            event['bootstrap_status'] = {m:asdict(s) for m,s in baseline.items()}
            self._validate(baseline)
            positions = [baseline[m].current_position for m in MOTOR_IDS]
            self._check_jump(candidate, positions, event)
            self._checkpoint(event, 'VERIFY_FRESH_0x29_HOLD')
            hold = self.writer.verify_recovery_hold(positions, cycle_id=audit.cycle_id+1)
            event['verification_cycle_seq'] = hold.cycle_id
            event['verification_targets'] = positions
            event['verification_start_ns'] = hold.cycle_start_ns
            event['fresh_feedback_capture_end_ns'] = hold.cycle_end_ns
            event['fresh_status'] = {m:asdict(s) for m,s in hold.feedbacks.items()}
            if hold.cycle_start_ns <= barrier:
                raise CommunicationRecoveryError('0x29 verification predates pause')
            self._validate(hold.feedbacks)
            if any(hold.feedbacks[mid].target_position != target for mid,target in zip(MOTOR_IDS,positions)):
                raise CommunicationRecoveryError('fresh 0x29 response does not match measured hold target')
            positions = [hold.feedbacks[m].current_position for m in MOTOR_IDS]
            event['fresh_positions'] = positions
            self._check_jump(candidate, positions, event)
            # Reuse the existing fault/current/1000ms watchdog enforcement.
            feedback_audit = self.feedback.ingest_cycle_feedback(
                hold.feedbacks, servo_cycle_id=hold.cycle_id, servo_cycle_timestamp_ns=hold.cycle_start_ns)
            if self.writer.stopped or self.feedback.stopped:
                raise CommunicationRecoveryError('existing hardware/feedback watchdog stopped recovery')
            event['fresh_feedback_timestamps_ns'] = {s.motor_id:s.timestamp_ns for s in feedback_audit.samples}
            self._checkpoint(event, 'REANCHOR')
            # The existing hard-stop interval also bounds waiting for upstream
            # reset confirmation. This is not a new performance safety gate.
            deadline = min(event['fresh_feedback_timestamps_ns'].values()) + self.feedback.hard_stop_feedback_age_ns
            if not self.reanchor(positions, start, deadline):
                raise CommunicationRecoveryError('command/shaper re-anchor not confirmed')
            event['reanchor_result'] = True
            if self.clock_ns() > deadline:
                raise CommunicationRecoveryError('existing 1000ms freshness boundary expired during re-anchor')
            self.writer.complete_communication_recovery(hold)
            self.pause.arm_after_confirmed_recovery()
            event.update(resume_ns=self.clock_ns(), recovery_duration_ns=self.clock_ns()-start, success=True)
            self._checkpoint(event, 'RESUME_SERVO')
            self.state = 'NORMAL_SERVO'
            print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)
            return RecoveryResult(feedback_audit, hold, positions, event)
        except Exception as failure:
            if getattr(failure, 'cycle_audit', None) is not None:
                event['verification_failure_motor_id'] = failure.cycle_audit.error_motor_id
                event['verification_failure_phase'] = failure.cycle_audit.error_phase
            event.update(failure_reason=f'{type(failure).__name__}: {failure}',
                         failure_phase=self.state,
                         recovery_duration_ns=self.clock_ns()-start, success=False)
            # Pause has already been requested once; the existing stop hook is
            # idempotent. No failed motion frame or partial batch is retried.
            self.writer.stop('communication_recovery_failed')
            self._checkpoint(event, 'HARD_STOP_LATCH')
            print('J14_COMM_RECOVERY_EVENT '+json.dumps(event), flush=True)
            raise CommunicationRecoveryError(event['failure_reason']) from failure
