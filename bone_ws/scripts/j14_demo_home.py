"""Demo startup joint trajectory and normal-stop Cartesian HOME adapter.

Never owns a publisher or UART. Uses the calling stage's sole command publisher and
imports the existing HOME predicate, timeout and sample-count default.
"""
from __future__ import annotations
import inspect
from pathlib import Path
import time

from j14_demo_controller import write_json
from j10_a1_single_candidate_workspace_measurement import (
    WorkspaceMeasurementHarness, _home_feedback_in_band, _home_timeout_from_feedback,
    run_workspace_measurement_experiment,
)
from pus6_motor_driver.safety import (
    HOME_MEASUREMENT_BAND_LOWER, HOME_MEASUREMENT_BAND_UPPER,
    OPERATING_HOME_RAW_STEPS, MAX_CURRENT_MA, FEEDBACK_LOWER_LIMIT, FEEDBACK_UPPER_LIMIT,
)
from pus6_motor_driver.j7_servo_feedback_scheduler import DEFAULT_HARD_STOP_FEEDBACK_AGE_S
from pus6_control.joint_trajectory_controller import compute_synchronized_step_point

HOME_SAMPLES=inspect.signature(run_workspace_measurement_experiment).parameters['home_consecutive_samples'].default


def attach_feedback(node):
    from rclpy.qos import qos_profile_sensor_data
    from pus6_msgs.msg import TimedJointFeedback
    state=dict(count=0,steps=[],rx_ns=[],faults=[],currents=[],last_rx_ns=0)
    def receive(msg):
        state.update(count=state['count']+1,steps=[int(x) for x in msg.raw_steps],
                     rx_ns=[int(x) for x in msg.per_axis_rx_done_monotonic_ns],
                     faults=[int(x) for x in msg.fault_codes],
                     currents=[int(x) for x in msg.currents_ma],last_rx_ns=time.monotonic_ns())
    node.create_subscription(TimedJointFeedback,'/pus6/joint_feedback/timed',receive,qos_profile_sensor_data)
    return state


def feedback_failure(node,feedback):
    if getattr(node,'driver_stop_latched',False):
        return 'Driver STOP: '+getattr(node,'driver_fault_msg','')
    if any(len(feedback.get(k,[]))!=6 for k in ('steps','rx_ns','faults','currents')):
        return 'missing six-axis feedback'
    if any(feedback['faults']):
        return 'hardware fault: '+str(feedback['faults'])
    if any(current>MAX_CURRENT_MA for current in feedback['currents']):
        return 'overcurrent: '+str(feedback['currents'])
    if any(not FEEDBACK_LOWER_LIMIT<=s<=FEEDBACK_UPPER_LIMIT for s in feedback['steps']):
        return 'position outside allowed feedback range'
    now=time.monotonic_ns()
    if any(t<=0 or t>now for t in feedback['rx_ns']):
        return 'invalid feedback timestamp'
    if now-min(feedback['rx_ns'])>DEFAULT_HARD_STOP_FEEDBACK_AGE_S*1e9:
        return 'feedback hard timeout'
    return None


def recover_home(node,feedback,out,stop,request_reader,*,phase="shutdown",allow_already_home=False):
    import rclpy
    start=time.monotonic_ns()
    path=out/("startup_home.json" if phase=="startup" else "home_recovery.json")
    already=allow_already_home and _home_feedback_in_band(feedback.get("steps", []))
    result=dict(state='HOME_RECOVERY',phase=phase,already_home=already,motion_start_ns=None,confirmation_samples=[],target=list(OPERATING_HOME_RAW_STEPS),
                initial_feedback=list(feedback.get('steps', [])),start_ns=start,
                tolerance=[HOME_MEASUREMENT_BAND_LOWER,HOME_MEASUREMENT_BAND_UPPER],
                required_consecutive_samples=HOME_SAMPLES,home_target_publish_count=0,
                source='J10 WorkspaceMeasurementHarness / pus6_control.compute_synchronized_step_point')
    timeout=_home_timeout_from_feedback(feedback.get('steps', []),OPERATING_HOME_RAW_STEPS)
    result['timeout_s']=timeout
    write_json(path,result)

    start_steps=[max(0,min(2000,int(round(s)))) for s in feedback.get('steps',[])] if feedback.get('steps') else list(OPERATING_HOME_RAW_STEPS)
    target_steps=list(OPERATING_HOME_RAW_STEPS)
    d_max=max((abs(t-s) for s,t in zip(start_steps,target_steps)),default=0)
    traj_duration=max(2.0,(d_max*1.875)/200.0) if d_max>0 else 0.5
    motion_t0=None

    deadline=time.monotonic()+timeout
    last_count=feedback['count'];consecutive=0
    last_axes=list(feedback['rx_ns'])
    next_publish=time.monotonic()
    while time.monotonic()<deadline:
        reason=feedback_failure(node,feedback)
        try:
            request=request_reader()
        except Exception as exc:
            request={};reason=reason or repr(exc)
        if stop.is_set() or request.get('action')=='safety_pause':
            reason=reason or 'safety stop requested during HOME'
        if reason:
            result.update(state='SKIPPED' if phase=='startup' and not result['home_target_publish_count'] else 'HOME_ABORTED',reason=reason)
            break
        if already and not _home_feedback_in_band(feedback['steps']):
            already=False;result['already_home']=False;consecutive=0;result['confirmation_samples']=[]
            start_steps=[max(0,min(2000,int(round(s)))) for s in feedback['steps']]
            d_max=max(abs(t-s) for s,t in zip(start_steps,target_steps))
            traj_duration=max(2.0,(d_max*1.875)/200.0) if d_max>0 else 0.5
            motion_t0=None
            result['motion_start_ns']=None
        if not already and time.monotonic()>=next_publish:
            now=time.monotonic()
            if result['motion_start_ns'] is None:
                result['motion_start_ns']=time.monotonic_ns()
                motion_t0=now
            if phase=="startup":
                elapsed=now-(motion_t0 or now)
                pt=compute_synchronized_step_point(start_steps,target_steps,elapsed,traj_duration)
                clamped_pt=[max(0,min(2000,int(round(s)))) for s in pt]
                if hasattr(node,'cmd_steps_pub') and node.cmd_steps_pub is not None:
                    from std_msgs.msg import Int32MultiArray
                    node.cmd_steps_pub.publish(Int32MultiArray(data=clamped_pt))
                elif hasattr(node,'target_pose_pub') and node.target_pose_pub is not None:
                    from std_msgs.msg import Int32MultiArray
                    try:
                        node.target_pose_pub.publish(Int32MultiArray(data=clamped_pt))
                    except TypeError:
                        WorkspaceMeasurementHarness.publish_target_pose(node,0.,0.,getattr(node,'z0',.132612268),0.,0.,0.)
            else:
                WorkspaceMeasurementHarness.publish_target_pose(node,0.,0.,node.z0,0.,0.,0.)
            result['home_target_publish_count']+=1
            next_publish=now+.02
        rclpy.spin_once(node,timeout_sec=.002)
        reason=feedback_failure(node,feedback)
        if reason:
            result.update(state='HOME_ABORTED',reason=reason)
            break
        if feedback['count']!=last_count:
            last_count=feedback['count']
            fresh_axes=len(feedback['rx_ns'])==6 and all(t>max(result["motion_start_ns"] or start,prior) for t,prior in zip(feedback['rx_ns'],last_axes))
            last_axes=list(feedback['rx_ns'])
            if fresh_axes and _home_feedback_in_band(feedback['steps']):
                consecutive+=1
                result['confirmation_samples'].append(dict(count=feedback['count'],rx_ns=list(feedback['rx_ns']),steps=list(feedback['steps'])))
            else:
                consecutive=0;result['confirmation_samples']=[]
            if consecutive>=HOME_SAMPLES:
                result['state']='HOME_CONFIRMED'
                break
    else:
        result.update(state='HOME_TIMEOUT',reason='STARTUP HOME TIMEOUT' if phase=='startup' else 'HOME RECOVERY TIMEOUT')
    result.update(end_ns=time.monotonic_ns(),feedback=list(feedback['steps']),
                  per_axis_rx_done_ns=list(feedback['rx_ns']),consecutive_samples=consecutive)
    write_json(path,result)
    return result
