#!/usr/bin/env python3
"""Demo-owned lifecycle prefix for unchanged production ROS node classes.

The original launch still supplies executable, remaps and parameter files. This
host owns signals/context only, rather than invoking legacy console main loops.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from j14_demo_controller import write_json


def create_node(role):
    if role == 'motor_driver_node':
        from pus6_motor_driver.production_driver import create_driver, read_configuration
        injection = os.environ.get('J14_DEMO_MOCK_INJECTION_DIR')
        if injection:
            if read_configuration()['hardware_mode'] != 'mock':
                raise RuntimeError('Demo fault injection requires Mock hardware mode')
            original = subprocess.Popen
            def test_child(argv, *args, **kwargs):
                if isinstance(argv,list) and len(argv)>1 and argv[1].endswith('j14_uart_child_main.py'):
                    argv=[argv[0],str(Path(__file__).with_name('j14_demo_f2_mock_child.py' if os.environ.get('J14_DEMO_F2_SCENARIO') else 'j14_demo_f1_mock_child.py')),*argv[2:]]
                return original(argv,*args,**kwargs)
            # Test-local substitution, restored immediately after construction.
            subprocess.Popen=test_child
            try:
                return create_driver()
            finally:
                subprocess.Popen=original
        return create_driver()
    if role == 'kinematics_node':
        from pus6_kinematics.kinematics_node import Pus6KinematicsNode
        return Pus6KinematicsNode()
    if role == 'control_node':
        from pus6_control.control_node import Pus6ControlNode
        return Pus6ControlNode()
    if role == 'fk_node':
        from pus6_kinematics.fk_node import Pus6FkNode
        return Pus6FkNode()
    raise ValueError(f'Unsupported lifecycle role: {role}')


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    role=Path(sys.argv[1]).name
    out=Path(os.environ['J14_DEMO_LOG_DIR'])
    status=out/f'{role}_host.json'
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.set())
    record=dict(role=role,pid=os.getpid(),ppid=os.getppid(),pgid=os.getpgrp(),
                state='starting',args=sys.argv[1:],start_ns=time.monotonic_ns())
    write_json(status,record)
    node=executor=None
    code=0
    rclpy.init(args=sys.argv[2:],signal_handler_options=SignalHandlerOptions.NO)
    try:
        node=create_node(role)
        if role=='control_node':
            # Demo-only startup lifecycle: no command before feedback admission.
            # Seed the existing shaper through its public reset API; no tuning or
            # replacement of the production control loop/trajectory algorithm.
            node.timer.cancel()
        # This is the Q3J production executor selection, not a replacement driver.
        executor=SingleThreadedExecutor()
        executor.add_node(node)
        record.update(state='ready',node_name=node.get_fully_qualified_name(),
                      executor='SingleThreadedExecutor')
        write_json(status,record)
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=.02)
            if role=='control_node' and node.timer.is_canceled():
                admission=out/'startup_command_admission.json'
                if admission.exists():
                    record_in=json.loads(admission.read_text())
                    from j14_demo_home import feedback_failure
                    reason=feedback_failure(node,record_in['feedback'])
                    if reason:
                        raise RuntimeError('Demo shaper initialization rejected: '+reason)
                    measured=[(float(p)-float(h))/node.controller.steps_per_meter
                              for p,h in zip(record_in['feedback']['steps'],node.operating_home_steps)]
                    node.controller.reset(initial_positions=measured)
                    node._current_positions=list(measured)
                    node._target_positions=None  # Wait for the sole Cartesian publisher.
                    node._last_time=node.get_clock().now()
                    node.timer.reset()
                    write_json(out/'startup_shaper_ready.json',dict(
                        reset_api='JointTrajectoryController.reset',
                        initial_feedback=record_in['feedback'],monotonic_ns=time.monotonic_ns(),
                        command_output_before_admission=False,target_waits_for_cartesian=True))
    except Exception as exc:
        code=1;record['error']=repr(exc)
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()  # Parent closes Child: original six-axis pause and joins.
            if executor is not None:
                executor.shutdown(timeout_sec=1)
        except Exception as exc:
            code=1;record['cleanup_error']=repr(exc)
            raise
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            record.update(state='exited',returncode=code,end_ns=time.monotonic_ns())
            write_json(status,record)
    return code


if __name__=='__main__':
    sys.exit(main())
