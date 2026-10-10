#!/usr/bin/env python3
"""Paired software-only benchmark. Mock UART timing is not hardware timing."""
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import statistics
import sys
import time
import rclpy
from rclpy.parameter import Parameter
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter

ROOT=Path('/home/lyapunov/bone_drill')
OUT=ROOT/'results/j14/j14_a1_8_software'


def stats(values):
    q=statistics.quantiles(values,n=100,method='inclusive')
    return dict(P50=statistics.median(values),P95=q[94],P99=q[98],Max=max(values))


class Port:
    write_timeout=.030;out_waiting=0;in_waiting=0;hardware_write_count=0
    def write(self,data):return len(data)


def main():
    name='pus6_motor_driver._a18_before'
    loader=importlib.machinery.SourceFileLoader(name,str(OUT/'motor_driver_node.py.before'))
    spec=importlib.util.spec_from_loader(name,loader);before=importlib.util.module_from_spec(spec)
    sys.modules[name]=before;loader.exec_module(before)
    rclpy.init();nodes=[]
    try:
        for label,cls,flag in [('before',before.Pus6MotorDriverNode,'0'),
                               ('default_after',Pus6MotorDriverNode,'0'),
                               ('isolated_after',Pus6MotorDriverNode,'1')]:
            os.environ['PUS6_J14_DEFER_DRIVER_DIAGNOSTICS']=flag
            node=cls(hardware_mode='mock',bus=MockServoBus(initial_positions=[1000]*6),
                node_name='a18_bench_'+label,servo_scheduler_mode='ros_timer',
                telemetry_publish_mode='async_latest',feedback_loss_policy='age_based',
                parameter_overrides=[Parameter('enable_runtime_diagnostics',value=True)])
            nodes.append((label,node))
            node._stall_recorder.output_dir_override=str(OUT/('benchmark_frozen_'+label))
            node._telemetry_thread_stop_event.set();node._telemetry_ready_event.set();node._telemetry_thread.join(timeout=1)
            node.j7_writer.min_frame_interval_ns=0  # Isolate bookkeeping from Mock busy-wait.
        data={label:{k:[] for k in ('callback_wall_us','callback_thread_cpu_us','callback_process_cpu_us',
                                  'telemetry_publish_wall_us','snapshot_handoff_us','write_wrapper_us')}
              for label,_ in nodes}
        for block in range(3):
            for label,node in (nodes if block%2==0 else list(reversed(nodes))):
                for _ in range(100):
                    t=time.perf_counter_ns();tc=time.thread_time_ns();pc=time.process_time_ns()
                    node._execute_servo_cycle()
                    data[label]['callback_wall_us'].append((time.perf_counter_ns()-t)/1000)
                    data[label]['callback_thread_cpu_us'].append((time.thread_time_ns()-tc)/1000)
                    data[label]['callback_process_cpu_us'].append((time.process_time_ns()-pc)/1000)
                    data[label]['snapshot_handoff_us'].append(node._stall_recorder.ring_buffer[-1].snapshot_handoff_ms*1000)
                snap=node._latest_telemetry_snapshot
                for _ in range(200):
                    t=time.perf_counter_ns();node._publish_telemetry_from_snapshot(snap)
                    data[label]['telemetry_publish_wall_us'].append((time.perf_counter_ns()-t)/1000)
                uart=J7UartAdapter(Port(),min_frame_interval_s=0,tx_queue_diagnostics=True)
                for _ in range(1000):
                    t=time.perf_counter_ns();uart.write(b'0123456789')
                    data[label]['write_wrapper_us'].append((time.perf_counter_ns()-t)/1000)
        result=dict(method='3 alternating blocks; 300 Mock callbacks / 600 direct telemetry calls / 3000 FakeSerial writes per condition',
            units='microseconds',hardware_write_count=0,
            limitations=['Mock inter-frame wait removed; no executor spun; original async workers stopped for direct measurement',
                         'Snapshot handoff includes lock and event, rounded to 1us; not exclusive lock-wait measurement',
                         'RMW native activity/OS noise remain; benchmark does not prove hardware behavior'],
            distributions={label:{k:stats(v) for k,v in values.items()} for label,values in data.items()})
        (OUT/'mock_benchmark_frozen.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for label,node in nodes:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
