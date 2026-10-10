"""A1.8 isolates one performance channel; all physical I/O is FakeSerial/Mock."""
import json
import threading
from pathlib import Path
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import pack_write_register_cmd
from pus6_motor_driver.j7_real_uart_adapter import J7UartAdapter,J7StopAdapter
from pus6_motor_driver.j7_six_axis_servo_transport import J7SixAxisServoTransport
from pus6_motor_driver.j7_servo_feedback_scheduler import J7ServoFeedbackScheduler
from test_j14_a1_1_timeout_semantics import QueueSerial


class Sink:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)


@pytest.fixture
def make_node(monkeypatch,tmp_path):
    nodes=[];context=Context();rclpy.init(context=context)
    def make(defer=True):
        monkeypatch.setenv('PUS6_J14_DEFER_DRIVER_DIAGNOSTICS','1' if defer else '0')
        node=Pus6MotorDriverNode(hardware_mode='mock',bus=MockServoBus(initial_positions=[1000]*6),
            context=context,servo_scheduler_mode='ros_timer',telemetry_publish_mode='async_latest',
            feedback_loss_policy='age_based',parameter_overrides=[Parameter('enable_runtime_diagnostics',value=True)],
            node_name='a18_test_'+str(len(nodes)))
        node._stall_recorder.output_dir_override=str(tmp_path/str(len(nodes)))
        node._communication_audit_dir=lambda:tmp_path/str(nodes.index(node))
        # The ROS timer is not spun. Stop the existing publisher thread for
        # deterministic unit calls; no new worker or serial port is introduced.
        node._telemetry_thread_stop_event.set();node._telemetry_ready_event.set()
        node._telemetry_thread.join(timeout=1)
        node.mock_bus.written_frames.clear()  # Exclude startup scan/configuration.
        for name in ('joint_state_pub','timed_feedback_pub','timed_command_pub','driver_diagnostics_pub',
                     'raw_steps_pub','raw_currents_pub','raw_faults_pub'):
            setattr(node,name,Sink())
        nodes.append(node)
        return node
    yield make
    for node in nodes:node.destroy_node()
    context.shutdown()


@pytest.mark.parametrize('defer',[False,True])
def test_frame_order_feedback_50hz_and_other_topics_unchanged(make_node,defer,tmp_path):
    node=make_node(defer)
    node._execute_servo_cycle()
    snap=node._latest_telemetry_snapshot
    node._publish_telemetry_from_snapshot(snap)
    assert node.mock_bus.written_frames==[pack_write_register_cmd(i,0x29,[1000]) for i in range(1,7)]
    assert node.j7_writer.period_ns==20_000_000 and node.write_rate_hz==50
    assert all(node._latest_rx_done_ns[i]>0 for i in range(1,7))
    assert len(node.joint_state_pub.messages)==len(node.timed_feedback_pub.messages)==1
    assert len(node.timed_command_pub.messages)==len(node.raw_faults_pub.messages)==1
    assert list(node.raw_faults_pub.messages[0].data)==[0]*6
    assert len(node.driver_diagnostics_pub.messages)==(0 if defer else 1)
    assert isinstance(snap.diag_data,tuple if defer else list) and len(snap.diag_data)==11
    assert node.mock_bus is node.j7_adapter and not hasattr(node.mock_bus,'serial')


def test_blocked_diagnostic_sink_cannot_block_other_topics_or_uart(make_node):
    node=make_node();node._execute_servo_cycle();snap=node._latest_telemetry_snapshot
    class Blocked:
        def publish(self,_):pytest.fail('Selected diagnostic workload ran during UART streaming')
    node.driver_diagnostics_pub=Blocked()
    snap.fault_codes=[0,0,4,0,0,0]
    node._publish_telemetry_from_snapshot(snap)
    assert list(node.raw_faults_pub.messages[-1].data)==snap.fault_codes
    assert list(node.timed_feedback_pub.messages[-1].fault_codes)==snap.fault_codes
    node._execute_servo_cycle()
    assert len(node.mock_bus.written_frames)==12


def test_actual_feedback_fault_still_latches_stop(make_node):
    node=make_node();node.mock_bus.fault_on_axis=3
    node._execute_servo_cycle()
    assert node.driver_fault_latched and node.j7_writer.stopped


def test_deferred_dump_after_worker_exit_keeps_same_values(make_node,tmp_path):
    node=make_node();node._execute_servo_cycle();snap=node._latest_telemetry_snapshot
    assert not (tmp_path/'0'/'driver_diagnostics_deferred.jsonl').exists()
    node.destroy_node()
    row=json.loads((tmp_path/'0'/'driver_diagnostics_deferred.jsonl').read_text())
    assert row['cycle_seq']==snap.uart_cycle_seq and row['retention']=='latest_only'
    assert row['data']==[float(x) for x in snap.diag_data]
    # Avoid a second destroy from the fixture after this shutdown-path check.
    node.destroy_node=lambda:None


@pytest.mark.parametrize('failed_id',[1,3,6])
def test_timeout_abort_pause_fresh_and_failure_telemetry_survive(make_node,monkeypatch,tmp_path,failed_id):
    monkeypatch.setenv('PUS6_J14_RECOVERY_STAGE','fresh_state_only')
    node=make_node();port=QueueSerial();uart=J7UartAdapter(port,min_frame_interval_s=0,tx_queue_diagnostics=True,
        read_observer=node._stall_recorder.record_single_read)
    pause=J7StopAdapter(uart,audit_observer=node._persist_pause_audit)
    writer=J7SixAxisServoTransport(uart,min_frame_interval_s=0,pause_callback=pause,recoverable_write_timeouts=True)
    writer.configure_servo(speed=50)
    feedback=J7ServoFeedbackScheduler(writer,feedback_loss_policy='age_based',max_feedback_age_s=1,
        warning_feedback_age_s=.2833,hard_stop_feedback_age_s=1)
    feedback.full_scan(servo_cycle_id=0,servo_cycle_timestamp_ns=0)
    node.j7_adapter=uart;node.j7_writer=writer;node.j7_feedback=feedback;node.j7_stop_adapter=pause
    port.timeout_motor=failed_id
    node._execute_servo_cycle()
    assert node._fresh_state_hold and not node._static_resume
    assert pause.audit_snapshot().success and len(pause.audit_snapshot().records)==6
    node._stall_recorder.dump()
    row=json.loads((tmp_path/'0'/'tx_write_events.jsonl').read_text())
    assert row['motor_id']==failed_id and row['cycle_seq']==0 and row['write_error']=='SerialTimeoutException'
    assert row['written_bytes']==-1 and row['requested_bytes']==10
    assert row['process_cpu_duration_ns']>=0 and row['voluntary_ctx_delta']>=0
    assert row['native_thread_id']==threading.get_native_id()
    assert len(writer.audit_snapshot().cycles[0].tx_records)==failed_id
    assert port.hardware_write_count==0
