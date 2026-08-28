import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from std_srvs.srv import Trigger
from pus6_motor_driver.motor_driver_node import Pus6MotorDriverNode
from pus6_motor_driver.fake_bus import MockServoBus
from pus6_motor_driver.protocol import parse_frame
from pus6_motor_driver.safety import HardwareMode
from pus6_motor_driver.six_motor_controller import SixMotorController
from test_six_motor_controller import FakeBus


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_motor_driver_node_mock_lifecycle(rclpy_init):
    # Initialize node in default mock mode
    node = Pus6MotorDriverNode()
    assert node.get_name() == "pus6_motor_driver_node"
    assert node.hardware_mode == HardwareMode.MOCK
    assert len(node.joint_names) == 6
    assert node.joint_names == ["motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"]

    # Trigger pause service
    req = Trigger.Request()
    resp = Trigger.Response()
    resp = node._handle_pause_all(req, resp)
    assert isinstance(resp.success, bool)

    node.destroy_node()


def test_motor_driver_node_read_only_mode_zero_writes(rclpy_init):
    fake_bus = FakeBus()
    node = Pus6MotorDriverNode()
    # Inject SixMotorController in READ_ONLY mode
    node.hardware_mode = HardwareMode.READ_ONLY
    node.controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)
    node.controller.connect()

    # 1. Test polling: publishes SI positions in meters, empty effort and velocity
    node._poll_and_publish()
    assert node.controller.write_count == 0

    # 2. Test configure service rejection
    req = Trigger.Request()
    resp = Trigger.Response()
    resp = node._handle_configure_motors(req, resp)
    assert resp.success is False
    assert "READ_ONLY" in resp.message
    assert node.controller.write_count == 0

    # 3. Test pause service rejection
    resp = node._handle_pause_all(req, resp)
    assert resp.success is False
    assert "READ_ONLY" in resp.message
    assert node.controller.write_count == 0

    # 4. Test command topics rejection
    cmd_msg = Float64MultiArray()
    cmd_msg.data = [0.001, 0.002, 0.003, 0.004, 0.005, 0.006]
    node._handle_command_positions_meters(cmd_msg)
    assert node.controller.write_count == 0

    step_msg = Int32MultiArray()
    step_msg.data = [50, 50, 50, 50, 50, 50]
    node._handle_command_joint_steps(step_msg)
    assert node.controller.write_count == 0

    single_msg = Int32MultiArray()
    single_msg.data = [1, 50]
    node._handle_command_single_step(single_msg)
    assert node.controller.write_count == 0

    # 5. Test node destruction in read_only mode has zero pause writes
    node.destroy_node()
    assert node.controller.write_count == 0
    assert len([e for e in fake_bus.events if e[1] == 0x32]) == 0


def test_motor_driver_node_single_axis_bringup_active_mode(rclpy_init):
    fake_bus = FakeBus()
    node = Pus6MotorDriverNode()
    # Inject SixMotorController in ACTIVE mode with fake bus
    node.hardware_mode = HardwareMode.ACTIVE
    node.controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.ACTIVE)
    node.controller.connect()

    # Command single axis ID3 -> step 40
    msg = Int32MultiArray()
    msg.data = [3, 40]
    node._handle_command_single_step(msg)

    # Check that exactly 1 write was issued to ID3
    writes = [e for e in fake_bus.events if e[1] == 0x32]
    assert len(writes) == 1
    assert writes[0][0] == 3  # ID3
    assert writes[0][3] == [40]
    node.destroy_node()


def test_motor_driver_node_position_only_read_mode(rclpy_init):
    fake_bus = FakeBus()
    node = Pus6MotorDriverNode()
    node.hardware_mode = HardwareMode.READ_ONLY
    node.read_mode = "position_only"
    node.controller = SixMotorController(bus=fake_bus, hardware_mode=HardwareMode.READ_ONLY)
    node.controller.connect()

    # Poll once in position_only mode
    node._poll_and_publish()
    assert node.controller.write_count == 0
    assert hasattr(node, "_last_positions_snapshot")
    assert node._last_positions_snapshot is not None

    # Check 0x31 read events
    read_events = [e for e in fake_bus.events if e[1] == 0x31]
    assert len(read_events) == 6

    node.destroy_node()
    assert node.controller.write_count == 0


def _j8_motion_frame_count(bus):
    return sum(1 for frame in bus.written_frames if parse_frame(frame).register_addr == 0x29)

def test_j8_stopped_worker_fails_fast_without_traceback_flood(rclpy_init):
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(hardware_mode="mock", write_rate_hz=50.0, use_j7_servo=True, bus=bus)
    node.j7_writer.stop("test_stop")
    count_after_stop = _j8_motion_frame_count(bus)
    node._uart_worker_cycle()
    node._uart_worker_cycle()
    assert node.driver_fault_latched is True
    assert node.driver_fault_reason == "test_stop"
    assert node._j7_fault_log_emitted is True
    assert _j8_motion_frame_count(bus) == count_after_stop
    node.destroy_node()


def test_j8_isolated_feedback_timeout_recovers_without_driver_latch(rclpy_init):
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(hardware_mode="mock", write_rate_hz=50.0, use_j7_servo=True, bus=bus)
    bus.timeout_on_axis = 1
    node._handle_command_joint_steps(Int32MultiArray(data=[1000] * 6))
    node._uart_worker_cycle()
    assert node.driver_fault_latched is False
    assert node.telemetry_health_ok is True
    bus.timeout_on_axis = None
    for _ in range(6):
        node._uart_worker_cycle()
    assert node.driver_fault_latched is False
    assert node.j7_writer.stopped is False
    node.destroy_node()

def test_j8_persistent_feedback_timeout_latches_and_stops_worker(rclpy_init):
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(hardware_mode="mock", write_rate_hz=50.0, use_j7_servo=True, bus=bus)
    bus.timeout_on_axis = 1
    node._handle_command_joint_steps(Int32MultiArray(data=[1000] * 6))
    node._uart_worker_cycle()
    for _ in range(10):
        node._uart_worker_cycle()
    node._uart_worker_cycle()
    assert node.driver_fault_latched is True
    assert node.driver_fault_reason == "persistent_feedback_loss_ID1"
    count_after_stop = _j8_motion_frame_count(bus)
    node._uart_worker_cycle()
    assert _j8_motion_frame_count(bus) == count_after_stop
    node.destroy_node()

def test_j8_hard_feedback_fault_latches_immediately(rclpy_init):
    bus = MockServoBus(initial_positions=[1000] * 6)
    node = Pus6MotorDriverNode(hardware_mode="mock", write_rate_hz=50.0, use_j7_servo=True, bus=bus)
    bus.faults[1] = 7
    node._handle_command_joint_steps(Int32MultiArray(data=[1000] * 6))
    node._uart_worker_cycle()
    assert node.driver_fault_latched is True
    assert node.driver_fault_reason == "motor_fault_ID1"
    count_after_stop = _j8_motion_frame_count(bus)
    node._uart_worker_cycle()
    assert _j8_motion_frame_count(bus) == count_after_stop
    node.destroy_node()


def test_j8_demo_aborts_on_unsafe_driver_health():
    import ast
    demo_path = PKG_ROOT.parent.parent / "scripts" / "run_j8_final_video_demonstration.py"
    tree = ast.parse(demo_path.read_text())
    selected = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in {"DemoSafetyAbort", "demo_health_failure_reason", "ensure_demo_health"}]
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(demo_path), "exec"), namespace)

    class UnsafeDriver:
        driver_fault_latched = True
        driver_fault_reason = "persistent_feedback_loss_ID1"
        telemetry_health_ok = False

    with pytest.raises(namespace["DemoSafetyAbort"], match="persistent_feedback_loss_ID1"):
        namespace["ensure_demo_health"](UnsafeDriver())
