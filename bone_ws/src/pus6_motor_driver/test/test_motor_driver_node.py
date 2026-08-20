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
