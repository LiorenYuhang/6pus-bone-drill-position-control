"""Recovery handshake resets state using the existing shaper reset API."""
import pytest
import rclpy
from std_msgs.msg import Float64MultiArray, Int64MultiArray
from pus6_control.control_node import Pus6ControlNode


@pytest.fixture
def node():
    if not rclpy.ok():rclpy.init()
    n=Pus6ControlNode()
    try:yield n
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def test_anchor_discards_old_goal_and_shaper_velocity_acceleration(node):
    node._target_positions=[.01]*6
    node.controller.shaper.dq=[.001]*6
    node.controller.shaper.ddq=[.002]*6
    node.controller.shaper.dddq=[.003]*6
    msg=Int64MultiArray(data=[123,*[1008]*6])
    node._handle_communication_reanchor(msg)
    expected=[8/node.controller.steps_per_meter]*6
    assert node._target_positions==expected and node._current_positions==expected
    assert node.controller.shaper.q==expected
    assert node.controller.shaper.dq==node.controller.shaper.ddq==node.controller.shaper.dddq==[0.0]*6
    assert node.controller.meters_to_raw_steps(node.controller.shaper.q,node.operating_home_steps)==[1008]*6


def test_stale_or_conflicting_anchor_cannot_reset_new_state(node):
    node._handle_communication_reanchor(Int64MultiArray(data=[123,*[1008]*6]))
    desired=Float64MultiArray(data=[.002]*6)
    node._handle_desired_deltas(desired)
    for payload in [[122,*[1000]*6],[123,*[1000]*6],[0,*[1000]*6],[124,*[2001]*6]]:
        node._handle_communication_reanchor(Int64MultiArray(data=payload))
        assert node._target_positions==[.002]*6


def test_repeat_of_same_anchor_is_idempotent(node):
    msg=Int64MultiArray(data=[123,*[1008]*6])
    node._handle_communication_reanchor(msg)
    node._target_positions=[.002]*6
    node._handle_communication_reanchor(msg)
    assert node._target_positions==[.002]*6
