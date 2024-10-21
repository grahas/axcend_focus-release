
from axcend_focus_test_utils_package.conftest import nodes, mock_serial_port
from axcend_focus_custom_interfaces.msg import CartridgeOvenControl
from axcend_focus_custom_interfaces.action import (
    Method,
)

def test_method(nodes):
    """Verify that the firmware is able to handle the method action."""
    # Unpack the test objects from the fixture
    test_node = nodes["test_node"]

    # Create a goal for the method action
    goal_msg = Method.Goal()
    goal_msg.gradient_x = [int(x * 60) for x in [0, 4, 5, 5.2, 9]]
    goal_msg.gradient_y = [5.0, 95.0, 95.0, 5.0, 5.55]
    goal_msg.control_mode = Method.Goal.FLOW_MODE
    goal_msg.flow_rate = 1.25
    goal_msg.operating_pressure = 1500
    goal_msg.oven_control.oven_state = CartridgeOvenControl.OVEN_ON
    goal_msg.oven_control.oven_set_point = 45.0
    goal_msg.injection_type = Method.Goal.TIMED_INJECTION
    goal_msg.injection_volume = 100
    goal_msg.equilibration_time = 180

    # Send the goal_msg
    test_node.method_action_client.wait_for_server()
    result = test_node.method_action_client.send_goal(goal_msg)

    assert result.success is True
