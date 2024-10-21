"""Conftest file for the axcend_focus_test_utils_package package."""

import os
from collections import deque
from threading import Thread

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Header

from axcend_focus_custom_interfaces.msg import PumpStatus, CartridgeOvenStatus
from axcend_focus_custom_interfaces.srv import CartridgeMemoryReadWrite
from axcend_focus_custom_interfaces.action import Method
from axcend_focus_ros2_firmware_bridge.firmware_bridge import (
    DataAcquisitionState,
    FirmwareNode,
)
from axcend_focus_test_utils_package.mock_serial_port import mock_serial_port

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the relative path to the JSON file
relative_path = "dummy_system_parameter_file.json"

# Join the current directory with the relative path
json_file_path = os.path.join(current_dir, relative_path)

# Set the SYS_PARAMS_FILE environment variable
os.environ["SYS_PARAMS_FILE"] = json_file_path

# Set the ENVIRONMENT environment variable
os.environ["ENVIRONMENT"] = "development"


class ExampleNode(Node):
    """Node is for use of the firmware test script."""

    def __init__(self):
        super().__init__("test_firmware_node")
        self.subscription = self.create_subscription(
            Temperature, "cartridge_temperature", self.listener_callback, 10
        )
        self.msg_data = None
        self.cartridge_temperature = deque(maxlen=10)

        # Create a client for the cartridge_memory_read_write service
        self.cartridge_memory_read_write_client = self.create_client(
            CartridgeMemoryReadWrite, "cartridge_memory_read_write"
        )

        # Create action client for the method action
        self.method_action_client = ActionClient(self, Method, 'method_action')

        # Create a publisher to write to the firmware_UART_write string topic
        self.firmware_UART_write_publisher = self.create_publisher(
            String, "firmware_UART_write", 10
        )

        # Add a pump status subscriber
        self.pump_status_subscription = self.create_subscription(
            PumpStatus, "pump_status", self.pump_status_callback, 10
        )
        self.pump_status_cache = PumpStatus()

        # Create a subscriber for cartridge oven data
        self.cartridge_oven_status_subscription = self.create_subscription(
            CartridgeOvenStatus,
            "cartridge_oven_status",
            self.cartridge_oven_status_callback,
            10,
        )
        # Cache for the cartridge oven data
        self.cartridge_oven_status_cache = CartridgeOvenStatus()
        self.cartridge_oven_status_cache.header = Header()

    def listener_callback(self, msg):
        """Callback function for the cartridge temperature subscriber."""
        self.cartridge_temperature.append(msg.temperature)

    def request_cartridge_memory(self):
        """Send a request to the cartridge_memory_read_write service."""
        request = CartridgeMemoryReadWrite.Request()
        request.command = "read"
        while not self.cartridge_memory_read_write_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Service not available, waiting again...")

        future = self.cartridge_memory_read_write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed!")
            return None

    def pump_status_callback(self, msg):
        """Callback function for the pump status subscriber."""
        self.pump_status_cache = msg

    def cartridge_oven_status_callback(self, msg):
        """Callback function for the cartridge oven status subscriber."""
        self.cartridge_oven_status_cache = msg


@pytest.fixture
def nodes():
    """Create the firmware node and the test node."""
    # Start ROS 2 client library
    rclpy.init()

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add nodes to executor
    firmware_node = FirmwareNode()
    test_node = ExampleNode()
    executor.add_node(firmware_node)
    executor.add_node(test_node)

    # Spin in a separate thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    yield {
        "mock_serial_port": firmware_node.firmware_serial_port,
        "test_node": test_node,
        "firmware_node": firmware_node,
        "executor": executor,
    }

    # Ensure resources are cleaned up properly
    firmware_node.close()
    rclpy.shutdown()
    executor_thread.join()
