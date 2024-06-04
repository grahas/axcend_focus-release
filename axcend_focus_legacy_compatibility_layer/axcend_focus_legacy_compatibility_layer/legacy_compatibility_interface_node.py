"""Node provides access to the firmware_bridge RX and TX topics."""

import threading
from queue import Empty, Queue

from rclpy.node import Node
from std_msgs.msg import String

from axcend_focus_custom_interfaces.srv import (
    CartridgeMemoryReadWrite,
    SystemParametersUpdate,
)
from axcend_focus_custom_interfaces.msg import PumpStatus


class LegacyCompatibilityInterface(Node):
    """Node provides access to the firmware_bridge RX and TX topics."""

    def __init__(self, write_queue: Queue, read_queue: Queue):
        super().__init__("legacy_compatibility_interface")

        self.write_queue = write_queue
        self.read_queue = read_queue
        self.shutdown_event = threading.Event()

        # Create a publisher for the firmware_UART_write topic
        self.firmware_UART_write_publisher = self.create_publisher(
            String, "firmware_UART_write", 10
        )

        # Create a subscriber for the firmware_UART_read topic
        self.firmware_UART_read_subscription = self.create_subscription(
            String, "firmware_UART_read", self.firmware_UART_read_callback, 10
        )

        # Create a client for the cartridge_memory_read_write service
        self.cartridge_memory_read_write_client = self.create_client(
            CartridgeMemoryReadWrite, "cartridge_memory_read_write"
        )

        # Create a client to request the system parameters to be updated
        self.system_parameters_update_client = self.create_client(
            SystemParametersUpdate, "system_parameters_update"
        )

        # Create a subscriber for the pump status topic
        self.pump_status_subscription = self.create_subscription(
            PumpStatus, "pump_status", self.pump_status_callback, 10
        )
        self.pump_status_cache = PumpStatus()

    def firmware_UART_read_callback(self, msg):
        """Callback function for the firmware_bridge TX topic."""
        # Remove the proto1 prefix
        msg.data = msg.data.split("proto1 ")[1]
        self.read_queue.put(msg.data)

    def publish_to_firmware_UART_write_thread(self):
        """Publish messages from the queue to the firmware_UART_write topic."""
        while not self.shutdown_event.is_set():
            try:
                msg = self.write_queue.get(timeout=0.5)
                self.firmware_UART_write_publisher.publish(msg)
            except Empty:
                continue

    def request_cartridge_memory(self, max_retries=10) -> dict:
        """Send a request to the cartridge_memory_read_write service."""
        cartridge_memory_request = CartridgeMemoryReadWrite.Request()
        cartridge_memory_request.command = "read"
        retries = 0
        while not self.cartridge_memory_read_write_client.wait_for_service(
            timeout_sec=1.0
        ):
            if retries >= max_retries:
                self.get_logger().error("Service not available after maximum retries.")
                return {}
            if retries % 5 == 0:  # Log every 5 retries
                self.get_logger().info("Service not available, waiting again...")
            retries += 1

        # Get the cartridge memory data
        response = self.cartridge_memory_read_write_client.call(
            cartridge_memory_request
        )

        # Convert it to a dictionary
        fields = (
            response.get_fields_and_field_types().keys()
        )  # Get the fields of the response message
        results_dict = {
            field: getattr(response, field)
            for field in fields
            if hasattr(response, field)
        }  # Convert the response message to a dictionary

        # Return the dictionary
        return results_dict

    def request_system_parameters_update(self, max_retries=10) -> bool:
        """Send a request to the system_parameters_update service."""
        system_parameters_update_request = SystemParametersUpdate.Request()
        retries = 0
        while not self.system_parameters_update_client.wait_for_service(
            timeout_sec=1.0
        ):
            if retries >= max_retries:
                self.get_logger().error("Service not available after maximum retries.")
                return False
            if retries % 5 == 0:  # Log every 5 retries
                self.get_logger().info("Service not available, waiting again...")
            retries += 1

        response = self.system_parameters_update_client.call(
            system_parameters_update_request
        )

        if not hasattr(response, "success"):
            self.get_logger().error("Response does not contain 'success' attribute.")
            return False

        return response.success

    def pump_status_callback(self, msg):
        """Update the pump status cache."""
        self.pump_status_cache = msg

    def get_machine_state_string(self) -> str:
        """Return the machine state as a string."""
        results = (
            "unused {\n} "
            f"flow {self.pump_status_cache.flow_rate} "
            f"streamId {self.pump_status_cache.header.stamp.sec} "
            f"phase {self.pump_status_cache.phase} "
            "packetId DV "
            f"positionA {self.pump_status_cache.position[0]} "
            f"pressureA {self.pump_status_cache.pressure[0]} "
            f"positionB {self.pump_status_cache.position[1]} "
            f"pressureB {self.pump_status_cache.pressure[1]}"
        )
        return results
