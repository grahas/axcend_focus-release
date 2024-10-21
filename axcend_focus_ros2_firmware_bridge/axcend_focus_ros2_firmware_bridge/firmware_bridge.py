"""High level wrapper over the firmware to make it easier to use."""

# import debugpy

# debugpy.listen(("0.0.0.0", 5678))
# print("Waiting for debugger attach")
# debugpy.wait_for_client()

import os
import queue
import signal
import sys
import threading
import time
import subprocess

import rclpy
import serial
from axcend_focus_custom_interfaces.action import ValveRotate, PumpPositioning
from axcend_focus_custom_interfaces.msg import (
    CartridgeOvenStatus,
    CartridgeDetectorStatus,
    PumpStatus,
)
from axcend_focus_custom_interfaces.srv import (
    CartridgeMemoryReadWrite,
    SystemParametersUpdate,
)
from axcend_focus_test_utils_package.mock_serial_port import create_mock_serial_port
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Header, String

from axcend_focus_ros2_firmware_bridge.packet_definitions import (
    DataAcquisitionState,
    PacketTranscoder,
    AbortPacketReasons,
)

# Define some constants
GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE = 0x01

CHANNEL_A = 0
CHANNEL_B = 1
PUMP_A = 0
PUMP_B = 1

class EnviromentConfig:
    @staticmethod
    def get_firmware_state():
        """Get the state of the firmware.

        Allows for easier testing by returning a constant value in the development environment.
        """
        if os.environ.get("ENVIRONMENT") == "development":
            return "running"
        
        # Check the state of the firmware from the remote processor driver and firmware helper script
        firmware_helper = os.environ.get("firmware")
        firmware_state_command = [firmware_helper, "state"]
        result = subprocess.run(firmware_state_command, capture_output=True, check=True)
        return result.stdout.decode().strip()
    
    @staticmethod
    def get_serial_port() -> serial.Serial:
        """Open the serial port to the firmware if availible.
        
        Will block until the serial port is availible.
        """
        if (os.environ.get("ENVIRONMENT") == "development"):
            return create_mock_serial_port()

        # Variables
        firmware_serial_port_path = ["/dev/ttyRPMSG0", "/dev/ttyRPMSG1"]

        # Try to open the serial port
        while True:
            for port in firmware_serial_port_path:
                try:
                    firmware_serial_port = serial.Serial(port, timeout=1)
                    if firmware_serial_port:
                        return firmware_serial_port
                except serial.SerialException:
                    time.sleep(1)
    
    @staticmethod
    def get_firmware_initialization_delay():
        """Get the delay before the firmware is ready to accept packets."""
        if os.environ.get("ENVIRONMENT") == "development":
            return 0
        return 5

class FirmwareState:
    def __init__(self):
        self._firmware_state = "offline"

    @property
    def firmware_state(self):
        return self._firmware_state

    @firmware_state.setter
    def firmware_state(self, new_state):
        if new_state != self._firmware_state:
            print(f"State changing from {self._firmware_state} to {new_state}")
            # Here you can add any additional logic you want to execute on state change
            self._firmware_state = new_state
            # For example, trigger an event or log the state change

# Example usage
firmware = FirmwareState()
firmware.firmware_state = "online"  # This will print: State changing from offline to online
firmware.firmware_state = "updating"  # This will print: State changing from online to updating

class FirmwareNode(Node):
    """Create a ROS2 node that will translate between the firmware and ROS2."""

    def __init__(self):
        # Initialize the node
        super().__init__("firmware_node")

        # Create a serial port to the firmware
        self.firmware_serial_port = None
        self.firmware_serial_port_status_ok = False

        # Create queues for transmitting and receiving packets
        self.transmit_queue = queue.Queue()

        # Load the packet encoding / decoding library
        self.packet_transcoder = PacketTranscoder()

        # Create a default acknowledgement packet
        self.acknowledgement_packet = (
            self.packet_transcoder.create_acknowledgement_packet()
        )

         # Used for coordinating the threads lifecycle
        self.is_alive = threading.Event()
        self.is_alive.set()

        # Create a thread to receive data from the firmware
        self.receive_thread_handler = threading.Thread(
            target=self.receive_thread, daemon=True
        )
        self.receive_thread_handler.start()

        # Create a thread to transmit data to the firmware
        self.transmit_thread_handler = threading.Thread(
            target=self.transmit_thread, daemon=True
        )
        self.transmit_thread_handler.start()

        # Create a publisher for raw serial data received from the firmware
        self.firmware_UART_read_publisher = self.create_publisher(String, "firmware_UART_read", 128)

        # Create a subscriber for raw serial data to be sent to the firmware
        self.subscription = self.create_subscription(
            String, "firmware_UART_write", self.listener_callback, 128
        )

        # Create a subscriber for the front panel button
        self.front_panel_button_subscription = self.create_subscription(
            String, "front_panel_button", self.front_panel_button_callback, 10
        )

        # Create a service to handle the write cartridge memory command {#5fb8cf,5}
        self.cartridge_memory_write_service = self.create_service(
            CartridgeMemoryReadWrite,
            "cartridge_memory_read_write",
            self.callback_cartridge_memory_read_write,
        )

        self.cartridge_memory_service_cache = CartridgeMemoryReadWrite.Response()

        # Service to write the system parameters to the firmware
        self.system_parameters_service = self.create_service(
            SystemParametersUpdate,
            "system_parameters_update",
            self.callback_system_parameters_update,
        )

        # Create a service to handle moving the valves {#6f4162,6}
        self.rotate_valves_action = ActionServer(
            self,
            ValveRotate,
            "rotate_valves",
            self.callback_rotate_valves,
        )

        # Create a publisher for detector data
        self.UV_data_publisher = self.create_publisher(
            CartridgeDetectorStatus, "UV_data", 10
        )

        # Cache for the cartridge sensor data
        self.cartridge_sensor_data_cache = CartridgeDetectorStatus()
        self.cartridge_sensor_data_cache.header = Header()

        # Create a publisher for oven data {#09513c,3}
        self.cartridge_oven_status = self.create_publisher(
            CartridgeOvenStatus, "cartridge_oven_status", 10
        )
        # Cache for the cartridge oven data
        self.cartridge_oven_status_cache = CartridgeOvenStatus()
        self.cartridge_oven_status_cache.header = Header()

        # Create a publisher for pressure / pump data
        self.pressure_publisher = self.create_publisher(PumpStatus, "pump_status", 10)

        # Create a action to handle moving the pump to a specified volume
        self.pump_positioning_service = ActionServer(
            self,
            PumpPositioning,
            "position_pumps",
            self.callback_pump_positioning,
        )

        # Create a publisher for phase packets
        self.phase_publisher = self.create_publisher(String, "device_phase", 10)

        # Initialize the firmware connection manager thread
        self.last_heartbeat_time = time.time()
        self.heartbeat_timeout = 5.0  # Timeout after 5 seconds
        self.firmware_connection_manager_thread = threading.Thread(
            target=self.firmware_connection_manager, daemon=True
        )
        self.firmware_connection_manager_thread.start()

        # Create a dictionary of functions for handling packets
        self.packet_handlers = {
            ("proto1", "GH"): self.handle_heart_beat_packet,
            ("proto1", "DA"): self.handle_command_acknowledgement_packet,
            ("cartridge_config", "DC"): self.handle_cartridge_config_packet,
            ("proto1", "DT"): self.handle_cartridge_oven_status_packet,
            ("proto1", "DU"): self.handle_cartridge_UV_data_packet,
            ("proto1", "GT"): self.handle_get_oven_state,
            ("proto1", "DN"): self.handle_phase,
            ("proto1", "DV"): self.handle_pressure_packet,
            ("proto1", "DR"): self.handle_pump_percent_composition_packet,
        }

    @property
    def firmware_state(self):
        return EnviromentConfig.get_firmware_state()

    def initialize_firmware(self):
        """Initialize the firmware.'

        This will send information about the hardware to the firmware.
        This will also collect initial readings from all the sensors.
        """
        # Establish RPmsg port
        self.transmit_queue.put(self.packet_transcoder.create_heartbeat_packet())

        # Send the firmware the system parameters packet
        system_parameters_packet = (
            self.packet_transcoder.create_system_parameters_packet()
        )
        self.transmit_queue.put(system_parameters_packet)

        # Populate the initial values of the cartridge memory
        self.transmit_queue.put(
            self.packet_transcoder.create_cartridge_memory_read_packet()
        )

        # Toggle the data acquisition state
        self.set_data_acquisition_state(DataAcquisitionState.CARTRIDGE_AND_VALVE_PUMPS)
        time.sleep(1)
        self.set_data_acquisition_state(DataAcquisitionState.DISABLED)

        # Send the firmware the heart beat packet
        self.transmit_queue.put(self.packet_transcoder.create_heartbeat_packet())

    def firmware_connection_manager(self):
        """Check if the firmware is still alive."""
        # Variables
        heartbeat_packet = self.packet_transcoder.create_heartbeat_packet()
        firmware_state_previous = "offline"
        
        while self.is_alive.is_set():
            # Update the current firmware state
            firmware_state_current = EnviromentConfig.get_firmware_state()

            if firmware_state_current == "offline":
                # Firmware is not running so sleep and check again
                self.get_logger().error(
                    "Firmware is not running, sleeping..."
                )

            elif firmware_state_current == "running":
                # Check if a serial port is open
                if (not self.firmware_serial_port) or (self.firmware_serial_port_status_ok is False):
                    self.firmware_serial_port = EnviromentConfig.get_serial_port()
                    self.firmware_serial_port_status_ok = True

                # If the firmware state has changed, reinitialize the firmware
                if firmware_state_previous != firmware_state_current:
                    time.sleep(EnviromentConfig.get_firmware_initialization_delay())
                    self.initialize_firmware()

                # Check if the firmware is alive
                self.transmit_queue.put(heartbeat_packet)
                if time.time() - self.last_heartbeat_time > self.heartbeat_timeout:
                    self.get_logger().error("Firmware is not responding!")
            else:
                self.get_logger().error(f"Firmware is in an unknown state: {firmware_state_current}!")
            
            # Sleep for a second
            firmware_state_previous = firmware_state_current
            time.sleep(1)
        

    def handle_heart_beat_packet(self, _):
        """Handle the heart beat packet."""
        self.last_heartbeat_time = time.time()

    def handle_phase(self, packet):
        """Receive the change in phase state packet."""
        phase = self.packet_transcoder.parse_phase_packet(packet)
        msg = String()
        msg.data = phase
        self.phase_publisher.publish(msg)

    def handle_command_acknowledgement_packet(self, packet):
        """Handle the command acknowledgement packet."""
        pass

    def handle_cartridge_config_packet(self, packet):
        """Update the cartridge_memory_service_cache with the data from the packet."""
        for field_name, _ in getattr(packet.data, "_fields_"):
            if hasattr(self.cartridge_memory_service_cache, field_name):
                value = getattr(packet.data, field_name)
                if isinstance(value, bytes):
                    value = value.decode()
                else:
                    value = int(value)
                setattr(self.cartridge_memory_service_cache, field_name, value)

    def handle_pressure_packet(self, packet):
        """Handle the pressure packet."""
        [
            phase,
            stream_id,
            pressure_a,
            pressure_b,
            position_a,
            position_b,
            flow_rate,
        ] = self.packet_transcoder.parse_pressure_packet(packet)

        # Create a pump status message
        pump_status_message = PumpStatus()
        pump_status_message.phase = phase
        pump_status_message.header.stamp.sec = stream_id
        pump_status_message.pressure[CHANNEL_A] = pressure_a
        pump_status_message.pressure[CHANNEL_B] = pressure_b
        pump_status_message.position[CHANNEL_A] = position_a
        pump_status_message.position[CHANNEL_B] = position_b
        pump_status_message.flow_rate = flow_rate

        # Publish the message
        self.pressure_publisher.publish(pump_status_message)

    def handle_pump_percent_composition_packet(self, packet):
        pass

    def handle_cartridge_UV_data_packet(self, packet):
        """Handle the UV data packet."""
        [channel_id, time_stamp, value] = self.packet_transcoder.parse_UV_data_packet(
            packet
        )

        # Update the cache
        self.cartridge_sensor_data_cache.signal[channel_id] = value
        self.cartridge_sensor_data_cache.header.stamp.sec = time_stamp

        # Publish the message
        self.UV_data_publisher.publish(self.cartridge_sensor_data_cache)

    def set_phase(self, phase):
        """Set the phase of the firmware."""
        pass

    def set_operational_mode(self, mode):
        """Set the operational mode of the firmware."""
        pass

    def callback_system_parameters_update(self, _, response):
        """Handle the system parameters write request."""
        # Create a system parameters packet
        system_parameters_packet = (
            self.packet_transcoder.create_system_parameters_packet()
        )

        # Send the packet
        self.transmit_queue.put(system_parameters_packet)

        response.success = True

        return response

    def callback_cartridge_memory_read_write(self, request, response):
        """Handle the read or write cartridge memory request."""
        if request.command == "read":
            # Create a packet to read the cartridge memory and send it
            self.transmit_queue.put(
                self.packet_transcoder.create_cartridge_memory_read_packet()
            )

            # Wait for one second for the response
            time.sleep(1)

            # Return the response
            response = self.cartridge_memory_service_cache

        elif request.command == "write":
            # Create a cartridge memory packet
            cartridge_memory_string = (
                self.packet_transcoder.create_cartridge_memory_write_packet(request)
            )
            # Send the packet
            self.transmit_queue.put(cartridge_memory_string)

        else:
            print("Invalid command received")

        # Wait for one second for the acknowledgement
        time.sleep(1)

        return response

    def callback_rotate_valves(self, goal_handle):
        """Handle the rotate valves request."""
        solvent_valve_position = goal_handle.request.valve_position[0]
        injection_valve_position = goal_handle.request.valve_position[1]

        packet_string = self.packet_transcoder.create_valve_rotate_packet(
            solvent_valve_position, injection_valve_position
        )

        # Send the packet to the firmware
        self.transmit_queue.put(packet_string)

        # Make the results
        result = ValveRotate.Result()
        result.success = True

        # Return a response
        goal_handle.succeed()

        return result

    def callback_pump_positioning(self, goal_handle):
        """Handle the pump positioning request."""
        # Get the goal
        goal = goal_handle.request

        # Create the packet
        packet_string = self.packet_transcoder.create_pump_positioning_packet(
            goal.volume[PUMP_A], goal.volume[PUMP_B]
        )

        # Send the packet to the firmware
        self.transmit_queue.put(packet_string)

        # Make the results
        result = PumpPositioning.Result()
        result.success = True

        # Return a response
        goal_handle.succeed()

        return result

    def handle_cartridge_oven_status_packet(self, packet):
        """Deserialize the cartridge temperature packet and publishing it to the ROS2 topic."""
        # Extract the values from the packet
        [sequence_number, oven_state, temperature_as_float, power_output, set_point] = (
            self.packet_transcoder.parse_values_oven_status_packet(packet)
        )

        # Set the header
        self.cartridge_oven_status_cache.header = Header()
        self.cartridge_oven_status_cache.header.stamp.sec = (
            sequence_number.value // 1000
        )
        self.cartridge_oven_status_cache.header.stamp.nanosec = int(
            (sequence_number.value % 1000) * 1e6
        )
        self.cartridge_oven_status_cache.header.frame_id = "cartridge"

        # Set the oven state
        self.cartridge_oven_status_cache.oven_state = bool(oven_state.value)

        # Set the power output
        self.cartridge_oven_status_cache.power_output = power_output.value

        # Set the temperature
        self.cartridge_oven_status_cache.current_temperature = temperature_as_float

        # Set the set point
        self.cartridge_oven_status_cache.target_temperature = set_point

        # Publish the message
        self.cartridge_oven_status.publish(self.cartridge_oven_status_cache)

    def handle_get_oven_state(self, packet):
        """Send the oven state packet to the firmware."""
        pass

    def set_data_acquisition_state(self, state: DataAcquisitionState):
        """Set the data acquisition state of the firmware."""
        packet = self.packet_transcoder.create_data_acquisition_state_packet(state)
        self.transmit_queue.put(packet)

    def transmit_thread(self):
        """Thread is responsible for sending data to the firmware."""
        while rclpy.ok() and self.is_alive.is_set():
            # Wait for serial port to be available
            if not self.firmware_serial_port_status_ok:
                time.sleep(1)
                continue
            try:
                packet = self.transmit_queue.get(timeout=1)
                if packet:
                    self.firmware_serial_port.write(packet)
                    print(f"Sent: {packet}")

            except queue.Empty:
                # No items in the queue this time, just continue with the loop
                continue

            except serial.SerialException:
                self.firmware_serial_port_status_ok = False

    def receive_thread(self):
        """Receive data from the firmware and publish it to the ROS2 topic."""
        while rclpy.ok() and self.is_alive.is_set():
            # Wait for serial port to be available
            if not self.firmware_serial_port_status_ok:
                time.sleep(1)
                continue
            try:
                received_data = self.firmware_serial_port.readline().decode().strip()
                if received_data:
                    # Publish the raw data
                    msg = String()
                    msg.data = received_data
                    self.firmware_UART_read_publisher.publish(msg)

                    # Print for debugging
                    print(f"Received: {received_data}")

                    # Decode and dispatch the packet to the appropriate handler
                    [prefix, packet_type, packet] = (
                        self.packet_transcoder.decode_packet(received_data)
                    )
                    self.packet_handlers[(prefix, packet_type)](packet)

            except serial.SerialTimeoutException:
                # No data received this time, just continue with the loop
                continue

            except AttributeError:
                # No data received this time, just continue with the loop
                continue

            except serial.SerialException:
                self.firmware_serial_port_status_ok = False                
                # Dump all the enqued packets
                self.transmit_queue.queue.clear()

            except Exception as e:
                print(f"Error: {e}")

    def listener_callback(self, msg):
        """Enqueue data to be written to the firmware."""
        self.transmit_queue.put(msg.data.encode())

    def front_panel_button_callback(self, msg):
        """Handle the front panel button press."""
        if "short" in msg.data:
            # Publish that an event packet was received
            msg2 = String()
            msg2.data = self.packet_transcoder.create_event_packet().decode('UTF-8').strip()
            self.firmware_UART_read_publisher.publish(msg2)
        elif "long" in msg.data:
            # Write an abort packet to the firmware
            self.transmit_queue.put(self.packet_transcoder.create_abort_packet(AbortPacketReasons.User))

    def close(self):
        """Clean up the firmware node."""
        # Teardown: join the receive transmit threads
        print("Joining the threads")
        self.is_alive.clear()
        self.firmware_connection_manager_thread.join()
        self.receive_thread_handler.join()
        self.transmit_thread_handler.join()
        print("Done joining the threads")

        # Teardown: close the serial port
        print("Closing the firmware serial port")
        self.firmware_serial_port.close()

        # Teardown: turn off the firmware
        # print("Turning off the firmware")
        # firmware_helper_script_path = os.environ.get("firmware")
        # os.system(f"{firmware_helper_script_path} stop")

        # Teardown: destroy the node
        print("Destroying the node")
        self.destroy_node()

    def signal_handler(self, signum, frame):
        """Handle the signal interrupt."""
        self.close()
        rclpy.shutdown()
        sys.exit(0)


def main():
    """Main function to start the firmware node."""
    # Start ROS 2 client library
    rclpy.init()

    # Create a multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add nodes to executor
    firmware_node = FirmwareNode()
    executor.add_node(firmware_node)

    # Register SIGINT handler for Ctrl+C
    signal.signal(signal.SIGINT, firmware_node.signal_handler)

    # Spin in a separate thread
    # executor_thread = Thread(target=executor.spin, daemon=True)
    # executor_thread.start()
    rclpy.spin(firmware_node)

    firmware_node.close()
    rclpy.shutdown()
    # executor_thread.join()

    # ros2 action send_goal /rotate_valves axcend_focus_custom_interfaces/action/ValveRotate "{valve_position: [0, 1]}"
    # pkill python


if __name__ == "__main__":
    main()
