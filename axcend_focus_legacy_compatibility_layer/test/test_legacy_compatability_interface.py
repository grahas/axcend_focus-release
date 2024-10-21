import threading
import time

import pytest

import axcend_focus_ros2_firmware_bridge.packet_definitions as packet_definitions
from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface import (
    app,
    system_state,
    update_legacy_compatibility_interface_reference,
)
from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface_node import (
    LegacyCompatibilityInterface,
)
from axcend_focus_test_utils_package.conftest import nodes

# Create an instance of the PacketTranscoder
packet_transcoder = packet_definitions.PacketTranscoder()


@pytest.fixture
def client():
    with app.test_client() as client:
        yield client


@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset the system state before and after each test."""
    # Code to reset the system state goes here
    system_state["operator"] = None
    system_state["operator_ip"] = None
    system_state["reservation_key"] = None
    system_state["is_reserved"] = False

    # Yield control to the test function
    yield

    # Code to clean up after the test goes here (if necessary)


@pytest.fixture
def nodes_with_legacy(nodes):
    """This turns the nodes fixture into a fixture that includes the legacy compatibility interface node."""
    # Add the legacy compatibility interface node
    legacy_compatibility_interface = LegacyCompatibilityInterface(
        system_state["firmware_UART_write_queue"],
        system_state["firmware_UART_read_queue"],
    )
    nodes["executor"].add_node(legacy_compatibility_interface)

    # Update the reference to the legacy compatibility interface node
    update_legacy_compatibility_interface_reference(legacy_compatibility_interface)

    # Start the publish thread
    publish_thread = threading.Thread(
        target=legacy_compatibility_interface.publish_to_firmware_UART_write_thread
    )
    publish_thread.start()

    # Yield the nodes
    yield nodes

    # Clean up the legacy compatibility interface node
    legacy_compatibility_interface.shutdown_event.set()
    publish_thread.join()


def test_home(client):
    """Test the home route."""
    response = client.get("/")
    assert response.status_code == 200
    assert response.data == b"Hello, World!"


def test_is_reserved(client):
    """Test the is_reserved route."""
    response = client.get("/RPC2/IsReserved")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_reserve_and_release_system(client):
    """Test the reserve_system and release_system routes together."""
    # Test reserve_system route
    response = client.get("/RPC2/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"RESERVED" in response.data
    test_key = response.json["result"]["key"]

    # Test release_system route
    response = client.get(f"/RPC2/ReleaseSystem?key={test_key}")
    assert response.status_code == 200
    assert b"OK" in response.data


def test_reserve_system_already_reserved(client):
    """Test the reserve_system route when the system is already reserved."""
    client.get("/RPC2/ReserveSystem?user=test_user")
    response = client.get("/RPC2/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"ALREADY RESERVED" in response.data


def test_release_system_invalid_key(client):
    """Test the release_system route with an invalid key."""
    client.get("/RPC2/ReserveSystem?user=test_user")
    response = client.get("/RPC2/ReleaseSystem?key=invalid_key")
    assert response.status_code == 200
    assert b"INVALID KEY" in response.data


def test_release_system_not_reserved(client):
    """Test the release_system route when the system is not reserved."""
    response = client.get("/RPC2/ReleaseSystem?key=test_key")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_get_machine_name(client):
    """Test the machinename route."""
    response = client.get("/RPC2/machinename")
    assert response.status_code == 200


def test_get_version(client):
    """Test the version route."""
    response = client.get("/RPC2/version")
    assert response.status_code == 200


def test_read(client, nodes_with_legacy):
    """Test the read route."""
    # Get testing tools
    serial_port = nodes_with_legacy["mock_serial_port"]

    # Populate some dummy data for the client to read
    serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())
    serial_port.add_to_read_buffer(packet_transcoder.create_heartbeat_packet().encode())

    # Create the expected response
    expected_response = "".join(
        packet_transcoder.create_heartbeat_packet().split("proto1 ")[1] * 4
    )
    expected_response = "proto1 " + expected_response

    # Sleep for one second to allow the data to be read
    time.sleep(1)

    # Test the read route
    response = client.get("/RPC2/read")

    # Check the response
    assert response.status_code == 200
    assert expected_response.encode() in response.data


def test_write(client, nodes_with_legacy):
    """Test the write route."""
    # Get testing tools
    serial_port = nodes_with_legacy["mock_serial_port"]

    # Create a dummy packet to write
    dummy_packet = packet_transcoder.create_heartbeat_packet()
    dummy_packet = "proto1 4748000000000001000000000000000A"

    # Write the packet
    response = client.get(f"/RPC2/write?data={dummy_packet.split('proto1 ')[1]}")

    # Sleep for one second to allow the data to be written
    time.sleep(1)

    # Check the response
    assert response.status_code == 200
    assert response.json["result"] == len(dummy_packet)

    # Check the data in the firmware_UART_write_queue
    assert dummy_packet.encode() in serial_port.write_data


def test_cartridge_write(client, nodes_with_legacy):
    """Test the cartridge_write route."""
    # Get testing tools
    serial_port = nodes_with_legacy["mock_serial_port"]

    # Create a dummy packet to write
    cartridge_data = {
        "version": 0,
        "revision": 0,
        "serial_number": "123ABC123ABC",
        "max_pressure_rating": 0,
        "has_oven": 0,
        "is_third_party": 0,
        "last_write_date": "000000",
        "total_run_time": 0,
        "cartridge_flags": 0,
        "detector_count": 0,
        "detector_1_type": 0,
        "detector_1_wavelength": 0,
        "detector_1_path_length": 0,
        "detector_2_type": 0,
        "detector_2_wavelength": 0,
        "detector_2_path_length": 0,
        "column_count": 0,
        "column_1_length": 0,
        "column_1_diameter": 0,
        "column_1_particle_size": 0,
        "column_1_description": "0000",
        "column_2_length": 0,
        "column_2_diameter": 0,
        "column_2_particle_size": 0,
        "column_2_description": "0000",
        "detector_1_adc_gain": 0,
        "detector_1_adc_delay": 0,
        "detector_1_led_pot": 0,
        "detector_2_adc_gain": 0,
        "detector_2_adc_delay": 0,
        "detector_2_led_pot": 0,
        "bluetooth_disabled": 0,
        "report_rate": 0,
        "word_rate": 0,
        "unused": 0,
        "samples_per_point": 0,
        "bits_to_remove": 0,
    }
    dummy_packet = packet_transcoder.create_cartridge_memory_write_packet(
        cartridge_data
    ).decode()

    # Write the packet
    response = client.get(
        f"/RPC2/cartridge_write?data={dummy_packet.split('cartridge_config ')[1]}"
    )

    # Sleep for one second to allow the data to be written
    time.sleep(1)

    # Check the response
    assert response.status_code == 200
    assert len(dummy_packet) == response.json['result']

    # Check the data in the firmware_UART_write_queue
    assert dummy_packet.encode() in serial_port.write_data


def test_cartridge_config(client, nodes_with_legacy):
    """Test the cartridge_config route."""
    response = client.get("/RPC2/cartridge_config")

    # Check the response
    assert response.status_code == 200

    # Check the response data
    assert b"version" in response.data


def test_device_config(client):
    """Test the device_config route."""
    response = client.get("/RPC2/deviceconfig")

    # Check the response
    assert response.status_code == 200

    # Check the response data
    assert b"deviceconfig" in response.data


def test_update_system_parameters(client, nodes_with_legacy):
    """Test the update_system_parameters route."""
    # Get testing tools
    serial_port = nodes_with_legacy["mock_serial_port"]

    # Write the packet
    response = client.get("/RPC2/update_system_parameters")

    # Sleep for one second to allow the data to be written
    time.sleep(1)

    # Check the response
    assert response.status_code == 200

    # Check the data in the firmware_UART_write_queue
    count = sum(s.count(b"SysParams") for s in serial_port.write_data)
    assert (
        count == 2
    )  # Two because one is in the queue by default and one more for our test


def test_machine_state(client, nodes_with_legacy):
    """Test the machinestate route."""
    response = client.get("/RPC2/machinestate")

    # Check the response
    assert response.status_code == 200

    # Check the response data
    assert b"machinestate" in response.data
