import os
import axcend_focus_ros2_firmware_bridge.packet_definitions as packet_definitions
from axcend_focus_ros2_firmware_bridge.packet_definitions import TestPhase

def test_packet_encoding():
    """Verify that the packet encoding library is working correctly."""
    # Print the current directory
    print("Current directory: ", os.getcwd())

    # Check the platform and architecture
    packet_transcoder = packet_definitions.PacketTranscoder()

    assert (
        packet_transcoder.create_heartbeat_packet().decode("utf-8")
        == "proto1 4748000000000000000000000000000A"
    ), "Heartbeat packet not created correctly"

# def test_parse_UV_data_packet():
#     """Check that we are able to parse a UV data packet."""
#     # Create a UV data packet
#     packet_transcoder = packet_definitions.PacketTranscoder()
#     test_channel_id = 1
#     test_time_stamp = 2^32 - 1
#     test_value = 3^32 - 1

#     uv_data_packet = packet_transcoder.create_dummy_UV_data_packet(test_channel_id, test_time_stamp, test_value)

#     [channel_id, time_stamp, value] = packet_transcoder.parse_UV_data_packet(uv_data_packet)

#     assert channel_id == test_channel_id, "Channel ID not parsed correctly"
#     assert time_stamp == test_time_stamp, "Timestamp not parsed correctly"
#     assert value == test_value, "Value not parsed correctly"

def test_parse_UV_data_packet():
    """Check that we are able to parse a UV data packet."""
    # Create a UV data packet
    packet_transcoder = packet_definitions.PacketTranscoder()
    packet_string = "proto1 44555200100000006623E1000000000A"
                            
    [prefix, packet_type, packet] = packet_transcoder.decode_packet(packet_string)

    [channel_id, time_stamp, value] = packet_transcoder.parse_UV_data_packet(packet)

    print("Channel ID: ", channel_id)
    print("Timestamp: ", time_stamp)
    print("Value: ", value)

def test_parse_phase_packet():
    """Check that we are able to parse a phase packet."""
    # Create a phase packet
    packet_transcoder = packet_definitions.PacketTranscoder()
    packet_string = "proto1 444e010000000000000000000000000A"
    [prefix, packet_type, packet] = packet_transcoder.decode_packet(packet_string)
    results = packet_transcoder.parse_phase_packet(packet)

    assert results == "Refill", "Phase not parsed correctly"

