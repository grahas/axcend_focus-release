import os
import axcend_focus_ros2_firmware_bridge.packet_definitions as packet_definitions


def test_packet_encoding():
    """Verify that the packet encoding library is working correctly."""
    # Print the current directory
    print("Current directory: ", os.getcwd())

    # Check the platform and architecture
    packet_transcoder = packet_definitions.PacketTranscoder()

    assert (
        packet_transcoder.create_heartbeat_packet()
        == "proto1 4748000000000000000000000000000A"
    ), "Heartbeat packet not created correctly"
