from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='axcend_focus_ros2_firmware_bridge',
            executable='firmware_bridge',
            name='firmware_bridge'
        ),
        Node(
            package='axcend_focus_legacy_compatibility_layer',
            executable='legacy_compatibility_interface',
            name='legacy_compatibility_interface'
        ),
        Node(
            package='axcend_focus_front_panel_button',
            executable='front_panel_button_controller',
            name='front_panel_button_controller',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
        )
    ])