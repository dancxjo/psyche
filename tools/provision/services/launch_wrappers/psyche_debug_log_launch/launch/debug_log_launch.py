from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(
        package='debug_log_node',
        executable='debug_log_node',
        name='debug_log_node'
    ))
    return ld
