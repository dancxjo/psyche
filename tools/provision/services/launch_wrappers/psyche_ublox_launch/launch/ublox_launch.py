from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(os.path.dirname(__file__), '..', 'params', 'ublox_params.yaml')
    ublox_node = Node(
        package='ublox',
        executable='ublox_serial_node',
        name='ublox_serial_node',
        parameters=[params_file]
    )
    return LaunchDescription([ublox_node])
