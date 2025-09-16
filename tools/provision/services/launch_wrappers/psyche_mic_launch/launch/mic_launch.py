from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(os.path.dirname(__file__), '..', 'params', 'mic_params.yaml')
    mic = Node(
        package='mic_node',
        executable='mic_node',
        name='mic_node',
        parameters=[params_file]
    )
    return LaunchDescription([mic])
