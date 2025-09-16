from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(os.path.dirname(__file__), '..', 'params', 'voice_params.yaml')
    node = Node(
        package='voice_node',
        executable='voice_node',
        name='voice_node',
        parameters=[params]
    )
    return LaunchDescription([node])
