from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = os.path.join(os.path.dirname(__file__), '..', 'params', 'create_params.yaml')
    create_node = Node(
        package='create_driver',
        executable='create_node',
        name='create_node',
        parameters=[params_file]
    )
    return LaunchDescription([create_node])
