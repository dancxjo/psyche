from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# The secondary, on board language processor; expected to run slowly
def generate_launch_description():
    return LaunchDescription([
        # The onboard lpu
        Node(
            package="psyche",
            executable="lpu",
            name="local_language_processor",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://127.0.0.1:11434"},
                {"action_server_name": "/instruct_backup"},
            ],
        ),
    ])
