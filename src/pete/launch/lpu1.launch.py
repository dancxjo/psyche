from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# The primary, most used language processor; run this on the fastest GPU closest to the edge
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="lpu",
            name="local_language_processor",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.129:11434"},
                {"action_server_name": "/instruct"},
            ],
        ),
        
    ])