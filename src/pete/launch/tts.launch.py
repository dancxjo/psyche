from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Faculties related to speaking out loud
def generate_launch_description():
    return LaunchDescription([
        # Listens for speech, turn it into audio and streams it over a tcp port as wav data
        Node(
            package="psyche",
            executable="stream_voice",
            name="arials_secret_weapon",
            output="screen",
        ),
    ])