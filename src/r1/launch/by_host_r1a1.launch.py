from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess  # Import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="stream_voice",
            name="the_voice",
            output="screen",
        ),
        ExecuteProcess(  # Add ExecuteProcess action
            cmd=[executable, "aplay", "/usr/share/sounds/alsa/Front_Center.wav"],  
            output="screen",
        ),
        Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
        )

   ])