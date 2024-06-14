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

forebrain_host = "192.168.0.7"

def generate_launch_description():
    return LaunchDescription([
        # Start streaming the voice so we can hear where we are in the boot sequence
        # The outer container host should be listening to this and playing it
        Node(
            package="r1",
            executable="speak_directly",
            name="the_voice",
            output="screen",
        ),
        # Announce the boot sequence
        Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
        ),
        
        # Start the LLMS
        Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://{forebrain_host}:11434"},
                {"action_server_name": "instruct"}
            ],
        ),
        Node(
            package="psyche",
            executable="vlpu",
            name="vision_lpu",
            output="screen",
            parameters=[
                {"model": "llava:13b"},
                {"base_url": f"http://{forebrain_host}:11434"},
                {"action_server_name": "inspect"}
            ],
        ),

   ])
