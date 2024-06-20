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
    plain_lpu_local_net = Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://192.168.0.20:11434"},
                {"action_server_name": "instruct"}
            ],
        )

    informant = Node(
            package="psyche",
            executable="informant",
            name="informant",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://192.168.0.7:11434"},
                {"action_server_name": "inform"}
            ],
        )

    vision_lpu = Node(
        package="psyche",
        executable="vlpu",
        name="vision_lpu",
        output="screen",
        parameters=[
            {"model": "llava:13b"},
            {"image_support": True},
            {"base_url": f"http://192.168.0.20:11434"},
            {"action_server_name": "inspect"}
        ],
    )

    return LaunchDescription([
        plain_lpu_local_net,
        vision_lpu,
        informant,
        Node(
            package="psyche",
            executable="transcribe_speech",
            name="audio_comprehension",
            output="screen",
        )
   ])