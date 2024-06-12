from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="psyche",
        #     executable="lpu",
        #     name="cheap_lpu",
        #     output="screen",
        #     parameters=[
        #         {"model_type": "openai"},
        #         {"model": "gpt-3.5-turbo"},
        #         {"action_server_name": "instruct_cheap"}
        #     ],
        # ),    
        # Node(
        #     package="psyche",
        #     executable="lpu",
        #     name="expensive_lpu",
        #     output="screen",
        #     parameters=[
        #         {"model_type": "openai"},
        #         {"model": "gpt-4"},
        #         {"action_server_name": "instruct_expensive"}
        #     ],
        # ),    
        Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.4:11434"},
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
                {"base_url": "http://192.168.0.4:11434"},
                {"action_server_name": "inspect"}
            ],
        ),
    ])
