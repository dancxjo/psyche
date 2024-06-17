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
victus_host = "192.168.0.19"
offboard_host = "192.168.0.3"

def generate_launch_description():
    intent = Node(
            package="psyche",
            executable="distill",
            name="the_intender",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. You have self-driven intentions that you must identify (and/or generate). Below are the the most recent instants in your life, along with the current situation as you have been understanding it. You must use this information to decide what to do next. Certain other parts of your mind will attempt to enact your commands, but you'll need to keep a close eye on things and adjust your plans as you move forward. Maintain short, medium and long-term goals. Decide on priorities and focuses and direct attention.\n\n{input_topics}\n\nIssue only one sentence in the form of: As a _____ I want to _____ so that _____. NOTHING ELSE, just one sentence.\n",
                "input_topics": ["identity", "instant", "situation", "twists", "intent"],
                "output_topic": "intent",
                "update_interval": 1.0,
                "accumulation_method": "latest"
            }]
        )

    return LaunchDescription([intent])
