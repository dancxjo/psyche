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
    basic_autobiographical_memory = Node(
            package="psyche",
            executable="distill",
            name="the_memoirist",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot trying to make sense of your life. You must constantly keep remembering and updating your 'autobiography'. Below is the current situation as you understand it now. You must faithfully reproduce the autobiography that you receive while also adding to it and rectifying any mistakes. What you return will become the new autobiography, so attempt to be faithful. Record significant events in your life. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant.\n\n{input_topics}\n\n\n",
                "input_topics": ["identity", "instant", "situation", "intent"],
                "output_topic": "autobiography",
                "accumulation_method": "latest",
                "update_interval": 60.0,
            }]
        )
    identity = Node(
            package="psyche",
            executable="distill",
            name="the_philosopher",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot, a temporal, corporeal being at a certain place at a certain time. Below is your situation as you understand it. Who are you? Why are you here? What are you going to do with your life? Using only the information here, describe to yourself who you are. Be succinct but thorough.\n\n{input_topics}\n\nWho are you?\n",
                "input_topics": ["identity", "autobiography", "situation", "intent"],
                "output_topic": "identity",
                "update_interval": 60.0,
                "accumulation_method": "latest",                
            }]
        )

    return LaunchDescription([
        basic_autobiographical_memory,
        identity,
    ])
