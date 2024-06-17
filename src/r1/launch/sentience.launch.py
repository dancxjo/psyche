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
    # Senses & Faculties
    sentience = Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Below are the sensations that you have recently felt. Narrate this information to yourself in the first person present. Using only the information here, describe the current instant as you are experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\nHey! What\'s going on?\n",
                "input_topics": ["/rosout", "sensation", "proprioception", "situation", "intent"],
                "output_topic": "instant",
                "update_interval": 2.5,
            }]
        )
    
    faculties = [sentience]    

    return LaunchDescription([
        *faculties,
    ])
