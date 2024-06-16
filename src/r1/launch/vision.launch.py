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
offboard_host = "192.168.0.19"

def generate_launch_description():
    # Senses & Faculties
    vision = Node(
            package="psyche",
            executable="distill",
            name="the_lookout",
            output="screen",
            parameters=[{
                "action_server_name": "inspect",
                "image_support": True,
                "prompt": "Describe the attached image(s) and their content (if any). {input_topics}\n",
                "input_topics": ["heartbeat"],
                "input_images": ["/image_raw/compressed"],
                "output_topic": "voice",
                "update_interval": 1.0,
                "accumulation_method": "latest"
            }]
        )

    faculties = [vision]
    
    return LaunchDescription([
        *faculties,
    ])
