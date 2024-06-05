from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Faculties related to memory
def generate_launch_description():
    return LaunchDescription([
        # The identity node is the core of the psyche. It maintains a /sense_of_self which informs all other faculties
        Node(
            package="psyche",
            executable="identity",
            name="identity",
            output="screen",
            parameters=[
                {"output_topic": "/sense_of_self"},
                {"input_topics": ["/context", "/instant", "/thought"]},
                {"update_interval": 5.0 * 60},      # It can update every five minutes
                {"action_server_name": "/recall"}   # It is well informed
            ]
        ),        
    ])
