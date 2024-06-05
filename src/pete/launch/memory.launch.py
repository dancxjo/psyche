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
        # The memorialist releases new /memory
        Node(
            package="psyche",
            executable="memorialist",
            name="memorialist",
            output="screen",
            parameters=[
                {"output_topic": "/memory"},
                {"input_topics": ["/sense_of_self", "/memory", "/context", "/instant", "/thought"]},
                {"update_interval": 2.0},
                {"action_server_name": "/recall"}   # We use an informant instead of a plain /instruct
            ]
        ),
    ])