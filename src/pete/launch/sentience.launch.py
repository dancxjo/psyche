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
        # The witness integrates buffered sensations into the current /instant
        Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "node_name": "the_witness",
                "action_server_name": "/recall",
                "narrative": "These are your recent sensations. Distill them into a coherent 'instant.' Use the raw data to tell a (true) story.",
                "input_topic_list": ["/sense_of_self", "/sensation", "/thought", "/voice"],
                "output_topic": "/instant",
                "update_interval": 30.0,    # Ideally this would be fast
            }]
        ),
        
    ])
