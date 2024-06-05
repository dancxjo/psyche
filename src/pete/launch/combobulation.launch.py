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
        # The contextualizer consolidates a buffer of recent /instant\s into a coherent /context 
        Node(
            package="psyche",
            executable="distill",
            name="the_combobulator",
            output="screen",
            parameters=[{
                "node_name": "the_combobulator",
                "action_server_name": "/recall",
                "narrative": "These are your understandings from the most recent instants of your life. Contextualize them into a coherent 'scene.' Use the raw data to tell a (true) story. Continue from the last response to this prompt, adding to it, revising it or moving on from it as necessary.",
                "input_topic_list": ["/sense_of_self", "/sensation", "/context", "/thought", "/instant"],
                "output_topic": "/context",
                "update_interval": 2 * 60.0,
            }]
        ),
    ])
