from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Faculties related to speaking
def generate_launch_description():
    return LaunchDescription([
        # This node monitors others and speaks directly; todo: should stream sentence by sentence
        Node(
            package="psyche",
            executable="distill",
            name="the_spokesperson",
            output="screen",
            parameters=[{
                "node_name": "the_spokesperson",
                "action_server_name": "/recall",
                "narrative": "This is what's happening now. Any text you emit will be spoken out loud in the real world. To suppress your voice, include anywhere in your response the exact token $$$ME_THINKS$$$. Make sure you pay attention to time so that you don't overwhelm your interlocutor. Use the methinks to time yourself. Only speak one sentence at a time.",
                "input_topic_list": ["/sense_of_self", "/instant", "/context"],
                "output_topic": "/voice",
                "update_interval": 3.0          # Faster than this causes the robot to talk waaaayyy too fast
            }]
        ),
    ])
