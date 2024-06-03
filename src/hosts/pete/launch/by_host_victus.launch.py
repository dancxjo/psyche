from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# This is off-board on our local server. As much as possible, limit this to extremely heavy lifting
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="transcribe_speech",
            name="transcription_service",
            output="screen",
        ),
        Node(
            package="psyche",
            executable="stream_voice",
            name="tts_service",
            output="screen",
            parameters=[{
                'port': 8000,
            }]
        ),
    ])
