from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Must run next to the wiki
def generate_launch_description():
    return LaunchDescription([
        # The informant knows (through RAG) the content of the wiki and thus has a "memory"
        Node(
            package="psyche",
            executable="informant",
            name="informant",
            output="screen",        
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.129:11434"},
                {"input_topics": ["/memory"]},  # This updates the rag and the wiki
                {"action_server_name": "/recall"}
            ],
        ),
    ])
