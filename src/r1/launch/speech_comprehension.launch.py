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
    listen_for_speech = Node(
            package="psyche",
            executable="listen_for_speech",
            name="the_ear",
            output="screen",
            parameters=[
                {"device_index": 1}
            ]
        )
    sensors = [listen_for_speech]
    
    # Senses & Faculties
    return LaunchDescription([
        *sensors,
    ])
