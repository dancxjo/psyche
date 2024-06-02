from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# This file is always executed when the package is launched
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="to_thine_own_self_be_true",
            name="autognosis_lpu",
            output="screen",
            parameters=[{
                "model": "llama3:instruct",
                "base_url": "http://192.168.0.129:11434",
            }]
        ),
    ])

