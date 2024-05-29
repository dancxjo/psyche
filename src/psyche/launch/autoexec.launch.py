from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# This file is always executed when the package is launched
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="lpu",
            name="default_lpu",
            output="screen"
        )
    ])
