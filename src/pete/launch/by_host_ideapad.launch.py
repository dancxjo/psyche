from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# This is onboard the robot. As much as possible should run here
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="memorialist",
            name="memorialist",
            output="screen",
            parameters=[
                {"model": "wizardlm2"},
                {"base_url": "http://192.168.0.129:11434"},
            ]
        ),
    ])
