from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# This is onboard the robot. As much as possible should run here
def generate_launch_description():
    return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('pete'),
                        'launch',
                        f'{script}.launch.py'
                    ])
                ])
            ) for script in ['hunger', 'comprehension', 'body']
    ])
