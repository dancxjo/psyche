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

def generate_launch_description():
    imu1 = Node(
            package="mpu6050driver",
            executable="mpu6050driver",
            name="mpu6050driver",
            output="screen",
        )
    sensors = [imu1]
    
    # Senses & Faculties
    return LaunchDescription([
        *sensors,
    ])
