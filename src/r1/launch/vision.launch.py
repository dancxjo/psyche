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
offboard_host = "192.168.0.19"

def generate_launch_description():
    vision_lpu = Node(
            package="psyche",
            executable="vlpu",
            name="vision_lpu",
            output="screen",
            parameters=[
                {"model": "llava:13b"},
                {"image_support": True},
                {"base_url": f"http://{forebrain_host}:11434"},
                {"action_server_name": "inspect"}
            ],
        )
    processors = [vision_lpu]

    # Sensors
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
        }]
    )
    sensors = [usb_cam]

    # Senses & Faculties
    vision = Node(
            package="psyche",
            executable="distill",
            name="the_lookout",
            output="screen",
            parameters=[{
                "action_server_name": "inspect",
                "image_support": True,
                "prompt": "You are a robot. Here is your internal state along with some recently processed images from your vision system. Using only the information here, describe what you see before you. Be succinct but thorough.\n\n{input_topics}\n\nWhat you're seeing as the robot:\n",
                "input_topics": ["identity", "instant", "situation", "intent"],
                "input_images": ["/image_raw"],
                "output_topic": "sensation",
                "update_interval": 1.0,
                "accumulation_method": "latest"
            }]
        )

    faculties = [vision]
    
    return LaunchDescription([
        *processors,
        *sensors,
        *faculties,
    ])
