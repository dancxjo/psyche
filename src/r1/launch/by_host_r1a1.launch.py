from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from .identity import basic_autobiographical_memory, identity, emerge
from .context import combobulation
from .llms import plain_lpu, vision_lpu
from .sentience import sentience
from .vision import vision
from .voice import voice
from .executive import executive

def generate_launch_description():
    direct_dev_talk = Node(
            package="r1",
            executable="direct_dev_talk",
            name="direct_dev_talk",
            output="screen",
    )
    
    boot_announcer = Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
        )

    platform = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('create_bringup'),
                        'launch',
                        'create_1.launch'
                    ])
                ])
        )    

    heartbeat = Node(
            package="psyche",
            executable="heartbeat",
            name="heartbeat",
            output="screen",
            parameters=[
                {"update_interval": 60.0},
            ],
        )
    
    return LaunchDescription([
        plain_lpu,
        vision_lpu,
        direct_dev_talk,
        voice,
        boot_announcer,
        heartbeat,
        vision,
        platform,
        sentience,
        combobulation,
        basic_autobiographical_memory,
        emerge,
        identity,
        executive,
    ])
