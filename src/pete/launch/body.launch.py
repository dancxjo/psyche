from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# This is onboard the robot. As much as possible should run here
def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('create_bringup'),
                        'launch',
                        'create_1.launch'
                    ])
                ])
        ),

        Node(
            package="pete",
            executable="hc_sr04_sensor",
            name="forward_ultrasonic_sensor",
            output="screen",
            parameters=[{
                'trigger_pin': 28,
                'trigger_chip': "/dev/gpiochip2",
                'echo_pin': 20,
                'echo_chip': "/dev/gpiochip1",
                'min_range': 0.03,
                'max_range': 4.0,
                'field_of_view_deg': 15.0,
                'frame_id': 'base_link',
                'output_topic': 'ultrasonic_sensor/forward_distance',
                'timer_period_sec': 0.1,
            }]
        ),
    
        #  Node(
        #     package="pete",
        #     executable="sense_buttons",
        #     name="button_sensations",
        #     output="screen",
        # ),

        # Node(
        #     package="pete",
        #     executable="sense_ir",
        #     name="omni_ir_sense",
        #     output="screen",
        # ),
        
        # Node(
        #     package="pete",
        #     executable="sense_metabolism",
        #     name="metabolism",
        #     output="screen",
        # ),
        
        # Node(
        #     package="pete",
        #     executable="sense_proprioception",
        #     name="proprioception",
        #     output="screen",
        # ),
    ])
