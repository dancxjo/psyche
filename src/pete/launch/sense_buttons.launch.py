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
        Node(
            package="psyche",
            executable="sense",
            name="button_sense",
            output="screen",
            parameters=[{
                'sensor_id': "Button array on the top of your body",
                'reliability': "high",
                'processing_notes': "No data payload, triggered by physical button press; the clean_button is your play button, the dock_button is your advance button",
                'input_types': ['std_msgs.msg.Empty', 'std_msgs.msg.Empty'],
                'input_topics': ['/clean_button', '/dock_button'],
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
