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
            name="hunger",
            output="screen",
            parameters=[{
                'sensor_id': "Your sense of hunger",
                'reliability': "high",
                'processing_notes': "",
                'input_topics': ['/battery/capacity', '/battery/charge', '/battery/charge_ratio', '/battery/charging_state', '/battery/current', '/battery/temperature', '/battery/voltage'],
                'input_types': ['std_msgs.msg.Float32', 'std_msgs.msg.Float32', 'std_msgs.msg.Float32', 'create_msgs.msg.ChargingState', 'std_msgs.msg.Float32', 'std_msgs.msg.Int16', 'std_msgs.msg.Float32'],
                'update_interval': 15.0,
                # 'accumulation_method': "distilled",
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
