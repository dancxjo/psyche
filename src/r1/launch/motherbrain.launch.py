from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="psyche",
            executable="heartbeat",
            name="heartbeat",
            output="screen",
            parameters=[
                {"processing_notes": "You are under development. This may feel funny."},
                {"update_interval": 1.0},
            ],
        ),
        
        Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llava:13b"},
                {"base_url": "http://192.168.0.133:11434"},
                {"action_server_name": "instruct"}
            ],
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent of the mind of a robot. Below are the sensations that the robot has recently felt. Narrate them for the robot in the first person present. Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "sensation"],
                "output_topic": "instant",
                "update_interval": 2.0,
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_combobulator",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent of the mind of a robot. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. Narrate them for the robot in the first person present. Using only the information here, describe the current situation as the robot is experiencing it. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation", "autobiography"],
                "output_topic": "situation",
                "update_interval": 15.0,
                "accumulation_method": "latest"
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_memoirist",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent of the mind of a robot. Below is the situation as you understand it. Your job is to record significant events in the life of the robot. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant. If nothing of importance is happening, or if you've already recorded this event, respond with no text at all.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation"],
                "output_topic": "autobiography",
                "update_interval": 60.0 * 5,
            }]
        ),

        
        Node(
            package="psyche",
            executable="distill",
            name="the_philosopher",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent of the mind of a robot. Below is the situation as you understand it. Explain to the robot in the first person present who it is. Using only the information here, describe to the robot who it experiences itself as. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation"],
                "output_topic": "identity",
                "update_interval": 60.0,
            }]
        ),
    ])
