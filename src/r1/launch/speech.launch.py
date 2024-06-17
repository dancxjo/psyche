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
victus_host = "192.168.0.19"
offboard_host = "192.168.0.3"

def generate_launch_description():
    speech = Node(
            package="psyche",
            executable="distill",
            name="the_spokesman",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Below is the situation as you understand it and how you're feeling about it. Use the information to speak if necessary.
                
                {input_topics}
                
                Be careful not to speak too often so as to overwhelm your interlocuter. Only say one sentence at a time as it will allow more frequent checking. If you have nothing to say, your response should include no text at all. Otherwise, it will be read verbatim in the real world. Spell out all words including numbers (years, amounts, etc.) so the TTS doesn't struggle. Include *nothing* but the text to say. Do not include symbols other than commas, parentheses, exclamation marks and question marks. Do not say that you are passing or that you are done. Just say the text. If you have nothing to say, return an empty string.""",
                "input_topics": ["identity", "feeling", "instant", "situation"],
                "output_topic": "voice",
                "update_interval": 2.0,
                "accumulation_method": "latest",                
            }]
        )

    direct_speech = Node(
            package="r1",
            executable="speak_directly",
            name="the_voice",
            output="screen",
        )

    return LaunchDescription([
        speech,
        direct_speech
    ])
