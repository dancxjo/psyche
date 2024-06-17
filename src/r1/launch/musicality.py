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

innate_musicality = Node(
        package="psyche",
        executable="distill",
        name="the_musician",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": """You are a robot. Below is the situation as you understand it and how you're feeling about it. Represent your current feelings in song. You must return valid JSON in the following format and nothing more! If you have no appropriate song to play, simply return nothing at all. Otherwise, specify your song as an array of between 1 and 16 notes. Each note in the array is an object with two fields:\n* `note` an unsigned 8-bit integer defined by the MIDI note numbering scheme (notes outside the range of [31-127] are rest notes)\n* `duration` a positive floating point number representing the duration of the note in seconds\nTry to use expressive and familiar tunes, but feel free to improvise.\n{input_topics}\n\nReminder:\nOnly respond with the actual JSON with no delimiting markdown codefences or otherwise. Return no other response.\nYour song:\n""",
            "input_topics": ["identity", "feeling", "instant", "situation", "intent"],
            "output_topic": "song",
            "update_interval": 5.0,
            "accumulation_method": "latest",
        }]
    )

sing = Node(
    package="r1",
    executable="play_song",
    name="song_player",
    output="screen",
)

