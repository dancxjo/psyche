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
        # Node(
        #     package="psyche",
        #     executable="lpu",
        #     name="expensive_language_processor",
        #     output="screen",
        #     parameters=[
        #         # {"model": "llama3:instruct"},
        #         # {"base_url": "http://192.168.0.129:11434"},
        #         {"model": "gpt-4"},
        #         {"model_type": "openai"},
        #     ],
        # ),
        Node(
            package="psyche",
            executable="lpu",
            name="local_language_processor",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.129:11434"},
                # {"model": "gpt-4"},
                # {"model_type": "openai"},
            ],
        ),
        # Node(
        #     package="psyche",
        #     executable="memorialist",
        #     name="memorialist",
        #     output="screen",
        #     parameters=[
        #         {"output_topic": "/memory_management"},
        #         {"input_topics": ["/memory", "/memory_management", "/context", "/instant"]},
        #         {"update_interval": 5.0}
        #     ]
        # ),
        Node(
            package="psyche",
            executable="informant",
            name="informant",
            output="screen",        
        ),
        # Node(
        #     package="psyche",
        #     executable="distill",
        #     name="the_witness",
        #     output="screen",
        #     parameters=[{
        #         "node_name": "the_witness",
        #         "action_server_name": "/instruct",
        #         "narrative": "These are your recent sensations. Distill them into a coherent 'instant.' Use the raw data to tell a (true) story.",
        #         "input_topic_list": ["/sense_of_self", "/sensation"],
        #         "output_topic": "/instant",
        #         "update_interval": 5.0,  # This should be longer than the time it takes to generate an instant but otherwise as short as possible
        #     }]
        # ),
        # Node(
        #     package="psyche",
        #     executable="distill",
        #     name="the_contextualizer",
        #     output="screen",
        #     parameters=[{
        #         "node_name": "the_contextualizer",
        #         "action_server_name": "/instruct",
        #         "narrative": "These are your understandings from the most recent instants of your life. Contextualize them into a coherent 'scene.' Use the raw data to tell a (true) story. Continue from the last response to this prompt, adding to it, revising it or moving on from it as necessary.",
        #         "input_topic_list": ["/sense_of_self", "/sensation", "/context"],
        #         "output_topic": "/context",
        #         "update_interval": 60.0,  # This should be longer than the time it takes to generate a scene but otherwise as short as possible
        #     }]
        # )
    ])
