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
        #         {"model": "gpt-4"},
        #         {"model_type": "openai"},
        #     ],
        # ),


        # The main lpu that runs offboard
        Node(
            package="psyche",
            executable="lpu",
            name="local_language_processor",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.129:11434"},
                {"action_server_name": "/instruct"},
            ],
        ),
        
        # The onboard lpu
        Node(
            package="psyche",
            executable="lpu",
            name="local_language_processor",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://127.0.0.1:11434"},
                {"action_server_name": "/instruct_local"},
            ],
        ),

        # The informant knows (through RAG) the content of the wiki and thus has a "memory"
        Node(
            package="psyche",
            executable="informant",
            name="informant",
            output="screen",        
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.129:11434"},
                {"input_topics": ["/memory"]},  # This updates the rag and the wiki
                {"action_server_name": "/recall"}
            ],
        ),

        # The memorialist releases new /memory
        Node(
            package="psyche",
            executable="memorialist",
            name="memorialist",
            output="screen",
            parameters=[
                {"output_topic": "/memory"},
                {"input_topics": ["/sense_of_self", "/memory", "/context", "/instant", "/thought"]},
                {"update_interval": 2.0},
                {"action_server_name": "/recall"}   # We use an informant instead of a plain /instruct
            ]
        ),

        # The identity node is the core of the psyche. It maintains a /sense_of_self which informs all other faculties
        Node(
            package="psyche",
            executable="identity",
            name="identity",
            output="screen",
            parameters=[
                {"output_topic": "/sense_of_self"},
                {"input_topics": ["/context", "/instant", "/thought"]},
                {"update_interval": 5.0 * 60},      # It can update every five minutes
                {"action_server_name": "/recall"}   # It is well informed
            ]
        ),
        
        # The witness integrates buffered sensations into the current /instant
        Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "node_name": "the_witness",
                "action_server_name": "/recall",
                "narrative": "These are your recent sensations. Distill them into a coherent 'instant.' Use the raw data to tell a (true) story.",
                "input_topic_list": ["/sense_of_self", "/sensation", "/thought", "/voice"],
                "output_topic": "/instant",
                "update_interval": 30.0,    # Ideally this would be fast
            }]
        ),
        
        # The contextualizer consolidates a buffer of recent /instant\s into a /context 
        Node(
            package="psyche",
            executable="distill",
            name="the_contextualizer",
            output="screen",
            parameters=[{
                "node_name": "the_contextualizer",
                "action_server_name": "/recall",
                "narrative": "These are your understandings from the most recent instants of your life. Contextualize them into a coherent 'scene.' Use the raw data to tell a (true) story. Continue from the last response to this prompt, adding to it, revising it or moving on from it as necessary.",
                "input_topic_list": ["/sense_of_self", "/sensation", "/context", "/thought", "/instant"],
                "output_topic": "/context",
                "update_interval": 2 * 60.0,
            }]
        ),
        
        # This node monitors others and speaks directly; todo: should stream sentence by sentence
        Node(
            package="psyche",
            executable="distill",
            name="the_spokesperson",
            output="screen",
            parameters=[{
                "node_name": "the_spokesperson",
                "action_server_name": "/instruct_local",
                "narrative": "This is what's happening now. Any text you emit will be spoken out loud in the real world. To suppress your voice, include anywhere in your response the exact token $$$ME_THINKS$$$. Make sure you pay attention to time so that you don't overwhelm your interlocutor. Use the methinks to time yourself.",
                "input_topic_list": ["/sense_of_self", "/instant", "/context"],
                "output_topic": "/voice",
                "update_interval": 3.0          # Faster than this causes the robot to talk waaaayyy too fast
            }]
        )
    ])
