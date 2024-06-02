from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource

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
            package="soma",
            
        ),
        Node(
            package="hc_sr04_sensor",
            executable="bringup",
            name="ultrasonic_sensor_node",
            output="screen",
        ),
        
        Node(
            package="distance",
            executable="sense",
            name="distance_sense",
            output="screen",
            parameters=[{
                "sensor_id": "forrward_ultrasonic_sensor",
                "reliability": "high",
                "processing_notes": "",
                "threshold": 1,
                "time_interval": 60,
            }]
        ),
    
        # Audio segmenter
        Node(
           package="whisper",
           executable="listen", 
           name="listen_for_voices",
           output="screen",
        ),
        
        # Reporting of the transcribed speech to the psyche (i.e. "/sensation")
        Node(
            package="whisper",
            executable="sense",
            name="voice_understanding_sense",
            output="screen",
        ),
        
        Node(
            package="psyche",
            executable="lpu",
            name="default_lpu",
            output="screen",
            parameters=[{
                "model": "llama3:instruct",
                "base_url": "http://192.168.0.129:11434",
            }]
        ),
        Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "node_name": "the_witness",
                "action_server_name": "/infer",
                "narrative": "These are your recent sensations. Distill them into a coherent 'instant.' Use the raw data to tell a (true) story.",
                "input_topic_list": ["/sense_of_self", "/sensation"],
                "output_topic": "/instant",
                "update_interval": 60.0,  # This should be longer than the time it takes to generate an instant but otherwise as short as possible
            }]
        ),
        Node(
            package="psyche",
            executable="distill",
            name="the_contextualizer",
            output="screen",
            parameters=[{
                "node_name": "the_contextualizer",
                "action_server_name": "/infer",
                "narrative": "These are your understandings from the most recent instants of your life. Contextualize them into a coherent 'scene.' Use the raw data to tell a (true) story. Continue from the last response to this prompt, adding to it, revising it or moving on from it as necessary.",
                "input_topic_list": ["/sense_of_self", "/sensation", "/context"],
                "output_topic": "/context",
                "update_interval": 3.0 * 60,  # This should be longer than the time it takes to generate a scene but otherwise as short as possible
            }]
        )
    ])
