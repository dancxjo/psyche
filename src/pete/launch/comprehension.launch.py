from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# The onboard processing of listening for speech and later reporting it to the psyche as a sensation
def generate_launch_description():
    return LaunchDescription([
        # Audio segmenter
        Node(
           package="psyche",
           executable="listen_for_speech", 
           name="speech_segmenter",
           output="screen",
           parameters=[{
               "device_index": 0
           }]
        ),
        
        # Reporting of the transcribed speech to the psyche (i.e. "/sensation")
        # Node(
        #     package="psyche",
        #     executable="sense_speech",
        #     name="transcription_sense",
        #     output="screen",
        # ),
        
                Node(
            package="psyche",
            executable="sense",
            name="hunger",
            output="screen",
            parameters=[{
                'sensor_id': "Speech comprehension",
                'reliability': "low",
                'processing_notes': "Provided by whisper. You're liable to have misheard. Try to piece together the conversation from context.",
                'input_topics': ['context', 'instant', 'audio/transcription'],
                'input_types': ['std_msgs.msg.Float32', 'std_msgs.msg.Float32', 'std_msgs.msg.Float32', 'create_msgs.msg.ChargingState', 'std_msgs.msg.Float32', 'std_msgs.msg.Int16', 'std_msgs.msg.Float32'],
                'update_interval': 15.0,
                # 'accumulation_method': "distilled",
            }]
        ),

    ])
