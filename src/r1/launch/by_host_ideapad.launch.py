from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

forebrain_host = "192.168.0.7"
victus_host = "192.168.0.20"
offboard_host = "ideapad.local"

def generate_launch_description():
    boot_announcer = Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
        )

    heartbeat = Node(
            package="psyche",
            executable="heartbeat",
            name="heartbeat",
            output="screen",
            parameters=[
                {"update_interval": 60.0},
            ],
        )
    
    sentience = Node(
        package="psyche",
        executable="integrate",
        name="the_witness",
        output="screen",
    )

    voice = Node(
        package="r1",
        executable="speak_directly",
        name="the_voice",
        output="screen",
    )
    
    graph_memory = Node(
        package="psyche",
        executable="memorize",
        name="the_memory",
        output="screen",
        
    )

    usb_cam = Node(
       package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
            "framerate": 30.0,
        }]
    )

    vision = Node(
        package="psyche",
        executable="watch",
        name="the_lookout",
        output="screen",
    )
    
    # TODO: Detect & filter non-speech
    audio_segmenter = Node(
        package="psyche",
        executable="listen_for_speech",
        name="audio_segmenter",
        output="screen",
    )
    
    audio_transcriber = Node(
            package="psyche",
            executable="transcribe_speech",
            name="audio_comprehension",
            output="screen",
        )
    
    dedicated_forebrain_lmu = Node(
            package="psyche",
            executable="lpu",
            name="dedicated_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://192.168.0.7:11434"},
                {"action_server_name": "dedicated_instruct"}
            ],
        )
    
    return LaunchDescription([
       dedicated_forebrain_lmu,
       voice,
       boot_announcer,
       heartbeat,
        usb_cam,
        vision,
        audio_segmenter,
        audio_transcriber,
        # imu,
        # platform,
       sentience,
    #    combobulation,
       graph_memory,
#        basic_autobiographical_memory,
#        identity,
    #    executive,
    #    thought_watcher,
    #    control_shell,
    #    basic_memory,
#        ear_horn,
    ])
