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
    llama3 = Node(
            package="psyche",
            executable="ollama_lpu",
            name="llama3_on_victus",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://forebrain.local:11434"},
                {"action_server_name": "instruct"}
            ],
        )

    codellama = Node(
            package="psyche",
            executable="ollama_lpu",
            name="codellama_on_forebrain",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://forebrain.local:11434"},
                {"action_server_name": "code"}
            ],
        )

    llava = Node(
        package="psyche",
        executable="vlpu",
        name="llava_on_forebrain",
        output="screen",
        parameters=[
            {"model": "llava:13b"},
            {"image_support": True},
            {"base_url": f"http://victus.local:11434"},
            {"action_server_name": "inspect"}
        ],
    )

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
        parameters=[{
            "action_server_name": "code",
        }]
    )
    
    usb_cam = Node(
       package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 800,
            "image_height": 600,
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
    
    executive = Node(
        package="psyche",
        executable="executive",
        name="the_executive",
        output="screen",
    )
    
    thought_watcher = Node(
        package="r1",
        executable="watch_thoughts_to_act",
        name="thought_watcher",
        output="screen",
    )
    
    return LaunchDescription([
        llama3,
        llava,
        codellama,
        voice,
        boot_announcer,
        heartbeat,
        usb_cam,
        vision,
        audio_segmenter,
        audio_transcriber,
        sentience,
        graph_memory,
        thought_watcher,
        executive,
    ])
