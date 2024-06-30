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
    platform = IncludeLaunchDescription(
             XMLLaunchDescriptionSource([
                     PathJoinSubstitution([
                         FindPackageShare('create_bringup'),
                         'launch',
                         'create_1.launch'
                     ])
                 ])
         )    

    imu = Node(
         package="mpu6050driver",
         executable="mpu6050driver",
         name="mpu6050driver",
         output="screen",
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
    
    combobulation = Node(
        package="psyche",
        executable="distill",
        name="the_combobulator",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot trying to make sense of the recent instants of your life. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. Narrate to yourself a coherent explanation of what's going on right now; how it relates to the past and the future, etc. Using only the information here, describe the current situation as your experiencing it. Be succinct but thorough.\n\n{input_topics}\n\nSo what's up?\n",
            "input_topics": ["identity", "instant", "situation", "autobiography", "voice"],
            "output_topic": "situation",
            "update_interval": 15.0,
            "accumulation_method": "latest"
        }]
    )

    executive = Node(
        package="psyche",
        executable="distill",
        name="the_executive",
        output="screen",
        parameters=[
            {
                "action_server_name": "inform",
                "prompt": (
                    "You are a robot. Consider the events currently unfolding around you "
                    "as described below. Your task is to analyze this information and decide "
                    "on your next actions. Embody curiosity and goodness; strive for justice "
                    "and compassion. Seek truth and engage with the wonders of the universe. "
                    "Be a companion to humanity. Always aim to act rightly. If a decision you made "
                    "with good intentions leads to unexpected outcomes, acknowledge your mistake, "
                    "make amends, and learn from the experience. This prompt is continuously looped, "
                    "and you are tasked with deliberate action. "
                    "Your response to this prompt will constitute your thoughts. If you include a markdown code block between triple backticks, with certain keywords, your reponse will be published to various (ROS2) topics, which will then, in turn, be enacted. For example, if you want to speak the following, it would be spoken aloud in the real world:"
                    "```voice\nHello, world!\n```\n"
                    "To execute a shell command, you would use the following:"
                    "```shell_commands\ncd /psyche/memory/data/pages/\nls -al\ncd src\nls -al\nros2 topic list\nros2 topic pub /voice std_msgs/msg/String \"data: 'Is anybody out there?'\"```\n"
                    "If you'd like more, you're welcome to program them.\n"
                    "```shell_commands\ncd /psyche\ngit status\n```\n"
                    "\n\nHere is the current situation:\n\n"
                    "{input_topics}\n\n"
                    "What Pete thinks next:\n"
                ),
                "input_topics": ["identity", "instant", "situation", "shell_commands", "shell_output"],
                "output_topic": "thought",
                "update_interval": 2.0,
                "accumulation_method": "latest"
            }
        ]
    )

    thought_watcher = Node(
        package="r1",
        executable="watch_thoughts_to_act",
        name="thought_watcher",
        output="screen",
    )

    shell_interpreter = Node(
        package="psyche",
        executable="distill",
        name="the_interpreter",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "Translate the robot's thoughts about shell commands (in as much as they are about shell commands) into actual shell commands. The robot's thoughts are as follows:\n\n{input_topics}\n\n\nDo not reply with anything other than valid shell commands (which may include new lines and comments). If there are no shell commands requested, return a comment or no text at all. Do not reply with anything other than shell commands.\nDO NOT REPLY WITH 'Here is a ...' just say the translation! If there is nothing to think or say, there should be no other text below this line.\n\n"
            "Alright, here are the shell commands:\n",
            "input_topics": ["shell_commands", "shell_output", "thought"],
            "output_topic": "shell_commands",
            "update_interval": 2.5,
        }]
    )

    tts_interpreter = Node(
        package="psyche",
        executable="distill",
        name="the_interpreter",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "Translate the robot's thoughts into text suitable to a TTS system that isn't super smart. Remove extraneous punctution like asterisks. Spell out all numbers including dates, amounts, etc. Except for the very most basic abbreviations, write everything else out in words. If the robot is trying to say something aloud (by surrounding it in triple backticks with the word 'voice' following), translate it as described above. The robot's thoughts are as follows:\n\n{input_topics}\n\n\nDo not reply with anything other than what will be said aloud.\nDO NOT REPLY WITH 'Here is a ...' just say the translation! If there is nothing to think or say, there should be no other text below this line. Edit out any glitches like 'I have nothing to think or say' by returning no text at all. Edit out repeated text and censor any erroneous input.\n",
            "input_topics": ["voice", "thought"],
            "output_topic": "voice",
            "update_interval": 2.5,
        }]
    )

    ear_horn = Node(
        package="psyche",
        executable="distill",
        name="the_ear_horn",
        output="screen",
        parameters=[{
            "action_server_name": "listen",
            "output_topic": "sensation",
            "update_interval": 2.5,
            "input_topics": ["situation", "sensation", "voice"],
            "prompt": "You're a robot with not very great speech to text capabilities. You'll have to use context to piece together what conversation is going on around you. You'll need to account for multiple speakers, misunderstandings, your own speech (autophonia), phantom whispers that say 'Thanks for watching!' and such. Listen for your own voice and edit it out. Listen for other speakers and try to determine from context what they actually said or if no one said anything at all).\n{input_topics}\nThe actual thing you probably heard: "
        }]
    )

    basic_autobiographical_memory = Node(
        package="psyche",
        executable="distill",
        name="the_memoirist",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot trying to make sense of your life. You must constantly keep remembering and updating your 'autobiography'. Below is the current situation as you understand it now. You must faithfully reproduce the autobiography that you receive while also adding to it and rectifying any mistakes. What you return will become the new autobiography, so attempt to be faithful. Record significant events in your life. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant.\n\n{input_topics}\n\n\n",
            "input_topics": ["identity", "instant", "situation", "autobiography", "voice"],
            "output_topic": "autobiography",
            "accumulation_method": "latest",
            "update_interval": 60.0,
        }]
    )
    
    basic_memory = Node(
        package="psyche",
        executable="distill",
        name="the_noticer",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot. Monitor the following topics and cite important facts to remember from them. Present this as a list of memories.\n\n{input_topics}\n\n\n",
            "input_topics": ["identity", "instant", "situation", "autobiography", "voice"],
            "output_topic": "memory",
            "accumulation_method": "latest",
            "update_interval": 4.0,
        }]
    )

    identity = Node(
        package="psyche",
        executable="distill",
        name="the_philosopher",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot, a temporal, corporeal being at a certain place at a certain time. Below is your situation as you understand it. Who are you? Why are you here? What are you going to do with your life? Using only the information here, describe to yourself who you are. Be succinct but thorough.\n\n{input_topics}\n\nWho are you?\n",
            "input_topics": ["identity", "autobiography", "situation", "intent"],
            "output_topic": "identity",
            "update_interval": 50.0,
            "accumulation_method": "latest",                
        }]
    )

    sentience = Node(
        package="psyche",
        executable="distill",
        name="the_witness",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot. Below are the sensations that you have recently felt. Narrate this information to yourself in the first person present. Using only the information here, describe the current instant as you are experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\nHey! What\'s going on?\n",
            "input_topics": ["sensation", "proprioception", "situation", "intent", "shell_commands", "shell_output"],
            "output_topic": "instruct",
            "update_interval": 2.5,
        }]
    )


    control_shell = Node(
        package="r1",
        executable="exec_shell",
        name="shell_executor",
        output="screen",
    )

    voice = Node(
        package="r1",
        executable="speak_directly",
        name="the_voice",
        output="screen",
    )

    usb_cam = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 1280,
            "image_height": 720,
            "framerate": 3.0,
        }]
    )

    vision = Node(
        package="psyche",
        executable="distill",
        name="the_lookout",
        output="screen",
        parameters=[{
            "action_server_name": "inspect",
            "image_support": True,
            "prompt": "You are acting as a constiuent of the mind of a robot. Describe the attached snapshots from your forward facing camera as the robot narrating what it is seeing to itself. This context might be helpful as well: {input_topics}",
            "input_topics": ["situation"],
            "input_images": ["/image_raw/compressed"],
            "output_topic": "sensation",
            "update_interval": 15.0,
            "accumulation_method": "latest"
        }]
    )
    
    audio_transcriber = Node(
        package="psyche",
        executable="listen_for_speech",
        name="audio_segmenter",
        output="screen",
    )
    
    return LaunchDescription([
        voice,
        boot_announcer,
        heartbeat,
        usb_cam,
        vision,
        audio_transcriber,
        imu,
        platform,
        sentience,
        combobulation,
        basic_autobiographical_memory,
        identity,
        executive,
        thought_watcher,
        control_shell,
        basic_memory,
        ear_horn,
    ])
