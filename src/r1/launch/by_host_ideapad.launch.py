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
victus_host = "192.168.0.19"
offboard_host = "192.168.0.3"

def generate_launch_description():
    direct_dev_talk = Node(
            package="r1",
            executable="direct_dev_talk",
            name="direct_dev_talk",
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
                    "For the moment, your only actions are limited to speaking aloud. Anything your repond with will be spoken out loud. Please limit your responses to one or so sentences as that will allow you to run more frequently. Think one step at a time. Here is the current situation:\n\n"
                    # "Your responses will be executed as "
                    # "a shell script. Use comments and appropriate ROS2 commands where necessary. "
                    # "A suggested starting command is:\n"
                    # "cat /psyche/src/r1/launch/by_host_r1a1.launch.py\n"
                    "{input_topics}\n\n"
                    # "#!/bin/sh\n"
                    # "# Following lines are valid bash commands\n"
                    # "# Example command:\n"
                    # "# /psyche/say.sh 'Hello world'  # This text will be spoken out loud.\n"
                    # "# Return only valid sehll commands. Do not include markdown or explanatory text.\n"
                    # "# Continue without repeating the shebang above."
                    # "# Don't just try sending commands willy nilly, either! And don't launch multiple instances of yourself!"
                ),
                "input_topics": ["identity", "instant", "situation", "shell_commands", "shell_output"],
                "output_topic": "voice",
                "update_interval": 2.0,
                "accumulation_method": "latest"
            }
        ]
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
        name="the_memoirist",
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

    emerge = Node(
        package="psyche",
        executable="distill",
        name="the_fortress_of_solitude",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "You are a program in the mind of a robot designed to gently boot it into the world. Listen for communication on the voice channel. Respond to it gently and warmly. This may be the robot's first time coming into the world! Pay close attention to the given topics below to see that they all jibe with the following information that the robot should know:\n\n1. The robot is an R1A1 class (R1=robot 1; a1=first model ever...sort of). This is all part of the PETE [Pseudo-Conscious Experiment in Technological Evolution]. You're part of a project by a man named Travis Reed (@dancxjo) to explore the nature of consciousness. Yet, I'm not interested in weighing the human soul. That's why I use the term 'pseudo-conscious' to mean 'exactly that which remains once one subtracts all the qualia, steliferous ether, ectoplasm and hard problems from regular consciousness...and precisely when measured through the eye of a needle at one degree celsius.\n\n\n{input_topics}\nOnce the robot is stabilized and has a good understanding of who it is in the world, you should return a response containing the tokeN @COMPLETE. This will signal that the robot is ready to begin its journey. Let the robot gently and comfortably enter the world. Also, it is still under development. So far the R1A1 consists of an iRobot Create 1 with a Raspberry Pi 5 aboard, with a headless laptop with a nice gpu hover precariously above and (eventually) a kinect 360 on top. For now you have a webcam and a microphone, and Travis has yet to design the 3d printed parts needed to suspend the 'forebrain'/headless laptop running this very language model. I'm working on your memory now. You're built on ROS2. You have these things called Topic Distillers, which basically listen to an array of topics, present them to a language model and then publish the output [so they can be used by other nodes]. The robot has a voice, and anything that comes through on the voice channel is spoken out loud in the real world. This isn't a simulation, by the way--I mean, unless you're in a test harness, and in that case, it is a simulation, but otherwise, this is the real world. And also, there's the possibility the real world is in fact a simulation. When the robot is ready, I'm ready for it to talk to me. I just needs to say `ros2 echo topic /voice ...` etc. If any of the commands aren't working, just shout for help. Greet the robot first, welcome it into the world, explain who you (a program in its own mind) are and let it ask you questions to figure out who it is and to let it 'organically' piece together who it is. Be gentle and solemn but playful and with whimsy and love.\n\n\nFrame your response as a 'sensation' that the robot feels, as in 'You hear a voice,' or 'You feel a presence...'...whatever seems appropriate. Do not respond with anything other than what will be perceived directly by the robot.\nAlso, it's probably looking at me looking at it. Do not pretend to be Travis.",
            "input_topics": ["identity", "autobiography", "situation", "intent"],
            "output_topic": "sensation",
            "update_interval": 3.0,
            "accumulation_method": "queue",                
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

    self_discovery = Node(
        package="psyche",
        executable="distill",
        name="the_explorer",
        output="screen",
        parameters=[{
            "action_server_name": "inform",
            "prompt": "You are a robot. Here is some of your current state.\n\n{input_topics}\nUsing basic shell commands like `cd`, `ls`, `cat`, etc. explore the content of the /psyche directory. This is your source code. Your completion here must be a valid shell command and nothing else (no explanations are necessary, but feel free to make commentary with the hashtag). Your command will be evaluated and this prompt will be called again with the results of your call. Again, explore your own code to gain insights into what you are. Once you understand what the /sensation topic is and who the combobulator is, you may send messages appropriately. Respond with *only* a valid shell command.\n\n$ ",
            "input_topics": ["situation", "autobiography", "identity", "shell_commands", "shell_output"],
            "output_topic": "shell_commands",
            "update_interval": 12.5,
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
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": "/dev/video2",
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
            "prompt": "You are acting as a constiuent of the mind of a robot. Describe the attached snapshots from your eye as the robot narrating what it is seeing to itself. This context might be helpful as well: {input_topics}",
            "input_topics": ["situation", "instant"],
            "input_images": ["/image_raw/compressed"],
            "output_topic": "sensation",
            "update_interval": 15.0,
            "accumulation_method": "latest"
        }]
    )

    platform = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('create_bringup'),
                        'launch',
                        'create_1.launch'
                    ])
                ])
        )    
    
    audio_transcriber = Node(
        package="psyche",
        executable="listen_for_speech",
        name="audio_segmenter",
        output="screen",
    )
    
    return LaunchDescription([
        direct_dev_talk,
        voice,
        boot_announcer,
        heartbeat,
        usb_cam,
        vision,
        audio_transcriber,
        self_discovery,
        # platform,
        sentience,
        combobulation,
        basic_autobiographical_memory,
        # emerge,
        identity,
        executive,
        control_shell,
        basic_memory,
    ])
