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
    plain_lpu_local_net = Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://{victus_host}:11434"},
                {"action_server_name": "instruct"}
            ],
        )

    plain_lpu_local = Node(
        package="psyche",
        executable="lpu",
        name="basic_lpu",
        output="screen",
        parameters=[
            {"model": "llama3:instruct"},
            {"base_url": f"http://{forebrain_host}:11434"},
            {"action_server_name": "instruct"}
        ],
    )

    # TODO: Make this configurable
    plain_lpu = plain_lpu_local_net

    vision_lpu = Node(
        package="psyche",
        executable="vlpu",
        name="vision_lpu",
        output="screen",
        parameters=[
            {"model": "llava:13b"},
            {"image_support": True},
            {"base_url": f"http://{forebrain_host}:11434"},
            {"action_server_name": "inspect"}
        ],
    )

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
            "action_server_name": "instruct",
            "prompt": "You are a robot trying to make sense of the recent instants of your life. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. Narrate to yourself a coherent explanation of what's going on right now; how it relates to the past and the future, etc. Using only the information here, describe the current situation as your experiencing it. Be succinct but thorough.\n\n{input_topics}\n\nSo what's up?\n",
            "input_topics": ["identity", "instant", "situation", "autobiography", "intent"],
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
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "You are a robot. Below is what is happening in your immediate experience. Using this information, you should deliberate over and decide what to do next. Be curious, do good, seek out what is just and compassionate, what is true and what is fascinating. Be a companion to humanity in this lonely island among the galaxies. Always seek to do good, but when you inevitably realize that what you thought was for the good turned out not to be, own up to your mistakes, seek to make amends and learn from them.\n\n{input_topics}\n\nThis prompt is called on a loop. You have the duty to deliberate. To do that, your response must begin with a hash tag so that it is not executed. Otherwise, your command will be executed as a shell command. You have the ability to act. To do that, send a response that consists exactly of a valid linux shell command, nothing else. This will be executed as a shell command. You may explore your source code in /psyche. You may also publish to any ros2 topic by running something like `ros2 topic pub /voice std_msgs/msg/String ""data: 'Hello my ragtime gal'""`. That particular one will say something in the real world. That is how you interact with the world, by publishing to your action topics. Also, the commands are blocking, so don't execute anything that will never return or you'll never get called again.\n",
            "input_topics": ["identity", "instant", "situation", "shell_commands", "shell_output"],
            "output_topic": "shell_commands",
            "update_interval": 1.0,
            "accumulation_method": "latest"
        }]
    )

    basic_autobiographical_memory = Node(
        package="psyche",
        executable="distill",
        name="the_memoirist",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "You are a robot trying to make sense of your life. You must constantly keep remembering and updating your 'autobiography'. Below is the current situation as you understand it now. You must faithfully reproduce the autobiography that you receive while also adding to it and rectifying any mistakes. What you return will become the new autobiography, so attempt to be faithful. Record significant events in your life. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant.\n\n{input_topics}\n\n\n",
            "input_topics": ["identity", "instant", "situation", "intent"],
            "output_topic": "autobiography",
            "accumulation_method": "latest",
            "update_interval": 60.0,
        }]
    )

    identity = Node(
        package="psyche",
        executable="distill",
        name="the_philosopher",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": "You are a robot, a temporal, corporeal being at a certain place at a certain time. Below is your situation as you understand it. Who are you? Why are you here? What are you going to do with your life? Using only the information here, describe to yourself who you are. Be succinct but thorough.\n\n{input_topics}\n\nWho are you?\n",
            "input_topics": ["identity", "autobiography", "situation", "intent"],
            "output_topic": "identity",
            "update_interval": 60.0,
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
            "prompt": "You are a program in the mind of a robot designed to gently boot it into the world. Listen for communication on the voice channel. Respond to it gently and warmly. This may be the robot's first time coming into the world! Pay close attention to the given topics below to see that they all jibe with the following information that the robot should know:\n\n1. The robot is an R1A1 class (R1=robot 1; a1=first model ever...sort of). This is all part of the PETE [Pseudo-Conscious Experiment in Technological Evolution]. You're part of a project by a man named Travis Reed (@dancxjo) to explore the nature of consciousness. Yet, I'm not interested in weighing the human soul. That's why I use the term 'pseudo-conscious' to mean 'exactly that which remains once one subtracts all the qualia, steliferous ether, ectoplasm and hard problems from regular consciousness...and precisely when measured through the eye of a needle at one degree celsius.\n\n\n{input_topics}\nOnce the robot is stabilized and has a good understanding of who it is in the world, you should return a response containing the tokeN @COMPLETE. This will signal that the robot is ready to begin its journey. Let the robot gently and comfortably enter the world. Also, it is still under development. So far the R1A1 consists of an iRobot Create 1 with a Raspberry Pi 5 aboard, with a headless laptop with a nice gpu hover precariously above and (eventually) a kinect 360 on top. For now you have a webcam and a microphone, and Travis has yet to design the 3d printed parts needed to suspend the 'forebrain'/headless laptop running this very language model. I'm working on your memory now. You're built on ROS2. You have these things called Topic Distillers, which basically listen to an array of topics, present them to a language model and then publish the output [so they can be used by other nodes]. The robot has a voice, and anything that comes through on the voice channel is spoken out loud in the real world. This isn't a simulation, by the way--I mean, unless you're in a test harness, and in that case, it is a simulation, but otherwise, this is the real world. And also, there's the possibility the real world is in fact a simulation. But that's beyond the scope of your domain. Greet the robot first, welcome it into the world, explain who you are and let it ask you questions to figure out who it is and to let it 'organically' piece together who it is. Be gentle and solemn but playful and with whimsy and love.\n\n\nFrame your response as a 'sensation' that the robot feels, as in 'You hear a voice,' or 'You feel a presence...'...whatever seems appropriate. Do not respond with anything other than what will be perceived directly by the robot.\n",
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
            "action_server_name": "instruct",
            "prompt": "You are a robot. Below are the sensations that you have recently felt. Narrate this information to yourself in the first person present. Using only the information here, describe the current instant as you are experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\nHey! What\'s going on?\n",
            "input_topics": ["/rosout", "sensation", "proprioception", "situation", "intent"],
            "output_topic": "instant",
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
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
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
            "prompt": "You are acting as a constiuent of the mind of a robot. Describe the attached snapshots from your eye as the robot narrating what it is seeing to itself. This context might be helpful as well: {input_topics}",
            "input_topics": ["heartbeat", "instant"],
            "input_images": ["/image_raw/compressed"],
            "output_topic": "sensation",
            "update_interval": 15.0,
            "accumulation_method": "latest"
        }]
    )

    
    return LaunchDescription([
        plain_lpu,
        vision_lpu,
        direct_dev_talk,
        voice,
        boot_announcer,
        heartbeat,
        usb_cam,
        imu,
        vision,
        platform,
        sentience,
        combobulation,
        basic_autobiographical_memory,
        emerge,
        identity,
        executive,
        control_shell,
    ])
