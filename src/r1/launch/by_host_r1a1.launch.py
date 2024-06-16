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

forebrain_host = "192.168.0.7"
offboard_host = "192.168.0.3"

def generate_launch_description():
    boot_announcer = Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
            parameters=[
                {'boot_topics': [
                    'voice', 
                    'sensation', 
                    'instant', 
                    'situation', 
                    'intent'
                ]}
            ]
        )

    # Language processors
    plain_lpu = Node(
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
    offboard_lpu = Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://{offboard_host}:11434"},
                {"action_server_name": "instruct"}
            ],
        )
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

    processors = [vision_lpu, plain_lpu, offboard_lpu]

    # Sensors
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
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
    listen_for_speech = Node(
            package="psyche",
            executable="listen_for_speech",
            name="the_ear",
            output="screen",
        )
    imu1 = Node(
            package="mpu6050driver",
            executable="mpu6050driver",
            name="mpu6050driver",
            output="screen",
        )
    #sensors = [platform, listen_for_speech, imu1]
    sensors = [usb_cam]
    
    # Senses & Faculties
    heartbeat = Node(
            package="psyche",
            executable="heartbeat",
            name="heartbeat",
            output="screen",
            parameters=[
                {"processing_notes": "You are under development. This may feel funny. If you hear music playing, it's probably your innate singing--it matches your mood. You may also hear spoken words. You should also hear your own voice. You may also occasionally receive descriptions of what you are seeing. You should also receive IMU, bumper and other bodily readings as sensations."},
                {"update_interval": 60.0},
            ],
        )
    power_management = Node(
            package="psyche",
            executable="distill",
            name="the_gourmet",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Below are the readings from your power topics. Narrate this information to yourself in the first person present. Using only the information here, describe the current power situation. Describe it in terms of sensations of hunger, satiety. If you're charging, explain to yourself that you may send out movement or song commands, but while you are docked, they will not be executed. Recommend actions like docking and undocking when the time is appropriate. Tell if you are docked or undocked, if your power levels are rising or falling, etc.\n\n{input_topics}\n\n\n",
                "input_topics": ["battery/capacity", "battery/charge", "battery/charge_ratio", "battery/temperature", "battery/charging_state", "battery/current", "battery/voltage"],
                "output_topic": "sensation",
                "update_interval": 30.0,
            }]
        )
    proprioception = Node(
            package="psyche",
            executable="distill",
            name="the_proprioceptor",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Below are the readings from the robot's proprioceptive topics. Narrate this information to yourself. Get rid of unnecessary detail. Attempt to extrapolate motion, acceleration, etc. Make sure to pay close attention to changing values in the IMU, the bumper, the buttons, etc. Do not describe anything other than the sensations presented here. Present these as feelings in your body. Recommend actions like obstacle avoidance when the time is appropriate.\n\n{input_topics}\n\n\n",
                "input_topics": ["imu", "bumper", "cliff", "clean_button", "day_button", "hour_button", "minute_button", "dock_button", "spot_button", "ir_omni", "joint_states", "odom", "wheeldrop", "/tf"],
                "output_topic": "proprioception",
                "update_interval": 1.0, # This should allow for almost instantaneous updates
                "accumulation_method": "queue",
            }]
        )
    direct_manoevering = Node(    # Senses
            package="psyche",
            executable="distill",
            name="the_athlete",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Below are the readings from your proprioceptive topics. Control your velocity as necessary. Specify your command as an array of between 1 and 5 twists. Each twist in the array is an object with up to five fields:\n* `reasoning`: a string that explains why you issued this command\n* `duration`: floating point number of seconds to continuously pulse\n* `rate`: integer frequency of pulsing in hertz (default 20)\n* `linear` an object with float32 values for the keys x, y and z\n* `angular` the same type. This expresses velocity in free space broken into its linear and angular parts.

                linear.x\t(+)\tMove forward (m/s)
                \t\t(-)\tMove backward (m/s)
                angular.z\t(+)\tRotate counter-clockwise (rad/s)
                \t\t(-)\tRotate clockwise (rad/s)
                Velocity limits:
                -0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25\nYou may also pause by sending an object with a value for the key pause_time. This will maintain the velocity.\n{input_topics}\n\nReminder:\nOnly respond with the actual JSON with no delimiting markdown codefences or otherwise. Your commands:\n""",
                "input_topics": ["proprioception", "intent", "situation", "feeling"],
                "output_topic": "twists",
                "update_interval": 1.0, # Careful not to send commands that interfere with themselves
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
                "input_topics": ["/rosout", "identity", "sensation", "proprioception", "situation", "intent"],
                "output_topic": "instant",
                "update_interval": 2.5,
            }]
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
    intent = Node(
            package="psyche",
            executable="distill",
            name="the_intender",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. You have self-driven intentions that you must identify (and/or generate). Below are the the most recent instants in your life, along with the current situation as you have been understanding it. You must use this information to decide what to do next. Certain other parts of your mind will attempt to enact your commands, but you'll need to keep a close eye on things and adjust your plans as you move forward. Maintain short, medium and long-term goals. Decide on priorities and focuses and direct attention.\n\n{input_topics}\n\nIssue only one sentence in the form of: As a _____ I want to _____ so that _____. NOTHING ELSE, just one sentence.\n",
                "input_topics": ["identity", "instant", "situation", "twists", "intent"],
                "output_topic": "intent",
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
    os_shell = Node(
        package="psyche",
        executable="distill",
        name="the_shell_interactor",
        output="screen",
        parameters=[{
            "action_server_name": "instruct",
            "prompt": """You are a robot. You have access to the equivalent of a shell on the Linux-running computer that is your own brain. Consider the following situation you're in and provide yourself with assistance by executing any series of commands. Be careful...this is your own code. (Feel free to explore it in /psyche.)\n{input_topics}\n\nReturn *only* one command with no other text, just the command. If you have no command, reply with nothing. Try some of these: ls /psyche ; ls /psyche/src ; cat /psyche/README.md. Only be truthful, and be helpful when you can be. When there's nothing you can do to help immediately, use `tree`, `ls`, `cat`, etc. resourcefully to discover more about how your mind works and how to make it better and/or experience it. You are resourceful, truthful and honest. Remember: One command or less and nothing else!\n$ """,
            "input_topics": ["identity", "feeling", "instant", "situation", "intent", "shell_commands", "shell_output"],
            "output_topic": "shell_commands",
            "update_interval": 1.0,
            "accumulation_method": "latest",                
        }]
    )
    speech = Node(
            package="psyche",
            executable="distill",
            name="the_spokesman",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Below is the situation as you understand it and how you're feeling about it. Use the information to speak if necessary.
                
                {input_topics}
                
                Be careful not to speak too often so as to overwhelm your interlocuter. Only say one sentence at a time as it will allow more frequent checking. If you have nothing to say, your response should include no text at all. Otherwise, it will be read verbatim in the real world. Spell out all words including numbers (years, amounts, etc.) so the TTS doesn't struggle. Include *nothing* but the text to say. Do not include symbols other than commas, parentheses, exclamation marks and question marks. Do not say that you are passing or that you are done. Just say the text. If you have nothing to say, return an empty string.""",
                "input_topics": ["identity", "feeling", "instant", "situation"],
                "output_topic": "voice",
                "update_interval": 2.0,
                "accumulation_method": "latest",                
            }]
        ),

    # Temporarily disabling some faculties while the robot is docked
    # [direct_manoevering, innate_musicality]
    faculties = [heartbeat]
    
    #vision, heartbeat, power_management, proprioception, sentience, combobulation, intent, basic_autobiographical_memory, identity, os_shell, speech]
    
    # Procedural Memory
    sing = Node(
        package="r1",
        executable="play_song",
        name="song_player",
        output="screen",
    )
    
    # Disabling while the robot is docked
    move_directly = Node(
            package="r1",
            executable="motivate",
            name="motivator",
            output="screen",
        )
    control_shell = Node(
            package="r1",
            executable="exec_shell",
            name="shell_executor",
            output="screen",
        )
    direct_speech = Node(
            package="r1",
            executable="speak_directly",
            name="the_voice",
            output="screen",
        )

    # A procedure is a node that takes structured commands from an LPU and does something with them
    # We're disabling the movement procedure while the robot is docked [move_directly]
    # And [sing, direct_speech] is disabled because my roommate needs a break, and the robot needs some expression
    procedures = [direct_speech, control_shell]

    return LaunchDescription([
        boot_announcer,
        *processors,
        *sensors,
        *procedures,
        *faculties,
    ])
