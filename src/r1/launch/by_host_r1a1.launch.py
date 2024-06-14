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

def generate_launch_description():
    return LaunchDescription([
        # Start streaming the voice so we can hear where we are in the boot sequence
        # The outer container host should be listening to this and playing it
        Node(
            package="r1",
            executable="speak_directly",
            name="the_voice",
            output="screen",
        ),
        # Announce the boot sequence
        Node(
            package="r1",
            executable="announce_boot",
            name="boot_announcer",
            output="screen",
        ),
        
        # Start the LLMS
        Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": f"http://{forebrain_host}:11434"},
                {"action_server_name": "instruct"}
            ],
        ),
        Node(
            package="psyche",
            executable="vlpu",
            name="vision_lpu",
            output="screen",
            parameters=[
                {"model": "llava:13b"},
                {"base_url": f"http://{forebrain_host}:11434"},
                {"action_server_name": "inspect"}
            ],
        ),
        
        Node(
            package="psyche",
            executable="heartbeat",
            name="heartbeat",
            output="screen",
            parameters=[
                {"processing_notes": "You are under development. This may feel funny."},
                {"update_interval": 1.0},
            ],
        ),
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
            package="mpu6050driver",
            executable="mpu6050driver",
            name="mpu6050driver",
            output="screen",
        ),

        Node(
            package="r1",
            executable="play_song",
            name="song_player",
            output="screen",
        ),
        Node(
            package="psyche",
            executable="distill",
            name="the_gourmet",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the gourmet') of the mind of a robot. Below are the readings from the robot's power topics. Narrate this information for yourself (i.e. the whole robot, not just this constituent) in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Be succinct. Describe them in terms of sensations of hunger, satiety. Recommend actions like docking and undocking when the time is appropriate.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["battery/capacity", "battery/charge", "battery/charge_ratio", "battery/temperature", "battery/charging_state", "battery/current", "battery/voltage"],
                "output_topic": "sensation",
                "update_interval": 30.0,
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_proprioceptor",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the proprioceptor') of the mind of a robot. Below are the readings from the robot's proprioceptive topics. Narrate this information for yourself (i.e. the whole robot, not just this constituent) in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Attempt to extrapolate motion, acceleration, etc. Make sure to pay close attention to changing values in the imu, the bumper, the buttons, etc.Do not describe anything other than the sensations presented here. Attempt to capture the feelings in the robot's body. Recommend actions like obstacle avoidance when the time is appropriate.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["imu", "bumper", "cliff", "clean_button", "day_button", "hour_button", "minute_button", "dock_button", "spot_button", "ir_omni", "joint_states", "odom", "wheeldrop", "/tf"],
                "output_topic": "proprioception",
                "update_interval": 1.0, # This should allow for almost instantaneous updates
                "accumulation_method": "queue",
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_athlete",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Specifically, you are a language model serving as a constituent ('the athlete') of the mind of a robot. Below are the readings from the robot's proprioceptive topics. Control the robot's velocity as necessary. Specify your command as an array of between 1 and 5 twists. Each twist in the array is an object with up to five fields:\n* `reasoning`: a string that explains why you issued this command\n* `duration`: floating point number of seconds to continuously pulse\n* `rate`: integer frequency of pulsing in hertz (default 20)\n* `linear` an object with float32 values for the keys x, y and z\n* `angular` the same type. This expresses velocity in free space broken into its linear and angular parts.

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
        ),

        Node(
            package="r1",
            executable="motivate",
            name="motivator",
            output="screen",
        ),
        

        Node(
            package="psyche",
            executable="distill",
            name="the_witness",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the witness') of the mind of a robot. Below are the sensations that the robot has recently felt. Narrate this information for yourself (i.e. the whole robot, not just this constituent) in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "sensation", "proprioception", "situation", "intent"],
                "output_topic": "instant",
                "update_interval": 2.5,
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_combobulator",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the combobulator') of the mind of a robot. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. Narrate this information for yourself (i.e. the whole robot, not just this constituent) in the first person present (from the entire robot's perspective). Using only the information here, describe the current situation as the robot is experiencing it. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation", "autobiography", "intent"],
                "output_topic": "situation",
                "update_interval": 15.0,
                "accumulation_method": "latest"
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_executive",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the executive') of the mind of a robot. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. You must use this information to decide what to do next (as the robot itself). Your intentions will become the robot's actions, but you'll need to keep a close eye on things and adjust your plans as you move forward. Maintain short, medium and long-term goals. Decide on priorities and focuses and direct attention.\n\n{input_topics}\n\nExplain your intentions simply, as a list of user stories. As a _____ I want to _____ so that _____.\n",
                "input_topics": ["identity", "instant", "situation", "twists", "intent"],
                "output_topic": "intent",
                "update_interval": 1.0,
                "accumulation_method": "latest"
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_memoirist",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the memoirist') of the mind of a robot. Below is the situation as you understand it. Your job is to maintain the autobiography. You must faithfully reproduce the autobiography that you receive while also adding to it and rectifying any mistakes. What you return will become the new autobiography, so attempt to be faithful. Record significant events in the life of the robot. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant. Use the first person to describe this as the robot.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation", "intent"],
                "output_topic": "autobiography",
                "accumulation_method": "latest",
                "update_interval": 60.0,
            }]
        ),
        
        Node(
            package="psyche",
            executable="distill",
            name="the_philosopher",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the philosopher') of the mind of a robot. Below is the situation as you understand it. Explain to the robot in the first person present (as the robot itself) who it is. Using only the information here, describe to the robot who it experiences itself as. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "autobiography", "situation", "intent"],
                "output_topic": "identity",
                "update_interval": 60.0,
                "accumulation_method": "latest",                
            }]
        ),
        
        Node(
            package="psyche",
            executable="distill",
            name="the_mime",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are a robot. Specifically, you are a language model serving as a constituent ('the mime') of the mind of a robot. Below is the situation as you understand it. In one or two emoji (as unicode characters) and no other words or symbols, represent the current emotional state of the robot.\n\n{input_topics}\n\nA few emoji characters and nothing else:\n",
                "input_topics": ["identity", "instant", "situation", "intent"],
                "output_topic": "feeling",
                "update_interval": 1.0,
                "accumulation_method": "latest",                
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_musician",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Specifically, you are a language model serving as a constituent ('the musician') of the mind of a robot. Below is the situation as you understand it and how you're feeling about it. Represent the robot's current feelings in song. You must return valid JSON in the following format and nothing more! If you have no appropriate song to play, simply return the token "null". Otherwise, specify your song as an array of between 1 and 16 notes. Each note in the array is an object with two fields:\n* `note` an unsigned 8-bit integer defined by the MIDI note numbering scheme (notes outside the range of [31-127] are rest notes)\n* `duration` a positive floating point number representing the duration of the note in seconds\nTry to use expressive and familiar tunes, but feel free to improvise.\n{input_topics}\n\nReminder:\nOnly respond with the actual JSON with no delimiting markdown codefences or otherwise. Return no other response.\nYour song:\n""",
                "input_topics": ["identity", "feeling", "instant", "situation", "intent"],
                "output_topic": "song",
                "update_interval": 5.0,
                "accumulation_method": "latest",                
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_shell_interactor",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Specifically, you are a language model serving as a constituent ('the shell interactor') of the mind of a robot. Listen to the output of other mental constiuents and help them out by running and interpretting shell commands. This shell is running on the same computer as your code is running. (Feel free to explore it in /psyche.) You have access to publish to any topic by running the appropriate ROS2 messages from the command line. To communicate with the other parts of the mind, you might try publishing a std_msgs.String to /sensation.\n{input_topics}\n\n$\n""",
                "input_topics": ["identity", "feeling", "instant", "situation", "intent", "shell_output"],
                "output_topic": "shell_commands",
                "update_interval": 1.0,
                "accumulation_method": "latest",                
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_spokesman",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are a robot. Below is the situation as you understand it and how you're feeling about it. Use the information to speak if necessary.
                
                {input_topics}
                
                Be careful not to speak too often so as to overwhelm your interlocuter. Only say one sentence at a time as it will allow more frequent checking. If you have nothing to say, your response should include no text at all. Otherwise, it will be read verbatim in the real world. Spell out all words including numbers (years, amounts, etc.) so the TTS doesn't struggle.""",
                "input_topics": ["identity", "feeling", "instant", "situation"],
                "output_topic": "voice",
                "update_interval": 3.0,
                "accumulation_method": "latest",                
            }]
        ),

   ])
