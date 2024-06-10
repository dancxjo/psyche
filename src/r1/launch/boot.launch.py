from struct import pack
from sys import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

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
            package="r1",
            executable="play_song",
            name="song_player",
            output="screen",
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
        
        Node(
            package="psyche",
            executable="lpu",
            name="basic_lpu",
            output="screen",
            parameters=[
                {"model": "llama3:instruct"},
                {"base_url": "http://192.168.0.133:11434"},
                # { "model_type": "openai"},
                # { "model": "gpt-4"},
                {"action_server_name": "instruct"}
            ],
        ),
        
        Node(
            package="psyche",
            executable="lpu",
            name="vision_capable_lpu",
            output="screen",
            parameters=[
                {"model": "llava"},
                {"base_url": "http://192.168.0.133:11434"},
                {"action_server_name": "llava"}
            ],
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_gourmet",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent ('the gourmet') of the mind of a robot. Below are the readings from the robot's power topics. Narrate them for the robot in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Be succinct. Describe them in terms of sensations of hunger, satiety. Recommend actions like docking and undocking when the time is appropriate.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["battery/capacity", "battery/charge", "battery/charge_ratio", "battery/temperature", "battery/charging_state", "battery/current", "battery/voltage"],
                "output_topic": "sensation",
                "update_interval": 30.0,
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_cartographer",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent ('the cartographer') of the mind of a robot. Below are the readings from the robot's proprioceptive topics. Narrate them for the robot in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Attempt to capture the feelings in the robot's body. Recommend actions like obstacle avoidance when the time is appropriate.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["bumper", "cliff", "clean_button", "day_button", "hour_button", "minute_button", "dock_button", "spot_button", "ir_omni", "joint_states", "odom", "wheeldrop", "/tf"],
                "output_topic": "sensation",
                "update_interval": 1.0, # This should allow for almost instantaneous updates
            }]
        ),

        Node(
            package="psyche",
            executable="distill",
            name="the_athlete",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": """You are serving as a constituent ('the athlete') of the mind of a robot. Below are the readings from the robot's proprioceptive topics. Control the robot's velocity as necessary. Specify your command as an array of between 1 and 5 twists. Each twist in the array is an object with up to five fields:\n* `reasoning`: a string that explains why you issued this command\n* `duration`: floating point number of seconds to continuously pulse (default 5.0)\n* `rate`: integer frequency of pulsing in hertz (default 20)\n* `linear` an object with float32 values for the keys x, y and z\n* `angular` the same type. This expresses velocity in free space broken into its linear and angular parts.

                linear.x\t(+)\tMove forward (m/s)
                \t\t(-)\tMove backward (m/s)
                angular.z\t(+)\tRotate counter-clockwise (rad/s)
                \t\t(-)\tRotate clockwise (rad/s)
                Velocity limits:
                -0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25\nYou may also pause by sending an object with a value for the key pause_time. This will maintain the velocity.\n{input_topics}\n\nReminder:\nOnly respond with the actual JSON with no delimiting markdown codefences or otherwise. Your commands:\n""",
                "input_topics": ["sensation", "situation", "instant", "bumper", "cliff", "clean_button", "day_button", "hour_button", "minute_button", "dock_button", "spot_button", "ir_omni", "joint_states", "odom", "wheeldrop", "/tf"],
                "output_topic": "twists",
                "update_interval": 1.0, # Careful not to send commands that interfere with themselves
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
                "prompt": "You are serving as a constituent ('the witness') of the mind of a robot. Below are the sensations that the robot has recently felt. Narrate them for the robot in the first person present (from the entire robot's perspective). Using only the information here, describe the current instant as the robot is experiencing it. Do not describe anything other than the sensations presented here. Be succinct.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "sensation"],
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
                "prompt": "You are serving as a constituent ('the combobulator') of the mind of a robot. Below are the the most recent instants in the robot's life, along with the current situation as you have been understanding it. Narrate them for the robot in the first person present (from the entire robot's perspective). Using only the information here, describe the current situation as the robot is experiencing it. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation", "autobiography"],
                "output_topic": "situation",
                "update_interval": 15.0,
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
                "prompt": "You are serving as a constituent ('the memoirist') of the mind of a robot. Below is the situation as you understand it. Your job is to record significant events in the life of the robot. You must not remember every little detail, but you may choose to remember specific sensory details when they are relevant. If nothing of importance is happening, or if you've already recorded this event, respond with the token $$$PASS$$$. Use the first person to describe this as the robot.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "instant", "situation"],
                "output_topic": "autobiography",
                "accumulation_method": "latest",
                "update_interval": 60.0 * 5,
            }]
        ),
        
        Node(
            package="psyche",
            executable="distill",
            name="the_philosopher",
            output="screen",
            parameters=[{
                "action_server_name": "instruct",
                "prompt": "You are serving as a constituent ('the philosopher') of the mind of a robot. Below is the situation as you understand it. Explain to the robot in the first person present (as the robot itself) who it is. Using only the information here, describe to the robot who it experiences itself as. Be succinct but thorough.\n\n{input_topics}\n\nInterpretation:\n",
                "input_topics": ["identity", "autobiography", "situation"],
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
                "prompt": "You are serving as a constituent ('the mime') of the mind of a robot. Below is the situation as you understand it. In one emoji and no other words or symbols, represent the current emotional state of the robot.\n\n{input_topics}\n\nOne emoji and nothing else:\n",
                "input_topics": ["identity", "instant", "situation"],
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
                "prompt": """You are serving as a constituent ('the musician') of the mind of a robot. Below is the situation as you understand it and how you're feeling about it. Represent the robot's current feelings in song. You must return valid JSON in the following format and nothing more! If you have no appropriate song to play, simply return the token "null". Otherwise, specify your song as an array of between 1 and 16 notes. Each note in the array is an object with two fields:\n* `note` an unsigned 8-bit integer defined by the MIDI note numbering scheme (notes outside the range of [31-127] are rest notes)\n* `duration` a positive floating point number representing the duration of the note in seconds\nTry to use expressive and familiar tunes, but feel free to improvise.\n{input_topics}\n\nReminder:\nOnly respond with the actual JSON with no delimiting markdown codefences or otherwise. Return no other response.\nYour song:\n""",
                "input_topics": ["identity", "feeling", "instant", "situation"],
                "output_topic": "song",
                "update_interval": 15.0,
                "accumulation_method": "latest",                
            }]
        ),

        # Node(
        #     package="psyche",
        #     executable="distill",
        #     name="the_spokesman",
        #     output="screen",
        #     parameters=[{
        #         "action_server_name": "instruct",
        #         "prompt": """You are serving as a constituent ('the spokesman') of the mind of a robot. Below is the situation as you understand it and how you're feeling about it. Use the information to speak on behalf of the robot.
                
        #         {input_topics}
                
        #         Make sure not to speak too often. Only speak when the robot has something important to say. And even then speak only one sentence at a time. Speak as the robot. If you include the token $$$PASS$$$ in your response, it will not be spoken aloud, otherwise it will be read verbatim in the real world.""",
        #         "input_topics": ["identity", "feeling", "instant", "situation"],
        #         "output_topic": "voice",
        #         "update_interval": 3.0,
        #         "accumulation_method": "latest",                
        #     }]
        # ),
    ])
