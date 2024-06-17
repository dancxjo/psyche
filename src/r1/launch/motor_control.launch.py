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
victus_host = "192.168.0.19"
offboard_host = "192.168.0.3"

def generate_launch_description():    
    # Senses & Faculties
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
    faculties = [proprioception, direct_manoevering]    

    # Procedural Memory
    # A procedure is a node that takes structured commands from an LPU and does something with them
    # We're disabling the movement procedure while the robot is docked [move_directly]
    # And [sing, direct_speech] is disabled because my roommate needs a break, and the robot needs some 
    move_directly = Node(
            package="r1",
            executable="motivate",
            name="motivator",
            output="screen",
        )

    procedures = [move_directly]

    return LaunchDescription([
        *procedures,
        *faculties,
    ])
