from launch_ros.actions import Node

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

