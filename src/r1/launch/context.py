from launch_ros.actions import Node

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
