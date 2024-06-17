from launch_ros.actions import Node

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
