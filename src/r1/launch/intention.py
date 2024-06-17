from launch_ros.actions import Node
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
