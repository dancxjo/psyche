from launch_ros.actions import Node

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

control_shell = Node(
        package="r1",
        executable="exec_shell",
        name="shell_executor",
        output="screen",
    )
