from launch_ros.actions import Node

voice = Node(
        package="r1",
        executable="speak_directly",
        name="the_voice",
        output="screen",
    )
