from launch_ros.actions import Node

listen_for_speech = Node(
        package="psyche",
        executable="listen_for_speech",
        name="the_ear",
        output="screen",
        parameters=[
            {"device_index": 1}
        ]
    )
