from launch_ros.actions import Node

usb_cam = Node(
    package="usb_cam",
    executable="usb_cam_node_exe",
    name="usb_cam",
    output="screen",
    parameters=[{
        "video_device": "/dev/video0",
        "image_width": 1280,
        "image_height": 720,
        "framerate": 3.0,
    }]
)

vision = Node(
        package="psyche",
        executable="distill",
        name="the_lookout",
        output="screen",
        parameters=[{
            "action_server_name": "inspect",
            "image_support": True,
            "prompt": "You are acting as a constiuent of the mind of a robot. Describe the attached snapshots from your eye as the robot narrating what it is seeing to itself. This context might be helpful as well: {input_topics}",
            "input_topics": ["heartbeat", "instant"],
            "input_images": ["/image_raw/compressed"],
            "output_topic": "sensation",
            "update_interval": 15.0,
            "accumulation_method": "latest"
        }]
    )
