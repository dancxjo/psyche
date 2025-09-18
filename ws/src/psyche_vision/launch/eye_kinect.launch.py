"""Launch the Kinect v1 driver alongside a depth-to-point-cloud pipeline.

This launch description remaps the topics produced by the libfreenect-based
`kinect_ros2` component into the shared `/camera` namespace and starts the
`depth_image_proc` point-cloud node so downstream consumers receive organized
XYZRGB data. The setup assumes the original Kinect (v1) sensor and libfreenect.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create a launch description for the eye_kinect service."""

    kinect_driver = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="eye_kinect_driver",
        output="screen",
        remappings=[
            ("image_raw", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("depth/image_raw", "/camera/depth/image_raw"),
            ("depth/camera_info", "/camera/depth/camera_info"),
        ],
    )

    point_cloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb",
        name="eye_kinect_point_cloud",
        output="screen",
        parameters=[{"approximate_sync": True}],
        remappings=[
            ("rgb/image_rect_color", "/camera/image_raw"),
            ("rgb/camera_info", "/camera/camera_info"),
            ("depth/image_rect", "/camera/depth/image_raw"),
            ("depth/camera_info", "/camera/depth/camera_info"),
            ("points", "/camera/depth/points"),
        ],
    )

    return LaunchDescription([kinect_driver, point_cloud])
