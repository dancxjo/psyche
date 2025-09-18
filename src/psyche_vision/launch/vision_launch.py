#!/usr/bin/env python3
"""Launch file for psyche vision system."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Object detector node
        Node(
            package='psyche_vision',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[{
                'target_color_lower': [0, 100, 100],    # Red lower bound (HSV)
                'target_color_upper': [10, 255, 255],   # Red upper bound (HSV)  
                'min_area': 500,                        # Minimum contour area in pixels
                'camera_fov_degrees': 60.0,             # Camera field of view
            }]
        ),
        
        # Object controller node
        Node(
            package='psyche_vision',
            executable='object_controller',
            name='object_controller',
            output='screen',
            parameters=[{
                'max_angular_velocity': 0.3,            # rad/s - conservative for safety
                'proportional_gain': 1.5,               # Controller gain
                'angle_tolerance_degrees': 3.0,         # Deadzone in degrees
                'min_confidence': 0.2,                  # Minimum confidence to act
                'timeout_seconds': 3.0,                 # Stop if no target
            }]
        ),
    ])