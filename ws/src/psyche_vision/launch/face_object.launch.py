#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='psyche_vision',
            executable='object_detector',
            name='detector',
            output='screen',
            parameters=[{
                'publish_target_point': True,
            }]
        ),
        Node(
            package='psyche_vision',
            executable='object_controller',
            name='controller',
            output='screen',
            parameters=[{
                'use_target_point': True,
            }]
        ),
    ])
