from launch import LaunchDescription

from .voice import voice

def generate_launch_description():
    return LaunchDescription([
        voice
    ])
