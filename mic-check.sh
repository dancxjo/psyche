#!/bin/bash
source /root/.bashrc
ros2 topic pub /voice std_msgs/msg/String "data: \"$@\""
