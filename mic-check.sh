#!/bin/bash
source /root/.bashrc
ros2 topic pub /voice std_msgs/msg/String "data: 'Hello my baby hello my darling hello my ragtime gal'"
