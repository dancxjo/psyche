#!/bin/bash

ros2 topic pub --once /voice std_msgs/msg/String "data: \"$@\""
