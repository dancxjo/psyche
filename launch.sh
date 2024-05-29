#!/bin/bash

if [ -e $(whereis ros2 | awk '{print $2}') ]; then
    # Sometimes we're launched on a non-ROS2 host
    if [ -e /psyche/src/psyche/launch/$(hostname).launch.py ]; then
        ros2 launch psyche $(hostname).launch.py
    fi
else
    if [ -e /psyche/launch/$(hostname).sh ]; then
        /psyche/launch/$(hostname).sh
    fi
fi

tail -f /dev/null
