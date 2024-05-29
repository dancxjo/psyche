#!/bin/bash

if [ -e $(whereis ros2 | awk '{print $2}') ]; then
    ros2 launch psyche autoexec.launch.py
    if [ -e /psyche/src/psyche/launch/by_host_$(hostname).launch.py ]; then
        ros2 launch psyche by_host_$(hostname).launch.py
    fi
else
    # Sometimes we're launched on a non-ROS2 host
    if [ -e /psyche/launch/$(hostname).sh ]; then
        /psyche/launch/$(hostname).sh
    fi
fi

tail -f /dev/null
