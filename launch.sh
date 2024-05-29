#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

source /opt/ros/$ROS_DISTRO/setup.bash
source /psyche/install/setup.bash

if [ -e $(whereis ros2 | awk '{print $2}') ]; then
    if [ -e /psyche/src/psyche/launch/by_host_$(hostname).launch.py ]; then
        ros2 launch psyche by_host_$(hostname).launch.py &
    fi

    ros2 launch psyche autoexec.launch.py
else
    # Sometimes we're launched on a non-ROS2 host
    if [ -e /psyche/launch/$(hostname).sh ]; then
        /psyche/launch/$(hostname).sh
    fi
fi

tail -f /dev/null
