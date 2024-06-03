#!/bin/bash
export HOST_PKG=pete

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

source /opt/ros/$ROS_DISTRO/setup.bash
source /psyche/install/setup.bash

if [ -e $(whereis ros2 | awk '{print $2}') ]; then
    echo "Found a ROS2 node"
    echo "Looking for /psyche/src/hosts/$HOST_PKG/launch/by_host_$(hostname).launch.py"
    if [ -e /psyche/src/hosts/$HOST_PKG/launch/by_host_$(hostname).launch.py ]; then
        echo "Found $(hostname) specific launch file"
        ros2 launch $HOST_PKG by_host_$(hostname).launch.py &
    fi
    
    echo "Running the autoexec launch file"
    ros2 launch $HOST_PKG autoexec.launch.py
else
    # Sometimes we're launched on a non-ROS2 HOST_PKG
    if [ -e /psyche/launch/$(hostname).sh ]; then
        /psyche/launch/$(hostname).sh
    fi
fi

tail -f /dev/null
