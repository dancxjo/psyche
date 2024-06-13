#!/bin/bash

### TODO: The word "host" here is used in at least three different senses in this file

# "r1" is the robot who _hosts_ the psyche toolset
export HOST_PKG=r1
export ROS_DISTRO=iron

### Check if running inside a Docker container
if [ -e "/ros_entrypoint.sh" ]; then

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42

    source /opt/ros/$ROS_DISTRO/setup.bash
    source /psyche/install/setup.bash

    echo "Inside docker node, use $HOST_PKG launch files"
    echo "Looking for /psyche/src/$HOST_PKG/launch/by_host_$(hostname).launch.py"
    if [ -e /psyche/src/$HOST_PKG/launch/by_host_$(hostname).launch.py ]; then
        echo "Found $(hostname) specific launch file"
        ros2 launch $HOST_PKG by_host_$(hostname).launch.py &
    fi
    
    echo "Running the autoexec launch file"
    ros2 launch $HOST_PKG autoexec.launch.py

else

    echo "Outside docker node, use base launch shell scripts"
    if [ -e /psyche/launch/$(hostname).sh ]; then
        /psyche/launch/$(hostname).sh
    fi


fi

### If we get here, just spin
tail -f /dev/null
