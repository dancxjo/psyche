#!/bin/bash

if [ -e /psyche/src/psyche/launch/$(hostname).launch.py ]; then
    ros2 launch psyche $(hostname).launch.py
else
    tail -f /dev/null
fi

