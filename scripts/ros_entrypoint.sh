#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /ros2_ws
source install/setup.bash
exec "$@"
