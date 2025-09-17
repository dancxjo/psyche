#!/usr/bin/env bash
set -euo pipefail

set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
HOST="${1:-$(hostname -s)}"
TEXT="${2:-Hello from Psyche voice}"
ros2 topic pub --once \
  "/voice/${HOST}" std_msgs/String "{data: '${TEXT}'}"
