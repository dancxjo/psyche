#!/usr/bin/env bash
set -euo pipefail
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash || true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=/opt/psyched/provision/bringup/nav2_params.yaml
