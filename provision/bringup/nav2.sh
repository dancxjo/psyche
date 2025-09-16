#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source /opt/psyched/ws/install/setup.bash || true

# Quick static transforms (adjust as needed)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser  >/tmp/tf_laser.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu    >/tmp/tf_imu.log   2>&1 &

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=/opt/psyched/provision/bringup/nav2_params.yaml
