#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
root="${PSY_ROOT:-$DEFAULT_ROOT}"
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
set +u; [ -f "${root}/ws/install/setup.bash" ] && source "${root}/ws/install/setup.bash"; set -u

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:="${root}/provision/bringup/nav2_params.yaml"
