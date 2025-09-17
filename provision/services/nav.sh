#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  # Install nav2 bringup and slam_toolbox (queued)
  export PSY_DEFER_APT=1
  common_apt_install ros-${ROS_DISTRO:-jazzy}-nav2-bringup ros-${ROS_DISTRO:-jazzy}-slam-toolbox ros-${ROS_DISTRO:-jazzy}-tf2-ros

  common_install_launcher nav LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
# Ensure sane env for logging under systemd
export HOME="${HOME:-/root}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/ros}"
export RCUTILS_LOGGING_DIR="${RCUTILS_LOGGING_DIR:-/var/log/ros}"
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u

# Run slam_toolbox (sync mode) and Nav2; assumes robot_state_publisher running
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/opt/psyched/provision/bringup/slam_params.yaml >/tmp/slam_toolbox.log 2>&1 &

exec ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false \
  params_file:=/opt/psyched/provision/bringup/nav2_params.yaml
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
