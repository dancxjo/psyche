#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
CFG="$ROOT/provision/hosts/$(hostname).toml"

ros_toml() {
  awk -F'=' '/ros_distro/{gsub(/[ "]/ ,"",$2);print $2}' "$CFG" 2>/dev/null || true
}
get() { awk -F'=' -v k="$1" '$1~k{gsub(/[ "]/ ,"",$2);print $2}' "$CFG" 2>/dev/null || true; }

provision() {
  ROS_DISTRO="$(ros_toml || echo jazzy)"
  sudo apt-get update -y
  sudo apt-get install -y curl gnupg lsb-release
  sudo apt-get install -y software-properties-common
  sudo apt-get install -y python3-pip python3-venv ros-dev-tools
  # Try distro colcon extensions; ignore if not found, we'll fallback to pip
  sudo apt-get install -y python3-colcon-common-extensions || true
  # Fallback: install colcon via pip if not present
  if ! command -v colcon >/dev/null 2>&1; then
    python3 -m pip install --upgrade --user pip wheel 'setuptools<80' || true
    python3 -m pip install --upgrade --user colcon-common-extensions colcon-ros colcon-mixin || true
  fi
  # ROS 2 base and CycloneDDS
  sudo apt-get install -y "ros-${ROS_DISTRO}-ros-base" "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
  # Nice to have packages
  sudo apt-get install -y "ros-${ROS_DISTRO}-diagnostic-updater" \
                          "ros-${ROS_DISTRO}-xacro" \
                          "ros-${ROS_DISTRO}-image-transport" \
                          "ros-${ROS_DISTRO}-image-transport-plugins" \
                          "ros-${ROS_DISTRO}-nav2-bringup" \
                          "ros-${ROS_DISTRO}-tf2-ros"
  # Environment
  grep -q "RMW_IMPLEMENTATION" ~/.bashrc || {
    echo "export RMW_IMPLEMENTATION=$(get rmw || echo rmw_cyclonedds_cpp)" >> ~/.bashrc
  }
  grep -q "ROS_DOMAIN_ID" ~/.bashrc || {
    echo "export ROS_DOMAIN_ID=$(get domain_id || echo 42)" >> ~/.bashrc
  }
  grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc || {
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
  }
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown subcmd" >&2; exit 1;;
esac
