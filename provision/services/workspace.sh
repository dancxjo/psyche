#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
WS="$ROOT/ws"

ensure_ws() {
  mkdir -p "$WS/src"
  touch "$WS/.colcon_keep"
}

provision() {
  ensure_ws
  sudo apt-get update -y
  sudo apt-get install -y ros-${ROS_DISTRO:-jazzy}-colcon-common-extensions
}
build() {
  source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true
  ensure_ws
  (cd "$WS" && colcon build --symlink-install)
}

case "${1:-provision}" in
  provision) provision ;;
  build) build ;;
  *) echo "unknown"; exit 1;;
esac
