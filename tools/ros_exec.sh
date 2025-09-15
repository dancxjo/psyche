#!/usr/bin/env bash
# Source a ROS 2 environment and exec the given command.
# Tries to detect distro under /opt/ros if ROS_DISTRO is not set.
set -euo pipefail

detect_ros_distro() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then
    echo "$ROS_DISTRO"; return 0
  fi
  local d
  for d in /opt/ros/*; do
    [ -d "$d" ] || continue
    if [ -f "$d/setup.sh" ]; then
      basename "$d"; return 0
    fi
  done
  echo ""; return 1
}

main() {
  local distro
  distro=$(detect_ros_distro || true)
  if [ -n "$distro" ]; then
    # shellcheck disable=SC1090
    . "/opt/ros/${distro}/setup.sh"
  fi
  if [ -f "/opt/ros_ws/install/setup.sh" ]; then
    # shellcheck disable=SC1091
    . "/opt/ros_ws/install/setup.sh"
  fi
  exec "$@"
}

main "$@"

