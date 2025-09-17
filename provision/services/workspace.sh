#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
WS="$ROOT/ws"

safe_source_ros() {
  # Avoid nounset errors from ROS setup using AMENT_TRACE_SETUP_FILES
  set +u
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || true
  set -u
}

ensure_ws() {
  mkdir -p "$WS/src"
  touch "$WS/.colcon_keep"
}

ensure_numpy() {
  # Ensure Python3 NumPy is available for rosidl_generator_py
  if ! python3 -c 'import numpy' >/dev/null 2>&1; then
    echo "[psy] Python NumPy not found; installing python3-numpy"
    sudo apt-get update -y || true
    sudo apt-get install -y python3-numpy || true
  fi
}

provision() {
  ensure_ws
  sudo apt-get update -y
  # Prefer distro colcon package name (python3-colcon-common-extensions), fallback to ros-dev-tools already installed by ros.sh
  sudo apt-get install -y python3-colcon-common-extensions || true
}
build() {
  safe_source_ros
  ensure_ws
  ensure_numpy
  # Resolve ROS package dependencies
  if command -v rosdep >/dev/null 2>&1; then
    echo "[psy] Resolving dependencies via rosdep"
    set +e
    sudo rosdep init 2>/dev/null || true
    set -e
    rosdep update || true
    (
      cd "$WS" && rosdep install --from-paths src --ignore-src -r -y || true
    )
  else
    echo "[psy] rosdep not found; skipping dependency resolution"
  fi
  (
    cd "$WS" && colcon build
  )
}

case "${1:-provision}" in
  provision) provision ;;
  build) build ;;
  *) echo "unknown"; exit 1;;
esac
