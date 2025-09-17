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

provision() {
  ensure_ws
  sudo apt-get update -y
  # Prefer distro colcon package name (python3-colcon-common-extensions), fallback to ros-dev-tools already installed by ros.sh
  sudo apt-get install -y python3-colcon-common-extensions || true
}
build() {
  safe_source_ros
  ensure_ws
  # Try symlink (editable) install first; if it fails (e.g., due to unsupported
  # --editable in setup.py paths), fall back to a regular build.
  local log="$WS/colcon_first_try.log"
  set +e
  (
    cd "$WS" && colcon build --symlink-install
  ) 2>&1 | tee "$log"
  local rc=${PIPESTATUS[0]}
  set -e
  if [ $rc -ne 0 ]; then
    echo "[psy] colcon --symlink-install failed (rc=$rc); falling back to non-symlink build"
    (
      cd "$WS" && colcon build
    )
  fi
}

case "${1:-provision}" in
  provision) provision ;;
  build) build ;;
  *) echo "unknown"; exit 1;;
esac
