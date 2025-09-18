#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS}"
SRC="$WS/src"

provision() {
  common_safe_source_ros || true
  
  # Install required packages for vision processing
  common_apt_install \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-vision-msgs \
    python3-opencv \
    python3-numpy

  # Vision package should already exist in workspace, just ensure structure
  common_ensure_ws
  # Clone additional vision-related repos if not already present
  common_clone_repo https://github.com/ptrmu/ros2_shared.git "$SRC/ros2_shared"
  # Note: psyche_vision package is committed to the repo in ws/src/

  # Build deferred: a single workspace build will run after provisioning

  # Create systemd launcher
  common_install_launcher vision LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
# Ensure sane env for logging under systemd
export HOME="${HOME:-/root}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/ros}"
export RCUTILS_LOGGING_DIR="${RCUTILS_LOGGING_DIR:-/var/log/ros}"
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
root="${PSY_ROOT:-/opt/psyched}"
set +u; [ -f "${root}/ws/install/setup.bash" ] && source "${root}/ws/install/setup.bash"; set -u
exec ros2 launch psyche_vision vision_launch.py
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
