#!/usr/bin/env bash
set -euo pipefail
WS="/opt/psyched/ws"
SRC="$WS/src"

provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  
  # Install required packages for vision processing
  sudo apt-get update -y
  sudo apt-get install -y \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-vision-msgs \
    python3-opencv \
    python3-numpy

  # Vision package should already exist in workspace, just ensure structure
  mkdir -p "$SRC"
  # Note: psyche_vision package is committed to the repo in ws/src/

  # Build the workspace
  (cd /opt/psyched && cli/psy build)

  # Create systemd launcher
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/vision.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash
exec ros2 launch psyche_vision vision_launch.py
LAUNCH
  sudo chmod +x /etc/psyched/vision.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac