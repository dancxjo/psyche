#!/usr/bin/env bash
set -euo pipefail
WS="/opt/psyched/ws"
SRC="$WS/src"

provision() {
  source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true
  sudo apt-get update -y
  sudo apt-get install -y git
  mkdir -p "$SRC"

  # Prefer sllidar_ros2, fallback rplidar_ros
  if [ ! -d "$SRC/sllidar_ros2" ]; then
    git clone https://github.com/Slamtec/sllidar_ros2 "$SRC/sllidar_ros2" || true
  fi
  if [ ! -d "$SRC/rplidar_ros" ]; then
    git clone https://github.com/Slamtec/rplidar_ros "$SRC/rplidar_ros" || true
  fi

  (cd /opt/psyched && cli/psy build)

  # udev rule for stable symlink
  sudo tee /etc/udev/rules.d/70-lidar.rules >/dev/null <<'RULE'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="lidar", MODE="0666"
RULE
  sudo udevadm control --reload-rules && sudo udevadm trigger || true

  # launcher
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/lidar.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source /opt/psyched/ws/install/setup.bash

if ros2 pkg executables sllidar_ros2 | grep -q sllidar_node; then
  exec ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/lidar -p frame_id:=laser
else
  exec ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/lidar -p frame_id:=laser
fi
LAUNCH
  sudo chmod +x /etc/psyched/lidar.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
