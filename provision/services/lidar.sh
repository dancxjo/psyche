#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS:-/opt/psyched/ws}"
SRC="$WS/src"

provision() {
  common_safe_source_ros || true
  export PSY_DEFER_APT=1
  common_apt_install git
  common_ensure_ws

  # Install HLS-LFCD2 driver (preferred via apt), fallback to source clone
  if ! apt-cache show "ros-${ROS_DISTRO:-jazzy}-hls-lfcd-lds-driver" >/dev/null 2>&1; then
    echo "[psy][lidar] Package ros-${ROS_DISTRO:-jazzy}-hls-lfcd-lds-driver not found in apt cache; will clone from source"
  else
  common_apt_install "ros-${ROS_DISTRO:-jazzy}-hls-lfcd-lds-driver"
  fi
  if ! ros2 pkg executables hls_lfcd_lds_driver >/dev/null 2>&1; then
    common_clone_repo https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver "$SRC/hls_lfcd_lds_driver"
  fi

  # If legacy ROS1 rplidar_ros exists from earlier runs, prevent colcon from building it
  if [ -d "$SRC/rplidar_ros" ]; then
    touch "$SRC/rplidar_ros/COLCON_IGNORE" || true
  fi

  # Build will be triggered once after all services provision sources

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
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash

# HLS-LFCD2 driver
if ros2 pkg executables hls_lfcd_lds_driver | grep -q hlds_laser_publisher; then
  exec ros2 run hls_lfcd_lds_driver hlds_laser_publisher --ros-args -p serial_port:=/dev/lidar -p frame_id:=laser
fi

echo "[psy][lidar] hls_lfcd_lds_driver not available; ensure package is installed or built" >&2
exit 1
LAUNCH
  sudo chmod +x /etc/psyched/lidar.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
