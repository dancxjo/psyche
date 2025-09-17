#!/usr/bin/env bash
set -euo pipefail
WS="/opt/psyched/ws"
SRC="$WS/src"
REPO_LIBCREATE="https://github.com/revyos-ros/libcreate.git"
BRANCH_LIBCREATE="fix-std-string"
REPO_CREATE_ROBOT="https://github.com/autonomylab/create_robot.git"

provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  mkdir -p "$SRC"
  [ -d "$SRC/libcreate" ] || git clone --branch "$BRANCH_LIBCREATE" "$REPO_LIBCREATE" "$SRC/libcreate"
  [ -d "$SRC/create_robot" ] || git clone "$REPO_CREATE_ROBOT" "$SRC/create_robot"
  # udev for stable /dev/create
  sudo tee /etc/udev/rules.d/70-create.rules >/dev/null <<'RULE'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="create", MODE="0666"
RULE
  sudo udevadm control --reload-rules && sudo udevadm trigger || true

  # launcher for systemd (optional, placeholder)
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/foot.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash
# Adjust port to /dev/create if udev rule applies
exec ros2 run create_driver create_driver_node --ros-args -p port:=/dev/create -r /odom:=/odom
LAUNCH
  sudo chmod +x /etc/psyched/foot.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
