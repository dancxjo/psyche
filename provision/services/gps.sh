#!/usr/bin/env bash
set -euo pipefail
provision() {
  source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true
  sudo apt-get update -y
  sudo apt-get install -y ros-${ROS_DISTRO:-jazzy}-nmea-navsat-driver
  sudo tee /etc/udev/rules.d/70-gps.rules >/dev/null <<'RULE'
KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", SYMLINK+="gps", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1546", SYMLINK+="gps", MODE="0666"
RULE
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/gps.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
exec ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/gps -p baud:=9600 -r /fix:=/gps/fix -r /vel:=/gps/vel
LAUNCH
  sudo chmod +x /etc/psyched/gps.launch.sh
}
case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
