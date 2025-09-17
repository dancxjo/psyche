#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  export PSY_DEFER_APT=1
  common_apt_install ros-${ROS_DISTRO:-jazzy}-nmea-navsat-driver
  common_write_udev_rules /etc/udev/rules.d/70-gps.rules RULE <<'RULE'
KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", SYMLINK+="gps", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1546", SYMLINK+="gps", MODE="0666"
RULE
  common_install_launcher gps LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
exec ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/gps -p baud:=9600 -r /fix:=/gps/fix -r /vel:=/gps/vel
LAUNCH
}
case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
