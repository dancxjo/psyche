#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

provision() {
  common_safe_source_ros || true
    export PSY_DEFER_APT=1
    common_apt_install "ros-${ROS_DISTRO:-jazzy}-usb-cam"
    # v4l-utils may be named v4l2-utils on some systems; try both (queue both, one will succeed)
    common_apt_install v4l-utils v4l2-utils

  common_install_launcher camera LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u
exec ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -p pixel_format:=mjpeg -p image_width:=1280 -p image_height:=720 -r image:=/camera/image_raw
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
