#!/usr/bin/env bash
set -euo pipefail
provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  sudo apt-get update -y
  sudo apt-get install -y ros-${ROS_DISTRO:-jazzy}-usb-cam v4l2-utils
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/camera.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
exec ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 -p pixel_format:=mjpeg -p image_width:=1280 -p image_height:=720 -r image:=/camera/image_raw
LAUNCH
  sudo chmod +x /etc/psyched/camera.launch.sh
}
case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
