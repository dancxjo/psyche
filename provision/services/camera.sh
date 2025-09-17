#!/usr/bin/env bash
set -euo pipefail
provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  sudo apt-get update -y
  sudo apt-get install -y ros-${ROS_DISTRO:-jazzy}-usb-cam
  # Prefer v4l-utils (modern name). Fall back to v4l2-utils if available on this distro.
  if ! sudo apt-get install -y v4l-utils; then
    echo "[psy][camera] v4l-utils not found; trying v4l2-utils"
    sudo apt-get install -y v4l2-utils || true
  fi
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
