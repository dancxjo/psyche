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
    python3-numpy \
    libusb-1.0-0-dev \
    pkg-config

  # Vision package should already exist in workspace, just ensure structure
  mkdir -p "$SRC"
  # Clone additional vision-related repos if not already present
  [ -d "$SRC/ros2_shared" ] || git clone https://github.com/ptrmu/ros2_shared.git "$SRC/ros2_shared" || true
  [ -d "$SRC/kinect_ros2" ] || git clone --branch frame_correction https://github.com/bribribriambriguy/kinect_ros2.git "$SRC/kinect_ros2" || true
  [ -d "$SRC/libfreenect" ] || git clone https://github.com/OpenKinect/libfreenect "$SRC/libfreenect" || true
  # Prevent colcon from attempting to build non-ROS libfreenect; we build/install it manually below
  [ -d "$SRC/libfreenect" ] && touch "$SRC/libfreenect/COLCON_IGNORE" || true

  # Best-effort build of libfreenect (non-ROS dep used by kinect stack)
  if [ -d "$SRC/libfreenect" ]; then
    (
      cd "$SRC/libfreenect"
      mkdir -p build && cd build
      cmake -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF -DBUILD_PYTHON=OFF .. && \
        make -j"$(nproc)" && \
        sudo make install && \
        sudo ldconfig || true
    )
  fi
  # Note: psyche_vision package is committed to the repo in ws/src/

  # Build deferred: a single workspace build will run after provisioning

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