#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS:-/opt/psyched/ws}"
SRC="$WS/src"

provision() {
  common_safe_source_ros || true
  
  # Install required packages for vision processing
  common_apt_install \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-vision-msgs \
    python3-opencv \
    python3-numpy \
    libusb-1.0-0-dev \
    pkg-config

  # Vision package should already exist in workspace, just ensure structure
  common_ensure_ws
  # Clone additional vision-related repos if not already present
  common_clone_repo https://github.com/ptrmu/ros2_shared.git "$SRC/ros2_shared"
  common_clone_repo https://github.com/bribribriambriguy/kinect_ros2.git "$SRC/kinect_ros2" frame_correction
  common_clone_repo https://github.com/OpenKinect/libfreenect "$SRC/libfreenect"
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
  common_install_launcher vision LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash
exec ros2 launch psyche_vision vision_launch.py
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac