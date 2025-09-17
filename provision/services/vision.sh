#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS:-/opt/psyched/ws}"
SRC="$WS/src"

provision() {
  common_safe_source_ros || true
  
  # Install required packages for vision processing (deferred)
  export PSY_DEFER_APT=1
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

  # Patch kinect_ros2 to ensure cv_bridge/image_transport dependencies are declared and used
  if [ -d "$SRC/kinect_ros2" ]; then
    PKG_XML="$SRC/kinect_ros2/package.xml"
    CMAKELISTS="$SRC/kinect_ros2/CMakeLists.txt"
    # Inject missing runtime/build deps into package.xml if absent
    if [ -f "$PKG_XML" ]; then
      if ! grep -q "<depend>cv_bridge</depend>" "$PKG_XML"; then
        sed -i '/<\/package>/i \  <depend>cv_bridge<\/depend>' "$PKG_XML" || true
      fi
      if ! grep -q "<depend>image_transport</depend>" "$PKG_XML"; then
        sed -i '/<\/package>/i \  <depend>image_transport<\/depend>' "$PKG_XML" || true
      fi
    fi
    # Ensure CMake finds and links cv_bridge and image_transport
    if [ -f "$CMAKELISTS" ]; then
      if ! grep -qi 'find_package(.*cv_bridge' "$CMAKELISTS"; then
        if grep -n 'find_package(ament_cmake' "$CMAKELISTS" >/dev/null; then
          line=$(grep -n 'find_package(ament_cmake' "$CMAKELISTS" | head -n1 | cut -d: -f1)
          awk -v n="$line" 'NR==n{print;print "find_package(cv_bridge REQUIRED)"; print "find_package(image_transport REQUIRED)"; next}1' "$CMAKELISTS" > "$CMAKELISTS.tmp" && mv "$CMAKELISTS.tmp" "$CMAKELISTS"
        else
          printf '%s\n' 'find_package(cv_bridge REQUIRED)' 'find_package(image_transport REQUIRED)' | cat - "$CMAKELISTS" > "$CMAKELISTS.tmp" && mv "$CMAKELISTS.tmp" "$CMAKELISTS"
        fi
      fi
      # Add ament_target_dependencies for cv_bridge/image_transport if not present
      if ! grep -q 'ament_target_dependencies(.*cv_bridge' "$CMAKELISTS"; then
        # Insert before ament_package if possible, else append
        if grep -n '^ament_package' "$CMAKELISTS" >/dev/null; then
          sed -i '/^ament_package/i ament_target_dependencies(kinect_ros2_component cv_bridge image_transport)' "$CMAKELISTS" || true
        else
          echo 'ament_target_dependencies(kinect_ros2_component cv_bridge image_transport)' >> "$CMAKELISTS"
        fi
      fi
    fi
  fi

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
# Ensure sane env for logging under systemd
export HOME="${HOME:-/root}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/ros}"
export RCUTILS_LOGGING_DIR="${RCUTILS_LOGGING_DIR:-/var/log/ros}"
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u
exec ros2 launch psyche_vision vision_launch.py
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac