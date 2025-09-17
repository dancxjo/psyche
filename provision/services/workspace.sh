#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
WS="$ROOT/ws"
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

safe_source_ros() {
  # Avoid nounset errors from ROS setup using AMENT_TRACE_SETUP_FILES
  set +u
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || true
  set -u
}

ensure_ws() { common_ensure_ws; }

ensure_numpy() { common_ensure_numpy; }

provision() {
  ensure_ws
  # Defer apt installs to combine them later
  export PSY_DEFER_APT=1
  # Prefer distro colcon package name; fallback handled in ros.sh
  common_apt_install python3-colcon-common-extensions
  # Ensure core C++ vision deps for packages like kinect_ros2
  # These provide cv_bridge headers and OpenCV C++ libs required at compile time
  common_apt_install \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-image-transport-plugins \
    ros-${ROS_DISTRO:-jazzy}-vision-msgs \
    libopencv-dev
}
build() {
  safe_source_ros
  ensure_ws
  ensure_numpy
  # Enable ccache to speed up rebuilds
  common_enable_ccache || true
  export CC="ccache gcc"
  export CXX="ccache g++"
  export CCACHE_COMPRESS=1
  export CCACHE_MAXSIZE="5G"
  export CCACHE_BASEDIR="$WS"
  export CCACHE_SLOPPINESS=time_macros
  # Also pass CMake launcher so projects use ccache even if they reset CC/CXX
  # Install any queued apt packages before building
  common_flush_apt_queue || true
  # Resolve ROS package dependencies
  if command -v rosdep >/dev/null 2>&1; then
    echo "[psy] Resolving dependencies via rosdep"
    set +e
    sudo rosdep init 2>/dev/null || true
    set -e
    rosdep update || true
    (
      cd "$WS" && rosdep install --from-paths src --ignore-src -r -y || true
    )
  else
    echo "[psy] rosdep not found; skipping dependency resolution"
  fi
  (
    # Try to obtain NumPy include dir to help CMake
    INC_DIR="$((/usr/bin/python3 -c 'import numpy as np; print(np.get_include())' 2>/dev/null) || echo "")"
    cd "$WS" && colcon build \
      --cmake-args \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPython3_FIND_VIRTUALENV=NEVER \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        ${INC_DIR:+-DPython3_NumPy_INCLUDE_DIRS=$INC_DIR}
  )
}

case "${1:-provision}" in
  provision) provision ;;
  build) build ;;
  *) echo "unknown"; exit 1;;
esac
