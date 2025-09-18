#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS}"

safe_source_ros() { common_safe_source_ros; }

ensure_ws() { common_ensure_ws; }

ensure_numpy() { common_ensure_numpy; }

ensure_ament_cmake() {
  local distro="${ROS_DISTRO:-jazzy}"
  local cmake_hint="/opt/ros/${distro}/share/ament_cmake/cmake/ament_cmakeConfig.cmake"

  if [ -f "$cmake_hint" ]; then
    return
  fi

  # Install the ROS distro-specific ament package; fall back to the generic name if needed.
  common_apt_install "ros-${distro}-ament-cmake" '?ament-cmake'
}

ensure_vision_deps() {
  # Kinect and other RGB-D pipelines depend on cv_bridge and related transports.
  common_apt_install \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-camera-calibration-parsers \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-image-transport-plugins \
    ros-${ROS_DISTRO:-jazzy}-image-pipeline \
    ros-${ROS_DISTRO:-jazzy}-perception \
    ros-${ROS_DISTRO:-jazzy}-perception-pcl \
    ros-${ROS_DISTRO:-jazzy}-vision-msgs \
    ros-${ROS_DISTRO:-jazzy}-usb-cam \
    libopencv-dev \
    python3-opencv \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    ?libogre-1.12-dev \
    ?ros-${ROS_DISTRO:-jazzy}-rviz2 \
    ?libgl1 \
    ?libegl1 \
    ?libxrandr2 \
    ?libxrandr-dev \
    ?libxinerama1 \
    ?libxinerama-dev \
    ?libxcursor1 \
    ?libxcursor-dev
}

ensure_cv_bridge_overlay() {
  local distro="${ROS_DISTRO:-jazzy}"
  local header="/opt/ros/${distro}/include/cv_bridge/cv_bridge.h"
  local pkg="ros-${distro}-cv-bridge"
  local repo="$SRC/vision_opencv"

  if dpkg -s "$pkg" >/dev/null 2>&1 || [ -f "$header" ]; then
    return
  fi

  if [ ! -d "$repo" ]; then
    common_clone_repo https://github.com/ros-perception/vision_opencv.git "$repo"
  fi

  if [ -d "$repo/.git" ]; then
    git -C "$repo" fetch --tags --force >/dev/null 2>&1 || true
    if git -C "$repo" rev-parse --verify "$distro" >/dev/null 2>&1; then
      git -C "$repo" checkout "$distro" >/dev/null 2>&1 || true
    elif git -C "$repo" rev-parse --verify "ros2" >/dev/null 2>&1; then
      git -C "$repo" checkout "ros2" >/dev/null 2>&1 || true
    fi
  fi

  if [ -d "$repo" ]; then
    rm -f "$repo/COLCON_IGNORE" 2>/dev/null || true
  fi
}

provision() {
  ensure_ws
  # Defer apt installs to combine them later
  export PSY_DEFER_APT=1
  # Prefer distro colcon package name; fallback handled in ros.sh
  common_apt_install python3-colcon-common-extensions
  # Ensure CMake tooling for ament packages is present before builds run.
  ensure_ament_cmake
  # Ensure core C++ vision deps used by RGB-D pipelines such as kinect_ros2
  # These provide cv_bridge headers and OpenCV C++ libs required at compile time
  ensure_vision_deps
  ensure_cv_bridge_overlay
}
build() {
  safe_source_ros
  ensure_ws
  ensure_numpy
  # Ensure vision dependencies are present even when provisioning hasn't queued them yet
  ensure_ament_cmake
  ensure_vision_deps
  ensure_cv_bridge_overlay
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
  # Resolve ROS package dependencies (only once, after packages are present)
  # - Skip if no packages found in src yet
  # - Skip if environment requests it (PSY_SKIP_ROSDEP=1)
  # - Cache based on src package manifest timestamps to avoid repeated runs
  if [ "${PSY_SKIP_ROSDEP:-0}" = "1" ]; then
    echo "[psy] PSY_SKIP_ROSDEP=1 — skipping rosdep"
  elif command -v rosdep >/dev/null 2>&1; then
    PKG_LIST_TMP="$(mktemp)"
    # Consider common ROS/ament package manifests
    find "$WS/src" -type f \( -name 'package.xml' -o -name 'setup.py' -o -name 'pyproject.toml' \) \
      -printf '%p %T@\n' 2>/dev/null | sort > "$PKG_LIST_TMP" || true
    if [ ! -s "$PKG_LIST_TMP" ]; then
      echo "[psy] No packages detected in $WS/src yet — deferring rosdep"
      rm -f "$PKG_LIST_TMP" || true
    else
      HASH_CUR=$(md5sum "$PKG_LIST_TMP" | awk '{print $1}')
      HASH_FILE="$WS/.rosdep_src_hash"
      HASH_PREV=""
      [ -f "$HASH_FILE" ] && HASH_PREV=$(cat "$HASH_FILE" 2>/dev/null || true)
      if [ "$HASH_CUR" = "$HASH_PREV" ]; then
        echo "[psy] src package set unchanged — skipping rosdep"
      else
        echo "[psy] Resolving dependencies via rosdep (packages changed)"
        # Initialize rosdep database only if not already initialized
        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
          set +e; sudo rosdep init 2>/dev/null || true; set -e
        fi
        # Ensure rosdep config is readable for non-root users
        if [ -d /etc/ros/rosdep ]; then
          sudo chown -R root:root /etc/ros/rosdep 2>/dev/null || true
          sudo chmod 755 /etc/ros/rosdep 2>/dev/null || true
        fi
        if [ -d /etc/ros/rosdep/sources.list.d ]; then
          sudo chmod 755 /etc/ros/rosdep/sources.list.d 2>/dev/null || true
        fi
        if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
          sudo chown root:root /etc/ros/rosdep/sources.list.d/20-default.list 2>/dev/null || true
          sudo chmod 644 /etc/ros/rosdep/sources.list.d/20-default.list 2>/dev/null || true
        fi
        rosdep update || true
        (
          cd "$WS" && rosdep install --from-paths src --ignore-src -r -y || true
        )
        echo "$HASH_CUR" > "$HASH_FILE" || true
      fi
      rm -f "$PKG_LIST_TMP" || true
    fi
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
