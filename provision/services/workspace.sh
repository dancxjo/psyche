#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS}"

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
  # Ensure core C++ vision deps used by RGB-D pipelines such as kinect_ros2
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
