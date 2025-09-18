#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

WS="${PSY_WS}"
SRC="$WS/src"

# Add vision as a dependency to ensure cv_bridge is installed first
if ! grep -q 'vision' "$PSY_HOST_FILE"; then
    echo "[psy][eye_kinect] This service requires the 'vision' service to be enabled." >&2
    #exit 1
fi

KINECT_REPO="https://github.com/fadlio/kinect_ros2.git"
LIBFREENECT_REPO="https://github.com/OpenKinect/libfreenect.git"

ensure_kinect_repo() {
  local dest="$SRC/kinect_ros2"
  if [ ! -d "$dest" ]; then
    common_clone_repo "$KINECT_REPO" "$dest"
    ensure_kinect_cmake_support "$dest"
    return
  fi

  if [ -d "$dest/.git" ]; then
    local remote
    remote="$(git -C "$dest" remote get-url origin 2>/dev/null || true)"
    if [ "$remote" != "$KINECT_REPO" ]; then
      echo "[psy][eye_kinect] Existing repo at $dest uses remote $remote (expected $KINECT_REPO). Leaving unchanged." >&2
    else
      git -C "$dest" fetch origin --tags || true
      local branch
      branch="$(git -C "$dest" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
      if [ -n "$branch" ]; then
        git -C "$dest" pull --ff-only origin "$branch" || true
      fi
    fi
  else
    echo "[psy][eye_kinect] Path $dest exists but is not a git repo; skipping clone/update." >&2
  fi
  ensure_kinect_cmake_support "$dest"
}

ensure_kinect_package_deps() {
    local repo="$1"
    [ -d "$repo" ] || return
    local package_xml="$repo/package.xml"
    if [ -f "$package_xml" ] && ! grep -q '<depend>cv_bridge</depend>' "$package_xml"; then
        # Add cv_bridge as a dependency
        sed -i '/<buildtool_depend>ament_cmake<\/buildtool_depend>/a \ \ <depend>cv_bridge<\/depend>' "$package_xml"
    fi
}

ensure_kinect_cmake_support() {
  local repo="$1"
  [ -d "$repo" ] || return

  local cmake_dir="$repo/cmake"
  local find_module="$cmake_dir/Findlibfreenect.cmake"

  mkdir -p "$cmake_dir"

  if [ ! -f "$find_module" ]; then
    cat >"$find_module" <<'EOF'
# Distributed under the BSD license.
# Minimal Find module for libfreenect when CMake config is not provided.

find_path(libfreenect_INCLUDE_DIR
  NAMES libfreenect/libfreenect.h libfreenect.h
  PATH_SUFFIXES libfreenect include
)

find_library(libfreenect_LIBRARY
  NAMES freenect
)

if (libfreenect_INCLUDE_DIR AND EXISTS "${libfreenect_INCLUDE_DIR}/libfreenect/libfreenect.h")
  set(libfreenect_INCLUDE_DIR "${libfreenect_INCLUDE_DIR}/libfreenect")
endif()

set(libfreenect_INCLUDE_DIRS "${libfreenect_INCLUDE_DIR}")
set(libfreenect_LIBRARIES "${libfreenect_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libfreenect
  REQUIRED_VARS libfreenect_LIBRARY libfreenect_INCLUDE_DIR)

mark_as_advanced(libfreenect_INCLUDE_DIR libfreenect_LIBRARY)
EOF
  fi

  # Ensure CMake uses the local Find module directory.
  if ! grep -Fq 'CMAKE_MODULE_PATH' "$repo/CMakeLists.txt"; then
    REPO_PATH="$repo" python3 - <<'PY'
import os
from pathlib import Path

repo = Path(os.environ["REPO_PATH"])
cmakelists = repo / "CMakeLists.txt"
text = cmakelists.read_text()
insertion = 'list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")\n\n'

if insertion.strip() in text:
    raise SystemExit(0)

marker = 'project(kinect_ros2)\n'
if marker in text:
    text = text.replace(marker, marker + insertion, 1)
else:
    marker = 'project(kinect_ros2)\r\n'
    insertion_crlf = insertion.replace('\n', '\r\n')
    if marker in text:
        text = text.replace(marker, marker + insertion_crlf, 1)
    else:
        raise SystemExit(0)

cmakelists.write_text(text)
PY
  fi

  # Ensure libfreenect include directory is added for compilation.
  if ! grep -Fq 'libfreenect_INCLUDE_DIRS' "$repo/CMakeLists.txt"; then
    REPO_PATH="$repo" python3 - <<'PY'
import os
from pathlib import Path

repo = Path(os.environ["REPO_PATH"])
cmakelists = repo / "CMakeLists.txt"
text = cmakelists.read_text()

newline = '\n'
if '\r\n' in text and '\n' not in text.replace('\r\n', ''):
    newline = '\r\n'

needle = 'include_directories(include)' + newline
replacement = 'include_directories(include){nl}include_directories(${{libfreenect_INCLUDE_DIRS}}){nl}'.format(nl=newline)

if needle not in text:
    needle = 'include_directories(include)'
    replacement = 'include_directories(include){nl}include_directories(${{libfreenect_INCLUDE_DIRS}})'.format(nl=newline)
    if needle not in text:
        raise SystemExit(0)

text = text.replace(needle, replacement, 1)
cmakelists.write_text(text)
PY
  fi
}

ensure_libfreenect() {
  local dest="$SRC/libfreenect"
  common_clone_repo "$LIBFREENECT_REPO" "$dest"
  [ -d "$dest" ] && touch "$dest/COLCON_IGNORE" || true

  if ldconfig -p 2>/dev/null | grep -q 'libfreenect.so'; then
    return
  fi

  if [ -d "$dest" ]; then
    (
      cd "$dest"
      mkdir -p build && cd build
      cmake -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF -DBUILD_PYTHON=OFF .. && \
        make -j"$(nproc)" && \
        sudo make install && \
        sudo ldconfig
    ) || echo "[psy][eye_kinect] libfreenect build failed" >&2
  fi
}

write_launcher() {
  common_install_launcher eye_kinect LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
LOG_FILE="/var/log/psyched-eye-kinect.log"
exec &> >(tee -a "$LOG_FILE")
echo "---"
echo "[psy][eye_kinect] Launching Kinect node at $(date)"
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
root="${PSY_ROOT:-/opt/psyched}"
set +u; [ -f "${root}/ws/install/setup.bash" ] && source "${root}/ws/install/setup.bash"; set -u
exec ros2 run kinect_ros2 kinect_ros2_node --ros-args \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  -r depth/image_raw:=/camera/depth/image_raw \
  -r depth/camera_info:=/camera/depth/camera_info \
  -r points:=/camera/depth/points
LAUNCH
}

provision() {
  common_safe_source_ros || true

  export PSY_DEFER_APT=1
  common_apt_install \
    ros-${ROS_DISTRO:-jazzy}-cv-bridge \
    ros-${ROS_DISTRO:-jazzy}-image-transport \
    ros-${ROS_DISTRO:-jazzy}-camera-info-manager \
    ros-${ROS_DISTRO:-jazzy}-depth-image-proc \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libopencv-dev \
    pkg-config \
    cmake

  common_ensure_ws
  ensure_kinect_repo
  ensure_kinect_package_deps "$SRC/kinect_ros2"
  ensure_libfreenect
  write_launcher
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
