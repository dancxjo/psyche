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

KINECT_REPO="${PSY_KINECT_REPO:-https://github.com/thallesgate/kinect_ros2.git}"
KINECT_BRANCH="${PSY_KINECT_BRANCH:-jazzy}"
LIBFREENECT_REPO="https://github.com/OpenKinect/libfreenect.git"

ensure_kinect_repo() {
  local dest="$SRC/kinect_ros2"
  if [ ! -d "$dest" ]; then
    common_clone_repo "$KINECT_REPO" "$dest" "$KINECT_BRANCH"
    return
  fi

  if [ -d "$dest/.git" ]; then
    local remote
    remote="$(git -C "$dest" remote get-url origin 2>/dev/null || true)"
    if [ "$remote" != "$KINECT_REPO" ]; then
      echo "[psy][eye_kinect] Existing repo at $dest uses remote $remote (expected $KINECT_REPO). Leaving unchanged." >&2
    else
      git -C "$dest" fetch origin --tags || true
      local desired_branch="$KINECT_BRANCH"
      if [ -n "$desired_branch" ]; then
        git -C "$dest" fetch origin "$desired_branch" || true
        local current_branch
        current_branch="$(git -C "$dest" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
        if [ "$current_branch" != "$desired_branch" ]; then
          git -C "$dest" checkout "$desired_branch" || true
        fi
        git -C "$dest" pull --ff-only origin "$desired_branch" || true
      else
        local branch
        branch="$(git -C "$dest" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
        if [ -n "$branch" ]; then
          git -C "$dest" pull --ff-only origin "$branch" || true
        fi
      fi
    fi
  else
    echo "[psy][eye_kinect] Path $dest exists but is not a git repo; skipping clone/update." >&2
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
exec ros2 launch psyche_vision eye_kinect.launch.py
LAUNCH
}

provision() {
  common_safe_source_ros || true

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
  ensure_libfreenect
  write_launcher
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
