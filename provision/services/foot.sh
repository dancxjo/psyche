#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS:-/opt/psyched/ws}"
SRC="$WS/src"
REPO_LIBCREATE="https://github.com/revyos-ros/libcreate.git"
BRANCH_LIBCREATE="fix-std-string"
REPO_CREATE_ROBOT="https://github.com/autonomylab/create_robot.git"

provision() {
  common_safe_source_ros || true
  common_ensure_ws
  common_clone_repo "$REPO_LIBCREATE" "$SRC/libcreate" "$BRANCH_LIBCREATE"
  common_clone_repo "$REPO_CREATE_ROBOT" "$SRC/create_robot"
  # udev for stable /dev/create
  common_write_udev_rules /etc/udev/rules.d/70-create.rules RULE <<'RULE'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="create", MODE="0666"
RULE

  # launcher for systemd with resilient retry loop
  common_install_launcher foot LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
LOG=/tmp/create_driver.log
PORT_DEFAULT=/dev/create
PORT="${CREATE_PORT:-$PORT_DEFAULT}"

# Optional overrides via /etc/default/psyched-foot
if [ -f /etc/default/psyched-foot ]; then
  . /etc/default/psyched-foot
  PORT="${CREATE_PORT:-$PORT}"
fi

# Source ROS and workspace overlays if present
set +u; [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
[ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash || true

echo "[foot] launcher started. Using port=$PORT" | tee -a "$LOG"

while true; do
  # Wait for device symlink to appear
  if [ ! -e "$PORT" ]; then
    # Settle udev and wait a bit
    command -v udevadm >/dev/null 2>&1 && sudo udevadm settle || true
    echo "[foot] Waiting for $PORT ..." | tee -a "$LOG"
    sleep 2
    continue
  fi

  echo "[foot] Starting create_driver on $PORT" | tee -a "$LOG"
  set +e
  ros2 run create_driver create_driver_node --ros-args -p port:="$PORT" -r /odom:=/odom >> "$LOG" 2>&1
  RC=$?
  set -e
  echo "[foot] create_driver exited rc=$RC; restarting in 3s" | tee -a "$LOG"
  sleep 3
done
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
