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
# FTDI FT232 (common on some Create cables)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="create", MODE="0666"
# Prolific PL2303 classic
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="create", MODE="0666"
# Prolific PL2303 (newer variants, e.g. 23c3 observed in the field)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23c3", SYMLINK+="create", MODE="0666"
RULE

  # launcher for systemd with resilient retry loop
  common_install_launcher foot LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -e
LOG=/tmp/create_driver.log
PORT_DEFAULT=/dev/create
PORT="${CREATE_PORT:-$PORT_DEFAULT}"

# Diagnostics / tuning env vars (may be exported via /etc/default/psyched-foot)
FAIL_MAX_BEFORE_BACKOFF="${FAIL_MAX_BEFORE_BACKOFF:-20}"   # after this many rapid fails, add longer sleep
BACKOFF_SLEEP="${BACKOFF_SLEEP:-10}"
FOOT_DEBUG="${FOOT_DEBUG:-0}"              # 1 for verbose
FOOT_DIAG_INTERVAL="${FOOT_DIAG_INTERVAL:-30}" # seconds between deep diagnostics
LAST_DIAG_TS=0
FAIL_COUNT=0

# Helper: timestamped log line
log() { echo "[foot $(date +%H:%M:%S)] $*" | tee -a "$LOG"; }

# Helper: run deep diagnostics (serial, permissions, udev, dmesg excerpt)
deep_diagnostics() {
  log "--- Deep diagnostics start ---"
  if [ -L "$PORT" ]; then ls -l "$PORT" | tee -a "$LOG" || true; fi
  if [ -e "$PORT" ]; then
    stat "$PORT" 2>&1 | tee -a "$LOG" || true
    sudo udevadm info --query=all --name="$PORT" 2>&1 | tee -a "$LOG" || true
  else
    log "Device $PORT does not exist (symlink or node missing)"
  fi
  # Enumerate candidate USB serial devices
  ls -l /dev/ttyUSB* 2>/dev/null | head -n 20 | tee -a "$LOG" || true
  # Recent dmesg lines related to USB / tty
  dmesg | egrep -i 'ttyUSB|ftdi|pl2303|usb' | tail -n 25 | tee -a "$LOG" || true
  log "ROS_DISTRO=${ROS_DISTRO:-unset}; create_driver presence check:"
  if command -v ros2 >/dev/null 2>&1; then
    ros2 pkg executables create_driver 2>&1 | tee -a "$LOG" || true
    ros2 pkg list | egrep -i 'create|libcreate' 2>/dev/null | tee -a "$LOG" || true
  else
    log "ros2 command not found in PATH"
  fi
  log "--- Deep diagnostics end ---"
}

# Optional overrides via /etc/default/psyched-foot
if [ -f /etc/default/psyched-foot ]; then
  . /etc/default/psyched-foot
  PORT="${CREATE_PORT:-$PORT}"
fi

# Source ROS and workspace overlays if present (guard nounset)
set +u; [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u

log "launcher started. Using port=$PORT"
log "FOOT_DEBUG=$FOOT_DEBUG FAIL_MAX_BEFORE_BACKOFF=$FAIL_MAX_BEFORE_BACKOFF BACKOFF_SLEEP=$BACKOFF_SLEEP"

udev_settled=0
waiting_logged=0
while true; do
  # Wait for device symlink to appear
  if [ ! -e "$PORT" ]; then
    if [ "$udev_settled" -eq 0 ]; then
      if command -v udevadm >/dev/null 2>&1; then
        sudo udevadm settle || true
      fi
      udev_settled=1
    fi
    if [ "$waiting_logged" -eq 0 ]; then
      log "Waiting for $PORT ..."
      waiting_logged=1
      deep_diagnostics || true
    fi
    sleep 2
    continue
  fi

  udev_settled=0
  waiting_logged=0
  log "Starting create_driver on $PORT (fail_count=$FAIL_COUNT)"
  set +e
  ros2 run create_driver create_driver_node --ros-args -p port:="$PORT" -r /odom:=/odom >> "$LOG" 2>&1
  RC=$?
  set -e
  FAIL_COUNT=$((FAIL_COUNT+1))
  log "create_driver exited rc=$RC (fail_count=$FAIL_COUNT)"
  NOW=$(date +%s)
  # Periodic deep diagnostics
  if [ $((NOW - LAST_DIAG_TS)) -ge "$FOOT_DIAG_INTERVAL" ]; then
    LAST_DIAG_TS=$NOW
    deep_diagnostics || true
  fi
  # If many rapid failures, introduce backoff
  if [ "$FAIL_COUNT" -ge "$FAIL_MAX_BEFORE_BACKOFF" ]; then
    log "Failure count >= $FAIL_MAX_BEFORE_BACKOFF applying backoff sleep $BACKOFF_SLEEP"
    sleep "$BACKOFF_SLEEP"
  else
    sleep 3
  fi
done
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
