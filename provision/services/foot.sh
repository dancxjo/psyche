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
set -euo pipefail

# Source ROS 2 underlay and the psyched overlay when available. We relax nounset
# around the sourcing steps because the setup files rely on expected shell
# variables that may be undefined otherwise.
set +u; [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u

# Allow operators to override the launch entrypoint if necessary while keeping
# the default behavior aligned with manual bringup instructions.
CREATE_LAUNCH_PACKAGE="${CREATE_LAUNCH_PACKAGE:-create_bringup}"
CREATE_LAUNCH_FILE="${CREATE_LAUNCH_FILE:-create_1.launch}"
CREATE_LAUNCH_ARGS="${CREATE_LAUNCH_ARGS:-}"

read -r -a extra_launch_args <<<"${CREATE_LAUNCH_ARGS}"
if [ ${#extra_launch_args[@]} -eq 1 ] && [ -z "${extra_launch_args[0]}" ]; then
  extra_launch_args=()
fi

if [ "${FOOT_DEBUG:-0}" != "0" ]; then
  echo "[foot] exec ros2 launch ${CREATE_LAUNCH_PACKAGE} ${CREATE_LAUNCH_FILE} ${extra_launch_args[*]}" >&2
fi

exec ros2 launch "${CREATE_LAUNCH_PACKAGE}" "${CREATE_LAUNCH_FILE}" "${extra_launch_args[@]}"
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
