#!/usr/bin/env bash
set -euo pipefail

# Source ROS 2 underlay and the psyched overlay when available.
set +u; [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; set -u
set +u; [ -f /opt/psyched/ws/install/setup.bash ] && source /opt/psyched/ws/install/setup.bash; set -u

# Allow overrides via environment variables to stay aligned with systemd launcher defaults.
CREATE_LAUNCH_PACKAGE="${CREATE_LAUNCH_PACKAGE:-create_bringup}"
CREATE_LAUNCH_FILE="${CREATE_LAUNCH_FILE:-create_1.launch}"
CREATE_LAUNCH_ARGS="${CREATE_LAUNCH_ARGS:-}"

read -r -a extra_launch_args <<<"${CREATE_LAUNCH_ARGS}"
if [ ${#extra_launch_args[@]} -eq 1 ] && [ -z "${extra_launch_args[0]}" ]; then
  extra_launch_args=()
fi

if [ "${FOOT_DEBUG:-0}" != "0" ]; then
  echo "[foot] exec ros2 launch ${CREATE_LAUNCH_PACKAGE} ${CREATE_LAUNCH_FILE} ${extra_launch_args[*]} $*" >&2
fi

exec ros2 launch "${CREATE_LAUNCH_PACKAGE}" "${CREATE_LAUNCH_FILE}" "${extra_launch_args[@]}" "$@"
