#!/usr/bin/env bash
# Simple ROS 2 entrypoint that sources the environment and execs the given command.
#
# Example:
#   docker compose exec auditor bash
#   docker compose exec auditor ros2 topic list
set -euo pipefail

# If available, source ROS 2 setup for the selected distro
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# If a user workspace exists and is built, overlay it
if [ -n "${ROS_WS:-}" ] && [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi

exec "$@"

