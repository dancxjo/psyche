#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Call the project-level entrypoint with the install-ros2 subcommand so the
# actual ROS2/Kaiju installation logic is centralized in tools/provision/setup_ros2.sh
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ENTRY="$PROJECT_ROOT/tools/provision/setup_ros2.sh"

if [ ! -x "$ENTRY" ]; then
	# Ensure the entrypoint is executable when invoked directly; otherwise call via bash
	echo "Calling setup entrypoint via bash: $ENTRY install-ros2"
	/bin/bash "$ENTRY" install-ros2
else
	echo "Calling setup entrypoint: $ENTRY install-ros2"
	"$ENTRY" install-ros2
fi

exit 0
