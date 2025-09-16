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

echo "Now running global workspace setup for ear.local"
GW="$PROJECT_ROOT/tools/provision/setup_global_workspace.sh"

# Ensure the per-host code is present in the global workspace src
WORKSPACE_DIR=/opt/psyche_workspace
SRC_DIR="$WORKSPACE_DIR/src"
if [ $(id -u) -ne 0 ]; then
	SUDO=sudo
else
	SUDO=
fi

clone_or_update() {
	local url="$1"
	local dest="$2"
	local branch="$3"

	if [ -d "$dest/.git" ]; then
		echo "Updating existing repo at $dest"
		$SUDO git -C "$dest" fetch --all --prune || true
		if [ -n "$branch" ]; then
			$SUDO git -C "$dest" checkout "$branch" || $SUDO git -C "$dest" checkout -B "$branch"
			$SUDO git -C "$dest" pull --ff-only || true
		else
			$SUDO git -C "$dest" pull --ff-only || true
		fi
	else
		echo "Cloning $url into $dest"
		$SUDO mkdir -p "$(dirname "$dest")"
		$SUDO git clone ${branch:+--branch "$branch"} "$url" "$dest"
	fi
}

echo "Preparing /opt/psyche_workspace/src for ear.local components"
$SUDO mkdir -p "$SRC_DIR"
$SUDO chown root:root "$SRC_DIR" || true

echo "Cloning/updating libcreate (fix-std-string)"
clone_or_update "https://github.com/revyos-ros/libcreate.git" "$SRC_DIR/libcreate" "fix-std-string"

echo "Cloning/updating create_robot"
clone_or_update "https://github.com/autonomylab/create_robot.git" "$SRC_DIR/create_robot" ""

if [ -f "$GW" ]; then
	if [ ! -x "$GW" ]; then
		/bin/bash "$GW"
	else
		"$GW"
	fi
else
	echo "Global workspace setup script not found at $GW"
fi

exit 0
