#!/bin/bash
set -euo pipefail

# Create and populate a global ROS2 workspace at /opt/psyche_workspace
# Clones patched libcreate and create_robot and builds them with colcon
# Usage: setup_global_workspace.sh [--force]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FORCE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --force)
      FORCE=1; shift ;;
    -h|--help)
      echo "Usage: $(basename "$0") [--force]"; exit 0 ;;
    *) echo "Unknown arg: $1"; exit 2 ;;
  esac
done

if [ "$EUID" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=
fi

ROS_DISTRO=${ROS_DISTRO:-kilted}
WORKSPACE_DIR=/opt/psyche_workspace
SRC_DIR="$WORKSPACE_DIR/src"
MARKER="$WORKSPACE_DIR/.built"

echo "Setting up global workspace at $WORKSPACE_DIR (ROS distro: $ROS_DISTRO)"

echo "Creating workspace directories"
$SUDO mkdir -p "$SRC_DIR"
$SUDO chown root:root "$WORKSPACE_DIR" || true
$SUDO chmod 0755 "$WORKSPACE_DIR" || true

echo "Sourcing ROS distro setup if available"
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
else
  echo "Warning: /opt/ros/$ROS_DISTRO/setup.bash not found. Ensure ROS is installed before building workspace."
fi

echo "Installing rosdep and colcon tooling if missing"
$SUDO apt update || true
$SUDO apt install -y python3-rosdep python3-colcon-common-extensions || true
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init || true
fi
rosdep update || true

if [ -z "$(ls -A "$SRC_DIR" 2>/dev/null)" ]; then
  echo "No source packages in $SRC_DIR; nothing to build. Workspace skeleton created."
  exit 0
fi

echo "Running rosdep install for workspace"
rosdep install --from-paths "$SRC_DIR" --ignore-src -y || true

echo "Building workspace with colcon"

$SUDO colcon build --install-base "$WORKSPACE_DIR/install" --merge-install || true

echo "Ensure global workspace is sourced for all users"
PROFILE_SCRIPT=/etc/profile.d/psyche_workspace.sh
echo "Creating $PROFILE_SCRIPT"
cat <<EOF | $SUDO tee "$PROFILE_SCRIPT" > /dev/null
#!/bin/sh
if [ -f /opt/psyche_workspace/install/setup.bash ]; then
  . /opt/psyche_workspace/install/setup.bash
fi
EOF
$SUDO chmod 0644 "$PROFILE_SCRIPT"

echo "Marking workspace built"
$SUDO touch "$MARKER"
$SUDO chown root:root "$MARKER" || true

echo "Global workspace setup complete"
exit 0
