#!/bin/bash
set -euo pipefail

# Idempotent bootstrap script for provisioning hosts in this repo.
# Usage: bootstrap.sh [--force] [--install-ros2]
# - --force: force rerun of actions even if already applied
# - --install-ros2: run tools/provision/setup_ros2.sh install-ros2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# If the script isn't running from inside a checked-out repository (for example
# when run via `curl ... | sudo bash`), clone the repo into a temporary
# directory and run provisioning from that clone. This makes the bootstrap
# launcher safe to run directly on a host.
if [ ! -f "$PROJECT_ROOT/tools/provision/setup_ros2.sh" ]; then
  echo "Repository files not found next to bootstrap script; cloning repo to temporary dir"
  TMP_CLONE_DIR=$(mktemp -d /tmp/psyched-bootstrap.XXXX)
  echo "Cloning https://github.com/dancxjo/psyched.git to $TMP_CLONE_DIR"
  if command -v git >/dev/null 2>&1; then
    git clone --depth 1 https://github.com/dancxjo/psyched.git "$TMP_CLONE_DIR" || {
      echo "git clone failed" >&2; exit 1
    }
    PROJECT_ROOT="$TMP_CLONE_DIR"
    echo "Using cloned project at $PROJECT_ROOT"
  else
    echo "git not available; cannot clone repository. Please run from a checkout or install git." >&2
    exit 1
  fi
fi
HOSTNAME_FULL="$(hostname --fqdn 2>/dev/null || hostname)"
STATE_DIR="/var/lib/psyched_bootstrap"
STATE_FILE="$STATE_DIR/$HOSTNAME_FULL.state"

FORCE=0
DO_INSTALL_ROS2=0

while [ $# -gt 0 ]; do
  case "$1" in
    --force)
      FORCE=1
      shift
      ;;
    --install-ros2)
      DO_INSTALL_ROS2=1
      shift
      ;;
    -h|--help)
      echo "Usage: $(basename "$0") [--force] [--install-ros2]"
      exit 0
      ;;
    *)
      echo "Unknown arg: $1"
      echo "Usage: $(basename "$0") [--force] [--install-ros2]"
      exit 2
      ;;
  esac
done

if [ "$EUID" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=
fi

echo "Bootstrap starting for host: $HOSTNAME_FULL"

# Ensure state dir exists
$SUDO mkdir -p "$STATE_DIR"
$SUDO chown $(id -u):$(id -g) "$STATE_DIR"

# Ensure system user 'pete' exists (system account, no home)
if ! id -u pete >/dev/null 2>&1; then
  echo "Creating system user 'pete'"
  $SUDO useradd --system --no-create-home --shell /usr/sbin/nologin pete || true
fi

state_current=""
if [ -f "$STATE_FILE" ]; then
  state_current=$(cat "$STATE_FILE") || true
fi

echo "Current state: ${state_current:-<none>}"

mark_state() {
  local newstate="$1"
  echo "$newstate" > "$STATE_FILE"
  $SUDO chown root:root "$STATE_FILE" || true
  $SUDO chmod 0644 "$STATE_FILE" || true
}

# If requested, run ros2 install via the project-level installer
if [ "$DO_INSTALL_ROS2" -eq 1 ]; then
  echo "--install-ros2 requested"
  if [ "$FORCE" -eq 0 ] && [ "$state_current" = "ros2-installed" ]; then
    echo "ros2 already installed (state file), skipping. Use --force to reinstall."
  else
    echo "Running install-ros2 via tools/provision/setup_ros2.sh"
    /bin/bash "$PROJECT_ROOT/tools/provision/setup_ros2.sh" install-ros2
    mark_state "ros2-installed"
  fi
fi

echo "Bootstrap finished"
exit 0
