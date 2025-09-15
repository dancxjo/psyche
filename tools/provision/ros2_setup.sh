#!/usr/bin/env bash
# Install ROS 2 base, create a global workspace, and set up environment.
set -euo pipefail

DISTRO="${1:-jazzy}"

log() { echo "[ros2] $*"; }
require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

add_repos_and_keys() {
  if command -v ros2 >/dev/null 2>&1; then
    log "ROS 2 already installed"
    return 0
  fi
  . /etc/os-release
  log "Configuring ROS 2 apt repo for $ID $VERSION_CODENAME"
  apt-get update -y
  apt-get install -y --no-install-recommends curl gnupg lsb-release
  mkdir -p /etc/apt/keyrings
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" > /etc/apt/sources.list.d/ros2.list
}

install_base() {
  log "Installing ros-$DISTRO-ros-base and tools"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-$DISTRO-ros-base python3-colcon-common-extensions python3-rosdep \
    build-essential cmake gcc g++ ninja-build pkg-config
  rosdep init || true
  rosdep update || true
}

make_workspace() {
  if [ ! -d /opt/ros_ws/src ]; then
    log "Creating workspace at /opt/ros_ws"
    mkdir -p /opt/ros_ws/src
  fi
  chgrp -R sudo /opt/ros_ws && chmod -R g+rwX /opt/ros_ws && find /opt/ros_ws -type d -exec chmod g+s {} +
}

main() {
  require_root
  add_repos_and_keys || true
  install_base || true
  make_workspace
}

main "$@"

