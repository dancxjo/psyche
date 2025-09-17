#!/usr/bin/env bash
# Common helpers for psyche service provision scripts
# Usage: source this file from a service script in the same folder.

# Do not set -euo here; let the caller control shell options.

# Root paths
export PSY_ROOT="${PSY_ROOT:-/opt/psyched}"
export PSY_WS="${PSY_WS:-$PSY_ROOT/ws}"
export WS="$PSY_WS"
export SRC="$PSY_WS/src"

common_safe_source_ros() {
  # Avoid nounset errors from ROS setup using AMENT_TRACE_SETUP_FILES
  set +u
  [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" || true
  set -u
}

common_ensure_ws() {
  mkdir -p "$SRC"
  touch "$WS/.colcon_keep" 2>/dev/null || true
}

common_apt_install() {
  # Idempotent apt install with update; ignore failures per-package
  sudo apt-get update -y || true
  # shellcheck disable=SC2068
  sudo apt-get install -y $@ || true
}

common_clone_repo() {
  # args: <repo_url> <dest_path> [branch]
  local url="$1" dest="$2" branch="${3:-}"
  if [ ! -d "$dest" ]; then
    if [ -n "$branch" ]; then
      git clone --branch "$branch" "$url" "$dest" || true
    else
      git clone "$url" "$dest" || true
    fi
  fi
}

common_disable_in_colcon() {
  # args: <path>
  local path="$1"
  [ -d "$path" ] && touch "$path/COLCON_IGNORE" || true
}

common_write_udev_rules() {
  # args: <rules_file_path> <heredoc_marker>
  local rules_path="$1" marker="$2"
  sudo tee "$rules_path" >/dev/null <<$marker
$(cat)
$marker
  sudo udevadm control --reload-rules && sudo udevadm trigger || true
}

common_install_launcher() {
  # args: <name> <heredoc_marker>
  local name="$1" marker="$2"
  sudo mkdir -p /etc/psyched
  sudo tee "/etc/psyched/${name}.launch.sh" >/dev/null <<$marker
$(cat)
$marker
  sudo chmod +x "/etc/psyched/${name}.launch.sh"
}

common_ensure_numpy() {
  if ! /usr/bin/python3 -c 'import numpy' >/dev/null 2>&1; then
    common_apt_install python3-numpy python3-dev
    sudo apt-get install -y python3-numpy-dev || true
  fi
}
