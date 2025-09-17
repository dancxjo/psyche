#!/usr/bin/env bash
# Common helpers for psyche service provision scripts
# Usage: source this file from a service script in the same folder.

# Do not set -euo here; let the caller control shell options.

# Root paths
export PSY_ROOT="${PSY_ROOT:-/opt/psyched}"
export PSY_WS="${PSY_WS:-$PSY_ROOT/ws}"
export WS="$PSY_WS"
export SRC="$PSY_WS/src"

# Apt install queue (to combine all apt-get into one call at the end)
export PSY_APT_QUEUE_FILE="${PSY_APT_QUEUE_FILE:-$PSY_ROOT/provision/.apt_queue}"
export PSY_DEFER_APT="${PSY_DEFER_APT:-0}"

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
  # If deferring, queue packages; otherwise install immediately
  if [ "$PSY_DEFER_APT" = "1" ]; then
    mkdir -p "$(dirname "$PSY_APT_QUEUE_FILE")"
    # shellcheck disable=SC2068
    for pkg in $@; do
      echo "$pkg" >> "$PSY_APT_QUEUE_FILE"
    done
  else
    sudo apt-get update -y || true
    # shellcheck disable=SC2068
    sudo apt-get install -y $@ || true
  fi
}

common_flush_apt_queue() {
  [ -f "$PSY_APT_QUEUE_FILE" ] || { echo "[psy] No queued apt packages"; return 0; }
  mapfile -t ALL < <(sed '/^\s*$/d' "$PSY_APT_QUEUE_FILE" | sort -u)
  if [ ${#ALL[@]} -eq 0 ]; then
    echo "[psy] Apt queue empty"
    return 0
  fi
  # Split required vs optional (prefix ?pkg for optional)
  REQUIRED=()
  OPTIONAL=()
  for p in "${ALL[@]}"; do
    if [[ "$p" == \?* ]]; then
      OPTIONAL+=("${p#?}")
    else
      REQUIRED+=("$p")
    fi
  done
  echo "[psy] Installing ${#REQUIRED[@]} required packages in one transaction"
  sudo apt-get update -y || true
  if [ ${#REQUIRED[@]} -gt 0 ]; then
    sudo apt-get install -y "${REQUIRED[@]}" || true
  fi
  # Install optional packages one by one; ignore failures
  if [ ${#OPTIONAL[@]} -gt 0 ]; then
    echo "[psy] Installing ${#OPTIONAL[@]} optional packages (best effort)"
    for op in "${OPTIONAL[@]}"; do
      sudo apt-get install -y "$op" || true
    done
  fi
  rm -f "$PSY_APT_QUEUE_FILE" || true
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
    common_apt_install python3-numpy python3-dev '?python3-numpy-dev'
  fi
}

common_enable_ccache() {
  # Queue ccache install and set CCACHE_DIR
  common_apt_install ccache
  export CCACHE_DIR="${CCACHE_DIR:-$PSY_ROOT/.ccache}"
  mkdir -p "$CCACHE_DIR" 2>/dev/null || true
}
