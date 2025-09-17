#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
CFG="$ROOT/provision/hosts/$(hostname).toml"

ros_toml() {
  awk -F'=' '/ros_distro/{gsub(/[ "]/ ,"",$2);print $2}' "$CFG" 2>/dev/null || true
}
get() { awk -F'=' -v k="$1" '$1~k{gsub(/[ "]/ ,"",$2);print $2}' "$CFG" 2>/dev/null || true; }

provision() {
  ROS_DISTRO="$(ros_toml || echo jazzy)"
  sudo apt-get update -y
  sudo apt-get install -y curl gnupg lsb-release
  sudo apt-get install -y software-properties-common
  sudo apt-get install -y python3-pip python3-venv python3-numpy python3-numpy-dev python3-dev python3-distutils python3-setuptools python3-rosdep ros-dev-tools || \
  sudo apt-get install -y python3-pip python3-venv python3-numpy python3-dev python3-distutils python3-setuptools python3-rosdep ros-dev-tools
  # Try distro colcon extensions; ignore if not found, we'll fallback to pip
  sudo apt-get install -y python3-colcon-common-extensions || true
  # Fallback: install colcon via pip if not present
  if ! command -v colcon >/dev/null 2>&1; then
    python3 -m pip install --upgrade --user pip wheel 'setuptools<80' || true
    python3 -m pip install --upgrade --user colcon-common-extensions colcon-ros colcon-mixin || true
  fi
  # Initialize rosdep globally (idempotent)
  sudo rosdep init 2>/dev/null || true
  rosdep update || true
  # ROS 2 base and CycloneDDS
  sudo apt-get install -y "ros-${ROS_DISTRO}-ros-base" "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
  # Nice to have packages
  sudo apt-get install -y "ros-${ROS_DISTRO}-diagnostic-updater" \
                          "ros-${ROS_DISTRO}-xacro" \
                          "ros-${ROS_DISTRO}-image-transport" \
                          "ros-${ROS_DISTRO}-image-transport-plugins" \
                          "ros-${ROS_DISTRO}-nav2-bringup" \
                          "ros-${ROS_DISTRO}-tf2-ros"
  # Environment
  grep -q "RMW_IMPLEMENTATION" ~/.bashrc || {
    echo "export RMW_IMPLEMENTATION=$(get rmw || echo rmw_cyclonedds_cpp)" >> ~/.bashrc
  }
  grep -q "ROS_DOMAIN_ID" ~/.bashrc || {
    echo "export ROS_DOMAIN_ID=$(get domain_id || echo 42)" >> ~/.bashrc
  }
  # Replace any existing plain source lines with a safe wrapper, else append wrapper
  SAFE_LINE='set +u; [ -f "/opt/ros/'"${ROS_DISTRO}"'/setup.bash" ] && source "/opt/ros/'"${ROS_DISTRO}"'/setup.bash"; set -u'
  if grep -qE '^\s*source\s+/opt/ros/'"${ROS_DISTRO}"'/setup.bash' ~/.bashrc; then
    sed -i "s#^\s*source\s\+/opt/ros/${ROS_DISTRO}/setup.bash.*#${SAFE_LINE}#" ~/.bashrc || true
  else
    grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc || echo "$SAFE_LINE" >> ~/.bashrc
  fi

  # Global environment for all users via /etc/profile.d
  GLOBAL_ENV="/etc/profile.d/psyched_env.sh"
  sudo tee "$GLOBAL_ENV" >/dev/null <<'ENV'
# psyched global ROS environment and helpers

# Set RMW implementation and domain if not already set
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Source ROS distro environment if present
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
  . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

# Source workspace overlay if present
if [ -f "/opt/psyched/ws/install/setup.bash" ]; then
  . "/opt/psyched/ws/install/setup.bash"
fi

# Convenience aliases
alias ros_source='[ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ] && . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"'
alias psy_source='[ -f "/opt/psyched/ws/install/setup.bash" ] && . "/opt/psyched/ws/install/setup.bash"'
alias resource='ros_source && psy_source'
alias pbuild='here=$(pwd) && cd /opt/psyched && ./cli/psy build && resource && cd "$here"'
ENV
  sudo chmod 644 "$GLOBAL_ENV"

  # Ensure interactive shells source the global env profile
  BASH_GLOBAL_FILE="/etc/bash.bashrc"
  if [ -f "$BASH_GLOBAL_FILE" ]; then
    if ! grep -Fq "$GLOBAL_ENV" "$BASH_GLOBAL_FILE" 2>/dev/null; then
      sudo bash -c "cat >> '$BASH_GLOBAL_FILE' <<'BASH_SRC'
# Source psyched global environment so interactive shells get ROS env
if [ -f '/etc/profile.d/psyched_env.sh' ]; then
  . '/etc/profile.d/psyched_env.sh'
fi
BASH_SRC"
    fi
  fi
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown subcmd" >&2; exit 1;;
esac
