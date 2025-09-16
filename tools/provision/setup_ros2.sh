#!/bin/bash
set -euo pipefail

# Provisioning entrypoint: determine host and run host-specific script
# Host scripts live in the hosts/ subdirectory beside this script.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOSTS_DIR="$PROJECT_ROOT/hosts"
if [ ! -d "$HOSTS_DIR" ]; then
	mkdir -p "$HOSTS_DIR"
    echo "Created hosts directory at $HOSTS_DIR; please add host-specific scripts and re-run this script."
    exit 1
fi

host_name="$(hostname --fqdn 2>/dev/null || hostname)"

echo "Provisioning host: $host_name"

host_script="$HOSTS_DIR/$host_name.sh"

if [ -f "$host_script" ]; then
	echo "Found host-specific script: $host_script"
	/bin/bash "$host_script"
else
	echo "No host-specific provisioning script for '$host_name' (looked for $host_script)"
	exit 0
fi

## Subcommands
install_ros2() {
		# Install ROS2 from prebuilt packages (apt) and required dev tooling.
		echo "Starting ROS2 (Kaiju/kilted) prebuilt package installation"

		if [ "$EUID" -ne 0 ]; then
			SUDO=sudo
		else
			SUDO=
		fi

		export DEBIAN_FRONTEND=noninteractive

		echo "Installing prerequisites"
		$SUDO apt update
		$SUDO apt install -y curl gnupg lsb-release software-properties-common locales ca-certificates

		echo "Configuring locale to en_US.UTF-8"
		$SUDO locale-gen en_US en_US.UTF-8 || true
		$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
		export LANG=en_US.UTF-8

		echo "Enabling universe repository"
		$SUDO add-apt-repository -y universe || true

		# Allow override via environment variable, default to 'kilted'
		: "${ROS_DISTRO:=kilted}"

		echo "Adding ROS 2 apt repository and key for distro: $ROS_DISTRO"
		# Install keyring in modern, apt-key-free way
		KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
		$SUDO mkdir -p "$(dirname "$KEYRING")"
		curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | $SUDO gpg --dearmour -o "$KEYRING" || true

		echo "Adding APT source for ROS 2"
		DIST_CODENAME=$(lsb_release -sc)
		echo "deb [signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu $DIST_CODENAME main" | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

		echo "Updating apt and installing ROS 2 base package and development tools"
		$SUDO apt update
		# Install ros base (no desktop) and common development tools
		$SUDO apt install -y "ros-${ROS_DISTRO}-ros-base" \
			python3-colcon-common-extensions \
			python3-pip \
			python3-rosdep \
			build-essential \
			cmake \
			git \
			wget \
			pkg-config \
			python3-dev \
			python3-setuptools \
			libasio-dev \
			libtinyxml2-dev \
			libssl-dev \
			libpcre3-dev || true

		echo "Initializing rosdep"
		if ! command -v rosdep >/dev/null 2>&1; then
		  $SUDO apt install -y python3-rosdep || true
		fi
		if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
		  $SUDO rosdep init || true
		fi
		rosdep update || true

		echo "ROS2 installation complete. To use it in a shell, source the distro setup if available (example):"
		echo "  source /opt/ros/$ROS_DISTRO/setup.bash"

		return 0
}

# If invoked with a subcommand, handle it and exit before host dispatch
if [ $# -ge 1 ]; then
	case "$1" in
		install-ros2|install_ros2)
			install_ros2
			exit $?
			;;
		*)
			echo "Unknown subcommand: $1"
			echo "Usage: $(basename "$0") [install-ros2]"
			exit 2
			;;
	esac
fi
