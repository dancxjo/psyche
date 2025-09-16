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
		# Minimal Kaiju (ROS 2 kilted) setup
		echo "Starting Kaiju (ROS 2 kilted) base setup"

		if [ "$EUID" -ne 0 ]; then
			SUDO=sudo
		else
			SUDO=
		fi

		export DEBIAN_FRONTEND=noninteractive

		echo "Updating apt and installing prerequisites"
		$SUDO apt update
		$SUDO apt install -y locales curl software-properties-common ca-certificates gnupg lsb-release build-essential git python3-pip python3-venv

		echo "Configuring locale to en_US.UTF-8"
		$SUDO apt install -y locales
		$SUDO locale-gen en_US en_US.UTF-8 || true
		$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
		export LANG=en_US.UTF-8

		echo "Enabling universe repository"
		$SUDO add-apt-repository -y universe

		echo "Installing ros2 apt-source package"
		TMP_DEB="/tmp/ros2-apt-source.deb"
		export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
		UBU_CODENAME=$(. /etc/os-release && echo $VERSION_CODENAME)
		curl -fsSL -o "$TMP_DEB" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBU_CODENAME}_all.deb"
		$SUDO dpkg -i "$TMP_DEB" || true

		echo "Installing development packages recommended by ROS 2"
		$SUDO apt update
		$SUDO apt install -y \
			python3-colcon-common-extensions \
			python3-flake8-blind-except \
			python3-flake8-class-newline \
			python3-flake8-deprecated \
			python3-mypy \
			python3-pip \
			python3-pytest \
			python3-pytest-cov \
			python3-pytest-mock \
			python3-pytest-repeat \
			python3-pytest-rerunfailures \
			python3-pytest-runner \
			python3-pytest-timeout \
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
			libpcre3-dev

		echo "Preparing workspace at ~/ros2_kilted/src"
		WORKSPACE_DIR="$HOME/ros2_kilted"
		mkdir -p "$WORKSPACE_DIR/src"
		cd "$WORKSPACE_DIR"

		if [ ! -d src/.rosinstall ]; then
			echo "Importing ROS 2 kilted source manifest (this may take a while)"
			if command -v vcs >/dev/null 2>&1; then
				vcs import --input https://raw.githubusercontent.com/ros2/ros2/kilted/ros2.repos src || true
			else
				echo "vcs tool not found; installing python3-vcstool"
				$SUDO pip3 install -U vcstool
				vcs import --input https://raw.githubusercontent.com/ros2/ros2/kilted/ros2.repos src || true
			fi
		else
			echo "Workspace already has sources, skipping vcs import"
		fi

		echo "Installing rosdep and dependencies"
		$SUDO apt install -y python3-rosdep
		if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
			sudo rosdep init || true
		fi
		rosdep update || true
		rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-7.3.0 urdfdom_headers" || true

		echo "Building workspace (colcon). This may take a long time."
		colcon build --symlink-install || true

		echo "Setup complete. To use Kaiju (kilted) in a shell run:"
		echo "  . $WORKSPACE_DIR/install/local_setup.bash"

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
