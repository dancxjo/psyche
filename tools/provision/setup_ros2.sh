#!/usr/bin/env bash
set -euo pipefail

# setup_ros2.sh
# Robust, idempotent installer for ROS APT source, ROS packages, and a global
# workspace at /opt/ros2_ws. Creates a profile script in /etc/profile.d so all
# users will automatically have the workspace environment and RMW settings.

DEFAULT_ROS_APT_SOURCE_REPO="https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest"
TMP_DEB="/tmp/ros2-apt-source.deb"
WS_DIR="/opt/ros2_ws"
PROFILE_D_FILE="/etc/profile.d/psyched_workspace.sh"

# Allow overriding from environment when provisioning (e.g. CI or different distro)
ROS_APT_SOURCE_VERSION_OVERRIDE="${ROS_APT_SOURCE_VERSION_OVERRIDE:-}" # set to full tag like v0.1.0 if desired
ROS_DISTRO="${ROS_DISTRO:-kilted}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "[setup_ros2] Starting ROS2 setup (workspace: ${WS_DIR})"

# Ensure we can run apt commands non-interactively
export DEBIAN_FRONTEND=noninteractive

# Ensure locale supports UTF-8 (best-effort)
if ! locale -a 2>/dev/null | grep -iq "en_US.utf-8"; then
	echo "[setup_ros2] Generating en_US.UTF-8 locale"
	sudo apt-get update -y
	sudo apt-get install -y locales
	sudo locale-gen en_US.UTF-8 || true
fi

# Ensure the Ubuntu Universe repository is enabled (idempotent)
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends software-properties-common curl
if ! grep -E '^deb .* universe' /etc/apt/sources.list /etc/apt/sources.list.d/* 2>/dev/null | grep -q .; then
	echo "[setup_ros2] Enabling 'universe' repository"
	sudo add-apt-repository -y universe
fi

sudo apt-get update -y

# Fetch the latest ros-apt-source release tag unless overridden
if [ -z "${ROS_APT_SOURCE_VERSION_OVERRIDE}" ]; then
	echo "[setup_ros2] Detecting latest ros-apt-source release"
	ROS_APT_SOURCE_VERSION=$(curl -s ${DEFAULT_ROS_APT_SOURCE_REPO} | grep -F "tag_name" | awk -F\" '{print $4}')
else
	ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_VERSION_OVERRIDE}"
fi

# Determine distro codename for package filename
if [ -r /etc/os-release ]; then
	. /etc/os-release
	CODENAME="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
fi
if [ -z "${CODENAME}" ]; then
	echo "[setup_ros2] Warning: could not determine OS codename; defaulting to 'focal'"
	CODENAME="focal"
fi

DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
echo "[setup_ros2] Downloading: ${DEB_URL}"
curl -fsSL -o "${TMP_DEB}" "${DEB_URL}"
sudo dpkg -i "${TMP_DEB}" || sudo apt-get -f install -y

# Install ROS packages (idempotent)
sudo apt-get update -y
echo "[setup_ros2] Installing ROS packages (distro: ${ROS_DISTRO})"
sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base || true

# Install recommended dev tools
sudo apt-get install -y --no-install-recommends ros-dev-tools || true

# Install the RMW implementation package
echo "[setup_ros2] Installing RMW implementation: ${RMW_IMPLEMENTATION}"
sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp || true

# Create a global workspace for our packages
sudo mkdir -p "${WS_DIR}/src"
sudo chown -R "${SUDO_USER:-$USER}":"${SUDO_USER:-$USER}" "${WS_DIR}"

# Install colcon and other Python tools into user pip (non-root)
echo "[setup_ros2] Installing colcon and python tools into user site-packages"
python3 -m pip install --upgrade --user colcon-common-extensions colcon-mixin colcon-ros || true

# Create a profile.d file so all users source the workspace and get RMW env
echo "[setup_ros2] Writing global profile at ${PROFILE_D_FILE}"
sudo tee "${PROFILE_D_FILE}" > /dev/null <<EOF
# psyched global ROS2 workspace configuration
export PSYCHED_WS="${WS_DIR}"
# Add user's local python bin (where pip --user installs colcon) to PATH if present
if [ -d "\$HOME/.local/bin" ]; then
	case ":\$PATH:" in
		*:\$HOME/.local/bin:*) ;;
		*) export PATH="\$HOME/.local/bin:\$PATH" ;;
	esac
fi
# Set the RMW implementation unless already set
if [ -z "\${RMW_IMPLEMENTATION:-}" ]; then
	export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
fi
# If workspace overlay contains ROS setup.*sh, source it automatically
if [ -f "\${PSYCHED_WS}/install/setup.bash" ]; then
	source "\${PSYCHED_WS}/install/setup.bash"
elif [ -f "\${PSYCHED_WS}/install/setup.sh" ]; then
	source "\${PSYCHED_WS}/install/setup.sh"
fi
EOF

sudo chmod 644 "${PROFILE_D_FILE}"

echo "[setup_ros2] Completed. Users will get the workspace and RMW via ${PROFILE_D_FILE} on next login."
