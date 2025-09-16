#!/usr/bin/env bash
set -euo pipefail

# bootstrap.sh
# Top-level provisioning bootstrap. This script is executed after the
# repository has been extracted (assumed location: /opt/psyched). It calls
# smaller setup helpers such as `setup_ros2.sh`.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_ROS2_SCRIPT="${SCRIPT_DIR}/setup_ros2.sh"

echo "[bootstrap] Running provisioning bootstrap"

if [ ! -f "${SETUP_ROS2_SCRIPT}" ]; then
	echo "[bootstrap] ERROR: setup script not found: ${SETUP_ROS2_SCRIPT}" >&2
	exit 1
fi

echo "[bootstrap] Calling ${SETUP_ROS2_SCRIPT}"
if [ ! -x "${SETUP_ROS2_SCRIPT}" ]; then
	chmod +x "${SETUP_ROS2_SCRIPT}"
fi

# If called under sudo, prefer running the setup as the original user so that
# pip --user and ownership behavior is correct. Preserve environment where safe.
if [ -n "${SUDO_USER:-}" ]; then
	echo "[bootstrap] Detected sudo, running setup as user: ${SUDO_USER}"
	sudo -E -u "${SUDO_USER}" bash -c "'${SETUP_ROS2_SCRIPT}'"
else
	bash "${SETUP_ROS2_SCRIPT}"
fi





echo "[bootstrap] Completed provisioning bootstrap"
