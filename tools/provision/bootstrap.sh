#!/usr/bin/env bash
set -euo pipefail

# bootstrap.sh
# Top-level provisioning bootstrap. This script is executed after the
# repository has been extracted (assumed location: /opt/psyched). It calls
# smaller setup helpers such as `setup_ros2.sh`.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_ROS2_SCRIPT="${SCRIPT_DIR}/setup_ros2.sh"

# Default host config path: allow override via PSYCHED_HOST_CONFIG env var
HOST_CONFIG="${PSYCHED_HOST_CONFIG:-"${SCRIPT_DIR}/../../hosts/$(hostname -s).json"}"

echo "[bootstrap] Running provisioning bootstrap"

if [ ! -f "${SETUP_ROS2_SCRIPT}" ]; then
	echo "[bootstrap] ERROR: setup script not found: ${SETUP_ROS2_SCRIPT}" >&2
	exit 1
fi

echo "[bootstrap] Calling ${SETUP_ROS2_SCRIPT}"
if [ ! -x "${SETUP_ROS2_SCRIPT}" ]; then
	chmod +x "${SETUP_ROS2_SCRIPT}"
fi

# Parse host config to decide whether to install ROS2. The host config can be
# overridden by setting PSYCHED_HOST_CONFIG. Expected JSON structure:
# { "host": "name", "install_ros2": true, ... }
install_ros2=false
if [ -f "${HOST_CONFIG}" ]; then
	echo "[bootstrap] Found host config: ${HOST_CONFIG}"
	# Prefer jq if available
	if command -v jq >/dev/null 2>&1; then
		install_ros2=$(jq -r '.install_ros2 // false' "${HOST_CONFIG}") || install_ros2=false
	else
		# Fallback to python parsing to avoid adding a dependency
		if command -v python3 >/dev/null 2>&1; then
			install_ros2=$(python3 - <<'PY'
import json,sys
try:
	cfg=json.load(open(sys.argv[1]))
	val=cfg.get('install_ros2', False)
	print('true' if val else 'false')
except Exception:
	print('false')
PY
 "%s"  "${HOST_CONFIG}")
		else
			echo "[bootstrap] Warning: neither jq nor python3 available; assuming install_ros2=false"
			install_ros2=false
		fi
	fi
else
	echo "[bootstrap] No host config found at ${HOST_CONFIG}; defaulting to install_ros2=false"
	install_ros2=false
fi

if [ "${install_ros2}" = "true" ] || [ "${install_ros2}" = "True" ]; then
	echo "[bootstrap] install_ros2 is true — running ${SETUP_ROS2_SCRIPT}"
	# If called under sudo, prefer running the setup as the original user so that
	# pip --user and ownership behavior is correct. Preserve environment where safe.
	if [ -n "${SUDO_USER:-}" ]; then
		echo "[bootstrap] Detected sudo, running setup as user: ${SUDO_USER}"
		sudo -E -u "${SUDO_USER}" bash -c "'${SETUP_ROS2_SCRIPT}'"
	else
		bash "${SETUP_ROS2_SCRIPT}"
	fi
else
	echo "[bootstrap] install_ros2 is false — skipping ${SETUP_ROS2_SCRIPT}"
fi





echo "[bootstrap] Completed provisioning bootstrap"
