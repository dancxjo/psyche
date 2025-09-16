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

# --- Service provisioning ---
# Parse host services list (array of service names) from host config
HOST_SERVICES=""
if [ -f "${HOST_CONFIG}" ]; then
	if command -v jq >/dev/null 2>&1; then
		# newline-separated list
		HOST_SERVICES=$(jq -r '.services[]? // empty' "${HOST_CONFIG}" 2>/dev/null || true)
	else
		if command -v python3 >/dev/null 2>&1; then
			HOST_SERVICES=$(python3 - <<'PY'
import json,sys
try:
	cfg=json.load(open(sys.argv[1]))
	for s in cfg.get('services', []):
		print(s)
except Exception:
	pass
PY
 "%s"  "${HOST_CONFIG}")
		else
			echo "[bootstrap] Warning: neither jq nor python3 available; cannot parse host services"
			HOST_SERVICES=""
		fi
	fi
fi

# Helper: run service tool if present. Tool path convention(s):
#  - tools/provision/tools/<service>_tool.py (legacy)
#  - services/<service>/(setup.sh|teardown.sh) (preferred)
run_service_action() {
	local svc="$1" action="$2"
	# service directories live at repo-root `services/<svc>`
	local svc_dir="${SCRIPT_DIR}/../../services/${svc}"
	local tool1="${SCRIPT_DIR}/tools/${svc}_tool.py"
	local tool2="${SCRIPT_DIR}/services/${svc}_tool.py"

	# Prefer service directory scripts (setup.sh/teardown.sh)
	if [ -d "${svc_dir}" ]; then
		local script="${svc_dir}/${action}.sh"
		if [ -x "${script}" ]; then
			echo "[bootstrap] Running ${action} via ${script}"
			"${script}" || echo "[bootstrap] Script ${script} returned non-zero"
			return
		elif [ -f "${script}" ]; then
			echo "[bootstrap] Running ${action} via ${script} (sh)"
			sh "${script}" || echo "[bootstrap] Script ${script} returned non-zero"
			return
		fi
	fi

	if [ -f "${tool1}" ]; then
		echo "[bootstrap] Running ${action} via ${tool1}"
		python3 "${tool1}" "${action}" || echo "[bootstrap] Tool ${tool1} returned non-zero"
		return
	fi
	if [ -f "${tool2}" ]; then
		echo "[bootstrap] Running ${action} via ${tool2}"
		python3 "${tool2}" "${action}" || echo "[bootstrap] Tool ${tool2} returned non-zero"
		return
	fi

	echo "[bootstrap] No tool hook found for service '${svc}' (checked ${tool1} and ${tool2}); skipping"
}

echo "[bootstrap] Processing services in ${SCRIPT_DIR}/../../services"
shopt -s nullglob
for svc_json in "${SCRIPT_DIR}/../../services"/*.json; do
	# get service name from descriptor
	svc_name=""
	if command -v jq >/dev/null 2>&1; then
		svc_name=$(jq -r '.name // empty' "${svc_json}" 2>/dev/null || true)
	else
		if command -v python3 >/dev/null 2>&1; then
			svc_name=$(python3 - <<'PY'
import json,sys
try:
	cfg=json.load(open(sys.argv[1]))
	print(cfg.get('name',''))
except Exception:
	print('')
PY
 "%s"  "${svc_json}")
		fi
	fi
	if [ -z "${svc_name}" ]; then
		echo "[bootstrap] Warning: could not determine service name for ${svc_json}; skipping"
		continue
	fi

	# Determine whether service is enabled on this host
	if printf '%s
' "${HOST_SERVICES}" | grep -xFq "${svc_name}"; then
		echo "[bootstrap] Service ${svc_name} is ENABLED on this host — setting up"
		run_service_action "${svc_name}" setup
	else
		echo "[bootstrap] Service ${svc_name} is NOT enabled on this host — tearing down if present"
		run_service_action "${svc_name}" teardown
	fi
done
shopt -u nullglob





echo "[bootstrap] Completed provisioning bootstrap"
