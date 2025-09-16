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

# Ensure Python tooling is available and create a shared venv for safe installs
ensure_python_env() {
	VENV_DIR="${PSYCHED_VENV:-/opt/psyched_venv}"
	export PSYCHED_VENV="${VENV_DIR}"

	if ! command -v python3 >/dev/null 2>&1; then
		echo "[bootstrap] ERROR: python3 is not installed. Please install python3." >&2
		return 1
	fi

	if ! command -v pip3 >/dev/null 2>&1; then
		if [ "$(id -u)" -eq 0 ]; then
			echo "[bootstrap] pip3 not found — installing python3-pip, python3-venv and build deps via apt"
			apt-get update -y
			# python3-distutils may be unavailable on some platforms; avoid installing it directly.
			apt-get install -y python3-pip python3-venv python3-setuptools build-essential || true
		else
			echo "[bootstrap] Warning: pip3 not found and not running as root; please install pip3 or run under sudo" >&2
		fi
	fi

	if [ ! -d "${PSYCHED_VENV}" ]; then
		echo "[bootstrap] Creating shared venv at ${PSYCHED_VENV}"
		# Ensure parent dir exists
		parent_dir="$(dirname "${PSYCHED_VENV}")"
		if [ ! -d "${parent_dir}" ]; then
			mkdir -p "${parent_dir}" || true
		fi
		# Try to create venv. If running as root, create as root then chown to SUDO_USER so ownership is sane.
		if [ "$(id -u)" -eq 0 ]; then
			# create venv as root
			if ! python3 -m venv "${PSYCHED_VENV}"; then
				echo "[bootstrap] ERROR: failed to create venv at ${PSYCHED_VENV}" >&2
			else
				# if we were invoked via sudo, change ownership to the original user
				if [ -n "${SUDO_USER:-}" ]; then
					chown -R "${SUDO_USER}:${SUDO_USER}" "${PSYCHED_VENV}" || true
				fi
			fi
		else
			if ! python3 -m venv "${PSYCHED_VENV}"; then
				echo "[bootstrap] ERROR: failed to create venv at ${PSYCHED_VENV} — permission denied or missing parent dir" >&2
			fi
		fi
	fi

	# Ensure pip is available in the venv; try ensurepip fallback then upgrade
	if [ -f "${PSYCHED_VENV}/bin/python" ]; then
		if ! "${PSYCHED_VENV}/bin/python" -m pip --version >/dev/null 2>&1; then
			# attempt to bootstrap pip into the venv
			"${PSYCHED_VENV}/bin/python" -m ensurepip --upgrade >/dev/null 2>&1 || true
		fi
		if [ -x "${PSYCHED_VENV}/bin/pip" ]; then
			echo "[bootstrap] Upgrading pip, setuptools and wheel inside ${PSYCHED_VENV}"
			"${PSYCHED_VENV}/bin/pip" install --upgrade pip setuptools wheel || true
		else
			echo "[bootstrap] Warning: ${PSYCHED_VENV}/bin/pip not found after ensurepip; venv may be broken" >&2
		fi
	else
		echo "[bootstrap] Warning: ${PSYCHED_VENV}/bin/python not found; venv creation may have failed" >&2
	fi

	# Export venv so child scripts can use it
	export PSYCHED_VENV
	export PATH="${PSYCHED_VENV}/bin:${PATH}"
}

ensure_python_env || true

# Parse flags: support --apply to make bootstrap perform actions
APPLY=false
for a in "$@"; do
	if [ "${a}" = "--apply" ]; then
		APPLY=true
	fi
done
if [ "${APPLY}" = true ]; then
	export APPLY_FLAG="--apply"
else
	export APPLY_FLAG=""
fi

echo "[bootstrap] Running provisioning bootstrap (apply=${APPLY})"

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
		# Preserve PSYCHED_VENV and PATH for the child environment so setup_ros2.sh
		# can use the shared venv. If sudo supports -E to preserve env, use it.
		if sudo -n true 2>/dev/null; then
			# When sudo doesn't clear env, try to export PSYCHED_VENV and PATH explicitly
			sudo -E -u "${SUDO_USER}" env "PSYCHED_VENV=${PSYCHED_VENV}" "PATH=${PSYCHED_VENV}/bin:${PATH}" bash -c "'${SETUP_ROS2_SCRIPT}'"
		else
			# Fallback: run as user with env assignment
			sudo -u "${SUDO_USER}" env "PSYCHED_VENV=${PSYCHED_VENV}" "PATH=${PSYCHED_VENV}/bin:${PATH}" bash -c "'${SETUP_ROS2_SCRIPT}'"
		fi
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
			echo "[bootstrap] Running ${action} via ${script} ${APPLY_FLAG}"
			"${script}" ${APPLY_FLAG} || echo "[bootstrap] Script ${script} returned non-zero"
			return
		elif [ -f "${script}" ]; then
			echo "[bootstrap] Running ${action} via ${script} (sh) ${APPLY_FLAG}"
			sh "${script}" ${APPLY_FLAG} || echo "[bootstrap] Script ${script} returned non-zero"
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
