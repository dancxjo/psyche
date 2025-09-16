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
# New KISS host config (TOML) path (preferred)
HOST_CONFIG_TOML="${SCRIPT_DIR}/../../provision/hosts/$(hostname -s).toml"

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

# Detect if new TOML KISS scaffolding is present
USE_TOML=false
if [ -f "${HOST_CONFIG_TOML}" ]; then
	USE_TOML=true
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

# Parse host config to decide whether to install ROS2.
#!/usr/bin/env bash
set -euo pipefail

# KISS bootstrap for Psyched
# - Assumes repo is installed at /opt/psyched (via tools/install.sh)
# - Ensures CLI and provision scripts are executable
# - Optionally applies host services and installs systemd units

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Root of installed tree (prefer /opt/psyched; fallback to repo-relative)
ROOT="${PSYCHED_ROOT:-/opt/psyched}"
if [ ! -d "${ROOT}" ]; then
  ROOT="$(cd "${SCRIPT_DIR}/../../" && pwd)"
fi

CLI="${ROOT}/cli/psy"
HOSTCFG_DIR="${ROOT}/provision/hosts"
SVCDIR="${ROOT}/provision/services"
SYSTEMD_INSTALL="${ROOT}/provision/systemd/install_units.sh"

# Parse flags
APPLY=false
HOSTNAME_SHORT="$(hostname -s)"
HOST="${HOSTNAME_SHORT}"
while [ $# -gt 0 ]; do
  case "$1" in
    --apply|-a) APPLY=true ;;
    --host|-H) HOST="${2:-${HOST}}"; shift ;;
    *) echo "[bootstrap] Unknown arg: $1" >&2 ;;
  esac
  shift
done

echo "[bootstrap] KISS bootstrap start (root=${ROOT}, host=${HOST}, apply=${APPLY})"

# Ensure executables
if [ -f "${CLI}" ]; then chmod +x "${CLI}" || true; fi
if [ -d "${ROOT}/provision" ]; then
  find "${ROOT}/provision" -type f -name "*.sh" -exec chmod +x {} + 2>/dev/null || true
fi

# Determine host config (TOML)
CFG_TOML="${HOSTCFG_DIR}/${HOST}.toml"
if [ ! -f "${CFG_TOML}" ]; then
  # Fallback to cerebellum if present
  if [ -f "${HOSTCFG_DIR}/cerebellum.toml" ]; then
    CFG_TOML="${HOSTCFG_DIR}/cerebellum.toml"
    HOST="cerebellum"
  fi
fi

if [ ! -f "${CFG_TOML}" ]; then
  echo "[bootstrap] ERROR: host config not found: ${HOSTCFG_DIR}/${HOST}.toml" >&2
  echo "[bootstrap] Create one (copy cerebellum.toml) or pass --host <name>." >&2
  exit 1
fi

echo "[bootstrap] Using host config: ${CFG_TOML}"

if [ "${APPLY}" = true ]; then
  # Apply services for host (provisions ROS, workspace, sensors, etc.)
  if [ -x "${CLI}" ]; then
    "${CLI}" host apply "${HOST}"
  else
    echo "[bootstrap] ERROR: CLI not found at ${CLI}" >&2
    exit 1
  fi
  # Install templated systemd units and enable those with launchers
  if [ -x "${SYSTEMD_INSTALL}" ]; then
    "${SYSTEMD_INSTALL}"
  else
    echo "[bootstrap] Warning: systemd install script not executable: ${SYSTEMD_INSTALL}" >&2
  fi
  # One safe build pass (services may already trigger a build)
  "${CLI}" build || true
  echo "[bootstrap] Apply complete for host=${HOST}"
else
  cat <<MSG
[bootstrap] Ready. Next steps:
  sudo ${CLI} host apply ${HOST}
  sudo ${CLI} systemd install
  ${CLI} build
  # Then, to run nav2 for testing:
  ${CLI} bringup nav
MSG
fi

echo "[bootstrap] Done"
		HOST_SERVICES=""
