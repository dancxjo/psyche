#!/usr/bin/env bash
set -euo pipefail

# bootstrap.sh
# Single-responsibility bootstrap: make CLI available system-wide, ensure ROS
# APT source is configured, then delegate provisioning to `psy host apply`.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${PSYCHED_ROOT:-/opt/psyched}"
[ -d "${ROOT}" ] || ROOT="$(cd "${SCRIPT_DIR}/../../" && pwd)"
CLI="${ROOT}/cli/psy"
CLI_ALT="${ROOT}/cli/psh"
HOSTCFG_DIR="${ROOT}/provision/hosts"
SETUP_ROS2_SCRIPT="${SCRIPT_DIR}/setup_ros2.sh"

# Optional: create a shared Python venv once (best-effort)
ensure_python_env() {
	VENV_DIR="${PSYCHED_VENV:-/opt/psyched_venv}"
	export PSYCHED_VENV="${VENV_DIR}"
	command -v python3 >/dev/null 2>&1 || return 0
	if [ ! -d "${VENV_DIR}" ]; then
		echo "[bootstrap] Creating shared venv at ${VENV_DIR}"
		mkdir -p "$(dirname "${VENV_DIR}")" || true
		if python3 -m venv "${VENV_DIR}" 2>/dev/null; then
			[ -n "${SUDO_USER:-}" ] && chown -R "${SUDO_USER}:${SUDO_USER}" "${VENV_DIR}" || true
		fi
	fi
	if [ -x "${VENV_DIR}/bin/pip" ]; then
		echo "[bootstrap] Upgrading pip/setuptools/wheel in ${VENV_DIR}"
		"${VENV_DIR}/bin/pip" install --upgrade pip setuptools wheel || true
		export PATH="${VENV_DIR}/bin:${PATH}"
	fi
}

# Parse flags
APPLY=false
HOST="$(hostname -s)"
while [ $# -gt 0 ]; do
	case "$1" in
		--apply|-a) APPLY=true ;;
		--host|-H) HOST="${2:-${HOST}}"; shift ;;
		*) echo "[bootstrap] Unknown arg: $1" >&2 ;;
	esac
	shift
done

echo "[bootstrap] KISS bootstrap start (root=${ROOT}, host=${HOST}, apply=${APPLY})"

# Ensure scripts are executable
[ -f "${CLI}" ] && chmod +x "${CLI}" || true
[ -f "${CLI_ALT}" ] && chmod +x "${CLI_ALT}" || true
if [ -d "${ROOT}/provision" ]; then
	find "${ROOT}/provision" -type f -name "*.sh" -exec chmod +x {} + 2>/dev/null || true
fi

# Link CLI into PATH for convenience (idempotent)
if [ -f "${CLI}" ]; then
	if [ "$(readlink -f /usr/bin/psy 2>/dev/null || true)" != "${CLI}" ]; then
		echo "[bootstrap] Linking ${CLI} -> /usr/bin/psy"
		sudo ln -sf "${CLI}" /usr/bin/psy || true
	fi
	if [ "$(readlink -f /usr/bin/psh 2>/dev/null || true)" != "${CLI}" ]; then
		echo "[bootstrap] Linking ${CLI} -> /usr/bin/psh"
		sudo ln -sf "${CLI}" /usr/bin/psh || true
	fi
fi

# Ensure a host config exists
CFG_TOML="${HOSTCFG_DIR}/${HOST}.toml"
if [ ! -f "${CFG_TOML}" ] && [ -f "${HOSTCFG_DIR}/cerebellum.toml" ]; then
	CFG_TOML="${HOSTCFG_DIR}/cerebellum.toml"; HOST="cerebellum"
fi
if [ ! -f "${CFG_TOML}" ]; then
	echo "[bootstrap] ERROR: host config not found: ${HOSTCFG_DIR}/${HOST}.toml" >&2
	echo "[bootstrap] Create one (copy cerebellum.toml) or pass --host <name>." >&2
	exit 1
fi
echo "[bootstrap] Using host config: ${CFG_TOML}"

# Minimal ROS APT source setup (repo-only; skip installing packages/profile here)
if [ -x "${SETUP_ROS2_SCRIPT}" ]; then
	echo "[bootstrap] Calling ${SETUP_ROS2_SCRIPT} (repo-only)"
	PSY_SETUP_ROS2_REPO_ONLY=1 PSY_SETUP_ROS2_SKIP_PROFILE=1 bash "${SETUP_ROS2_SCRIPT}" || true
fi

ensure_python_env || true

if [ "${APPLY}" = true ]; then
	# Delegate full provisioning/build/systemd to CLI once
	if [ -x "${CLI}" ]; then
		"${CLI}" host apply "${HOST}"
	else
		echo "[bootstrap] ERROR: CLI not found at ${CLI}" >&2
		exit 1
	fi
	echo "[bootstrap] Apply complete for host=${HOST}"
else
	cat <<MSG
[bootstrap] Ready. Next steps:
	sudo ${CLI} host apply ${HOST}
	# Optional: then bring up nav for testing
	${CLI} bringup nav
MSG
fi

echo "[bootstrap] Done"
