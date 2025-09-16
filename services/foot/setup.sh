#!/usr/bin/env bash
set -euo pipefail

# Foot service setup (repo-level)
# Usage: services/foot/setup.sh [--apply]
DRY_RUN=true
if [ "${1:-}" = "--apply" ]; then
    DRY_RUN=false
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="${SCRIPT_DIR}/manifest.json"
CANONICAL_ROOT="/opt/psyched"
WORKSPACE_SRC="/opt/psyched_workspace/src"
WORKSPACE_ROOT="/opt/psyched_workspace"

run() {
    echo "${DRY_RUN:+DRY-RUN: }$*"
    if [ "$DRY_RUN" = false ]; then
        eval "$@"
    fi
}

# Create canonical and workspace dirs
run mkdir -p "${CANONICAL_ROOT}"
run mkdir -p "${WORKSPACE_SRC}"

# Clone/update repos
for repo in $(jq -c '.repos[]' "${MANIFEST}"); do
    url=$(echo "$repo" | jq -r '.url')
    relpath=$(echo "$repo" | jq -r '.relpath')
    branch=$(echo "$repo" | jq -r '.branch // empty')
    dest="${CANONICAL_ROOT}/${relpath}"
    if [ -d "$dest/.git" ]; then
        run git -C "$dest" fetch --all
        if [ -n "$branch" ]; then
            run git -C "$dest" checkout "$branch"
            run git -C "$dest" pull --ff-only origin "$branch"
        else
            run git -C "$dest" pull
        fi
    else
        run git clone "$url" "$dest"
        if [ -n "$branch" ]; then
            run git -C "$dest" checkout "$branch"
        fi
    fi
    # Link into workspace src
    dest_link="${WORKSPACE_SRC}/${relpath}"
    if [ -e "$dest_link" ]; then
        run rm -rf "$dest_link"
    fi
    run ln -s "$dest" "$dest_link"
done

# Run rosdep as non-root (attempt to run as the original sudo user if present)
if [ "$DRY_RUN" = false ]; then
    if [ -n "${SUDO_USER:-}" ]; then
        echo "Running rosdep as ${SUDO_USER}"
        sudo -E -u "${SUDO_USER}" -H bash -c "cd ${WORKSPACE_ROOT} && rosdep update && rosdep install --from-paths src --ignore-src -r -y"
    else
        echo "Running rosdep as current user"
        (cd "${WORKSPACE_ROOT}" && rosdep update && rosdep install --from-paths src --ignore-src -r -y)
    fi
else
    echo "DRY-RUN: rosdep update && rosdep install --from-paths src --ignore-src -r -y (not executed)"
fi

# Build via colcon and install
if [ "$DRY_RUN" = false ]; then
    (cd "${WORKSPACE_ROOT}" && colcon build --symlink-install)
    echo "Running colcon install (if applicable)"
    (cd "${WORKSPACE_ROOT}" && colcon build --symlink-install --install-base install)
else
    echo "DRY-RUN: colcon build --symlink-install"
    echo "DRY-RUN: colcon build --symlink-install --install-base install"
fi

# Create systemd unit for foot launch (template). Ensure the venv is sourced.
UNIT_NAME="psyched-foot.service"
UNIT_PATH="/etc/systemd/system/${UNIT_NAME}"
VENV_SOURCE_LINE=""
if [ -n "${PSYCHED_VENV:-}" ]; then
    VENV_SOURCE_LINE="source ${PSYCHED_VENV}/bin/activate && "
fi
ENV_LINES="Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
if [ -n "${PSYCHED_VENV:-}" ]; then
    # Export PSYCHED_VENV so systemd unit processes can prefer the venv
    ENV_LINES="${ENV_LINES}\nEnvironment=PSYCHED_VENV=${PSYCHED_VENV}"
fi
UNIT_CONTENT="[Unit]\nDescription=Psyched foot launch wrapper\nAfter=network.target\n\n[Service]\nUser=${SUDO_USER:-$USER}\n${ENV_LINES}\nExecStart=/bin/bash -lc '${VENV_SOURCE_LINE}source /opt/psyched_workspace/install/setup.bash && ros2 launch psyched_create_launch foot_launch.py'\nRestart=always\nRestartSec=5\n\n[Install]\nWantedBy=multi-user.target\n"

if [ "$DRY_RUN" = false ]; then
    echo "$UNIT_CONTENT" | sudo tee "$UNIT_PATH" >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable "$UNIT_NAME"
    sudo systemctl restart "$UNIT_NAME" || true
else
    echo "DRY-RUN: create systemd unit ${UNIT_PATH} with content:\n${UNIT_CONTENT}"
fi

echo "Foot setup complete (dry_run=${DRY_RUN})"
