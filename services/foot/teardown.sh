#!/usr/bin/env bash
set -euo pipefail

# Foot service teardown (repo-level)
# Usage: services/foot/teardown.sh [--apply]
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

# Remove workspace links
for repo in $(jq -c '.repos[]' "${MANIFEST}"); do
    relpath=$(echo "$repo" | jq -r '.relpath')
    dest_link="${WORKSPACE_SRC}/${relpath}"
    if [ -L "$dest_link" ] || [ -e "$dest_link" ]; then
        run rm -rf "$dest_link"
    else
        echo "Workspace entry not present: $dest_link"
    fi
done

# Optionally remove canonical clones (not removing by default)
for repo in $(jq -c '.repos[]' "${MANIFEST}"); do
    relpath=$(echo "$repo" | jq -r '.relpath')
    dest="${CANONICAL_ROOT}/${relpath}"
    if [ -d "$dest" ]; then
        echo "Canonical repo present at $dest (not removing by default)"
        # To actually purge, re-run with custom logic or --purge flag
    fi
done

# Remove systemd unit
UNIT_NAME="psyched-foot.service"
UNIT_PATH="/etc/systemd/system/${UNIT_NAME}"
if [ "$DRY_RUN" = false ]; then
    sudo systemctl stop "$UNIT_NAME" || true
    sudo systemctl disable "$UNIT_NAME" || true
    sudo rm -f "$UNIT_PATH"
    sudo systemctl daemon-reload
else
    echo "DRY-RUN: would stop & disable systemd unit ${UNIT_NAME} and remove ${UNIT_PATH}"
fi

echo "Foot teardown complete (dry_run=${DRY_RUN})"
