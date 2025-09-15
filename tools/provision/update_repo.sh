#!/usr/bin/env bash
# Update the repo and re-apply provisioning. Intended for systemd service.
set -euo pipefail

REPO_DIR="/opt/psycheos"
BRANCH="main"

log() { echo "[update] $*"; }

if [ ! -d "$REPO_DIR/.git" ]; then
  log "Repo not found at $REPO_DIR"; exit 0
fi

cd "$REPO_DIR"
git fetch --prune origin || { log "git fetch failed"; exit 0; }
LOCAL_SHA=$(git rev-parse HEAD || echo unknown)
REMOTE_SHA=$(git rev-parse "origin/${BRANCH}" || echo unknown)
if [ "$LOCAL_SHA" != "$REMOTE_SHA" ]; then
  log "Updating repo ($LOCAL_SHA -> $REMOTE_SHA)"
  git reset --hard "origin/${BRANCH}"
  chgrp -R sudo "$REPO_DIR" && chmod -R g+rwX "$REPO_DIR" && find "$REPO_DIR" -type d -exec chmod g+s {} +
else
  log "Repo already up to date"
fi

if [ -x tools/provision/apply.sh ]; then
  log "Re-applying provisioning"
  exec tools/provision/apply.sh
else
  log "apply.sh not found or not executable"
fi

