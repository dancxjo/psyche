#!/usr/bin/env bash
# Bootstrap installer: clones repo, installs auto-updater, applies config.
#
# Usage (no args):
#   curl -fsSL https://raw.githubusercontent.com/dancxjo/psyche/refs/heads/main/tools/provision/bootstrap.sh?token=GHSAT0AAAAAADK7LS54HA6T4RJVUFOKL5TO2GHMPNA | sudo bash
#
# Optional flags:
#   -r|--repo URL   Git URL to clone (default from script variable)
#   -b|--branch BR  Branch to checkout (default: main)
set -euo pipefail

DEFAULT_REPO_URL="https://github.com/dancxjo/psyched.git"
DEFAULT_BRANCH="main"

log() { echo "[bootstrap] $*"; }
require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

parse_args() {
  REPO_URL="$DEFAULT_REPO_URL"
  BRANCH="$DEFAULT_BRANCH"
  while [ $# -gt 0 ]; do
    case "$1" in
      -r|--repo) REPO_URL="$2"; shift 2 ;;
      -b|--branch) BRANCH="$2"; shift 2 ;;
      *) echo "Unknown arg: $1" >&2; exit 2 ;;
    esac
  done
}

ensure_basics() {
  log "Installing git, curl, and Python"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git curl python3
}

clone_repo() {
  local dest="/opt/psyched"
  # Determine clone strategy: prefer SSH clone as the invoking user (SUDO_USER)
  # if they have SSH keys or an agent available. Otherwise use HTTPS with
  # optional GITHUB_TOKEN, or public HTTPS.
  CLONE_AS_USER="${SUDO_USER-}"
  USE_SSH=0
  if [ -n "$CLONE_AS_USER" ]; then
    # Check for public key files or SSH_AUTH_SOCK for agent
    if sudo -u "$CLONE_AS_USER" bash -lc 'shopt -s nullglob; keys=(~/.ssh/id_*); ((${#keys[@]}>0))' >/dev/null 2>&1; then
      USE_SSH=1
    elif sudo -u "$CLONE_AS_USER" bash -lc 'test -n "${SSH_AUTH_SOCK-}"' >/dev/null 2>&1; then
      USE_SSH=1
    fi
  fi

  if [ -d "$dest/.git" ]; then
    log "Repo already present at $dest"
    log "Repo already present at $dest; attempting to update"
    # Try a best-effort update: fetch and fast-forward merge, falling back to reset
    set +e
    cd "$dest" || return 0
    git fetch --prune origin "$BRANCH" 2>/dev/null
    if git rev-parse --verify "origin/${BRANCH}" >/dev/null 2>&1; then
      if git merge --ff-only "origin/${BRANCH}" >/dev/null 2>&1; then
        log "Fast-forwarded $dest to origin/$BRANCH"
      else
        log "Fast-forward not possible; performing hard reset to origin/$BRANCH"
        git reset --hard "origin/${BRANCH}" >/dev/null 2>&1 || log "git reset failed; repository may be in detached state"
      fi
    else
      log "Remote branch origin/$BRANCH not found; skipping update"
    fi
    set -e
  else
    if [ "$USE_SSH" -eq 1 ]; then
      CLONE_URL="git@github.com:dancxjo/psyched.git"
      log "Cloning via SSH as $CLONE_AS_USER: $CLONE_URL@$BRANCH to $dest"
      sudo -u "$CLONE_AS_USER" git clone --depth 1 --branch "$BRANCH" "$CLONE_URL" "$dest"
    else
      log "Cloning $REPO_URL@$BRANCH to $dest"
      git clone --depth 1 --branch "$BRANCH" "$REPO_URL" "$dest"
    fi
  fi
  # Make group-writable by sudo group and setgid on dirs
  log "Setting group write permissions for sudo group"
  groupadd -f sudo || true
  chgrp -R sudo "$dest"
  chmod -R g+rwX "$dest"
  find "$dest" -type d -exec chmod g+s {} +
}

install_updater() {
  log "Installing updater service and timer"
  install -m 0755 /opt/psyched/tools/provision/update_repo.sh /usr/local/bin/psyched-update
  ln -sf /usr/local/bin/psyched-update /usr/bin/update-psyche
  install -m 0644 /opt/psyched/systemd/psyched-updater.service /etc/systemd/system/psyched-updater.service
  install -m 0644 /opt/psyched/systemd/psyched-updater.timer /etc/systemd/system/psyched-updater.timer
  systemctl daemon-reload
  systemctl enable --now psyched-updater.timer
}

apply_config() {
  log "Applying provisioning configuration"
  /opt/psyched/tools/provision/apply.sh || true
}

main() {
  require_root
  parse_args "$@"
  ensure_basics
  clone_repo
  install_updater
  apply_config
  log "Bootstrap complete. Provisioner will keep repo up to date."
}

main "$@"
