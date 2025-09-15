#!/usr/bin/env bash
# Bootstrap installer: clones repo, installs auto-updater, applies config.
#
# Usage (no args):
#   curl -fsSL https://your.repo.example/tools/provision/bootstrap.sh | sudo bash
#
# Optional flags:
#   -r|--repo URL   Git URL to clone (default from script variable)
#   -b|--branch BR  Branch to checkout (default: main)
set -euo pipefail

# TODO: set to your canonical git URL (e.g., https://github.com/<org>/<repo>.git)
DEFAULT_REPO_URL="https://example.com/your/psycheos.git"  # CHANGEME
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
  local dest="/opt/psycheos"
  if [ -d "$dest/.git" ]; then
    log "Repo already present at $dest"
  else
    log "Cloning $REPO_URL@$BRANCH to $dest"
    git clone --depth 1 --branch "$BRANCH" "$REPO_URL" "$dest"
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
  install -m 0755 /opt/psycheos/tools/provision/update_repo.sh /usr/local/bin/psycheos-update
  ln -sf /usr/local/bin/psycheos-update /usr/bin/update-psyche
  install -m 0644 /opt/psycheos/systemd/psycheos-updater.service /etc/systemd/system/psycheos-updater.service
  install -m 0644 /opt/psycheos/systemd/psycheos-updater.timer /etc/systemd/system/psycheos-updater.timer
  systemctl daemon-reload
  systemctl enable --now psycheos-updater.timer
}

apply_config() {
  log "Applying provisioning configuration"
  /opt/psycheos/tools/provision/apply.sh || true
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
