#!/usr/bin/env bash
# Bootstrap installer: clones repo, installs auto-updater, applies config.
#
# Usage (piped):
#   # recommended (preserves your SSH agent so the clone can use your keys):
#   ssh -A you@host "curl -fsSL https://dancxjo.github.io/psyched | sudo bash -s -- --git-as-user"
#
#   # or when running locally on the machine as root (no SSH-as-user):
#   curl -fsSL https://dancxjo.github.io/psyched | sudo bash
#
# Optional flags:
#   -r|--repo URL        Git URL to clone (default from script variable)
#   -b|--branch BR       Branch to checkout (default: main)
#   --git-as-user        Force cloning via SSH as the invoking user (requires SSH key or agent)
set -euo pipefail

DEFAULT_REPO_URL="https://github.com/dancxjo/psyched.git"
DEFAULT_BRANCH="main"

log() { echo "[bootstrap] $*"; }
require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

debug_env() {
  log "Debug: SUDO_USER='${SUDO_USER-}' SSH_AUTH_SOCK='${SSH_AUTH_SOCK-}'"
  if [ -n "${SUDO_USER-}" ]; then
    if sudo -u "$SUDO_USER" bash -lc 'shopt -s nullglob; keys=(~/.ssh/id_*); printf "%s\n" "${#keys[@]}"' >/tmp/psyched_ssh_key_count 2>/dev/null; then
      kc=$(cat /tmp/psyched_ssh_key_count 2>/dev/null || echo "0")
      log "Debug: $SUDO_USER has approximately $kc SSH key files in ~/.ssh"
      rm -f /tmp/psyched_ssh_key_count || true
    fi
  fi
}

parse_args() {
  REPO_URL="$DEFAULT_REPO_URL"
  BRANCH="$DEFAULT_BRANCH"
  GIT_AS_USER=0
  while [ $# -gt 0 ]; do
    case "$1" in
      -r|--repo) REPO_URL="$2"; shift 2 ;;
      -b|--branch) BRANCH="$2"; shift 2 ;;
      --git-as-user) GIT_AS_USER=1; shift ;;
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
  # If user explicitly requested git-as-user, require SUDO_USER to be set
  if [ "${GIT_AS_USER-0}" -eq 1 ]; then
    if [ -z "$CLONE_AS_USER" ]; then
      echo "--git-as-user requires invoking the installer via sudo from your user (SUDO_USER not set)." >&2
      echo "Recommended: ssh -A you@host 'curl -fsSL https://dancxjo.github.io/psyched | sudo bash -s -- --git-as-user'" >&2
      exit 2
    fi
    # Force SSH usage, but verify keys or agent exist
    if sudo -u "$CLONE_AS_USER" bash -lc 'shopt -s nullglob; keys=(~/.ssh/id_*); ((${#keys[@]}>0))' >/dev/null 2>&1 || sudo -u "$CLONE_AS_USER" bash -lc 'test -n "${SSH_AUTH_SOCK-}"' >/dev/null 2>&1; then
      USE_SSH=1
    else
      echo "Requested --git-as-user but no SSH keys or agent detected for $CLONE_AS_USER." >&2
      echo "Start an SSH session with agent forwarding and re-run:" >&2
      echo "  ssh -A $CLONE_AS_USER@host 'curl -fsSL https://dancxjo.github.io/psyched | sudo bash -s -- --git-as-user'" >&2
      exit 2
    fi
  else
    if [ -n "$CLONE_AS_USER" ]; then
      # Check for public key files or SSH_AUTH_SOCK for agent
      if sudo -u "$CLONE_AS_USER" bash -lc 'shopt -s nullglob; keys=(~/.ssh/id_*); ((${#keys[@]}>0))' >/dev/null 2>&1; then
        USE_SSH=1
      elif sudo -u "$CLONE_AS_USER" bash -lc 'test -n "${SSH_AUTH_SOCK-}"' >/dev/null 2>&1; then
        USE_SSH=1
      fi
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
    debug_env
    log "Decision: USE_SSH=$USE_SSH; CLONE_AS_USER='$CLONE_AS_USER'"
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
