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
  # Ensure basic tools: git, curl, python3, unzip
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git curl python3 unzip
  # Try to install GitHub CLI if available via apt
  if ! command -v gh >/dev/null 2>&1; then
    # Install GitHub CLI via official repository if not present
    if command -v curl >/dev/null 2>&1; then
      curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg >/dev/null 2>&1 || true
      chmod 644 /usr/share/keyrings/githubcli-archive-keyring.gpg || true
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list >/dev/null
      apt-get update -y
      DEBIAN_FRONTEND=noninteractive apt-get install -y gh || true
    fi
  fi
}

clone_repo() {
  local dest="/opt/psyched"
  local zip_url="https://github.com/dancxjo/psyche/archive/refs/heads/${BRANCH}.zip"
  local tmpdir
  tmpdir=$(mktemp -d)
  local zippath="${tmpdir}/psyche.zip"

  log "Downloading repository zip from ${zip_url}"
  if command -v curl >/dev/null 2>&1; then
    curl -fSL -o "${zippath}" "${zip_url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "${zippath}" "${zip_url}"
  else
    echo "Error: need curl or wget to download repository zip" >&2
    rm -rf "${tmpdir}"
    exit 1
  fi

  # Extract into staging dir for atomic swap
  local staged="${dest}.new"
  rm -rf "${staged}"
  mkdir -p "${staged}"

  if command -v unzip >/dev/null 2>&1; then
    unzip -q "${zippath}" -d "${tmpdir}"
  elif command -v bsdtar >/dev/null 2>&1; then
    bsdtar -xf "${zippath}" -C "${tmpdir}"
  else
    echo "Error: no suitable extractor (unzip or bsdtar) available" >&2
    rm -rf "${tmpdir}"
    exit 1
  fi

  TOPDIR=$(find "${tmpdir}" -maxdepth 1 -mindepth 1 -type d | head -n1)
  if [ -z "${TOPDIR}" ]; then
    echo "Error: extracted archive did not produce expected directory" >&2
    rm -rf "${tmpdir}"
    exit 1
  fi

  log "Copying extracted tree to staging dir ${staged}"
  cp -a "${TOPDIR}/." "${staged}/"

  # Set permissions on staged tree
  groupadd -f sudo || true
  chgrp -R sudo "${staged}" || true
  chmod -R g+rwX "${staged}" || true
  find "${staged}" -type d -exec chmod g+s {} + || true

  # Swap into place atomically: back up existing dest then move staged
  if [ -d "${dest}" ]; then
    log "Backing up existing ${dest} to ${dest}.bak"
    rm -rf "${dest}.bak"
    mv "${dest}" "${dest}.bak"
  fi
  mv "${staged}" "${dest}"

  # Clean up
  rm -rf "${tmpdir}"
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
