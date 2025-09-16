#!/bin/bash
set -euo pipefail

# Idempotent bootstrap script for provisioning hosts in this repo.
# Usage: bootstrap.sh [--force] [--install-ros2]
# - --force: force rerun of actions even if already applied
# - --install-ros2: run tools/provision/setup_ros2.sh install-ros2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 || pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Help/usage block (also printed when running with -h/--help)
print_help() {
  cat <<'EOF'
Usage: bootstrap.sh [--force] [--install-ros2]

This bootstrap script prepares a host to run the Psyched workspace. It is
safe to run from a checkout or via a piped installer (e.g. curl ... | sudo bash).

Non-interactive cloning options (recommended):
  1) Preferred (private repo): run on the host as your normal user and let
     the script clone using your SSH keys. Example:
       ssh you@host 'curl -fsSL https://dancxjo.github.io/psyched | sudo bash'

  2) Token-based (CI or automation): export a short-lived GITHUB_TOKEN that
     has at least read access and run the script. The token is used only for
     the one-time clone URL and is not stored by the script:
       sudo GITHUB_TOKEN=ghp_xxx bash bootstrap.sh

  3) Manual (fallback): clone the repository yourself as your user, then run
     the bootstrap from the checked-out copy as root:
       git clone git@github.com:dancxjo/psyched.git ~/psyched
       sudo bash ~/psyched/tools/provision/bootstrap.sh

If cloning-as-invoker (SSH) is selected but fails because the agent or
keys are not available, the script prints clear fallback steps.

EOF
}

# If the script isn't running from inside a checked-out repository (for example
# when run via `curl ... | sudo bash`), clone the repo into a temporary
# directory and run provisioning from that clone. This makes the bootstrap
# launcher safe to run directly on a host.
if [ ! -f "$PROJECT_ROOT/tools/provision/setup_ros2.sh" ]; then
  echo "Repository files not found next to bootstrap script; cloning repo to temporary dir"
  TMP_CLONE_DIR=$(mktemp -d /tmp/psyched-bootstrap.XXXX)
  echo "Cloning repository to $TMP_CLONE_DIR"
  # Try to download a ZIP of the repository (preferred for piped/root installs).
  ZIP_URL="https://github.com/dancxjo/psyched/archive/refs/heads/main.zip"
  REPO_ZIP="$TMP_CLONE_DIR/psyched-main.zip"
  DL_OK=1

  if command -v curl >/dev/null 2>&1; then
    echo "Downloading repository ZIP via curl"
    if [ -n "${GITHUB_TOKEN-}" ]; then
      curl -sS -H "Authorization: token ${GITHUB_TOKEN}" -L "$ZIP_URL" -o "$REPO_ZIP" || DL_OK=0
    else
      curl -sS -L "$ZIP_URL" -o "$REPO_ZIP" || DL_OK=0
    fi
  elif command -v wget >/dev/null 2>&1; then
    echo "Downloading repository ZIP via wget"
    if [ -n "${GITHUB_TOKEN-}" ]; then
      wget -q --header="Authorization: token ${GITHUB_TOKEN}" -O "$REPO_ZIP" "$ZIP_URL" || DL_OK=0
    else
      wget -q -O "$REPO_ZIP" "$ZIP_URL" || DL_OK=0
    fi
  else
    DL_OK=0
  fi

  if [ "$DL_OK" -eq 1 ] && [ -s "$REPO_ZIP" ]; then
    echo "Extracting repository ZIP to $TMP_CLONE_DIR"
    # Prefer unzip, fall back to python's zipfile module
    if command -v unzip >/dev/null 2>&1; then
      unzip -q "$REPO_ZIP" -d "$TMP_CLONE_DIR" || { echo "unzip failed" >&2; exit 1; }
    else
      # Use Python to extract if unzip isn't available
      python3 - <<'PY'
import sys, zipfile
z='''"$REPO_ZIP"'''
out='''"$TMP_CLONE_DIR"'''
with zipfile.ZipFile(z) as zf:
    zf.extractall(out)
PY
      if [ $? -ne 0 ]; then echo "python unzip failed" >&2; exit 1; fi
    fi
    # The zip extracts into psyched-main/ or similar; detect the first subdir
    EXTRACTED_DIR=$(find "$TMP_CLONE_DIR" -maxdepth 1 -type d -name "psyched-*" | head -n 1)
    if [ -n "$EXTRACTED_DIR" ]; then
      PROJECT_ROOT="$EXTRACTED_DIR"
      echo "Using downloaded project at $PROJECT_ROOT"
    else
      echo "Could not find extracted project directory" >&2; exit 1
    fi
  else
    # ZIP download failed — fall back to git clone if available.
    if command -v git >/dev/null 2>&1; then
      echo "ZIP download failed; falling back to git clone (no user switch)."
      if [ -n "${GITHUB_TOKEN-}" ]; then
        CLONE_URL="https://x-access-token:${GITHUB_TOKEN}@github.com/dancxjo/psyched.git"
      else
        CLONE_URL="https://github.com/dancxjo/psyched.git"
      fi
      git clone --depth 1 "$CLONE_URL" "$TMP_CLONE_DIR" || { echo "git clone failed" >&2; exit 1; }
      git config --global --add safe.directory "$TMP_CLONE_DIR" 2>/dev/null || true
      PROJECT_ROOT="$TMP_CLONE_DIR"
      echo "Using cloned project at $PROJECT_ROOT"
    else
      echo "Neither ZIP download tools nor git are available; cannot acquire repository." >&2
      echo "Please run this script from a local checkout or install curl/wget/unzip or git." >&2
      exit 1
    fi
  fi
fi
HOSTNAME_FULL="$(hostname --fqdn 2>/dev/null || hostname)"
STATE_DIR="/var/lib/psyched_bootstrap"
STATE_FILE="$STATE_DIR/$HOSTNAME_FULL.state"

FORCE=0
DO_INSTALL_ROS2=0

while [ $# -gt 0 ]; do
  case "$1" in
    --force)
      FORCE=1
      shift
      ;;
    --install-ros2)
      DO_INSTALL_ROS2=1
      shift
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "Unknown arg: $1"
      echo "Usage: $(basename "$0") [--force] [--install-ros2]"
      exit 2
      ;;
  esac
done

if [ "$EUID" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=
fi

echo "Bootstrap starting for host: $HOSTNAME_FULL"

# Ensure state dir exists
$SUDO mkdir -p "$STATE_DIR"
$SUDO chown $(id -u):$(id -g) "$STATE_DIR"

# Ensure system user 'pete' exists (system account, no home)
if ! id -u pete >/dev/null 2>&1; then
  echo "Creating system user 'pete'"
  # Create pete with a normal shell and home so we can run git and maintenance
  # operations as that user. We intentionally create a constrained account but
  # with a usable shell to allow non-interactive maintenance via sudo -u pete.
  $SUDO useradd --system --create-home --shell /bin/bash --home-dir /home/pete pete || true
fi

# If an invoking user exists (SUDO_USER) try to import their public SSH
# keys into a managed location and into pete's authorized_keys so pete can
# perform git operations using SSH when appropriate. We ONLY copy public
# keys (files ending in .pub) — we do not copy private keys.
if [ -n "${SUDO_USER-}" ]; then
  INVOKER="$SUDO_USER"
  INV_SSH_DIR="/home/${INVOKER}/.ssh"
  DEST_KEYS_DIR="/opt/psyched/ssh_keys"
  $SUDO mkdir -p "$DEST_KEYS_DIR"
  $SUDO chown root:root "$DEST_KEYS_DIR"
  $SUDO chmod 0755 "$DEST_KEYS_DIR"

  if [ -d "$INV_SSH_DIR" ]; then
    # Collect public keys from invoker's .ssh (id_*.pub, *.pub)
    PUB_FILES=$(sudo -u "$INVOKER" bash -lc 'ls -1 ~/.ssh/*.pub 2>/dev/null || true') || PUB_FILES=""
    if [ -n "$PUB_FILES" ]; then
      echo "Importing public SSH keys from $INVOKER"
      for f in $PUB_FILES; do
        base=$(basename "$f")
        # Copy the public key into a managed dir (overwriting existing)
        $SUDO cp "$f" "$DEST_KEYS_DIR/${INVOKER}_${base}"
        $SUDO chown root:root "$DEST_KEYS_DIR/${INVOKER}_${base}"
        $SUDO chmod 0644 "$DEST_KEYS_DIR/${INVOKER}_${base}"
      done

      # Ensure pete has an .ssh and an authorized_keys file and append keys
      PETE_SSH_DIR="/home/pete/.ssh"
      $SUDO mkdir -p "$PETE_SSH_DIR"
      $SUDO chown pete:root "$PETE_SSH_DIR"
      $SUDO chmod 0700 "$PETE_SSH_DIR"

      AUTH_FILE="$PETE_SSH_DIR/authorized_keys"
      touch /tmp/psyched_pub_concat || true
      > /tmp/psyched_pub_concat
      for f in $PUB_FILES; do
        sudo -u "$INVOKER" bash -lc "cat '$f' >> /tmp/psyched_pub_concat" || true
      done
      # Append unique keys into pete's authorized_keys
      if [ -s /tmp/psyched_pub_concat ]; then
        # Use awk to only add keys not already present
        $SUDO touch "$AUTH_FILE"
        $SUDO chown pete:root "$AUTH_FILE"
        $SUDO chmod 0600 "$AUTH_FILE"
        # Append any keys that are not already in authorized_keys
        sudo awk 'FNR==NR{a[$0];next}!($0 in a){print $0}' "$AUTH_FILE" /tmp/psyched_pub_concat | $SUDO tee -a "$AUTH_FILE" >/dev/null || true
        $SUDO chown pete:root "$AUTH_FILE"
        $SUDO chmod 0600 "$AUTH_FILE"
      fi
      rm -f /tmp/psyched_pub_concat || true
    fi
  fi
fi

state_current=""
if [ -f "$STATE_FILE" ]; then
  state_current=$(cat "$STATE_FILE") || true
fi

echo "Current state: ${state_current:-<none>}"

mark_state() {
  local newstate="$1"
  echo "$newstate" > "$STATE_FILE"
  $SUDO chown root:root "$STATE_FILE" || true
  $SUDO chmod 0644 "$STATE_FILE" || true
}

# If requested, run ros2 install via the project-level installer
if [ "$DO_INSTALL_ROS2" -eq 1 ]; then
  echo "--install-ros2 requested"
  if [ "$FORCE" -eq 0 ] && [ "$state_current" = "ros2-installed" ]; then
    echo "ros2 already installed (state file), skipping. Use --force to reinstall."
  else
    echo "Running install-ros2 via tools/provision/setup_ros2.sh"
    /bin/bash "$PROJECT_ROOT/tools/provision/setup_ros2.sh" install-ros2
    mark_state "ros2-installed"
  fi
fi

echo "Bootstrap finished"
exit 0
