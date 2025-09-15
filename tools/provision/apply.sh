#!/usr/bin/env bash
# Idempotent provisioner: installs services/configs based on hostname + device TOML
set -euo pipefail

REPO_DIR="/opt/psycheos"
HOST_SHORT=$(hostname -s)
DEVICE_TOML="$REPO_DIR/devices/${HOST_SHORT}.toml"

log() { echo "[apply] $*"; }

require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

ensure_common_packages() {
  log "Ensuring common packages"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl unzip jq python3 avahi-daemon
}

install_zenoh() {
  if command -v zenohd >/dev/null 2>&1; then
    log "zenohd present"
    return 0
  fi
  arch=$(uname -m)
  case "$arch" in
    aarch64) url="https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-plugins-bin-aarch64-unknown-linux-musl.zip" ;;
    armv7l) url="" ;;
    *) url="" ;;
  esac
  if [ -n "${url}" ]; then
    tmp=$(mktemp -d)
    curl -fsSL "$url" -o "$tmp/zenoh.zip" || { log "zenoh download failed"; rm -rf "$tmp"; return 0; }
    unzip -q "$tmp/zenoh.zip" -d "$tmp/z"
    bin=$(find "$tmp/z" -type f -name zenohd | head -n1 || true)
    if [ -n "$bin" ]; then install -m 0755 "$bin" /usr/local/bin/zenohd; fi
    rm -rf "$tmp"
    log "Installed zenohd"
  else
    log "No prebuilt zenohd for $(uname -m); please install manually"
  fi
}

install_files_and_units() {
  log "Installing configs and systemd units"
  install -d /etc/zenoh /etc/psycheos /run/zenoh
  install -m 0644 "$REPO_DIR/layer1/zenoh/router.json5" /etc/zenoh/router.json5
  install -m 0644 "$REPO_DIR/layer1/zenoh/bridge_ros2.json5" /etc/zenoh/bridge_ros2.json5
  install -m 0755 "$REPO_DIR/layer1/scripts/zenoh_autonet.sh" /usr/local/bin/zenoh_autonet.sh
  # Ensure update CLI is available on every apply
  install -m 0755 "$REPO_DIR/tools/provision/update_repo.sh" /usr/local/bin/psycheos-update
  ln -sf /usr/local/bin/psycheos-update /usr/bin/update-psyche

  install -m 0644 "$REPO_DIR/systemd/layer1-zenoh.service" /etc/systemd/system/layer1-zenoh.service
  install -m 0644 "$REPO_DIR/systemd/layer1-bridge.service" /etc/systemd/system/layer1-bridge.service
  install -m 0644 "$REPO_DIR/systemd/layer3-launcher.service" /etc/systemd/system/layer3-launcher.service

  # device roles for autonet: parse from TOML roles
  if [ -f "$DEVICE_TOML" ]; then
    python3 - "$DEVICE_TOML" >/etc/psycheos/device_roles.json <<'PY'
import json, pathlib, sys, tomllib
path = pathlib.Path(sys.argv[1])
data = tomllib.loads(path.read_text())
roles = data.get('device', {}).get('roles', [])
print(json.dumps({'roles': roles}))
PY
  fi

  systemctl daemon-reload
  systemctl enable --now layer1-zenoh.service || true
  # Enable bridge if requested in device TOML
  if [ -f "$DEVICE_TOML" ]; then
    br=$(python3 - "$DEVICE_TOML" <<'PY'
import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text())
print(str(d.get('layer1',{}).get('bridge_ros2dds',{}).get('enabled', False)).lower())
PY
)
    if [ "$br" = "true" ]; then
      systemctl enable --now layer1-bridge.service || true
    else
      systemctl disable --now layer1-bridge.service 2>/dev/null || true
    fi
  fi
}

setup_profile_env() {
  log "Setting global shell environment"
  cat > /etc/profile.d/psycheos.sh <<'SH'
# PsycheOS environment setup for all users
export PSYCHEOS_DIR=/opt/psycheos
if [ -f "$PSYCHEOS_DIR/layer2/rmw.env" ]; then
  set -a; . "$PSYCHEOS_DIR/layer2/rmw.env"; set +a
fi
if [ -n "${ROS_DISTRO:-}" ] && [ -d "/opt/ros/${ROS_DISTRO}" ]; then
  . "/opt/ros/${ROS_DISTRO}/setup.sh"
fi
if [ -d "/opt/ros_ws/install" ]; then
  . "/opt/ros_ws/install/setup.sh"
fi
SH
}

maybe_setup_ros2() {
  if [ ! -f "$DEVICE_TOML" ]; then
    log "Device TOML not found: $DEVICE_TOML"
    return 0
  fi
  distro=$(python3 - "$DEVICE_TOML" <<'PY'
import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text()).get('layer2',{}).get('ros_distro','none')
print(d)
PY
)
  if [ "$distro" = "none" ]; then
    log "ROS 2 not requested"
    return 0
  fi
  "$REPO_DIR/tools/provision/ros2_setup.sh" "$distro"
  systemctl enable --now layer3-launcher.service || true
}

main() {
  require_root
  ensure_common_packages
  install_zenoh
  install_files_and_units
  maybe_setup_ros2
  setup_profile_env
  log "Apply complete for host: $HOST_SHORT"
}

main "$@"
