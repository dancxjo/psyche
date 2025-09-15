#!/usr/bin/env bash
# Idempotent provisioner: installs services/configs based on hostname + device TOML
set -euo pipefail

REPO_DIR="/opt/psyched"
HOST_SHORT=$(hostname -s)
DEVICE_TOML="$REPO_DIR/devices/${HOST_SHORT}.toml"

log() { echo "[apply] $*"; }

require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

ensure_common_packages() {
  log "Ensuring common packages"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl unzip jq python3 python3-pip python3-venv avahi-daemon
}

ensure_docker() {
  if command -v docker >/dev/null 2>&1; then
    log "Docker already installed"
    systemctl enable --now docker.service || true
    return 0
  fi
  log "Installing Docker Engine and Compose plugin"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    docker.io docker-compose-plugin
  systemctl enable --now docker.service || true
}

ensure_py_zenoh() {
  VENV_DIR="${REPO_DIR}/venv"
  # Use python -m pip to avoid missing pip executable in minimal venvs
  if [ -x "$VENV_DIR/bin/python" ]; then
    "$VENV_DIR/bin/python" -m pip install --upgrade pip || true
    "$VENV_DIR/bin/python" -m pip install "zenoh-python>=0.11.0" || true
  else
    log "venv python not found at $VENV_DIR/bin/python"
  fi
}

ensure_alsa() {
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends alsa-utils || true
}

ensure_opencv() {
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-opencv v4l-utils || true
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
  install -d /etc/zenoh /etc/psyched /run/zenoh
  install -m 0644 "$REPO_DIR/layer1/zenoh/router.json5" /etc/zenoh/router.json5
  install -m 0644 "$REPO_DIR/layer1/zenoh/bridge_ros2.json5" /etc/zenoh/bridge_ros2.json5
  install -m 0755 "$REPO_DIR/layer1/scripts/zenoh_autonet.sh" /usr/local/bin/zenoh_autonet.sh
  # Ensure update CLI is available on every apply
  install -m 0755 "$REPO_DIR/tools/provision/update_repo.sh" /usr/local/bin/psyched-update
  ln -sf /usr/local/bin/psyched-update /usr/bin/update-psyched

  install -m 0644 "$REPO_DIR/systemd/layer1-zenoh.service" /etc/systemd/system/layer1-zenoh.service
  install -m 0644 "$REPO_DIR/systemd/layer1-bridge.service" /etc/systemd/system/layer1-bridge.service
  install -m 0644 "$REPO_DIR/systemd/layer3-launcher.service" /etc/systemd/system/layer3-launcher.service

  # Forebrain/Cerebellum container stacks
  install -m 0644 "$REPO_DIR/systemd/forebrain-containers.service" /etc/systemd/system/forebrain-containers.service
  install -m 0644 "$REPO_DIR/systemd/cerebellum-containers.service" /etc/systemd/system/cerebellum-containers.service

  # Zenoh publishers (audio/video)
  install -m 0644 "$REPO_DIR/systemd/zenoh-audio-pub.service" /etc/systemd/system/zenoh-audio-pub.service
  install -m 0644 "$REPO_DIR/systemd/zenoh-camera-pub.service" /etc/systemd/system/zenoh-camera-pub.service

  # ROS 2 republishers
  install -m 0644 "$REPO_DIR/systemd/ros2-audio-repub.service" /etc/systemd/system/ros2-audio-repub.service
  install -m 0644 "$REPO_DIR/systemd/ros2-camera-repub.service" /etc/systemd/system/ros2-camera-repub.service

  # ROS exec helper
  install -m 0755 "$REPO_DIR/tools/ros_exec.sh" /usr/local/bin/ros_exec

  # device roles for autonet: parse from TOML roles
  if [ -f "$DEVICE_TOML" ]; then
    python3 -c 'import json, pathlib, sys, tomllib
path = pathlib.Path(sys.argv[1])
data = tomllib.loads(path.read_text())
roles = data.get("device", {}).get("roles", [])
print(json.dumps({"roles": roles}))' "$DEVICE_TOML" >/etc/psyched/device_roles.json
  fi

  # Python venv setup for daemons
  VENV_DIR="$REPO_DIR/venv"
  if [ ! -d "$VENV_DIR" ]; then
    log "Creating Python venv at $VENV_DIR"
    # Try creating the venv; if `ensurepip` is missing (common on minimal installs),
    # install the python3-venv package and retry.
    set +e
    python3 -m venv "$VENV_DIR"
    rc=$?
    set -e
    if [ $rc -ne 0 ]; then
      log "python3 -m venv failed (rc=$rc), installing python3-venv and retrying"
      DEBIAN_FRONTEND=noninteractive apt-get update -y
      DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-venv || true
      python3 -m venv "$VENV_DIR"
    fi
  fi
  log "Ensuring pip is available in venv"
  PY_BIN="$VENV_DIR/bin/python"
  if [ ! -x "$PY_BIN" ]; then
    log "Expected python binary missing in venv: $PY_BIN"
    return 0
  fi
  # Check whether pip is present
  set +e
  "$PY_BIN" -m pip --version >/dev/null 2>&1
  rc=$?
  set -e
  if [ $rc -ne 0 ]; then
    log "pip not found in venv, attempting to bootstrap via ensurepip"
    set +e
    "$PY_BIN" -m ensurepip --upgrade >/dev/null 2>&1
    rc2=$?
    set -e
    if [ $rc2 -ne 0 ]; then
      log "ensurepip failed; attempting to download get-pip.py and install pip"
      tmpdir=$(mktemp -d)
      if curl -fsSL "https://bootstrap.pypa.io/get-pip.py" -o "$tmpdir/get-pip.py"; then
        "$PY_BIN" "$tmpdir/get-pip.py" || log "get-pip.py install failed"
      else
        log "Failed to download get-pip.py"
      fi
      rm -rf "$tmpdir"
    fi
  fi
  # Use python -m pip for upgrades/installs (pip binary may still be absent)
  "$PY_BIN" -m pip install --upgrade pip || true
  # Read python requirements from device TOML and install into venv
  if [ -f "$DEVICE_TOML" ]; then
    REQS=$(python3 -c 'import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text())
reqs = d.get("python", {}).get("requirements", [])
print(" ".join(reqs))' "$DEVICE_TOML")
    if [ -n "$REQS" ]; then
      log "Installing Python requirements for host: $REQS"
      "$PY_BIN" -m pip install $REQS || true
    fi
  fi

  systemctl daemon-reload
  systemctl enable --now layer1-zenoh.service || true
  # Enable bridge if requested in device TOML
  if [ -f "$DEVICE_TOML" ]; then
    br=$(python3 -c 'import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text())
print(str(d.get("layer1",{}).get("bridge_ros2dds",{}).get("enabled", False)).lower())' "$DEVICE_TOML")
    if [ "$br" = "true" ]; then
      systemctl enable --now layer1-bridge.service || true
    else
      systemctl disable --now layer1-bridge.service 2>/dev/null || true
    fi
  fi
}

setup_profile_env() {
  log "Setting global shell environment"
  cat > /etc/profile.d/psyched.sh <<'SH'
# psyched environment setup for all users
export PSYCHED_DIR=/opt/psyched
if [ -f "$PSYCHED_DIR/layer2/rmw.env" ]; then
  set -a; . "$PSYCHED_DIR/layer2/rmw.env"; set +a
  . "/opt/ros_ws/install/setup.sh"
fi
SH
}

maybe_setup_ros2() {
  if [ ! -f "$DEVICE_TOML" ]; then
    log "Device TOML not found: $DEVICE_TOML"
    return 0
  fi
  distro=$(python3 -c 'import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text()).get("layer2",{}).get("ros_distro","none")
print(d)' "$DEVICE_TOML")
  if [ "$distro" = "none" ]; then
    log "ROS 2 not requested"
    return 0
  fi
  "$REPO_DIR/tools/provision/ros2_setup.sh" "$distro"
  systemctl enable --now layer3-launcher.service || true
}

enable_role_stacks() {
  # Enable container stacks based on roles in /etc/psyched/device_roles.json
  if [ -f /etc/psyched/device_roles.json ]; then
    if grep -q '"forebrain"' /etc/psyched/device_roles.json; then
      ensure_docker
      log "Enabling forebrain container stack"
      systemctl enable --now forebrain-containers.service || true
    else
      systemctl disable --now forebrain-containers.service 2>/dev/null || true
    fi
    if grep -q '"cerebellum"' /etc/psyched/device_roles.json; then
      ensure_docker
      log "Enabling cerebellum container stack"
      systemctl enable --now cerebellum-containers.service || true
      # Republish Zenoh audio/camera into ROS 2 on cerebellum
      ensure_py_zenoh
      ensure_alsa
      ensure_opencv
      systemctl enable --now ros2-audio-repub.service || true
      systemctl enable --now ros2-camera-repub.service || true
    else
      systemctl disable --now cerebellum-containers.service 2>/dev/null || true
      systemctl disable --now ros2-audio-repub.service 2>/dev/null || true
      systemctl disable --now ros2-camera-repub.service 2>/dev/null || true
    fi
    if grep -q '"mic"' /etc/psyched/device_roles.json; then
      log "Ensuring ALSA tools and zenoh python for mic"
      ensure_alsa
      ensure_py_zenoh
      log "Enabling zenoh-audio-pub"
      systemctl enable --now zenoh-audio-pub.service || true
    else
      systemctl disable --now zenoh-audio-pub.service 2>/dev/null || true
    fi
    if grep -q '"camera"' /etc/psyched/device_roles.json; then
      log "Ensuring OpenCV and zenoh python for camera"
      ensure_opencv
      ensure_py_zenoh
      log "Enabling zenoh-camera-pub"
      systemctl enable --now zenoh-camera-pub.service || true
    else
      systemctl disable --now zenoh-camera-pub.service 2>/dev/null || true
    fi
  fi
}

main() {
  require_root
  ensure_common_packages
  install_zenoh
  install_files_and_units
  maybe_setup_ros2
  # Ensure data dirs for container persistence
  install -d /opt/psyched/data/ollama /opt/psyched/data/qdrant \
             /opt/psyched/data/neo4j/data /opt/psyched/data/neo4j/logs \
             /opt/psyched/data/neo4j/plugins
  enable_role_stacks
  setup_profile_env
  log "Apply complete for host: $HOST_SHORT"
}

main "$@"
