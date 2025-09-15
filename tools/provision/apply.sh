#!/usr/bin/env bash
# Idempotent provisioner: installs services/configs based on hostname + device TOML
set -euo pipefail

REPO_DIR="/opt/psyched"
HOST_SHORT=$(hostname -s)
DEVICE_TOML="$REPO_DIR/devices/${HOST_SHORT}.toml"

log() { echo "[apply] $*"; }

require_root() { [ "${EUID:-$(id -u)}" -eq 0 ] || { echo "Run as root" >&2; exit 1; }; }

# Helper: install a unit file from the repo if it exists
install_unit() {
  src="$REPO_DIR/systemd/$1"
  dst="/etc/systemd/system/$1"
  mode="${2:-0644}"
  if [ -f "$src" ]; then
    install -m "$mode" "$src" "$dst"
  else
    log "Unit $1 not present in repo; skipping install"
  fi
}

# Helper: enable a unit if it exists either in repo or already installed
enable_unit() {
  unit="$1"
  if [ -f "$REPO_DIR/systemd/$unit" ] || [ -f "/etc/systemd/system/$unit" ]; then
    systemctl daemon-reload || true
    systemctl enable --now "$unit" || true
  else
    log "Unit $unit not found; skipping enable"
  fi
}

# Helper: disable a unit if present
disable_unit() {
  unit="$1"
  if [ -f "$REPO_DIR/systemd/$unit" ] || [ -f "/etc/systemd/system/$unit" ]; then
    systemctl disable --now "$unit" 2>/dev/null || true
  else
    log "Unit $unit not found; skipping disable"
  fi
}

ensure_common_packages() {
  log "Ensuring common packages"
  # If a Docker apt source exists but the keyring is missing, temporarily disable the
  # docker source so `apt-get update` can run to install tools (curl/gnupg) that we
  # then use to fetch the Docker GPG key. This avoids the NO_PUBKEY failure during
  # provisioning when the system has an existing docker.list but no key file.
  DOCKER_LIST=/etc/apt/sources.list.d/docker.list
  DOCKER_DISABLED=/etc/apt/sources.list.d/docker.list.disabled
  DISABLED_DOCKER=0
  if [ -f "$DOCKER_LIST" ] && [ ! -f /etc/apt/keyrings/docker.gpg ]; then
    log "Docker apt source present but key missing; temporarily disabling docker source for initial update"
    mv -f "$DOCKER_LIST" "$DOCKER_DISABLED" || true
    DISABLED_DOCKER=1
  fi

  apt-get update -y || true
  # Install common packages plus gnupg so we can dearmor keys
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl unzip jq python3 python3-pip python3-venv avahi-daemon git \
    python3-serial gpsd-clients gnupg || true

  # If we disabled the docker source above, attempt to fetch Docker's GPG key now,
  # write it to /etc/apt/keyrings/docker.gpg, and then re-enable the source and
  # perform another update so subsequent installs can use the Docker repo.
  if [ "$DISABLED_DOCKER" -eq 1 ]; then
    mkdir -p /etc/apt/keyrings
    if command -v curl >/dev/null 2>&1 && command -v gpg >/dev/null 2>&1; then
      log "Fetching Docker GPG key to /etc/apt/keyrings/docker.gpg"
      curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg || true
      chmod a+r /etc/apt/keyrings/docker.gpg || true
    else
      log "curl or gpg missing; will attempt to fetch Docker key later"
    fi
    if [ -f "$DOCKER_DISABLED" ]; then
      mv -f "$DOCKER_DISABLED" "$DOCKER_LIST" || true
    fi
    apt-get update -y || true
  fi
}

ensure_docker() {
  if command -v docker >/dev/null 2>&1; then
    log "Docker already installed"
    systemctl enable --now docker.service || true
    return 0
  fi

  log "Installing Docker Engine from Docker's official repository"
  set -e
  # Install prerequisites
  DEBIAN_FRONTEND=noninteractive apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release || true

  # Add Docker's official GPG key and repository
  mkdir -p /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg || true
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
    | tee /etc/apt/sources.list.d/docker.list > /dev/null

  # Try to install docker-ce and the compose plugin
  DEBIAN_FRONTEND=noninteractive apt-get update -y
  if DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; then
    log "Installed docker from official Docker repo"
    systemctl enable --now docker.service || true
    return 0
  fi

  log "Official Docker packages not available or failed; falling back to distro packages"
  DEBIAN_FRONTEND=noninteractive apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends docker.io || true
  # If docker-compose plugin missing, install python-based docker-compose as fallback
  if ! command -v docker-compose >/dev/null 2>&1; then
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends docker-compose || true
  fi
  systemctl enable --now docker.service || true
  set +e
}

ensure_py_zenoh() {
  VENV_DIR="${REPO_DIR}/venv"
  # Use python -m pip to avoid missing pip executable in minimal venvs
  if [ -x "$VENV_DIR/bin/python" ]; then
    "$VENV_DIR/bin/python" -m pip install --upgrade pip || true
    "$VENV_DIR/bin/python" -m pip install "zenoh>=0.4.0" || {
      log "Failed to install zenoh; zenoh services may be unavailable"
      return 1
    }
    log "zenoh installed successfully"
  else
    log "venv python not found at $VENV_DIR/bin/python"
    return 1
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
  ln -sf /usr/local/bin/psyched-update /usr/bin/update-psyche

  # Mark the repo safe for global git operations (avoid unsafe repo errors)
  # Mark the repo safe for global git operations (avoid unsafe repo errors)
  mark_git_safe() {
    repo="$1"
    # Prefer using git when available
    if command -v git >/dev/null 2>&1; then
      # Use --global when possible to make the setting available to the provisioning user
      git config --global --add safe.directory "$repo" || true
      return 0
    fi

    # Fallback: ensure /etc/gitconfig contains the safe.directory entry so all users
    # treat the repo as safe. Use a temporary file and atomic move to avoid partial writes.
    cfg=/etc/gitconfig
    tmp=$(mktemp)
    safe_section="[safe]\n\tdirectory = $repo\n"
    # If /etc/gitconfig exists and already contains the directory, skip
    if [ -f "$cfg" ] && grep -Fq "directory = $repo" "$cfg" >/dev/null 2>&1; then
      rm -f "$tmp" || true
      return 0
    fi
    # Merge existing config if present
    if [ -f "$cfg" ]; then
      cat "$cfg" > "$tmp" || true
    fi
    # Append safe section
    printf "%b\n" "$safe_section" >> "$tmp" || true
    install -m 0644 "$tmp" "$cfg" || true
    rm -f "$tmp" || true
  }
  mark_git_safe "$REPO_DIR"

  install_unit layer1-zenoh.service
  install_unit layer1-bridge.service
  install_unit layer3-launcher.service

  # Forebrain/Cerebellum container stacks
  install_unit forebrain-containers.service
  install_unit cerebellum-containers.service

  # Zenoh publishers (audio/video) - install if present
  # Note: several zenoh/ros2 republisher units were removed from the repo.
  # Only install units that are still present in `systemd/` via install_unit.

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

  # Create dedicated 'pete' system user for psyched services (no home)
  if ! id -u pete >/dev/null 2>&1; then
    log "Creating system user 'pete'"
    useradd --system --no-create-home --shell /usr/sbin/nologin pete || true
  fi
  # Ensure pete is in dialout for serial access
  if getent group dialout >/dev/null 2>&1; then
    usermod -a -G dialout pete || true
  fi

  # Udev rule: set owner to pete and group to dialout for ttyACM devices
  cat > /etc/udev/rules.d/99-psyched-ttyacm.rules <<'RULE'
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTR{devname}=="/dev/ttyACM*", OWNER=="pete", GROUP=="dialout", MODE=="0660"
RULE
  udevadm control --reload-rules || true
  udevadm trigger --action=add || true

  # If device exists now, ensure ACL so pete can access it immediately
  if [ -e /dev/ttyACM0 ]; then
    # Ensure setfacl is available; install 'acl' package on Debian/Ubuntu if missing
    if ! command -v setfacl >/dev/null 2>&1; then
      log "setfacl not found; attempting to install 'acl' package"
      DEBIAN_FRONTEND=noninteractive apt-get update -y || true
      DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends acl || true
    fi
    setfacl -m u:pete:rw /dev/ttyACM0 || true
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

  # Also install repository-wide requirements if present
  if [ -f "$REPO_DIR/requirements.txt" ]; then
    log "Installing repository requirements into venv"
    # Use --no-cache-dir to avoid excessive disk use on small devices
    "$PY_BIN" -m pip install --no-cache-dir -r "$REPO_DIR/requirements.txt" || true
  fi

  systemctl daemon-reload
  enable_unit layer1-zenoh.service
  # Enable bridge if requested in device TOML
  if [ -f "$DEVICE_TOML" ]; then
    br=$(python3 -c 'import tomllib, sys, pathlib
p=pathlib.Path(sys.argv[1])
d=tomllib.loads(p.read_text())
print(str(d.get("layer1",{}).get("bridge_ros2dds",{}).get("enabled", False)).lower())' "$DEVICE_TOML")
    if [ "$br" = "true" ]; then
      enable_unit layer1-bridge.service
    else
      disable_unit layer1-bridge.service
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
fi
# Only source the global ROS workspace setup if it exists
if [ -f "/opt/ros_ws/install/setup.sh" ]; then
  . "/opt/ros_ws/install/setup.sh"
fi
SH
}

setup_web_service() {
  log "Checking device TOML for web service"
  if [ ! -f "$DEVICE_TOML" ]; then
    log "No device TOML ($DEVICE_TOML); skipping web service setup"
    return 0
  fi

  enabled=$(python3 -c "import tomllib,sys,pathlib
p=pathlib.Path('$DEVICE_TOML')
d=tomllib.loads(p.read_text())
web=d.get('layer1',{}).get('services',{}).get('web',{})
print(str(web.get('enabled',False)).lower())")
  if [ "$enabled" != "true" ]; then
    log "Web service not enabled in $DEVICE_TOML; skipping"
    return 0
  fi
  host=$(python3 -c "import tomllib,sys,pathlib
p=pathlib.Path('$DEVICE_TOML')
d=tomllib.loads(p.read_text())
web=d.get('layer1',{}).get('services',{}).get('web',{})
print(web.get('host','0.0.0.0'))")
  port=$(python3 -c "import tomllib,sys,pathlib
p=pathlib.Path('$DEVICE_TOML')
d=tomllib.loads(p.read_text())
web=d.get('layer1',{}).get('services',{}).get('web',{})
print(web.get('port',8080))")

  log "Provisioning web service (host=$host port=$port)"
  VENV_DIR="$REPO_DIR/venv"
  PY_BIN="$VENV_DIR/bin/python"
  if [ ! -x "$PY_BIN" ]; then
    log "Venv python not found at $PY_BIN; ensure venv was created"
    return 0
  fi

  # Install fastapi, uvicorn, zenoh-python into venv (best-effort)
  log "Installing fastapi and uvicorn into venv"
  set +e
  "$PY_BIN" -m pip install --upgrade pip
  "$PY_BIN" -m pip install --no-cache-dir fastapi "uvicorn[standard]" "zenoh>=0.4.0" || true
  set -e

  # Create systemd unit using venv python to run uvicorn
  cat > /etc/systemd/system/psyche-web.service <<'UNIT'
[Unit]
Description=Psyche Web UI (uvicorn)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=$REPO_DIR
ExecStart=$PY_BIN -m uvicorn web.app:app --host $host --port $port
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT

  chmod 644 /etc/systemd/system/psyche-web.service || true
  systemctl daemon-reload || true
  enable_unit psyche-web.service
  log "psyche-web.service enabled"
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
  # Ensure rosdep is installed and initialized before proceeding
  DEBIAN_FRONTEND=noninteractive apt-get update -y || true
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-rosdep || true
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "Initializing rosdep sources"
    set +e
    rosdep init || true
    rosdep update || true
    set -e
  fi

  "$REPO_DIR/tools/provision/ros2_setup.sh" "$distro"
    enable_unit layer3-launcher.service
}

enable_role_stacks() {
  # Enable container stacks based on roles in /etc/psyched/device_roles.json
  if [ -f /etc/psyched/device_roles.json ]; then
    if grep -q '"forebrain"' /etc/psyched/device_roles.json; then
      ensure_docker
      log "Enabling forebrain container stack"
      enable_unit forebrain-containers.service
    else
      disable_unit forebrain-containers.service
    fi
    if grep -q '"cerebellum"' /etc/psyched/device_roles.json; then
      ensure_docker
      log "Enabling cerebellum container stack"
      enable_unit cerebellum-containers.service
  # Republish Zenoh audio/camera into ROS 2 on cerebellum
  ensure_py_zenoh
  ensure_alsa
  ensure_opencv
    else
      disable_unit cerebellum-containers.service
      log "ros2 republisher units removed from repo; skipping disable"
  fi
  if grep -q '"mic"' /etc/psyched/device_roles.json; then
      log "Ensuring ALSA tools and zenoh python for mic"
      ensure_alsa
      ensure_py_zenoh
  log "Enabling zenoh audio publisher (unit removed from repo)"
    else
      log "zenoh audio publisher unit removed from repo; skipping disable"
    fi
    # GNSS / GPS: enable if the device has 'ear' or 'gps' roles
    if grep -q '"ear"' /etc/psyched/device_roles.json || grep -q '"gps"' /etc/psyched/device_roles.json; then
      log "Ensuring GNSS serial tooling and enabling GNSS services"
      # Ensure serial device permissions are usable; add dialout or fallback to setfacl
      if getent group dialout >/dev/null 2>&1; then
        usermod -a -G dialout $(logname 2>/dev/null || echo root) || true
      else
        log "dialout group not present; ensure /dev/serial0 is accessible by service user"
      fi
    log "GNSS republishers removed from repo; ensure serial tooling only"
    else
      log "GNSS/ros2 republisher units removed from repo; skipping disable"
    fi
    if grep -q '"camera"' /etc/psyched/device_roles.json; then
      log "Ensuring OpenCV and zenoh python for camera"
      ensure_opencv
      ensure_py_zenoh
  log "Enabling zenoh camera publisher (unit removed from repo)"
    else
      log "zenoh camera publisher unit removed from repo; skipping disable"
    fi
  fi
}

main() {
  require_root
  ensure_common_packages
  install_zenoh
  install_files_and_units
  # After venv and device-specific requirements are installed, provision web service if requested
  setup_web_service
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
