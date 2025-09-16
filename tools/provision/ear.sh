#!/usr/bin/env bash
set -euo pipefail
# Provisioning helper for ublox7 on the 'ear' device.
# Creates a udev rule to provide a stable symlink and ensures gpsd is installed and started.

SERIAL_ID="ublox7-ear"
DEV_SYMLINK="/dev/${SERIAL_ID}"

echo "Looking for USB serial devices..."
# Try to find a USB serial by product/vendor or just use first /dev/ttyUSB*
PORT=""
for p in /dev/serial/by-id/*USB* /dev/serial/by-id/*; do
  if [ -e "$p" ]; then
    realp=$(readlink -f "$p")
    echo "Found candidate: $p -> $realp"
    PORT=$realp
    break
  fi
done

if [ -z "${PORT}" ]; then
  # fallback to /dev/ttyUSB0
  if [ -e /dev/ttyUSB0 ]; then
    PORT=/dev/ttyUSB0
  else
    echo "No serial GPS device found. Connect the ublox7 and re-run."
    exit 1
  fi
fi

echo "Creating symlink ${DEV_SYMLINK} -> ${PORT}"
sudo ln -sf "$PORT" "$DEV_SYMLINK"

echo "Creating udev rule for stable symlink"
UDEV_RULE="SUBSYSTEM==\"tty\", SYMLINK+=\"${SERIAL_ID}\", ATTRS{idVendor}==\"1546\", ATTRS{idProduct}==\"01a8\""
echo "${UDEV_RULE}" | sudo tee /etc/udev/rules.d/99-ublox.rules >/dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Ensure gpsd is installed and started (Debian/Ubuntu)."
if command -v apt >/dev/null; then
  sudo apt update
  sudo apt install -y gpsd gpsd-clients
fi

echo "Stop gpsd and restart bound to device"
sudo systemctl stop gpsd.socket || true
sudo systemctl disable gpsd.socket || true
sudo killall gpsd || true
sudo gpsd -N -n "$DEV_SYMLINK" &

echo "Provisioning complete. Device available at ${DEV_SYMLINK}"
#!/usr/bin/env bash
# Provision a Raspberry Pi Zero 2 W as the "ear" device.
#
# Idempotent; safe to re-run. Designed to be piped via curl:
#   curl -fsSL <URL>/tools/provision/ear.sh | sudo bash
set -euo pipefail

require_root() {
  if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    echo "Please run as root (use sudo)" >&2
    exit 1
  fi
}

log() { echo "[ear] $*"; }

# Temporarily disable zenoh pieces; set to true to re-enable
ENABLE_ZENOH=false

detect_arch() {
  local u
  u=$(uname -m)
  case "$u" in
    aarch64) echo "aarch64" ;;
    armv7l|armv6l) echo "armv7l" ;;
    *) echo "$u" ;;
  esac
}

set_hostname() {
  local name="ear"
  if [ "$(hostname -s)" != "$name" ]; then
    log "Setting hostname to $name"
    hostnamectl set-hostname "$name" || true
    sed -i "s/^127.0.1.1.*/127.0.1.1\t${name}/" /etc/hosts || true
  fi
}

ensure_packages() {
  log "Updating apt and installing dependencies"
  apt-get update -y
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates curl unzip jq python3 avahi-daemon
}

install_zenoh() {
  local arch url tmp dir bin
  arch=$(detect_arch)
  dir=/usr/local/bin
  mkdir -p "$dir"
  if [ "$ENABLE_ZENOH" != true ]; then
    log "Zenoh disabled: skipping zenohd installation"
    return 0
  fi

  if command -v zenohd >/dev/null 2>&1; then
    log "zenohd already installed"
  else
    case "$arch" in
      aarch64)
        url="https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-plugins-bin-aarch64-unknown-linux-musl.zip" ;;
      armv7l)
        # No official ARMv7 binary published upstream at this time.
        # Skip automated install and leave a note.
        url="" ;;
      *)
        url="" ;;
    esac
    if [ -n "$url" ]; then
      tmp=$(mktemp -d)
      log "Downloading zenoh bundle ($arch)"
      curl -fsSL "$url" -o "$tmp/zenoh.zip" || {
        log "Failed to download zenoh bundle. Please install zenohd manually."; rm -rf "$tmp"; return 0; }
      unzip -q "$tmp/zenoh.zip" -d "$tmp/z"
      # Try to locate zenohd in unpacked content
      bin=$(find "$tmp/z" -type f -name zenohd | head -n1 || true)
      if [ -n "$bin" ]; then
        install -m 0755 "$bin" "$dir/zenohd"
        log "Installed zenohd to $dir"
      else
        log "zenohd not found in archive; skipping install"
      fi
      rm -rf "$tmp"
    else
      log "No prebuilt zenohd for $arch; please install manually."
    fi
  fi

  if command -v zenoh-bridge-ros2dds >/dev/null 2>&1; then
    log "zenoh-bridge-ros2dds already installed"
  else
    # Optional for ear (disabled in config); skip if not available
    true
  fi
}

install_files() {
  log "Installing psyched files"
  install -d /opt/psyched/devices /opt/psyched/layer1/zenoh \
             /opt/psyched/layer1/scripts /opt/psyched/layer3/launcher \
             /opt/psyched/layer3/services /etc/zenoh /etc/psyched

  # Device roles for zenoh_autonet.sh
  cat > /etc/psyched/device_roles.json << 'JSON'
{"roles": ["imu", "mic"]}
JSON

  # Device TOML for ear
  cat > /opt/psyched/devices/ear.toml << 'TOML'
[device]
id = "ear"
roles = ["imu", "mic"]
hostname = "ear.local"

[layer1]
mode = "client"
scouting = true
bridge_ros2dds = { enabled = false }

[layer1.services.web]
enabled = true
host = "0.0.0.0"
port = 8080

[layer2]
ros_distro = "none"

[layer3]
nodes = []
TOML

  # Zenoh configs (skipped when zenoh disabled)
  if [ "$ENABLE_ZENOH" = true ]; then
    cat > /etc/zenoh/router.json5 << 'JSON'
{
  "mode": "router",
  "listen": ["tcp/0.0.0.0:7447"],
  "scouting": { "enabled": true }
}
JSON
  else
    log "Zenoh disabled: skipping zenoh config files"
  fi

setup_web_service() {
  local toml=/opt/psyched/devices/ear.toml
  if [ ! -f "$toml" ]; then
    log "No device TOML at $toml; skipping web setup"
    return 0
  fi

  # Check if layer1.services.web.enabled = true using a simple grep-based parse
  if ! grep -A3 "\[layer1.services.web\]" "$toml" | grep -q "enabled\s*=\s*true"; then
    log "Web service not enabled in $toml; skipping web setup"
    return 0
  fi

  # Extract host and port (fallback to defaults)
  local host port
  host=$(grep -A3 "\[layer1.services.web\]" "$toml" | sed -n 's/^[[:space:]]*host[[:space:]]*=[[:space:]]*"\?\([^"]*\)"\?$/\1/p' | tr -d '\n')
  port=$(grep -A3 "\[layer1.services.web\]" "$toml" | sed -n 's/^[[:space:]]*port[[:space:]]*=[[:space:]]*\([0-9]*\).*$/\1/p' | tr -d '\n')
  host=${host:-0.0.0.0}
  port=${port:-8080}

  log "Setting up web service (host=$host port=$port)"

  # Ensure pip is available
  if ! command -v pip3 >/dev/null 2>&1; then
    log "Installing python3-pip"
    DEBIAN_FRONTEND=noninteractive apt-get install -y python3-pip || true
  fi

  # Install runtime requirements (fastapi, uvicorn). zenoh-python is optional.
  if [ "$ENABLE_ZENOH" = true ]; then
    log "Installing Python packages (fastapi, uvicorn, zenoh)"
    pip3 install --no-cache-dir fastapi "uvicorn[standard]" "zenoh>=0.4.0" || true
  else
    log "Installing Python packages (fastapi, uvicorn) -- zenoh skipped"
    pip3 install --no-cache-dir fastapi "uvicorn[standard]" || true
  fi

  # Create systemd unit to run the FastAPI app. Assumes code lives under /opt/psyched
  cat > /etc/systemd/system/psyche-web.service << UNIT
[Unit]
Description=Psyche Web UI (uvicorn)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=/opt/psyched
ExecStart=/usr/bin/python3 -m uvicorn web.app:app --host ${host} --port ${port}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT

  chmod 644 /etc/systemd/system/psyche-web.service || true
  systemctl daemon-reload || true
  systemctl enable --now psyche-web.service || true
  log "Web service unit installed and enabled"
}
JSON

  if [ "$ENABLE_ZENOH" = true ]; then
    cat > /etc/zenoh/bridge_ros2.json5 << 'JSON'
{
  "ros2_to_zenoh": { "mode": "complete" },
  "zenoh_to_ros2": { "mode": "complete" },
  "scouting": { "enabled": true }
}
JSON
  else
    log "Zenoh disabled: skipping bridge config"
  fi

  # Layer1 autonet helper and systemd units are only created when zenoh enabled
  if [ "$ENABLE_ZENOH" = true ]; then
    # Layer1 autonet helper
    cat > /usr/local/bin/zenoh_autonet.sh << 'BASH'
#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-auto}"
SCOUT_SECS="${SCOUT_SECS:-4}"
SEED="${ZENOH_PEERS:-}"
found_router() { timeout "${SCOUT_SECS}" zenohd --scout 2>/dev/null | grep -q "ROUTER"; }
start_router() { exec zenohd -c /etc/zenoh/router.json5; }
start_peer() {
  if [ -n "$SEED" ]; then exec zenohd --peer "$SEED"; fi
  if [ -f /run/zenoh/peer.txt ]; then exec zenohd --peer "$(cat /run/zenoh/peer.txt)"; fi
  exec zenohd
}
case "$MODE" in
  router) start_router ;;
  peer|client)
    if [ -n "$SEED" ]; then start_peer; fi
    if found_router; then zenohd --scout | awk '/ROUTER/ {print $NF}' >/run/zenoh/peer.txt || true; start_peer; else start_peer; fi ;;
  auto|*)
    if found_router; then zenohd --scout | awk '/ROUTER/ {print $NF}' >/run/zenoh/peer.txt || true; start_peer;
    else if grep -q '"router"' /etc/psyched/device_roles.json; then start_router; else start_peer; fi; fi ;;
esac
BASH
    chmod +x /usr/local/bin/zenoh_autonet.sh

    # Systemd units
    cat > /etc/systemd/system/layer1-zenoh.service << 'UNIT'
[Unit]
Description=Layer1 Zenoh fabric
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=SCOUT_SECS=4
ExecStartPre=/usr/bin/mkdir -p /run/zenoh
ExecStart=/usr/local/bin/zenoh_autonet.sh auto
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT

    # layer1-bridge is intentionally not enabled for ear
    cat > /etc/systemd/system/layer1-bridge.service << 'UNIT'
[Unit]
Description=Zenoh <-> ROS2 DDS bridge

[Service]
Type=simple
ExecStart=/usr/bin/zenoh-bridge-ros2dds -c /etc/zenoh/bridge_ros2.json5
Restart=always

[Install]
WantedBy=multi-user.target
UNIT
  else
    log "Zenoh disabled: not creating autonet script or systemd units"
  fi
}

enable_services() {
  log "Enabling services"
  systemctl daemon-reload
  if [ "$ENABLE_ZENOH" = true ]; then
    systemctl enable --now layer1-zenoh.service
  else
    log "Zenoh disabled: not enabling layer1-zenoh.service"
  fi
}

main() {
  require_root
  set_hostname
  ensure_packages
  if [ "$ENABLE_ZENOH" = true ]; then
    install_zenoh
  else
    log "Zenoh disabled: skipping install_zenoh"
  fi
  install_files
  enable_services
  log "Provisioning complete. Reboot recommended."
}

main "$@"
