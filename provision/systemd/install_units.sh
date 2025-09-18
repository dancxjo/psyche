#!/usr/bin/env bash
set -euo pipefail
# Derive host config for environment values
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
if [ -n "${PSY_ROOT:-}" ] && [ -d "${PSY_ROOT%/}/provision/hosts" ]; then
  ROOT="${PSY_ROOT%/}"
elif [ -d "${DEFAULT_ROOT}/provision/hosts" ]; then
  ROOT="${DEFAULT_ROOT%/}"
elif [ -d "/opt/psyched/provision/hosts" ]; then
  ROOT="/opt/psyched"
else
  ROOT="${DEFAULT_ROOT%/}"
fi
CFG="${ROOT}/provision/hosts/$(hostname).toml"
CONFIG_HELPER="${ROOT}/tools/psy_config.py"
if [ -f "$CONFIG_HELPER" ]; then
  ROS_DOMAIN_ID="$(python3 "$CONFIG_HELPER" --file "$CFG" get domain_id)"
  RMW_IMPLEMENTATION="$(python3 "$CONFIG_HELPER" --file "$CFG" get rmw)"
else
  get_kv() { awk -F'=' -v k="$1" '$1~k{gsub(/[ "\t]/, "", $2); print $2}' "$CFG" 2>/dev/null || true; }
  ROS_DOMAIN_ID="$(get_kv domain_id || echo 42)"
  RMW_IMPLEMENTATION="$(get_kv rmw || echo rmw_cyclonedds_cpp)"
fi

# Extract the list of services granted to this host from the TOML config.
HOST_SERVICES=()
if [ -f "$CONFIG_HELPER" ] && [ -f "$CFG" ]; then
  mapfile -t HOST_SERVICES < <(python3 "$CONFIG_HELPER" --file "$CFG" services)
elif [ -f "$CFG" ]; then
  mapfile -t HOST_SERVICES < <(awk '
    BEGIN { inarr = 0 }
    /^[[:space:]]*services[[:space:]]*=/ { inarr = 1 }
    inarr {
      line = $0
      sub(/#.*/, "", line)
      while (match(line, /"[^"]+"/)) {
        svc = substr(line, RSTART + 1, RLENGTH - 2)
        print svc
        line = substr(line, RSTART + RLENGTH)
      }
      if ($0 ~ /\]/) { exit }
    }
  ' "$CFG") || HOST_SERVICES=()
fi

declare -A HOST_SERVICE_MAP=()
for svc in "${HOST_SERVICES[@]}"; do
  [ -n "$svc" ] || continue
  HOST_SERVICE_MAP["$svc"]=1
done

prune_ungranted_services() {
  # Skip pruning if we could not read host services; avoid deleting blindly.
  if [ ! -f "$CFG" ]; then
    return
  fi

  local removed_launcher=0

  if compgen -G "/etc/psyched/*.launch.sh" >/dev/null; then
    for f in /etc/psyched/*.launch.sh; do
      [ -e "$f" ] || continue
      local base name
      base="$(basename "$f")"
      name="${base%.launch.sh}"
      if [ -z "${HOST_SERVICE_MAP[$name]:-}" ]; then
        echo "[systemd] Pruning unauthorized launcher: $name"
        sudo systemctl disable "psyched@${name}.service" >/dev/null 2>&1 || true
        sudo systemctl stop "psyched@${name}.service" >/dev/null 2>&1 || true
        sudo rm -f "$f" || true
        removed_launcher=1
      fi
    done
  fi

  mapfile -t EXISTING_UNITS < <(systemctl list-unit-files 'psyched@*.service' --no-legend 2>/dev/null \
    | awk '$1 ~ /^psyched@.+\.service$/ { sub(/^psyched@/, "", $1); sub(/\.service$/, "", $1); if (!seen[$1]++) print $1 }') || EXISTING_UNITS=()

  for unit in "${EXISTING_UNITS[@]}"; do
    if [ -z "${HOST_SERVICE_MAP[$unit]:-}" ]; then
      echo "[systemd] Disabling unauthorized unit: psyched@${unit}.service"
      sudo systemctl disable "psyched@${unit}.service" >/dev/null 2>&1 || true
      sudo systemctl stop "psyched@${unit}.service" >/dev/null 2>&1 || true
    fi
  done

  if [ "$removed_launcher" -eq 1 ]; then
    sudo systemctl daemon-reload >/dev/null 2>&1 || true
  fi
}
# Templated unit that runs /etc/psyched/<name>.launch.sh if present
sudo tee /etc/systemd/system/psyched@.service >/dev/null <<'UNIT'
[Unit]
Description=Psyched service %i
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
# Base environment
Environment=ROS_DISTRO=jazzy
Environment=PYTHONUNBUFFERED=1
# Ensure a HOME and logging directory to avoid rcl logging errors under systemd
Environment=HOME=/root
Environment=ROS_LOG_DIR=/var/log/ros
Environment=RCUTILS_LOGGING_DIR=/var/log/ros
# Avoid setup scripts tripping on unbound COLCON_TRACE under nounset
Environment=AMENT_TRACE_SETUP_FILES=0
Environment=COLCON_TRACE=
Environment=XDG_RUNTIME_DIR=/run/user/%U
Environment=PSY_ROOT=__PSY_ROOT__
# Prepare log directories before start
ExecStartPre=/bin/mkdir -p /run/user/%U
ExecStartPre=/bin/chmod 700 /run/user/%U
ExecStartPre=/bin/mkdir -p /var/log/ros
ExecStartPre=/bin/chmod 755 /var/log/ros
ExecStartPre=/bin/mkdir -p /root/.ros
# NOTE: The following two lines are substituted after heredoc
# Environment=RMW_IMPLEMENTATION=__RMW__
# Environment=ROS_DOMAIN_ID=__DOMAIN__
ExecStart=/bin/bash -lc '/etc/psyched/%i.launch.sh'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT

# Inject host-specific env values into the template
ROOT_ESCAPED="${ROOT//\//\\/}"
ROOT_ESCAPED="${ROOT_ESCAPED//&/\&}"
sudo sed -i \
  -e "s#__RMW__#${RMW_IMPLEMENTATION}#g" \
  -e "s#__DOMAIN__#${ROS_DOMAIN_ID}#g" \
  -e "s#__PSY_ROOT__#${ROOT_ESCAPED}#g" \
  /etc/systemd/system/psyched@.service

# Reload systemd daemon to pick up changes to the template
sudo systemctl daemon-reload || true

# Remove any launchers/units not granted to this host
prune_ungranted_services

# Enable units for services that have launchers dynamically
if compgen -G "/etc/psyched/*.launch.sh" >/dev/null; then
  for f in /etc/psyched/*.launch.sh; do
    [ -e "$f" ] || continue
    s="$(basename "$f")"; s="${s%.launch.sh}"
    if [ -f "$CFG" ] && [ -z "${HOST_SERVICE_MAP[$s]:-}" ]; then
      echo "[systemd] Skipping launcher without host authorization: $s"
      continue
    fi
    sudo systemctl enable "psyched@${s}.service" || true
  done
fi

echo "[systemd] installed. Use: sudo systemctl start psyched@imu.service  (etc.)"
