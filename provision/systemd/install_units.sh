#!/usr/bin/env bash
set -euo pipefail
# Derive host config for environment values
CFG="/opt/psyched/provision/hosts/$(hostname).toml"
get_kv() { awk -F'=' -v k="$1" '$1~k{gsub(/[ "\t]/, "", $2); print $2}' "$CFG" 2>/dev/null || true; }
ROS_DOMAIN_ID="$(get_kv domain_id || echo 42)"
RMW_IMPLEMENTATION="$(get_kv rmw || echo rmw_cyclonedds_cpp)"
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
sudo sed -i \
  -e "s#__RMW__#${RMW_IMPLEMENTATION}#g" \
  -e "s#__DOMAIN__#${ROS_DOMAIN_ID}#g" \
  /etc/systemd/system/psyched@.service

# Reload systemd daemon to pick up changes to the template
sudo systemctl daemon-reload || true

# Enable units for services that have launchers dynamically
if compgen -G "/etc/psyched/*.launch.sh" >/dev/null; then
  for f in /etc/psyched/*.launch.sh; do
    [ -e "$f" ] || continue
    s="$(basename "$f")"; s="${s%.launch.sh}"
    sudo systemctl enable "psyched@${s}.service" || true
  done
fi

echo "[systemd] installed. Use: sudo systemctl start psyched@imu.service  (etc.)"
