#!/usr/bin/env bash
set -euo pipefail
# Templated unit that runs /etc/psyched/<name>.launch.sh if present
sudo tee /etc/systemd/system/psyched@.service >/dev/null <<'UNIT'
[Unit]
Description=Psyched service %i
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=ROS_DISTRO=jazzy
ExecStart=/bin/bash -lc '/etc/psyched/%i.launch.sh'
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT

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
