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

# Enable units for services that have launchers
for s in imu lidar camera gps foot robot nav voice; do
  if [ -x "/etc/psyched/${s}.launch.sh" ]; then
    sudo systemctl enable "psyched@${s}.service"
  fi
done

echo "[systemd] installed. Use: sudo systemctl start psyched@imu.service  (etc.)"
