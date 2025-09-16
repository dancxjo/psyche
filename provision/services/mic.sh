#!/usr/bin/env bash
set -euo pipefail
provision() {
  sudo apt-get update -y
  sudo apt-get install -y alsa-utils libasound2-dev
  # Optional defaults can be set in /etc/asound.conf
}
case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
