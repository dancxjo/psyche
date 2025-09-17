#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
provision() {
  export PSY_DEFER_APT=1
  common_apt_install alsa-utils libasound2-dev
  # Optional defaults can be set in /etc/asound.conf
}
case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
