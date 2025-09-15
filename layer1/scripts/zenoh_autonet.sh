#!/usr/bin/env bash
# Auto-networking for Zenoh.
# Scans for routers and decides whether to start a router or peer instance.
set -euo pipefail

MODE="${1:-auto}"
SCOUT_SECS="${SCOUT_SECS:-4}"
SEED="${ZENOH_PEERS:-}"

found_router() {
  timeout "${SCOUT_SECS}" zenohd --scout 2>/dev/null | grep -q "ROUTER"
}

start_router() {
  exec zenohd -c /etc/zenoh/router.json5
}

start_peer() {
  if [ -n "$SEED" ]; then
    exec zenohd --peer "$SEED"
  fi
  if [ -f /run/zenoh/peer.txt ]; then
    exec zenohd --peer "$(cat /run/zenoh/peer.txt)"
  fi
  exec zenohd
}

case "$MODE" in
  router)
    start_router ;;
  peer|client)
    if [ -n "$SEED" ]; then start_peer; fi
    if found_router; then
      zenohd --scout | awk '/ROUTER/ {print $NF}' >/run/zenoh/peer.txt || true
      start_peer
    else
      start_peer
    fi ;;
  auto|*)
    if found_router; then
      zenohd --scout | awk '/ROUTER/ {print $NF}' >/run/zenoh/peer.txt || true
      start_peer
    else
      if grep -q '"router"' /etc/psycheos/device_roles.json; then
        start_router
      else
        start_peer
      fi
    fi ;;
 esac
