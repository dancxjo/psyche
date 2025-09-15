#!/usr/bin/env bash
# Simple supervisor wrapper to keep a ros2 command running.
set -euo pipefail

REPO_WS=/opt/ros_ws
SLEEP_SECS=2

log() { echo "[psyche-ros2-wrapper] $*"; }

# First arg: command to run (as a single string)
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <ros2-cmd>"
  exit 1
fi

CMD="$*"

# Run forever, sourcing workspace if present
while true; do
  if [ -f "$REPO_WS/install/setup.sh" ]; then
    # Use a login-style shell to source env
    bash -lc ". $REPO_WS/install/setup.sh; exec $CMD"
  else
    bash -lc "exec $CMD"
  fi
  rc=$?
  log "Command exited (rc=$rc); restarting in $SLEEP_SECS seconds"
  sleep $SLEEP_SECS
done
