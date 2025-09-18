#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS}"
SRC="$WS/src"
REPO_LIBCREATE="https://github.com/revyos-ros/libcreate.git"
BRANCH_LIBCREATE="fix-std-string"
REPO_CREATE_ROBOT="https://github.com/autonomylab/create_robot.git"

provision() {
  common_safe_source_ros || true
  common_ensure_ws
  common_clone_repo "$REPO_LIBCREATE" "$SRC/libcreate" "$BRANCH_LIBCREATE"
  common_clone_repo "$REPO_CREATE_ROBOT" "$SRC/create_robot"
  # udev for stable /dev/create
  common_write_udev_rules /etc/udev/rules.d/70-create.rules RULE <<'RULE'
# FTDI FT232 (common on some Create cables)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="create", MODE="0666"
# Prolific PL2303 classic
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="create", MODE="0666"
# Prolific PL2303 (newer variants, e.g. 23c3 observed in the field)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23c3", SYMLINK+="create", MODE="0666"
RULE

  # launcher for systemd with resilient retry loop
  common_install_launcher foot LAUNCH <<'LAUNCH'
#!/usr/bin/env bash
set -euo pipefail

# Delegate to the shared bringup script so manual and service launches stay aligned.
root="${PSY_ROOT:-/opt/psyched}"
exec "${root}/provision/bringup/create.sh" "$@"
LAUNCH
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
