#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true
WS="${PSY_WS}"
SRC="$WS/src"
REPO_IMU="https://github.com/hiwad-aziz/ros2_mpu6050_driver.git"

provision() {
  common_safe_source_ros || true
  export PSY_DEFER_APT=1
  common_apt_install i2c-tools libi2c-dev
  sudo raspi-config nonint do_i2c 0 2>/dev/null || true
  grep -q "i2c-dev" /etc/modules || echo i2c-dev | sudo tee -a /etc/modules
  common_ensure_ws
  common_clone_repo "$REPO_IMU" "$SRC/ros2_mpu6050_driver"
  # Patch missing <array> include for GCC 13 (std::array incomplete type)
  HDR="$SRC/ros2_mpu6050_driver/include/mpu6050driver/mpu6050sensor.h"
  if [ -f "$HDR" ]; then
    if ! grep -qE '^#include <array>' "$HDR"; then
      echo "[psy][imu] Patching $HDR to include <array>"
      # Insert after the first block of includes
      awk '
        BEGIN{inserted=0}
        /^#include/ {
          print
          last_include=NR
          next
        }
        NR==last_include+1 && inserted==0 {
          print "#include <array>"
          inserted=1
        }
        {print}
      ' "$HDR" > "$HDR.tmp" && mv "$HDR.tmp" "$HDR"
    fi
  fi
  common_build_ws
  # systemd unit launcher
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/imu.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
root="${PSY_ROOT:-/opt/psyched}"
set +u; [ -f "${root}/ws/install/setup.bash" ] && source "${root}/ws/install/setup.bash"; set -u
exec ros2 run mpu6050_driver mpu6050_node --ros-args -p i2c_bus:=1 -p i2c_address:=0x68 -r imu/data:=/imu
LAUNCH
  sudo chmod +x /etc/psyched/imu.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
