#!/usr/bin/env bash
set -euo pipefail
WS="/opt/psyched/ws"
SRC="$WS/src"
REPO_IMU="https://github.com/hiwad-aziz/ros2_mpu6050_driver.git"

provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  sudo apt-get update -y
  sudo apt-get install -y i2c-tools libi2c-dev
  sudo raspi-config nonint do_i2c 0 2>/dev/null || true
  grep -q "i2c-dev" /etc/modules || echo i2c-dev | sudo tee -a /etc/modules
  mkdir -p "$SRC"
  [ -d "$SRC/ros2_mpu6050_driver" ] || git clone "$REPO_IMU" "$SRC/ros2_mpu6050_driver"
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
  # systemd unit launcher
  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/imu.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
source /opt/psyched/ws/install/setup.bash
exec ros2 run mpu6050_driver mpu6050_node --ros-args -p i2c_bus:=1 -p i2c_address:=0x68 -r imu/data:=/imu
LAUNCH
  sudo chmod +x /etc/psyched/imu.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
