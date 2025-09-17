#!/usr/bin/env bash
set -euo pipefail
ROOT="/opt/psyched"
URDF_DIR="${ROOT}/provision/robot"

provision() {
  set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash || true; set -u
  sudo mkdir -p "${URDF_DIR}"

  # Minimal URDF for Create base + laser + imu frames
  sudo tee "${URDF_DIR}/robot.urdf" >/dev/null <<'URDF'
<?xml version="1.0"?>
<robot name="psyched_bot" xmlns:xacro="http://wiki.ros.org/xacro">
  <link name="base_link"/>
  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.20 0.0 0.15" rpy="0 0 0"/>
  </joint>
  <link name="laser"/>
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu"/>
    <origin xyz="0.0 0.0 0.10" rpy="0 0 0"/>
  </joint>
  <link name="imu"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.10 0.0 0.20" rpy="0 0 0"/>
  </joint>
  <link name="camera_link"/>
</robot>
URDF

  sudo mkdir -p /etc/psyched
  sudo tee /etc/psyched/robot.launch.sh >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u
exec ros2 run robot_state_publisher robot_state_publisher /opt/psyched/provision/robot/robot.urdf
LAUNCH
  sudo chmod +x /etc/psyched/robot.launch.sh
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
