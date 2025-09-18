# Psyche Architecture

This document outlines the major subsystems that make up the Psyche robot stack
and how they interact at runtime.

## High-Level View

```
+-------------+   install.sh   +--------------+   host apply   +--------------------+
| GitHub Repo| --------------> | /opt/psyched | --------------> | Provision Services |
+-------------+                 |  (immutable) |                 |  (camera, lidar...)|
                                +--------------+                 +----------+---------+
                                                                           |
                                                                           v
                                                             +-----------------------+
                                                             | /etc/psyched/*.sh     |
                                                             | systemd launchers     |
                                                             +----------+------------+
                                                                        |
                                                                        v
                                                            +------------------------+
                                                            | ROS 2 Nodes / Drivers  |
                                                            +------------------------+
```

Key layers:

1. **Distribution** - `tools/install.sh` obtains the repository archive, extracts
   to `/opt/psyched`, and invokes the bootstrapper.
2. **Bootstrap** - `tools/provision/bootstrap.sh` links the `psy` CLI into the
   PATH, ensures ROS APT sources exist, and optionally calls `psy host apply`.
3. **Provisioning** - Per-service scripts under `provision/services/` install
   apt dependencies, clone ROS packages, patch third-party sources, and emit
   launchers to `/etc/psyched/<service>.launch.sh`.
4. **Systemd** - `provision/systemd/install_units.sh` writes the templated
   `psyched@.service` unit and enables any service with a launcher.
5. **Runtime** - systemd starts launchers, which source ROS environments and
   exec the hardware drivers or ROS nodes.

## Runtime Data Flow

```
 Sensors                        Base Control                   Voice
 ---------------                ---------------------------    ---------------------------
 /camera/image_raw -------> psyche_vision::object_detector --> /target_pose
                                             |                     |
                                             v                     v
                                       /vision_debug       psyche_vision::object_controller
                                             |                     |
                                             v                     v
                                      rqt_image_view        /cmd_vel ---> create_driver (foot)
                                                              /odom       (odometry)
                                                             ^    ^
                                                             |    |
                               slam_toolbox <-- /scan -- depthimage_to_laserscan <-- Kinect depth
                                                                  |                 mpu6050 (IMU)
                                                                  v
                                                        /camera/depth/image_raw
                                                             |
                                                             v
                                                        nav2 bringup
                                                             |
                                                             v
                                                        navigation stack
```

- **Perception** - `psyche_vision` consumes the camera feed (typically from
  `usb_cam`), performs HSV color segmentation, and publishes both bearing
  estimates and optional debug imagery.
- **Control** - The object controller applies a proportional controller to keep
  the target centered, commanding angular velocity on `/cmd_vel`. The `foot`
  service routes this to an iRobot Create base using the `create_driver` node.
- **Localization and Navigation** - `depthimage_to_laserscan` converts Kinect
  depth images into `/scan`, `imu` publishes `/imu/data`, `gps` provides
  `/gps/fix`, and these combine through nav stack components (`slam_toolbox`,
  Nav2, optional `robot_localization`). Tunable parameters live in
  `provision/bringup/`. If a planar lidar is present it can still publish
  directly to `/scan` and replace the depth bridge.
- **Voice** - The `voice` service exposes `/voice/<hostname>` topics. It bridges
  ROS text messages to Piper or eSpeak, queues utterances, and plays them back
  via ALSA.

## Host Configuration Layer

Host files (`provision/hosts/*.toml`) define the service set and core defaults.
For example, `cerebellum.toml` enables ROS, workspace management, foot (base
control), and optional sensors. Services can safely be commented out while
hardware is absent.

Provision order matters: `psy host apply` runs each service's `provision`
function, queues apt installs (to run once at the end), and finally invokes a
single workspace build. New services should follow this pattern to keep the
stack reproducible.

## Systemd Integration

- All launchers live in `/etc/psyched/`. They are small shell wrappers that
  source ROS, export log directories, and exec the underlying ROS command.
- `psyched@.service` is a single template that injects `ROS_DOMAIN_ID` and
  `RMW_IMPLEMENTATION` from the host TOML.
- Services can be managed via the CLI (`psy systemd up`, `psy systemd down`,
  `psy systemd info`) or directly with `systemctl`.

## ROS Graph Summary

At steady state the ROS graph typically includes:

- `usb_cam` -> `/camera/image_raw`
- `psyche_vision/object_detector` -> `/target_pose`, `/vision_debug`, `/target_point`
- `psyche_vision/object_controller` -> `/cmd_vel`
- `create_driver` -> `/odom`
- `slam_toolbox` -> `/map`
- `nav2` stack nodes (planner, controller, behavior tree)
- Sensor nodes (`nmea_serial_driver`, `mpu6050_node`, `hlds_laser_publisher`)
- Voice node (`/voice/<hostname>`)

This modular approach lets you enable only the pieces a host needs while keeping
common tooling, launch conventions, and provisioning logic centralized.
