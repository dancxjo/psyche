# ROS Workspace Guide

The Psyche workspace lives at `/opt/psyched/ws` and is maintained by the
`provision/services/workspace.sh` helper. This document explains the layout,
build pipeline, and project-specific packages.

## Layout and Symlinks

- The real workspace directory defaults to `/opt/psyched_ws` to decouple it from
  the repository tree under `/opt/psyched`. `_common.sh` exposes `PSY_WS_REAL`
  and keeps `/opt/psyched/ws` as a symlink pointing at the real location.
- `common_ensure_ws` creates the workspace (with `src/` and `.colcon_keep`) and
  establishes or repairs the symlink on each provisioning run.
- A legacy workspace at `/opt/ros2_ws` is automatically merged into the new
  location by `merge_legacy_workspace` when `psy build` runs.

## Provisioning Step

`provision/services/workspace.sh provision`:

1. Defers apt installs via `PSY_DEFER_APT=1` so packages are queued for a single
   transaction at the end of the host apply.
2. Queues key ROS dependencies used by C++ vision packages:
   - `ros-${ROS_DISTRO}-cv-bridge`
   - `ros-${ROS_DISTRO}-image-transport`
   - `ros-${ROS_DISTRO}-image-transport-plugins`
   - `ros-${ROS_DISTRO}-vision-msgs`
   - `libopencv-dev`
3. Queues `python3-colcon-common-extensions` when available.

Other service scripts clone or queue additional dependencies into the workspace,
for example:

- `foot.sh` - clones `libcreate` and `create_robot`.
- `vision.sh` - clones `ros2_shared`, `kinect_ros2`, and `libfreenect`; patches
  `kinect_ros2` dependencies; builds `libfreenect` from source.
- `imu.sh` - clones `ros2_mpu6050_driver` and inserts a missing `<array>` include.
- `lidar.sh` - clones `hls_lfcd_lds_driver` if the apt package is unavailable.

All clones land under `ws/src/` so they are visible to `colcon`.

## Build Step

`provision/services/workspace.sh build` performs the following:

1. Sources `/opt/ros/${ROS_DISTRO}/setup.bash` without triggering nounset errors.
2. Enables `ccache`, sets compiler launcher environment variables, and stores
   cache data in `/opt/psyched/.ccache` by default.
3. Flushes any queued apt packages (`common_flush_apt_queue`).
4. Runs `rosdep install --from-paths src` when sources change. A hash file
   (`.rosdep_src_hash`) prevents redundant runs between builds.
5. Attempts to detect the NumPy include directory for CMake builds.
6. Invokes `colcon build` with explicit Python executable hints and ccache
   launchers.

To rebuild after making changes, run `psy build` or call the script directly:
`/opt/psyched/provision/services/workspace.sh build`.

## `psyche_vision` Package

Location: `ws/src/psyche_vision`

Components:

- `psyche_vision/object_detector.py` - HSV color-based detector publishing
  `/target_pose`, `/target_point`, and `/vision_debug`.
- `psyche_vision/object_controller.py` - Proportional controller that drives the
  robot to face the target using `/cmd_vel`.
- Launch files:
  - `launch/vision_launch.py` - default detector + controller.
  - `launch/face_object.launch.py` - variant enabling `/target_point` usage.
- Tests: `test_vision.py` exercises bearing maths and controller clamping without
  requiring ROS to be installed. Run with `python3 ws/src/psyche_vision/test_vision.py`.
- Package metadata: `package.xml`, `setup.py`, and `resource/psyche_vision`.

See `ws/src/psyche_vision/README.md` for node-level architecture and tuning
parameters.

## External Package Notes

- `libfreenect` is marked with `COLCON_IGNORE`; it is built manually during
  provisioning and installed system-wide.
- Any legacy ROS1 packages (e.g., `rplidar_ros`) that remain in the workspace are
  also ignored via `COLCON_IGNORE` to keep colcon focused on ROS 2 projects.
- Workspace builds respect `PSY_SKIP_ROSDEP=1`, allowing advanced users to skip
  dependency resolution when operating in constrained environments.

## Logs and Environment

- Systemd launchers export `ROS_LOG_DIR=/var/log/ros` and
  `RCUTILS_LOGGING_DIR=/var/log/ros` before exec'ing ROS nodes.
- The global profile `/etc/profile.d/psyched_env.sh` (installed by `ros.sh`)
  sources both `/opt/ros/<distro>/setup.bash` and the workspace overlay if it
  exists, ensuring shells have an up-to-date environment.

For further details on individual drivers, consult `docs/SERVICES.md`.
