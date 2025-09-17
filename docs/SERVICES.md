# Service Catalogue

Each service under `provision/services/` encapsulates provisioning logic for a
hardware peripheral or subsystem. Provisioning scripts are idempotent; running
them repeatedly is safe.

## Shared Helper: `_common.sh`
- Defines root paths (`PSY_ROOT`, `PSY_WS`, `PSY_WS_REAL`) and ensures the
  workspace symlink is correct (`common_ensure_ws`).
- Provides the apt queueing system (`common_apt_install`, `common_flush_apt_queue`)
  so multiple services can request packages before a single combined install.
- Supplies convenience helpers: cloning repos (`common_clone_repo`), writing
  udev rules (`common_write_udev_rules`), creating launchers
  (`common_install_launcher`), enabling `ccache`, and sourcing ROS safely.

## `ros.sh`
- Reads ROS defaults from the host TOML (distro, domain ID, RMW implementation).
- Queues essential apt packages: Python tooling, `ros-${ROS_DISTRO}-ros-base`,
  CycloneDDS, Nav2 extras, and diagnostics.
- Installs `rosdep`, initialises its database, and adds environment exports to
  `~/.bashrc` plus `/etc/profile.d/psyched_env.sh`.
- Ensures interactive shells source the global profile by patching
  `/etc/bash.bashrc` and the current user's `~/.bashrc`.

## `workspace.sh`
- `provision` step queues `colcon` tooling and vision-related C++ dependencies.
- `build` step sources ROS, enables `ccache`, runs `rosdep` when the `src/`
  contents change, and executes `colcon build` with NumPy include hints.
- Called implicitly after all services provision, or manually via `psy build`.

## `foot.sh` (Create base)
- Clones `libcreate` (branch `fix-std-string`) and `create_robot` into the
  workspace.
- Installs a udev rule mapping recognised USB adapters to `/dev/create`.
- Creates `/etc/psyched/foot.launch.sh` which runs `ros2 run create_driver
  create_driver_node` with retry and diagnostic loops. Environment variables such
  as `CREATE_PORT`, `FAIL_MAX_BEFORE_BACKOFF`, and `FOOT_DEBUG` can be set in
  `/etc/default/psyched-foot`.

## `camera.sh`
- Installs `ros-${ROS_DISTRO}-usb-cam` and `v4l-utils` (or `v4l2-utils`).
- Launch script runs `ros2 run usb_cam usb_cam_node_exe` targeting `/dev/video0`
  at 1280x720 MJPEG and remaps output to `/camera/image_raw`.

## `gps.sh`
- Installs `ros-${ROS_DISTRO}-nmea-navsat-driver`.
- Writes udev rules that expose `/dev/gps` for common u-blox adapters.
- Launch script runs `ros2 run nmea_navsat_driver nmea_serial_driver` with topic
  remaps for `/gps/fix` and `/gps/vel`.

## `imu.sh`
- Installs I2C tooling, enables I2C via `raspi-config`, and ensures `i2c-dev` is
  loaded.
- Clones `ros2_mpu6050_driver` and patches `mpu6050sensor.h` to include `<array>`
  for GCC 13 compatibility.
- Launch script runs `ros2 run mpu6050_driver mpu6050_node` with `/imu/data`.

## `lidar.sh`
- Installs git, ensures the workspace exists, and attempts to install the
  `hls_lfcd_lds_driver` apt package. If unavailable, clones the driver source.
- Adds `COLCON_IGNORE` to legacy `rplidar_ros` packages.
- Writes udev rules mapping supported adapters to `/dev/lidar`.
- Launch script repeatedly tries to run `ros2 run hls_lfcd_lds_driver
  hlds_laser_publisher` with logging to `/tmp/lidar_driver.log`.

## `nav.sh`
- Queues Nav2, SLAM Toolbox, and TF2 packages.
- Launch script (invoked by systemd) sources ROS, starts `slam_toolbox
  online_sync_launch.py` with `provision/bringup/slam_params.yaml`, then launches
  Nav2 using `provision/bringup/nav2_params.yaml`.

## `robot.sh`
- Writes a minimal URDF to `/opt/psyched/provision/robot/robot.urdf` describing
  `base_link`, `laser`, `imu`, and `camera_link` frames.
- Launch script runs `ros2 run robot_state_publisher robot_state_publisher` with
  the generated URDF.

## `vision.sh`
- Queues image transport, vision messages, OpenCV, and libusb packages.
- Clones supporting repos (`ros2_shared`, `kinect_ros2`, `libfreenect`) and marks
  `libfreenect` with `COLCON_IGNORE` so colcon ignores it.
- Patches `kinect_ros2` `package.xml` and `CMakeLists.txt` to declare
  `cv_bridge` and `image_transport` dependencies.
- Builds `libfreenect` from source (outside colcon) and installs it system-wide.
- Launch script execs `ros2 launch psyche_vision vision_launch.py`.

## `voice.sh`
- Resolves the requested TTS engine (Piper by default, fallbacks to espeak).
- Queues ALSA and eSpeak packages, installs Piper CLI if available or downloads a
  prebuilt archive when packages are missing.
- Downloads voice models based on alias-driven candidate lists (e.g.,
  `en_male_default`). Creates `/etc/default/psyched-voice` capturing the selected
  model and fallbacks.
- Installs `voice_node.py` into `/etc/psyched/` and creates `voice.launch.sh`
  which sources ROS, resolves the Piper binary, exports `PSY_VOICE_MODEL`, and
  execs `python3 /etc/psyched/voice_node.py`.
- The node exposes:
  - `/voice/<hostname>` (`std_msgs/String`) for queued speech.
  - `/voice/<hostname>/cmd` (`std_msgs/String`) accepting `interrupt`,
    `resume`, and `abandon` style commands.
  - Convenience topics `/voice/<hostname>/{interrupt,resume,abandon}` emitting
    `std_msgs/Empty`.
- See `provision/services/VOICE_README.md` for detailed tuning guidance.

## `voice_node.py`
- Python ROS 2 node responsible for queueing utterances, synthesising audio via
  Piper or eSpeak, and playing back with `aplay`.
- Supports environment overrides: `PSY_VOICE_MODEL`, `PSY_VOICE_MODEL_FALLBACKS`,
  `PSY_PIPER_BIN`, `PSY_ESPEAK_*`.
- Handles interrupt, resume, and abandon operations by manipulating the playback
  process using POSIX signals.

## `ai_stack.sh`
- Queues Docker Engine packages and installs a launcher that ensures the Docker
  service is active before the stack starts.
- Writes a compose bundle under `/opt/psyched/docker/ai-stack` covering Ollama
  (default model `phi4`), Neo4j, Qdrant, and Coqui TTS with persistent volumes.
- Creates `/etc/default/psyched-ai-stack` so ports, credentials, and model names
  can be customised without editing the compose file.
- Launcher pre-pulls the configured Ollama model and runs
  `docker compose up --remove-orphans`, exposing the services on a dedicated
  `psy-ai-stack` Docker network.

## `mic.sh`
- Installs ALSA utilities and headers to enable audio capture.
- A companion runtime node `mic_node.py` (installed separately when needed)
  publishes voice activity detection (VAD) boolean frames on
  `/voice/<hostname>/vad` (`std_msgs/Bool`).
- Environment overrides for the node:
  - `PSY_MIC_SAMPLE_RATE` (default 16000)
  - `PSY_MIC_FRAME_MS` (10|20|30, default 20)
  - `PSY_MIC_VAD_AGGRESSIVE` (0-3, webrtcvad aggressiveness, default 2)
  - `PSY_MIC_VAD_PUBLISH_HZ` (max Boolean publish rate, default derived from frame)
  - `PSY_MIC_DEVICE` (ALSA device, default `default`)
  - `PSY_MIC_DISABLE_AUDIO` (if set, uses a synthetic silence source for tests)
  - Falls back to an internal energy threshold detector if `webrtcvad` is not
    available.

## `nav2.sh` and Bringup Configs
- `provision/bringup/nav2.sh` is launched through `psy bring up nav`
  (previously `psy bringup nav`).
- Configuration files:
  - `provision/bringup/nav2_params.yaml` - Nav2 behaviour tree and planner
    settings.
  - `provision/bringup/slam_params.yaml` - SLAM Toolbox tuning.
  - `provision/bringup/ekf.yaml` - Robot Localization EKF parameters (not
    launched by default but available for integration).

## `systemd/install_units.sh`
- Writes `/etc/systemd/system/psyched@.service`, substituting `ROS_DOMAIN_ID` and
  `RMW_IMPLEMENTATION` from the host TOML.
- Enables units for each launcher in `/etc/psyched` and reloads the systemd
  daemon.

## `tools/install.sh` and `tools/provision/bootstrap.sh`
- `install.sh` downloads the repo, extracts to `/opt/psyched`, links the CLI into
  `/usr/bin`, merges any existing workspace, and runs the bootstrap script.
- `bootstrap.sh` ensures executables have the correct permissions, links the CLI,
  prepares ROS APT sources via `setup_ros2.sh`, and optionally runs
  `psy host apply`.

For hardware-specific environment tweaks or manual overrides, create files under
`/etc/default/psyched-<service>` and export environment variables that the
launchers read before starting the ROS nodes.
