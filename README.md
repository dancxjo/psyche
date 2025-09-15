# psyched Stack

This repository scaffolds a multi-layer robotics stack.

## Layers

1. **Layer 0** – Debian-based OS (e.g. Ubuntu Server, Pi OS Lite).
2. **Layer 1** – Zenoh fabric and auto-networking scripts.
3. **Layer 2** – ROS 2 runtime configuration.
4. **Layer 3** – Device-specific service launcher driven by TOML files.

Each device provides a `devices/<hostname>.toml` describing its roles and
runtime configuration.

## Provisioning via curl | sudo bash

- One-liner bootstrap (clones the repo globally, installs auto-updater, applies config):
  - Replace the URL with where you host raw files for this repo.
  - By default, it clones branch `main` into `/opt/psyched`, group‑writable by `sudo`.

```
curl -fsSL https://dancxjo.github.io/psyched | sudo bash
```

- What happens:
  - Clones this repo to `/opt/psyched` and sets `sudo` group write permissions.
  - Installs `psyched-updater.timer` which periodically runs an updater that:
    - `git fetch`/fast‑forwards the repo.
    - Re-runs `/opt/psyched/tools/provision/apply.sh` to apply any changes for the host.
  - Runs the initial apply immediately.

- Apply behavior (host‑specific):
  - Parses `/opt/psyched/devices/$(hostname -s).toml`.
  - Installs Layer 1 zenoh configs + `layer1-zenoh.service` and enables it.
  - If `[layer2].ros_distro != "none"`, sets up ROS 2 base for that distro, creates `/opt/ros_ws` and global shell env.
  - Installs `/etc/profile.d/psyched.sh` to source RMW env, ROS 2, and workspace for all users.

### ROS 2 Machines

- Devices with `layer2.ros_distro` set (e.g., `"jazzy"`) will:
  - Add the ROS 2 apt repo and key.
  - Install `ros-<distro>-ros-base`, `python3-colcon-common-extensions`, `python3-rosdep`.
  - Create a global workspace at `/opt/ros_ws` (group‑writable for `sudo`).
  - Source `/opt/ros/<distro>/setup.sh` and `/opt/ros_ws/install/setup.sh` for all users.

### Keeping the Repo Up To Date

- Systemd timer: `psyched-updater.timer` runs every 5 minutes.
- Service: `psyched-updater.service` runs `/usr/local/bin/psyched-update` which pulls the repo and re-applies provisioning.
 - Manual update at any time: run `/usr/bin/update-psyched` as root.

## ROS 2 Auditor (Docker)

A host-like ROS 2 container using docker-compose. It uses host networking so it participates in DDS discovery on your LAN like a normal machine. The service is named `auditor` by default.

Quick start:

1) Copy env template and set your UID/GID and ROS distro

```
cp .env.example .env
sed -n '1,200p' .env
```

2) Build the image

```
docker compose build auditor
```

3) Start the container (detached)

```
docker compose up -d auditor
```

4) Exec into the container and use ROS 2

```
docker compose exec auditor bash
ros2 node list
```

Notes:
- Networking: `network_mode: host` enables multicast discovery and lets the container behave like a ROS 2 host.
- Domain: Set `ROS_DOMAIN_ID` in `.env` to match other machines (default `0`).
- Workspace: Your repo mounts to `/workspaces/psyche`; an overlay workspace mounts at `/workspaces/ros2_ws`.
- Middleware: Default `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`; change if your fleet uses another RMW.
- Distro: Defaults to `jazzy` (override via `.env` or build arg).
- GUI tools (optional): Uncomment X11 `DISPLAY` and volumes in `docker-compose.yml` for `rqt`, `rviz2` on Linux.
- Devices (optional): Uncomment `privileged` and `devices` in `docker-compose.yml` to access serial, cameras, etc.

Image details:
- Dockerfile uses `ros:${ROS_DISTRO}-ros-base` and installs `python3-colcon-common-extensions`.
- Non-root user `rosuser` matches your host UID/GID to avoid volume permission issues.
- Entry point sources `/opt/ros/$ROS_DISTRO/setup.bash` and overlays `/workspaces/ros2_ws/install/setup.bash` if present.

## Role-Based Stacks

Devices declare roles in their TOML (e.g., `devices/forebrain.toml`). The provisioner enables stacks based on roles:

- Forebrain: runs docker compose services for `ollama` and `coqui-tts`.
- Cerebellum: runs docker compose services for `ollama`, `qdrant`, and `neo4j`.

Files:
- `containers/forebrain/docker-compose.yml`
- `containers/cerebellum/docker-compose.yml`
- `systemd/forebrain-containers.service`
- `systemd/cerebellum-containers.service`

Data persistence paths (auto-created by apply):
- `/opt/psyched/data/ollama`
- `/opt/psyched/data/qdrant`
- `/opt/psyched/data/neo4j/{data,logs,plugins}`

Notes:
- Requires Docker + Compose v2 on the host.
- Provisioner auto-installs Docker on devices with roles `forebrain` or `cerebellum` (packages: `docker.io`, `docker-compose-plugin`).
- Units are enabled automatically if the device role list contains `"forebrain"` and/or `"cerebellum"`.
- Change images/ports/env in the compose files to match your environment; default ports: Ollama `11434`, Qdrant `6333/6334`, Neo4j `7474/7687`, Coqui TTS `5002`.

## Audio/Video via Zenoh

Ear (mic) and Cerebellum (camera) can publish raw streams over Zenoh for bridging into ROS 2.

- Mic (Ear):
  - Role: add `"mic"` to `[device].roles` in the ear TOML
  - Service: `zenoh-audio-pub.service`
  - Publishes: PCM16 mono at 16kHz in 20ms chunks to key `rt/audio/ear0`
  - Config via env: `AUDIO_KEY`, `AUDIO_RATE`, `AUDIO_CHUNK_MS`, `AUDIO_CHANNELS`, `AUDIO_DEVICE`
- Camera (Cerebellum):
  - Role: add `"camera"` to `[device].roles` in the cerebellum TOML
  - Service: `zenoh-camera-pub.service`
  - Publishes: JPEG frames to key `rt/vision/cam0` (default 640x480@15fps, quality 80)
  - Config via env: `CAM_KEY`, `CAM_DEVICE`, `CAM_WIDTH`, `CAM_HEIGHT`, `CAM_FPS`, `CAM_QUALITY`

Files:
- `layer3/services/zenoh_audio_pub.py` — arecord-based PCM publisher (no external audio lib)
- `layer3/services/zenoh_camera_pub.py` — OpenCV-based JPEG publisher
- `systemd/zenoh-audio-pub.service`, `systemd/zenoh-camera-pub.service`

Bridge to ROS 2:
- Option A: Use the included ROS 2 republishers on Cerebellum:
  - `ros2-audio-repub.service` republishes `rt/audio/ear0` to `/audio/ear0/raw` (`std_msgs/UInt8MultiArray`).
  - `ros2-camera-repub.service` republishes `rt/vision/cam0` to `/camera/cam0/image/compressed` (`sensor_msgs/CompressedImage`).
  - Enabled automatically on devices with role `"cerebellum"`.
- Option B: Use `layer1-bridge.service` with custom mappings in `layer1/zenoh/bridge_ros2.json5`.
