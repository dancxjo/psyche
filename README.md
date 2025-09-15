# PsycheOS Stack

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
  - By default, it clones branch `main` into `/opt/psycheos`, group‑writable by `sudo`.

```
curl -fsSL https://raw.githubusercontent.com/dancxjo/psyche/main/tools/provision/bootstrap.sh | sudo bash
```

- What happens:
  - Clones this repo to `/opt/psycheos` and sets `sudo` group write permissions.
  - Installs `psycheos-updater.timer` which periodically runs an updater that:
    - `git fetch`/fast‑forwards the repo.
    - Re-runs `/opt/psycheos/tools/provision/apply.sh` to apply any changes for the host.
  - Runs the initial apply immediately.

- Apply behavior (host‑specific):
  - Parses `/opt/psycheos/devices/$(hostname -s).toml`.
  - Installs Layer 1 zenoh configs + `layer1-zenoh.service` and enables it.
  - If `[layer2].ros_distro != "none"`, sets up ROS 2 base for that distro, creates `/opt/ros_ws` and global shell env.
  - Installs `/etc/profile.d/psycheos.sh` to source RMW env, ROS 2, and workspace for all users.

### ROS 2 Machines

- Devices with `layer2.ros_distro` set (e.g., `"jazzy"`) will:
  - Add the ROS 2 apt repo and key.
  - Install `ros-<distro>-ros-base`, `python3-colcon-common-extensions`, `python3-rosdep`.
  - Create a global workspace at `/opt/ros_ws` (group‑writable for `sudo`).
  - Source `/opt/ros/<distro>/setup.sh` and `/opt/ros_ws/install/setup.sh` for all users.

### Keeping the Repo Up To Date

- Systemd timer: `psycheos-updater.timer` runs every 5 minutes.
- Service: `psycheos-updater.service` runs `/usr/local/bin/psycheos-update` which pulls the repo and re-applies provisioning.
 - Manual update at any time: run `/usr/bin/update-psyche` as root.
