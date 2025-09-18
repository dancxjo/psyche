# Psyche Robotics Stack

Psyche provides a reproducible ROS 2 stack for the "psyched" robot platform. The
repository bundles a provisioning system, systemd launchers, hardware bring-up
scripts, and the `psyche_vision` perception package, along with a lightweight CLI
tool for day-to-day host management.

The project is intended for single-board computers (tested on Raspberry Pi 5)
controlling an iRobot Create base with camera, lidar, IMU, GPS, and voice I/O.

## Highlights
- Turn-key installer (`tools/install.sh`) that deploys to `/opt/psyched` and links
the CLI into `/usr/bin`.
- Host provisioning driven by TOML (`provision/hosts/*.toml`) and per-service
shell scripts under `provision/services/`.
- Automated systemd integration via templated `psyched@<svc>.service` units.
- ROS 2 workspace at `/opt/psyched/ws` with the `psyche_vision` nodes and launch files.
- Voice stack combining Piper TTS with a ROS node that exposes queueing and
runtime controls.

## Repository Layout
| Path | Purpose |
|------|---------|
| `cli/` | Command-line entrypoints (`psy`, `psh`). |
| `provision/` | Host configs, service provision scripts, bringup parameters, systemd tools. |
| `tools/` | Bootstrap installer and ROS APT setup helpers. |
| `ws/` | Colcon workspace containing the `psyche_vision` package and tests. |
| `docs/` | Supplementary documentation authored for this repository. |

## Quick Start
1. **Install** - on the target host run the bootstrap installer:
   ```bash
   curl -fsSL https://dancxjo.github.io/psyched | sudo bash
   ```
   This fetches the repo, installs into `/opt/psyched`, and runs
   `tools/provision/bootstrap.sh` in apply mode.
2. **Pick a host profile** - copy `provision/hosts/cerebellum.toml` or author a new
   `*.toml` file that lists the services to enable. The default expects ROS 2 Jazzy
   and enumerates each hardware service.
3. **Apply provisioning** - once installed, apply the chosen host definition:
   ```bash
   sudo psy host apply cerebellum
   ```
   Provisioning queues apt packages, clones required ROS sources, and writes
   launchers under `/etc/psyched/`.
4. **Build the workspace** - provisioning triggers a single `colcon build` via
   `provision/services/workspace.sh build`. You can re-run at any time:
   ```bash
   psy build
   ```
5. **Bring services online** - install or refresh systemd units:
   ```bash
   psy systemd install
   psy systemd up            # start all launchers immediately
   sudo systemctl status psyched@vision.service
   ```
   Use `psy systemd down` to stop all, or manage individual units with
   `sudo systemctl restart psyched@<name>.service`.

## Operations Cheat Sheet
- List available services: `psy svc list`
- Enable/disable a service in the host TOML: `psy svc enable lidar`
- See recent systemd logs: `psy systemd info vision`
- Publish speech: `psy say "Hello from Psyche"`

The CLI sources `_common.sh` helpers that consolidate apt installs, ensure the
workspace is symlinked correctly, and create launchers in `/etc/psyched`.

## ROS Workspace Overview
- Repository-managed packages live under `src/` and are copied into the active
  workspace during `psy build` / `workspace.sh` runs.
- `src/psyche_vision` contains `object_detector` and `object_controller` nodes,
  launch descriptions (`vision_launch.py`, `face_object.launch.py`), and a README
  documenting the perception pipeline.
- `src/psyche_vision/test_vision.py` offers a ROS-free smoke test for the
  controller maths and HSV parameters.
- External dependencies (e.g., `libcreate`, `kinect_ros2`) are cloned by service
  scripts directly into the workspace during provisioning.

See `docs/WORKSPACE.md` for full package breakdown and build behaviour.

## Additional Documentation
- `docs/ARCHITECTURE.md` - end-to-end system architecture and data flow.
- `docs/SERVICES.md` - per-service provisioning actions, launch commands, and
  runtime considerations.
- `docs/CLI.md` - complete CLI reference with subcommand semantics.
- `docs/WORKSPACE.md` - workspace layout, build tooling, and dependency
  resolution strategy.
- `provision/services/VOICE_README.md` - Piper voice stack tuning and model cache
  management.
- `src/psyche_vision/README.md` - perception pipeline details and tuning tips.

## Support & Contribution Notes
- Scripts assume Ubuntu-based environments with `sudo` available; see each
  service script for optional environment overrides.
- When adding hardware, define a new service script under `provision/services/`
  and include it in your host TOML.
- Follow the existing pattern of queueing apt packages (`PSY_DEFER_APT=1`) and
  using `_common.sh` helpers to keep provisioning idempotent.
