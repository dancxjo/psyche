# psyched — Provisioned ROS2 Service Workspace

This repository contains provisioning tooling and local service code for the `psyched` project.

High-level goals
- Provide an idempotent, host-aware provisioning entrypoint that sets up a global ROS2 workspace and canonical clones.
- Allow services to be declared per-host in `hosts/*.json` files and implemented either as remote Git repos or local code under `tools/provision/services/*_code`.
- Provision heavy system and TTS tooling only when requested by a host's services (per-tool `setup()` and `teardown()` hooks).
- Provide a single shared Python virtualenv for runtime dependencies used by services and CLI.
- Install systemd units for ROS2 launch wrappers so services start at boot.

Repository layout (important files)
- `hosts/*.json` — per-host configuration listing `services`, and optional `install_ros2` and `repos`.
- `tools/provision/host_provision.py` — main Python provisioner. Reads host JSON, expands services, copies or clones repos to `/opt/psyched`, symlinks them into `/opt/psyched_workspace/src`, installs system-level deps, creates a shared venv, runs `rosdep` and `colcon build`, installs helper CLI, and creates systemd units for launch wrappers.
- `tools/provision/services/*.json` — service descriptors. Each service lists `repos` (remote or `local: true`) and optional `launch_wrappers` (local wrapper packages).
- `tools/provision/services/*_code/*` — local ament packages used as local service sources (e.g., `mic_node`, `voice_node`, `debug_log_node`).
- `tools/provision/tools/*_tool.py` — per-tool best-effort installers with `setup()` and optional `teardown()` hooks (e.g., `piper_tool.py`, `espeak_tool.py`, `mbrola_tool.py`).
- `tools/provision/psyched_cli.py` — CLI helper (installed at `/usr/bin/psyched` during provisioning) with commands `psyched reprovision`, `psyched update`, and `psyched deprovision`.

Key behaviors
- Canonical clones: `/opt/psyched/<relpath>` — canonical copies of repos. The provisioner copies local code there and clones remotes.
- Global workspace: `/opt/psyched_workspace` — the colcon workspace. The provisioner symlinks canonical clones into `.../src/` and runs `colcon build` as user `pete`.
- Shared Python venv: `/opt/psyched_venv` — a shared virtual environment where per-service pip requirements are installed. The provisioner writes `/etc/profile.d/psyched_venv.sh` to make it available to shells and the systemd unit templates source it at ExecStart.
- RMW selection: When services requiring DDS are present (`mic`, `voice`, `debug_log`), we attempt to install Cyclone DDS and append `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `/etc/profile.d/psyched_workspace.sh` so shells and logins pick it up.
- Systemd units: For each local `launch_wrapper` package the provisioner creates a systemd unit named `psyched-<wrapper>.service` that runs `ros2 launch <wrapper> <launch_py>` as user `pete`, sourcing the workspace and venv first.
- Per-tool installation: `tools/provision/tools/*.py` modules expose `setup()` and `teardown()` to allow best-effort installation and cleanup for heavy or OS-specific tooling.

Quick provisioning (on target host)
1. Copy repository to the host or make it available under `/opt/psyched`.
2. Run the provisioner as root (example):

```bash
sudo python3 tools/provision/host_provision.py hosts/ear.json
```

This will:
- Ensure `pete` user exists
- Create `/opt/psyched` and `/opt/psyched_workspace`
- Copy/clone service repos
- Create `/opt/psyched_venv` and install service pip deps
- Attempt Cyclone DDS installs and persist RMW env
- Run `rosdep` and `colcon build` as `pete`
- Install `/usr/bin/psyched` helper CLI
- Create and enable systemd units for launch wrappers

Using the CLI
- `sudo /usr/bin/psyched reprovision hosts/ear.json` — run provisioning again
- `sudo /usr/bin/psyched update hosts/ear.json` — update code (git pulls) and reprovision
- `sudo /usr/bin/psyched deprovision hosts/ear.json [--purge]` — remove symlinks, call tool teardown hooks, and optionally purge canonical repos and remove the CLI wrapper

Developer notes
- Local code pattern: place local packages under `tools/provision/services/<service>_code/<package>` and list them in the service's `<service>.json` with `local: true` and a `relpath` to use under `/opt/psyched`.
- Launch wrappers: include a `launch/<..._launch.py>` file, a `package.xml`, and a `setup.py` so provisioning can copy and install it into the workspace. The systemd unit generator looks for `launch/*.py` files when creating units.
- Testing: run `colcon build` as `pete` inside `/opt/psyched_workspace` after provisioning steps complete. Use `ros2 topic echo` and `ros2 run` to validate nodes.

If you need me to adapt the provisioning behavior (different venv path, different unit templates, container-based deployment), say so and I will implement it.
