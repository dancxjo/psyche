# Agents & Provisioning Hooks

This project uses a small set of programmatic "agents" and hooks to keep provisioning idempotent and modular.

Primary agent: `host_provision.py`
- Responsibilities:
  - Read a host descriptor (`hosts/<host>.json`) and expand service definitions.
  - Create canonical clones under `/opt/psyched` and symlink them into the workspace `/opt/psyched_workspace/src`.
  - Install system packages and create a shared Python virtualenv at `/opt/psyched_venv` for Python dependencies.
  - Attempt to install Cyclone DDS (for consistent RMW) and persist `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in `/etc/profile.d/psyched_workspace.sh`.
  - Run `rosdep` and `colcon build` as `pete`.
  - Install a CLI wrapper at `/usr/bin/psyched` (module `tools.provision.psyched_cli`) to allow `reprovision`, `update`, and `deprovision` operations.
  - Create systemd units for any `launch_wrappers` provided by services so nodes start at boot.

Service descriptor hooks
- Each service JSON under `tools/provision/services/*.json` contains:
  - `repos`: list of repos (local or remote) to add to canonical clones and symlink into the workspace.
  - `launch_wrappers`: optional list of local launch wrapper packages (these are installed and have systemd units created).

Tool hooks
- Tools under `tools/provision/tools/*_tool.py` should expose two functions (best-effort):
  - `setup()` — perform any OS package installs and configuration needed for the tool (e.g., installing `espeak-ng`, MBROLA voices, or `piper`).
  - `teardown()` — undo or remove the tool when a host is deprovisioned (best-effort; may be a no-op on some tools).

CLI agent: `psyched_cli` (wrapper `/usr/bin/psyched`)
- `psyched reprovision <host.json>` — re-run provisioning for the host.
- `psyched update <host.json>` — update canonical clones (git pulls) and local services, then reprovision.
- `psyched deprovision <host.json> [--purge]` — call tool teardowns, remove symlinks, and if `--purge` delete canonical clones and the CLI wrapper.

Systemd lifecycle
- Units generated use `User=pete`, `Restart=always`, and source both the ROS workspace and the shared Python venv before running `ros2 launch` for the wrapper package and launch file.

Design principles & guidelines for contributors
- Keep heavy package installs out of global provisioning; only install tool dependencies when a service explicitly lists a dependency via the tool hooks.
- Service descriptors should be deterministic and include enough information to copy or clone repositories into the canonical directory with a stable `relpath`.
- Keep the provisioner idempotent: re-running should not duplicate or break the system. Use `git` for updates and `chown`/`chmod` for ownership management.

Troubleshooting
- If a systemd unit doesn't run:
  - Check `sudo systemctl status psyched-<wrapper>.service` and `sudo journalctl -u psyched-<wrapper>.service`.
  - Verify the workspace build succeeded and `/opt/psyched_workspace/install/setup.bash` exists.
  - Verify the shared venv at `/opt/psyched_venv` exists and that required Python packages are installed.

If you want a more orchestrated or container-based approach (kubernetes, podman-compose), the current patterns can be adapted to produce container images or Dockerfiles per-service.
