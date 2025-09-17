# `psy` CLI Reference

The `psy` command manages host provisioning, services, workspace builds, and
systemd units for the Psyche robot stack. The executable lives at `/opt/psyched/cli/psy`
and is linked into `/usr/bin/psy` during bootstrap.

Run `psy` with no arguments to see the built-in usage text. Key subcommands are
summarised below.

## `psy host apply [<host>]`
- Reads the host definition from `provision/hosts/<host>.toml` (defaults to the
  current hostname).
- Executes each service's `provision` function in sequence, queueing apt packages
  via `_common.sh` so a single `apt-get install` runs at the end.
- After provisioning, triggers one workspace build by calling
  `provision/services/workspace.sh build`.
- Installs systemd units and starts launchers whose scripts are present in
  `/etc/psyched`.

## `psy svc ...`
- `psy svc list` - Lists available service scripts (`*.sh`) in
  `provision/services/`.
- `psy svc enable <name>` - Adds a service to the `services` array in the
  current host TOML. Uses `awk` to append the entry if it is missing.
- `psy svc disable <name>` - Removes the named service from the `services` array.

Services are idempotent; re-running `psy host apply` after editing the host file
resynchronises the machine with the desired configuration.

## `psy build`
- Ensures the workspace exists (merging a legacy `/opt/ros2_ws` if detected).
- Flushes any queued apt packages (`common_flush_apt_queue`).
- Delegates to `provision/services/workspace.sh build`, which sources ROS,
  resolves dependencies via `rosdep`, enables `ccache`, and runs `colcon build`.

## `psy bringup nav`
- Convenience launcher for manual navigation testing.
- Sources ROS if needed, then starts `provision/bringup/nav2.sh`, which launches
  `slam_toolbox` (online sync mode) and the Nav2 bringup launch file using the
  parameters under `provision/bringup/`.

## `psy systemd ...`
- `psy systemd install` - Installs or refreshes the templated `psyched@.service`
  unit and enables services for which launchers exist.
- `psy systemd info [svc ...]` - Lists installed `psyched@` units and, when
  service names are supplied, prints recent journalctl output plus the first few
  lines of `systemctl status` for each.
- `psy systemd up` - Starts all services that have launch scripts in
  `/etc/psyched`. Falls back to querying `systemctl list-unit-files` if no
  launchers are detected.
- `psy systemd down` - Stops all running `psyched@*.service` units.

All systemd operations shell out to `sudo systemctl`; expect to supply sudo
credentials when prompted.

## `psy say <text>`
- Publishes a one-shot `std_msgs/String` message to `/voice/<hostname>` using the
  `ros2 topic pub --once` CLI. The command auto-sources `/opt/ros/<distro>/setup.bash`
  if the ROS environment is not already in the shell.

## Helper Scripts

Two helper utilities ship alongside the CLI:

- `cli/psh` - Thin wrapper that simply execs `psy`.
- `cli/psy` (this script) - Sources `_common.sh` so shared helpers like
  `common_apt_install`, `common_install_launcher`, and `common_flush_apt_queue`
  are available to all subcommands.

For additional service-specific controls (e.g., environment variables consumed
by launchers), consult `docs/SERVICES.md` and the comments within each
`provision/services/*.sh` file.
