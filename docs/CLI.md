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

## `psy bring ...`
- `psy bring up [svc ..]` - Starts the named services through their
  `psyched@<svc>.service` units. When no services are supplied it auto-discovers
  launch scripts in `/etc/psyched` (falling back to installed units).
- `psy bring down [svc ..]` - Stops the selected services (or all discovered).
- `psy bring restart [svc ..]` - Convenience wrapper that restarts the chosen
  services in place.
- `psy bring status [svc ..]` - Prints the first few lines of
  `systemctl status` for each service, making it easy to spot failing units.

These subcommands share the same discovery logic as `psy systemd up/down`, so
either interface may be used interchangeably.

## `psy bringup nav`
- Backwards-compatible shim that now shells out to `psy bring up nav`, emitting
  a deprecation notice. Nav still launches via the systemd-managed
  `nav.launch.sh`.

## `psy systemd ...`
- `psy systemd install` - Installs or refreshes the templated `psyched@.service`
  unit and enables services for which launchers exist.
- `psy systemd info [svc ...]` - Lists installed `psyched@` units and, when
  service names are supplied, prints recent journalctl output plus the first few
  lines of `systemctl status` for each. Also available via `psy debug` (see
  below).
- `psy systemd up` / `down` - Share their service discovery with `psy bring`, so
  they accept optional service names and otherwise act on all available units.

## `psy debug [svc ..]`
- Shortcut for `psy systemd info`, combining a filtered `systemctl` summary
  with recent `journalctl` output and a brief status snippet for each supplied
  service.

All systemd operations shell out to `sudo systemctl`; expect to supply sudo
credentials when prompted.

## `psy say <text>`
- Publishes a one-shot `std_msgs/String` message to `/voice/<hostname>` using the
  `ros2 topic pub --once` CLI. The command auto-sources `/opt/ros/<distro>/setup.bash`
  if the ROS environment is not already in the shell.

## `psy bearing <degrees>`
- Computes the proportional angular velocity that the vision controller would use
  for the supplied bearing and publishes a single `geometry_msgs/msg/Twist` on
  `/cmd_vel` (configurable via `--topic`).
- Mirrors the defaults from `psyche_vision/object_controller.py`:
  gain of `2.0`, maximum angular velocity of `0.5` rad/s, and a ±`2°` deadzone.
- Options allow quick experimentation: `--gain`, `--max`, `--tolerance`,
  `--topic`, and `--dry-run` (print without publishing).
- Environment variables provide persistent overrides:
  `PSH_BEARING_GAIN`, `PSH_BEARING_MAX_ANGULAR`, `PSH_BEARING_TOLERANCE`, and
  `PSH_BEARING_TOPIC`.
- Example: `psh bearing 15` prints the computed command and publishes the
  matching Twist to `/cmd_vel` so the base rotates right.

## Helper Scripts

Two helper utilities ship alongside the CLI:

- `cli/psh` - Thin wrapper that simply execs `psy`.
- `cli/psy` (this script) - Sources `_common.sh` so shared helpers like
  `common_apt_install`, `common_install_launcher`, and `common_flush_apt_queue`
  are available to all subcommands.

For additional service-specific controls (e.g., environment variables consumed
by launchers), consult `docs/SERVICES.md` and the comments within each
`provision/services/*.sh` file.
