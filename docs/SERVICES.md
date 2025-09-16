# Services provisioning

Services live at the repository root under `services/<service>`.

Each service directory should provide a minimal manifest and two scripts:

- `manifest.json` - describes the service, repositories to provision and optional launch wrappers.
- `setup.sh` - idempotent setup script. By default the script runs in dry-run mode; pass `--apply` to perform actions.
- `teardown.sh` - idempotent teardown script. By default dry-run; pass `--apply` to remove workspace links and systemd units.

Example layout:

services/foot/
- manifest.json
- setup.sh
- teardown.sh

Bootstrap behavior

The top-level `tools/provision/bootstrap.sh` will:
- Read the host config (e.g. `hosts/cerebellum.json`) and determine the `services` list.
- For each known service (files in `services/*.json`), it calls either the service `setup.sh` or `teardown.sh` depending on whether the service is enabled on the host.
- By default `bootstrap.sh` invokes service scripts in dry-run mode; pass `--apply` to the bootstrap script to forward `--apply` to each service script (TODO).

Service responsibilities

A service's `setup.sh` should:
- Clone or update canonical repositories under `/opt/psyched`.
- Symlink each repository into `/opt/psyched_workspace/src/<relpath>`.
- Run `rosdep install` for the workspace (non-root where possible).
- Run `colcon build` and optionally install the workspace.
- Create and enable a systemd unit that launches any required ROS 2 launch wrappers.

A service's `teardown.sh` should:
- Remove workspace symlinks and optionally purge canonical repos.
- Stop and disable the systemd unit installed by the service.

Security and safety

- All service scripts default to dry-run to avoid accidental destructive changes. To perform actions pass `--apply` and run with the appropriate privileges (e.g. `sudo`).
- Scripts prefer to run `rosdep` and build steps as the original sudo user when `SUDO_USER` is set.

Next steps

- Standardize a small Python helper library for common operations and refactor existing service scripts to use it.
- Add `--purge` support to allow removing canonical clones when cleaning up services.
