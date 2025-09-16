## Brief

This repo (psyche) is a small robot platform combining LLM-driven "forebrain" agents with deterministic
runtime components (drivers, safety filters) and an idempotent provisioning system. The goal of these
instructions is to help an AI coding assistant be productive quickly by calling out project structure,
developer workflows, and repository-specific conventions.

## Big picture
- Forebrain (LLM): plans, composes macros, and proposes actions. Look for agent code under service packages
  and in any future `tools/provision/services/*_code/` packages (example: `voice_node`, `mic_node`).
- Brainstem / Cerebellum: deterministic execution, drivers, and safety filters. These are installed into
  a shared `colcon` workspace and launched via generated systemd units.
- Provisioner (`tools/provision/host_provision.py`): copies/clones repos into `/opt/psyched` canonical
  locations, creates `/opt/psyched_workspace` (colcon workspace) and `/opt/psyched_venv` (shared venv), runs
  `rosdep` + `colcon build`, installs the CLI (`/usr/bin/psyched`) and creates `psyched-<wrapper>.service` units.

Key files: `README.md`, `AGENTS.md`, `tools/provision/host_provision.py`, `tools/provision/services/*.json`.

## How code is organized / conventions
- Service descriptors: `tools/provision/services/<service>.json` list `repos`, `launch_wrappers`, and `tools`.
  - Local ament/python packages live under `tools/provision/services/<service>_code/<package>` and are referenced
    from the service JSON with `local: true` + a `relpath`.
  - Launch wrappers are small ROS packages (must include `package.xml` and `setup.py`) that the provisioner
    installs and for which it generates systemd units.
- Python packaging: packages include `setup.py` (not a pure pyproject layout). Keep `resource/` and entry
  points consistent with existing packages (see `tools/provision/services/debug_log_code/...`).

## Developer workflows (commands you'll need)
- Install/provision host (quick): `curl -fsSL https://dancxjo.github.io/psyched | sudo bash` (uses `tools/install.sh`).
- Run provisioner manually: `sudo python3 tools/provision/host_provision.py hosts/<host>.json`.
- Update workspace: the provisioner runs `rosdep` and `colcon build` as user `pete`; confirm build success by
  checking `/opt/psyched_workspace/install/setup.bash`.
- CLI installed as `/usr/bin/psyched` provides: `psyched reprovision <host.json>`, `psyched update <host.json>`,
  `psyched deprovision <host.json> [--purge]`.

## Debugging / validation quick checks
- If a launch wrapper fails to start: `sudo systemctl status psyched-<wrapper>.service` and
  `sudo journalctl -u psyched-<wrapper>.service`.
- Confirm shared venv: `/opt/psyched_venv` exists and required pip packages installed.
- Workspace build: verify `/opt/psyched_workspace/install/setup.bash` exists after `colcon build`.

## Patterns to follow when editing or adding code
- Keep provisioning idempotent: follow the `host_provision.py` pattern for deterministic cloning, symlinks,
  and ownership (`chown`/`chmod`). Prefer `git` for updates rather than ad-hoc copying.
- Tool hooks: modules in `tools/provision/tools/*_tool.py` should implement `setup()` and optionally `teardown()`.
  Use these hooks for heavy OS-level installs (TTS engines, MBROLA voices, piper, etc.).
- Launch wrappers & services: include `package.xml` + `setup.py` so the provisioner can install them into the
  workspace and generate systemd units. Use `launch_wrappers` entries in service JSON to register them.

## Security / runtime notes
- Never convert LLM output directly to actuator commands — route through the brainstem safety filters.
- Cyclone DDS: the provisioner may install Cyclone and set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## Examples (concrete references)
- Local package example: `tools/provision/services/mic_code/mic_node/setup.py` and `mic_node/mic_node.py`.
- Launch wrapper example: `tools/provision/services/launch_wrappers/psyched_mic_launch/` (contains `package.xml` & `setup.py`).

## When unsure
- Prefer minimal, low-risk changes: add a small unit test or a tiny helper script under `tools/` and update
  `tools/provision/services/*` only when you must change provisioning behavior.

---
If you'd like, I can iterate on any unclear section or merge in an existing `.github/copilot-instructions.md`
if you have one to preserve. What would you like changed or added? 
