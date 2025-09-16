# psyched — a platform that gives robots a soul

Psyched is more than a provisioning helper. It's a pragmatic robot platform that pairs deterministic low-level control with a deliberative forebrain (LLM-backed) to provide long-term intent, macro composition, and an auditable "soul" for autonomous systems.

Design intent (short)
- Soul (Forebrain): an LLM agent that composes macros, schedules middle-term plans, and proposes high-level actions.
- Safety & Execution (Brainstem/Cerebellum): deterministic controllers, drivers, and safety filters that validate and execute forebrain plans.
- Reproducible runtime: idempotent provisioning, canonical clones, a shared workspace, and a shared Python venv so the system boots into a known state.

Quick install (Ubuntu 22.04)
```bash
curl -fsSL https://dancxjo.github.io/psyched | sudo bash
```
The installer configures `/opt/psyched*` artifacts, creates a shared `colcon` workspace and Python venv, and installs a small CLI to manage provisioning.

What Psyched provides
- A reproducible runtime: `/opt/psyched` (canonical clones), `/opt/psyched_workspace` (colcon workspace), and `/opt/psyched_venv` (shared venv).
- A place for service code and launch wrappers: `tools/provision/services/` holds descriptors and optional local packages.
- A CLI to manage host provisioning: `tools/provision/psyched_cli.py` → installed as `/usr/bin/psyched`.

Repository layout (quick)
- `hosts/*.json` — declare which services run on a host and optional install flags.
- `tools/provision/host_provision.py` — main provisioner (copies/clones repos, creates venv/workspace, runs `rosdep` + `colcon build`, installs CLI, and creates systemd units for launch wrappers).
- `tools/provision/services/*.json` — service descriptors listing `repos`, `launch_wrappers`, and optional `tools`.
- `tools/provision/services/*_code/` — local ament packages (example: `mic_node`, `voice_node`).
- `tools/provision/tools/*_tool.py` — per-tool installers exposing `setup()`/`teardown()` for heavy or OS-specific tooling.

Quick provisioning (example)
Make the repository available on the target host and run:
```bash
sudo python3 tools/provision/host_provision.py hosts/ear.json
```

Provisioner summary
- Ensures `pete` user and prepares `/opt/psyched*` directories.
- Copies local packages or clones remote repos to canonical paths.
- Creates `/opt/psyched_venv` and installs per-service pip requirements.
- Runs `rosdep` and `colcon build` as `pete`.
- Optionally installs Cyclone DDS and persists `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` when DDS services are present.
- Installs `/usr/bin/psyched` and creates systemd units for declared `launch_wrappers`.

CLI (installed as `/usr/bin/psyched`)
- `psyched reprovision <host.json>` — run provisioning for a host.
- `psyched update <host.json>` — update canonical clones and local services (git pulls) then reprovision.
- `psyched deprovision <host.json> [--purge]` — call per-tool `teardown()` hooks, remove symlinks, and optionally delete canonical clones and the CLI.

Developer notes (practical)
- Local code pattern: place ament packages under `tools/provision/services/<service>_code/<package>` and reference them from the service JSON with `local: true` and a `relpath`.
- Launch wrappers: include `launch/*.py`, `package.xml`, and `setup.py` so provisioning can install and generate systemd units.
- Tool hooks: modules in `tools/provision/tools/*_tool.py` should implement `setup()` and optional `teardown()` for heavy installs (TTS, MBROLA, piper, etc.).

Runtime and safety notes
- Never allow raw LLM output to command actuators directly — always route through the brainstem safety filters and explicit pre/postconditions.
- Provide simulation-first workflows: test macros/plans in sim before hardware.
- Persist logs and decisions for auditability (systemd logs, ros2 topic records, structured op-logs).

Troubleshooting (common checks)
- Systemd unit problems: `sudo systemctl status psyched-<wrapper>.service` and `sudo journalctl -u psyched-<wrapper>.service`.
- Workspace build: check `/opt/psyched_workspace/install/setup.bash` exists after `colcon build`.
- Shared venv: ensure `/opt/psyched_venv` exists and required Python packages are installed.

Next steps / ideas
- Add JSON schemas for service and macro definitions and an example `forebrain` agent package under `tools/provision/services/`.
- Add a `--sim` provisioning mode to prepare simulation-only dependencies and disable hardware units.

License / contribution
See the repository root for license and contribution guidelines.

---
If you'd like, I can now add a short example `hosts/*.json` that demonstrates how the forebrain and ear services are declared, or scaffold a tiny `forebrain` agent package to show macro composition.

