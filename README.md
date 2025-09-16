# psyched — Provisioned ROS2 Service Workspace

This repository contains provisioning tooling and local service code for the `psyched` project.


To install, from a system running Ubuntu Server 22.04, run:
```bash
curl -fsSL https://dancxjo.github.io/psyched | sudo bash
```


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


Psyched: Robot OS (vision & architecture)
-----------------------------------------

Psyched is intentionally more than "a ROS2 service workspace" — it's designed to be a lightweight, auditable robot operating system that combines deterministic low-level control with a higher-level, deliberative intelligence provided by a large language model (LLM). The provisioning tooling in this repo is the installer and initial system integrator for that OS.

Core architectural metaphor
- Cerebellum — low-latency motor control and reflexes: real-time controllers, motion primitives, low-level joints/controllers, sensor fusion loops. Implemented as a set of deterministic nodes and native drivers intended to run with real-time constraints (kernel/RT tuning or isolated control loops).
- Brainstem — hardware abstraction and safety-critical I/O: sensor drivers, actuators, communications (UART, SPI, I2C, serial), watchdogs, and emergency-stop handling. This layer exposes safe, rate-limited primitives to higher layers.
- Ear — perceptual front-ends and sensors: audio capture + preprocessing, microphone arrays, speech/VAD pipelines, and other exteroceptive sensors (camera, lidar). The `ear` maps directly to the `voice`/`mic` services in this repo but also embraces multimodal sensing.
- Forebrain — planning, memory, and deliberate decision-making: planners, mission managers, and the LLM agent(s). This is the place that composes middle-term commands, sequences of macros, and high-level goals.

How the LLM fits in
- The LLM is not a real-time controller. Instead, treat it as a deliberative forebrain that can operate at a slower cadence. It issues middle-term commands and composes macros (reusable sequences of lower-level primitives) that the cerebellum and brainstem execute.
- Examples:
	- A macro: "approach table → extend arm → open gripper → place object" — the LLM composes and parameterizes this macro, then sends it to the cerebellum/brainstem for execution.
	- A middle-term plan: "navigate to charging station, wait if busy, retry in 30s" — the LLM can schedule and monitor such plans.
- The LLM should be sandboxed and monitored: macro execution must be verified by the safety layer (brainstem) and should support explicit human-in-the-loop approvals for risky actions.

Macros, stacks, and guarantees
- Macros are first-class citizens: the system supports composing, storing, and invoking macros with parameters and pre/postconditions. Macros are versioned and can be replayed or simulated.
- Stacking: the forebrain can queue macros and stack them into higher-level behaviors. The brainstem enforces resource constraints and prevents conflicting macro executions (e.g., simultaneous incompatible actuator commands).
- Guarantees: low-level controllers should provide bounds (time, current draw, safe joint limits). The provisioner and packaged nodes should document those expectations.

Provisioning as OS installation
- `tools/provision/host_provision.py` and the `bootstrap.sh` are the installer and baseline system integrator: they create the canonical clone area (`/opt/psyched`), the global workspace (`/opt/psyched_workspace`), and the shared Python venv (`/opt/psyched_venv`). These are the OS-level artifacts that let the robot run reproducibly.
- Provisioning installs the pieces (drivers, launch wrappers, tools) and wires up systemd units so the system boots into a known state: the OS image + `psyched` provisioning forms a reproducible robot runtime.

Safety and operational constraints
- Never allow raw LLM output to control actuators directly. Always route commands through a validation layer in the brainstem that enforces safety policies and preconditions.
- Provide simulation-first workflows: run macros and plans in simulation before allowing them on hardware. The OS should make it easy to flip between `--sim` and `--hw` provisioning paths.
- Logging & observability: ensure macros, LLM decisions, and low-level telemetry are persisted and auditable. Systemd logs, ros2 topic recording, and structured op-logs are recommended.

Short roadmap / next steps
1. Define explicit JSON schemas for macros and middle-term plans and add them to `tools/provision/services` as example services.
2. Add a sandboxed LLM agent package under `tools/provision/services/forebrain_code/` that demonstrates macro composition, simulation dry-runs, and human approval flows.
3. Add a `--sim` provisioning mode to the provisioner that sets up simulation-only dependencies (gazebo/ros2_sim) and disables hardware-level systemd units.
4. Add CI checks that run simple macro simulations and unit tests for safety invariants.
5. Document runtime safety contracts for each ament package (expected rates, timeouts, resource limits).

If you'd like, I can (pick one):
- scaffold the LLM agent package and a macro JSON schema, or
- add a `--sim` mode to `host_provision.py` that installs simulation dependencies and writes alternative systemd unit templates, or
- create a small example that shows an LLM composing a macro and triggering a simulated run.

These additions will help the repo reflect Psyched's ambition: an OS that gives a robot long-term intent and a cautious, verifiable way to execute it.
