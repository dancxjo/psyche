# Docker Environment Setup

This guide explains how to build and run the Psyche system inside Docker. The repository provides several helper scripts and compose files to streamline the process.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/) and [Docker Compose](https://docs.docker.com/compose/install/) installed on the host machine.
- Optional: hardware such as audio devices or GPIO should be made accessible to Docker if required.

## Building the Image
Run the following command from the project root to build the Docker image using the provided compose configuration:

```bash
# build all services defined in forebrain.yaml
docker compose -f forebrain.yaml build
```

This step caches packages so subsequent builds are faster. If you modify the `Dockerfile` or compose files, rerun the build command.

## Running the Container
Two helper scripts exist under `launch/` for different hardware setups:

- `launch/forebrain.sh` – launches the main ROS stack alongside additional services like OLLAMA and Neo4j.
- `launch/motherbrain.sh` – tailored for a single-board computer with attached devices.

Execute the appropriate script to start the containers in detached `screen` sessions:

```bash
./launch/forebrain.sh
```

Each script invokes `docker compose` with the matching YAML file so you do not need to run compose manually.

## Accessing the Container
Once running, attach to the container with:

```bash
./terminal.sh
```

This opens an interactive shell inside the `psyche` service.

## Host Setup Script
For a bare‑metal installation (without Docker) a `setup.sh` script is provided. It installs ROS, Python dependencies, and configures the environment:

```bash
sudo ./setup.sh
```

This script is not required for Docker usage but can be helpful for troubleshooting or development directly on the host.

## Stopping Services
To stop the running containers use standard Docker Compose commands. For example:

```bash
docker compose -f forebrain.yaml down
```

The `screen` sessions created by the launch scripts can also be terminated with `screen -ls` and `screen -X -S <name> quit` if needed.

