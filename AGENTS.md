# Guidelines
- After modifying ROS interfaces, attempt `colcon build --packages-select psyche_interfaces`.
- If `colcon` or ROS 2 is unavailable, note the limitation in the PR.
- Favor thorough inline documentation and add tests when feasible.
- After editing `Dockerfile` or compose files, run `docker compose -f forebrain.yaml build` to ensure images build successfully.
- Reuse Docker build caches when possible to avoid re-downloading dependencies.
- Document new services (e.g., Qdrant) in the Docker guides and create
  persistent data directories when adding them to compose files.
- When defining ROS messages, reference types from this package as
  `psyche_interfaces/MessageType` (without a `/msg` suffix).
