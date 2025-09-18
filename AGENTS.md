# Psyche Agent Notes

- When a provisioning service needs Docker, call `common_ensure_docker_runtime`
  from `provision/services/_common.sh` instead of duplicating package installs.
  The helper takes care of the official repository, keyring, and package queueing.
- Full `pytest` runs pull in `src/psyche_vision/test_vision.py`, which imports
  `numpy`. Install it (`pip install numpy`) before executing the entire suite or
  target narrower tests when the dependency is unavailable.
