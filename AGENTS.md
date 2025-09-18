# Psyche Agent Notes

- When a provisioning service needs Docker, call `common_ensure_docker_runtime`
  from `provision/services/_common.sh` instead of duplicating package installs.
  The helper takes care of the official repository, keyring, and package queueing.
