# PsycheOS Stack

This repository scaffolds a multi-layer robotics stack.

## Layers

1. **Layer 0** – Debian-based OS (e.g. Ubuntu Server, Pi OS Lite).
2. **Layer 1** – Zenoh fabric and auto-networking scripts.
3. **Layer 2** – ROS 2 runtime configuration.
4. **Layer 3** – Device-specific service launcher driven by TOML files.

Each device provides a `devices/<hostname>.toml` describing its roles and
runtime configuration.
