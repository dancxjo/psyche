Psyche: Layer3 services notes
===============================

This file documents how to enable and run Layer3 services (web UI and the
`psyche_ublox_publisher` ROS package) on a device.

1) Enabling the web service via device TOML
------------------------------------------
Add a `layer3.services.web` section to the device TOML (for ROS-enabled hosts).
Example in `devices/ear.toml`:

[layer3.services.web]
enabled = true
host = "0.0.0.0"
port = 8080

Provisioning (`tools/provision/apply.sh`) will install dependencies into
`/opt/psyched/venv` and create `psyche-web.service` which runs Uvicorn
serving `layer3.web.app`.

2) Building the `psyche_ublox_publisher` package
------------------------------------------------
The package is located at `layer3/ros2_pkgs/psyche_ublox_publisher`.
To include it in the ROS workspace and build:

```
sudo mkdir -p /opt/ros_ws/src
sudo cp -r layer3/ros2_pkgs/psyche_ublox_publisher /opt/ros_ws/src/
sudo chown -R root:root /opt/ros_ws/src/psyche_ublox_publisher
cd /opt/ros_ws
colcon build --merge-install
source /opt/ros_ws/install/setup.bash
ros2 run psyche_ublox_publisher ublox_publisher
```

3) Accessing the web UI
----------------------
- `GET /topics` — returns a JSON list of topics
- `GET /topic/{topic_name}/history` — returns recent text history for that topic
- `WS  /ws/topic/{topic_name}` — open a websocket and receive live textual lines

Notes:
- The web app uses `ros2 topic echo -p` to stream topic output as text. It's a
  pragmatic approach that avoids importing every message type. For structured
  message streaming we could implement type-aware serializers.
- The server expects `ros2` to be available in the environment; the systemd unit
  sources `/opt/ros_ws/install/setup.sh` if present before starting Uvicorn.
