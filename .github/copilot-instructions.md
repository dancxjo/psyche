# psycheOS ROS2 Robotics System

psycheOS is a ROS2-based robotics framework for mobile platforms like the iRobot Create base. It provides modular service provisioning, workspace management, and system integration tools via the `psy` CLI.

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## Working Effectively

### Initial Setup and Installation
The system is designed to install to `/opt/psyched`. Use these commands:

- **Bootstrap the system (requires sudo):**
  ```bash
  sudo mkdir -p /opt/psyched
  sudo cp -r . /opt/psyched/
  sudo chown -R $USER:$USER /opt/psyched
  chmod +x /opt/psyched/cli/psy /opt/psyched/tools/provision/bootstrap.sh
  chmod +x /opt/psyched/provision/services/*.sh /opt/psyched/provision/systemd/*.sh
  ```

- **Create host configuration:**
  ```bash
  sudo cp /opt/psyched/provision/hosts/cerebellum.toml /opt/psyched/provision/hosts/$(hostname).toml
  ```

### Core Build Process
- **NEVER CANCEL**: All builds and service provisioning may take 10-45 minutes. Use timeouts of 60+ minutes for build commands and 30+ minutes for service provisioning.

- **Install dependencies (network dependent):**
  ```bash
  # ROS repository setup (may fail due to network restrictions)
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
  echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list
  sudo apt-get update
  
  # Fallback: install colcon via pip if ROS repo unavailable
  python3 -m pip install --user colcon-common-extensions
  export PATH="$HOME/.local/bin:$PATH"
  ```

- **Build workspace (NEVER CANCEL - timeout 60+ minutes):**
  ```bash
  cd /opt/psyched
  ./cli/psy build  # Takes ~5-45 minutes depending on packages
  ```

- **Service provisioning (NEVER CANCEL - timeout 60+ minutes):**
  ```bash
  cd /opt/psyched
  ./cli/psy host apply  # Installs ROS2 + services, takes 10-45 minutes
  ```

### Essential Commands and Timings
- **List available services:** `./cli/psy svc list` (~0.1s)
- **Enable service:** `./cli/psy svc enable <name>` (~0.1s)
- **Build workspace:** `./cli/psy build` (~0.3s empty, 5-45min with packages)
- **Install systemd units:** `./cli/psy systemd install` (~0.5s)
- **Bootstrap system:** `./tools/provision/bootstrap.sh --apply` (~10-45 minutes)

### Network Limitations Workarounds
- **ROS repository blocked:** Install colcon via pip instead of apt packages
- **GitHub cloning blocked:** Manually copy repositories to `ws/src/` if needed
- **API calls blocked:** Use direct download URLs for packages when available

## Validation

### Always Test After Changes
1. **Build validation:** `./cli/psy build` should complete without errors (may warn about missing ROS setup)
2. **Service listing:** `./cli/psy svc list` should show all 10 services
3. **Workspace structure:** Check `/opt/psyched/ws/src` exists and has expected packages

### Manual Testing Scenarios
- **CLI functionality:** Test all psy commands (host, svc, build, systemd, bringup)
- **Service provisioning:** Verify services can be enabled/disabled in host config
- **Systemd integration:** Check that systemd units are generated correctly
- **File permissions:** Ensure all scripts remain executable after changes

### Expected Failures
- **ROS commands fail** without proper ROS installation (expected in sandboxed environments)
- **Network-dependent operations** may timeout or fail (document as limitations)
- **Hardware-specific services** (imu, lidar, gps) will fail without physical hardware

## System Architecture

### Core Components
- **CLI tool:** `/opt/psyched/cli/psy` - main management interface
- **Host configs:** `/opt/psyched/provision/hosts/*.toml` - service configurations per host
- **Services:** `/opt/psyched/provision/services/*.sh` - modular provisioning scripts
- **Workspace:** `/opt/psyched/ws` - colcon workspace for ROS packages
- **Systemd units:** Template units for service management

### Available Services
1. **ros** - ROS2 base installation and environment setup
2. **workspace** - colcon workspace setup and build tools
3. **foot** - iRobot Create base support (libcreate + create_robot)
4. **imu** - MPU6050 IMU sensor support
5. **lidar** - LIDAR sensor integration
6. **camera** - USB camera support (usb_cam)
7. **gps** - U-blox GPS via nmea_navsat_driver
8. **mic** - Audio/microphone ALSA setup
9. **robot** - robot_state_publisher and URDF
10. **nav** - nav2 + slam_toolbox navigation stack

### Key Files and Locations
- **Main CLI:** `/opt/psyched/cli/psy`
- **Bootstrap:** `/opt/psyched/tools/provision/bootstrap.sh`
- **Host config example:** `/opt/psyched/provision/hosts/cerebellum.toml`
- **Service scripts:** `/opt/psyched/provision/services/*.sh`
- **Systemd template:** `/opt/psyched/provision/systemd/install_units.sh`
- **Navigation config:** `/opt/psyched/provision/bringup/nav2_params.yaml`

## Common Issues and Solutions

### Build Failures
- **"colcon: command not found"**: Install via `python3 -m pip install --user colcon-common-extensions` and add `~/.local/bin` to PATH
- **ROS setup not found**: Expected when ROS2 isn't installed; workspace will build but warn
- **Permission denied**: Ensure scripts are executable with `chmod +x`

### Service Provisioning Failures  
- **"Unable to locate package ros-dev-tools"**: ROS repository not configured, use pip fallbacks
- **Network timeouts**: Expected in sandboxed environments, document limitations
- **Hardware service failures**: Expected without physical sensors attached

### Best Practices
- **Always use full paths** when referring to `/opt/psyched` files
- **Set long timeouts** for any network or build operations (60+ minutes)
- **Test incrementally** - validate each command before proceeding
- **Check executability** of shell scripts after any file operations
- **Use host configs** to manage service sets per deployment target