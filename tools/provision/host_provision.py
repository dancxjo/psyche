#!/usr/bin/env python3
"""
Improved host provisioner.

This script replaces the shell helper and global workspace setup logic.
It performs the following idempotent steps:
 - ensures system user 'pete' exists
 - creates workspace and canonical clone directories and fixes ownership
 - clones or updates specified repositories as 'pete'
 - symlinks canonical clones into the workspace src
 - runs rosdep and colcon build for the workspace (as 'pete')

Usage: host_provision.py /path/to/host.json
"""
import json
import os
import shlex
import subprocess
import sys
import pwd
from pathlib import Path

# Tool hooks for optional TTS provisioning
try:
    from tools.provision.tools import piper_tool, mbrola_tool, espeak_tool
except Exception:
    piper_tool = None
    mbrola_tool = None
    espeak_tool = None


def run(cmd, check=True, capture_output=False, env=None):
    print("+ ", " ".join(cmd))
    return subprocess.run(cmd, check=check, capture_output=capture_output, env=env)


def run_sudo(cmd):
    return run(["sudo"] + cmd)


def run_as_pete(cmd_str):
    # Run a shell command as user pete via sudo -u pete bash -lc
    full = ["sudo", "-u", "pete", "bash", "-lc", cmd_str]
    return run(full)


def ensure_pete():
    try:
        run(["id", "-u", "pete"], check=True)
        print("user 'pete' exists")
    except subprocess.CalledProcessError:
        print("creating system user 'pete'")
        run_sudo(["useradd", "--system", "--no-create-home", "--shell", "/usr/sbin/nologin", "pete"])


def ensure_dirs(workspace_dir: Path, clone_dir: Path):
    print(f"Ensuring workspace {workspace_dir} and clone dir {clone_dir} exist")
    run_sudo(["mkdir", "-p", str(workspace_dir)])
    run_sudo(["mkdir", "-p", str(clone_dir)])
    run_sudo(["chown", "-R", "pete:root", str(clone_dir)])
    run_sudo(["chown", "-R", "pete:root", str(workspace_dir)])
    run_sudo(["chmod", "0755", str(clone_dir)])
    run_sudo(["chmod", "0775", str(workspace_dir)])


def copy_current_user_ssh_to_pete():
    """Copy the invoking user's public SSH key(s) into /home/pete/.ssh/authorized_keys.

    Uses SUDO_USER if available, else falls back to $USER or getpass.getuser(). This is
    best-effort and will not error out provisioning if a key cannot be found or written.
    """
    try:
        src_user = os.environ.get("SUDO_USER") or os.environ.get("USER")
        if not src_user:
            try:
                import getpass

                src_user = getpass.getuser()
            except Exception:
                src_user = None

        if not src_user:
            print("No invoking user detected for SSH key copy; skipping")
            return

        if src_user == "pete":
            print("Invoking user is 'pete'; no key copy needed")
            return

        try:
            user_info = pwd.getpwnam(src_user)
            user_home = Path(user_info.pw_dir)
        except KeyError:
            print(f"Could not determine home directory for user {src_user}; skipping SSH key copy")
            return

        ssh_dir = user_home / ".ssh"
        pub_candidates = [
            ssh_dir / "id_ed25519.pub",
            ssh_dir / "id_rsa.pub",
            ssh_dir / "id_ecdsa.pub",
            ssh_dir / "id_dsa.pub",
            ssh_dir / "authorized_keys",
        ]

        found_keys = []
        for p in pub_candidates:
            try:
                if p.exists():
                    text = p.read_text().strip()
                    for line in text.splitlines():
                        line = line.strip()
                        if line:
                            found_keys.append(line)
            except Exception:
                # ignore unreadable files
                continue

        if not found_keys:
            print(f"No public SSH keys found in {ssh_dir} for user {src_user}; skipping")
            return

        # Ensure pete home and .ssh exist (create early so SSH can write known_hosts)
        pete_home = Path("/home/pete")
        run_sudo(["mkdir", "-p", str(pete_home)])
        run_sudo(["chown", "-R", "pete:root", str(pete_home)])
        run_sudo(["chmod", "0755", str(pete_home)])

        pete_ssh = pete_home / ".ssh"
        run_sudo(["mkdir", "-p", str(pete_ssh)])
        run_sudo(["chown", "-R", "pete:root", str(pete_ssh)])
        run_sudo(["chmod", "0700", str(pete_ssh)])

        auth_keys = pete_ssh / "authorized_keys"
        # ensure the file exists so later grep/append won't fail
        try:
            if not auth_keys.exists():
                p = subprocess.Popen(["sudo", "tee", str(auth_keys)], stdin=subprocess.PIPE)
                p.communicate(input=("\n").encode())
                run_sudo(["chmod", "0600", str(auth_keys)])
                run_sudo(["chown", "pete:root", str(auth_keys)])
        except Exception:
            # non-fatal; we'll attempt to append keys later
            pass
        for key in found_keys:
            # append if not already present
            q = shlex.quote(key)
            cmd = (
                f"bash -lc 'grep -F {q} {shlex.quote(str(auth_keys))} >/dev/null 2>&1 || echo {q} | tee -a {shlex.quote(str(auth_keys))}'"
            )
            try:
                run_sudo(["bash", "-lc", cmd])
            except Exception as e:
                print(f"Failed to add key for {src_user} to {auth_keys}: {e}")

        # Ensure ownership and permissions
        run_sudo(["chown", "-R", "pete:root", str(pete_ssh)])
        run_sudo(["chmod", "0600", str(auth_keys)])
        print(f"Copied SSH keys from {src_user} to /home/pete/.ssh/authorized_keys")
    except Exception as e:
        print(f"Unexpected error while copying SSH keys to pete: {e}")


def ensure_cyclone_dds_if_needed(services, ros_distro: str = ""):
    """If any service requires DDS, attempt to install Cyclone DDS RMW and persist RMW env var.

    This is best-effort and will not fail provisioning if packages are unavailable.
    """
    needed = {"mic", "voice", "debug_log"}
    if not needed.intersection(set(services)):
        return

    print("Ensuring Cyclone DDS (rmw_cyclonedds_cpp) is installed and configured")
    run_sudo(["apt", "update"])

    candidates = []
    if ros_distro:
        candidates.append(f"ros-{ros_distro}-rmw-cyclonedds-cpp")
    candidates.extend([
        "ros-rmw-cyclonedds-cpp",
        "rmw-cyclonedds-cpp",
        "libcyclonedds-dev",
        "cyclonedds",
    ])

    for pkg in candidates:
        try:
            print(f"Attempting to install {pkg}")
            run_sudo(["apt", "install", "-y", pkg],)
        except Exception as e:
            print(f"install {pkg} failed (continuing): {e}")

    # Persist RMW implementation in profile script if not already present
    profile_script = Path("/etc/profile.d/psyched_workspace.sh")
    export_line = "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
    try:
        if profile_script.exists():
            content = profile_script.read_text()
            if "RMW_IMPLEMENTATION" not in content:
                p = subprocess.Popen(["sudo", "tee", "-a", str(profile_script)], stdin=subprocess.PIPE)
                p.communicate(input=export_line.encode())
        else:
            content = """#!/bin/sh
if [ -f /opt/psyched_workspace/install/setup.bash ]; then
    . /opt/psyched_workspace/install/setup.bash
fi
"""
            content += export_line
            p = subprocess.Popen(["sudo", "tee", str(profile_script)], stdin=subprocess.PIPE)
            p.communicate(input=content.encode())
        run_sudo(["chmod", "0644", str(profile_script)])
    except Exception as e:
        print(f"Failed to update profile script for RMW env: {e}")


def ensure_shared_python_venv(services, venv_path: Path = Path("/opt/psyched_venv")):
    """Create a shared virtualenv for python service requirements and install per-service pip packages.

    The venv is owned by root but accessible to `pete` (group or permissions as needed). It's recommended that services and systemd units use the venv python.
    """
    print(f"Ensuring shared python virtualenv at {venv_path}")
    run_sudo(["apt", "update"])
    # Ensure venv tooling
    run_sudo(["apt", "install", "-y", "python3-venv", "python3-pip"]) 

    if not venv_path.exists():
        run_sudo(["mkdir", "-p", str(venv_path)])
        run_sudo(["python3", "-m", "venv", str(venv_path)])
        run_sudo(["chown", "-R", "root:root", str(venv_path)])
        run_sudo(["chmod", "0755", str(venv_path)])

    # Install service related pip packages into the venv
    # Base packages that may be needed by services
    pkgs = set()
    if "mic" in services:
        pkgs.update(["sounddevice", "webrtcvad"]) 
    if "gps" in services:
        pkgs.update(["pynmea2", "pyserial"]) 

    if pkgs:
        pip = str(venv_path / "bin" / "pip")
        cmd = ["sudo", pip, "install"] + list(pkgs)
        try:
            run(cmd)
        except Exception as e:
            print(f"Failed to install pip packages into venv: {e}")

    # Ensure pete can use the venv; create a small wrapper script for services to source
    envshim = Path("/etc/profile.d/psyched_venv.sh")
    content = f"""# psyched shared Python virtualenv
VIRTUAL_ENV={venv_path}
PATH=$VIRTUAL_ENV/bin:$PATH
export VIRTUAL_ENV PATH
"""
    try:
        p = subprocess.Popen(["sudo", "tee", str(envshim)], stdin=subprocess.PIPE)
        p.communicate(input=content.encode())
        run_sudo(["chmod", "0644", str(envshim)])
    except Exception as e:
        print(f"Failed to write venv shim: {e}")


def clone_or_update(url: str, dest: Path, branch: str = ""):
    # Normalize GitHub HTTPS URLs to SSH to prefer SSH auth
    def normalize_git_url(u: str) -> str:
        # Convert https://github.com/owner/repo(.git) -> git@github.com:owner/repo(.git)
        if u and u.startswith("https://github.com/"):
            rest = u[len("https://github.com/"):]
            # remove leading slash if present
            if rest.startswith("/"):
                rest = rest[1:]
            return f"git@github.com:{rest}"
        return u

    url = normalize_git_url(url)
    if (dest / ".git").exists():
        print(f"Updating existing repo at {dest}")
        # Be tolerant of fetch/pull failures (network/auth); continue provisioning
        try:
            run(["sudo", "-u", "pete", "git", "-C", str(dest), "fetch", "--all", "--prune"], check=False)
        except Exception as e:
            print(f"Warning: git fetch failed for {dest}: {e}")
        if branch:
            try:
                run(["sudo", "-u", "pete", "git", "-C", str(dest), "checkout", branch], check=False)
            except Exception:
                try:
                    run(["sudo", "-u", "pete", "git", "-C", str(dest), "checkout", "-B", branch], check=False)
                except Exception as e:
                    print(f"Warning: failed to create/checkout branch {branch} in {dest}: {e}")
            try:
                run(["sudo", "-u", "pete", "git", "-C", str(dest), "pull", "--ff-only"], check=False)
            except Exception as e:
                print(f"Warning: git pull failed for {dest}: {e}")
        else:
            try:
                run(["sudo", "-u", "pete", "git", "-C", str(dest), "pull", "--ff-only"], check=False)
            except Exception as e:
                print(f"Warning: git pull failed for {dest}: {e}")
    else:
        print(f"Cloning {url} into {dest}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["sudo", "-u", "pete", "git", "clone"]
        if branch:
            cmd += ["--branch", branch]
        cmd += [url, str(dest)]
        try:
            run(cmd)
        except subprocess.CalledProcessError as e:
            # If the clone failed, attempt an HTTPS fallback for GitHub SSH URLs
            try:
                if isinstance(url, str) and url.startswith("git@github.com:"):
                    https_url = "https://github.com/" + url.split(":", 1)[1]
                    print(f"SSH clone failed; retrying with HTTPS {https_url}")
                    cmd2 = ["sudo", "-u", "pete", "git", "clone"]
                    if branch:
                        cmd2 += ["--branch", branch]
                    cmd2 += [https_url, str(dest)]
                    run(cmd2)
                else:
                    # Last-resort: try cloning with check=False so provisioning continues
                    print(f"Clone failed for {url} into {dest}: {e}; continuing provisioning")
            except subprocess.CalledProcessError as e2:
                print(f"Fallback HTTPS clone also failed for {url}: {e2}; continuing provisioning")
            except Exception as e2:
                print(f"Unexpected error during clone fallback for {url}: {e2}; continuing provisioning")


def copy_local_repo(src: Path, dest: Path):
    print(f"Copying local repo {src} to canonical location {dest}")
    # Ensure dest parent exists and then copy (overwrite safe)
    run_sudo(["mkdir", "-p", str(dest.parent)])
    # Use rsync-like copy via sudo tar to preserve perms as pete will own later
    run(["sudo", "rm", "-rf", str(dest)], check=False)
    run(["sudo", "cp", "-a", str(src), str(dest)])
    run_sudo(["chown", "-R", "pete:root", str(dest)])


def link_into_workspace(canonical: Path, linkpath: Path):
    # Ensure the parent directory for the link exists (e.g. /opt/psyched_workspace/src)
    parent = linkpath.parent
    if not parent.exists():
        print(f"Parent directory {parent} does not exist; creating")
        try:
            run_sudo(["mkdir", "-p", str(parent)])
            run_sudo(["chown", "-R", "pete:root", str(parent)])
            run_sudo(["chmod", "0775", str(parent)])
        except Exception as e:
            print(f"Failed to create parent directory {parent}: {e}")
            raise

    if linkpath.exists() and not linkpath.is_symlink():
        print(f"{linkpath} exists and is not a symlink; leaving in place")
        return

    if linkpath.is_symlink():
        current = linkpath.resolve()
        if current != canonical.resolve():
            print(f"Updating symlink {linkpath} -> {canonical}")
            run_sudo(["rm", "-f", str(linkpath)])
            run_sudo(["ln", "-s", str(canonical), str(linkpath)])
        else:
            print(f"Symlink {linkpath} already points to {canonical}; skipping")
    else:
        print(f"Creating symlink {linkpath} -> {canonical}")
        run_sudo(["ln", "-s", str(canonical), str(linkpath)])


def build_workspace(repo_root: Path, workspace_dir: Path, ros_distro: str = "kilted"):
    src_dir = workspace_dir / "src"
    marker = workspace_dir / ".built"

    print("Setting up workspace build dependencies and environment")
    # install rosdep and colcon tooling
    run_sudo(["apt", "update"])
    run_sudo(["apt", "install", "-y", "python3-rosdep", "python3-colcon-common-extensions"])

    # init rosdep if necessary
    if not Path("/etc/ros/rosdep/sources.list.d/20-default.list").exists():
        run_sudo(["rosdep", "init"])  # may already be present
    run(["rosdep", "update"], check=False)

    # If no sources, nothing to do
    if not any(src_dir.iterdir()):
        print(f"No source packages in {src_dir}; nothing to build")
        return

    # Run rosdep install for workspace
    rosdep_cmd = f"rosdep install --from-paths {src_dir} --ignore-src -y || true"
    run_as_pete(rosdep_cmd)

    # Build workspace as pete, sourcing the ROS distro if available
    build_cmd = (
        f"if [ -f /opt/ros/{ros_distro}/setup.bash ]; then . /opt/ros/{ros_distro}/setup.bash; fi && "
        f"colcon build --install-base '{workspace_dir}/install' --merge-install || true"
    )
    run_as_pete(build_cmd)

    # Ensure global workspace is sourced for all users
    profile_script = Path("/etc/profile.d/psyched_workspace.sh")
    content = """#!/bin/sh
if [ -f /opt/psyched_workspace/install/setup.bash ]; then
    . /opt/psyched_workspace/install/setup.bash
fi
"""
    try:
        p = subprocess.Popen(["sudo", "tee", str(profile_script)], stdin=subprocess.PIPE)
        p.communicate(input=content.encode())
        run_sudo(["chmod", "0644", str(profile_script)])
    except Exception as e:
        print(f"Failed to write profile script: {e}")
    run_sudo(["chmod", "0644", str(profile_script)])

    run_sudo(["touch", str(marker)])
    run_sudo(["chown", "root:root", str(marker)])
    print("Workspace build complete")


def install_cli(repo_root: Path):
    """Install the /usr/bin/psyched wrapper that invokes the packaged CLI module."""
    print("Installing /usr/bin/psyched CLI wrapper")
    wrapper = """#!/usr/bin/env bash
# Auto-generated psyched wrapper (uses shared venv if present)
VENV=/opt/psyched_venv
if [ -x "$VENV/bin/python3" ]; then
    PYTHON_EXEC="$VENV/bin/python3"
else
    PYTHON_EXEC=python3
fi
export PYTHONPATH=/opt/psyched:$PYTHONPATH
exec "$PYTHON_EXEC" -m tools.provision.psyched_cli "$@"
"""
    try:
        p = subprocess.Popen(["sudo", "tee", "/usr/bin/psyched"], stdin=subprocess.PIPE)
        p.communicate(input=wrapper.encode())
        run_sudo(["chmod", "+x", "/usr/bin/psyched"])
    except Exception as e:
        print(f"Failed to install psyched CLI wrapper: {e}")


def install_systemd_units(repo_root: Path, services, workspace_dir: Path, clone_dir: Path, ros_distro: str = "kilted"):
    """Create, enable, and start systemd units for any service that provides a launch_wrapper.

    This is best-effort: units are named `psyched-<wrapper>.service` and run as `pete`.
    """
    services_dir = repo_root / "tools" / "provision" / "services"
    units_created = []
    for svc in services:
        svc_file = services_dir / f"{svc}.json"
        if not svc_file.exists():
            continue
        try:
            with svc_file.open() as sf:
                sdata = json.load(sf)
        except Exception as e:
            print(f"Failed to load service file {svc_file}: {e}")
            continue

        for lw in sdata.get("launch_wrappers", []):
            # lw is expected to be a dict with relpath for local wrappers
            rel = None
            if isinstance(lw, dict):
                rel = lw.get("relpath")
            else:
                rel = lw
            if not rel:
                continue

            pkg_name = rel
            # Determine a launch file name in the canonical clone
            launch_dir = clone_dir / pkg_name / "launch"
            launch_file = None
            if launch_dir.exists() and launch_dir.is_dir():
                for p in launch_dir.iterdir():
                    if p.suffix == ".py":
                        launch_file = p.stem
                        break

            # fallback: try to use a common naming
            if not launch_file:
                launch_file = f"{pkg_name}_launch"

            unit_name = f"psyched-{pkg_name}.service"
            unit_path = Path("/etc/systemd/system") / unit_name

            unit_content = f"""[Unit]
Description=psyched {pkg_name} launch
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pete
Group=root
ExecStart=/bin/bash -lc 'source /opt/psyched_workspace/install/setup.bash 2>/dev/null || true; source /opt/psyched_venv/bin/activate 2>/dev/null || true; exec ros2 launch {pkg_name} {launch_file}'
Restart=always
RestartSec=5
LimitNOFILE=65536

[Install]
WantedBy=multi-user.target
"""

            try:
                print(f"Writing systemd unit {unit_path}")
                p = subprocess.Popen(["sudo", "tee", str(unit_path)], stdin=subprocess.PIPE)
                p.communicate(input=unit_content.encode())
                run_sudo(["chmod", "0644", str(unit_path)])
                # reload systemd, enable and start the unit
                run_sudo(["systemctl", "daemon-reload"]) 
                run_sudo(["systemctl", "enable", unit_name],)
                run_sudo(["systemctl", "restart", unit_name],)
                units_created.append(unit_name)
            except Exception as e:
                print(f"Failed to create/start systemd unit for {pkg_name}: {e}")

    if units_created:
        print("Systemd units created:", units_created)


def main():
    if len(sys.argv) != 2:
        print("Usage: host_provision.py /path/to/host.json")
        sys.exit(2)
    host_json = Path(sys.argv[1])
    if not host_json.exists():
        print(f"Host config not found: {host_json}")
        sys.exit(2)

    repo_root = host_json.parent.parent
    with host_json.open() as f:
        cfg = json.load(f)

    host = cfg.get("host")
    print(f"Provisioning host (json): {host}")

    # Optionally install ros2 via the existing shell installer
    if cfg.get("install_ros2"):
        print("Delegating ROS2 install to setup_ros2.sh install-ros2")
        setup = repo_root / "tools" / "provision" / "setup_ros2.sh"
        if setup.exists():
            if os.access(str(setup), os.X_OK):
                run([str(setup), "install-ros2"], check=False)
            else:
                run(["/bin/bash", str(setup), "install-ros2"], check=False)
        else:
            print("Warning: setup_ros2.sh not found; skipping ROS2 installation")

    WORKSPACE_DIR = Path(os.environ.get("WORKSPACE_DIR", "/opt/psyched_workspace"))
    CLONE_DIR = Path(os.environ.get("CLONE_DIR", "/opt/psyched"))

    ensure_pete()
    ensure_dirs(WORKSPACE_DIR, CLONE_DIR)
    # Try to copy the invoking user's SSH public key(s) into pete's authorized_keys so SSH access works
    try:
        copy_current_user_ssh_to_pete()
    except Exception as e:
        print(f"Failed to copy SSH keys to pete: {e}")

    # Expand services into repo lists by loading service definitions
    repos = []
    services = cfg.get("services", [])
    services_dir = repo_root / "tools" / "provision" / "services"
    for svc in services:
        svc_file = services_dir / f"{svc}.json"
        if not svc_file.exists():
            print(f"Warning: service definition not found: {svc_file}")
            continue
        with svc_file.open() as sf:
            sdata = json.load(sf)
            srepos = sdata.get("repos", [])
            repos.extend(srepos)
            # also collect any local launch wrapper packages
            slaunch = sdata.get("launch_wrappers", [])
            for lw in slaunch:
                # treat launch wrappers like local repos for copying
                if lw.get("local"):
                    repos.append({"local": True, "path": lw.get("path"), "relpath": lw.get("relpath")})

    # Allow a host to also list explicit repos (backwards-compat)
    repos.extend(cfg.get("repos", []))

    for r in repos:
        # support service entries that are local code copies instead of remote git
        if r.get("local"):
            src_path = repo_root / r.get("path")
            relpath = r.get("relpath")
            dest = CLONE_DIR / relpath
            linkpath = WORKSPACE_DIR / "src" / relpath
            if not src_path.exists():
                print(f"Warning: local service code not found: {src_path}")
                continue
            copy_local_repo(src_path, dest)
            link_into_workspace(dest, linkpath)
            continue

        url = r.get("url")
        relpath = r.get("relpath")
        branch = r.get("branch", "")
        dest = CLONE_DIR / relpath
        linkpath = WORKSPACE_DIR / "src" / relpath
        clone_or_update(url, dest, branch)
        link_into_workspace(dest, linkpath)

    # Ensure shared python virtualenv for service python requirements
    ensure_shared_python_venv(services)

    # Ensure Cyclone DDS if any services require DDS and persist RMW selection
    ensure_cyclone_dds_if_needed(services, ros_distro=cfg.get("ros_distro", "kilted"))

    build_workspace(repo_root, WORKSPACE_DIR, ros_distro=cfg.get("ros_distro", "kilted"))

    # Install helper CLI wrapper to /usr/bin/psyched for easy reprovision/update/deprovision
    try:
        install_cli(repo_root)
    except Exception as e:
        print(f"Failed to install psyched CLI wrapper: {e}")

    # Install systemd units for any launch_wrappers provided by services
    try:
        install_systemd_units(repo_root, services, WORKSPACE_DIR, CLONE_DIR, ros_distro=cfg.get("ros_distro", "kilted"))
    except Exception as e:
        print(f"Failed to install systemd units: {e}")

    # If gps service present, perform device setup
    if "gps" in services:
        setup_gps_device()

    # Ensure Python GPS deps are installed for the python bridge
    if "gps" in services:
        print("Installing Python GPS dependencies: pynmea2, pyserial")
        run_sudo(["pip3", "install", "pynmea2", "pyserial"])    

    # Ensure Python audio deps for mic service
    if "mic" in services:
        print("Installing system and Python audio dependencies for mic service")
        # libportaudio for sounddevice
        run_sudo(["apt", "update"])
        run_sudo(["apt", "install", "-y", "libportaudio2", "portaudio19-dev", "python3-pip"])
        # pip packages: sounddevice, webrtcvad
        run_sudo(["pip3", "install", "sounddevice", "webrtcvad"]) 

    # Ensure TTS/playback deps for voice service
    if "voice" in services:
        print("Running voice tool setup hooks (voice service present)")
        # Basic playback tools
        run_sudo(["apt", "update"])
        run_sudo(["apt", "install", "-y", "alsa-utils"])
        # Call tool-specific setup hooks so tools can manage their own installs
        if espeak_tool is not None:
            try:
                espeak_tool.setup()
            except Exception as e:
                print(f"espeak_tool.setup failed: {e}")
        else:
            # Try system install as fallback
            run_sudo(["apt", "install", "-y", "espeak-ng", "espeak"],)

        if mbrola_tool is not None:
            try:
                mbrola_tool.setup()
            except Exception as e:
                print(f"mbrola_tool.setup failed: {e}")

        if piper_tool is not None:
            try:
                piper_tool.setup()
            except Exception as e:
                print(f"piper_tool.setup failed: {e}")
        else:
            print("piper_tool not available; piper may need manual installation")

    # If debug_log present, ensure pete can read the system journal
    if "debug_log" in services:
        print("Configuring journal access for 'pete' (debug_log present)")
        # Prefer systemd-journal group; fallback to adm
        try:
            run_sudo(["getent", "group", "systemd-journal"], check=False)
            run_sudo(["usermod", "-aG", "systemd-journal", "pete"], check=False)
        except Exception:
            try:
                run_sudo(["usermod", "-aG", "adm", "pete"], check=False)
            except Exception as e:
                print(f"Failed to add pete to journal group: {e}")


def setup_gps_device():
    """Perform basic ublox/gpsd setup: install gpsd, enable service, add udev rule for u-blox8/7 devices.
    This is minimal and intended to get the device visible at /dev/ttyACM0 or similar and publish via gpsd.
    """
    print("Configuring gpsd and udev rules for ublox device (best-effort)")
    # Install gpsd and dependencies
    run_sudo(["apt", "update"])
    run_sudo(["apt", "install", "-y", "gpsd", "gpsd-clients", "python3-gpsdclient", "python3-serial"])    

    # Create a udev rule for common u-blox vendor/device ids (this is generic and may need tuning)
    udev_rule = (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", MODE="0666", GROUP="dialout"'
    )
    rule_path = Path("/etc/udev/rules.d/99-ublox.rules")
    p = subprocess.Popen(["sudo", "tee", str(rule_path)], stdin=subprocess.PIPE)
    p.communicate(input=(udev_rule + "\n").encode())
    run_sudo(["udevadm", "control", "--reload-rules"])    

    # Enable and restart gpsd
    run_sudo(["systemctl", "enable", "gpsd"])
    run_sudo(["systemctl", "restart", "gpsd"])

    print("gpsd and udev configured (you may need to adjust device path or udev ids for your hardware)")


if __name__ == "__main__":
    main()


def provision_from_json(host_json_path: str):
    """Programmatic entrypoint: run provisioning for given host json path."""
    sys.argv = [sys.argv[0], host_json_path]
    main()


def deprovision_from_json(host_json_path: str, purge_repos: bool = False):
    """Undo provisioning as best-effort: call teardown hooks, remove symlinks, and optionally remove cloned repos.

    This is intentionally conservative: it will attempt to remove symlinks in the workspace src and call tool teardowns.
    Purging repos (deleting from CLONE_DIR) is only performed if purge_repos is True.
    """
    host_json = Path(host_json_path)
    if not host_json.exists():
        print(f"Host config not found: {host_json}")
        return

    repo_root = host_json.parent.parent
    with host_json.open() as f:
        cfg = json.load(f)

    WORKSPACE_DIR = Path(os.environ.get("WORKSPACE_DIR", "/opt/psyched_workspace"))
    CLONE_DIR = Path(os.environ.get("CLONE_DIR", "/opt/psyched"))

    services = cfg.get("services", [])

    # Call tool teardown hooks if available
    # Auto-discover any tool modules under tools/provision/tools and call teardown() if provided
    tools_dir = repo_root / "tools" / "provision" / "tools"
    if tools_dir.exists():
        for f in tools_dir.glob("*_tool.py"):
            modname = f.stem
            try:
                # import via relative package name
                pkg = __import__(f"tools.provision.tools.{modname}", fromlist=[modname])
                if hasattr(pkg, "teardown"):
                    try:
                        print(f"Running teardown for tool {modname}")
                        pkg.teardown()
                    except Exception as e:
                        print(f"teardown for {modname} raised: {e}")
            except Exception as e:
                print(f"Failed to import tool module {modname}: {e}")

    # Remove symlinks from workspace src for services' repos
    services_dir = repo_root / "tools" / "provision" / "services"
    repos = []
    for svc in services:
        svc_file = services_dir / f"{svc}.json"
        if not svc_file.exists():
            continue
        with svc_file.open() as sf:
            sdata = json.load(sf)
            srepos = sdata.get("repos", [])
            repos.extend(srepos)
            slaunch = sdata.get("launch_wrappers", [])
            for lw in slaunch:
                # support both object and string forms
                if isinstance(lw, dict):
                    if lw.get("local"):
                        repos.append({"local": True, "path": lw.get("path"), "relpath": lw.get("relpath")})
                else:
                    # string wrapper name; assume local under services/launch_wrappers
                    repos.append({"local": True, "path": str(services_dir / "launch_wrappers" / lw), "relpath": lw})

    # Remove symlinks and optionally purge repos
    for r in repos:
        relpath = r.get("relpath")
        if not relpath:
            # derive a relpath from path for local entries if missing
            relpath = Path(r.get("path")).name
        linkpath = WORKSPACE_DIR / "src" / relpath
        canonical = CLONE_DIR / relpath
        if linkpath.is_symlink():
            print(f"Removing symlink {linkpath}")
            run_sudo(["rm", "-f", str(linkpath)])
        else:
            print(f"No symlink at {linkpath}; skipping")

        if purge_repos:
            if canonical.exists():
                print(f"Purging canonical repo {canonical}")
                run_sudo(["rm", "-rf", str(canonical)])

        # Optionally remove the installed CLI wrapper when purging
        if purge_repos:
            try:
                if Path('/usr/bin/psyched').exists():
                    print('Removing /usr/bin/psyched')
                    run_sudo(['rm', '-f', '/usr/bin/psyched'])
            except Exception as e:
                print(f'Failed to remove /usr/bin/psyched: {e}')

    print("Deprovision complete (best-effort)")

