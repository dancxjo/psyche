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
import subprocess
import sys
from pathlib import Path


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


def clone_or_update(url: str, dest: Path, branch: str = ""):
    if (dest / ".git").exists():
        print(f"Updating existing repo at {dest}")
        run(["sudo", "-u", "pete", "git", "-C", str(dest), "fetch", "--all", "--prune"])
        if branch:
            try:
                run(["sudo", "-u", "pete", "git", "-C", str(dest), "checkout", branch])
            except subprocess.CalledProcessError:
                run(["sudo", "-u", "pete", "git", "-C", str(dest), "checkout", "-B", branch])
            run(["sudo", "-u", "pete", "git", "-C", str(dest), "pull", "--ff-only"], check=False)
        else:
            run(["sudo", "-u", "pete", "git", "-C", str(dest), "pull", "--ff-only"], check=False)
    else:
        print(f"Cloning {url} into {dest}")
        dest.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["sudo", "-u", "pete", "git", "clone"]
        if branch:
            cmd += ["--branch", branch]
        cmd += [url, str(dest)]
        run(cmd)


def link_into_workspace(canonical: Path, linkpath: Path):
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
    profile_script = Path("/etc/profile.d/psyche_workspace.sh")
    content = """#!/bin/sh
if [ -f /opt/psyche_workspace/install/setup.bash ]; then
  . /opt/psyche_workspace/install/setup.bash
fi
"""
    run_sudo(["tee", str(profile_script)],)
    # write via sudo tee
    p = subprocess.Popen(["sudo", "tee", str(profile_script)], stdin=subprocess.PIPE)
    p.communicate(input=content.encode())
    run_sudo(["chmod", "0644", str(profile_script)])

    run_sudo(["touch", str(marker)])
    run_sudo(["chown", "root:root", str(marker)])
    print("Workspace build complete")


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

    WORKSPACE_DIR = Path(os.environ.get("WORKSPACE_DIR", "/opt/psyche_workspace"))
    CLONE_DIR = Path(os.environ.get("CLONE_DIR", "/opt/psyched"))

    ensure_pete()
    ensure_dirs(WORKSPACE_DIR, CLONE_DIR)

    repos = cfg.get("repos", [])
    for r in repos:
        url = r.get("url")
        relpath = r.get("relpath")
        branch = r.get("branch", "")
        dest = CLONE_DIR / relpath
        linkpath = WORKSPACE_DIR / "src" / relpath
        clone_or_update(url, dest, branch)
        link_into_workspace(dest, linkpath)

    build_workspace(repo_root, WORKSPACE_DIR, ros_distro=cfg.get("ros_distro", "kilted"))


if __name__ == "__main__":
    main()
