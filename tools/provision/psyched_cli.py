#!/usr/bin/env python3
"""psyched command-line helper: reprovision, update, deprovision

Usage: psyched reprovision | update | deprovision [--purge]
"""
import argparse
import subprocess
import sys
import os
from pathlib import Path


def run_cmd(cmd, check=True):
    print(
        "+ ", " ".join(cmd)
    )
    return subprocess.run(cmd, check=check)


def reprovision(host_json: str):
    # Call the host_provisioner script programmatically
    from tools.provision.host_provision import provision_from_json
    provision_from_json(host_json)


def update_and_reprovision(host_json: str):
    # Perform git pulls / local copies, then reprovision
    repo_root = Path(host_json).parent.parent
    # Pull top-level repository as current user
    try:
        run_cmd(["git", "pull"], check=False)
    except Exception:
        pass

    # Update canonical clones under /opt/psyched as pete (if they exist)
    clone_dir = Path(os.environ.get("CLONE_DIR", "/opt/psyched"))
    if clone_dir.exists():
        for p in clone_dir.rglob(".git"):
            repo_dir = p.parent
            try:
                run_cmd(["sudo", "-u", "pete", "git", "-C", str(repo_dir), "pull", "--ff-only"], check=False)
            except Exception:
                pass

    # Lastly, pull any service-local git repos under tools/provision/services
    services_dir = repo_root / "tools" / "provision" / "services"
    if services_dir.exists():
        for p in services_dir.rglob(".git"):
            repo_dir = p.parent
            try:
                run_cmd(["sudo", "-u", "pete", "git", "-C", str(repo_dir), "pull", "--ff-only"], check=False)
            except Exception:
                pass

    reprovision(host_json)


def deprovision(host_json: str, purge: bool = False):
    from tools.provision.host_provision import deprovision_from_json
    deprovision_from_json(host_json, purge_repos=purge)


def main(argv=None):
    p = argparse.ArgumentParser(prog="psyched")
    p.add_argument("command", choices=["reprovision", "update", "deprovision"])
    p.add_argument("host_json", nargs="?", default="hosts/ear.json")
    p.add_argument("--purge", action="store_true", help="When deprovisioning, also delete cloned repos")
    args = p.parse_args(argv)

    host_json = args.host_json
    if args.command == "reprovision":
        reprovision(host_json)
    elif args.command == "update":
        update_and_reprovision(host_json)
    elif args.command == "deprovision":
        deprovision(host_json, purge=args.purge)


if __name__ == "__main__":
    main()
