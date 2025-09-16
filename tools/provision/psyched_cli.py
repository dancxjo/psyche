#!/usr/bin/env python3
"""psyched command-line helper: reprovision, update, deprovision

Usage: psyched reprovision | update | deprovision [--purge]
"""
import argparse
import subprocess
import sys
import os
import shutil
import tempfile
import getpass
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


def update_and_reprovision(host_json: str, enable_git: bool = False, git_method: str = "ssh"):
    # Perform git pulls / local copies, then reprovision
    repo_root = Path(host_json).parent.parent
    zip_url = "https://github.com/dancxjo/psyche/archive/refs/heads/main.zip"
    # If the user opted into git behavior, try to ensure this repo is a git repo and pull
    if enable_git:
        try:
            # If not a git repo, initialize and set origin
            if not (repo_root / ".git").exists():
                print(f"Initializing git repository in {repo_root}")
                run_cmd(["git", "-C", str(repo_root), "init"], check=True)
                # Set a sensible default user for the initial commit
                run_cmd([
                    "git", "-C", str(repo_root), "-c", "user.name=psyched", "-c", "user.email=psyched@example.com",
                    "add", "."
                ], check=False)
                run_cmd([
                    "git", "-C", str(repo_root), "-c", "user.name=psyched", "-c", "user.email=psyched@example.com",
                    "commit", "-m", "Initial import from zip"
                ], check=False)
                # Use HTTPS origin by default to avoid SSH key requirement
                run_cmd(["git", "-C", str(repo_root), "remote", "add", "origin", "https://github.com/dancxjo/psyche.git"], check=False)

            # Now attempt to pull from origin
            if git_method == "ssh":
                try:
                    run_cmd(["git", "-C", str(repo_root), "pull", "--ff-only", "origin", "main"], check=False)
                except Exception:
                    # Try pulling as SUDO_USER if available
                    sudo_user = os.environ.get("SUDO_USER")
                    if sudo_user:
                        try:
                            run_cmd(["sudo", "-u", sudo_user, "git", "-C", str(repo_root), "pull", "--ff-only", "origin", "main"], check=False)
                        except Exception:
                            print("Warning: git pull via SSH failed; continuing")
                    else:
                        print("Warning: git pull via SSH failed; continuing")
            else:
                token = os.environ.get("GITHUB_TOKEN")
                if token:
                    extra = f"Authorization: bearer {token}"
                    run_cmd(["git", "-C", str(repo_root), "-c", f"http.extraheader={extra}", "pull", "--ff-only", "origin", "main"], check=False)
                else:
                    print("Warning: GITHUB_TOKEN not set; cannot perform https git pull without credentials; continuing")
        except Exception as e:
            print(f"Warning: git-based update failed: {e}; falling back to zip update")

    # Default behavior (or fallback) — download zip and overwrite repo root
    tmpdir = Path(tempfile.mkdtemp(prefix="psyched_update_"))
    zippath = tmpdir / "psyche.zip"
    try:
        # Download
        if shutil.which("curl"):
            run_cmd(["curl", "-fSL", "-o", str(zippath), zip_url], check=True)
        elif shutil.which("wget"):
            run_cmd(["wget", "-q", "-O", str(zippath), zip_url], check=True)
        else:
            print("Warning: no curl or wget found; skipping zip update step and reprovisioning")
            return reprovision(host_json)

        # Extract
        if shutil.which("unzip"):
            run_cmd(["unzip", "-q", str(zippath), "-d", str(tmpdir)], check=True)
        elif shutil.which("bsdtar"):
            run_cmd(["bsdtar", "-xf", str(zippath), "-C", str(tmpdir)], check=True)
        else:
            print("Warning: no extractor found (unzip/bsdtar); skipping update and reprovisioning")
            return reprovision(host_json)

        topdirs = [p for p in tmpdir.iterdir() if p.is_dir()]
        if not topdirs:
            print("Warning: extracted archive did not contain expected contents; skipping update")
            return reprovision(host_json)
        topdir = topdirs[0]

        # Copy into repo_root
        if shutil.which("rsync"):
            run_cmd(["sudo", "rsync", "-a", "--delete", f"{topdir}/", f"{repo_root}/"], check=True)
        else:
            run_cmd(["sudo", "cp", "-a", f"{topdir}/.", str(repo_root)], check=True)

    except Exception as e:
        print(f"Warning: update failed: {e}; continuing to reprovision")
    finally:
        try:
            shutil.rmtree(str(tmpdir))
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
    # Opt-in git behavior for update
    p.add_argument("--git", action="store_true", help="Enable git-managed updates (opt-in).")
    p.add_argument("--git-method", choices=["ssh", "https"], default="ssh", help="Method for git pulls when --git is used.")
    p.add_argument("--purge", action="store_true", help="When deprovisioning, also delete cloned repos")
    args = p.parse_args(argv)

    host_json = args.host_json
    if args.command == "reprovision":
        reprovision(host_json)
    elif args.command == "update":
        update_and_reprovision(host_json, enable_git=args.git, git_method=args.git_method)
    elif args.command == "deprovision":
        deprovision(host_json, purge=args.purge)


if __name__ == "__main__":
    main()
