import subprocess
import shutil
import sys

def setup():
    """Install espeak-ng via apt (best-effort)."""
    print("[espeak_tool] Installing espeak-ng (best-effort)")
    try:
        subprocess.run(["sudo", "apt", "update"], check=True)
        subprocess.run(["sudo", "apt", "install", "-y", "espeak-ng"], check=True)
    except subprocess.CalledProcessError:
        print("[espeak_tool] apt install failed; espeak-ng may not be available on this distro")


def teardown():
    print("[espeak_tool] Removing espeak-ng (best-effort)")
    try:
        subprocess.run(["sudo", "apt", "purge", "-y", "espeak-ng"], check=False)
    except Exception:
        pass
