import subprocess
import shutil

def setup():
    print("[piper_tool] Attempting to install piper (best-effort)")
    # Some distros may package piper; otherwise we leave to manual installation
    try:
        subprocess.run(["sudo", "apt", "update"], check=True)
        subprocess.run(["sudo", "apt", "install", "-y", "piper"], check=True)
    except subprocess.CalledProcessError:
        print("[piper_tool] piper not available via apt. Please install Piper manually or via your preferred method.")


def teardown():
    print("[piper_tool] Teardown not implemented (manual cleanup may be required)")
