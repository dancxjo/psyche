import subprocess
import os

MBROLA_VOICE_URL = "https://github.com/numediart/MBROLA-voices/raw/master/en1/en1.zip"

def setup():
    """Install mbrola and the default English mbrola voice (best-effort).
    This downloads the voice archive and installs it under /usr/share/mbrola or /usr/local/share/mbrola.
    """
    print("[mbrola_tool] Installing mbrola and default voice (best-effort)")
    try:
        subprocess.run(["sudo", "apt", "update"], check=True)
        subprocess.run(["sudo", "apt", "install", "-y", "mbrola"], check=True)
    except subprocess.CalledProcessError:
        print("[mbrola_tool] apt install mbrola failed; continuing to attempt voice install only")

    tmp = "/tmp/mbrola_en.zip"
    try:
        subprocess.run(["wget", "-O", tmp, MBROLA_VOICE_URL], check=True)
        target = "/usr/share/mbrola"
        os.makedirs(target, exist_ok=True)
        subprocess.run(["sudo", "unzip", "-o", tmp, "-d", target], check=True)
        print("[mbrola_tool] mbrola voice installed to ", target)
    except Exception as e:
        print("[mbrola_tool] failed to download/install mbrola voice:", e)


def teardown():
    print("[mbrola_tool] Teardown not implemented (manual cleanup may be required)")
