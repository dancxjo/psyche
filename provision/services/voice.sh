#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

# voice.sh — Provision Piper TTS + ROS2 node and systemd launcher
# Features:
# - Subscribes to /voice/$HOSTNAME for text (std_msgs/String)
# - Queueing of utterances
# - Control via /voice/$HOSTNAME/cmd (std_msgs/String): interrupt|resume|abandon
#   and convenience topics /voice/$HOSTNAME/{interrupt,resume,abandon}
# - Uses piper to synthesize WAV and aplay for audio output

ROOT="/opt/psyched"
ETC_DIR="/etc/psyched"
VOICES_DIR="${ROOT}/voices"
PY_NODE_PATH="${ETC_DIR}/voice_node.py"
LAUNCH_PATH="${ETC_DIR}/voice.launch.sh"

ensure_deps() {
  # piper for TTS, alsa-utils for aplay playback
    export PSY_DEFER_APT=1
    common_apt_install piper alsa-utils
  # Try to get packaged voices if available (ignored if not found)
                common_apt_install ?piper-voices
}

ensure_voice_model() {
  local model_name="${1:-en_US-lessac-medium}"
  local model_basename="$model_name"
  local onnx="${VOICES_DIR}/${model_basename}.onnx"
  local json="${VOICES_DIR}/${model_basename}.onnx.json"

  sudo mkdir -p "${VOICES_DIR}"

  # If already present, nothing to do
  if [ -f "$onnx" ] && [ -f "$json" ]; then
    echo "[voice] Using existing model: $onnx"
    return 0
  fi

    # Try to link or copy from common system voice directories if available
    local sysdirs=(
        "/usr/share/piper-voices"
        "/usr/local/share/piper-voices"
        "/usr/share/tts/piper"
        "/usr/share/voices/piper"
    )
    for sysdir in "${sysdirs[@]}"; do
        if [ -d "$sysdir" ]; then
            local found_onnx
            found_onnx="$(find "$sysdir" -type f -name "${model_basename}.onnx" 2>/dev/null | head -n1 || true)"
            local found_json
            found_json="$(find "$sysdir" -type f -name "${model_basename}.onnx.json" 2>/dev/null | head -n1 || true)"
            if [ -n "$found_onnx" ] && [ -n "$found_json" ]; then
                echo "[voice] Linking model from $sysdir"
                sudo ln -sf "$found_onnx" "$onnx"
                sudo ln -sf "$found_json" "$json"
                return 0
            fi
        fi
    done

    # If a local tarball cache exists, extract it
    local cache_dirs=("${VOICES_DIR}" "/opt/psyched/cache/voices" "/opt/psyched/voices/cache")
    for cdir in "${cache_dirs[@]}"; do
        if [ -f "$cdir/${model_basename}.tar.gz" ]; then
            echo "[voice] Extracting cached tarball: $cdir/${model_basename}.tar.gz"
            sudo tar -xzf "$cdir/${model_basename}.tar.gz" -C "$VOICES_DIR" || true
            if [ -f "$onnx" ] && [ -f "$json" ]; then
                echo "[voice] Model extracted from local cache"
                return 0
            fi
        fi
    done

    # Construct possible mirrors/URLs
    # Prefer Hugging Face piper-voices main branch layout:
    #   https://huggingface.co/rhasspy/piper-voices/resolve/main/<lang>/<locale>/<voice>/<quality>/<basename>.*
    local lang_locale="${model_basename%%-*}"           # en_US
    local voice_rest="${model_basename#*-}"            # lessac-medium
    local voice_name="${voice_rest%%-*}"               # lessac
    local quality="${voice_rest#*-}"                   # medium
    local lang_prefix="${lang_locale%%_*}"             # en

    # Mirrors
    local HF_BASE="https://huggingface.co/rhasspy/piper-voices/resolve/main"
    local HF_URL_ONNX="${HF_BASE}/${lang_prefix}/${lang_locale}/${voice_name}/${quality}/${model_basename}.onnx"
    local HF_URL_JSON="${HF_BASE}/${lang_prefix}/${lang_locale}/${voice_name}/${quality}/${model_basename}.onnx.json"

    # Some distributions ship .tar.gz bundles in GitHub releases (piper-voices)
    local GH_VOICES_REL="https://github.com/rhasspy/piper-voices/releases/download/v1.0.0/${model_basename}.tar.gz"
    local GH_PIPER_REL_DIR="https://github.com/rhasspy/piper/releases/download"
    # Try a few known release tags that included direct files/tarballs (best-effort)
    local GH_PIPER_URLS=(
        "${GH_PIPER_REL_DIR}/v1.2.0/${model_basename}.tar.gz"
        "${GH_PIPER_REL_DIR}/2023.11.14/${model_basename}.tar.gz"
    )

    echo "[voice] Attempting to obtain voice model ${model_basename} (multiple mirrors)"

    # Helper: fetch function with curl/wget and retries
    _fetch() {
        local url="$1" out="$2"
        if command -v curl >/dev/null 2>&1; then
            sudo curl -fL --retry 3 --connect-timeout 15 -o "$out" "$url" 2>/dev/null || return 1
            return 0
        elif command -v wget >/dev/null 2>&1; then
            sudo wget -q -O "$out" "$url" || return 1
            return 0
        else
            return 1
        fi
    }

    # Try Hugging Face direct files
    if [ ! -f "$onnx" ] || [ ! -f "$json" ]; then
        echo "[voice] Trying Hugging Face (direct files)"
        _fetch "$HF_URL_ONNX" "$onnx" || true
        _fetch "$HF_URL_JSON" "$json" || true
    fi

    # If still missing, try tarballs from GitHub mirrors and extract
    if [ ! -f "$onnx" ] || [ ! -f "$json" ]; then
        tmp_tar="${VOICES_DIR}/${model_basename}.tar.gz"
        echo "[voice] Trying GitHub tarball(s)"
        _fetch "$GH_VOICES_REL" "$tmp_tar" || true
        if [ ! -s "$tmp_tar" ]; then
            for u in "${GH_PIPER_URLS[@]}"; do
                _fetch "$u" "$tmp_tar" && break || true
            done
        fi
        if [ -s "$tmp_tar" ]; then
            sudo tar -xzf "$tmp_tar" -C "$VOICES_DIR" || true
            sudo rm -f "$tmp_tar" || true
        fi
    fi

  if [ -f "$onnx" ] && [ -f "$json" ]; then
    echo "[voice] Model downloaded to $VOICES_DIR"
  else
        echo "[voice] WARNING: Could not obtain model files automatically." >&2
        echo "[voice] Hints:" >&2
        echo "[voice]  - If apt is available: sudo apt-get install piper-voices (then re-run)" >&2
        echo "[voice]  - Offline: place ${model_basename}.tar.gz in one of: ${cache_dirs[*]} and re-run" >&2
        echo "[voice]  - Manual: download from Hugging Face and copy to: $onnx and $json" >&2
  fi
}

install_node() {
    sudo mkdir -p "$ETC_DIR"
    # Prefer installing Python node from repository if available
    if [ -f "${ROOT}/provision/services/voice_node.py" ]; then
        sudo install -m 0755 "${ROOT}/provision/services/voice_node.py" "$PY_NODE_PATH"
    else
        # Fallback: embed Python ROS2 node into /etc/psyched/voice_node.py
        sudo tee "$PY_NODE_PATH" >/dev/null <<'PY'
#!/usr/bin/env python3
import os
import signal
import socket
import subprocess
import tempfile
import threading
import queue
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class PiperVoiceNode(Node):
    def __init__(self):
        super().__init__('piper_voice')
        # Determine host segment for topic
        host = os.environ.get('PSY_HOST', '') or socket.gethostname().split('.')[0]
        self.base_topic = f'/voice/{host}'

        # Config
        self.model_path = os.environ.get('PSY_VOICE_MODEL', '/opt/psyched/voices/en_US-lessac-medium.onnx')

        # Queue and player state
        self.queue: 'queue.Queue[str]' = queue.Queue()
        self.current_proc: Optional[subprocess.Popen] = None
        self.current_wav: Optional[str] = None
        self.paused = False
        self.abandon_flag = False
        self.lock = threading.RLock()

        # Subscribers
        self.create_subscription(String, self.base_topic, self.text_cb, 10)
        self.create_subscription(String, f'{self.base_topic}/cmd', self.cmd_cb, 10)
        # Convenience: accept empty messages on individual topics
        self.create_subscription(Empty, f'{self.base_topic}/interrupt', lambda _: self._interrupt(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/resume', lambda _: self._resume(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/abandon', lambda _: self._abandon(), 10)

        # Player thread
        self.player_thread = threading.Thread(target=self._player_loop, daemon=True)
        self.player_thread.start()

        self.get_logger().info(f'PiperVoiceNode ready. Text: {self.base_topic} | Cmd: {self.base_topic}/cmd')

    # Callbacks
    def text_cb(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'Queue: {text[:64]}...')
        self.queue.put(text)

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('interrupt', 'pause', 'stop'):
            self._interrupt()
        elif cmd in ('resume', 'continue'):
            self._resume()
        elif cmd in ('abandon', 'cancel', 'flush'):
            self._abandon()
        else:
            self.get_logger().warn(f'Unknown cmd: {cmd}')

    # Control actions
    def _interrupt(self):
        with self.lock:
            if self.current_proc and not self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGSTOP)
                    self.paused = True
                    self.get_logger().info('Paused (interrupt).')
                except Exception as e:
                    self.get_logger().error(f'Interrupt failed: {e}')

    def _resume(self):
        with self.lock:
            if self.current_proc and self.paused:
                try:
                    os.kill(self.current_proc.pid, signal.SIGCONT)
                    self.paused = False
                    self.get_logger().info('Resumed.')
                except Exception as e:
                    self.get_logger().error(f'Resume failed: {e}')

    def _abandon(self):
        with self.lock:
            self.abandon_flag = True
            # Clear queue
            try:
                while True:
                    self.queue.get_nowait()
            except queue.Empty:
                pass
            # Terminate current playback
            if self.current_proc:
                try:
                    self.current_proc.terminate()
                    self.get_logger().info('Abandon: terminated current playback and flushed queue.')
                except Exception as e:
                    self.get_logger().error(f'Abandon terminate failed: {e}')

    # Worker
    def _player_loop(self):
        while rclpy.ok():
            try:
                text = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                continue
            self._speak_text(text)

    def _speak_text(self, text: str):
        # Synthesize to temp wav using piper
        wav_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                wav_path = tf.name
            synth = subprocess.run(
                ['piper', '-m', self.model_path, '-o', wav_path],
                input=text.encode('utf-8'),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if synth.returncode != 0 or not os.path.exists(wav_path):
                self.get_logger().error(f'piper synth failed rc={synth.returncode}: {synth.stderr.decode(errors="ignore")[:200]}')
                return

            # Play via aplay
            with self.lock:
                self.current_wav = wav_path
                self.current_proc = subprocess.Popen(['aplay', '-q', wav_path])
                self.paused = False
                self.abandon_flag = False

            # Monitor playback
            while True:
                with self.lock:
                    proc = self.current_proc
                    abandon = self.abandon_flag
                if proc is None:
                    break
                ret = proc.poll()
                if ret is not None:
                    break
                if abandon:
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

        except Exception as e:
            self.get_logger().error(f'Playback error: {e}')
        finally:
            with self.lock:
                if self.current_proc:
                    try:
                        self.current_proc.wait(timeout=0.2)
                    except Exception:
                        try:
                            self.current_proc.kill()
                        except Exception:
                            pass
                self.current_proc = None
                self.paused = False
                self.abandon_flag = False
                cw = self.current_wav
                self.current_wav = None
            if wav_path and os.path.exists(wav_path):
                try:
                    os.unlink(wav_path)
                except Exception:
                    pass


def main():
    rclpy.init()
    node = PiperVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
        sudo chmod +x "$PY_NODE_PATH"
    fi

  # Install launch wrapper
  sudo tee "$LAUNCH_PATH" >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u

# Preferred model can be overridden via /etc/default/psyched-voice
if [ -f /etc/default/psyched-voice ]; then
  source /etc/default/psyched-voice
fi

export PSY_VOICE_MODEL="${PSY_VOICE_MODEL:-/opt/psyched/voices/en_US-lessac-medium.onnx}"
exec python3 /etc/psyched/voice_node.py
LAUNCH
  sudo chmod +x "$LAUNCH_PATH"
}

provision() {
  ensure_deps
  ensure_voice_model "${PSY_VOICE_MODEL_NAME:-en_US-lessac-medium}"
  install_node
  echo "[voice] provisioned. Use: sudo systemctl start psyched@voice.service"
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
