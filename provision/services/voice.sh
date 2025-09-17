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
  # Install Piper CLI engine (piper-tts) and ALSA playback tools
  export PSY_DEFER_APT=1
  common_apt_install piper-tts alsa-utils
  # Try to get packaged voices if available (ignored if not found)
  common_apt_install ?piper-voices
}

ensure_voice_model() {
        # Attempt to obtain a specific model (basename e.g. en_US-kyle-high)
        local model_name="${1:-en_US-kyle-high}"
  local model_basename="$model_name"
  local onnx="${VOICES_DIR}/${model_basename}.onnx"
  local json="${VOICES_DIR}/${model_basename}.onnx.json"
    local expected_sha="${PSY_VOICE_MODEL_SHA256:-}"  # Optional: expected SHA256 of the .onnx for integrity

  sudo mkdir -p "${VOICES_DIR}"

  # If already present, nothing to do
    if [ -f "$onnx" ] && [ -f "$json" ]; then
    echo "[voice] Using existing model: $onnx"
        if [ -n "$expected_sha" ] && command -v sha256sum >/dev/null 2>&1; then
            local have_sha
            have_sha="$(sha256sum "$onnx" | awk '{print $1}')"
            if [ "$have_sha" != "$expected_sha" ]; then
                echo "[voice] WARNING: Existing model SHA256 mismatch (have=$have_sha expected=$expected_sha); re-downloading" >&2
                sudo rm -f "$onnx" "$json" || true
            else
                return 0
            fi
        else
            return 0
        fi
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
        # Integrity verification if expected SHA given
        if [ -n "$expected_sha" ] && command -v sha256sum >/dev/null 2>&1; then
            have_sha="$(sha256sum "$onnx" | awk '{print $1}')"
            if [ "$have_sha" != "$expected_sha" ]; then
                echo "[voice] ERROR: Downloaded model SHA256 mismatch (have=$have_sha expected=$expected_sha)" >&2
                echo "[voice] Removing corrupt model files." >&2
                sudo rm -f "$onnx" "$json" || true
            else
                echo "[voice] Model downloaded & verified ($model_basename)"
            fi
        else
            echo "[voice] Model downloaded to $VOICES_DIR"
        fi
  else
        echo "[voice] WARNING: Could not obtain model files automatically." >&2
        echo "[voice] Hints:" >&2
        echo "[voice]  - If apt is available: sudo apt-get install piper-voices (then re-run)" >&2
        echo "[voice]  - Offline: place ${model_basename}.tar.gz in one of: ${cache_dirs[*]} and re-run" >&2
        echo "[voice]  - Manual: download from Hugging Face and copy to: $onnx and $json" >&2
    fi

    # Return success if files present
    if [ -f "$onnx" ] && [ -f "$json" ]; then
        return 0
    fi
    return 1
}

# Map friendly alias to ordered candidate list (colon separated)
voice_alias_candidates() {
    local alias="$1"
    case "$alias" in
        en_male_high|male_en|en_us_male)
            echo "en_US-kyle-high:en_US-ryan-high:en_GB-southern_english_male-medium:en_US-lessac-medium" ;;
        en_female_high|female_en)
            echo "en_US-amy-high:en_US-lessac-medium" ;;
        minimal|small)
            echo "en_US-lessac-medium:en_US-kyle-high" ;;
        *)
            # Unknown alias; treat as direct model name
            echo "$alias" ;;
    esac
}

# Select a voice model when PSY_VOICE_MODEL_NAME not explicitly set
select_voice_model() {
    if [ -n "${PSY_VOICE_MODEL_NAME:-}" ]; then
        echo "$PSY_VOICE_MODEL_NAME"
        return 0
    fi
    local alias_list="${PSY_VOICE_MODEL_ALIAS:-en_male_high}"  # allow comma-separated aliases
    local final_candidates=()
    IFS=',' read -r -a alias_arr <<<"$alias_list"
    for a in "${alias_arr[@]}"; do
        local cands
        cands="$(voice_alias_candidates "$a")"
        IFS=':' read -r -a tmpc <<<"$cands"
        for c in "${tmpc[@]}"; do
            # de-duplicate while preserving order
            local already=0
            for existing in "${final_candidates[@]}"; do
                [ "$existing" = "$c" ] && already=1 && break
            done
            [ $already -eq 0 ] && final_candidates+=("$c")
        done
    done
    # Fallback absolute default if list somehow empty
    [ ${#final_candidates[@]} -eq 0 ] && final_candidates+=("en_US-kyle-high")
    echo "${final_candidates[0]}"  # primary candidate returned; full list used later for fallback download attempts
    PSY_INTERNAL_CANDIDATES="${final_candidates[*]}"
}

attempt_candidates_download() {
    local chosen primary ok=1
    # If internal candidate list exported by select_voice_model we reuse; else build from chosen only
    local list
    if [ -n "${PSY_INTERNAL_CANDIDATES:-}" ]; then
        list="$PSY_INTERNAL_CANDIDATES"
    else
        list="$1"
    fi
    for cand in $list; do
        echo "[voice] Ensuring voice model candidate: $cand"
        if ensure_voice_model "$cand"; then
            echo "[voice] Selected model: $cand"
            export PSY_EFFECTIVE_MODEL_NAME="$cand"
            return 0
        fi
    done
    echo "[voice] ERROR: All candidate voice models failed to download or verify." >&2
    return 1
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

# Figure out which Piper binary to use (prefer CLI engine)
if [ -n "${PSY_PIPER_BIN:-}" ]; then
  if command -v "$PSY_PIPER_BIN" >/dev/null 2>&1; then
    resolved="$(command -v "$PSY_PIPER_BIN")"
    if [ -f "$resolved" ]; then
      if grep -q "gi.require_version('Gtk'" "$resolved" 2>/dev/null; then
        echo "[psyched-voice] Configured PSY_PIPER_BIN ($resolved) appears to be the GTK frontend; ignoring" >&2
        PSY_PIPER_BIN=""
      else
        PSY_PIPER_BIN="$resolved"
      fi
    else
      PSY_PIPER_BIN="$resolved"
    fi
  else
    echo "[psyched-voice] Configured PSY_PIPER_BIN '$PSY_PIPER_BIN' not found; attempting auto-detect" >&2
    PSY_PIPER_BIN=""
  fi
fi

if [ -z "${PSY_PIPER_BIN:-}" ]; then
  for candidate in piper-tts piper /usr/local/bin/piper; do
    [ -n "$candidate" ] || continue
    if command -v "$candidate" >/dev/null 2>&1; then
      resolved="$(command -v "$candidate")"
      if [ -f "$resolved" ]; then
        if grep -q "gi.require_version('Gtk'" "$resolved" 2>/dev/null; then
          continue
        fi
      fi
      PSY_PIPER_BIN="$resolved"
      break
    fi
  done
fi

if [ -z "${PSY_PIPER_BIN:-}" ]; then
  echo "[psyched-voice] ERROR: Could not locate a Piper CLI binary (piper-tts)." >&2
  echo "[psyched-voice] Install 'piper-tts' or make it available and set PSY_PIPER_BIN." >&2
  exit 1
fi

export PSY_PIPER_BIN
export PSY_VOICE_MODEL="${PSY_VOICE_MODEL:-/opt/psyched/voices/en_US-kyle-high.onnx}"
exec python3 /etc/psyched/voice_node.py
LAUNCH
  sudo chmod +x "$LAUNCH_PATH"
}

provision() {
    ensure_deps

    # Determine model selection
    local chosen
    chosen="$(select_voice_model)"
    export PSY_VOICE_MODEL_NAME="${PSY_EFFECTIVE_MODEL_NAME:-$chosen}"

    # Attempt to download primary/fallback candidates
    attempt_candidates_download "$PSY_VOICE_MODEL_NAME" || true
    # If a candidate succeeded, PSY_EFFECTIVE_MODEL_NAME set
    local effective="${PSY_EFFECTIVE_MODEL_NAME:-$PSY_VOICE_MODEL_NAME}"

    install_node

    # Auto-generate default config if missing
    if [ ! -f /etc/default/psyched-voice ]; then
        echo "[voice] Creating /etc/default/psyched-voice with selected model and fallbacks"
        local fallback_line=""
        if [ -n "${PSY_INTERNAL_CANDIDATES:-}" ]; then
            # remove primary from list and build fallback env var
            local fb=()
            for c in $PSY_INTERNAL_CANDIDATES; do
                [ "$c" = "$effective" ] && continue
                fb+=("/opt/psyched/voices/${c}.onnx")
            done
            if [ ${#fb[@]} -gt 0 ]; then
                fallback_line="PSY_VOICE_MODEL_FALLBACKS=$(IFS=:; echo "${fb[*]}")"
            fi
        fi
        sudo tee /etc/default/psyched-voice >/dev/null <<CFG
# Auto-generated by voice.sh provision on $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Primary Piper model
PSY_VOICE_MODEL=/opt/psyched/voices/${effective}.onnx
# Fallback models (first existing used)
${fallback_line}
# Example: verify integrity (uncomment and set correct hash)
# PSY_VOICE_MODEL_SHA256=<sha256sum>
CFG
    fi

    echo "[voice] provisioned. Use: sudo systemctl start psyched@voice.service"
    echo "[voice] Effective model: ${effective} (override with PSY_VOICE_MODEL_NAME or PSY_VOICE_MODEL; alias via PSY_VOICE_MODEL_ALIAS)"
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
