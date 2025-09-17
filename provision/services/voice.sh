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

find_cli_piper() {
  local candidate resolved
  for candidate in "$@"; do
    [ -n "$candidate" ] || continue
    if resolved="$(command -v "$candidate" 2>/dev/null)"; then
      if [ -f "$resolved" ]; then
        if LC_ALL=C grep -a -m1 "gi.require_version('Gtk'" "$resolved" >/dev/null 2>&1; then
          continue
        fi
      fi
      echo "$resolved"
      return 0
    fi
  done
  return 1
}

ensure_deps() {
  # Install Piper CLI engine (piper-tts) and ALSA playback tools
  export PSY_DEFER_APT=1
  common_apt_install piper-tts alsa-utils
  # Try to get packaged voices if available (ignored if not found)
  common_apt_install ?piper-voices

  if ! find_cli_piper piper-tts piper /usr/local/bin/piper >/dev/null 2>&1; then
    install_piper_cli_fallback || echo "[voice] WARNING: Piper CLI fallback install failed; voice service may not start" >&2
  fi
}

install_piper_cli_fallback() {
  # Fetch a prebuilt Piper CLI release when apt packages are unavailable.
  local arch suffix tmp_dir tar_path version url dest dest_tmp bin_path final_dir wrapper

  arch="$(dpkg --print-architecture 2>/dev/null || uname -m || echo unknown)"
  case "$arch" in
    amd64|x86_64) suffix="x86_64" ;;
    arm64|aarch64) suffix="aarch64" ;;
    armhf|armv7l) suffix="armv7l" ;;
    *)
      echo "[voice] WARNING: Unsupported architecture '$arch' for Piper prebuilt binaries" >&2
      return 1
      ;;
  esac

  tmp_dir="$(mktemp -d 2>/dev/null || echo /tmp/piper.$$)"
  tar_path="${tmp_dir}/piper.tar.gz"
  local downloaded=0 asset version_clean arch_candidates=() candidate asset_url fetch_tool

  case "$suffix" in
    x86_64) arch_candidates=("x86_64" "amd64") ;;
    aarch64) arch_candidates=("aarch64" "arm64") ;;
    armv7l) arch_candidates=("armv7l" "armhf" "armv7") ;;
  esac

  if command -v curl >/dev/null 2>&1; then
    fetch_tool="curl"
  elif command -v wget >/dev/null 2>&1; then
    fetch_tool="wget"
  else
    echo "[voice] Neither curl nor wget available to download Piper binary" >&2
    rm -rf "$tmp_dir" || true
    return 1
  fi

  for version in "${PSY_PIPER_VERSION:-v1.3.0}" v1.2.0 2023.11.14 2023.06.05; do
    version_clean="${version#v}"
    for arch_tag in "${arch_candidates[@]}"; do
      for asset in \
        "piper_${version_clean}_linux_${arch_tag}.tar.gz" \
        "piper_linux_${arch_tag}.tar.gz" \
        "piper_${version_clean}_${arch_tag}.tar.gz" \
        "piper_${arch_tag}.tar.gz"; do
        candidate="${asset}"
        [[ -n "$candidate" ]] || continue
        asset_url="https://github.com/rhasspy/piper/releases/download/${version}/${candidate}"
        echo "[voice] Attempting Piper CLI download: ${asset_url}"
        if [ "$fetch_tool" = "curl" ]; then
          if curl -fL --retry 3 --connect-timeout 20 -o "$tar_path" "$asset_url"; then
            downloaded=1
            break 3
          fi
        else
          if wget -q -O "$tar_path" "$asset_url"; then
            downloaded=1
            break 3
          fi
        fi
      done
    done
  done

  if [ "$downloaded" -ne 1 ] || [ ! -s "$tar_path" ]; then
    echo "[voice] WARNING: Unable to download Piper CLI release" >&2
    rm -rf "$tmp_dir" || true
    return 1
  fi

  dest="/opt/psyched/piper-cli/${suffix}"
  dest_tmp="${dest}.tmp"
  sudo rm -rf "$dest_tmp" >/dev/null 2>&1 || true
  sudo mkdir -p "$dest_tmp"
  if ! sudo tar -xzf "$tar_path" -C "$dest_tmp"; then
    echo "[voice] WARNING: Failed to extract Piper CLI archive" >&2
    sudo rm -rf "$dest_tmp" || true
    rm -rf "$tmp_dir" || true
    return 1
  fi

  bin_path="$(sudo find "$dest_tmp" -maxdepth 4 -type f -name piper -print -quit 2>/dev/null)"
  if [ -z "$bin_path" ]; then
    echo "[voice] WARNING: Piper CLI binary not found after extraction" >&2
    sudo rm -rf "$dest_tmp" || true
    rm -rf "$tmp_dir" || true
    return 1
  fi

  sudo chmod +x "$bin_path" || true
  sudo rm -rf "$dest" >/dev/null 2>&1 || true
  sudo mv "$dest_tmp" "$dest"

  bin_path="$(sudo find "$dest" -maxdepth 2 -type f -name piper -print -quit 2>/dev/null)"
  if [ -z "$bin_path" ]; then
    echo "[voice] WARNING: Piper CLI binary missing after final move" >&2
    rm -rf "$tmp_dir" || true
    return 1
  fi

  final_dir="$(dirname "$bin_path")"
  wrapper="/usr/local/bin/piper-tts"
  sudo tee "$wrapper" >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
DIR="__PIPER_DIR__"
if [ -z "${ESPEAK_DATA:-}" ] && [ -d "${DIR}/espeak-ng-data" ]; then
  export ESPEAK_DATA="${DIR}/espeak-ng-data"
fi
if [ -d "${DIR}/lib" ]; then
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    export LD_LIBRARY_PATH="${DIR}/lib:${LD_LIBRARY_PATH}"
  else
    export LD_LIBRARY_PATH="${DIR}/lib"
  fi
fi
cd "${DIR}"
exec "${DIR}/piper" "$@"
EOF
  local escaped_dir
  escaped_dir="$(printf '%s' "$final_dir" | sed 's/[&/]/\\&/g')"
  sudo sed -i "s#__PIPER_DIR__#${escaped_dir}#g" "$wrapper"
  sudo chmod +x "$wrapper"
  sudo ln -sf "$wrapper" /usr/local/bin/piper

  rm -rf "$tmp_dir" || true
  echo "[voice] Installed Piper CLI fallback to $final_dir"
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
        # New: balanced default male ordering prioritizing widely distributed medium model
        en_male_default|male_en_default|male_default)
            # lessac-medium ships in many voice bundles; then kyle-high for higher quality; then ryan-high; then a UK male; final fallback repeats lessac
            echo "en_US-lessac-medium:en_US-kyle-high:en_US-ryan-high:en_GB-southern_english_male-medium" ;;
        en_male_high|male_en|en_us_male)
            echo "en_US-kyle-high:en_US-ryan-high:en_US-lessac-medium:en_GB-southern_english_male-medium" ;;
        en_female_high|female_en)
            echo "en_US-amy-high:en_US-lessac-medium" ;;
        minimal|small)
            echo "en_US-lessac-medium:en_US-kyle-high" ;;
        *)
            echo "$alias" ;;
    esac
}

# Select a voice model when PSY_VOICE_MODEL_NAME not explicitly set
select_voice_model() {
    # If explicit model provided, honor it directly
    if [ -n "${PSY_VOICE_MODEL_NAME:-}" ]; then
        PSY_SELECTED_MODEL="$PSY_VOICE_MODEL_NAME"
        PSY_INTERNAL_CANDIDATES="$PSY_VOICE_MODEL_NAME"
        return 0
    fi
    local alias_list="${PSY_VOICE_MODEL_ALIAS:-en_male_default}" # default alias changed
    local final_candidates=()
    IFS=',' read -r -a alias_arr <<<"$alias_list"
    for a in "${alias_arr[@]}"; do
        local cands
        cands="$(voice_alias_candidates "$a")"
        IFS=':' read -r -a tmpc <<<"$cands"
        for c in "${tmpc[@]}"; do
            local already=0
            for existing in "${final_candidates[@]}"; do
                [ "$existing" = "$c" ] && already=1 && break
            done
            [ $already -eq 0 ] && final_candidates+=("$c")
        done
    done
    [ ${#final_candidates[@]} -eq 0 ] && final_candidates+=("en_US-lessac-medium")
    PSY_SELECTED_MODEL="${final_candidates[0]}"
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
import shutil
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class PiperVoiceNode(Node):
    def __init__(self):
        super().__init__('piper_voice')
        host = os.environ.get('PSY_HOST', '') or socket.gethostname().split('.')[0]
        self.base_topic = f'/voice/{host}'

        # Model selection (primary + fallbacks)
        primary_model = os.environ.get('PSY_VOICE_MODEL', '/opt/psyched/voices/en_US-kyle-high.onnx')
        fallback_list = os.environ.get('PSY_VOICE_MODEL_FALLBACKS', '').strip()
        candidates = [primary_model]
        if fallback_list:
            candidates.extend([p for p in fallback_list.split(':') if p])
        resolved_model = None
        for cand in candidates:
            if os.path.isfile(cand):
                resolved_model = cand
                break
        if not resolved_model:
            self.get_logger().warn(
                f"No available voice model found among candidates: {candidates}. Using primary path; synthesis may fail." )
            resolved_model = primary_model
        self.model_path = resolved_model
        self.piper_bin = self._resolve_piper_bin()

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
        self.create_subscription(Empty, f'{self.base_topic}/interrupt', lambda _: self._interrupt(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/resume', lambda _: self._resume(), 10)
        self.create_subscription(Empty, f'{self.base_topic}/abandon', lambda _: self._abandon(), 10)

        # Player thread
        self.player_thread = threading.Thread(target=self._player_loop, daemon=True)
        self.player_thread.start()

        self.get_logger().info(
            f'PiperVoiceNode ready. Text: {self.base_topic} | Cmd: {self.base_topic}/cmd | Piper: {self.piper_bin}'
        )

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
        wav_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                wav_path = tf.name
            synth = subprocess.run(
                [self.piper_bin, '-m', self.model_path, '-o', wav_path],
                input=text.encode('utf-8'),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if synth.returncode != 0 or not os.path.exists(wav_path):
                self.get_logger().error(
                    f'piper synth failed rc={synth.returncode}: '
                    f'{synth.stderr.decode(errors="ignore")[:200]}'
                )
                return

            with self.lock:
                self.current_wav = wav_path
                self.current_proc = subprocess.Popen(['aplay', '-q', wav_path])
                self.paused = False
                self.abandon_flag = False

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

    def _resolve_piper_bin(self) -> str:
        configured = os.environ.get('PSY_PIPER_BIN', '').strip()
        candidates = []
        if configured:
            candidates.append(configured)
        candidates.extend(['piper-tts', 'piper'])
        examined = set()
        for candidate in candidates:
            if not candidate:
                continue
            resolved = shutil.which(candidate)
            if resolved is None and os.path.isabs(candidate) and os.access(candidate, os.X_OK):
                resolved = candidate
            if not resolved or resolved in examined:
                continue
            examined.add(resolved)
            try:
                with open(resolved, 'rb') as handle:
                    head = handle.read(4096)
                if b"gi.require_version('Gtk'" in head:
                    continue
            except OSError:
                pass
            return resolved
        raise RuntimeError("No Piper CLI binary found. Install 'piper-tts' or set PSY_PIPER_BIN.")


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

    # Build candidate list (no subshell capture)
    select_voice_model
    export PSY_VOICE_MODEL_NAME="${PSY_EFFECTIVE_MODEL_NAME:-$PSY_SELECTED_MODEL}"

    # Attempt downloads over all candidates
    attempt_candidates_download "$PSY_VOICE_MODEL_NAME" || true
    local effective="${PSY_EFFECTIVE_MODEL_NAME:-$PSY_VOICE_MODEL_NAME}"

    install_node

    if [ ! -f /etc/default/psyched-voice ]; then
        echo "[voice] Creating /etc/default/psyched-voice with selected model and fallbacks"
        local fallback_line=""
        if [ -n "${PSY_INTERNAL_CANDIDATES:-}" ]; then
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
# Primary Piper model (selected via alias: ${PSY_VOICE_MODEL_ALIAS:-en_male_default})
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
