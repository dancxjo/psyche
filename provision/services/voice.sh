#!/usr/bin/env bash
set -euo pipefail
. "$(dirname "$0")/_common.sh" 2>/dev/null || true

# voice.sh — Provision voice service (Piper or eSpeak) + ROS2 node and systemd launcher
# Features:
# - Subscribes to /voice/$HOSTNAME for text (std_msgs/String)
# - Queueing of utterances
# - Control via /voice/$HOSTNAME/cmd (std_msgs/String): interrupt|resume|abandon
#   and convenience topics /voice/$HOSTNAME/{interrupt,resume,abandon}
# - Pluggable TTS backends (Piper, espeak-ng) with aplay playback

ROOT="/opt/psyched"
ETC_DIR="/etc/psyched"
VOICES_DIR="${ROOT}/voices"
PY_NODE_PATH="${ETC_DIR}/voice_node.py"
LAUNCH_PATH="${ETC_DIR}/voice.launch.sh"

# Resolve requested TTS engine early (lowercase for comparisons)
TTS_ENGINE_RAW="${PSY_TTS_ENGINE:-piper}"
TTS_ENGINE="${TTS_ENGINE_RAW,,}"
case "$TTS_ENGINE" in
  "" )
    TTS_ENGINE="espeak"
    ;;
  espeak-ng|espeakng)
    TTS_ENGINE="espeak"
    ;;
  espeak|piper|auto)
    :
    ;;
  *)
    echo "[voice] WARNING: Unknown PSY_TTS_ENGINE value '$TTS_ENGINE_RAW'; defaulting to espeak" >&2
    TTS_ENGINE="espeak"
    ;;
esac

RESOLVED_ENGINE="$TTS_ENGINE"
RESOLVED_PIPER_BIN=""

find_cli_piper() {
  local candidate resolved
  for candidate in "$@"; do
    [ -n "$candidate" ] || continue
    if resolved="$(command -v "$candidate" 2>/dev/null)"; then
      if [ -f "$resolved" ]; then
        if LC_ALL=C grep -a -m1 -E "gi.require_version\(['\"]Gtk" "$resolved" >/dev/null 2>&1; then
          continue
        fi
      fi
      echo "$resolved"
      return 0
    fi
  done
  return 1
}

detect_piper_bin() {
  local resolved
  if [ -n "${PSY_PIPER_BIN:-}" ]; then
    if resolved="$(find_cli_piper "$PSY_PIPER_BIN" 2>/dev/null)"; then
      echo "$resolved"
      return 0
    fi
  fi
  if resolved="$(find_cli_piper piper-tts piper /usr/local/bin/piper 2>/dev/null)"; then
    echo "$resolved"
    return 0
  fi
  return 1
}

resolve_tts_engine() {
  RESOLVED_ENGINE="$TTS_ENGINE"
  RESOLVED_PIPER_BIN=""
  case "$TTS_ENGINE" in
    piper)
      if RESOLVED_PIPER_BIN="$(detect_piper_bin 2>/dev/null)"; then
        RESOLVED_ENGINE="piper"
      else
        echo "[voice] WARNING: Requested Piper engine but CLI binary not found; falling back to espeak" >&2
        RESOLVED_ENGINE="espeak"
      fi
      ;;
    auto)
      if RESOLVED_PIPER_BIN="$(detect_piper_bin 2>/dev/null)"; then
        RESOLVED_ENGINE="piper"
      else
        RESOLVED_ENGINE="espeak"
      fi
      ;;
    espeak)
      RESOLVED_ENGINE="espeak"
      ;;
  esac
}

ensure_deps() {
  export PSY_DEFER_APT=1

  # Core runtime dependencies for all engines
  common_apt_install alsa-utils espeak-ng espeak-ng-data

  case "$TTS_ENGINE" in
    piper|auto)
      # Piper CLI engine (if available) plus optional packaged voices
      common_apt_install piper-tts
      common_apt_install ?piper-voices

      if ! find_cli_piper piper-tts piper /usr/local/bin/piper >/dev/null 2>&1; then
        install_piper_cli_fallback || echo "[voice] WARNING: Piper CLI fallback install failed; Piper engine may be unavailable" >&2
      fi
      ;;
    espeak|*)
      : # espeak-ng already covered above
      ;;
  esac
}

install_piper_cli_fallback() {
  # Fetch a prebuilt Piper CLI release when apt packages are unavailable.
  local arch suffix tmp_dir tar_path dest dest_tmp bin_path final_dir wrapper
  local -a asset_candidates url_candidates

  arch="$(dpkg --print-architecture 2>/dev/null || uname -m || echo unknown)"
  case "$arch" in
    amd64|x86_64)
      suffix="x86_64"
      asset_candidates=(
        "piper_linux_x86_64.tar.gz"
        "piper_x86_64.tar.gz"
        "piper_linux_amd64.tar.gz"
      )
      ;;
    arm64|aarch64)
      suffix="aarch64"
      asset_candidates=(
        "piper_linux_aarch64.tar.gz"
        "piper_aarch64.tar.gz"
        "piper_linux_arm64.tar.gz"
      )
      ;;
    armhf|armv7l)
      suffix="armv7l"
      asset_candidates=(
        "piper_linux_armv7l.tar.gz"
        "piper_armv7l.tar.gz"
        "piper_linux_armhf.tar.gz"
      )
      ;;
    *)
      echo "[voice] WARNING: Unsupported architecture '$arch' for Piper prebuilt binaries" >&2
      return 1
      ;;
  esac

  tmp_dir="$(mktemp -d 2>/dev/null || echo /tmp/piper.$$)"
  tar_path="${tmp_dir}/piper.tar.gz"
  local downloaded=0 asset asset_url fetch_tool

  if command -v curl >/dev/null 2>&1; then
    fetch_tool="curl"
  elif command -v wget >/dev/null 2>&1; then
    fetch_tool="wget"
  else
    echo "[voice] Neither curl nor wget available to download Piper binary" >&2
    rm -rf "$tmp_dir" || true
    return 1
  fi

  url_candidates=()
  for asset in "${asset_candidates[@]}"; do
    [ -n "$asset" ] || continue
    url_candidates+=(
      "https://github.com/rhasspy/piper/releases/latest/download/${asset}"
    )
    if [ -n "${PSY_PIPER_VERSION:-}" ]; then
      url_candidates+=("https://github.com/rhasspy/piper/releases/download/${PSY_PIPER_VERSION}/${asset}")
    fi
    url_candidates+=(
      "https://github.com/rhasspy/piper/releases/download/2023.11.14-2/${asset}"
      "https://github.com/rhasspy/piper/releases/download/2023.11.14/${asset}"
      "https://github.com/rhasspy/piper/releases/download/v1.2.0/${asset}"
    )
  done

  for asset_url in "${url_candidates[@]}"; do
    [ -n "$asset_url" ] || continue
    echo "[voice] Attempting Piper CLI download: ${asset_url}"
    if [ "$fetch_tool" = "curl" ]; then
      if curl -fL --retry 3 --connect-timeout 20 -o "$tar_path" "$asset_url"; then
        downloaded=1
        break
      fi
    else
      if wget -q -O "$tar_path" "$asset_url"; then
        downloaded=1
        break
      fi
    fi
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
    local alias_list="${PSY_VOICE_MODEL_ALIAS:-en_male_high}" # default alias emphasises high-quality male voice
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

write_default_config() {
    local engine="$1" effective_model="$2" fallback_paths="$3"
    local config="/etc/default/psyched-voice"

    if [ ! -f "$config" ]; then
        if [ "$engine" = "piper" ]; then
            local fallback_line=""
            local model_line=""
            if [ -n "$fallback_paths" ]; then
                fallback_line="PSY_VOICE_MODEL_FALLBACKS=${fallback_paths}"
            fi
            if [ -n "$effective_model" ]; then
                model_line="PSY_VOICE_MODEL=/opt/psyched/voices/${effective_model}.onnx"
            fi
            sudo tee "$config" >/dev/null <<CFG
# Auto-generated by voice.sh provision on $(date -u +%Y-%m-%dT%H:%M:%SZ)
# Primary Piper model (selected via alias: ${PSY_VOICE_MODEL_ALIAS:-en_male_high})
PSY_TTS_ENGINE=piper
${model_line}
${fallback_line}
# Optional integrity check (SHA256 of the .onnx file)
# PSY_VOICE_MODEL_SHA256=<sha256sum>
# Set PSY_TTS_ENGINE=espeak to switch to the espeak-ng backend.
CFG
        else
            sudo tee "$config" >/dev/null <<'CFG'
# Auto-generated by voice.sh provision
PSY_TTS_ENGINE=espeak
# Optional espeak-ng customisation examples:
# PSY_ESPEAK_VOICE=en+m7
# PSY_ESPEAK_RATE=170
# PSY_ESPEAK_PITCH=50
# PSY_ESPEAK_ARGS=--path=/usr/share/espeak-ng-data
# To use Piper instead, install piper-tts and set PSY_TTS_ENGINE=piper.
CFG
        fi
        return
    fi

    if ! sudo grep -qE '^[[:space:]]*PSY_TTS_ENGINE=' "$config"; then
        printf '\nPSY_TTS_ENGINE=%s\n' "$engine" | sudo tee -a "$config" >/dev/null
    fi

    if [ "$engine" = "piper" ]; then
        if [ -n "$effective_model" ] && ! sudo grep -qE '^[[:space:]]*PSY_VOICE_MODEL=' "$config"; then
            printf 'PSY_VOICE_MODEL=/opt/psyched/voices/%s.onnx\n' "$effective_model" | sudo tee -a "$config" >/dev/null
        fi
        if [ -n "$fallback_paths" ] && ! sudo grep -qE '^[[:space:]]*PSY_VOICE_MODEL_FALLBACKS=' "$config"; then
            printf 'PSY_VOICE_MODEL_FALLBACKS=%s\n' "$fallback_paths" | sudo tee -a "$config" >/dev/null
        fi
    fi
}

install_node() {
  sudo mkdir -p "$ETC_DIR"
  if [ -f "${ROOT}/provision/services/voice_node.py" ]; then
    sudo install -m 0755 "${ROOT}/provision/services/voice_node.py" "$PY_NODE_PATH"
  else
    echo "[voice] ERROR: voice_node.py missing from repository; cannot install" >&2
    return 1
  fi

  sudo tee "$LAUNCH_PATH" >/dev/null <<'LAUNCH'
#!/usr/bin/env bash
set -e
set +u; source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; set -u

if [ -f /etc/default/psyched-voice ]; then
  source /etc/default/psyched-voice
fi

ENGINE="${PSY_TTS_ENGINE:-espeak}"
ENGINE="${ENGINE,,}"

if [ "$ENGINE" = "auto" ]; then
  if [ -z "${PSY_PIPER_BIN:-}" ]; then
    for candidate in piper-tts piper /usr/local/bin/piper; do
      [ -n "$candidate" ] || continue
      if command -v "$candidate" >/dev/null 2>&1; then
        resolved="$(command -v "$candidate")"
        if [ -f "$resolved" ] && grep -qE "gi.require_version\(['\"]Gtk" "$resolved" 2>/dev/null; then
          continue
        fi
        PSY_PIPER_BIN="$resolved"
        break
      fi
    done
  fi
  if [ -n "${PSY_PIPER_BIN:-}" ]; then
    ENGINE="piper"
  else
    ENGINE="espeak"
  fi
fi

if [ "$ENGINE" = "piper" ]; then
  if [ -n "${PSY_PIPER_BIN:-}" ]; then
    resolved="$(command -v "$PSY_PIPER_BIN" 2>/dev/null || true)"
    if [ -n "$resolved" ] && [ -f "$resolved" ] && grep -qE "gi.require_version\(['\"]Gtk" "$resolved" 2>/dev/null; then
      resolved=""
    fi
    if [ -n "$resolved" ]; then
      PSY_PIPER_BIN="$resolved"
    else
      PSY_PIPER_BIN=""
    fi
  fi
  if [ -z "${PSY_PIPER_BIN:-}" ]; then
    for candidate in piper-tts piper /usr/local/bin/piper; do
      [ -n "$candidate" ] || continue
      if command -v "$candidate" >/dev/null 2>&1; then
        resolved="$(command -v "$candidate")"
        if [ -f "$resolved" ] && grep -qE "gi.require_version\(['\"]Gtk" "$resolved" 2>/dev/null; then
          continue
        fi
        PSY_PIPER_BIN="$resolved"
        break
      fi
    done
  fi
  if [ -z "${PSY_PIPER_BIN:-}" ]; then
    echo "[psyched-voice] Piper CLI not available; falling back to espeak" >&2
    ENGINE="espeak"
  fi
fi

if [ "$ENGINE" = "piper" ]; then
  export PSY_PIPER_BIN
  export PSY_VOICE_MODEL="${PSY_VOICE_MODEL:-/opt/psyched/voices/en_US-kyle-high.onnx}"
else
  unset PSY_PIPER_BIN
fi

export PSY_TTS_ENGINE="$ENGINE"
exec python3 /etc/psyched/voice_node.py
LAUNCH
  sudo chmod +x "$LAUNCH_PATH"
}



provision() {
    ensure_deps

    resolve_tts_engine

    export PSY_TTS_ENGINE="$RESOLVED_ENGINE"
    if [ -n "$RESOLVED_PIPER_BIN" ]; then
        export PSY_PIPER_BIN="$RESOLVED_PIPER_BIN"
    else
        unset PSY_PIPER_BIN || true
    fi

    local effective=""
    local fallback_paths=""

    if [ "$RESOLVED_ENGINE" = "piper" ]; then
        # Build candidate list (no subshell capture)
        select_voice_model
        export PSY_VOICE_MODEL_NAME="${PSY_EFFECTIVE_MODEL_NAME:-$PSY_SELECTED_MODEL}"

        # Attempt downloads over all candidates
        attempt_candidates_download "$PSY_VOICE_MODEL_NAME" || true
        effective="${PSY_EFFECTIVE_MODEL_NAME:-$PSY_VOICE_MODEL_NAME}"

        if [ -n "${PSY_INTERNAL_CANDIDATES:-}" ]; then
            local fb=()
            for c in $PSY_INTERNAL_CANDIDATES; do
                [ "$c" = "$effective" ] && continue
                fb+=("/opt/psyched/voices/${c}.onnx")
            done
            if [ ${#fb[@]} -gt 0 ]; then
                fallback_paths="$(IFS=:; echo "${fb[*]}")"
            fi
        fi
    else
        unset PSY_VOICE_MODEL_NAME PSY_INTERNAL_CANDIDATES PSY_EFFECTIVE_MODEL_NAME || true
    fi

    install_node

    write_default_config "$RESOLVED_ENGINE" "$effective" "$fallback_paths"

    if [ "$RESOLVED_ENGINE" = "piper" ]; then
        echo "[voice] Provisioned Piper engine. Effective model: ${effective}"
        if [ -n "$RESOLVED_PIPER_BIN" ]; then
            echo "[voice] Piper CLI detected at: ${RESOLVED_PIPER_BIN}"
        fi
        echo "[voice] Override with PSY_VOICE_MODEL_NAME, PSY_VOICE_MODEL, or PSY_VOICE_MODEL_ALIAS as needed."
    else
        echo "[voice] Provisioned espeak-ng engine. Customise via PSY_ESPEAK_VOICE/PSY_ESPEAK_RATE/PSY_ESPEAK_PITCH."
    fi

    echo "[voice] provisioned. Use: sudo systemctl start psyched@voice.service"
}

case "${1:-provision}" in
  provision) provision ;;
  *) echo "unknown"; exit 1;;
esac
