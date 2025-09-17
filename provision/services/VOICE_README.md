# Voice Service (Piper TTS)

This service provides text-to-speech for your robot using [Piper](https://github.com/rhasspy/piper).

Topics (assuming host is `$(hostname -s)`):

- Text: `/voice/$(hostname -s)` (std_msgs/String)
- Command: `/voice/$(hostname -s)/cmd` (std_msgs/String)
  - Values: `interrupt|resume|abandon` (aliases: `pause|stop`, `continue`, `cancel|flush`)
- Convenience control topics (std_msgs/Empty):
  - `/voice/$(hostname -s)/interrupt`
  - `/voice/$(hostname -s)/resume`
  - `/voice/$(hostname -s)/abandon`

Behavior:

- New texts are queued while one is speaking.
- `interrupt` pauses current playback and allows `resume` to continue.
- `abandon` stops current playback and clears the queue; new text speaks immediately.

Requirements:

- ROS 2 (e.g., Jazzy) environment sourced.
- Audio output via ALSA (`aplay`).
- Piper CLI engine (`piper-tts`) available in `PATH` (provisioning installs it and auto-detects the binary; override with `PSY_PIPER_BIN` if needed).

Model selection:

- Default behavior now performs alias-based auto-selection when `PSY_VOICE_MODEL_NAME` is not set.
  - Alias `en_male_high` (default) expands to ordered candidates:
    `en_US-kyle-high`, `en_US-ryan-high`, `en_GB-southern_english_male-medium`, `en_US-lessac-medium`.
  - The provisioning script attempts each candidate until one successfully downloads or already exists.
  - A generated `/etc/default/psyched-voice` file pins the effective model and records fallbacks (first existing is used at runtime via the node's fallback logic).

- To force a specific model explicitly, set `PSY_VOICE_MODEL_NAME=<basename>` before provisioning, or set `PSY_VOICE_MODEL` in `/etc/default/psyched-voice`.

Available built-in aliases (can be comma-separated in `PSY_VOICE_MODEL_ALIAS` to merge lists):

| Alias | Candidate Order |
|-------|-----------------|
| `en_male_high` (default) | kyle-high → ryan-high → southern_english_male-medium → lessac-medium |
| `en_female_high` | amy-high → lessac-medium |
| `minimal` / `small` | lessac-medium → kyle-high |

Example forcing alias list (merged, de-duplicated in order):
```bash
PSY_VOICE_MODEL_ALIAS="en_female_high,en_male_high" /opt/psyched/provision/services/voice.sh provision
```
- To use a high-quality male voice (example: `en_US-kyle-high`), set one of:
  - At provisioning time (temporary):
    ```bash
    PSY_VOICE_MODEL_NAME=en_US-kyle-high /opt/psyched/provision/services/voice.sh provision
    ```
  - Persistent via `/etc/default/psyched-voice`:
    ```bash
    sudo tee /etc/default/psyched-voice >/dev/null <<'EOF'
    # Primary male Piper model (high quality)
    PSY_VOICE_MODEL=/opt/psyched/voices/en_US-kyle-high.onnx
    # Optional: fallbacks (colon separated) – will pick first that exists
    PSY_VOICE_MODEL_FALLBACKS=/opt/psyched/voices/en_US-ryan-high.onnx:/opt/psyched/voices/en_GB-southern_english_male-medium.onnx
    # Optional integrity check (SHA256 of the .onnx file)
    # PSY_VOICE_MODEL_SHA256=<sha256sum-of-en_US-kyle-high.onnx>
    EOF
    ```
  - Then re-provision or restart service:
    ```bash
    sudo systemctl restart psyched@voice.service || /opt/psyched/provision/services/voice.sh provision
    ```

Supported environment variables:

- `PSY_VOICE_MODEL_NAME`: Concrete model basename (skips alias auto-selection if set).
- `PSY_VOICE_MODEL_ALIAS`: Comma-separated alias names producing ordered candidate list (default: `en_male_high`).
- `PSY_VOICE_MODEL`: Absolute path to primary model for runtime (overrides *_NAME and alias resolution at execution time).
- `PSY_VOICE_MODEL_FALLBACKS`: Colon-separated model paths; the node picks the first existing file (auto-generated if using alias selection).
- `PSY_VOICE_MODEL_SHA256`: If set during provisioning, verifies the downloaded `.onnx` integrity.

Finding a male voice:

Common high-quality male English voices (choose one):

- `en_US-kyle-high` (clear US male, high quality)
- `en_US-ryan-high` (US male, high quality)
- `en_GB-southern_english_male-medium` (UK male)

You can list available voices from the upstream repository: https://github.com/rhasspy/piper-voices

Manual download example (default first candidate of `en_male_high` alias):

```bash
sudo mkdir -p /opt/psyched/voices
curl -fL -o /opt/psyched/voices/en_US-kyle-high.onnx \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/kyle/high/en_US-kyle-high.onnx
curl -fL -o /opt/psyched/voices/en_US-kyle-high.onnx.json \
  https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/kyle/high/en_US-kyle-high.onnx.json
``` 

If provisioning doesn't auto-download, set `PSY_VOICE_MODEL` as shown above after manual download.

Systemd:

- Installed as `psyched@voice.service` running `/etc/psyched/voice.launch.sh`.

Quick test:

```bash
ros2 topic pub --once \
  /voice/$(hostname -s) std_msgs/String '{data: "Hello from Psyche"}'
```

Offline or network-restricted environments:

- Install the Piper CLI engine first (prefer `piper-tts` package when available):

  ```bash
  sudo apt-get update && sudo apt-get install piper-tts
  ```

- Preferred: install packaged voices if available

  ```bash
  sudo apt-get update && sudo apt-get install piper-voices
  ```

- Manual download from Hugging Face mirrors (example for `en_US-lessac-medium`):

  Older example (previous default female/neutral voice):

  ```bash
  sudo mkdir -p /opt/psyched/voices
  curl -fL -o /opt/psyched/voices/en_US-lessac-medium.onnx \
    https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
  curl -fL -o /opt/psyched/voices/en_US-lessac-medium.onnx.json \
    https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
  ```

- Alternatively, download a tarball and place it in a cache directory and re-run provisioning:

  ```bash
  mkdir -p /opt/psyched/voices
  curl -fL -o /opt/psyched/voices/en_US-lessac-medium.tar.gz \
    https://github.com/rhasspy/piper-voices/releases/download/v1.0.0/en_US-lessac-medium.tar.gz
  # then re-run
  /opt/psyched/cli/psy svc enable voice && /opt/psyched/cli/psy host apply
  ```

If you use a different model, set `PSY_VOICE_MODEL_NAME` in the environment before provisioning or create `/etc/default/psyched-voice` with:

```bash
PSY_VOICE_MODEL=/opt/psyched/voices/<your-model>.onnx
# Optional: point to a specific Piper binary (defaults to auto-detect)
# PSY_PIPER_BIN=/usr/local/bin/piper
```
