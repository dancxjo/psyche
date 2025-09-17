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

- Default model path: `/opt/psyched/voices/en_US-kyle-high.onnx` (male high-quality voice).
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

- `PSY_VOICE_MODEL_NAME`: Model basename (without path) fetched into `/opt/psyched/voices/<name>.onnx`.
- `PSY_VOICE_MODEL`: Absolute path to model to load (overrides *_NAME at runtime).
- `PSY_VOICE_MODEL_FALLBACKS`: Colon-separated list of alternative model paths the node will try if the primary does not exist.
- `PSY_VOICE_MODEL_SHA256`: If set, provisioning verifies the `.onnx` file's SHA256 and re-downloads if mismatched.

Finding a male voice:

Common high-quality male English voices (choose one):

- `en_US-kyle-high` (clear US male, high quality)
- `en_US-ryan-high` (US male, high quality)
- `en_GB-southern_english_male-medium` (UK male)

You can list available voices from the upstream repository: https://github.com/rhasspy/piper-voices

Manual download example (default male high-quality model):

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
