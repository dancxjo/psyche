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

- Default voice model path: `/opt/psyched/voices/en_US-lessac-medium.onnx`
- Override by placing `PSY_VOICE_MODEL=/path/to/model.onnx` in `/etc/default/psyched-voice`.

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

  ```bash
  sudo mkdir -p /opt/psyched/voices
  sudo curl -fL -o /opt/psyched/voices/en_US-lessac-medium.onnx \
    https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
  sudo curl -fL -o /opt/psyched/voices/en_US-lessac-medium.onnx.json \
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
