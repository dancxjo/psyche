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
