This service provides text-to-speech for your robot. Provisioning now installs
the high-quality [Piper](https://github.com/rhasspy/piper) engine and a default
English voice so speech works immediately, while automatically falling back to
the reliable [espeak-ng](https://github.com/espeak-ng/espeak-ng) synthesiser if
Piper is unavailable.

## ROS 2 topics

Assuming the host name is `$(hostname -s)`:

- Text input: `/voice/$(hostname -s)` (`std_msgs/String`)
- Command input: `/voice/$(hostname -s)/cmd` (`std_msgs/String`)
  - Accepted values: `interrupt|resume|abandon` (aliases: `pause|stop`,
    `continue`, `cancel|flush`)
- Convenience controls (`std_msgs/Empty`):
  - `/voice/$(hostname -s)/interrupt`
  - `/voice/$(hostname -s)/resume`
  - `/voice/$(hostname -s)/abandon`

New utterances are queued while one is speaking. `interrupt` pauses the current
playback so it can be resumed; `abandon` stops playback and clears the queue
before the next message is spoken.

## Engine selection

Provisioning writes `/etc/default/psyched-voice` to capture the selected engine
and its configuration. At runtime the launch script honours
`PSY_TTS_ENGINE` (values: `piper`, `espeak`, or `auto`). `auto` tries to use
Piper when a CLI binary is present and falls back to espeak-ng otherwise.

### Piper (default)

- Installs the `piper-tts` CLI with `python3 -m pip`.
- Downloads the `en_US-kyle-high` voice into `${PSY_ROOT}/voices` by default.
- Records the model path in `/etc/default/psyched-voice` so
  `voice_node.py` picks it up automatically.
- Override the download via environment variables before provisioning:
  - `PSY_PIPER_VOICE` – base name of the voice (default `en_US-kyle-high`).
  - `PSY_VOICE_MODEL` / `PSY_VOICE_MODEL_JSON` – explicit file locations.
  - `PSY_PIPER_MODEL_URL` / `PSY_PIPER_JSON_URL` – direct download URLs.
- Use `PSY_PIPER_BIN` in `/etc/default/psyched-voice` to pin a specific
  executable when multiple versions are installed.

### espeak-ng (fallback)

- Always installed (`espeak-ng`, `espeak-ng-data`, `alsa-utils`) so playback
  works even without Piper.
- Customise the voice with environment variables:
  - `PSY_ESPEAK_VOICE` (e.g., `en+m7`, `en+f3`)
  - `PSY_ESPEAK_RATE` (words per minute)
  - `PSY_ESPEAK_PITCH` (0–99)
  - `PSY_ESPEAK_ARGS` (additional flags, such as `--path=...`)
- Switch permanently by editing `/etc/default/psyched-voice`:

  ```bash
  PSY_TTS_ENGINE=espeak
  PSY_ESPEAK_VOICE=en+m7
  PSY_ESPEAK_RATE=175
  PSY_ESPEAK_PITCH=55
  ```

`psh say "Hello there"` or `ros2 topic pub` (see below) are handy smoke tests.

## Configuration reference

| Variable | Description |
|----------|-------------|
| `PSY_TTS_ENGINE` | `piper` (default), `espeak`, or `auto` |
| `PSY_PIPER_BIN` | Override for Piper CLI detection (optional) |
| `PSY_VOICE_MODEL` / `PSY_VOICE_MODEL_JSON` | Piper model paths (auto-filled) |
| `PSY_PIPER_VOICE` / `PSY_PIPER_RELEASE` | Voice + release used when downloading |
| `PSY_ESPEAK_VOICE` / `PSY_ESPEAK_RATE` / `PSY_ESPEAK_PITCH` / `PSY_ESPEAK_ARGS` | espeak-ng tuning |

## Quick test

```bash
ros2 topic pub --once \
  /voice/$(hostname -s) std_msgs/String '{data: "Hello from Psyche"}'
```

Audio should play through ALSA (`aplay`). If nothing happens, check the
`psyched@voice.service` journal and confirm the selected engine has been
installed and configured correctly.
