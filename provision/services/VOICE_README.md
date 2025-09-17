This service provides text-to-speech for your robot. It now defaults to the
reliable, lightweight [espeak-ng](https://github.com/espeak-ng/espeak-ng)
engine so the voice service works immediately after provisioning, while still
supporting the higher-quality [Piper](https://github.com/rhasspy/piper) CLI when
it is available.

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
`PSY_TTS_ENGINE` (values: `espeak`, `piper`, or `auto`). `auto` tries to use
Piper when a CLI binary is present and falls back to espeak-ng otherwise.

### espeak-ng (default)

- Packages (`espeak-ng`, `espeak-ng-data`, `alsa-utils`) are installed
  automatically during provisioning.
- Customise the voice with environment variables:
  - `PSY_ESPEAK_VOICE` (e.g., `en+m7`, `en+f3`)
  - `PSY_ESPEAK_RATE` (words per minute)
  - `PSY_ESPEAK_PITCH` (0–99)
  - `PSY_ESPEAK_ARGS` (additional flags, such as `--path=...`)
- Example configuration appended to `/etc/default/psyched-voice`:

  ```bash
  PSY_TTS_ENGINE=espeak
  PSY_ESPEAK_VOICE=en+m7
  PSY_ESPEAK_RATE=175
  PSY_ESPEAK_PITCH=55
  ```

`psh say "Hello there"` or `ros2 topic pub` (see below) are handy smoke tests.

### Piper (optional high-quality voices)

Set `PSY_TTS_ENGINE=piper` (or `auto`) before re-running
`/opt/psyched/provision/services/voice.sh provision`. The script attempts to
install the Piper CLI (`piper-tts`) and download one of the configured voice
models. If the CLI is not found the service falls back to espeak-ng automatically
but prints a warning so you can investigate.

Voice selection uses aliases to provide an ordered list of candidate models.
Defaults favour male English voices that are widely distributed:

| Alias | Candidate order |
|-------|-----------------|
| `en_male_default` | lessac-medium → kyle-high → ryan-high → southern_english_male-medium |
| `en_male_high` | kyle-high → ryan-high → lessac-medium → southern_english_male-medium |
| `en_female_high` | amy-high → lessac-medium |
| `minimal` / `small` | lessac-medium → kyle-high |

Override the alias list at provisioning time, e.g.

```bash
PSY_TTS_ENGINE=piper PSY_VOICE_MODEL_ALIAS="en_female_high,en_male_high" \
  /opt/psyched/provision/services/voice.sh provision
```

The script downloads the first working candidate into `/opt/psyched/voices` and
records the chosen model plus fallbacks in `/etc/default/psyched-voice`.

Manual voice installation remains available. Download an ONNX model (and
matching `.onnx.json`) into `/opt/psyched/voices/` and set
`PSY_VOICE_MODEL=/opt/psyched/voices/<model>.onnx` inside
`/etc/default/psyched-voice`.

## Configuration reference

| Variable | Description |
|----------|-------------|
| `PSY_TTS_ENGINE` | `espeak` (default), `piper`, or `auto` |
| `PSY_PIPER_BIN` | Override for Piper CLI detection (optional) |
| `PSY_VOICE_MODEL` | Primary Piper model path (auto-filled when provisioning Piper) |
| `PSY_VOICE_MODEL_FALLBACKS` | Colon-separated Piper model paths checked in order |
| `PSY_VOICE_MODEL_ALIAS` | Alias list used during provisioning (Piper) |
| `PSY_ESPEAK_VOICE` / `PSY_ESPEAK_RATE` / `PSY_ESPEAK_PITCH` / `PSY_ESPEAK_ARGS` | espeak-ng tuning |

## Quick test

```bash
ros2 topic pub --once \
  /voice/$(hostname -s) std_msgs/String '{data: "Hello from Psyche"}'
```

Audio should play through ALSA (`aplay`). If nothing happens, check the
`psyched@voice.service` journal and confirm the selected engine has been
installed and configured correctly.
