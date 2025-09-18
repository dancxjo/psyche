# Agent Guidelines for `provision/services`

- When editing the voice provisioning scripts or node, run `pytest tests/test_voice_piper_detection.py` to keep the Piper/espeak behaviour verified.
- Maintain the `voice.sh` default configuration scaffolding (`engine="piper"` and `PSY_TTS_ENGINE=${engine}`) so the tests remain meaningful.
- Update the voice service documentation alongside behavioural changes so operators understand the defaults.
