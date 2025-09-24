# Agent Guidelines for `provision/services`

- When editing the voice provisioning scripts or node, run `pytest tests/test_voice_piper_detection.py` to keep the Piper/espeak behaviour verified.
- Maintain the `voice.sh` default configuration scaffolding (`engine="piper"` and `PSY_TTS_ENGINE=${engine}`) so the tests remain meaningful.
- Update the voice service documentation alongside behavioural changes so operators understand the defaults.
- When adjusting the navigation service launchers, run `pytest tests/services/test_nav_depth_to_scan.py` to confirm the
  depthimage_to_laserscan bridge expectations still hold.
- When editing the network access point provisioning, run `pytest tests/services/test_network_ap.py` to keep the hostapd/dnsmasq
  contract verified.
