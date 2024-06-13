#!/bin/bash

## This is a small SBC with GPIO pins and lots of ports
### This is the device the robot is always with

# It runs OLLAMA, as a backup in case everything else crashes. It's very slow and CPU only, so se need to use a small model
# screen -Smd ollama bash -c "OLLAMA_DEBUG=false OLLAMA_HOST='0.0.0.0:11434' OLLAMA_KEEP_ALIVE=1h ollama serve"

### It also is connected to a Coral TPU...we'll use this to do live training, hopefully
### Ultimately, we'd like to see as much converge on this device as possible
### However, since this is ROS, we can distribute the processes, as long as we have access to the right devices

### We forward many of our devices to this host
screen -Smd container bash -c "docker compose -f /psyche/r1a1.yaml up"

## Finally, we loop forever playing the voice
while true; do
    # Read from TCP port
    read line < <(nc -l 127.0.0.1 8200)
    
    # URL encode the line to ensure it is sent correctly over HTTP
    encoded_line=$(echo "$line" | jq -sRr @uri)
    
    # Send the line to the TTS API and get back the speech in WAV format
    curl "http://192.168.0.4:5002/api/tts?text=$encoded_line&speaker_id=p227&style_wav=&language_id=" \
        --insecure --output - | aplay
done