#!/bin/bash

## This is a headless laptop with a nice gpu
# It runs OLLAMA
screen -Smd ollama bash -c "OLLAMA_DEBUG=false OLLAMA_HOST='0.0.0.0:11434' OLLAMA_KEEP_ALIVE=1h ollama serve"
# which can be connected to with `screen -R ollama` and is handy to know if this server is up and running

## 
## Other services will be launched inside the container, which we'll launch forthwith
## This will contain the actual ROS2 instance.
##
screen -Smd container bash -c "docker compose -f /psyche/forebrain.yaml up"
