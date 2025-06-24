#!/bin/bash

## 
## Other services will be launched inside the container, which we'll launch forthwith
## This will contain the actual ROS2 instance.
##
screen -Smd container bash -c "docker compose -f /psyche/forebrain.yaml up"
