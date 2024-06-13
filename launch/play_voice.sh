#!/bin/bash
while true; do
  # Read from TCP port
  read line < <(nc 127.0.0.1 8200)

  echo $line
  
  # Trim whitespace and potential problematic characters
  trimmed_line=$(echo "$line" | tr -d '\n' | xargs -0)

  echo $timmed_line
  
  # Continue to the next iteration if the line is empty
  if [ -z "$trimmed_line" ]; then
    echo "No input received, continuing..."
    continue
  fi
  
  # URL encode the non-empty line to ensure it is sent correctly over HTTP
  encoded_line=$(echo "$trimmed_line" | jq -sRr @uri)

  echo "$encoded_line"
  
  # Send the line to the TTS API and get back the speech in WAV format
  curl "http://192.168.0.4:5002/api/tts?text=${encoded_line}&speaker_id=p238&style_wav=&language_id=" \
       --insecure --output - | ffplay - -nodisp -autoexit -loglevel error
done
