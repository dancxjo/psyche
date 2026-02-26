# Rolling utterance buffer published by ``segmenter`` on /audio/segment/current.
# ``sequence`` increments per publish so downstream ASR nodes can detect missed
# updates. ``wav_data`` carries a self-contained little-endian WAV clip encoded
# with the advertised sample rate and channel count.
builtin_interfaces/Time stamp_start  # first sample timestamp
builtin_interfaces/Time stamp_end    # end timestamp for the chunk window
string segment_id                    # stable identifier for the active segment
uint32 sequence                      # monotonically increasing chunk counter
uint32 sample_rate                   # samples per second of the clip
uint8  channels                      # channel count within the WAV payload
uint8[] wav_data                     # PCM WAV data for the chunk window
