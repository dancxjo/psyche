# Finalised utterance published by ``segmenter`` on /audio/segment/final.
# ``cut_by_timeout`` indicates the segment was truncated because
# ``max_segment_ms`` elapsed; this allows ASR nodes to optionally stitch
# ``/audio/segment/final/chunks3s`` sub-clips before decoding.
builtin_interfaces/Time stamp_start  # start time for the full segment
builtin_interfaces/Time stamp_end    # end time of the last sample
string segment_id                    # unique identifier for the closed segment
uint32 sample_rate                   # samples per second for the payload
uint8  channels                      # channel count of the WAV data
uint8[] wav_data                     # PCM WAV data for the final segment
bool cut_by_timeout                  # true when the segment hit the timeout guard
