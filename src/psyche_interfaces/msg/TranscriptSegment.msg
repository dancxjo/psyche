# Stable transcription for a completed audio segment.
# ``segment_start``/``segment_end`` reflect the timestamps emitted with the
# corresponding ``AudioSegment`` message so timeline visualisations can align
# text and waveform samples.
string segment_id                     # identifier for the audio segment
builtin_interfaces/Time segment_start # first sample time in the segment
builtin_interfaces/Time segment_end   # final sample time in the segment
string text                           # final transcript text
float32 confidence                    # decoder confidence for the full segment
