# Low-latency hypothesis for the currently open audio segment.
# ``rev`` should increment for each update so ``transcript_fuser`` can discard
# superseded revisions. ``confidence`` expresses a lightweight heuristic for UI
# display; downstream components may ignore it.
string segment_id                    # identifier that matches the audio segment
uint32 rev                           # monotonically increasing revision number
builtin_interfaces/Time stamp_emitted # emission time of this partial hypothesis
string text                          # raw UTF-8 transcription
float32 confidence                   # optional confidence (0.0-1.0)
