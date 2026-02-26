# Raw audio frame captured by ``ear_pcm`` and published on /audio/pcm.
#
# Consumers can reconstruct the absolute position using ``frame_index`` and the
# ``sample_rate``/``channels`` metadata. ``sample_format`` follows the standard
# ROS audio conventions: 1 = signed 16-bit PCM, 3 = 32-bit float PCM.
builtin_interfaces/Time stamp        # wall-clock time of capture
uint32 sample_rate                   # samples per second (e.g., 16000)
uint8  channels                      # number of interleaved channels
uint8  sample_format                 # encoded sample format identifier
uint32 frame_index                   # monotonically increasing frame counter
uint8[] data                         # interleaved PCM frame payload
