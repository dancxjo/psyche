
## 2026-05-13 - [MicVADNode Energy Calculation Optimization]
**Learning:** Replacing manual Python loops with bulk `struct.unpack` and `sum(map(abs, samples))` for processing PCM16 audio frames provides a significant (~4.3x) performance boost. Pre-calculating format strings can offer a slight further gain, but dynamic calculation `f'<{count}h'` is necessary if the method must handle variable-length frames (e.g., in unit tests).
**Action:** Use bulk `struct.unpack` for high-frequency primitive data processing instead of individual `struct.unpack_from` in loops.
