## 2024-05-14 - Pre-allocate Constant Objects in ROS 2 Callbacks
**Learning:** High-frequency ROS 2 image callbacks suffer from increased memory allocation overhead and GC pauses when instantiating constant arrays (like morphological kernels via `np.ones`) on every frame.
**Action:** Always pre-allocate constant arrays or objects in the `__init__` method of the node and reuse them across callback invocations to minimize per-frame overhead.
