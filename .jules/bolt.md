## 2024-06-18 - Pre-allocating Numpy Arrays in High-Frequency Callbacks
**Learning:** Instantiating Numpy arrays (like morphological kernels `np.ones(...)`) inside high-frequency ROS 2 callbacks (e.g., vision processing at 30+ FPS) adds measurable memory allocation overhead and triggers more frequent garbage collection.
**Action:** Always pre-allocate constant objects (like Numpy kernels) in `__init__` and reuse them across frames when working with high-frequency topics.
