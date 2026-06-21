
## 2025-05-18 - Pre-allocate constant arrays in high-frequency callbacks
**Learning:** In high-frequency ROS 2 callbacks (like vision processing `image_callback` -> `detect_object`), creating constant arrays like Numpy kernels `np.ones((5, 5), np.uint8)` on every frame adds up to noticeable overhead and triggers garbage collection.
**Action:** Always pre-allocate constant objects in `__init__` and reuse them in the callback when the underlying library (like OpenCV) treats them as read-only.
