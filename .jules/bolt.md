## 2024-05-24 - Pre-allocate constant objects in high-frequency ROS 2 callbacks
**Learning:** In high-frequency ROS 2 callbacks (e.g., vision processing), creating constant objects like Numpy kernels (`np.ones((5, 5), np.uint8)`) on every frame introduces unnecessary memory allocation overhead, which can accumulate and cause stutter or high CPU usage.
**Action:** Always pre-allocate constant objects (like Numpy kernels, standard matrices, or repeated transforms) in the `__init__` method of the node rather than creating them inside the callback or processing function.
