## 2024-05-24 - High-Frequency Loop Optimizations in ROS 2 Vision Nodes

**Learning:** In high-frequency loops (e.g., image processing callbacks like `ObjectDetector`), creating objects repeatedly (like `np.ones((5, 5), np.uint8)` for morphology kernels) causes unnecessary memory allocation and garbage collection overhead. Pre-allocating these constants in `__init__` is a better performance pattern.
Furthermore, within the inner loop of contour processing, checking `if area > min_area and area > largest_area:` evaluates two conditions per contour. By initializing `largest_area = min_area` instead of `0` before the loop, we can simplify the condition to just `if area > largest_area:`, thereby halving the number of conditional checks in the hot path.

**Action:** Whenever writing or reviewing vision nodes or other high-frequency callbacks, look for static arrays or constants defined inside the loop/callback and move them to `__init__`. Additionally, inspect loops searching for extrema (e.g., maximum size) and initialize tracking variables to the minimum threshold to reduce conditional branch count inside the loop.
