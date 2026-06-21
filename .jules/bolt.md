
## 2024-05-24 - Optimizing Python CV/Vision loops
**Learning:** In high-frequency CV/vision loops (like `ObjectDetector`), creating objects inside the loop (like `np.ones` for morphology kernels) causes redundant memory allocations which are slow. In addition, nested condition checks inside contour loops (e.g. `if area > min_area and area > largest_area`) can be eliminated by initializing search variables cleverly (e.g. `largest_area = min_area`).
**Action:** Always pre-allocate static arrays/kernels in class `__init__` methods for any high-frequency loops. Identify and optimize branch predictions within performance-critical algorithms.
