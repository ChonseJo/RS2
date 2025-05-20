# Bottle Cap Vision — RealSense Capture & ROI Grid

> **Package**: `bottle_cap_vision`  
> **Node**: `realsense_capture_node`  
> **Topic**: Subscribes to `/camera/camera/color/image_raw`

---

## 1. Overview

`realsense_capture_node`:

1. Converts each incoming color frame to **HSV**.
2. Applies **red**, **green**, **blue** masks (via `cv::inRange`).
3. Splits the frame into a **3 × 2** grid (6 equal cells).
4. Counts mask pixels in each cell to pick the **dominant colour**.
5. Logs a 6‑element code sequence (e.g. `C S F | S C F`).
6. (Optionally) draws colored outlines on a black “dot” canvas.

---

## 2. ROI Grid Logic

Treat the full image as one “outer box”:

```
+-------+-------+-------+
|  0    |   1   |   2   |
+-------+-------+-------+
|  3    |   4   |   5   |
+-------+-------+-------+
```

- **cell width**  = `image.cols / 3`  
- **cell height** = `image.rows / 2`

For each cell `roi`, the node does:
```cpp
int cr = cv::countNonZero(red_mask(roi));
int cg = cv::countNonZero(green_mask(roi));
int cb = cv::countNonZero(blue_mask(roi));

if      (cr>cg && cr>cb) slotColour = 1;  // Red
else if (cg>cr && cg>cb) slotColour = 2;  // Green
else if (cb>cr && cb>cg) slotColour = 3;  // Blue
else                     slotColour = 0;  // None/ambiguous
```

Then maps:
```cpp
1 → 'C'  // Coke (red)
2 → 'S'  // Sprite (green)
3 → 'F'  // Fanta (blue)
```

---

## 3. Building

```bash
# From your ROS2 workspace root
colcon build   --packages-select bottle_cap_vision   --cmake-clean-cache
```

Ensure your **CMakeLists.txt** contains:
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
```

---

## 4. Running

1. **Launch RealSense driver**  
   ```bash
   ros2 launch realsense2_camera rs_camera.launch.py
   ```

2. **Run the capture node**  
   ```bash
   ros2 run bottle_cap_vision realsense_capture_node
   ```

3. **Observe console log**, e.g.:  
   ```
   [INFO] Slots R→B: [1 2 3 | 2 1 3]
   [INFO] Grid codes: C S F | S C F
   ```

4. **Inspect OpenCV windows**:  
   - **Camera View**: raw color feed  
   - **Bottle Type**: black‑background outlines  

5. **Press ESC** in either window to exit.

---

## 5. Troubleshooting & Tuning

- **HSV thresholds** live in `realsense_capture_node.cpp`.  
- If colours bleed or miss, adjust the `cv::inRange(...)` scalar values.  
- To disable the black‑background view, replace  
  ```cpp
  cv::Mat dotImage = cv::Mat::zeros(image.size(), image.type());
  ```
  with  
  ```cpp
  cv::Mat dotImage = image.clone();
  ```

---

### Colour→Letter Mapping

| Mask ID | Mask  | Letter | Example |
|:-------:|:-----:|:------:|:-------:|
| 1       | Red   | `C`    | Coke    |
| 2       | Green | `S`    | Sprite  |
| 3       | Blue  | `F`    | Fanta   |
