Bottle Cap Vision — RealSense Capture & ROI Grid

This README covers how to build and run the realsense_capture_node and explains the 3×2 ROI grid logic used to classify coloured bottle caps.

Overview

Package: bottle_cap_vision

Node: realsense_capture_node

Subscribes to /camera/camera/color/image_raw

Detects red, green, blue regions via HSV masks

Divides the frame into a 3‑column × 2‑row grid (6 cells)

Counts mask pixels in each cell to decide the dominant colour

Logs a 6‑element code sequence (e.g. [C S F | S C F])

Optionally draws outlines on a black-background “dot” view

Building

cd ~/ros2_ws
colcon build --packages-select bottle_cap_vision --cmake-clean-cache

Make sure you have these dependencies in your CMakeLists.txt:

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

Running

Launch your RealSense driver (e.g. via ros2 launch realsense2_camera rs_camera.launch.py).

Run the capture node:

ros2 run bottle_cap_vision realsense_capture_node

You should see console output like:

[INFO] Slots R→B: [1 2 3 | 2 1 3]
[INFO] Grid codes: C S F | S C F

And two OpenCV windows:

Camera View: raw colour feed

Bottle Type: black-background outlines of detected caps

Press ESC to exit.

ROI Grid Explanation

We define an outer rectangle covering the full frame:  cv::Rect(0,0,image.cols,image.rows).

Compute cell size:

int cellW = image.cols / 3;
int cellH = image.rows / 2;

Loop rows 0–1, cols 0–2 to form six cv::Rect ROIs:

roi.x = col*cellW;
roi.y = row*cellH;
roi.width  = cellW;
roi.height = cellH;

Count red, green, blue mask pixels inside each ROI:

int cr = cv::countNonZero(red_mask(roi));
int cg = cv::countNonZero(green_mask(roi));
int cb = cv::countNonZero(blue_mask(roi));

Assign a slot code based on the maximum count:

1 = red, 2 = green, 3 = blue, 0 = none

Convert slot codes → letters via a map:

{1→'C', 2→'S', 3→'F'}

Cell index helper:

auto findCellIndex = [&](Point2f pt){
  int col = min(int(pt.x/cellW),2);
  int row = min(int(pt.y/cellH),1);
  return row*3 + col;
};

This yields a consistent ordering:

 [0] [1] [2]
 [3] [4] [5]

Customization

Black vs Live background: switch between Mat::zeros(...) or image.clone() in the drawing section.

Mask thresholds: adjust HSV ranges in cv::inRange(...) calls per your lighting and cap colours.

Noise cleaning: tune medianBlur, morphologyEx, and area thresholds to remove false positives.

Feel free to modify and extend this node for your sorting pipeline!

