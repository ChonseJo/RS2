cmake_minimum_required(VERSION 3.5)
project(bottle_cap_vision)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# -----------------------------------------------------------------------------
# Declare your ROS2‐based nodes
# -----------------------------------------------------------------------------
add_executable(hsv_click_detector src/hsv_click_detector.cpp)
ament_target_dependencies(hsv_click_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# add_executable(bottle_cap_vision_node src/bottle_cap_vision_node.cpp)
# ament_target_dependencies(bottle_cap_vision_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   OpenCV
# )

add_executable(cap_dot_representation src/cap_dot_representation.cpp)
ament_target_dependencies(cap_dot_representation
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# NEW: Add realsense_capture_node executable.
add_executable(realsense_capture_node src/realsense_capture_node.cpp)
ament_target_dependencies(realsense_capture_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(image_saver_node src/image_saver_node.cpp)
ament_target_dependencies(image_saver_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(test_roi src/test_roi.cpp)
target_link_libraries(test_roi ${OpenCV_LIBS})

# Install all executables into the lib directory of your package.
install(TARGETS
  # bottle_cap_vision_node
  test_roi
  cap_dot_representation
  realsense_capture_node
  hsv_click_detector
  image_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
