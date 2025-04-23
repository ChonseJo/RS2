cmake_minimum_required(VERSION 3.8)
project(servo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include the serial library headers
include_directories(
  external/serial/include
)

# build the servo_serial_node using serial.cc
add_executable(servo_serial_node
  src/servo_serial_node.cpp
  external/serial/src/serial.cc
  external/serial/src/impl/unix.cc
)

# build the test node (no serial dependency)
add_executable(servo_test_node
  src/servo_test_node.cpp
)

# build the reset node
add_executable(servo_reset_node
  src/servo_reset_node.cpp
)

# link ROS2 dependencies
ament_target_dependencies(servo_serial_node
  rclcpp
  std_msgs
)

ament_target_dependencies(servo_test_node
  rclcpp
  std_msgs
)

ament_target_dependencies(servo_reset_node
  rclcpp
  std_msgs
)

# install all executables
install(TARGETS
  servo_serial_node
  servo_test_node
  servo_reset_node
  DESTINATION lib/${PROJECT_NAME}
)

# enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

