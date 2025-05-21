cmake_minimum_required(VERSION 3.8)
project(dashboard)

# Default to C++14 for ROS 2 Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Uncomment this if you're using custom_interfaces as a separate package
# find_package(custom_interfaces REQUIRED)

# Declare service files if you're defining them in this package
# If you're using custom_interfaces from another package, comment this section out
set(srv_files
  "srv/CurrentState.srv"
)

# Generate interfaces for services
# If you're using custom_interfaces from another package, comment this section out
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Create ament index resource to make the package visible
ament_export_dependencies(rosidl_default_runtime)

# If you have a C++ executable, uncomment and modify these lines:
# add_executable(${PROJECT_NAME}_node src/main.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   std_msgs
#   ${PROJECT_NAME}
# )
# rosidl_target_interfaces(${PROJECT_NAME}_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python scripts
install(PROGRAMS
  src/web_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install templates directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/templates)
  install(DIRECTORY
    templates
    DESTINATION share/${PROJECT_NAME}
    PATTERN ".svn" EXCLUDE
  )
endif()

# Install static directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/static)
  install(DIRECTORY
    static
    DESTINATION share/${PROJECT_NAME}
    PATTERN ".svn" EXCLUDE
  )
endif()

# Export package dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  rosidl_default_runtime
  # Uncomment if using custom_interfaces as a separate package
  # custom_interfaces
)
# Declare service files
set(srv_files
  "srv/CurrentState.srv"
  "srv/NumberOfBottle.srv"
)

# Must be called after all targets and exports
ament_package()