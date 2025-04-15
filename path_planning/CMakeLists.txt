cmake_minimum_required(VERSION 3.8)
project(ur3_bottle_sorter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Create pathplanning library
add_library(path_planning src/path_planning.cpp)

add_executable(bottle_sorter src/main.cpp)

# Link dependencies to the executable
target_link_libraries(bottle_sorter
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  path_planning
)

# Include directories for the executable and library
target_include_directories(bottle_sorter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}  # Include directories for rclcpp
)

# Specify C++17 and C99 standards
target_compile_features(bottle_sorter PUBLIC c_std_99 cxx_std_17)

# Specify ROS dependencies for the executable
ament_target_dependencies(path_planning 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  tf2_ros 
  tf2
  )
ament_target_dependencies(bottle_sorter 
  rclcpp 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  tf2_ros 
  tf2
  )

# Install the executable
install(TARGETS bottle_sorter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
