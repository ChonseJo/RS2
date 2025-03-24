cmake_minimum_required(VERSION 3.5)
project(bottle_cap_vision)

find_package(OpenCV REQUIRED)

add_executable(cap_centroid_detector src/cap_centroid_detector.cpp)
target_link_libraries(cap_centroid_detector ${OpenCV_LIBS})

add_executable(hsv_click_detector src/hsv_click_detector.cpp)
target_link_libraries(hsv_click_detector ${OpenCV_LIBS})

install(TARGETS cap_centroid_detector hsv_click_detector
  DESTINATION lib/${PROJECT_NAME})
