cmake_minimum_required(VERSION 3.8)
project(tag_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# TF2 (needed by apriltag_pose_publisher)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# AprilTag system library (libapriltag-dev)
find_library(APRILTAG_LIB apriltag REQUIRED)

# -------------------------------
# Node 1: apriltag_pose_publisher
# -------------------------------
add_executable(apriltag_pose_publisher src/apriltag_pose_publisher.cpp)
ament_target_dependencies(apriltag_pose_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(apriltag_pose_publisher ${APRILTAG_LIB})

install(TARGETS
  apriltag_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Node 2: cube_color_detector
# -------------------------------
add_executable(cube_color_detector src/cube_color_detector.cpp)
ament_target_dependencies(cube_color_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(cube_color_detector ${APRILTAG_LIB})

install(TARGETS
  cube_color_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

