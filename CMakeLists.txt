cmake_minimum_required(VERSION 3.8)
project(tag_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(apriltag_pose_publisher src/apriltag_pose_publisher.cpp)

ament_target_dependencies(apriltag_pose_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  cv_bridge
)

# ---- link system apriltag ----
find_path(APRILTAG_INCLUDE_DIR apriltag/apriltag.h)
find_library(APRILTAG_LIBRARY apriltag)

target_include_directories(apriltag_pose_publisher PRIVATE ${APRILTAG_INCLUDE_DIR})
target_link_libraries(apriltag_pose_publisher ${APRILTAG_LIBRARY})
# ------------------------------

install(TARGETS
  apriltag_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

