cmake_minimum_required(VERSION 3.8)
project(cube_swapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(cube_swapper_node src/cube_swapper.cpp)

ament_target_dependencies(cube_swapper_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS cube_swapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

