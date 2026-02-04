cmake_minimum_required(VERSION 3.8)
project(vortex_utils_ros_tf)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

add_library(vortex_utils_ros_tf INTERFACE)

target_include_directories(vortex_utils_ros_tf INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vortex_utils_ros_tf INTERFACE
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  vortex_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vortex_utils_ros_tf
  EXPORT vortex_utils_ros_targets
)

ament_export_targets(vortex_utils_ros_targets)

ament_export_dependencies()
ament_export_include_directories(include)

if(BUILD_TESTING)
  add_subdirectory(cpp_test)
endif()

ament_package()
