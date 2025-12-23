cmake_minimum_required(VERSION 3.8)
project(vortex_utils_ros)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(vortex_utils_ros INTERFACE)

target_include_directories(vortex_utils_ros INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vortex_utils_ros INTERFACE
  vortex_utils
  rclcpp
  Eigen3
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vortex_utils_ros
  EXPORT vortex_utils_ros_targets
)

ament_export_targets(vortex_utils_ros_targets)

ament_export_dependencies(vortex_utils)
ament_export_include_directories(include)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(py_test)
  add_subdirectory(cpp_test)
endif()

ament_package()
