cmake_minimum_required(VERSION 3.8)
project(vortex_utility_nodes)
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rpy_publisher_node
  src/rpy_publisher.cpp
)

ament_target_dependencies(rpy_publisher_node PUBLIC
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  vortex_utils
  vortex_msgs
)

target_link_libraries(rpy_publisher_node PUBLIC Eigen3::Eigen)

add_executable(pressure_to_depth_node
  src/pressure_to_depth.cpp
)

ament_target_dependencies(pressure_to_depth_node PUBLIC
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS rpy_publisher_node pressure_to_depth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
