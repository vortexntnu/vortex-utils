cmake_minimum_required(VERSION 3.8)
project(vortex_utility_nodes)
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(euler_odometry_publisher_node
  src/euler_odometry_publisher_node.cpp
)

ament_target_dependencies(euler_odometry_publisher_node PUBLIC
  rclcpp
  nav_msgs
  Eigen3
  vortex_utils
)

install(TARGETS euler_odometry_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
