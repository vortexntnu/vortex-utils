cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_math.cpp
    test_types.cpp
    test_concepts.cpp
    test_ros_conversions.cpp
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::gtest_main
)

ament_target_dependencies(${TEST_BINARY_NAME} PUBLIC Eigen3)

gtest_discover_tests(${TEST_BINARY_NAME})

set(TEST_BINARY_ROS_NAME ${PROJECT_NAME}_ros_test)

add_executable(
  ${TEST_BINARY_ROS_NAME}
  test_ros_transforms.cpp
)

target_link_libraries(
  ${TEST_BINARY_ROS_NAME}
    ${PROJECT_NAME}
    GTest::GTest
)

ament_target_dependencies(
  ${TEST_BINARY_ROS_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

gtest_discover_tests(${TEST_BINARY_ROS_NAME})
