cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_math.cpp
    test_types.cpp
    test_concepts.cpp
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
